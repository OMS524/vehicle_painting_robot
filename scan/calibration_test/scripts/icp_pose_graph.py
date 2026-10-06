#!/usr/bin/env python3
"""저장된 로봇 TF -> 모든 뷰 쌍 ICP -> Pose Graph 최적화 -> 병합.

하드웨어/SDK 호출, 원본 수정, 로봇·hand-eye 캘리브레이션 갱신 없음.
기존 icp_common의 입출력/ICP 구현을 재사용하며 기존 네 스크립트는 변경하지 않는다.
"""
from __future__ import annotations

import argparse
from contextlib import redirect_stderr, redirect_stdout
import copy
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from itertools import combinations
from pathlib import Path
import sys
import time
import traceback

import icp_common as icp  # NumPy/OpenMP 초기화 순서도 기존 테스트와 동일
import numpy as np


@dataclass(frozen=True)
class GraphSettings:
    # 각 방향에서 모두 충족해야 함. 확률/정확도 보장이 아닌 실험용 필터.
    min_fitness: float = 0.15
    min_correspondences: int = 30
    max_pair_rmse_mm: float = 6.0
    max_pair_centroid_shift_mm: float = 50.0
    max_pair_rotation_deg: float = 5.0
    edge_prune_threshold: float = 0.25
    preference_loop_closure: float = 1.0


# ======================== 사용자가 수정할 설정 ========================
HERE = Path(__file__).resolve().parent
CAPTURE_DIR = HERE.parents[1] / "doosan_a0912_scan_test/log/20260923_220542_796308"
OUTPUT_ROOT = HERE.parent / "log"
REFERENCE_VIEW = None  # manifest의 첫 뷰 고정. 예: "view_02"
ICP_METHOD = "point_to_plane"  # point_to_point / point_to_plane / generalized / colored
SETTINGS = icp.Settings(
    scales=((10.0, 30.0, 60), (5.0, 15.0, 40), (2.0, 6.0, 30)),
    evaluation_voxel_mm=5.0,
    evaluation_distance_mm=10.0,
    output_voxel_mm=2.0,
    roi_min_mm=None,  # 정합/평가만 제한. 전체 출력 점은 유지. base XYZ, mm.
    roi_max_mm=None,
)
GRAPH_SETTINGS = GraphSettings()
# =====================================================================


def validate_graph_settings(g):
    if not np.isfinite(g.min_fitness) or not 0 < g.min_fitness <= 1:
        raise ValueError("min_fitness는 0 초과 1 이하이어야 합니다.")
    if type(g.min_correspondences) is not int or g.min_correspondences < 3:
        raise ValueError("min_correspondences는 3 이상의 정수여야 합니다.")
    for value in (g.max_pair_rmse_mm, g.max_pair_centroid_shift_mm,
                  g.max_pair_rotation_deg, g.preference_loop_closure):
        if not np.isfinite(value) or value <= 0:
            raise ValueError("PGO 거리/각도/가중치는 유한한 양수여야 합니다.")
    if not np.isfinite(g.edge_prune_threshold) or not 0 < g.edge_prune_threshold < 1:
        raise ValueError("edge_prune_threshold는 0~1 사이여야 합니다.")


def overlap_reasons(metrics, g, check_rmse=False):
    reasons = []
    for direction, value in metrics.items():
        if value["correspondences"] < g.min_correspondences or value["fitness"] < g.min_fitness:
            reasons.append(f"{direction}: 대응점/겹침 비율 부족")
        if check_rmse and (value["inlier_rmse_mm"] is None
                           or value["inlier_rmse_mm"] > g.max_pair_rmse_mm):
            reasons.append(f"{direction}: 대응점 RMSE 한도 초과")
    return reasons


def pair_rejection_reasons(metrics, forward, reverse, g):
    reasons = overlap_reasons(metrics, g, check_rmse=True)
    if max(forward["centroid_shift_mm"], reverse["centroid_shift_mm"]) > g.max_pair_centroid_shift_mm:
        reasons.append("점군 중심 보정 이동량 한도 초과")
    if forward["rotation_deg"] > g.max_pair_rotation_deg:
        reasons.append("회전 보정량 한도 초과")
    return reasons


def spanning_tree(node_count, pairs):
    """겹침이 크고 RMSE가 작은 검사 통과 간선부터 연결. 촬영 순서로 확신하지 않음."""
    parents = list(range(node_count))

    def root(i):
        while parents[i] != i:
            parents[i] = parents[parents[i]]
            i = parents[i]
        return i

    chosen = set()
    for pair in sorted(pairs, key=lambda p: (-p["quality_score"], p["source_id"], p["target_id"])):
        i, j = pair["source_id"], pair["target_id"]
        a, b = root(i), root(j)
        if a != b:
            parents[b] = a
            chosen.add((i, j))
    if len(chosen) != node_count - 1:
        groups = {}
        for i in range(node_count):
            groups.setdefault(root(i), []).append(i)
        raise ValueError(f"검사 통과 간선으로 모든 뷰가 연결되지 않습니다: {list(groups.values())}. "
                         "겹침/TF/ROI를 확인하세요. 미연결 뷰를 버리거나 강제로 연결하지 않습니다.")
    return chosen


def make_graph(node_count, pairs, o3d):
    tree = spanning_tree(node_count, pairs)
    graph = o3d.pipelines.registration.PoseGraph()
    # 점군에 이미 원본 TF 적용: 노드 pose는 base에서의 추가 보정 Delta.
    # 모든 Delta=I에서 시작하는 것이 로봇 TF 초기 배치와 같다.
    for _ in range(node_count):
        graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(np.eye(4)))
    for pair in pairs:
        i, j = pair["source_id"], pair["target_id"]
        pair["role"] = "backbone" if (i, j) in tree else "loop"
        pair["uncertain"] = (i, j) not in tree
        graph.edges.append(o3d.pipelines.registration.PoseGraphEdge(
            i, j, np.asarray(pair["delta_source_to_target_base"]),
            np.asarray(pair["information"]), uncertain=pair["uncertain"]))
    return graph


def optimize_graph(graph, reference_id, s, g, o3d):
    reg = o3d.pipelines.registration
    reg.global_optimization(graph, reg.GlobalOptimizationLevenbergMarquardt(),
                            reg.GlobalOptimizationConvergenceCriteria(),
                            reg.GlobalOptimizationOption(
                                max_correspondence_distance=s.scales[-1][1] / 1000,
                                edge_prune_threshold=g.edge_prune_threshold,
                                preference_loop_closure=g.preference_loop_closure,
                                reference_node=reference_id))
    # 기준 노드 Delta=I를 수치 오차까지 정규화. 모든 상대 변환은 유지된다.
    anchor_inv = np.linalg.inv(icp.validate_transform(graph.nodes[reference_id].pose))
    for node in graph.nodes:
        node.pose = icp.validate_transform(anchor_inv @ node.pose)
    return [np.asarray(node.pose).copy() for node in graph.nodes]


def save_graph(path, graph, o3d):
    # Open3D 0.18의 Python 바인딩은 성공해도 None을 반환한다.
    o3d.io.write_pose_graph(str(path), graph)
    if not path.is_file() or path.stat().st_size == 0:
        raise OSError(f"Pose Graph 저장 실패: {path}")


def collect_pairs(views, method, s, g, o3d, report, output):
    levels, evals = [], []
    for view in views:
        region = icp.registration_region(view["cloud"], s, o3d)
        levels.append([icp.prepare_cloud(region, voxel, view["T_initial"][:3, 3], method, s, o3d)
                       for voxel, _, _ in s.scales])
        evals.append(region.voxel_down_sample(s.evaluation_voxel_mm / 1000))
    reg = o3d.pipelines.registration
    for i, j in combinations(range(len(views)), 2):
        source, target = views[i], views[j]
        print(f"ICP 쌍: {source['name']} -> {target['name']}", flush=True)
        pair = {"source_id": i, "target_id": j, "source": source["name"], "target": target["name"],
                "status": "rejected", "stages": [], "rejection_reasons": []}
        report["pairs"].append(pair)
        pair["initial_overlap"] = icp.evaluation(levels[i][0], levels[j][0], np.eye(4), s.scales[0][1], o3d)
        pair["before"] = icp.evaluation(evals[i], evals[j], np.eye(4), s.evaluation_distance_mm, o3d)
        pair["rejection_reasons"] = overlap_reasons(pair["initial_overlap"], g)
        if not pair["rejection_reasons"]:
            try:
                delta = icp.register_pair(levels[i], levels[j], method, s, o3d, pair["stages"])
                pair["delta_source_to_target_base"] = delta.tolist()
                pair["T_target_depth_source_depth"] = (
                    np.linalg.inv(target["T_initial"]) @ delta @ source["T_initial"]).tolist()
                pair["after_pairwise"] = icp.evaluation(evals[i], evals[j], delta, s.evaluation_distance_mm, o3d)
                pair["correction"] = icp.correction_info(delta, source["cloud"])
                pair["inverse_correction"] = icp.correction_info(np.linalg.inv(delta), target["cloud"])
                pair["rejection_reasons"] = pair_rejection_reasons(
                    pair["after_pairwise"], pair["correction"], pair["inverse_correction"], g)
                # 그래프 가중치 역시 최종 ICP 스케일의 대응점에서 계산한다.
                info = np.asarray(reg.get_information_matrix_from_point_clouds(
                    levels[i][-1], levels[j][-1], s.scales[-1][1] / 1000, delta))
                if (info.shape != (6, 6) or not np.isfinite(info).all()
                        or not np.allclose(info, info.T) or np.linalg.eigvalsh(info).min() <= 0):
                    pair["rejection_reasons"].append("정보 행렬이 유한한 양의 정부호가 아님")
                if not pair["rejection_reasons"]:
                    pair["information"] = info.tolist()
                    metrics = pair["after_pairwise"].values()
                    pair["quality_score"] = min(v["fitness"] for v in metrics) / max(
                        0.001, max(v["inlier_rmse_mm"] for v in metrics))
                    pair["status"] = "accepted"
            except (ValueError, RuntimeError) as error:
                pair["rejection_reasons"].append(f"ICP 실패: {error}")
        print(f"  {pair['status']}: {', '.join(pair['rejection_reasons'])}", flush=True)
        icp.save_json(output / "summary.json", report)
    return [p for p in report["pairs"] if p["status"] == "accepted"], evals


def save_results(views, deltas, reference_id, output, s, o3d, report):
    before, after = o3d.geometry.PointCloud(), o3d.geometry.PointCloud()
    before_colors, after_colors = o3d.geometry.PointCloud(), o3d.geometry.PointCloud()
    palette = ([1, 0.35, 0.25], [0.2, 0.8, 0.35], [0.2, 0.5, 1], [1, 0.8, 0.15])
    for i, (view, delta) in enumerate(zip(views, deltas)):
        initial, cloud = view["T_initial"], view["cloud"]
        correction = icp.correction_info(delta, cloud)
        warnings = []
        if correction["centroid_shift_mm"] > s.warn_centroid_shift_mm:
            warnings.append("PGO 후 점군 중심 이동량이 경고 한도 초과")
        if correction["rotation_deg"] > s.warn_rotation_deg:
            warnings.append("PGO 후 회전 보정량이 경고 한도 초과")
        record = {"name": view["name"], "source": view["source"], "fixed": i == reference_id,
                  "T_base_depth_initial": initial.tolist(), "delta_base": delta.tolist(),
                  "T_base_depth_corrected": icp.validate_transform(delta @ initial).tolist(),
                  "correction": correction, "warnings": warnings}
        report["views"].append(record)
        corrected = copy.deepcopy(cloud).transform(delta)
        folder = output / view["name"]
        folder.mkdir()
        icp.save_cloud(folder / "registered_base.ply", corrected, o3d)
        icp.save_json(folder / "registration.json", record)
        before += cloud
        after += corrected
        before_colors += copy.deepcopy(cloud).paint_uniform_color(palette[i % len(palette)])
        after_colors += copy.deepcopy(corrected).paint_uniform_color(palette[i % len(palette)])
    voxel = s.output_voxel_mm / 1000
    for filename, cloud in (("merged_before.ply", before), ("merged.ply", after),
                            ("merged_before_view_colors.ply", before_colors), ("merged_view_colors.ply", after_colors)):
        icp.save_cloud(output / filename, cloud.voxel_down_sample(voxel), o3d)
    icp.save_cloud(output / "merged_raw.ply", after, o3d)
    report["raw_point_count"] = len(after.points)


def run(method, capture_dir, output_root, reference_view=None, settings=None, graph_settings=None):
    s = SETTINGS if settings is None else settings
    g = GRAPH_SETTINGS if graph_settings is None else graph_settings
    icp.validate_settings(s)
    validate_graph_settings(g)
    if method not in icp.METHOD_DIRS:
        raise ValueError(f"지원하지 않는 ICP 방식: {method}")
    output = Path(output_root).expanduser().resolve() / "icp_pose_graph" / method / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    output.mkdir(parents=True, exist_ok=False)
    report = {"schema": "offline_icp_pose_graph_v1", "status": "running", "method": method,
              "started_at": datetime.now(timezone.utc).isoformat(), "python": sys.executable,
              "settings": asdict(s), "graph_settings": asdict(g), "reference_frame": "base_link",
              "point_cloud_unit": "m", "icp_applied": False, "pose_graph_applied": False,
              "hand_eye_calibration_updated": False, "robot_pose_prior_applied": False,
              "strategy": "all_pairs_icp_then_pose_graph", "views": [], "pairs": [], "warnings": [],
              "graph_node_convention": "Delta after original T_base_depth; all initial nodes are identity",
              "notes": ["검사 통과 간선 중 품질 점수 기반 연결 트리만 certain; 나머지는 uncertain loop.",
                        "트리 간선은 자동 제거되지 않으며 검사 통과도 물리적 정합 정확도를 보장하지 않습니다.",
                        "로봇 TF는 초기값이며 prior 제약이 아닙니다. 기준 뷰의 원래 base 자세만 고정합니다.",
                        "정보 행렬은 Open3D 대응점 근사 가중치이며 실제 센서 불확실성 보정값이 아닙니다.",
                        "완료는 계산/저장 완료입니다. 로봇 운전/캘리브레이션에 자동 적용하지 않습니다."]}
    started = time.perf_counter()
    with (output / "run.log").open("w", encoding="utf-8") as stream:
        with redirect_stdout(icp.Tee(sys.stdout, stream)), redirect_stderr(icp.Tee(sys.stderr, stream)):
            print(f"ICP + Pose Graph: {method}\n입력: {capture_dir}\n저장: {output}", flush=True)
            try:
                icp.save_json(output / "summary.json", report)
                import open3d as o3d
                report["open3d_version"] = o3d.__version__
                views, report["input"] = icp.load_capture(capture_dir, method, o3d)
                names = [v["name"] for v in views]
                if reference_view is not None and reference_view not in names:
                    raise ValueError(f"기준 뷰 {reference_view!r} 없음: {names}")
                reference_id = names.index(reference_view) if reference_view is not None else 0
                report["reference_view"] = names[reference_id]
                report["node_names"] = names
                report["input_views"] = [{"name": v["name"], "source": v["source"]} for v in views]
                pairs, evals = collect_pairs(views, method, s, g, o3d, report, output)
                report["icp_applied"] = bool(pairs)
                graph = make_graph(len(views), pairs, o3d)
                report["accepted_edge_count"] = len(pairs)
                report["loop_edge_count_before"] = len(pairs) - len(views) + 1
                save_graph(output / "pose_graph_initial.json", graph, o3d)
                icp.save_json(output / "summary.json", report)
                print(f"PGO: {len(views)} nodes, {len(pairs)} edges / 고정: {names[reference_id]}", flush=True)
                deltas = optimize_graph(graph, reference_id, s, g, o3d)
                save_graph(output / "pose_graph_optimized.json", graph, o3d)
                retained = {(e.source_node_id, e.target_node_id): e for e in graph.edges}
                spanning_tree(len(views), [p for p in pairs if (p["source_id"], p["target_id"]) in retained])
                report["pose_graph_applied"] = True
                report["retained_edge_count"] = len(retained)
                report["loop_edge_count_after"] = len(retained) - len(views) + 1
                if report["loop_edge_count_after"] == 0:
                    report["warnings"].append("유효한 루프가 없어 연결 트리만 남았습니다. 여러 관계의 오차를 함께 조정하는 이점이 제한됩니다.")
                for pair in pairs:
                    i, j = pair["source_id"], pair["target_id"]
                    edge = retained.get((i, j))
                    pair["retained_after_pgo"] = edge is not None
                    pair["confidence_after_pgo"] = float(edge.confidence) if edge is not None else None
                    if edge is None:
                        report["warnings"].append(f"PGO에서 loop 제거: {pair['source']} -> {pair['target']}")
                    # 두 노드 pose에서 source -> target 상대 변환 복원.
                    relative = np.linalg.inv(deltas[j]) @ deltas[i]
                    pair["delta_source_to_target_after_pgo"] = relative.tolist()
                    pair["after_pgo"] = icp.evaluation(evals[i], evals[j], relative, s.evaluation_distance_mm, o3d)
                    pair["warnings"] = icp.review_warnings(pair["before"], pair["after_pgo"],
                                                          icp.correction_info(relative, views[i]["cloud"]), s)
                save_results(views, deltas, reference_id, output, s, o3d, report)
                has_warnings = (report["warnings"] or any(v["warnings"] for v in report["views"])
                                or any(p.get("warnings") for p in pairs))
                report["status"] = "complete_with_warnings" if has_warnings else "complete"
                for warning in report["warnings"]:
                    print(f"주의: {warning}", flush=True)
                print(f"완료 ({report['status']}): {output}", flush=True)
            except Exception:
                report["status"] = "failed"
                report["error"] = traceback.format_exc()
                (output / "error.log").write_text(report["error"], encoding="utf-8")
                print(report["error"], file=sys.stderr, flush=True)
                raise
            finally:
                report["elapsed_sec"] = time.perf_counter() - started
                report["finished_at"] = datetime.now(timezone.utc).isoformat()
                icp.save_json(output / "summary.json", report)
    return output


def main():
    parser = argparse.ArgumentParser(description="로봇 TF -> pairwise ICP -> PGO 오프라인 테스트 (하드웨어 연결 없음)")
    parser.add_argument("--capture-dir", default=str(CAPTURE_DIR))
    parser.add_argument("--output-root", default=str(OUTPUT_ROOT))
    parser.add_argument("--reference-view", default=REFERENCE_VIEW)
    parser.add_argument("--method", choices=list(icp.METHOD_DIRS), default=ICP_METHOD)
    parser.add_argument("--check", action="store_true", help="입력 검증만 수행, ICP/PGO/저장 안 함")
    parser.add_argument("--visualize", action="store_true", help="계산/저장 후 결과 보기")
    parser.add_argument("--view", help="기존 결과 폴더 보기만 수행")
    parser.add_argument("--layer", choices=["before", "after", "before-colors", "after-colors"], default="after")
    args = parser.parse_args()
    try:
        if args.view:
            icp.view_result(args.view, args.layer)
            return 0
        icp.validate_settings(SETTINGS)
        validate_graph_settings(GRAPH_SETTINGS)
        if args.check:
            import open3d as o3d
            views, info = icp.load_capture(args.capture_dir, args.method, o3d)
            names = [v["name"] for v in views]
            if args.reference_view is not None and args.reference_view not in names:
                raise ValueError(f"기준 뷰가 없습니다: {args.reference_view}")
            print(f"입력 확인 완료: {info['format']}, {names}, Open3D {o3d.__version__} (겹침은 실행 시 검사)")
            return 0
        output = run(args.method, args.capture_dir, args.output_root, args.reference_view)
        if args.visualize:
            icp.view_result(output, args.layer)
        return 0
    except Exception as error:
        print(f"오류: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
