"""2단계: TF 초기 배치·병합, ICP 및 PGO 보정. 원본 데이터는 변경하지 않는다."""
from __future__ import annotations

import copy
import hashlib
import json
import os
from pathlib import Path
import time
from contextlib import redirect_stderr, redirect_stdout
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from itertools import combinations
import sys
import traceback

# 기존 실행 환경의 OpenMP 설정은 유지한다.
os.environ.setdefault("OMP_NUM_THREADS", "8")
import numpy as np


# ========================================================================
# ICP 계산·평가·입출력
# ========================================================================

METHOD_DIRS = {
    "point_to_point": "point-to-point_ICP",
    "point_to_plane": "point-to-plane_ICP",
    "generalized": "generalized_ICP",
    "colored": "colored_ICP",
}


@dataclass(frozen=True)
class Settings:
    # (계산용 voxel mm, 대응점 최대 거리 mm, 최대 반복 횟수), 성김 -> 촘촘함.
    scales: tuple = ((10.0, 30.0, 60), (5.0, 15.0, 40), (2.0, 6.0, 30))
    evaluation_voxel_mm: float = 5.0
    evaluation_distance_mm: float = 10.0
    output_voxel_mm: float = 2.0
    normal_radius_factor: float = 3.0
    normal_max_nn: int = 30
    relative_fitness: float = 1e-6
    relative_rmse: float = 1e-6
    colored_lambda_geometric: float = 0.968
    generalized_epsilon: float = 0.001
    # 초기 TF 기준 base 좌표 ROI. 정합/평가에만 적용하고 출력 전체 점은 보존.
    roi_min_mm: tuple | None = None
    roi_max_mm: tuple | None = None
    # 아래 값은 검토 경고용이며 자동 승인/로봇 운전 허용 기준이 아니다.
    warn_min_fitness: float = 0.15
    warn_centroid_shift_mm: float = 50.0
    warn_rotation_deg: float = 5.0


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def save_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2,
                                   allow_nan=False) + "\n", encoding="utf-8")


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def validate_transform(value):
    t = np.asarray(value, dtype=float)
    if (t.shape != (4, 4) or not np.isfinite(t).all()
            or not np.allclose(t[3], [0, 0, 0, 1], atol=1e-7)
            or not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(t[:3, :3]), 1, atol=1e-5)):
        raise ValueError("유효한 4x4 강체 변환이 아닙니다.")
    return t


def validate_settings(s):
    positive = (s.evaluation_voxel_mm, s.evaluation_distance_mm, s.output_voxel_mm,
                s.normal_radius_factor, s.relative_fitness, s.relative_rmse,
                s.generalized_epsilon, s.warn_centroid_shift_mm, s.warn_rotation_deg)
    if any(not np.isfinite(x) or x <= 0 for x in positive):
        raise ValueError("거리/수렴/경고 파라미터는 유한한 양수여야 합니다.")
    if type(s.normal_max_nn) is not int or s.normal_max_nn < 3:
        raise ValueError("normal_max_nn은 3 이상의 정수여야 합니다.")
    if not 0 < s.colored_lambda_geometric < 1 or not 0 <= s.warn_min_fitness <= 1:
        raise ValueError("colored_lambda_geometric은 0~1 사이, warn_min_fitness는 0~1이어야 합니다.")
    previous = float("inf")
    if not s.scales:
        raise ValueError("ICP 계산 스케일이 비어 있습니다.")
    for voxel, distance, iterations in s.scales:
        if (not np.isfinite([voxel, distance]).all() or not 0 < voxel < previous
                or distance <= 0 or type(iterations) is not int or iterations < 1):
            raise ValueError("scales: voxel은 감소하는 양수, 거리는 양수, 반복은 양의 정수여야 합니다.")
        previous = voxel
    if (s.roi_min_mm is None) != (s.roi_max_mm is None):
        raise ValueError("ROI 최소/최대는 함께 지정하세요.")
    if s.roi_min_mm is not None:
        lo, hi = np.asarray(s.roi_min_mm), np.asarray(s.roi_max_mm)
        if (lo.shape != (3,) or hi.shape != (3,) or not np.isfinite([lo, hi]).all()
                or np.any(lo >= hi)):
            raise ValueError("ROI는 최소 < 최대인 유한한 XYZ(mm)여야 합니다.")


def load_capture(directory, method, o3d, min_views=2):
    root = Path(directory).expanduser().resolve()
    if not root.is_dir():
        raise ValueError("merged.ply 하나가 아닌, manifest.json과 각 뷰가 들어 있는 촬영 폴더를 지정하세요.")
    manifest_path = root / "manifest.json"
    manifest = read_json(manifest_path)
    if manifest.get("status") != "complete" or manifest.get("reference_frame") != "base_link":
        raise ValueError("완료된 base_link 좌표계 스캔 결과만 지원합니다.")
    if manifest.get("schema") == "scan_geometry_v1":
        schema = "scan_geometry_v1"
        if manifest.get("units", {}).get("point_cloud") != "m":
            raise ValueError("재구성 점군 단위는 m여야 합니다.")
        if not manifest.get("views") or any(v.get("status") != "accepted" for v in manifest["views"]):
            raise ValueError("모든 재구성 뷰가 accepted여야 합니다.")
        entries = [(v["name"], v["cloud"], v["point_count"]) for v in manifest["views"]]
        if any(Path(filename).name != filename for _, filename, _ in entries):
            raise ValueError("점군 파일명은 뷰 폴더 내부 파일이어야 합니다.")
        if method == "colored" and manifest.get("color_mode") != "rgb":
            raise ValueError("Colored ICP에는 실제 RGB가 필요합니다.")
    elif (manifest.get("units", {}).get("point_cloud") == "m"
            and manifest.get("icp_applied") is False):
        schema = "three_view_scan"
        if not manifest.get("views") or any(v.get("status") != "accepted" for v in manifest["views"]):
            raise ValueError("모든 촬영 뷰가 accepted여야 합니다.")
        entries = [(v["name"], "camera_filtered.ply", v.get("point_count"))
                   for v in manifest["views"] if v.get("status") == "accepted"]
        if method == "colored" and manifest.get("color_mode") != "rgb":
            raise ValueError("Colored ICP에는 실제 RGB가 필요합니다. 뷰 구분용 인공 색상은 사용할 수 없습니다.")
    elif (manifest.get("point_cloud_unit") == "m"
          and manifest.get("method") == "saved_T_base_depth_no_ICP"):
        schema = "sam_masked_depth"
        # 디렉토리가 이동돼도 결과 내부 복사본 사용. 원본 경로로 fallback하지 않음.
        entries = [(Path(v["output"]).name, "part_camera.ply", v.get("selected_points"))
                   for v in manifest["views"]]
    else:
        raise ValueError("ICP 미적용 three_view_scan 또는 SAM masked-depth 결과가 필요합니다.")
    views, seen = [], set()
    for name, filename, expected_count in entries:
        if not isinstance(name, str) or name in seen or Path(name).name != name:
            raise ValueError("중복되거나 잘못된 뷰 이름입니다.")
        folder = (root / name).resolve()
        if folder.parent != root:
            raise ValueError("뷰 폴더는 촬영 폴더 바로 아래에 있어야 합니다.")
        seen.add(name)
        pose_path, cloud_path = folder / "pose.json", folder / filename
        pose = read_json(pose_path)
        if pose.get("accepted") is not True or pose.get("translation_unit") != "m":
            raise ValueError(f"승인되지 않았거나 단위가 다른 자세: {pose_path}")
        t = validate_transform(pose["T_base_depth"])
        cloud = o3d.io.read_point_cloud(str(cloud_path))
        if not len(cloud.points) or (expected_count is not None and len(cloud.points) != expected_count):
            raise ValueError(f"점군이 비었거나 manifest 점 개수와 다릅니다: {cloud_path}")
        valid = np.isfinite(np.asarray(cloud.points)).all(axis=1)
        dropped = int((~valid).sum())
        cloud = cloud.select_by_index(np.flatnonzero(valid).tolist())
        if len(cloud.points) < 30:
            raise ValueError(f"{name}: 유효점이 30개 미만입니다.")
        if method == "colored":
            colors = np.asarray(cloud.colors)
            if (not cloud.has_colors() or not np.isfinite(colors).all()
                    or np.any(colors < 0) or np.any(colors > 1)):
                raise ValueError(f"{name}: Colored ICP에 필요한 유효한 RGB가 없습니다.")
        # 카메라 좌표에 TF를 딱 한 번 적용. 이미 base인 PLY에 다시 TF를 곱하지 않음.
        base_cloud = copy.deepcopy(cloud)
        if schema != "scan_geometry_v1":
            base_cloud.transform(t)  # 재구성 결과는 이미 base 좌표이므로 중복 변환 금지.
        views.append({"name": name, "cloud": base_cloud, "T_initial": t,
                      "source": {"cloud": str(cloud_path), "cloud_sha256": sha256(cloud_path),
                                 "pose": str(pose_path), "pose_sha256": sha256(pose_path),
                                 "valid_points": len(cloud.points), "dropped_nonfinite": dropped}})
    if len(views) < min_views:
        raise ValueError(f"유효한 뷰가 최소 {min_views}개 필요합니다.")
    return views, {"directory": str(root), "format": schema,
                   "manifest_sha256": sha256(manifest_path)}


def registration_region(cloud, s, o3d):
    if s.roi_min_mm is None:
        return copy.deepcopy(cloud)
    box = o3d.geometry.AxisAlignedBoundingBox(np.asarray(s.roi_min_mm) / 1000,
                                             np.asarray(s.roi_max_mm) / 1000)
    return cloud.crop(box)


def prepare_cloud(cloud, voxel_mm, origin, method, s, o3d):
    result = cloud.voxel_down_sample(voxel_mm / 1000)
    if len(result.points) < 30:
        raise ValueError("ROI/다운샘플링 이후 계산용 점이 30개 미만입니다.")
    if method != "point_to_point":
        result.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(
            radius=voxel_mm / 1000 * s.normal_radius_factor, max_nn=s.normal_max_nn))
        result.orient_normals_towards_camera_location(origin)
        result.normalize_normals()
    return result


def evaluation(source, target, transform, distance_mm, o3d):
    def one(a, b, t):
        result = o3d.pipelines.registration.evaluate_registration(a, b, distance_mm / 1000, t)
        count = len(result.correspondence_set)
        return {"source_points": len(a.points), "target_points": len(b.points),
                "correspondences": count, "fitness": float(result.fitness),
                "inlier_rmse_mm": float(result.inlier_rmse * 1000) if count else None}
    return {"source_to_reference": one(source, target, transform),
            "reference_to_source": one(target, source, np.linalg.inv(transform))}


def register_pair(source, target, method, s, o3d, stages=None):
    """입력은 초기 TF 적용 base 점군. 반환 Delta는 base에서 추가 적용할 변환."""
    reg = o3d.pipelines.registration
    delta = np.eye(4)
    stages = [] if stages is None else stages
    for source_level, target_level, (voxel, distance, iterations) in zip(source, target, s.scales):
        initial = reg.evaluate_registration(source_level, target_level, distance / 1000, delta)
        if len(initial.correspondence_set) < 3:
            raise ValueError(f"voxel {voxel:g} mm: 대응점이 부족합니다. 겹침/TF/대응 거리/ROI를 확인하세요.")
        criteria = reg.ICPConvergenceCriteria(s.relative_fitness, s.relative_rmse, iterations)
        started = time.perf_counter()
        if method == "point_to_point":
            result = reg.registration_icp(source_level, target_level, distance / 1000, delta,
                                          reg.TransformationEstimationPointToPoint(False), criteria)
        elif method == "point_to_plane":
            result = reg.registration_icp(source_level, target_level, distance / 1000, delta,
                                          reg.TransformationEstimationPointToPlane(), criteria)
        elif method == "generalized":
            result = reg.registration_generalized_icp(source_level, target_level, distance / 1000, delta,
                reg.TransformationEstimationForGeneralizedICP(s.generalized_epsilon), criteria)
        elif method == "colored":
            result = reg.registration_colored_icp(source_level, target_level, distance / 1000, delta,
                reg.TransformationEstimationForColoredICP(s.colored_lambda_geometric), criteria)
        else:
            raise ValueError(f"지원하지 않는 방식: {method}")
        elapsed = time.perf_counter() - started
        delta = validate_transform(result.transformation)
        if len(result.correspondence_set) < 3:
            raise ValueError(f"voxel {voxel:g} mm: ICP 결과의 대응점이 부족합니다.")
        stages.append({"voxel_mm": voxel, "max_correspondence_mm": distance,
                       "max_iterations": iterations, "registration_sec": elapsed,
                       "fitness": float(result.fitness), "inlier_rmse_mm": float(result.inlier_rmse * 1000),
                       "correspondences": len(result.correspondence_set), "delta_base": delta.tolist()})
        print(f"  voxel {voxel:g} mm: fitness={result.fitness:.4f}, "
              f"RMSE={result.inlier_rmse * 1000:.3f} mm, {elapsed:.3f}초", flush=True)
    return delta


def correction_info(delta, cloud):
    angle = np.degrees(np.arccos(np.clip((np.trace(delta[:3, :3]) - 1) / 2, -1, 1)))
    center = np.asarray(cloud.get_center())
    shift = delta[:3, :3] @ center + delta[:3, 3] - center
    return {"rotation_deg": float(angle), "translation_vector_base_mm": (delta[:3, 3] * 1000).tolist(),
            "translation_norm_mm": float(np.linalg.norm(delta[:3, 3]) * 1000),
            "centroid_shift_mm": float(np.linalg.norm(shift) * 1000)}


def review_warnings(before, after, correction, s):
    warnings = []
    for direction in ("source_to_reference", "reference_to_source"):
        b, a = before[direction], after[direction]
        if a["correspondences"] < 30 or a["fitness"] < s.warn_min_fitness:
            warnings.append(f"{direction}: 보정 후 유효 대응점/겹침이 부족합니다.")
        if a["fitness"] < b["fitness"] - 0.02:
            warnings.append(f"{direction}: 동일 평가 조건에서 fitness가 0.02 초과 감소했습니다.")
        if (a["inlier_rmse_mm"] is not None and b["inlier_rmse_mm"] is not None
                and a["inlier_rmse_mm"] > b["inlier_rmse_mm"] + 0.1):
            warnings.append(f"{direction}: 동일 평가 조건에서 RMSE가 0.1 mm 초과 증가했습니다.")
    if correction["centroid_shift_mm"] > s.warn_centroid_shift_mm:
        warnings.append("점군 중심 이동량이 경고 한도를 초과했습니다.")
    if correction["rotation_deg"] > s.warn_rotation_deg:
        warnings.append("회전 보정량이 경고 한도를 초과했습니다.")
    return warnings


def save_cloud(path, cloud, o3d):
    if not len(cloud.points) or not o3d.io.write_point_cloud(str(path), cloud):
        raise OSError(f"PLY 저장 실패: {path}")


class Tee:
    def __init__(self, terminal, log):
        self.terminal, self.log = terminal, log

    def write(self, text):
        self.terminal.write(text)
        self.log.write(text)
        return len(text)

    def flush(self):
        self.terminal.flush()
        self.log.flush()

def view_result(directory, layer="after"):
    import open3d as o3d
    root = Path(directory).expanduser().resolve()
    names = {"before": "merged_before.ply", "after": "merged.ply",
             "before-colors": "merged_before_view_colors.ply", "after-colors": "merged_view_colors.ply"}
    path = root / names[layer]
    if not path.is_file():
        raise FileNotFoundError(path)
    cloud = o3d.io.read_point_cloud(str(path))
    if not len(cloud.points):
        raise ValueError(f"비어 있는 점군: {path}")
    o3d.visualization.draw_geometries([cloud], window_name=f"ICP {layer} | {root.name}", width=1280, height=900)


# ========================================================================
# 실측 TF 기반 초기 배치·병합
# ========================================================================

def geometry_manifest(output, views, source, *, corrected, status="complete"):
    import open3d as o3d
    output = Path(output)
    merged = o3d.io.read_point_cloud(str(output / "merged.ply"))
    raw = o3d.io.read_point_cloud(str(output / "merged_raw.ply"))
    manifest = {
        "schema": "scan_geometry_v1", "status": "complete",
        "calculation_status": status, "reference_frame": "base_link",
        "units": {"point_cloud": "m"}, "icp_applied": corrected,
        "source_directory": str(Path(source).resolve()),
        "point_count": len(merged.points), "raw_point_count": len(raw.points),
        "views": views,
    }
    save_json(output / "manifest.json", manifest)
    return manifest


def reconstruct(capture_dir, output_dir, voxel_mm=2.0):
    """SDK가 저장한 카메라 점군에 실측 T_base_depth를 한 번 적용해 병합."""
    import open3d as o3d
    if not np.isfinite(voxel_mm) or voxel_mm <= 0:
        raise ValueError("재구성 voxel_mm은 양수여야 합니다.")
    source = Path(capture_dir).resolve()
    manifest = read_json(source / "manifest.json")
    if manifest.get("schema") == "scan_geometry_v1" or manifest.get("icp_applied") is not False:
        raise ValueError("재구성 입력은 보정 전 원본 취득 폴더여야 합니다.")
    entries = manifest.get("views", [])
    if not entries or any(v.get("status") != "accepted" for v in entries):
        raise ValueError("모든 촬영 뷰가 accepted여야 합니다. 일부만 골라 병합하지 않습니다.")
    views, info = load_capture(source, "point_to_point", o3d, min_views=1)
    output = Path(output_dir).resolve()
    output.mkdir(parents=True, exist_ok=False)
    merged, colored, records = o3d.geometry.PointCloud(), o3d.geometry.PointCloud(), []
    for i, view in enumerate(views):
        name, cloud = view["name"], view["cloud"]
        folder = output / name
        folder.mkdir()
        save_cloud(folder / "base.ply", cloud, o3d)
        save_json(folder / "pose.json", {
            "accepted": True, "translation_unit": "m",
            "T_base_depth": view["T_initial"].tolist(),
            "source": view["source"], "icp_applied": False,
        })
        merged += cloud
        import colorsys
        colored += copy.deepcopy(cloud).paint_uniform_color(colorsys.hsv_to_rgb(i / len(views), .75, 1))
        records.append({"name": name, "status": "accepted", "cloud": "base.ply",
                        "point_count": len(cloud.points)})
    save_cloud(output / "merged_raw.ply", merged, o3d)
    save_cloud(output / "merged.ply", merged.voxel_down_sample(voxel_mm / 1000), o3d)
    save_cloud(output / "merged_view_colors.ply", colored.voxel_down_sample(voxel_mm / 1000), o3d)
    result = geometry_manifest(output, records, source, corrected=False)
    result.update(color_mode=manifest.get("color_mode"), reconstruction_voxel_mm=voxel_mm, input=info)
    save_json(output / "manifest.json", result)
    return output


# ========================================================================
# 뷰 쌍 ICP·Pose Graph Optimization
# ========================================================================

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
    anchor_inv = np.linalg.inv(validate_transform(graph.nodes[reference_id].pose))
    for node in graph.nodes:
        node.pose = validate_transform(anchor_inv @ node.pose)
    return [np.asarray(node.pose).copy() for node in graph.nodes]


def save_graph(path, graph, o3d):
    # Open3D 0.18의 Python 바인딩은 성공해도 None을 반환한다.
    o3d.io.write_pose_graph(str(path), graph)
    if not path.is_file() or path.stat().st_size == 0:
        raise OSError(f"Pose Graph 저장 실패: {path}")


def collect_pairs(views, method, s, g, o3d, report, output):
    levels, evals = [], []
    for view in views:
        region = registration_region(view["cloud"], s, o3d)
        levels.append([prepare_cloud(region, voxel, view["T_initial"][:3, 3], method, s, o3d)
                       for voxel, _, _ in s.scales])
        evals.append(region.voxel_down_sample(s.evaluation_voxel_mm / 1000))
    reg = o3d.pipelines.registration
    for i, j in combinations(range(len(views)), 2):
        source, target = views[i], views[j]
        print(f"ICP 쌍: {source['name']} -> {target['name']}", flush=True)
        pair = {"source_id": i, "target_id": j, "source": source["name"], "target": target["name"],
                "status": "rejected", "stages": [], "rejection_reasons": []}
        report["pairs"].append(pair)
        pair["initial_overlap"] = evaluation(levels[i][0], levels[j][0], np.eye(4), s.scales[0][1], o3d)
        pair["before"] = evaluation(evals[i], evals[j], np.eye(4), s.evaluation_distance_mm, o3d)
        pair["rejection_reasons"] = overlap_reasons(pair["initial_overlap"], g)
        if not pair["rejection_reasons"]:
            try:
                delta = register_pair(levels[i], levels[j], method, s, o3d, pair["stages"])
                pair["delta_source_to_target_base"] = delta.tolist()
                pair["T_target_depth_source_depth"] = (
                    np.linalg.inv(target["T_initial"]) @ delta @ source["T_initial"]).tolist()
                pair["after_pairwise"] = evaluation(evals[i], evals[j], delta, s.evaluation_distance_mm, o3d)
                pair["correction"] = correction_info(delta, source["cloud"])
                pair["inverse_correction"] = correction_info(np.linalg.inv(delta), target["cloud"])
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
        save_json(output / "summary.json", report)
    return [p for p in report["pairs"] if p["status"] == "accepted"], evals


def save_results(views, deltas, reference_id, output, s, o3d, report):
    before, after = o3d.geometry.PointCloud(), o3d.geometry.PointCloud()
    before_colors, after_colors = o3d.geometry.PointCloud(), o3d.geometry.PointCloud()
    palette = ([1, 0.35, 0.25], [0.2, 0.8, 0.35], [0.2, 0.5, 1], [1, 0.8, 0.15])
    for i, (view, delta) in enumerate(zip(views, deltas)):
        initial, cloud = view["T_initial"], view["cloud"]
        correction = correction_info(delta, cloud)
        warnings = []
        if correction["centroid_shift_mm"] > s.warn_centroid_shift_mm:
            warnings.append("PGO 후 점군 중심 이동량이 경고 한도 초과")
        if correction["rotation_deg"] > s.warn_rotation_deg:
            warnings.append("PGO 후 회전 보정량이 경고 한도 초과")
        record = {"name": view["name"], "source": view["source"], "fixed": i == reference_id,
                  "T_base_depth_initial": initial.tolist(), "delta_base": delta.tolist(),
                  "T_base_depth_corrected": validate_transform(delta @ initial).tolist(),
                  "correction": correction, "warnings": warnings}
        report["views"].append(record)
        corrected = copy.deepcopy(cloud).transform(delta)
        folder = output / view["name"]
        folder.mkdir()
        save_cloud(folder / "registered_base.ply", corrected, o3d)
        save_json(folder / "registration.json", record)
        save_json(folder / "pose.json", {
            "accepted": True, "translation_unit": "m",
            "T_base_depth": record["T_base_depth_corrected"],
            "T_base_depth_initial": record["T_base_depth_initial"], "icp_applied": True,
        })
        before += cloud
        after += corrected
        before_colors += copy.deepcopy(cloud).paint_uniform_color(palette[i % len(palette)])
        after_colors += copy.deepcopy(corrected).paint_uniform_color(palette[i % len(palette)])
    voxel = s.output_voxel_mm / 1000
    for filename, cloud in (("merged_before.ply", before), ("merged.ply", after),
                            ("merged_before_view_colors.ply", before_colors), ("merged_view_colors.ply", after_colors)):
        save_cloud(output / filename, cloud.voxel_down_sample(voxel), o3d)
    save_cloud(output / "merged_raw.ply", after, o3d)
    report["raw_point_count"] = len(after.points)


def run(method, capture_dir, output_dir, reference_view=None, settings=None, graph_settings=None):
    s = Settings() if settings is None else settings
    g = GraphSettings() if graph_settings is None else graph_settings
    validate_settings(s)
    validate_graph_settings(g)
    if method not in METHOD_DIRS:
        raise ValueError(f"지원하지 않는 ICP 방식: {method}")
    output = Path(output_dir).expanduser().resolve()
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
        with redirect_stdout(Tee(sys.stdout, stream)), redirect_stderr(Tee(sys.stderr, stream)):
            print(f"ICP + Pose Graph: {method}\n입력: {capture_dir}\n저장: {output}", flush=True)
            try:
                save_json(output / "summary.json", report)
                import open3d as o3d
                report["open3d_version"] = o3d.__version__
                views, report["input"] = load_capture(capture_dir, method, o3d)
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
                save_json(output / "summary.json", report)
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
                    pair["after_pgo"] = evaluation(evals[i], evals[j], relative, s.evaluation_distance_mm, o3d)
                    pair["warnings"] = review_warnings(pair["before"], pair["after_pgo"],
                                                          correction_info(relative, views[i]["cloud"]), s)
                save_results(views, deltas, reference_id, output, s, o3d, report)
                has_warnings = (report["warnings"] or any(v["warnings"] for v in report["views"])
                                or any(p.get("warnings") for p in pairs))
                report["status"] = "complete_with_warnings" if has_warnings else "complete"

                geometry = geometry_manifest(output, [
                    {"name": v["name"], "status": "accepted", "cloud": "registered_base.ply",
                     "point_count": len(v["cloud"].points)} for v in views],
                    capture_dir, corrected=True, status=report["status"])
                geometry["color_mode"] = read_json(Path(capture_dir) / "manifest.json").get("color_mode")
                save_json(output / "manifest.json", geometry)
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
                save_json(output / "summary.json", report)
    return output
