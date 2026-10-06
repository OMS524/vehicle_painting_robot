#!/usr/bin/env python3
"""저장된 스캔의 TF 초기값을 사용하는 네 가지 ICP 테스트 공통부.

하드웨어 연결/SDK 호출/원본 수정/hand-eye calibration/pose graph 최적화 없음.
첫 뷰(또는 지정 뷰)를 고정하고 나머지를 각각 기준 뷰에 맞춘다.
"""
from __future__ import annotations

import argparse
from contextlib import redirect_stderr, redirect_stdout
import copy
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import sys
import time
import traceback

# 많은 코어에서의 OpenMP 과다 병렬화를 피함. 실행 환경에 지정된 값은 유지.
os.environ.setdefault("OMP_NUM_THREADS", "8")
import numpy as np


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


def load_capture(directory, method, o3d):
    root = Path(directory).expanduser().resolve()
    if not root.is_dir():
        raise ValueError("merged.ply 하나가 아닌, manifest.json과 각 뷰가 들어 있는 촬영 폴더를 지정하세요.")
    manifest_path = root / "manifest.json"
    manifest = read_json(manifest_path)
    if manifest.get("status") != "complete" or manifest.get("reference_frame") != "base_link":
        raise ValueError("완료된 base_link 좌표계 스캔 결과만 지원합니다.")
    if (manifest.get("units", {}).get("point_cloud") == "m"
            and manifest.get("icp_applied") is False):
        schema = "three_view_scan"
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
        base_cloud = copy.deepcopy(cloud).transform(t)
        views.append({"name": name, "cloud": base_cloud, "T_initial": t,
                      "source": {"cloud": str(cloud_path), "cloud_sha256": sha256(cloud_path),
                                 "pose": str(pose_path), "pose_sha256": sha256(pose_path),
                                 "valid_points": len(cloud.points), "dropped_nonfinite": dropped}})
    if len(views) < 2:
        raise ValueError("ICP 테스트에는 유효한 뷰가 최소 2개 필요합니다.")
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


def run(method, capture_dir, output_root, reference_view=None, settings=None):
    s = Settings() if settings is None else settings
    validate_settings(s)
    if method not in METHOD_DIRS:
        raise ValueError(f"지원하지 않는 방식: {method}")
    output = Path(output_root).expanduser().resolve() / METHOD_DIRS[method] / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    output.mkdir(parents=True, exist_ok=False)
    report = {"schema": "offline_icp_test_v1", "status": "running", "method": method,
              "started_at": datetime.now(timezone.utc).isoformat(), "python": sys.executable,
              "settings": asdict(s), "omp_num_threads": os.environ.get("OMP_NUM_THREADS"),
              "reference_frame": "base_link", "point_cloud_unit": "m", "icp_applied": True,
              "pose_graph_applied": False, "hand_eye_calibration_updated": False,
              "strategy": "each_source_to_fixed_reference", "views": [],
              "notes": ["테스트 후보 결과이며 자동 승인된 정합/로봇 경로가 아닙니다.",
                        "기준 뷰의 원래 base 자세를 고정하므로 절대 좌표 오차는 남을 수 있습니다.",
                        "평가 RMSE는 고정 거리 이내의 대응점만 대상입니다. 낮은 값만으로 정확도를 판단하지 마세요.",
                        "경고가 있어도 보정 후보를 저장합니다. 실패 시 최종 통합본은 생성하지 않습니다."]}
    started = time.perf_counter()
    with (output / "run.log").open("w", encoding="utf-8") as stream:
        with redirect_stdout(Tee(sys.stdout, stream)), redirect_stderr(Tee(sys.stderr, stream)):
            print(f"ICP: {method}\n입력: {capture_dir}\n저장: {output}", flush=True)
            try:
                save_json(output / "summary.json", report)
                import open3d as o3d
                report["open3d_version"] = o3d.__version__
                views, source_info = load_capture(capture_dir, method, o3d)
                report["input"] = source_info
                ref = next((v for v in views if v["name"] == reference_view), None) if reference_view else views[0]
                if ref is None:
                    raise ValueError(f"기준 뷰 {reference_view!r} 없음. 가능한 이름: {[v['name'] for v in views]}")
                report["reference_view"] = ref["name"]
                print(f"고정 기준 뷰: {ref['name']} / {len(views)}개 뷰", flush=True)
                target_region = registration_region(ref["cloud"], s, o3d)
                target_levels = [prepare_cloud(target_region, voxel, ref["T_initial"][:3, 3], method, s, o3d)
                                 for voxel, _, _ in s.scales]
                target_eval = target_region.voxel_down_sample(s.evaluation_voxel_mm / 1000)
                before_merge, after_merge = o3d.geometry.PointCloud(), o3d.geometry.PointCloud()
                before_colors, after_colors = o3d.geometry.PointCloud(), o3d.geometry.PointCloud()
                palette = ([1, 0.35, 0.25], [0.2, 0.8, 0.35], [0.2, 0.5, 1], [1, 0.8, 0.15])
                for i, view in enumerate(views):
                    print(f"[{i + 1}/{len(views)}] {view['name']}", flush=True)
                    initial, cloud = view["T_initial"], view["cloud"]
                    record = {"name": view["name"], "source": view["source"], "fixed": view is ref,
                              "T_base_depth_initial": initial.tolist(), "status": "running", "stages": []}
                    report["views"].append(record)
                    delta = np.eye(4)
                    if view is not ref:
                        region = registration_region(cloud, s, o3d)
                        source_levels = [prepare_cloud(region, voxel, initial[:3, 3], method, s, o3d)
                                         for voxel, _, _ in s.scales]
                        source_eval = region.voxel_down_sample(s.evaluation_voxel_mm / 1000)
                        record["before"] = evaluation(source_eval, target_eval, delta, s.evaluation_distance_mm, o3d)
                        delta = register_pair(source_levels, target_levels, method, s, o3d, record["stages"])
                        record["after"] = evaluation(source_eval, target_eval, delta, s.evaluation_distance_mm, o3d)
                    record["correction"] = correction_info(delta, cloud)
                    record["warnings"] = ([] if view is ref else review_warnings(
                        record["before"], record["after"], record["correction"], s))
                    # Delta는 초기 TF 적용 후의 base 좌표에 작용하므로 왼쪽에서 곱한다.
                    corrected_t = validate_transform(delta @ initial)
                    record.update(delta_base=delta.tolist(), T_base_depth_corrected=corrected_t.tolist(), status="complete")
                    corrected = copy.deepcopy(cloud).transform(delta)
                    folder = output / view["name"]
                    folder.mkdir()
                    save_cloud(folder / "registered_base.ply", corrected, o3d)
                    save_json(folder / "registration.json", record)
                    before_merge += cloud
                    after_merge += corrected
                    before_colors += copy.deepcopy(cloud).paint_uniform_color(palette[i % len(palette)])
                    after_colors += copy.deepcopy(corrected).paint_uniform_color(palette[i % len(palette)])
                    for warning in record["warnings"]:
                        print(f"  주의: {warning}", flush=True)
                    if view is not ref:
                        b, a = record["before"]["source_to_reference"], record["after"]["source_to_reference"]
                        print(f"  공통 평가: fitness {b['fitness']:.4f} -> {a['fitness']:.4f}, "
                              f"RMSE {b['inlier_rmse_mm']} -> {a['inlier_rmse_mm']} mm", flush=True)
                    save_json(output / "summary.json", report)
                voxel = s.output_voxel_mm / 1000
                save_cloud(output / "merged_before.ply", before_merge.voxel_down_sample(voxel), o3d)
                save_cloud(output / "merged_raw.ply", after_merge, o3d)
                save_cloud(output / "merged.ply", after_merge.voxel_down_sample(voxel), o3d)
                save_cloud(output / "merged_before_view_colors.ply", before_colors.voxel_down_sample(voxel), o3d)
                save_cloud(output / "merged_view_colors.ply", after_colors.voxel_down_sample(voxel), o3d)
                report["raw_point_count"] = len(after_merge.points)
                report["status"] = "complete_with_warnings" if any(v["warnings"] for v in report["views"]) else "complete"
                print(f"완료 ({report['status']}): {output}", flush=True)
            except Exception:
                report["status"] = "failed"
                report["error"] = traceback.format_exc()
                for record in report["views"]:
                    if record["status"] == "running":
                        record["status"] = "failed"
                (output / "error.log").write_text(report["error"], encoding="utf-8")
                print(report["error"], file=sys.stderr, flush=True)
                raise
            finally:
                report["elapsed_sec"] = time.perf_counter() - started
                report["finished_at"] = datetime.now(timezone.utc).isoformat()
                save_json(output / "summary.json", report)
    return output


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


def main(method, capture_dir, output_root, reference_view=None, settings=None):
    parser = argparse.ArgumentParser(description=f"저장된 스캔의 {method} ICP 테스트 (하드웨어 연결 없음)")
    parser.add_argument("--capture-dir", default=str(capture_dir), help="manifest.json과 각 뷰가 있는 폴더")
    parser.add_argument("--reference-view", default=reference_view, help="고정할 뷰 폴더 이름. 기본: 첫 뷰")
    parser.add_argument("--output-root", default=str(output_root), help="이 아래에 방식/실행시간 폴더 생성")
    parser.add_argument("--check", action="store_true", help="입력 검증만 수행, 계산/저장 안 함")
    parser.add_argument("--visualize", action="store_true", help="계산/저장 후 결과 보기")
    parser.add_argument("--view", help="기존 결과 폴더 보기만 수행, ICP 재실행 안 함")
    parser.add_argument("--layer", choices=["before", "after", "before-colors", "after-colors"], default="after")
    args = parser.parse_args()
    try:
        if args.view:
            view_result(args.view, args.layer)
            return 0
        s = Settings() if settings is None else settings
        validate_settings(s)
        if args.check:
            import open3d as o3d
            views, info = load_capture(args.capture_dir, method, o3d)
            if args.reference_view and args.reference_view not in [v["name"] for v in views]:
                raise ValueError("지정된 기준 뷰가 없습니다.")
            print(f"입력 확인 완료: {info['format']}, {[v['name'] for v in views]}, Open3D {o3d.__version__}")
            return 0
        output = run(method, args.capture_dir, args.output_root, args.reference_view, s)
        if args.visualize:
            view_result(output, args.layer)
        return 0
    except Exception as error:
        print(f"오류: {error}", file=sys.stderr)
        return 1
