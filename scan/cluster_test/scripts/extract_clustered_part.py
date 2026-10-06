#!/usr/bin/env python3
"""저장된 점군을 DBSCAN으로 나누고 여러 촬영 시선 기준으로 부품 후보 선택.

vehicle_painting_robot_scan 환경에서 실행. 카메라/로봇 연결, SAM, 재정합 없음.
상단 설정을 수정하고 실행한다. --view 결과폴더는 저장된 결과만 표시한다.
"""
from __future__ import annotations

import argparse
import colorsys
import copy
from datetime import datetime
import hashlib
import json
from pathlib import Path
import sys
import time

import numpy as np

HERE = Path(__file__).resolve().parent

# ======================== 사용자가 수정할 설정 ========================
CAPTURE_DIR = HERE.parents[1] / "doosan_a0912_scan_test/log/20260923_220542_796308"
POINT_CLOUD_FILENAME = "merged.ply"  # 같은 촬영의 merged_raw.ply도 가능. 좌표 단위 m.
OUTPUT_ROOT = HERE.parent / "log/clustered_masked_depth"
VOXEL_SIZE_MM = 5.0             # DBSCAN 계산용. 최종 part.ply는 입력 점을 그대로 보존.
DBSCAN_EPS_MM = 12.0            # 이 거리 이내의 이웃을 연결. 집합 전체 크기 제한이 아님.
DBSCAN_MIN_POINTS = 8           # 한 점 주변의 최소 이웃 수(자기 자신 포함).
MIN_CLUSTER_VOXELS = 300        # 자동 선택 후보의 최소 다운샘플 점 수.
TARGET_POINT_BASE_MM = None     # None: 시선으로 추정. 직접 지정: [x, y, z], base_link 기준 mm.
SELECT_CLUSTER_IDS = None       # None: 자동. 수동 예: [0] 또는 [0, 3]. summary.json 번호 사용.
MAX_RAY_RMS_MM = 100.0          # 시선이 너무 어긋나면 자동 선택 중지. 군집 결과는 저장.
MAX_TARGET_DISTANCE_MM = 600.0  # 기준점과 집합 중심 사이 최대 거리. 자동 선택에만 적용.
VISUALIZE_AFTER_RUN = False     # True: 저장 후 확인 창 표시.
# =====================================================================


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def validate_settings():
    for name in ("VOXEL_SIZE_MM", "DBSCAN_EPS_MM", "MAX_RAY_RMS_MM", "MAX_TARGET_DISTANCE_MM"):
        value = globals()[name]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value) or value <= 0:
            raise ValueError(f"{name}은 0보다 큰 유한한 숫자여야 합니다.")
    for name in ("DBSCAN_MIN_POINTS", "MIN_CLUSTER_VOXELS"):
        if type(globals()[name]) is not int or globals()[name] < 1:
            raise ValueError(f"{name}은 양의 정수여야 합니다.")
    if POINT_CLOUD_FILENAME not in {"merged.ply", "merged_raw.ply"}:
        raise ValueError("POINT_CLOUD_FILENAME은 merged.ply 또는 merged_raw.ply로 지정하세요.")
    if TARGET_POINT_BASE_MM is not None:
        point = np.asarray(TARGET_POINT_BASE_MM, dtype=float)
        if point.shape != (3,) or not np.isfinite(point).all():
            raise ValueError("TARGET_POINT_BASE_MM은 유한한 [x, y, z] 좌표여야 합니다.")


def validate_transform(value):
    t = np.asarray(value, dtype=float)
    if (t.shape != (4, 4) or not np.isfinite(t).all()
            or not np.allclose(t[3], [0, 0, 0, 1], atol=1e-6)
            or not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-3)
            or not np.isclose(np.linalg.det(t[:3, :3]), 1, atol=1e-3)):
        raise ValueError("pose.json의 T_base_depth가 유효한 강체 변환이 아닙니다.")
    return t


def load_capture(directory, o3d):
    root = Path(directory).expanduser().resolve()
    manifest_path = root / "manifest.json"
    manifest = read_json(manifest_path)
    if (manifest.get("status") != "complete" or manifest.get("reference_frame") != "base_link"
            or manifest.get("units", {}).get("point_cloud") != "m"):
        raise ValueError("완료된 three_view_scan 결과(base_link 좌표, 점군 단위 m)가 필요합니다.")
    if manifest.get("icp_applied") is not False:
        raise ValueError("원래 카메라 자세와 좌표가 일치하는 ICP 미적용 점군이 필요합니다.")
    origins, directions, sources, seen = [], [], [], set()
    for view in manifest["views"]:
        if view.get("status") != "accepted":
            continue
        view_dir = (root / view["name"]).resolve()
        if view_dir.parent != root or view_dir in seen:
            raise ValueError("뷰 경로가 중복되거나 촬영 디렉토리 밖입니다.")
        seen.add(view_dir)
        path = view_dir / "pose.json"
        pose = read_json(path)
        if pose.get("accepted") is not True or pose.get("translation_unit") != "m":
            raise ValueError(f"승인되지 않았거나 단위가 다른 자세: {path}")
        t = validate_transform(pose["T_base_depth"])
        origins.append(t[:3, 3])
        # Depth optical frame의 +Z가 촬영 방향. 로봇 flange의 축이 아니다.
        directions.append(t[:3, 2] / np.linalg.norm(t[:3, 2]))
        sources.append({"path": str(path), "sha256": sha256(path)})
    if not origins:
        raise ValueError("촬영 manifest에 승인된 뷰가 없습니다.")
    path = root / POINT_CLOUD_FILENAME
    if not path.is_file():
        raise FileNotFoundError(path)
    cloud = o3d.io.read_point_cloud(str(path))
    expected = manifest.get("point_count" if POINT_CLOUD_FILENAME == "merged.ply" else "raw_point_count")
    if not len(cloud.points) or (expected is not None and len(cloud.points) != expected):
        raise ValueError("입력 점군이 비어 있거나 촬영 manifest의 점 개수와 다릅니다.")
    return cloud, np.asarray(origins), np.asarray(directions), {
        "cloud": {"path": str(path), "sha256": sha256(path)},
        "manifest": {"path": str(manifest_path), "sha256": sha256(manifest_path)}, "poses": sources}


def estimate_target(origins, directions, max_rms_mm):
    """촬영 직선까지 수직거리 제곱합 최소점. 모든 카메라 앞에 있어야 함."""
    o, d = np.asarray(origins, float), np.asarray(directions, float)
    if o.ndim != 2 or o.shape[1:] != (3,) or d.shape != o.shape or len(o) < 2:
        raise ValueError("시선 기준점 추정에는 최소 2개의 촬영 자세가 필요합니다.")
    if not np.isfinite(o).all() or not np.isfinite(d).all() or np.any(np.linalg.norm(d, axis=1) < 1e-12):
        raise ValueError("카메라 위치/방향에 유효하지 않은 값이 있습니다.")
    d = d / np.linalg.norm(d, axis=1)[:, None]
    projectors = np.eye(3)[None] - d[:, :, None] * d[:, None, :]
    a = projectors.sum(axis=0)
    condition = float(np.linalg.cond(a))
    if not np.isfinite(condition) or condition > 10000:
        raise ValueError("촬영 시선이 거의 평행하여 기준점을 안정적으로 추정할 수 없습니다.")
    target = np.linalg.solve(a, np.einsum("nij,nj->i", projectors, o))
    forward = np.einsum("ij,ij->i", target - o, d)
    errors_mm = np.linalg.norm(np.einsum("nij,nj->ni", projectors, target - o), axis=1) * 1000
    rms_mm = float(np.sqrt(np.mean(errors_mm ** 2)))
    if np.any(forward <= 0):
        raise ValueError("추정 기준점이 카메라 뒤에 있습니다. 자세/촬영 방향을 확인하세요.")
    if rms_mm > max_rms_mm:
        raise ValueError(f"시선 교차 오차 RMS {rms_mm:.1f} mm > 제한 {max_rms_mm:.1f} mm")
    return target, {"method": "least_squares_optical_axes", "condition_number": condition,
                    "rms_mm": rms_mm, "ray_residuals_mm": errors_mm.tolist(),
                    "forward_distances_mm": (forward * 1000).tolist()}


def cluster_cloud(cloud, o3d, *, voxel_mm, eps_mm, min_points, min_cluster_voxels):
    """voxel 점에서 DBSCAN 후 정확한 voxel 소속으로 입력 점 라벨 복원."""
    points = np.asarray(cloud.points)
    valid_indices = np.flatnonzero(np.isfinite(points).all(axis=1))
    if not len(valid_indices):
        raise ValueError("유효한 XYZ 점이 없습니다.")
    valid = cloud.select_by_index(valid_indices.tolist())
    size = voxel_mm / 1000.0
    voxels, _, trace = valid.voxel_down_sample_and_trace(
        size, valid.get_min_bound() - size * 0.5, valid.get_max_bound() + size * 0.5, False)
    print(f"입력 {len(points):,}점 / DBSCAN 계산용 {len(voxels.points):,}점", flush=True)
    labels = np.asarray(voxels.cluster_dbscan(eps=eps_mm / 1000.0, min_points=min_points,
                                            print_progress=False), dtype=np.int32)
    counts = np.fromiter((len(indices) for indices in trace), dtype=np.int64)
    source_indices = np.concatenate(trace)
    labels_valid = np.full(len(valid_indices), -2, dtype=np.int32)
    labels_valid[source_indices] = np.repeat(labels, counts)
    if len(source_indices) != len(valid_indices) or np.any(labels_valid == -2):
        raise RuntimeError("voxel과 원본 점의 대응이 누락되었습니다.")
    original_labels = np.full(len(points), -2, dtype=np.int32)
    original_labels[valid_indices] = labels_valid
    records = []
    xyz = np.asarray(voxels.points)
    for label in np.unique(labels):
        if label < 0:
            continue
        mask = labels == label
        q = xyz[mask]
        center = q.mean(axis=0)  # 밀도 편향을 줄이기 위해 다운샘플 점의 중심 사용.
        records.append({"id": int(label), "voxel_count": int(mask.sum()),
                        "input_point_count": int(counts[mask].sum()),
                        "eligible": bool(mask.sum() >= min_cluster_voxels),
                        "center_base_mm": (center * 1000).tolist(),
                        "bounds_min_base_mm": (q.min(axis=0) * 1000).tolist(),
                        "bounds_max_base_mm": (q.max(axis=0) * 1000).tolist()})
    return voxels, labels, original_labels, records


def choose_clusters(records, target, manual_ids, max_distance_mm):
    for record in records:
        record["target_distance_mm"] = (None if target is None else
            float(np.linalg.norm(np.asarray(record["center_base_mm"]) - target * 1000)))
    if manual_ids is not None:
        if (not isinstance(manual_ids, (list, tuple)) or not manual_ids
                or any(type(x) is not int or x < 0 for x in manual_ids)
                or len(set(manual_ids)) != len(manual_ids)):
            raise ValueError("SELECT_CLUSTER_IDS는 중복 없는 0 이상의 정수 목록이어야 합니다.")
        unknown = set(manual_ids) - {r["id"] for r in records}
        if unknown:
            raise ValueError(f"존재하지 않는 집합 번호: {sorted(unknown)}")
        return list(manual_ids)
    if target is None:
        return []
    candidates = [r for r in records if r["eligible"] and r["target_distance_mm"] <= max_distance_mm]
    candidates.sort(key=lambda r: (r["target_distance_mm"], -r["voxel_count"], r["id"]))
    return [candidates[0]["id"]] if candidates else []


def label_color(label):
    return colorsys.hsv_to_rgb((label * 0.61803398875) % 1, 0.75, 0.95)


def save_cloud(path, cloud, o3d):
    if len(cloud.points) and not o3d.io.write_point_cloud(str(path), cloud):
        raise IOError(f"점군 저장 실패: {path}")


def run(capture_dir=None, *, output_root=None, manual_ids=None, check_only=False):
    import open3d as o3d

    started = time.perf_counter()
    validate_settings()
    cloud, origins, directions, sources = load_capture(CAPTURE_DIR if capture_dir is None else capture_dir, o3d)
    target, target_info, warnings = None, {}, []
    if TARGET_POINT_BASE_MM is not None:
        target = np.asarray(TARGET_POINT_BASE_MM, float) / 1000.0
        target_info = {"method": "manual_base_point"}
    else:
        try:
            target, target_info = estimate_target(origins, directions, MAX_RAY_RMS_MM)
        except ValueError as error:
            warnings.append(str(error))
            target_info = {"method": "unavailable", "error": str(error)}
    if target is not None:
        print(f"선택 기준점 [base, mm]: {np.round(target * 1000, 2).tolist()}", flush=True)
    if check_only:
        print(f"입력/자세/Open3D 확인 완료: {len(cloud.points):,}점, {len(origins)}개 뷰. 저장하지 않았습니다.")
        for warning in warnings:
            print(f"주의: {warning}")
        return None

    voxels, voxel_labels, labels, records = cluster_cloud(
        cloud, o3d, voxel_mm=VOXEL_SIZE_MM, eps_mm=DBSCAN_EPS_MM,
        min_points=DBSCAN_MIN_POINTS, min_cluster_voxels=MIN_CLUSTER_VOXELS)
    manual_ids = SELECT_CLUSTER_IDS if manual_ids is None else manual_ids
    selected_ids = choose_clusters(records, target, manual_ids, MAX_TARGET_DISTANCE_MM)
    kept = np.isin(labels, selected_ids)
    if not selected_ids:
        warnings.append("부품 선택 실패. 집합 시각화 확인 후 SELECT_CLUSTER_IDS 또는 TARGET_POINT_BASE_MM을 지정하세요.")
    candidates = sorted((r for r in records if r["eligible"]),
                        key=lambda r: (r["target_distance_mm"] if r["target_distance_mm"] is not None else float("inf"), r["id"]))
    if manual_ids is None and len(candidates) >= 2 and target is not None:
        margin = candidates[1]["target_distance_mm"] - candidates[0]["target_distance_mm"]
        if margin < 50:
            warnings.append(f"가까운 두 집합의 중심 거리 차이가 {margin:.1f} mm뿐입니다. 선택 결과를 눈으로 확인하세요.")
    warnings.append("DBSCAN은 물체 의미를 알지 못합니다. 부품/거치대가 연결되면 함께 남고, 분리된 부품 조각은 누락될 수 있습니다.")
    for r in candidates:
        distance = "N/A" if r["target_distance_mm"] is None else f"{r['target_distance_mm']:.1f} mm"
        marker = " [선택]" if r["id"] in selected_ids else ""
        print(f"집합 {r['id']}: 원본 {r['input_point_count']:,}점 / 중심 거리 {distance}{marker}", flush=True)

    output = Path(OUTPUT_ROOT if output_root is None else output_root).expanduser().resolve() / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    output.mkdir(parents=True, exist_ok=False)
    report = {"status": "running", "schema": "dbscan_part_extraction_v1", "python": sys.executable,
              "open3d_version": o3d.__version__, "reference_frame": "base_link", "point_cloud_unit": "m",
              "sources": sources, "input_points": len(cloud.points), "clustering_points": len(voxels.points),
              "settings": {"voxel_size_mm": VOXEL_SIZE_MM, "dbscan_eps_mm": DBSCAN_EPS_MM,
                           "dbscan_min_points": DBSCAN_MIN_POINTS, "min_cluster_voxels": MIN_CLUSTER_VOXELS,
                           "max_ray_rms_mm": MAX_RAY_RMS_MM, "max_target_distance_mm": MAX_TARGET_DISTANCE_MM},
              "target": dict(target_info, point_base_mm=None if target is None else (target * 1000).tolist()),
              "rays": {"origins_base_m": origins.tolist(), "directions_base": directions.tolist()},
              "selection_method": "manual_ids" if manual_ids is not None else "nearest_voxel_centroid",
              "selected_cluster_ids": selected_ids, "selected_points": int(kept.sum()),
              "noise_points": int((labels == -1).sum()), "invalid_points": int((labels == -2).sum()),
              "labels": {"file": "cluster_labels.npy", "order": "input PLY point order", "noise": -1, "invalid": -2},
              "clusters": records, "warnings": warnings, "manually_verified": False}
    try:
        np.save(output / "cluster_labels.npy", labels, allow_pickle=False)
        preview = copy.deepcopy(voxels)
        colors = np.full((len(voxel_labels), 3), 0.22)
        for r in records:
            r["preview_color_rgb"] = list(label_color(r["id"]))
            colors[voxel_labels == r["id"]] = r["preview_color_rgb"]
        preview.colors = o3d.utility.Vector3dVector(colors)
        save_cloud(output / "clusters_colored.ply", preview, o3d)
        selected_preview = copy.deepcopy(voxels)
        colors = np.full((len(voxel_labels), 3), 0.22)
        colors[np.isin(voxel_labels, selected_ids)] = [0.15, 1.0, 0.3]
        selected_preview.colors = o3d.utility.Vector3dVector(colors)
        save_cloud(output / "selection_preview.ply", selected_preview, o3d)
        candidate_dir = output / "clusters"
        candidate_dir.mkdir()
        for r in records:
            if r["eligible"] or r["id"] in selected_ids:
                relative = f"clusters/cluster_{r['id']:03d}.ply"
                save_cloud(output / relative, cloud.select_by_index(np.flatnonzero(labels == r["id"]).tolist()), o3d)
                r["file"] = relative
        if selected_ids:
            save_cloud(output / "part.ply", cloud.select_by_index(np.flatnonzero(kept).tolist()), o3d)
        report["status"] = "complete" if selected_ids else "needs_selection"
    except BaseException as error:
        report.update(status="failed", error=f"{type(error).__name__}: {error}")
        raise
    finally:
        report["elapsed_sec"] = time.perf_counter() - started
        write_json(output / "summary.json", report)
    for warning in warnings:
        print(f"주의: {warning}")
    print(f"완료: {output}\n선택 집합: {selected_ids} / {int(kept.sum()):,}점", flush=True)
    return output


def view_result(directory, layer="selection"):
    import open3d as o3d

    root = Path(directory).expanduser().resolve()
    report = read_json(root / "summary.json")
    names = {"selection": "selection_preview.ply", "clusters": "clusters_colored.ply", "part": "part.ply"}
    path = root / names[layer]
    if not path.is_file():
        raise FileNotFoundError(f"표시할 파일이 없습니다: {path}")
    geometries = [o3d.io.read_point_cloud(str(path))]
    center = report["target"]["point_base_mm"]
    if center is not None:
        center = np.asarray(center) / 1000.0
        sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.02)
        sphere.compute_vertex_normals()
        sphere.paint_uniform_color([1, 0.2, 0.1])
        sphere.translate(center)
        geometries.append(sphere)
        origins = np.asarray(report["rays"]["origins_base_m"])
        directions = np.asarray(report["rays"]["directions_base"])
        lengths = np.maximum(np.einsum("ij,ij->i", center - origins, directions), 0.1)
        # 실제 광축 직선이다. 카메라를 기준점에 직접 잇는 선으로 대체하지 않는다.
        endpoints = origins + directions * lengths[:, None]
        lines = o3d.geometry.LineSet(
            o3d.utility.Vector3dVector(np.vstack((origins, endpoints))),
            o3d.utility.Vector2iVector([[i, i + len(origins)] for i in range(len(origins))]))
        lines.paint_uniform_color([0.2, 0.8, 1.0])
        geometries.append(lines)
    o3d.visualization.draw_geometries(geometries, window_name=f"DBSCAN - {layer} - {root.name}",
                                    width=1280, height=800)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture-dir", type=Path, help="저장된 three_view_scan 촬영 폴더")
    parser.add_argument("--check", action="store_true", help="입력/자세/의존성만 확인, 저장하지 않음")
    parser.add_argument("--cluster-ids", type=int, nargs="+", help="자동 선택 대신 집합 번호 지정")
    parser.add_argument("--visualize", action="store_true", help="추출 후 시각화")
    parser.add_argument("--view", type=Path, help="기존 결과 폴더만 표시. 추출을 다시 하지 않음")
    parser.add_argument("--layer", choices=["selection", "clusters", "part"], default="selection")
    args = parser.parse_args()
    try:
        if args.view is not None:
            view_result(args.view, args.layer)
        else:
            output = run(args.capture_dir, manual_ids=args.cluster_ids, check_only=args.check)
            if output is not None and (args.visualize or VISUALIZE_AFTER_RUN):
                view_result(output, args.layer)
    except Exception as error:
        print(f"오류: {error}", file=sys.stderr)
        raise SystemExit(1) from error


if __name__ == "__main__":
    main()
