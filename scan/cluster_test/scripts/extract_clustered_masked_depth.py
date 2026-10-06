#!/usr/bin/env python3
"""SAM 마스크로 추출·정합한 점군에 DBSCAN을 적용한다.

실행: conda activate vehicle_painting_robot_scan
      python extract_clustered_masked_depth.py --visualize
기존 결과 보기: python extract_clustered_masked_depth.py --view /결과/폴더

입력은 extract_masked_depth.py가 저장한 결과 폴더이다. merged.ply,
manifest.json과 각 출력 뷰의 pose.json만 읽는다. SAM/SDK/로봇 재실행 없음.
DBSCAN/시선 추정/집합 선택/뷰어는 같은 폴더의 extract_clustered_part.py를
가져와 사용하며 해당 파일이나 그 모듈의 설정은 변경하지 않는다.
최종 part.ply에는 선택된 입력 점의 XYZ/RGB를 그대로 저장한다.
거치대가 부품에 연결되어 같은 집합이 되면 이 방법으로 분리되지 않는다.
"""
from __future__ import annotations

import argparse
import copy
from datetime import datetime
from pathlib import Path
import sys
import time

import numpy as np

import extract_clustered_part as cluster

HERE = Path(__file__).resolve().parent

# ======================== 사용자가 수정할 설정 ========================
CAPTURE_DIR = HERE.parents[1] / "sam_test/log/sam3_masked_depth/20260926_070101_635768"
POINT_CLOUD_FILENAME = "merged.ply"  # 전체 통합 원본 점을 쓰려면 merged_raw.ply.
OUTPUT_ROOT = HERE.parent / "log/clustered_masked_depth"
VOXEL_SIZE_MM = 5.0             # 계산용 다운샘플 크기. 출력에는 입력 점을 보존.
DBSCAN_EPS_MM = 12.0            # 이웃 연결 거리. 집합 전체 크기 제한이 아님.
DBSCAN_MIN_POINTS = 8           # 핵심점 주변 최소 이웃 수(자기 자신 포함).
MIN_CLUSTER_VOXELS = 300        # 자동 선택 가능한 집합의 최소 계산용 점 수.
TARGET_POINT_BASE_MM = None     # None: 촬영 시선으로 추정. 수동: [x, y, z] mm.
SELECT_CLUSTER_IDS = None       # None: 자동 선택. 수동 예: [0] 또는 [0, 3].
MAX_RAY_RMS_MM = 100.0          # 시선 추정 오차 한도. 초과하면 수동 선택 필요.
MAX_TARGET_DISTANCE_MM = 600.0  # 자동 선택할 집합 중심과 기준점의 최대 거리.
VISUALIZE_AFTER_RUN = False
# =====================================================================


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


def load_masked_capture(directory, o3d):
    root = Path(directory).expanduser().resolve()
    manifest_path = root / "manifest.json"
    manifest = cluster.read_json(manifest_path)
    if (manifest.get("status") != "complete" or manifest.get("reference_frame") != "base_link"
            or manifest.get("point_cloud_unit") != "m"
            or manifest.get("method") != "saved_T_base_depth_no_ICP"
            or manifest.get("icp_applied", False) is not False):
        raise ValueError("완료된 extract_masked_depth 결과(base_link, 단위 m, ICP 미적용)가 필요합니다.")
    origins, directions, sources, seen = [], [], [], set()
    for view in manifest["views"]:
        # output은 생성 당시 절대 경로. 결과 폴더를 옮겨도 같은 하위 폴더의
        # 복사된 pose만 사용한다. source_view의 원본 촬영 경로를 읽지 않는다.
        view_dir = (root / Path(view["output"]).name).resolve()
        if view_dir.parent != root or view_dir in seen:
            raise ValueError("뷰 경로가 중복되거나 결과 디렉토리 밖입니다.")
        seen.add(view_dir)
        path = view_dir / "pose.json"
        pose = cluster.read_json(path)
        if pose.get("accepted") is not True or pose.get("translation_unit") != "m":
            raise ValueError(f"승인되지 않았거나 단위가 다른 자세: {path}")
        t = cluster.validate_transform(pose["T_base_depth"])
        origins.append(t[:3, 3])
        directions.append(t[:3, 2] / np.linalg.norm(t[:3, 2]))  # Depth optical +Z
        sources.append({"path": str(path), "sha256": cluster.sha256(path)})
    if not origins:
        raise ValueError("SAM 추출 결과에 촬영 자세가 없습니다.")
    path = root / POINT_CLOUD_FILENAME
    if not path.is_file():
        raise FileNotFoundError(path)
    count_key = "merged_points" if POINT_CLOUD_FILENAME == "merged.ply" else "merged_raw_points"
    expected = manifest.get(count_key)
    if type(expected) is not int or expected < 1:
        raise ValueError(f"manifest의 {count_key}가 유효한 양의 정수가 아닙니다.")
    cloud = o3d.io.read_point_cloud(str(path))
    if len(cloud.points) != expected:
        raise ValueError("점군의 점 개수와 SAM 추출 manifest가 일치하지 않습니다.")
    return cloud, np.asarray(origins), np.asarray(directions), {
        "format": "sam_masked_depth",
        "cloud": {"path": str(path), "sha256": cluster.sha256(path)},
        "manifest": {"path": str(manifest_path), "sha256": cluster.sha256(manifest_path)},
        "poses": sources}


def run(capture_dir=None, *, output_root=None, manual_ids=None, check_only=False):
    import open3d as o3d

    started = time.perf_counter()
    validate_settings()
    cloud, origins, directions, sources = load_masked_capture(
        CAPTURE_DIR if capture_dir is None else capture_dir, o3d)
    target, target_info, warnings = None, {}, []
    if TARGET_POINT_BASE_MM is not None:
        target = np.asarray(TARGET_POINT_BASE_MM, float) / 1000.0
        target_info = {"method": "manual_base_point"}
    else:
        try:
            target, target_info = cluster.estimate_target(origins, directions, MAX_RAY_RMS_MM)
        except ValueError as error:
            warnings.append(str(error))
            target_info = {"method": "unavailable", "error": str(error)}
    if target is not None:
        print(f"선택 기준점 [base, mm]: {np.round(target * 1000, 2).tolist()}", flush=True)
    if check_only:
        print(f"SAM 점군/자세/Open3D 확인 완료: {len(cloud.points):,}점, {len(origins)}개 뷰. 저장하지 않았습니다.")
        for warning in warnings:
            print(f"주의: {warning}")
        return None

    voxels, voxel_labels, labels, records = cluster.cluster_cloud(
        cloud, o3d, voxel_mm=VOXEL_SIZE_MM, eps_mm=DBSCAN_EPS_MM,
        min_points=DBSCAN_MIN_POINTS, min_cluster_voxels=MIN_CLUSTER_VOXELS)
    manual_ids = SELECT_CLUSTER_IDS if manual_ids is None else manual_ids
    selected_ids = cluster.choose_clusters(records, target, manual_ids, MAX_TARGET_DISTANCE_MM)
    kept = np.isin(labels, selected_ids)
    if not selected_ids:
        warnings.append("부품 선택 실패. 집합 확인 후 SELECT_CLUSTER_IDS 또는 TARGET_POINT_BASE_MM을 지정하세요.")
    candidates = sorted((r for r in records if r["eligible"]),
                        key=lambda r: (r["target_distance_mm"] if r["target_distance_mm"] is not None else float("inf"), r["id"]))
    if manual_ids is None and len(candidates) >= 2 and target is not None:
        margin = candidates[1]["target_distance_mm"] - candidates[0]["target_distance_mm"]
        if margin < 50:
            warnings.append(f"가까운 두 집합의 중심 거리 차이가 {margin:.1f} mm뿐입니다. 선택 결과를 확인하세요.")
    warnings.append("붙어 있는 잡음은 같은 집합에 남고, 떨어진 부품 조각은 다른 집합으로 제외될 수 있습니다.")
    for r in candidates:
        distance = "N/A" if r["target_distance_mm"] is None else f"{r['target_distance_mm']:.1f} mm"
        marker = " [선택]" if r["id"] in selected_ids else ""
        print(f"집합 {r['id']}: 입력 {r['input_point_count']:,}점 / 중심 거리 {distance}{marker}", flush=True)

    output = Path(OUTPUT_ROOT if output_root is None else output_root).expanduser().resolve() / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    output.mkdir(parents=True, exist_ok=False)
    report = {"status": "running", "schema": "sam_masked_depth_dbscan_v1", "python": sys.executable,
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
            r["preview_color_rgb"] = list(cluster.label_color(r["id"]))
            colors[voxel_labels == r["id"]] = r["preview_color_rgb"]
        preview.colors = o3d.utility.Vector3dVector(colors)
        cluster.save_cloud(output / "clusters_colored.ply", preview, o3d)
        colors = np.full((len(voxel_labels), 3), 0.22)
        colors[np.isin(voxel_labels, selected_ids)] = [0.15, 1.0, 0.3]
        preview.colors = o3d.utility.Vector3dVector(colors)
        cluster.save_cloud(output / "selection_preview.ply", preview, o3d)
        (output / "clusters").mkdir()
        for r in records:
            if r["eligible"] or r["id"] in selected_ids:
                relative = f"clusters/cluster_{r['id']:03d}.ply"
                cluster.save_cloud(output / relative, cloud.select_by_index(np.flatnonzero(labels == r["id"]).tolist()), o3d)
                r["file"] = relative
        if selected_ids:
            cluster.save_cloud(output / "part.ply", cloud.select_by_index(np.flatnonzero(kept).tolist()), o3d)
        report["status"] = "complete" if selected_ids else "needs_selection"
    except BaseException as error:
        report.update(status="failed", error=f"{type(error).__name__}: {error}")
        raise
    finally:
        report["elapsed_sec"] = time.perf_counter() - started
        cluster.write_json(output / "summary.json", report)
    for warning in warnings:
        print(f"주의: {warning}")
    print(f"완료: {output}\n선택 집합: {selected_ids} / {int(kept.sum()):,}점", flush=True)
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture-dir", type=Path, help="SAM 마스크 Depth 추출 결과 폴더")
    parser.add_argument("--check", action="store_true", help="입력/자세/의존성만 확인. 저장 없음")
    parser.add_argument("--cluster-ids", type=int, nargs="+", help="자동 선택 대신 집합 번호 지정")
    parser.add_argument("--visualize", action="store_true", help="추출 후 시각화")
    parser.add_argument("--view", type=Path, help="기존 결과 폴더만 표시. 추출하지 않음")
    parser.add_argument("--layer", choices=["selection", "clusters", "part"], default="selection")
    args = parser.parse_args()
    try:
        if args.view is not None:
            cluster.view_result(args.view, args.layer)
        else:
            output = run(args.capture_dir, manual_ids=args.cluster_ids, check_only=args.check)
            if output is not None and (args.visualize or VISUALIZE_AFTER_RUN):
                cluster.view_result(output, args.layer)
    except Exception as error:
        print(f"오류: {error}", file=sys.stderr)
        raise SystemExit(1) from error


if __name__ == "__main__":
    main()
