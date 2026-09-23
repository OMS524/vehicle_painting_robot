#!/usr/bin/env python3
"""저장된 원본 RGB 마스크 -> 부품 Depth/점군/다중 뷰 통합 점군.

vehicle_painting_robot_scan 환경에서 실행. 카메라/로봇 연결은 하지 않는다.
MASK_PATHS에는 첫 번째 코드가 생성한 mask.png(또는 candidates/mask_XXX.png)를 넣는다.
각 마스크 옆의 동명 .json을 통해 원본 Depth/camera.json/pose.json을 찾는다.
저장값으로 SDK 프레임을 복원하여 PointCloudFilter로 XYZ, AlignFilter로 RGB/마스크 대응을 구한다.
통합은 저장된 T_base_depth 변환으로 수행하며 ICP나 결손 보간은 하지 않는다.
"""
from __future__ import annotations

import argparse
from datetime import datetime
import hashlib
from importlib import metadata
import json
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent

# ======================== 사용자가 수정할 설정 ========================
MASK_PATHS: list[str | Path] = [
    # "/.../sam_test/log/sam3_masks/실행일시/01_view_01/mask.png",
    # "/.../sam_test/log/sam3_masks/실행일시/02_view_02/mask.png",
]
OUTPUT_ROOT = HERE.parent / "log" / "masked_depth"
VOXEL_SIZE_MM = 2.0  # 통합본 다운샘플링 간격. 0이면 하지 않음. 원본 통합본은 항상 별도 저장.
# =====================================================================


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def write_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def validate_calibration(calibration: dict, shape: tuple[int, int]) -> None:
    k = calibration["intrinsic"]
    values = np.array([k[key] for key in ("fx", "fy", "cx", "cy", "width", "height")], dtype=float)
    if not np.isfinite(values).all() or min(k["fx"], k["fy"], k["width"], k["height"]) <= 0:
        raise ValueError("카메라 내부 파라미터가 올바르지 않습니다.")
    if (k["height"], k["width"]) != shape:
        raise ValueError("영상과 보정 파라미터의 해상도가 다릅니다.")
    model = calibration["distortion_model"].split(".")[-1]
    if model not in {"NONE", "BROWN_CONRADY", "BROWN_CONRADY_K6"}:
        raise ValueError(f"이 스크립트가 지원하지 않는 왜곡 모델: {model}")
    coefficients = np.array([calibration["distortion"][key] for key in
                             ("k1", "k2", "k3", "k4", "k5", "k6", "p1", "p2")], dtype=float)
    if not np.isfinite(coefficients).all():
        raise ValueError("왜곡 계수에 비정상 값이 있습니다.")


def validate_transform(value, label: str) -> np.ndarray:
    transform = np.asarray(value, dtype=float)
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError(f"{label}: 유효한 4x4 행렬이 필요합니다.")
    rotation = transform[:3, :3]
    if (not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-6)
            or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-3)
            or not np.isclose(np.linalg.det(rotation), 1, atol=1e-3)):
        raise ValueError(f"{label}: 강체 변환 행렬이 아닙니다.")
    return transform


def load_view(mask_path: str | Path) -> dict:
    from PIL import Image

    path = Path(mask_path).expanduser().resolve()
    info = read_json(path.with_suffix(".json"))
    if info.get("schema") != "sam3_rgb_mask_v1" or info.get("coordinate_space") != "native_color_pixels":
        raise ValueError(f"원본 RGB 좌표의 SAM 3 마스크 메타데이터가 필요합니다: {path}")
    sources = {}
    for key in ("rgb", "depth", "camera", "pose"):
        record = info.get("sources", {}).get(key)
        if not record:
            raise ValueError(f"{key} 원본 정보가 없습니다. RGB와 같은 폴더에 Depth/camera/pose를 두고 마스크를 생성하세요: {path}")
        source = (path.parent / record["path"]).resolve()
        if sha256(source) != record["sha256"]:
            raise ValueError(f"마스크 생성 이후 원본 파일이 변경되었습니다: {source}")
        sources[key] = source
    parents = {p.parent for p in sources.values()}
    if len(parents) != 1:
        raise ValueError("RGB/Depth/camera/pose는 같은 촬영 뷰에 속해야 합니다.")
    with Image.open(path) as image:
        mask = np.asarray(image)
    if mask.ndim != 2 or not np.isin(mask, [0, 1, 255]).all():
        raise ValueError(f"단일 채널 이진 마스크가 필요합니다 (overlay 사용 금지): {path}")
    mask = mask > 0
    if not mask.any():
        raise ValueError(f"마스크가 비어 있습니다. SAM 3 결과를 먼저 확인하세요: {path}")
    with Image.open(sources["rgb"]) as image:
        rgb = np.asarray(image.convert("RGB"))
    if mask.shape != rgb.shape[:2] or list(reversed(mask.shape)) != info["image_size"]:
        raise ValueError("마스크와 원본 RGB의 해상도가 다릅니다. 마스크를 리사이즈하지 마세요.")
    depth = np.load(sources["depth"], allow_pickle=False)
    camera, pose = read_json(sources["camera"]), read_json(sources["pose"])
    if depth.dtype != np.uint16 or depth.shape != (camera["height"], camera["width"]):
        raise ValueError("depth_raw.npy는 camera.json과 크기가 같은 uint16 Y16이어야 합니다.")
    if not pose.get("accepted") or pose.get("translation_unit") != "m":
        raise ValueError("촬영이 accepted이고 pose.json의 translation_unit이 m이어야 합니다.")
    transform = validate_transform(pose["T_base_depth"], "T_base_depth")
    scale = float(camera["depth_scale_mm_per_unit"])
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError("Depth scale이 유효하지 않습니다.")
    for calibration, shape in ((camera, depth.shape), (camera["color"], mask.shape)):
        validate_calibration(calibration, shape)
    d2c = camera["color"]["depth_to_color"]
    extrinsic = np.eye(4)
    extrinsic[:3, :3] = np.asarray(d2c["rotation"]).reshape(3, 3)
    extrinsic[:3, 3] = np.asarray(d2c["translation_mm"]).reshape(3)
    # SDK가 반환한 보정 행렬은 수치상 완전한 정규직교 행렬이 아닐 수 있다.
    # 로봇 pose와 달리 재정규화/강체 오차 제한을 적용하지 않고 저장값을 보존한다.
    if not np.isfinite(extrinsic).all() or np.linalg.det(extrinsic[:3, :3]) <= 0:
        raise ValueError("Depth→RGB 보정 행렬이 유효하지 않습니다.")
    return {"mask_path": path, "mask": mask, "rgb": rgb, "depth": depth,
            "camera": camera, "pose": pose, "transform": transform, "sources": sources,
            "view_dir": sources["depth"].parent, "capture_dir": sources["depth"].parent.parent}


def sdk_video_frame(data: np.ndarray, calibration: dict, sdk, *, depth: bool = False):
    """파일의 영상/보정값으로 SDK 프레임 복원. 장치나 Pipeline은 생성하지 않는다."""
    frame = sdk.Frame.create_video_frame(
        sdk.OBFrameType.DEPTH_FRAME if depth else sdk.OBFrameType.COLOR_FRAME,
        sdk.OBFormat.Y16 if depth else sdk.OBFormat.RGB, data.shape[1], data.shape[0])
    profile = frame.get_stream_profile().as_video_stream_profile()
    intrinsic = sdk.OBCameraIntrinsic()
    for key in ("fx", "fy", "cx", "cy", "width", "height"):
        value = calibration["intrinsic"][key]
        setattr(intrinsic, key, int(value) if key in {"width", "height"} else float(value))
    distortion = sdk.OBCameraDistortion()
    model = calibration["distortion_model"].split(".")[-1]
    for key in ("k1", "k2", "k3", "k4", "k5", "k6", "p1", "p2"):
        value = 0.0 if model == "NONE" else calibration["distortion"][key]
        setattr(distortion, key, float(value))
    # AlignFilter가 지원하는 무왜곡 표현: 계수 0인 Brown. 실제 K6 보정은 원래 enum 유지.
    distortion.model = getattr(sdk.OBCameraDistortionModel, "BROWN_CONRADY" if model == "NONE" else model)
    profile.set_intrinsic(intrinsic)
    profile.set_distortion(distortion)
    frame.set_stream_profile(profile)
    frame.update_data(np.ascontiguousarray(data).tobytes())
    if depth:
        frame = frame.as_depth_frame()
        frame.set_value_scale(float(calibration["depth_scale_mm_per_unit"]))
    return frame


def sdk_depth_points_m(depth_frame, sdk) -> np.ndarray:
    """three_view_scan.py와 동일한 PointCloudFilter/좌표계/단위 변환. 픽셀 순서 유지."""
    frames = sdk.Frame.create_frame_set()
    frames.push_frame(depth_frame)
    point_filter = sdk.PointCloudFilter()
    point_filter.set_create_point_format(sdk.OBFormat.POINT)
    point_filter.set_coordinate_system(sdk.OBCoordinateSystemType.RIGHT_HAND)
    result = point_filter.process(frames)
    if result is None:
        raise RuntimeError("SDK Depth → point cloud 변환 실패")
    points = result.as_points_frame()
    scale = float(points.get_position_value_scale())
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError("잘못된 SDK point position scale")
    xyz = np.frombuffer(points.get_data(), dtype=np.float32).reshape(-1, 3)
    if len(xyz) != depth_frame.get_width() * depth_frame.get_height():
        raise ValueError("SDK 점군과 Depth 픽셀 개수가 다릅니다.")
    return xyz.astype(np.float64) * (scale / 1000.0)


def sdk_align_color_to_depth(depth_frame, color_frame, align, sdk) -> np.ndarray:
    """기존 스캔과 같은 C2D 정렬. 입력 Color는 원본 RGB 또는 마스크 전달용 RGB."""
    frames = sdk.Frame.create_frame_set()
    frames.push_frame(depth_frame)
    frames.push_frame(color_frame)
    result = align.process(frames)
    color = result.as_frame_set().get_color_frame() if result is not None else None
    if color is None:
        raise RuntimeError("SDK RGB/마스크 → Depth 정렬 실패")
    if color.get_format() != sdk.OBFormat.RGB or (color.get_height(), color.get_width()) != (
            depth_frame.get_height(), depth_frame.get_width()):
        raise ValueError("SDK 정렬 결과가 native Depth 크기의 RGB가 아닙니다.")
    return np.frombuffer(color.get_data(), dtype=np.uint8).reshape(color.get_height(), color.get_width(), 3).copy()


def extract_view(view: dict, sdk) -> dict:
    camera = view["camera"]
    depth_frame = sdk_video_frame(view["depth"], camera, sdk, depth=True)
    xyz = sdk_depth_points_m(depth_frame, sdk)
    valid = np.isfinite(xyz).all(axis=1) & (xyz[:, 2] > 0) & (view["depth"].ravel() > 0)
    if not valid.any():
        raise ValueError("유효한 Depth 점이 없습니다.")

    color_frame = sdk_video_frame(view["rgb"], camera["color"], sdk)
    extrinsic = sdk.OBExtrinsic()
    extrinsic.rot = np.asarray(camera["color"]["depth_to_color"]["rotation"], dtype=np.float32).reshape(3, 3)
    extrinsic.transform = np.asarray(camera["color"]["depth_to_color"]["translation_mm"], dtype=np.float32)
    depth_frame.get_stream_profile().bind_extrinsic_to(color_frame.get_stream_profile(), extrinsic)
    align = sdk.AlignFilter(sdk.OBStreamType.DEPTH_STREAM)
    align.set_config_value("TargetDistortion", 1)  # three_view_scan.py와 동일: native Depth 픽셀 유지
    aligned_rgb = sdk_align_color_to_depth(depth_frame, color_frame, align, sdk)

    # RGB만 받는 C2D 필터에 마스크를 전달한다. R=부품 여부, G=RGB 유효 영역(255).
    # 정렬 불가 위치는 0이므로 실제 검은 RGB 픽셀도 배경/미대응과 혼동하지 않는다.
    mask_rgb = np.zeros_like(view["rgb"])
    mask_rgb[..., 0] = view["mask"].astype(np.uint8) * 255
    mask_rgb[..., 1] = 255
    color_frame.update_data(mask_rgb.tobytes())
    aligned_mask = sdk_align_color_to_depth(depth_frame, color_frame, align, sdk)
    if not np.isin(aligned_mask[..., :2], [0, 255]).all():
        raise ValueError("SDK 정렬이 이진 마스크 값을 변경했습니다. SDK의 정렬 방식을 확인하세요.")
    mapped = valid & (aligned_mask[..., 1].ravel() == 255)
    selected = np.flatnonzero(mapped & (aligned_mask[..., 0].ravel() == 255))
    if not len(selected):
        raise ValueError("RGB 마스크와 대응되는 유효 Depth가 없습니다.")
    keep = np.zeros(view["depth"].shape, dtype=bool)
    keep.ravel()[selected] = True
    masked_depth = np.where(keep, view["depth"], 0).astype(np.uint16)
    points = xyz[selected]
    transform = view["transform"]
    base_points = points @ transform[:3, :3].T + transform[:3, 3]
    colors = aligned_rgb.reshape(-1, 3)[selected].astype(float) / 255.0
    return {"depth": masked_depth, "keep": keep, "points": points, "base_points": base_points,
            "colors": colors, "input_valid_depth": int(np.count_nonzero(view["depth"])),
            "unprojected_points": int(valid.sum()), "rgb_mapped_points": int(mapped.sum()), "selected_points": len(selected)}


def point_cloud(points, colors, o3d):
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    cloud.colors = o3d.utility.Vector3dVector(colors)
    return cloud


def save_cloud(path: Path, cloud, o3d) -> None:
    if not o3d.io.write_point_cloud(str(path), cloud):
        raise IOError(f"점군 저장 실패: {path}")


def run(paths: list[str | Path] | None = None, *, check_only: bool = False) -> Path | None:
    from PIL import Image
    import open3d as o3d
    import pyorbbecsdk as sdk

    selected_paths = MASK_PATHS if paths is None else paths
    if not selected_paths:
        raise ValueError("상단 MASK_PATHS에 첫 번째 코드에서 생성한 mask.png 경로를 넣으세요.")
    if not np.isfinite(VOXEL_SIZE_MM) or VOXEL_SIZE_MM < 0:
        raise ValueError("VOXEL_SIZE_MM은 0 이상의 유한한 값이어야 합니다.")
    views = [load_view(path) for path in selected_paths]
    if len({view["view_dir"] for view in views}) != len(views):
        raise ValueError("같은 촬영 뷰의 마스크가 중복되었습니다. 후보 여러 개는 첫 코드에서 all로 합치세요.")
    if len({view["capture_dir"] for view in views}) != 1:
        raise ValueError("서로 다른 촬영 실행의 뷰는 통합하지 않습니다. 같은 촬영 폴더의 마스크를 사용하세요.")
    if len({view["camera"]["frame"] for view in views}) != 1:
        raise ValueError("Depth 기준 프레임이 서로 다릅니다.")
    print(f"Python: {sys.executable}\n입력: {len(views)}개 뷰", flush=True)
    if check_only:
        print("마스크/원본 해시/보정/자세/API 확인 완료. 추출과 저장은 실행하지 않았습니다.")
        return None
    output = Path(OUTPUT_ROOT).expanduser().resolve() / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    output.mkdir(parents=True, exist_ok=False)
    manifest = {"status": "running", "python": sys.executable,
                "sdk": metadata.version("pyorbbecsdk2"), "reference_frame": "base_link",
                "point_cloud_unit": "m", "method": "saved_T_base_depth_no_ICP",
                "depth_to_xyz": "Orbbec PointCloudFilter, RIGHT_HAND",
                "mask_to_depth": "Orbbec AlignFilter, DEPTH_STREAM, TargetDistortion=1",
                "voxel_size_mm": VOXEL_SIZE_MM,
                "notes": "원본 RGB 마스크를 SDK로 Depth에 대응. 결손 보간/ICP/가림 보정 없음.", "views": []}
    write_json(output / "manifest.json", manifest)
    merged = o3d.geometry.PointCloud()
    try:
        for index, view in enumerate(views, 1):
            print(f"[{index}/{len(views)}] {view['view_dir'].name}: SDK 점군/마스크 정렬 중", flush=True)
            result = extract_view(view, sdk)
            directory = output / f"{index:02d}_{view['view_dir'].name}"
            directory.mkdir()
            np.save(directory / "depth_masked.npy", result["depth"], allow_pickle=False)
            Image.fromarray(result["depth"]).save(directory / "depth_masked.png")
            Image.fromarray(result["keep"].astype(np.uint8) * 255).save(directory / "depth_mask.png")
            camera_cloud = point_cloud(result["points"], result["colors"], o3d)
            base_cloud = point_cloud(result["base_points"], result["colors"], o3d)
            save_cloud(directory / "part_camera.ply", camera_cloud, o3d)
            save_cloud(directory / "part_base.ply", base_cloud, o3d)
            merged += base_cloud
            write_json(directory / "camera.json", view["camera"])
            write_json(directory / "pose.json", view["pose"])
            entry = {"mask": str(view["mask_path"]), "mask_sha256": sha256(view["mask_path"]),
                     "source_view": str(view["view_dir"]), "output": str(directory),
                     "depth_storage": "uint16, original raw units; excluded pixels=0",
                     "depth_scale_mm_per_unit": view["camera"]["depth_scale_mm_per_unit"],
                     **{key: result[key] for key in ("input_valid_depth", "unprojected_points", "rgb_mapped_points", "selected_points")}}
            write_json(directory / "extraction.json", entry)
            manifest["views"].append(entry)
            write_json(output / "manifest.json", manifest)
            print(f"  부품 점 {result['selected_points']:,}개 저장", flush=True)
        save_cloud(output / "merged_raw.ply", merged, o3d)
        downsampled = merged.voxel_down_sample(VOXEL_SIZE_MM / 1000.0) if VOXEL_SIZE_MM else merged
        save_cloud(output / "merged.ply", downsampled, o3d)
        manifest.update(status="complete", merged_raw_points=len(merged.points), merged_points=len(downsampled.points))
    except Exception as error:
        manifest.update(status="failed", error=str(error))
        raise
    finally:
        write_json(output / "manifest.json", manifest)
    print(f"완료: {output}\n통합 점군: {output / 'merged.ply'}")
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="입력/보정/의존성만 검사 (저장하지 않음)")
    args = parser.parse_args()
    try:
        run(check_only=args.check)
    except Exception as error:
        print(f"오류: {error}", file=sys.stderr)
        raise SystemExit(1) from error


if __name__ == "__main__":
    main()
