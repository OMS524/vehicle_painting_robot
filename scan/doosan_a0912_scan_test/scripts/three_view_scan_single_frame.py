#!/usr/bin/env python3
"""A0912 단일 프레임 촬영: 워밍업 이후 Depth 한 장과 동기 RGB를 저장.

다중 프레임 통합 전 버전(deec341)을 별도 파일로 복원한 비교용 스크립트.
기본 설정은 three_view_scan.yaml을 공유한다. warmup_frames만큼 버린 뒤
한 장만 사용하며 capture_frames/min_valid_frames는 실행 설정에서 1로 고정한다.

기본 실행/--check: 파일, TF, 라이브러리만 검사 (하드웨어 연결 없음).
--execute: 설정 확인 및 터미널 동의 후 실제 로봇 이동/카메라 촬영.
--view FILE.ply: 저장한 결과만 보기.
"""
import argparse
import colorsys
from contextlib import AbstractContextManager
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
import os
from pathlib import Path
import queue
import signal
import subprocess
import sys
import threading
import time
import traceback
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from visualize_xacro import link_transforms

HERE = Path(__file__).resolve().parent
DEFAULT_CONFIG = HERE / "three_view_scan.yaml"
JOINT_NAMES = tuple(f"joint_{i}" for i in range(1, 7))
BASE_FRAME = "base_link"
DEPTH_FRAME = "camera_depth_optical_frame"
VIEW_COLORS = {"front": [1.0, 0.35, 0.25], "left": [0.25, 0.85, 0.4], "right": [0.25, 0.5, 1.0]}


def view_colors(views):
    """기존 세 이름의 색은 유지하고, 다른 이름에는 목록 순서로 구분색을 배정."""
    colors = {}
    custom_index = 0
    for view in views:
        name = view["name"]
        if name in VIEW_COLORS:
            colors[name] = VIEW_COLORS[name]
        else:
            hue = (0.12 + custom_index * 0.618033988749895) % 1.0
            colors[name] = list(colorsys.hsv_to_rgb(hue, 0.73, 1.0))
            custom_index += 1
    return colors


def save_json(path, data):
    """JSON 한 파일은 임시 파일을 완성한 뒤 교체. NaN 저장 금지."""
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False) + "\n", encoding="utf-8")
    temporary.replace(path)


def finite_vector(values, count, label):
    if not isinstance(values, list) or len(values) != count or any(
            isinstance(x, bool) or not isinstance(x, (int, float)) for x in values):
        raise ValueError(f"{label}: 숫자 {count}개를 입력하세요.")
    result = np.asarray(values, dtype=float)
    if not np.isfinite(result).all():
        raise ValueError(f"{label}: NaN/inf는 사용할 수 없습니다.")
    return result


def load_config(path):
    path = Path(path).expanduser().resolve()
    cfg = yaml.safe_load(path.read_text(encoding="utf-8"))
    camera = cfg["camera"]
    camera.setdefault("capture_color", False)
    camera.setdefault("color_width", 0)
    camera.setdefault("color_height", 0)
    camera.setdefault("color_max_time_delta_ms", 50.0)
    # YAML 원본은 변경하지 않는다. 저장되는 settings.json에는 실제 동작을 기록한다.
    camera["capture_frames"] = 1
    camera["min_valid_frames"] = 1
    if type(camera["capture_color"]) is not bool:
        raise ValueError("camera.capture_color는 true/false여야 합니다.")
    for name in ("control_python", "control_wrapper", "control_library", "xacro", "output_dir"):
        value = Path(cfg[name]).expanduser()
        cfg[name] = str((path.parent / value).resolve() if not value.is_absolute() else value.resolve())
    views = cfg.get("views")
    if not isinstance(views, list) or not views:
        raise ValueError("views는 촬영 시점이 1개 이상인 목록이어야 합니다.")
    view_names = set()
    for index, view in enumerate(views, start=1):
        if not isinstance(view, dict):
            raise ValueError(f"views의 {index}번째 항목에 name과 joint_deg를 지정하세요.")
        name = view.get("name")
        # 이름은 결과 폴더명으로 사용한다. 경로와 세션 파일 이름 충돌을 차단한다.
        if (not isinstance(name, str) or not name or len(name.encode("utf-8")) > 255
                or not all(char.isalnum() or char in "_-" for char in name)):
            raise ValueError(f"views의 {index}번째 name은 문자·숫자·밑줄(_)·하이픈(-)으로 "
                             "된 255바이트 이내의 비어 있지 않은 이름이어야 합니다.")
        if name in view_names:
            raise ValueError(f"views의 name이 중복됩니다: {name}. 시점마다 고유한 이름을 지정하세요.")
        view_names.add(name)
        if "joint_deg" not in view:
            raise ValueError(f"views.{name}.joint_deg에 J1~J6 또는 null을 지정하세요.")
        if view["joint_deg"] is not None:
            finite_vector(view["joint_deg"], 6, f"views.{name}.joint_deg")
    for group, names in {
        "robot": ("connect_timeout_sec", "move_timeout_sec", "max_velocity_deg_s", "acceleration_deg_s2",
                  "target_tolerance_deg", "command_time_sec", "max_move_delta_deg", "settle_sec",
                  "settle_timeout_sec", "stationary_velocity_deg_s", "capture_drift_deg"),
        "camera": ("warmup_frames", "capture_timeout_sec", "min_depth_mm", "max_depth_mm", "min_points",
                   "color_max_time_delta_ms"),
        "reconstruction": ("voxel_size_mm",),
    }.items():
        for name in names:
            value = cfg[group][name]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value) or value <= 0:
                raise ValueError(f"{group}.{name}은 양의 유한한 숫자여야 합니다.")
    for group, names in {"robot": ("port", "rt_port", "connect_timeout_sec"),
                         "camera": ("width", "height", "fps", "warmup_frames", "min_points",
                                    "color_width", "color_height")}.items():
        for name in names:
            value = cfg[group][name]
            if type(value) is not int or value < 0:
                raise ValueError(f"{group}.{name}은 0 이상의 정수여야 합니다.")
    if not all(0 < cfg["robot"][key] < 65536 for key in ("port", "rt_port")):
        raise ValueError("로봇 포트 범위는 1~65535입니다.")
    if cfg["robot"]["settle_timeout_sec"] <= cfg["robot"]["settle_sec"]:
        raise ValueError("settle_timeout_sec은 settle_sec보다 커야 합니다.")
    if cfg["camera"]["min_depth_mm"] >= cfg["camera"]["max_depth_mm"]:
        raise ValueError("min_depth_mm은 max_depth_mm보다 작아야 합니다.")
    low, high = (cfg["reconstruction"][k] for k in ("roi_min_mm", "roi_max_mm"))
    if (low is None) != (high is None):
        raise ValueError("ROI의 최소/최대 좌표를 함께 입력하세요.")
    if low is not None and np.any(finite_vector(low, 3, "roi_min_mm") >= finite_vector(high, 3, "roi_max_mm")):
        raise ValueError("ROI 최소 좌표는 최대 좌표보다 작아야 합니다.")
    return cfg


class ScanModel:
    def __init__(self, path):
        import xacro

        self.xml = xacro.process_file(str(path)).toxml()
        self.robot = ET.fromstring(self.xml)
        movable = {j.get("name") for j in self.robot.findall("joint") if j.get("type") != "fixed"}
        if movable != set(JOINT_NAMES):
            raise ValueError(f"A0912의 joint_1~6만 지원합니다: {movable}")
        self.transforms([0.0] * 6)

    def transforms(self, joints):
        values = finite_vector(joints, 6, "joint_deg")
        _, _, frames = link_transforms(self.robot, dict(zip(JOINT_NAMES, values)))
        inv_base = np.linalg.inv(frames[BASE_FRAME])
        return {name: inv_base @ frames[name] for name in ("link_6", "bracket_link", DEPTH_FRAME)}


def worker_environment(python):
    # 자식 프로세스에만 적용. SDK/.so/Conda 전역 설정은 변경하지 않는다.
    env = os.environ.copy()
    for name in ("PYTHONHOME", "PYTHONPATH", "LD_LIBRARY_PATH", "LD_PRELOAD"):
        env.pop(name, None)
    prefix = str(Path(python).parent.parent)
    env.update(CONDA_PREFIX=prefix, PYTHONNOUSERSITE="1", PYTHONDONTWRITEBYTECODE="1")
    env["PATH"] = str(Path(python).parent) + os.pathsep + env.get("PATH", "")
    return env


def worker_command(cfg, mode):
    return [cfg["control_python"], "-u", str(HERE / "robot_scan_worker.py"),
            "--wrapper", cfg["control_wrapper"], "--library", cfg["control_library"], mode]


def preflight(cfg, model, execute=False):
    for name in ("control_python", "control_wrapper", "control_library"):
        if not Path(cfg[name]).is_file():
            raise FileNotFoundError(f"{name}: {cfg[name]}")
    missing = []
    for view in cfg["views"]:
        if view["joint_deg"] is None:
            missing.append(view["name"])
        else:
            model.transforms(view["joint_deg"])  # 조인트 제한과 TF 연결까지 검사.
    if execute and missing:
        raise ValueError(f"촬영 자세를 먼저 입력하세요: {', '.join(missing)} (연결/이동하지 않았습니다)")
    if execute and cfg["mounting_tf_confirmed"] is not True:
        raise ValueError("실제 장착과 Xacro TF를 확인한 후 mounting_tf_confirmed: true로 설정하세요.")
    checked = subprocess.run(worker_command(cfg, "--check"), env=worker_environment(cfg["control_python"]),
                             capture_output=True, text=True, timeout=30)
    if checked.returncode:
        raise RuntimeError("제어 환경 라이브러리 검사 실패:\n" + checked.stdout + checked.stderr)
    import open3d  # noqa: F401 -- 실행 전에 스캔 환경 의존성 로딩 확인
    import pyorbbecsdk  # noqa: F401 -- Context/Pipeline 생성은 하지 않음
    if cfg["camera"]["capture_color"]:
        import cv2  # noqa: F401 -- RGB PNG 저장용

    print("파일·TF·분리된 제어 라이브러리 검사 통과. 하드웨어 연결 없음.")
    print(f"촬영 시점 {len(cfg['views'])}개 (목록 순서): " + " → ".join(v["name"] for v in cfg["views"]))
    print(f"단일 프레임 모드: 시점마다 초기 {cfg['camera']['warmup_frames']}프레임을 버린 뒤 "
          "다음 유효 Depth 1프레임을 저장합니다. 시간 방향 통합 없음.")
    if missing:
        print("미입력 목표 자세:", ", ".join(missing))
    print("Xacro 기준 flange → depth optical (translation: m):")
    frames = model.transforms([0.0] * 6)
    print(np.linalg.inv(frames["link_6"]) @ frames[DEPTH_FRAME])


class RobotClient(AbstractContextManager):
    def __init__(self, cfg, output):
        self.cfg, self.sequence = cfg, 0
        self.responses = queue.Queue()
        self.log = (output / "robot_worker.log").open("w", encoding="utf-8")
        try:
            self.process = subprocess.Popen(worker_command(cfg, "--allow-motion"),
                env=worker_environment(cfg["control_python"]), stdin=subprocess.PIPE,
                stdout=subprocess.PIPE, stderr=self.log, text=True, bufsize=1,
                start_new_session=True)
        except BaseException:
            self.log.close()
            raise

        def receive():
            try:
                for line in self.process.stdout:
                    self.responses.put(line)
            finally:
                self.responses.put(None)
        self.reader = threading.Thread(target=receive, daemon=True)
        self.reader.start()

    def call(self, op, timeout=10, **data):
        self.sequence += 1
        self.process.stdin.write(json.dumps({"id": self.sequence, "op": op, **data}, allow_nan=False) + "\n")
        self.process.stdin.flush()
        try:
            line = self.responses.get(timeout=timeout)
        except queue.Empty as exc:
            raise TimeoutError(f"로봇 worker 응답 시간 초과: {op}. robot_worker.log 확인") from exc
        if line is None:
            raise RuntimeError("로봇 worker 종료. robot_worker.log 확인")
        reply = json.loads(line)
        if reply.get("id") != self.sequence:
            raise RuntimeError("로봇 worker 응답 순서 불일치")
        if "error" in reply:
            raise RuntimeError(reply["error"])
        return reply["result"]

    def move(self, joints):
        return self.call("move", joint_deg=joints,
                         timeout=self.cfg["robot"]["move_timeout_sec"] + self.cfg["robot"]["settle_timeout_sec"] + 15)

    def snapshot(self):
        return self.call("snapshot")

    def __exit__(self, exc_type, exc, tb):
        try:
            if self.process.poll() is None:
                if exc_type is None:
                    self.call("close", timeout=15)
                else:
                    # 이동 중 예외/사용자 중단도 worker finally의 shutdown으로 전달.
                    self.process.send_signal(signal.SIGTERM)
                self.process.wait(timeout=20)
        finally:
            try:
                if self.process.poll() is None:
                    self.process.send_signal(signal.SIGTERM)
                    try:
                        self.process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        print("경고: 제어 프로세스 종료 응답 없음. 현장에서 로봇 정지를 확인하세요.", file=sys.stderr)
                        self.process.kill()
                        self.process.wait(timeout=5)
            finally:
                self.process.stdin.close()
                self.reader.join(timeout=1)
                self.process.stdout.close()
                self.log.close()
        return False


def points_in_meters(points_frame):
    """SDK PointsFrame 원시 좌표 × frame scale = mm. 깊이 scale 중복 적용 금지."""
    scale = float(points_frame.get_position_value_scale())
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError("잘못된 SDK point position scale")
    xyz = np.frombuffer(points_frame.get_data(), dtype=np.float32).reshape(-1, 3)
    return xyz.astype(np.float64) * (scale / 1000.0)


@dataclass
class CapturedView:
    depth_raw: np.ndarray
    xyz: np.ndarray
    metadata: dict
    color_rgb: np.ndarray | None = None
    color_aligned_rgb: np.ndarray | None = None


def profile_calibration(profile):
    intrinsic, distortion = profile.get_intrinsic(), profile.get_distortion()
    return {"intrinsic": {k: float(getattr(intrinsic, k)) for k in ("fx", "fy", "cx", "cy", "width", "height")},
            "distortion": {k: float(getattr(distortion, k)) for k in ("k1", "k2", "k3", "k4", "k5", "k6", "p1", "p2")},
            "distortion_model": str(distortion.model)}


def color_to_rgb_frame(sdk, color):
    if color.get_format() == sdk.OBFormat.RGB:
        return color
    conversions = {sdk.OBFormat.MJPG: sdk.OBConvertFormat.MJPG_TO_RGB888,
                   sdk.OBFormat.YUYV: sdk.OBConvertFormat.YUYV_TO_RGB888,
                   sdk.OBFormat.NV12: sdk.OBConvertFormat.NV12_TO_RGB888}
    if color.get_format() not in conversions:
        raise ValueError(f"지원하지 않는 컬러 포맷: {color.get_format()}")
    converter = sdk.FormatConvertFilter()
    converter.set_format_convert_format(conversions[color.get_format()])
    converted = converter.process(color)
    if converted is None:
        raise RuntimeError("컬러 프레임 RGB 변환 실패")
    return converted.as_color_frame()


def rgb_image(sdk, frame):
    if frame.get_format() != sdk.OBFormat.RGB:
        raise ValueError("RGB888 프레임이 필요합니다.")
    return np.frombuffer(frame.get_data(), dtype=np.uint8).reshape(frame.get_height(), frame.get_width(), 3).copy()


def align_color_to_depth(sdk, depth, color):
    """RGB를 native Depth 픽셀에 매핑한다. XYZ는 원래 Depth 광학 좌표계에 유지한다."""
    rgb_frame = color_to_rgb_frame(sdk, color)
    color_profile = rgb_frame.get_stream_profile().as_video_stream_profile()
    model = color_profile.get_distortion().model
    supported = (sdk.OBCameraDistortionModel.BROWN_CONRADY,
                 sdk.OBCameraDistortionModel.BROWN_CONRADY_K6,
                 sdk.OBCameraDistortionModel.KANNALA_BRANDT4)
    # SDK C2D는 원래 Depth 픽셀과 왜곡이 있는 Color 픽셀 사이를 매핑해야 한다.
    if model not in supported:
        raise ValueError(f"RGB 정렬에서 지원하지 않는 컬러 왜곡 모델: {model}")
    frames = sdk.Frame.create_frame_set()
    frames.push_frame(depth)
    frames.push_frame(rgb_frame)
    align = sdk.AlignFilter(sdk.OBStreamType.DEPTH_STREAM)
    align.set_config_value("TargetDistortion", 1)
    aligned = align.process(frames)
    aligned_color = aligned.as_frame_set().get_color_frame() if aligned is not None else None
    if aligned_color is None:
        raise RuntimeError("RGB → Depth 정렬 실패")
    aligned_rgb = rgb_image(sdk, aligned_color)
    if aligned_rgb.shape[:2] != (depth.get_height(), depth.get_width()):
        raise ValueError("RGB 정렬 결과와 native Depth 해상도가 다릅니다.")
    return rgb_image(sdk, rgb_frame), aligned_rgb


class DepthCamera:
    def __init__(self, settings):
        import pyorbbecsdk as sdk

        self.sdk, self.settings = sdk, settings
        self.context = sdk.Context()
        devices = self.context.query_devices()
        if settings["serial_number"]:
            self.device = devices.get_device_by_serial_number(settings["serial_number"])
        elif devices.get_count() == 1:
            self.device = devices.get_device_by_index(0)
        else:
            raise RuntimeError(f"카메라 {devices.get_count()}대 발견. 한 대 연결하거나 serial_number를 지정하세요.")
        self.pipeline = sdk.Pipeline(self.device)
        profiles = self.pipeline.get_stream_profile_list(sdk.OBSensorType.DEPTH_SENSOR)
        self.profile = profiles.get_video_stream_profile(
            settings["width"], settings["height"], sdk.OBFormat.Y16, settings["fps"])
        self.config = sdk.Config()
        self.config.enable_stream(self.profile)
        self.color_profile = None
        if settings.get("capture_color", False):
            color_profiles = self.pipeline.get_stream_profile_list(sdk.OBSensorType.COLOR_SENSOR)
            # Depth와 FPS를 맞추고, USB 전송량이 작은 MJPG를 우선 선택한다.
            errors = []
            for fmt in (sdk.OBFormat.MJPG, sdk.OBFormat.RGB, sdk.OBFormat.YUYV, sdk.OBFormat.NV12):
                try:
                    self.color_profile = color_profiles.get_video_stream_profile(
                        settings["color_width"], settings["color_height"], fmt, self.profile.get_fps())
                    break
                except (sdk.OBError, RuntimeError) as exc:
                    errors.append(str(exc))
            if self.color_profile is None:
                raise RuntimeError("Depth와 FPS가 같은 컬러 프로파일이 없습니다: " + "; ".join(errors))
            self.config.enable_stream(self.color_profile)
            self.pipeline.enable_frame_sync()
        info = self.device.get_device_info()
        if "femto bolt" not in info.get_name().lower():
            raise ValueError(f"Femto Bolt용 Xacro에 다른 카메라를 사용할 수 없습니다: {info.get_name()}")
        self.device_info = {"name": info.get_name(), "serial_number": info.get_serial_number(),
                            "firmware_version": info.get_firmware_version()}

    def capture(self):
        sdk, settings = self.sdk, self.settings
        # 각 이동이 끝난 뒤 새 스트림 시작. 이동 중 쌓인 이전 프레임 사용 방지.
        self.pipeline.start(self.config)
        try:
            deadline = time.monotonic() + settings["capture_timeout_sec"]
            count, seen_frames = 0, set()
            while time.monotonic() < deadline:
                frames = self.pipeline.wait_for_frames(500)
                depth = frames.get_depth_frame() if frames is not None else None
                if depth is None:
                    continue
                frame_id = (depth.get_index(), depth.get_timestamp_us())
                if frame_id in seen_frames:
                    continue
                color = frames.get_color_frame() if settings.get("capture_color", False) else None
                if settings.get("capture_color", False) and (color is None or abs(
                        color.get_timestamp_us() - depth.get_timestamp_us()) > settings["color_max_time_delta_ms"] * 1000):
                    continue
                seen_frames.add(frame_id)
                count += 1
                if count <= settings["warmup_frames"]:
                    continue
                received_ns = time.time_ns()
                if depth.get_format() != sdk.OBFormat.Y16:
                    raise ValueError("Y16 Depth만 지원합니다.")
                raw = np.frombuffer(depth.get_data(), dtype=np.uint16).reshape(depth.get_height(), depth.get_width()).copy()
                point_filter = sdk.PointCloudFilter()
                point_filter.set_create_point_format(sdk.OBFormat.POINT)
                point_filter.set_coordinate_system(sdk.OBCoordinateSystemType.RIGHT_HAND)
                points = point_filter.process(frames)
                if points is None:
                    raise RuntimeError("SDK Depth → point cloud 변환 실패")
                points = points.as_points_frame()
                xyz = points_in_meters(points)
                scale = float(depth.get_depth_scale())
                if not np.isfinite(scale) or scale <= 0:
                    raise ValueError("잘못된 Depth scale")
                metadata = {
                    "device": self.device_info, "frame": DEPTH_FRAME, "alignment": "native_depth_no_color_alignment",
                    "axes": "x right, y down, z forward", "host_received_ns": received_ns,
                    "device_timestamp_us": depth.get_timestamp_us(),
                    "sdk_system_timestamp_us": depth.get_system_timestamp_us(), "frame_index": depth.get_index(),
                    "width": depth.get_width(), "height": depth.get_height(), "fps": self.profile.get_fps(),
                    "format": "Y16", "depth_scale_mm_per_unit": scale,
                    "point_scale_mm_per_unit": float(points.get_position_value_scale()),
                    **profile_calibration(self.profile), "discarded_warmup_frames": count - 1,
                    "capture_mode": "single_frame", "captured_frames": 1,
                }
                result = CapturedView(raw, xyz, metadata)
                if color is not None:
                    result.color_rgb, result.color_aligned_rgb = align_color_to_depth(sdk, depth, color)
                    if len(xyz) != raw.size:
                        raise ValueError("점군과 Depth 픽셀 개수가 달라 RGB를 대응시킬 수 없습니다.")
                    extrinsic = self.profile.get_extrinsic_to(self.color_profile)
                    metadata["alignment"] = "color_to_native_depth"
                    metadata["color"] = {
                        "width": color.get_width(), "height": color.get_height(),
                        "fps": self.color_profile.get_fps(), "source_format": str(color.get_format()),
                        "saved_format": "RGB8", "frame_index": color.get_index(),
                        "device_timestamp_us": color.get_timestamp_us(),
                        "sdk_system_timestamp_us": color.get_system_timestamp_us(),
                        "depth_color_time_delta_us": abs(color.get_timestamp_us() - depth.get_timestamp_us()),
                        **profile_calibration(self.color_profile),
                        "depth_to_color": {"rotation": np.asarray(extrinsic.rot).reshape(3, 3).tolist(),
                                           "translation_mm": np.asarray(extrinsic.transform).reshape(3).tolist()},
                        "unmapped_color": "black", "alignment_target_distortion": True,
                        "selection": "same_pair_as_saved_depth",
                    }
                print(f"Depth 1프레임 취득: 유효 {np.count_nonzero(raw):,}픽셀 (시간 방향 통합 없음)", flush=True)
                return result
            raise TimeoutError("유효한 새 Depth/RGB 프레임 취득 시간 초과" if settings.get("capture_color", False)
                               else "유효한 새 Depth 프레임 취득 시간 초과")
        finally:
            self.pipeline.stop()


def validate_capture(before, after, settings):
    for state in (before, after):
        finite_vector(state["joint_deg"], 6, "actual_joint_deg")
        velocity = finite_vector(state["joint_velocity_deg_s"], 6, "actual_joint_velocity_deg_s")
        if state["motion_running"] or np.max(np.abs(velocity)) > settings["stationary_velocity_deg_s"]:
            raise RuntimeError("촬영 전/후 로봇 정지 확인 실패. 해당 점군은 정합에 사용하지 않습니다.")
    drift = float(np.max(np.abs(np.asarray(after["joint_deg"]) - before["joint_deg"])))
    if drift > settings["capture_drift_deg"]:
        raise RuntimeError(f"촬영 중 관절 변화 {drift:.6f} deg 초과. 해당 점군은 정합에 사용하지 않습니다.")
    return drift


def transform_points(points, transform):
    return points @ transform[:3, :3].T + transform[:3, 3]


def save_cloud(path, points, color=None, colors=None):
    import open3d as o3d

    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    if colors is not None:
        colors = np.asarray(colors, dtype=float)
        if colors.shape != (len(points), 3) or not np.isfinite(colors).all() or np.any((colors < 0) | (colors > 1)):
            raise ValueError("점별 RGB는 점군과 개수가 같은 0~1 범위의 Nx3 배열이어야 합니다.")
        cloud.colors = o3d.utility.Vector3dVector(colors)
    elif color is not None:
        cloud.paint_uniform_color(color)
    if not o3d.io.write_point_cloud(str(path), cloud):
        raise OSError(f"점군 저장 실패: {path}")
    return cloud


def save_rgb(path, rgb):
    import cv2

    if not cv2.imwrite(str(path), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)):
        raise OSError(f"RGB 이미지 저장 실패: {path}")


def capture_sequence(cfg, model, robot, camera, output):
    """주입 가능한 robot/camera를 사용하여 순서와 저장 과정을 오프라인 테스트 가능."""
    import open3d as o3d

    manifest = {"status": "in_progress", "created_at": datetime.now(timezone.utc).isoformat(),
                "units": {"point_cloud": "m", "joint": "deg"}, "reference_frame": BASE_FRAME,
                "method": "actual_joint_fk_and_xacro_tf_only", "icp_applied": False,
                "settle_criterion": "stationary_actual_joints",
                "capture_mode": "single_frame",
                "color_mode": "rgb" if cfg["camera"].get("capture_color", False) else "view_colors", "views": []}
    save_json(output / "manifest.json", manifest)
    save_json(output / "settings.json", cfg)
    (output / "model.urdf").write_text(model.xml, encoding="utf-8")
    manifest["model_sha256"] = hashlib.sha256(model.xml.encode()).hexdigest()
    manifest["python"] = sys.executable
    manifest["packages"] = {name: importlib.metadata.version(name)
                            for name in ("numpy", "open3d", "pyorbbecsdk2", "xacro")}
    merged = o3d.geometry.PointCloud()
    merged_view_colors = o3d.geometry.PointCloud()
    palette = view_colors(cfg["views"])
    try:
        for view in cfg["views"]:
            name = view["name"]
            directory = output / name
            directory.mkdir()
            entry = {"name": name, "status": "moving", "target_joint_deg": view["joint_deg"],
                     "view_color_rgb": palette[name]}
            manifest["views"].append(entry)
            save_json(output / "manifest.json", manifest)
            print(f"[{name}] 이동 및 정지 확인", flush=True)
            entry["settled_state"] = robot.move(view["joint_deg"])
            settled = entry["settled_state"]
            print(f"[{name}] 정지 확인: 실제 관절각 {settled['joint_deg']} deg, "
                  f"목표와 최대 차이 {settled['max_target_error_deg']:.4f} deg (기록용)", flush=True)
            save_json(output / "manifest.json", manifest)
            before = robot.snapshot()
            # 이동 명령 종료와 실제 자세의 정지 유지가 확인된 뒤 새 스트림을 시작한다.
            captured = camera.capture()
            raw, xyz, calibration = captured.depth_raw, captured.xyz, captured.metadata
            after = None
            try:
                after = robot.snapshot()  # 디스크 저장보다 먼저 촬영 직후 자세 조회.
            finally:
                # 직후 자세 조회가 실패해도 이미 받은 원본은 남긴다.
                np.save(directory / "depth_raw.npy", raw, allow_pickle=False)
                if captured.color_rgb is not None:
                    save_rgb(directory / "color_rgb.png", captured.color_rgb)
                if captured.color_aligned_rgb is not None:
                    save_rgb(directory / "color_aligned_depth.png", captured.color_aligned_rgb)
                save_json(directory / "camera.json", calibration)
                entry.update(status="captured", before_capture=before, after_capture=after)
                save_json(directory / "pose.json", {"before": before, "after": after, "accepted": False})
                save_json(output / "manifest.json", manifest)
            # 원본 XYZ는 invalid 점만 제외. 거리/ROI 필터 전 데이터 보존.
            valid = np.isfinite(xyz).all(axis=1) & (xyz[:, 2] > 0)
            raw_points = xyz[valid]
            colors = None
            if cfg["camera"].get("capture_color", False):
                aligned_rgb = captured.color_aligned_rgb
                if aligned_rgb is None or aligned_rgb.shape != (*raw.shape, 3) or len(xyz) != raw.size:
                    raise ValueError("RGB 촬영 결과가 없거나 Depth/점군과 픽셀이 대응하지 않습니다.")
                colors = aligned_rgb.reshape(-1, 3)[valid].astype(float) / 255.0
            if len(raw_points):
                save_cloud(directory / "camera_raw.ply", raw_points, colors=colors)
            drift = validate_capture(before, after, cfg["robot"])
            target_error = (np.asarray(after["joint_deg"]) - view["joint_deg"]).tolist()
            max_target_error = max(abs(value) for value in target_error)
            frames = model.transforms(after["joint_deg"])
            transform = frames[DEPTH_FRAME]
            camera_cfg = cfg["camera"]
            keep = ((raw_points[:, 2] >= camera_cfg["min_depth_mm"] / 1000)
                    & (raw_points[:, 2] <= camera_cfg["max_depth_mm"] / 1000))
            points = raw_points[keep]
            if colors is not None:
                colors = colors[keep]
            base_points = transform_points(points, transform)
            recon = cfg["reconstruction"]
            if recon["roi_min_mm"] is not None:
                low, high = np.asarray(recon["roi_min_mm"]) / 1000, np.asarray(recon["roi_max_mm"]) / 1000
                keep = ((base_points >= low) & (base_points <= high)).all(axis=1)
                points, base_points = points[keep], base_points[keep]
                if colors is not None:
                    colors = colors[keep]
            if len(points) < camera_cfg["min_points"]:
                raise RuntimeError(f"{name}: 범위/ROI 내 유효점 부족 {len(points)} < {camera_cfg['min_points']}")
            save_cloud(directory / "camera_filtered.ply", points, colors=colors)
            cloud = save_cloud(directory / "base.ply", base_points, palette[name], colors=colors)
            pose = {"before": before, "after": after, "accepted": True, "joint_drift_deg": drift,
                    "target_joint_deg": view["joint_deg"], "target_error_deg": target_error,
                    "max_target_error_deg": max_target_error,
                    "pose_source": "actual_joint_measurement_after_capture_plus_URDF_FK",
                    "T_base_flange": frames["link_6"].tolist(), "T_base_bracket": frames["bracket_link"].tolist(),
                    "T_flange_depth": (np.linalg.inv(frames["link_6"]) @ transform).tolist(),
                    "T_base_depth": transform.tolist(), "translation_unit": "m"}
            save_json(directory / "pose.json", pose)
            merged += cloud
            view_cloud = o3d.geometry.PointCloud(cloud)
            view_cloud.paint_uniform_color(palette[name])
            merged_view_colors += view_cloud
            entry.update(status="accepted", point_count=len(points), joint_drift_deg=drift,
                         target_error_deg=target_error, max_target_error_deg=max_target_error)
            save_json(output / "manifest.json", manifest)
            print(f"[{name}] 저장 완료: {len(points):,} points", flush=True)
        if not o3d.io.write_point_cloud(str(output / "merged_raw.ply"), merged):
            raise OSError("통합 원본 저장 실패")
        downsampled = merged.voxel_down_sample(cfg["reconstruction"]["voxel_size_mm"] / 1000)
        if not o3d.io.write_point_cloud(str(output / "merged.ply"), downsampled):
            raise OSError("통합 다운샘플 점군 저장 실패")
        if cfg["camera"].get("capture_color", False):
            comparison = merged_view_colors.voxel_down_sample(cfg["reconstruction"]["voxel_size_mm"] / 1000)
            if not o3d.io.write_point_cloud(str(output / "merged_view_colors.ply"), comparison):
                raise OSError("시점 구분색 통합 점군 저장 실패")
        manifest.update(status="complete", raw_point_count=len(merged.points), point_count=len(downsampled.points))
    except BaseException:
        manifest.update(status="failed", error=traceback.format_exc())
        (output / "error.log").write_text(manifest["error"], encoding="utf-8")
        raise
    finally:
        manifest["finished_at"] = datetime.now(timezone.utc).isoformat()
        save_json(output / "manifest.json", manifest)


def show_cloud(path):
    import open3d as o3d

    if not path.is_file():
        raise FileNotFoundError(path)
    cloud = o3d.io.read_point_cloud(str(path))
    if cloud.is_empty():
        raise ValueError(f"빈 점군: {path}")
    o3d.visualization.draw_geometries([cloud, o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)],
                                     window_name=f"Multi-view reconstruction | {path.name}", width=1280, height=900)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check", action="store_true")
    mode.add_argument("--execute", action="store_true")
    mode.add_argument("--view", type=Path, metavar="FILE.ply")
    args = parser.parse_args()
    if args.view:
        show_cloud(args.view.expanduser().resolve())
        return 0
    cfg = load_config(args.config)
    model = ScanModel(cfg["xacro"])
    preflight(cfg, model, args.execute)
    if not args.execute:
        return 0
    for view in cfg["views"]:
        print(f"{view['name']}: {view['joint_deg']} deg")
    print("주의: 실제 로봇의 서보/자동 제어가 활성화됩니다. 이동 구간 충돌 검사는 없습니다.\n"
          "툴 무게·장착·전체 이동 경로·주변 안전·비상정지를 확인하세요. 종료 시 servo-off 합니다.")
    if input("실제 이동/촬영을 시작하려면 SCAN 입력: ").strip() != "SCAN":
        print("취소했습니다. 하드웨어에 연결하지 않았습니다.")
        return 0
    output = Path(cfg["output_dir"]) / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    output.mkdir(parents=True, exist_ok=False)
    print(f"저장 경로: {output}", flush=True)
    try:
        camera = DepthCamera(cfg["camera"])  # 카메라/프로파일 오류는 로봇 초기화 전에 검사.
        with RobotClient(cfg, output) as robot:
            robot.call("initialize", settings=cfg["robot"], timeout=cfg["robot"]["connect_timeout_sec"] + 15)
            capture_sequence(cfg, model, robot, camera, output)
    except BaseException:
        error = traceback.format_exc()
        (output / "error.log").write_text(error, encoding="utf-8")
        manifest_path = output / "manifest.json"
        manifest = json.loads(manifest_path.read_text()) if manifest_path.is_file() else {"views": []}
        manifest.update(status="failed", error=error)
        save_json(manifest_path, manifest)
        raise
    print(f"완료: {output / 'merged.ply'}\n보기: python {Path(__file__).name} --view {output / 'merged.ply'}")
    return 0


if __name__ == "__main__":
    def interrupt(_signum, _frame):
        raise KeyboardInterrupt("스캔 중단 요청")
    signal.signal(signal.SIGTERM, interrupt)
    try:
        raise SystemExit(main())
    except (Exception, KeyboardInterrupt) as exc:
        print(f"스캔 오류: {exc}", file=sys.stderr)
        raise SystemExit(1)
