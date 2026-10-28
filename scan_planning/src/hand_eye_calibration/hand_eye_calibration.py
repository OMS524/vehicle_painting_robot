#!/usr/bin/env python3
# 실행 (먼저 hand_eye_calibration.yaml의 보드 치수/기준 자세/연결 설정 확인):
#   conda activate vehicle_painting_robot_scan_planning
#   python -s /home/oms/vehicle_painting_robot/scan_planning/src/hand_eye_calibration/hand_eye_calibration.py --check
# 카메라만 미리보기: --check 대신 --preview
# 실제 로봇 이동/촬영/보정: --check 대신 --execute (작업 공간 확인 및 시작 동의 필요).
# 저장된 촬영으로 재계산: --check 대신 --solve /촬영/폴더
"""A0912 + Femto Bolt eye-in-hand: SDK 이동/실측 자세, 원본 RGB/C2D 체커보드, OpenCV.

기본/--check: 하드웨어 연결 없는 검사. --preview: 카메라만 사용.
--execute: 기준 자세로 생성한 촬영 자세에서 촬영 후 계산. --solve SESSION: 저장 데이터로 재계산.
--export-depth-variants LOG: 원본/축 길이 보정 Depth TF를 포함한 calibration.json 두 개 저장.
출력 T_parent_child는 child 좌표를 parent 좌표로 변환합니다. 위치 단위 m.
기존 Xacro/로봇 TCP/스캔 알고리즘에 보정값을 자동 적용하지 않습니다.
"""
from __future__ import annotations

import argparse
from datetime import datetime
from itertools import combinations
import json
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
import traceback
from zoneinfo import ZoneInfo

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
SCAN_ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE.parent))
from scan_planner import acquisition as scan
from scan_planner.acquisition import pose_matrix, sdk_pose_matrix, target_key, validate_targets, vector6

DEFAULT_CONFIG = HERE / "hand_eye_calibration.yaml"
DEFAULT_OUTPUT = SCAN_ROOT / "log/hand_eye_calibration"
METHODS = {name: getattr(cv2, "CALIB_HAND_EYE_" + name)
           for name in ("PARK", "TSAI", "HORAUD", "ANDREFF", "DANIILIDIS")}
CAMERA_FRAMES = {"rgb": "camera_color_optical_frame", "c2d": "camera_depth_optical_frame"}
IMAGE_FILES = {"rgb": "color_rgb.png", "c2d": "color_aligned_depth.png"}


def image_source(settings):
    # 과거 설정/로그에는 이 항목이 없으며 모두 원본 RGB이다.
    source = settings.get("image_source", "rgb")
    if not isinstance(source, str) or source not in CAMERA_FRAMES:
        raise ValueError("camera.image_source는 rgb / c2d만 가능합니다.")
    return source


def stamp():
    return datetime.now(ZoneInfo("Asia/Seoul")).strftime("%Y%m%d_%H%M%S_%f")


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def validate_transform(value):
    t = np.asarray(value, dtype=float)
    if (t.shape != (4, 4) or not np.isfinite(t).all()
            or not np.allclose(t[3], [0, 0, 0, 1], atol=1e-8)
            or not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-4)
            or abs(np.linalg.det(t[:3, :3]) - 1) > 1e-4):
        raise ValueError("유한한 SE(3) 변환이 아닙니다. 회전/단위/TF 방향을 확인하세요.")
    return t


def transform(rotation, translation):
    t = np.eye(4)
    t[:3, :3], t[:3, 3] = np.asarray(rotation).reshape(3, 3), np.asarray(translation).reshape(3)
    return validate_transform(t)


def difference(a, b):
    return {"translation_mm": float(np.linalg.norm(a[:3, 3] - b[:3, 3]) * 1000),
            "rotation_deg": float(np.linalg.norm(cv2.Rodrigues(a[:3, :3].T @ b[:3, :3])[0]) * 180 / np.pi)}


def positive(value, name):
    if isinstance(value, bool) or not isinstance(value, (float, int)) or not np.isfinite(value) or value <= 0:
        raise ValueError(f"{name}: 양의 유한한 숫자가 필요합니다.")


def validate_settings(cfg, execute=False):
    board, cal = cfg["board"], cfg["calibration"]
    source = image_source(cfg["camera"])
    corners = board["inner_corners"]
    if (not isinstance(corners, list) or len(corners) != 2
            or any(type(v) is not int or v < 3 for v in corners) or corners[0] == corners[1]):
        raise ValueError("inner_corners: 서로 다른 가로/세로 내부 코너 개수(각 3 이상)를 입력하세요.")
    if board["square_size_mm"] is not None:
        positive(board["square_size_mm"], "square_size_mm")
    if cal["method"] not in METHODS:
        raise ValueError(f"method: {list(METHODS)} 중 선택하세요.")
    if type(cal["min_samples"]) is not int or cal["min_samples"] < 4:
        raise ValueError("min_samples는 4 이상이어야 합니다(leave-one-out 검증 포함). 권장 10~20.")
    for name, value in cal.items():
        if name != "method":
            positive(value, name)
    if cal["min_rotation_axis_ratio"] >= 1 or cal["min_rotation_deg"] >= 180:
        raise ValueError("회전축 비율은 1 미만, 최소 회전각은 180도 미만이어야 합니다.")
    for name in ("width", "height", "fps", "warmup_frames"):
        if type(cfg["camera"][name]) is not int or cfg["camera"][name] <= 0:
            raise ValueError(f"camera.{name}: 양의 정수이어야 합니다.")
    positive(cfg["camera"]["capture_timeout_sec"], "capture_timeout_sec")
    if source == "c2d":
        for name in ("depth_width", "depth_height"):
            value = cfg["camera"].get(name)
            if type(value) is not int or value <= 0:
                raise ValueError(f"camera.{name}: c2d에는 양의 정수 해상도가 필요합니다.")
        positive(cfg["camera"].get("max_frame_time_delta_ms", 50.0), "max_frame_time_delta_ms")
    for name in ("max_velocity_deg_s", "acceleration_deg_s2", "max_move_delta_deg"):
        positive(cfg["motion"][name], name)
    for section, name in ((board, "fixed_board_confirmed"), (cfg["motion"], "plan_confirmed"),
                          (cfg["motion"], "confirm_each_pose")):
        if type(section[name]) is not bool:
            raise ValueError(f"{name}: true/false를 사용하세요.")
    generation = cfg.get("pose_generation")
    if generation is not None:
        validate_generation(generation)
    poses = cfg.get("poses", [])
    if not isinstance(poses, list) or (not poses and generation is None):
        raise ValueError("poses 목록이 필요합니다.")
    names = set()
    for pose in poses:
        if not isinstance(pose, dict) or not re.fullmatch(r"[A-Za-z0-9_-]{1,80}", pose.get("name", "")):
            raise ValueError("pose name은 영문/숫자/_/-의 1~80글자여야 합니다.")
        if pose["name"] in names:
            raise ValueError("중복된 pose name입니다.")
        names.add(pose["name"])
    validate_targets({"views": poses, "cartesian": {"target_frame": "link_6"}})
    if execute:
        if generation is not None and generation["reference_pose_mm_deg"] is None:
            raise ValueError("pose_generation.reference_pose_mm_deg에 기준 플랜지 자세를 입력하세요.")
        if board["square_size_mm"] is None or not board["fixed_board_confirmed"]:
            raise ValueError("실제 보정판 치수와 fixed_board_confirmed를 먼저 설정하세요.")
        if not cfg["motion"]["plan_confirmed"]:
            raise ValueError("안전한 전체 이동 구간을 확인한 뒤 motion.plan_confirmed: true로 설정하세요.")
        if len(poses) < cal["min_samples"] or any(p[target_key(p)] is None for p in poses):
            raise ValueError("min_samples 이상의 검증된 자세를 입력하세요. null 목표로 실행할 수 없습니다.")


def validate_generation(settings):
    reference = settings["reference_pose_mm_deg"]
    if reference is not None:
        vector6(reference, "reference_pose_mm_deg")
    height = settings["board_height_mm"]
    if isinstance(height, bool) or not isinstance(height, (int, float)) or not np.isfinite(height):
        raise ValueError("board_height_mm: base_link 기준의 유한한 높이(mm)가 필요합니다.")
    positive(settings["tilt_deg"], "tilt_deg")
    if settings["tilt_deg"] > 30:
        raise ValueError("tilt_deg는 30도 이하로 설정하세요. 큰 기울기는 자동 생성하지 않습니다.")
    positive(settings["min_camera_height_mm"], "min_camera_height_mm")


def nominal_camera_transform(model):
    """Xacro 고정 장착 TF. 촬영 계획에만 사용하며 보정 계산의 입력은 아니다."""
    parents = {j.find("child").get("link"): j for j in model.robot.findall("joint")}
    child = "camera_color_optical_frame"
    while child != "link_6":
        joint = parents.get(child)
        if joint is None or joint.get("type") != "fixed":
            raise ValueError("link_6 -> camera_color_optical_frame 고정 TF가 필요합니다.")
        child = joint.find("parent").get("link")
    _, _, frames = scan.link_transforms(model.robot)
    return validate_transform(np.linalg.inv(frames["link_6"]) @ frames["camera_color_optical_frame"])


def generate_poses(settings, flange_camera):
    """기준 RGB 시선과 높이 평면의 교점을 중심으로 작은/큰 기울기 원주를 순회한다."""
    validate_generation(settings)
    reference = pose_matrix(settings["reference_pose_mm_deg"])
    flange_camera = validate_transform(flange_camera)
    camera = reference @ flange_camera
    height = settings["board_height_mm"] / 1000.0
    direction = camera[:3, 2]  # optical +Z: RGB 시선
    if direction[2] > -np.cos(np.deg2rad(20)):
        raise ValueError("기준 RGB 시선이 수직 아래에서 20도 이내여야 합니다. 플랜지 RPY/장착 TF를 확인하세요.")
    distance = (height - camera[2, 3]) / direction[2]
    if distance <= 0:
        raise ValueError("기준 카메라가 설정한 보정판 높이보다 위에 있어야 합니다.")
    center = camera[:3, 3] + distance * direction
    camera_flange = np.linalg.inv(flange_camera)
    # 중심 1 + 작은 기울기 8 + 큰 기울기 8 = 17개. 원주 경계의 불필요한 점프는 피한다.
    rotations = [camera[:3, :3]]
    for tilt, angles in ((settings["tilt_deg"] / 2, range(8)),
                         (settings["tilt_deg"], range(7, -1, -1))):
        for index in angles:
            azimuth = index * np.pi / 4
            vector = np.deg2rad(tilt) * np.array([np.cos(azimuth), np.sin(azimuth), 0])
            rotations.append(camera[:3, :3] @ Rotation.from_rotvec(vector).as_matrix())
    poses, heights = [], []
    for index, rotation in enumerate(rotations):
        desired_camera = transform(rotation, center - distance * rotation[:, 2])
        clearance = float((desired_camera[2, 3] - height) * 1000)
        if clearance < settings["min_camera_height_mm"]:
            raise ValueError(f"pose_{index + 1:02d}: 명목 카메라 높이 {clearance:.1f} mm < min_camera_height_mm. "
                             "기준 자세를 높이거나 기울기를 줄이세요. 로봇/툴 충돌 검사는 별도입니다.")
        desired_flange = validate_transform(desired_camera @ camera_flange)
        values = np.r_[desired_flange[:3, 3] * 1000,
                       Rotation.from_matrix(desired_flange[:3, :3]).as_euler("xyz", degrees=True)].tolist()
        # 기준 자세는 입력한 RPY 표현 그대로 보존한다(동등한 Euler 표현으로 바꾸지 않음).
        poses.append({"name": f"pose_{index + 1:02d}",
                      "pose_mm_deg": list(settings["reference_pose_mm_deg"]) if index == 0 else values})
        heights.append(clearance)
    return poses, {"planning_only": True, "optical_frame": "camera_color_optical_frame",
                   "T_flange_color_nominal": flange_camera.tolist(),
                   "estimated_target_center_mm": (center * 1000).tolist(),
                   "camera_target_distance_mm": float(distance * 1000),
                   "camera_heights_above_assumed_plane_mm": heights,
                   "collision_checked": False}


def load_settings(path):
    path = Path(path).expanduser().resolve()
    cfg = yaml.safe_load(path.read_text(encoding="utf-8"))
    for key in ("scan_config", "output_dir"):
        p = Path(cfg[key]).expanduser()
        cfg[key] = str((path.parent / p).resolve() if not p.is_absolute() else p.resolve())
    validate_settings(cfg)
    return cfg


def robot_config(cfg):
    result = scan.load_config(cfg["scan_config"])
    result["views"] = cfg.get("poses", [])
    result["cartesian"] = {"target_frame": "link_6"}
    for name in ("max_velocity_deg_s", "acceleration_deg_s2", "max_move_delta_deg"):
        result["robot"][name] = cfg["motion"][name]
    return result


def preflight(cfg, require_control=True):
    robot_cfg = robot_config(cfg)
    model = scan.ScanModel(robot_cfg["xacro"])
    if cfg.get("pose_generation") is not None:
        cfg["poses"] = []  # 생성 설정이 있으면 과거/수동 목표를 실행하지 않는다.
        cfg.pop("generated_plan", None)
        if cfg["pose_generation"]["reference_pose_mm_deg"] is not None:
            cfg["poses"], cfg["generated_plan"] = generate_poses(
                cfg["pose_generation"], nominal_camera_transform(model))
            cfg["generated_plan"]["excitation"] = excitation(
                [pose_matrix(p["pose_mm_deg"]) for p in cfg["poses"]], cfg["calibration"])
            print("촬영용 추정 보정판 중심 (base_link, mm):",
                  cfg["generated_plan"]["estimated_target_center_mm"])
            print("계획 자세 회전 다양성:", cfg["generated_plan"]["excitation"])
            print("높이/장착 TF는 계획에만 사용합니다. 아래 목표는 IK·충돌 안전을 검증한 경로가 아닙니다.")
            for pose in cfg["poses"]:
                print(pose["name"], "XYZ/RPY:", [round(v, 3) for v in pose["pose_mm_deg"]])
        else:
            print("미입력: pose_generation.reference_pose_mm_deg (보정판 중심을 아래로 보는 플랜지 자세)")
        robot_cfg["views"] = cfg["poses"]
    missing_control = [key for key in ("control_python", "control_wrapper", "control_library")
                       if not robot_cfg.get(key) or not Path(robot_cfg[key]).is_file()]
    if require_control and missing_control:
        raise ValueError("scan_config에 새 제어 연결부 경로를 설정하세요: " + ", ".join(missing_control)
                         + ". 이전 테스트 제어 코드를 자동으로 사용하지 않습니다.")
    for pose in cfg["poses"]:
        if target_key(pose) == "joint_deg" and pose["joint_deg"] is not None:
            model.transforms(pose["joint_deg"])
    if not require_control:
        print(f"설정 검사 완료: {len(cfg['poses'])}개 자세 / {image_source(cfg['camera'])} / "
              f"{cfg['calibration']['method']} / 로그: {cfg['output_dir']}")
        if missing_control:
            print("실장비 실행 전 설정 필요: " + ", ".join(missing_control))
        print("제어 프로세스·로봇·카메라는 실행하지 않았습니다. 이동 안전 검사는 별도입니다.")
        return robot_cfg, model
    result = subprocess.run(scan.worker_command(robot_cfg, "--check"),
        env=scan.worker_environment(robot_cfg["control_python"]),
        input=json.dumps({"model_xml": model.xml, "cartesian": robot_cfg["cartesian"]}),
        text=True, capture_output=True, timeout=30)
    if result.returncode:
        raise RuntimeError("제어 환경 검사 실패:\n" + result.stdout + result.stderr)
    import pyorbbecsdk  # noqa: F401 -- Context 생성/장치 연결 없음
    print("파일·SDK 연결부 검사 완료. 로봇/카메라에 연결하지 않았습니다.")
    print(f"제어 Python: {robot_cfg['control_python']}\n촬영 자세: {len(cfg['poses'])}개")
    source = image_source(cfg["camera"])
    print(f"보정 이미지: {source} / 결과 광학 프레임: {CAMERA_FRAMES[source]}")
    if source == "c2d":
        print("C2D는 Depth 측정 및 SDK RGB–Depth 보정값에 의존합니다. 별도 정확도 검증이 필요합니다.")
    if cfg["board"]["square_size_mm"] is None:
        print("미입력: board.square_size_mm (실제 보정판 한 칸 길이)")
    missing = [p["name"] for p in cfg["poses"] if p[target_key(p)] is None]
    if missing:
        print("미입력 목표: " + ", ".join(missing))
    return robot_cfg, model


def camera_matrix(profile):
    intrinsic, distortion = profile.get_intrinsic(), profile.get_distortion()
    model = str(distortion.model)
    if not (model.endswith(".BROWN_CONRADY") or model.endswith(".BROWN_CONRADY_K6")):
        raise ValueError(f"지원하지 않는 카메라 왜곡 모델: {model}. pinhole PnP에 그대로 넣지 않습니다.")
    k = [[intrinsic.fx, 0, intrinsic.cx], [0, intrinsic.fy, intrinsic.cy], [0, 0, 1]]
    # OpenCV rational model: k1,k2,p1,p2,k3,k4,k5,k6 (SDK 필드 순서와 다름).
    d = [distortion.k1, distortion.k2, distortion.p1, distortion.p2, distortion.k3]
    if model.endswith(".BROWN_CONRADY_K6"):
        d += [distortion.k4, distortion.k5, distortion.k6]
    if not np.isfinite(k).all() or not np.isfinite(d).all() or intrinsic.fx <= 0 or intrinsic.fy <= 0:
        raise ValueError("SDK 카메라 내부 파라미터가 유효하지 않습니다.")
    return {"K": k, "dist_coeffs": d, "distortion_model": model,
            "width": profile.get_width(), "height": profile.get_height(), "fps": profile.get_fps()}


class CalibrationCamera:
    """rgb는 원본 Color, c2d는 native Depth 격자/왜곡에 맞춘 Color로 보정."""
    def __init__(self, settings):
        import pyorbbecsdk as sdk
        self.sdk, self.settings = sdk, settings
        self.image_source = image_source(settings)
        self.last_raw_rgb, self.last_depth_raw = None, None
        self.context = sdk.Context()
        devices = self.context.query_devices()
        if settings["serial_number"]:
            device = devices.get_device_by_serial_number(settings["serial_number"])
        elif devices.get_count() == 1:
            device = devices.get_device_by_index(0)
        else:
            raise RuntimeError("카메라를 한 대만 연결하거나 serial_number를 지정하세요.")
        info = device.get_device_info()
        if "femto bolt" not in info.get_name().lower():
            raise ValueError(f"이 테스트는 Femto Bolt용입니다: {info.get_name()}")
        self.pipeline = sdk.Pipeline(device)
        profiles = self.pipeline.get_stream_profile_list(sdk.OBSensorType.COLOR_SENSOR)
        profile = profiles.get_video_stream_profile(settings["width"], settings["height"],
                                                   sdk.OBFormat.MJPG, settings["fps"])
        rgb_camera = camera_matrix(profile)
        self.config = sdk.Config()
        self.config.enable_stream(profile)
        dp = None
        if self.image_source == "c2d":
            dp = self.pipeline.get_stream_profile_list(sdk.OBSensorType.DEPTH_SENSOR).get_video_stream_profile(
                settings["depth_width"], settings["depth_height"], sdk.OBFormat.Y16, settings["fps"])
            self.config.enable_stream(dp)
            self.config.set_frame_aggregate_output_mode(sdk.OBFrameAggregateOutputMode.FULL_FRAME_REQUIRE)
            self.pipeline.enable_frame_sync()
            # scan.align_color_to_depth의 TargetDistortion=1에 맞춰 native Depth K/왜곡을 사용.
            self.metadata = camera_matrix(dp)
            self.metadata["alignment"] = "color_to_native_depth"
            self.metadata["alignment_target_distortion"] = True
        else:
            self.metadata = dict(rgb_camera)
            self.metadata["alignment"] = "none"
        self.metadata.update(image_source=self.image_source,
                             camera_frame=CAMERA_FRAMES[self.image_source],
                             calibration_image_file=IMAGE_FILES[self.image_source],
                             rgb_camera=rgb_camera)
        self.metadata["device"] = {"name": info.get_name(), "serial": info.get_serial_number(),
                                   "firmware": info.get_firmware_version()}
        # Optional factory Depth->RGB transform. RGB hand-eye remains usable if this is unavailable.
        try:
            if dp is None:
                dp = self.pipeline.get_stream_profile_list(sdk.OBSensorType.DEPTH_SENSOR).get_video_stream_profile(
                    0, 0, sdk.OBFormat.Y16, settings["fps"])
            ex = dp.get_extrinsic_to(profile)
            raw = np.eye(4)
            raw[:3, :3], raw[:3, 3] = np.asarray(ex.rot).reshape(3, 3), np.asarray(ex.transform) / 1000
            self.metadata["T_color_depth_sdk_raw"] = raw.tolist()
            self.metadata["T_color_depth"] = validate_transform(raw).tolist()
        except Exception as error:
            self.metadata["depth_extrinsic_warning"] = str(error)

    def capture(self):
        # 매 자세마다 새 스트림을 시작하므로 이전 이동 중 버퍼의 이미지를 사용하지 않는다.
        deadline = time.monotonic() + self.settings["capture_timeout_sec"]
        count, previous, previous_depth = 0, None, None
        self.last_raw_rgb, self.last_depth_raw = None, None
        try:
            self.pipeline.start(self.config)
            while time.monotonic() < deadline:
                frames = self.pipeline.wait_for_frames(500)
                frame = frames.get_color_frame() if frames is not None else None
                if frame is None:
                    continue
                identity = (frame.get_index(), frame.get_timestamp_us())
                if identity == previous:
                    continue
                depth, depth_identity, delta_ms = None, None, None
                if self.image_source == "c2d":
                    depth = frames.get_depth_frame()
                    if depth is None:
                        continue
                    depth_identity = (depth.get_index(), depth.get_timestamp_us())
                    delta_ms = abs(identity[1] - depth_identity[1]) / 1000.0
                    if (depth_identity == previous_depth
                            or delta_ms > self.settings.get("max_frame_time_delta_ms", 50.0)):
                        continue
                previous = identity
                previous_depth = depth_identity
                count += 1
                if count <= self.settings["warmup_frames"]:
                    continue
                info = {"frame_index": identity[0], "device_timestamp_us": identity[1],
                        "host_received_ns": time.time_ns(), "warmup_frames": count - 1,
                        "image_source": self.image_source, "camera_frame": CAMERA_FRAMES[self.image_source]}
                if depth is not None:
                    # 기존 스캔과 같은 SDK C2D. 깊이 보간/회전행렬 보정은 추가하지 않는다.
                    self.last_raw_rgb, rgb = scan.align_color_to_depth(self.sdk, depth, frame)
                    self.last_depth_raw = np.frombuffer(depth.get_data(), dtype=np.uint16).reshape(
                        depth.get_height(), depth.get_width()).copy()
                    info.update(depth_frame_index=depth_identity[0],
                                depth_device_timestamp_us=depth_identity[1],
                                color_depth_time_delta_ms=delta_ms,
                                depth_value_scale_mm=float(depth.get_depth_scale()))
                else:
                    rgb = scan.rgb_image(self.sdk, scan.color_to_rgb_frame(self.sdk, frame))
                    self.last_raw_rgb = rgb
                if rgb.shape[:2] != (self.metadata["height"], self.metadata["width"]):
                    raise ValueError("보정 이미지와 선택한 광학 프레임의 내부 파라미터 해상도가 다릅니다.")
                return rgb, info
            raise TimeoutError(f"새 {self.image_source} 프레임을 제한 시간 안에 취득하지 못했습니다.")
        finally:
            self.pipeline.stop()

    def save_capture(self, folder, rgb):
        save_image(folder / "color_rgb.png", cv2.cvtColor(self.last_raw_rgb, cv2.COLOR_RGB2BGR))
        if self.image_source == "c2d":
            save_image(folder / IMAGE_FILES["c2d"], cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            np.save(folder / "depth_raw.npy", self.last_depth_raw, allow_pickle=False)


def board_points(board):
    if board["square_size_mm"] is None:
        raise ValueError("board.square_size_mm를 입력하세요.")
    points = np.zeros((np.prod(board["inner_corners"]), 3), dtype=np.float64)
    points[:, :2] = np.mgrid[0:board["inner_corners"][0], 0:board["inner_corners"][1]].T.reshape(-1, 2)
    return points * board["square_size_mm"] / 1000


def detect_board(rgb, board, camera, max_error):
    if rgb.shape[:2] != (camera["height"], camera["width"]):
        raise ValueError("보정 이미지 해상도와 내부 파라미터가 다릅니다.")
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    pattern = tuple(board["inner_corners"])
    found, corners = cv2.findChessboardCornersSB(gray, pattern,
        flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
    if not found:
        raise ValueError("체커보드 전체 코너를 찾지 못했습니다. 조명/초점/가림/보정판 설정을 확인하세요.")
    # Symmetric boards have no unique ID. This only resolves the common 180-degree reversal;
    # the same physical corner MUST stay upper-left across the capture sequence.
    if corners[0].sum() > corners[-1].sum():
        corners = corners[::-1].copy()
    points = board_points(board)
    k, d = np.asarray(camera["K"], dtype=float), np.asarray(camera["dist_coeffs"], dtype=float)
    ok, rv, tv = cv2.solvePnP(points, corners, k, d, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise ValueError("체커보드 PnP 실패")
    t = transform(cv2.Rodrigues(rv)[0], tv)
    if np.min((points @ t[:3, :3].T + t[:3, 3])[:, 2]) <= 0:
        raise ValueError("체커보드 PnP가 카메라 뒤쪽 해를 반환했습니다.")
    projected = cv2.projectPoints(points, rv, tv, k, d)[0].reshape(-1, 2)
    error = float(np.sqrt(np.mean(np.sum((projected - corners.reshape(-1, 2)) ** 2, axis=1))))
    if not np.isfinite(error) or error > max_error:
        raise ValueError(f"재투영 RMS {error:.3f}px > 허용 {max_error:.3f}px")
    overlay = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    cv2.drawChessboardCorners(overlay, pattern, corners, True)
    cv2.drawFrameAxes(overlay, k, d, rv, tv, board["square_size_mm"] * 3 / 1000)
    origin = tuple(np.rint(corners[0, 0]).astype(int))
    cv2.circle(overlay, origin, 12, (0, 0, 255), 3)
    cv2.putText(overlay, "O", origin, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    source = image_source(camera)
    target_key = "T_depth_board" if source == "c2d" else "T_color_board"
    return {target_key: t.tolist(), "image_source": source, "camera_frame": CAMERA_FRAMES[source],
            "reprojection_rms_px": error,
            "corners_px": corners.reshape(-1, 2).tolist()}, overlay


def excitation(poses, cal):
    rotations, equations = [], []
    for a, b in combinations(poses, 2):
        relative = np.linalg.inv(a) @ b
        vector = cv2.Rodrigues(relative[:3, :3])[0].ravel()
        if np.rad2deg(np.linalg.norm(vector)) >= cal["min_rotation_deg"]:
            rotations.append(vector)
            equations.append(relative[:3, :3] - np.eye(3))
    if len(rotations) < 2:
        raise ValueError("유효한 상대 회전이 부족합니다. 위치 이동만으로 hand-eye를 계산할 수 없습니다.")
    sv = np.linalg.svd(np.array(rotations), compute_uv=False)
    ratio = float(sv[1] / sv[0])
    ts = np.linalg.svd(np.vstack(equations), compute_uv=False)
    condition = float(ts[0] / ts[-1]) if ts[-1] > 1e-12 else float("inf")
    if ratio < cal["min_rotation_axis_ratio"] or condition > cal["max_translation_condition"]:
        raise ValueError(f"자세 다양성 부족/퇴화: 회전축 비율 {ratio:.4g}, 이동 방정식 조건수 {condition:.4g}. "
                         "서로 평행하지 않은 회전축으로 보정판을 바라보세요.")
    return {"rotation_axis_ratio": ratio, "translation_condition": condition,
            "usable_relative_motions": len(rotations)}


def fit_hand_eye(grippers, targets, method):
    r, t = cv2.calibrateHandEye([p[:3, :3] for p in grippers], [p[:3, 3] for p in grippers],
                              [p[:3, :3] for p in targets], [p[:3, 3] for p in targets],
                              method=METHODS[method])
    return transform(r, t)


def solve_samples(samples, cal, source="rgb"):
    source = image_source({"image_source": source})
    if len(samples) < cal["min_samples"]:
        raise ValueError(f"유효 시점 {len(samples)}개 < min_samples {cal['min_samples']}")
    grippers = [validate_transform(s["T_base_flange"]) for s in samples]
    target_key = "T_depth_board" if source == "c2d" else "T_color_board"
    if any(image_source(s) != source for s in samples):
        raise ValueError("RGB/C2D 표본을 섞어서 보정할 수 없습니다.")
    targets = [validate_transform(s[target_key]) for s in samples]
    diversity = excitation(grippers, cal)
    x = fit_hand_eye(grippers, targets, cal["method"])
    boards = [g @ x @ c for g, c in zip(grippers, targets)]
    u, _, vt = np.linalg.svd(sum(p[:3, :3] for p in boards))
    correction = np.diag([1, 1, np.linalg.det(u @ vt)])
    center = transform(u @ correction @ vt, np.mean([p[:3, 3] for p in boards], axis=0))
    residuals = [{"name": s["name"], **difference(p, center)} for s, p in zip(samples, boards)]
    loo, warnings = [], []
    for i, sample in enumerate(samples):
        others_g, others_c = grippers[:i] + grippers[i+1:], targets[:i] + targets[i+1:]
        try:
            excitation(others_g, cal)
            estimate = fit_hand_eye(others_g, others_c, cal["method"])
            loo.append({"omitted": sample["name"], **difference(estimate, x)})
        except (ValueError, cv2.error) as error:
            warnings.append(f"leave-one-out {sample['name']}: {error}")
    if max(r["translation_mm"] for r in residuals) > cal["max_board_consistency_mm"]:
        warnings.append("고정 보정판의 위치 일관성 한도 초과")
    if max(r["rotation_deg"] for r in residuals) > cal["max_board_consistency_deg"]:
        warnings.append("고정 보정판의 회전 일관성 한도 초과. 코너 원점 뒤집힘도 확인하세요.")
    if loo and max(r["translation_mm"] for r in loo) > cal["max_leave_one_out_translation_mm"]:
        warnings.append("한 시점 제외 시 보정 이동량 변화 한도 초과")
    if loo and max(r["rotation_deg"] for r in loo) > cal["max_leave_one_out_rotation_deg"]:
        warnings.append("한 시점 제외 시 보정 회전 변화 한도 초과")
    result_key = "T_flange_depth" if source == "c2d" else "T_flange_color"
    return {result_key: x.tolist(), "image_source": source, "camera_frame": CAMERA_FRAMES[source],
            "T_base_board_mean": center.tolist(),
            "excitation": diversity, "board_consistency": residuals, "leave_one_out": loo,
            "quality_checks_passed": not warnings, "warnings": warnings}


def validate_capture(before, after, robot_settings, cal):
    scan.validate_capture(before, after, robot_settings)
    a = sdk_pose_matrix(before["flange_pose_mm_zyz_deg"])
    b = sdk_pose_matrix(after["flange_pose_mm_zyz_deg"])
    drift = difference(a, b)
    if (drift["translation_mm"] > cal["max_capture_translation_mm"]
            or drift["rotation_deg"] > cal["max_capture_rotation_deg"]):
        raise RuntimeError(f"촬영 중 실제 플랜지 자세 변화: {drift}. 해당 표본을 사용하지 않습니다.")
    return b, drift


def save_image(path, image_bgr):
    if not cv2.imwrite(str(path), image_bgr):
        raise OSError(f"이미지 저장 실패: {path}")


def add_camera_transforms(report, camera):
    """핸드아이 해의 기준을 유지한다. C2D 해에 SDK 외부 변환을 다시 곱하지 않는다."""
    if image_source(camera) == "c2d":
        validate_transform(report["T_flange_depth"])
        report["depth_transform_source"] = "hand_eye_from_c2d"
        report["warnings"].append(
            "C2D 이미지 생성은 SDK RGB–Depth 외부 파라미터와 Depth 측정에 의존합니다. "
            "T_flange_depth가 SE(3)이어도 실제 정합 정확도는 독립 검증해야 합니다.")
        if camera.get("depth_extrinsic_warning"):
            report["warnings"].append("C2D에 사용한 SDK 외부 파라미터 주의: " + camera["depth_extrinsic_warning"])
    elif "T_color_depth" in camera:
        report["T_flange_depth"] = (np.asarray(report["T_flange_color"]) @
                                     validate_transform(camera["T_color_depth"])).tolist()
        report["depth_transform_source"] = "rgb_hand_eye_composed_with_sdk_extrinsic"
    else:
        report["warnings"].append("Depth TF 미출력: SDK Depth->RGB 외부 파라미터 없음/검증 실패: "
                                  + camera.get("depth_extrinsic_warning", "unknown"))


def solve_session(session, excluded=(), output_dir=None):
    session = Path(session).expanduser().resolve()
    manifest, cfg = read_json(session / "manifest.json"), read_json(session / "settings.json")
    if manifest.get("schema") != "doosan_eye_in_hand_v1":
        raise ValueError("이 도구로 촬영한 eye-in-hand 세션이 아닙니다.")
    validate_settings(cfg)
    known = {s["name"] for s in manifest["samples"]}
    if set(excluded) - known:
        raise ValueError(f"존재하지 않는 제외 시점: {set(excluded) - known}")
    output = Path(output_dir or DEFAULT_OUTPUT).expanduser().resolve() / ("calibration_" + stamp())
    output.mkdir(parents=True, exist_ok=False)
    report = {"status": "running", "method": cfg["calibration"]["method"], "source_session": str(session),
              "excluded": list(excluded), "translation_unit": "m", "opencv_version": cv2.__version__,
              "transform_convention": "T_parent_child maps child coordinates to parent coordinates",
              "robot_pose_source": "Doosan SDK measured base->flange, mm/Euler ZYZ converted to m",
              "applied_to_robot_or_xacro": False, "samples": []}
    try:
        camera = read_json(session / "camera.json")
        source = image_source(cfg["camera"])
        if (image_source(camera) != source
                or camera.get("camera_frame", CAMERA_FRAMES["rgb"]) != CAMERA_FRAMES[source]
                or camera.get("calibration_image_file", IMAGE_FILES["rgb"]) != IMAGE_FILES[source]):
            raise ValueError("촬영 설정과 camera.json의 이미지 종류/좌표계가 다릅니다. 로그의 모드를 변경하지 마세요.")
        report.update(image_source=source, camera_frame=CAMERA_FRAMES[source],
                      calibration_image_file=IMAGE_FILES[source])
        robot_settings = read_json(session / "robot_settings.json")["robot"]
        for entry in manifest["samples"]:
            if entry["name"] in excluded or entry.get("status") != "accepted":
                continue
            folder = (session / entry["name"]).resolve()
            if folder.parent != session:
                raise ValueError("시점 경로가 세션 밖을 가리킵니다.")
            record = read_json(folder / "sample.json")
            if image_source(record) != source:
                raise ValueError("세션과 표본의 RGB/C2D 모드가 다릅니다.")
            _, drift = validate_capture(record["before"], record["after"],
                                        robot_settings, cfg["calibration"])
            rgb = cv2.imread(str(folder / IMAGE_FILES[source]))
            if rgb is None:
                raise ValueError(f"보정 이미지 읽기 실패: {folder / IMAGE_FILES[source]}")
            observation, overlay = detect_board(cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB), cfg["board"], camera,
                                               cfg["calibration"]["max_reprojection_error_px"])
            sample = {"name": entry["name"], **observation, "capture_drift": drift,
                      "T_base_flange": sdk_pose_matrix(record["after"]["flange_pose_mm_zyz_deg"]).tolist()}
            report["samples"].append(sample)
            save_image(output / (entry["name"] + "_corners.png"), overlay)
        report.update(solve_samples(report["samples"], cfg["calibration"], source))
        add_camera_transforms(report, camera)
        report["status"] = "computed_requires_physical_validation"
        report["notes"] = ["표본 내 보정판 일관성과 leave-one-out 안정성은 절대 정확도 보증이 아닙니다.",
                           "체커보드 코너 원점이 모든 이미지에서 같은 물리적 모서리인지 확인하세요.",
                           "SDK 플랜지 기준 보정입니다. URDF FK 기반 스캔에 적용할 때 기구학 차이를 검증하세요.",
                           ("C2D의 Depth 내부 파라미터로 T_flange_depth를 직접 추정합니다. SDK 외부 TF를 다시 곱하지 않습니다."
                            if source == "c2d" else
                            "T_flange_color는 RGB 광학 프레임. Depth는 SDK Depth->RGB TF가 유효할 때만 계산합니다.")]
        result_key = "T_flange_depth" if source == "c2d" else "T_flange_color"
        print(f"{source} / {result_key} (위치 m):\n", np.asarray(report[result_key]))
        print("표본 품질 검사:", "통과 (별도 실물 검증 필요)" if report["quality_checks_passed"] else "재검토 필요")
        for warning in report["warnings"]:
            print("주의:", warning)
    except BaseException:
        report["status"] = "failed"
        report["error"] = traceback.format_exc()
        raise
    finally:
        scan.save_json(output / "calibration.json", report)
        print(f"계산 로그: {output / 'calibration.json'}")
    return output


def depth_variant_sources(path):
    """세션은 최근 완료 계산을, 계산 폴더/JSON은 지정한 계산을 사용한다."""
    path = Path(path).expanduser().resolve()
    if path.is_dir() and (path / "calibration.json").is_file():
        path = path / "calibration.json"
    elif path.is_dir():
        for candidate in sorted(path.glob("calibration_*/calibration.json"), reverse=True):
            report = read_json(candidate)
            if report.get("status") == "computed_requires_physical_validation":
                path = candidate
                break
        else:
            raise ValueError("완료된 calibration_*/calibration.json이 없습니다. 먼저 --solve LOG를 실행하세요.")
    report = read_json(path)
    if report.get("status") != "computed_requires_physical_validation":
        raise ValueError("완료된 RGB 핸드아이 calibration.json을 지정하세요.")
    if image_source(report) != "rgb" or "T_flange_color" not in report:
        raise ValueError("--export-depth-variants는 RGB 보정 전용입니다. C2D의 직접 계산 Depth TF에는 적용하지 않습니다.")
    if report.get("translation_unit") != "m":
        raise ValueError("위치 단위가 m인 핸드아이 결과가 필요합니다.")
    # 로그를 옮긴 경우에도 현재 계산 파일 주변의 camera.json을 먼저 사용한다.
    folders = [path.parent, path.parent.parent, path.parent.parent.parent]
    if report.get("source_session"):
        folders.append(Path(report["source_session"]).expanduser().resolve())
    for session in folders:
        if (session / "camera.json").is_file():
            return session, path, report, read_json(session / "camera.json")
    raise ValueError("계산에 대응하는 세션의 camera.json을 찾을 수 없습니다.")


def depth_tf_variants(flange_color, color_depth_raw, axis="x"):
    """비교용: 원본을 보존하고 지정한 열 벡터만 정규화한다. 직교화는 하지 않는다."""
    flange_color = validate_transform(flange_color)
    raw = np.asarray(color_depth_raw, dtype=float)
    if (raw.shape != (4, 4) or not np.isfinite(raw).all()
            or not np.allclose(raw[3], [0, 0, 0, 1], atol=1e-8, rtol=0)):
        raise ValueError("SDK 원본은 유한한 4×4 동차행렬이어야 합니다.")
    if axis not in ("x", "y", "z"):
        raise ValueError("보정할 축은 x / y / z 중 하나여야 합니다.")
    column = ("x", "y", "z").index(axis)
    length = float(np.linalg.norm(raw[:3, column]))
    if not np.isfinite(length) or length <= 1e-12:
        raise ValueError(f"{axis}축 벡터 길이가 0 또는 너무 작아 정규화할 수 없습니다.")
    normalized = raw.copy()
    normalized[:3, column] /= length

    def diagnostics(t):
        r = t[:3, :3]
        gram = r.T @ r
        try:
            validate_transform(t)
            valid = True
        except ValueError:
            valid = False
        return {"axis_lengths_xyz": np.linalg.norm(r, axis=0).tolist(),
                "axis_dot_products": gram.tolist(),
                "max_abs_RtR_minus_I": float(np.max(np.abs(gram - np.eye(3)))),
                "determinant": float(np.linalg.det(r)), "passes_se3_tolerance": valid}

    variants = {}
    for name, tf in (("raw", raw), ("unit_axis", normalized)):
        flange_depth = flange_color @ tf
        variants[name] = {
            "variant": name, "T_flange_color": flange_color.tolist(),
            "T_color_depth": tf.tolist(), "T_flange_depth": flange_depth.tolist(),
            "normalization": {"method": "none" if name == "raw" else "divide_selected_column_by_its_norm",
                              "axis": axis, "column_index": column,
                              "original_axis_length": length,
                              "scale_factor": 1.0 if name == "raw" else 1.0 / length,
                              "orthogonalization_applied": False},
            "diagnostics": {"T_color_depth": diagnostics(tf),
                            "T_flange_depth": diagnostics(flange_depth)},
        }
    return variants


def export_depth_variants(path, axis="x", output_dir=None):
    """기존 보정 보고서 전체에 원본/축 길이 보정 Depth TF를 추가해 각각 저장한다."""
    session, calibration_file, report, camera = depth_variant_sources(path)
    source_key = next((key for key in ("T_color_depth_sdk_raw", "T_color_depth") if key in camera), None)
    if source_key is None:
        raise ValueError("camera.json에 저장된 SDK Depth->RGB 행렬이 없습니다. 로그만으로 계산할 수 없습니다.")
    variants = depth_tf_variants(report["T_flange_color"], camera[source_key], axis)
    provenance = {
        "source_calibration_file": str(calibration_file),
        "source_camera_file": str(session / "camera.json"), "source_camera_transform_key": source_key,
        "source_warnings": report.get("warnings", []), "source_notes": report.get("notes", []),
        "composition": "T_flange_depth = T_flange_color @ T_color_depth",
    }
    output = Path(output_dir or DEFAULT_OUTPUT).expanduser().resolve() / ("depth_tf_variants_" + stamp())
    output.mkdir(parents=True, exist_ok=False)
    print(f"사용한 RGB 보정값: {calibration_file}")
    for name, result in variants.items():
        folder = output / name
        folder.mkdir()
        filename = folder / "calibration.json"
        # RGB 해, 표본, 방법, 오차, 품질 플래그와 기존 최상위 형식을 유지한다.
        derived = {**report, "T_color_depth": result["T_color_depth"],
                   "T_flange_depth": result["T_flange_depth"],
                   "depth_tf_export": {**provenance, "variant": name,
                                       "normalization": result["normalization"],
                                       "diagnostics": result["diagnostics"]}}
        derived["warnings"] = [w for w in report.get("warnings", []) if not w.startswith("Depth TF 미출력:")]
        if not result["diagnostics"]["T_flange_depth"]["passes_se3_tolerance"]:
            derived["warnings"].append("Depth TF 회전행렬 검사 실패: 원본 값을 보존한 정합 비교용 결과입니다.")
        derived["notes"] = [n for n in report.get("notes", [])
                            if not n.startswith("T_flange_color는 RGB 광학 프레임.")]
        derived["notes"].append("Depth TF 추가 저장: " + ("SDK 원본 행렬 그대로 사용." if name == "raw" else
                                  f"SDK 행렬의 {axis.upper()}축 열 벡터 길이만 1로 보정. 방향/다른 축/이동량 유지, 직교화 없음."))
        scan.save_json(filename, derived)
        print(f"[{name}] T_flange_depth (위치 m):\n", np.asarray(result["T_flange_depth"]))
        print("Depth->RGB 축 길이:", result["diagnostics"]["T_color_depth"]["axis_lengths_xyz"])
        print(f"저장: {filename}")
    return output


def capture_session(cfg, robot_cfg, model):
    validate_settings(cfg, execute=True)  # SDK Context/로봇 생성보다 먼저 차단
    if cfg.get("pose_generation") is not None and robot_cfg["mounting_tf_confirmed"] is not True:
        raise ValueError("자동 촬영 계획에 사용할 Xacro 장착 TF를 확인하고 scan_config의 mounting_tf_confirmed를 설정하세요.")
    print("실제 로봇의 서보/자동 제어가 활성화됩니다. 충돌 검사/자동 회피는 없습니다.\n"
          "카메라 고정, 보정판 고정, 툴 무게/무게중심, 전체 이동 구간, 케이블, 비상정지를 확인하세요.")
    for pose in cfg["poses"]:
        print(pose["name"], target_key(pose), pose[target_key(pose)])
    if input("촬영을 시작하려면 Enter (다른 문자 입력 시 취소): ").strip():
        return None
    output = Path(cfg["output_dir"]) / stamp()
    output.mkdir(parents=True, exist_ok=False)
    scan.save_json(output / "settings.json", cfg)
    scan.save_json(output / "robot_settings.json", robot_cfg)
    if "generated_plan" in cfg:
        scan.save_json(output / "pose_plan.json", {**cfg["generated_plan"], "poses": cfg["poses"]})
    source = image_source(cfg["camera"])
    manifest = {"schema": "doosan_eye_in_hand_v1", "status": "capturing", "samples": [],
                "image_source": source, "camera_frame": CAMERA_FRAMES[source]}
    scan.save_json(output / "manifest.json", manifest)
    try:
        camera = CalibrationCamera(cfg["camera"])
        print(f"보정 이미지: {source} / {camera.metadata['width']}×{camera.metadata['height']} / "
              f"{camera.metadata['camera_frame']}")
        scan.save_json(output / "camera.json", camera.metadata)
        with scan.RobotClient(robot_cfg, output) as robot:
            robot.call("initialize", settings=robot_cfg["robot"], **scan.ik_request(robot_cfg, model),
                       timeout=robot_cfg["robot"]["connect_timeout_sec"] + 15)
            for pose in cfg["poses"]:
                if cfg["motion"]["confirm_each_pose"]:
                    if input(f"{pose['name']}로 이동: {pose[target_key(pose)]} / Enter=이동, 다른 입력=종료: ").strip():
                        raise KeyboardInterrupt("사용자 촬영 중단")
                key = target_key(pose)
                settled = robot.move(pose[key]) if key == "joint_deg" else robot.move_cartesian(pose[key])
                before = robot.call("snapshot_cartesian")
                rgb, frame = camera.capture()
                after = robot.call("snapshot_cartesian")
                folder = output / pose["name"]
                folder.mkdir()
                camera.save_capture(folder, rgb)
                record = {"name": pose["name"], "command": pose, "before": before, "after": after,
                          "settled": settled, "frame": frame, "status": "pending",
                          "image_source": source, "camera_frame": CAMERA_FRAMES[source],
                          "calibration_image_file": IMAGE_FILES[source]}
                entry = {"name": pose["name"], "status": "pending"}
                manifest["samples"].append(entry)
                try:
                    tf, drift = validate_capture(before, after, robot_cfg["robot"], cfg["calibration"])
                    observed, overlay = detect_board(rgb, cfg["board"], camera.metadata,
                                                     cfg["calibration"]["max_reprojection_error_px"])
                    save_image(folder / "corners.png", overlay)
                    record.update(observed, T_base_flange=tf.tolist(), capture_drift=drift, status="accepted")
                    entry["status"] = "accepted"
                    print(f"{pose['name']}: 재투영 RMS {observed['reprojection_rms_px']:.3f}px", flush=True)
                except Exception:
                    record.update(status="rejected", error=traceback.format_exc())
                    entry["status"] = "rejected"
                    raise
                finally:
                    scan.save_json(folder / "sample.json", record)
                    scan.save_json(output / "manifest.json", manifest)
        manifest["status"] = "captured"  # 여기서는 로봇 연결/서보가 이미 종료됨
    except BaseException:
        manifest.update(status="capture_failed_or_interrupted", error=traceback.format_exc())
        (output / "error.log").write_text(manifest["error"], encoding="utf-8")
        raise
    finally:
        scan.save_json(output / "manifest.json", manifest)
        print(f"촬영 로그: {output}")
    return output


def preview(cfg):
    board_points(cfg["board"])
    camera = CalibrationCamera(cfg["camera"])
    window = f"Hand-eye {camera.image_source.upper()} ({camera.metadata['camera_frame']})"
    print("카메라만 사용합니다. 로봇 연결/서보 활성화 없음. Q/ESC/창 닫기로 종료.")
    print(f"보정 이미지: {camera.image_source} / {camera.metadata['width']}×{camera.metadata['height']}")
    try:
        while True:
            rgb, _ = camera.capture()
            try:
                observation, overlay = detect_board(rgb, cfg["board"], camera.metadata,
                                                   cfg["calibration"]["max_reprojection_error_px"])
                message = f"RMS {observation['reprojection_rms_px']:.3f} px"
            except ValueError as error:
                overlay = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                message = "Board not accepted (see terminal)"
                print(error)
            cv2.putText(overlay, message, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.namedWindow(window, cv2.WINDOW_NORMAL)
            cv2.imshow(window, overlay)
            if cv2.waitKey(30) & 0xFF in (27, ord('q')) or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                break
    finally:
        cv2.destroyAllWindows()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output-dir", type=Path,
                        help="새 결과 저장 폴더. 재계산 기본: scan_planning/log/hand_eye_calibration")
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--check", action="store_true")
    modes.add_argument("--execute", action="store_true")
    modes.add_argument("--preview", action="store_true")
    modes.add_argument("--solve", type=Path, metavar="SESSION")
    modes.add_argument("--export-depth-variants", type=Path, metavar="LOG",
                       help="Depth TF를 포함한 calibration.json 두 개 저장: 세션/계산 폴더 또는 calibration.json")
    parser.add_argument("--normalize-axis", choices=("x", "y", "z"),
                        help="--export-depth-variants에서 길이를 1로 만들 열 벡터 (기본 x)")
    parser.add_argument("--exclude", nargs="*", default=[], help="--solve에서만 사용: 제외할 pose 이름")
    args = parser.parse_args(argv)
    if args.exclude and args.solve is None:
        parser.error("--exclude는 --solve와 함께 사용하세요.")
    if args.normalize_axis and args.export_depth_variants is None:
        parser.error("--normalize-axis는 --export-depth-variants와 함께 사용하세요.")
    try:
        if args.export_depth_variants:
            export_depth_variants(args.export_depth_variants, args.normalize_axis or "x", args.output_dir)
        elif args.solve:
            solve_session(args.solve, args.exclude, args.output_dir)
        else:
            cfg = load_settings(args.config)
            if args.output_dir is not None:
                cfg["output_dir"] = str(args.output_dir.expanduser().resolve())
            if args.preview:
                preview(cfg)
            else:
                robot_cfg, model = preflight(cfg, require_control=args.execute)
                if args.execute:
                    session = capture_session(cfg, robot_cfg, model)
                    if session is not None:
                        solve_session(session, output_dir=cfg["output_dir"])
        return 0
    except (KeyboardInterrupt, EOFError):
        print("사용자 중단. 연결된 로봇에는 종료/servo-off를 요청합니다. 실제 정지는 현장에서 확인하세요.", file=sys.stderr)
        return 130
    except Exception:
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    def stop(_signum, _frame):
        raise KeyboardInterrupt("종료 신호")
    signal.signal(signal.SIGTERM, stop)
    raise SystemExit(main())
