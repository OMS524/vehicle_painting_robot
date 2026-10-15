#!/usr/bin/env python3
"""A0912 + Femto Bolt eye-in-hand: SDK 이동/실측 자세, RGB 체커보드, OpenCV.

기본/--check: 하드웨어 연결 없는 검사. --preview: 카메라만 사용.
--execute: 기준 자세로 생성한 촬영 자세에서 촬영 후 계산. --solve SESSION: 저장 데이터로 재계산.
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
SCAN_SCRIPTS = HERE.parents[1] / "doosan_a0912_scan_test/scripts"
sys.path.insert(0, str(SCAN_SCRIPTS))
import multi_view_scan as scan
from cartesian_scan import pose_matrix, sdk_pose_matrix, target_key, validate_targets, vector6

DEFAULT_CONFIG = HERE / "hand_eye_calibration.yaml"
METHODS = {name: getattr(cv2, "CALIB_HAND_EYE_" + name)
           for name in ("PARK", "TSAI", "HORAUD", "ANDREFF", "DANIILIDIS")}


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


def preflight(cfg):
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
    for key in ("control_python", "control_wrapper", "control_library"):
        if not Path(robot_cfg[key]).is_file():
            raise FileNotFoundError(f"{key}: {robot_cfg[key]}")
    for pose in cfg["poses"]:
        if target_key(pose) == "joint_deg" and pose["joint_deg"] is not None:
            model.transforms(pose["joint_deg"])
    result = subprocess.run(scan.worker_command(robot_cfg, "--check"),
        env=scan.worker_environment(robot_cfg["control_python"]),
        input=json.dumps({"model_xml": model.xml, "cartesian": robot_cfg["cartesian"]}),
        text=True, capture_output=True, timeout=30)
    if result.returncode:
        raise RuntimeError("제어 환경 검사 실패:\n" + result.stdout + result.stderr)
    import pyorbbecsdk  # noqa: F401 -- Context 생성/장치 연결 없음
    print("파일·SDK 연결부 검사 완료. 로봇/카메라에 연결하지 않았습니다.")
    print(f"제어 Python: {robot_cfg['control_python']}\n촬영 자세: {len(cfg['poses'])}개")
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
        raise ValueError(f"지원하지 않는 RGB 왜곡 모델: {model}. pinhole PnP에 그대로 넣지 않습니다.")
    k = [[intrinsic.fx, 0, intrinsic.cx], [0, intrinsic.fy, intrinsic.cy], [0, 0, 1]]
    # OpenCV rational model: k1,k2,p1,p2,k3,k4,k5,k6 (SDK 필드 순서와 다름).
    d = [distortion.k1, distortion.k2, distortion.p1, distortion.p2, distortion.k3]
    if model.endswith(".BROWN_CONRADY_K6"):
        d += [distortion.k4, distortion.k5, distortion.k6]
    if not np.isfinite(k).all() or not np.isfinite(d).all() or intrinsic.fx <= 0 or intrinsic.fy <= 0:
        raise ValueError("SDK RGB 내부 파라미터가 유효하지 않습니다.")
    return {"K": k, "dist_coeffs": d, "distortion_model": model,
            "width": profile.get_width(), "height": profile.get_height(), "fps": profile.get_fps()}


class ColorCamera:
    """원본 RGB만 촬영. 깊이 정렬/Depth 품질에 체커보드 검출을 의존하지 않음."""
    def __init__(self, settings):
        import pyorbbecsdk as sdk
        self.sdk, self.settings = sdk, settings
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
        self.metadata = camera_matrix(profile)
        self.metadata["device"] = {"name": info.get_name(), "serial": info.get_serial_number(),
                                   "firmware": info.get_firmware_version()}
        # Optional factory Depth->RGB transform. RGB hand-eye remains usable if this is unavailable.
        try:
            dp = self.pipeline.get_stream_profile_list(sdk.OBSensorType.DEPTH_SENSOR).get_video_stream_profile(
                0, 0, sdk.OBFormat.Y16, settings["fps"])
            ex = dp.get_extrinsic_to(profile)
            raw = np.eye(4)
            raw[:3, :3], raw[:3, 3] = np.asarray(ex.rot).reshape(3, 3), np.asarray(ex.transform) / 1000
            self.metadata["T_color_depth_sdk_raw"] = raw.tolist()
            self.metadata["T_color_depth"] = validate_transform(raw).tolist()
        except Exception as error:
            self.metadata["depth_extrinsic_warning"] = str(error)
        self.config = sdk.Config()
        self.config.enable_stream(profile)

    def capture(self):
        # 매 자세마다 새 스트림을 시작하므로 이전 이동 중 버퍼의 이미지를 사용하지 않는다.
        deadline = time.monotonic() + self.settings["capture_timeout_sec"]
        count, previous = 0, None
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
                previous = identity
                count += 1
                if count <= self.settings["warmup_frames"]:
                    continue
                rgb = scan.rgb_image(self.sdk, scan.color_to_rgb_frame(self.sdk, frame))
                return rgb, {"frame_index": identity[0], "device_timestamp_us": identity[1],
                             "host_received_ns": time.time_ns(), "warmup_frames": count - 1}
            raise TimeoutError("새 RGB 프레임을 제한 시간 안에 취득하지 못했습니다.")
        finally:
            self.pipeline.stop()


def board_points(board):
    if board["square_size_mm"] is None:
        raise ValueError("board.square_size_mm를 입력하세요.")
    points = np.zeros((np.prod(board["inner_corners"]), 3), dtype=np.float64)
    points[:, :2] = np.mgrid[0:board["inner_corners"][0], 0:board["inner_corners"][1]].T.reshape(-1, 2)
    return points * board["square_size_mm"] / 1000


def detect_board(rgb, board, camera, max_error):
    if rgb.shape[:2] != (camera["height"], camera["width"]):
        raise ValueError("원본 RGB 해상도와 내부 파라미터가 다릅니다.")
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
    return {"T_color_board": t.tolist(), "reprojection_rms_px": error,
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


def solve_samples(samples, cal):
    if len(samples) < cal["min_samples"]:
        raise ValueError(f"유효 시점 {len(samples)}개 < min_samples {cal['min_samples']}")
    grippers = [validate_transform(s["T_base_flange"]) for s in samples]
    targets = [validate_transform(s["T_color_board"]) for s in samples]
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
    return {"T_flange_color": x.tolist(), "T_base_board_mean": center.tolist(),
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


def solve_session(session, excluded=()):
    session = Path(session).expanduser().resolve()
    manifest, cfg = read_json(session / "manifest.json"), read_json(session / "settings.json")
    if manifest.get("schema") != "doosan_eye_in_hand_v1":
        raise ValueError("이 도구로 촬영한 eye-in-hand 세션이 아닙니다.")
    validate_settings(cfg)
    known = {s["name"] for s in manifest["samples"]}
    if set(excluded) - known:
        raise ValueError(f"존재하지 않는 제외 시점: {set(excluded) - known}")
    output = session / ("calibration_" + stamp())
    output.mkdir()
    report = {"status": "running", "method": cfg["calibration"]["method"], "source_session": str(session),
              "excluded": list(excluded), "translation_unit": "m", "opencv_version": cv2.__version__,
              "transform_convention": "T_parent_child maps child coordinates to parent coordinates",
              "robot_pose_source": "Doosan SDK measured base->flange, mm/Euler ZYZ converted to m",
              "applied_to_robot_or_xacro": False, "samples": []}
    try:
        camera = read_json(session / "camera.json")
        robot_settings = read_json(session / "robot_settings.json")["robot"]
        for entry in manifest["samples"]:
            if entry["name"] in excluded or entry.get("status") != "accepted":
                continue
            folder = (session / entry["name"]).resolve()
            if folder.parent != session:
                raise ValueError("시점 경로가 세션 밖을 가리킵니다.")
            record = read_json(folder / "sample.json")
            _, drift = validate_capture(record["before"], record["after"],
                                        robot_settings, cfg["calibration"])
            rgb = cv2.imread(str(folder / "color_rgb.png"))
            if rgb is None:
                raise ValueError(f"RGB 파일 읽기 실패: {folder}")
            observation, overlay = detect_board(cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB), cfg["board"], camera,
                                               cfg["calibration"]["max_reprojection_error_px"])
            sample = {"name": entry["name"], **observation, "capture_drift": drift,
                      "T_base_flange": sdk_pose_matrix(record["after"]["flange_pose_mm_zyz_deg"]).tolist()}
            report["samples"].append(sample)
            save_image(output / (entry["name"] + "_corners.png"), overlay)
        report.update(solve_samples(report["samples"], cfg["calibration"]))
        if "T_color_depth" in camera:
            report["T_flange_depth"] = (np.asarray(report["T_flange_color"]) @
                                         validate_transform(camera["T_color_depth"])).tolist()
        else:
            report["warnings"].append("Depth TF 미출력: SDK Depth->RGB 외부 파라미터 없음/검증 실패: "
                                      + camera.get("depth_extrinsic_warning", "unknown"))
        report["status"] = "computed_requires_physical_validation"
        report["notes"] = ["표본 내 보정판 일관성과 leave-one-out 안정성은 절대 정확도 보증이 아닙니다.",
                           "체커보드 코너 원점이 모든 이미지에서 같은 물리적 모서리인지 확인하세요.",
                           "SDK 플랜지 기준 보정입니다. URDF FK 기반 스캔에 적용할 때 기구학 차이를 검증하세요.",
                           "T_flange_color는 RGB 광학 프레임. Depth는 SDK Depth->RGB TF가 유효할 때만 계산합니다."]
        print("T_flange_color (위치 m):\n", np.asarray(report["T_flange_color"]))
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
    manifest = {"schema": "doosan_eye_in_hand_v1", "status": "capturing", "samples": []}
    scan.save_json(output / "manifest.json", manifest)
    try:
        camera = ColorCamera(cfg["camera"])
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
                save_image(folder / "color_rgb.png", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
                record = {"name": pose["name"], "command": pose, "before": before, "after": after,
                          "settled": settled, "frame": frame, "status": "pending"}
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
    camera = ColorCamera(cfg["camera"])
    print("카메라만 사용합니다. 로봇 연결/서보 활성화 없음. Q/ESC/창 닫기로 종료.")
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
            cv2.namedWindow("Hand-eye RGB", cv2.WINDOW_NORMAL)
            cv2.imshow("Hand-eye RGB", overlay)
            if cv2.waitKey(30) & 0xFF in (27, ord('q')) or cv2.getWindowProperty("Hand-eye RGB", cv2.WND_PROP_VISIBLE) < 1:
                break
    finally:
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--check", action="store_true")
    modes.add_argument("--execute", action="store_true")
    modes.add_argument("--preview", action="store_true")
    modes.add_argument("--solve", type=Path, metavar="SESSION")
    parser.add_argument("--exclude", nargs="*", default=[], help="--solve에서만 사용: 제외할 pose 이름")
    args = parser.parse_args()
    if args.exclude and args.solve is None:
        parser.error("--exclude는 --solve와 함께 사용하세요.")
    try:
        if args.solve:
            solve_session(args.solve, args.exclude)
        else:
            cfg = load_settings(args.config)
            if args.preview:
                preview(cfg)
            else:
                robot_cfg, model = preflight(cfg)
                if args.execute:
                    session = capture_session(cfg, robot_cfg, model)
                    if session is not None:
                        solve_session(session)
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
