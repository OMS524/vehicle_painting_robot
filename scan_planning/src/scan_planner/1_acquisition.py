"""1단계: 로봇 이동·RGB/Depth 취득·필터·촬영 TF·뷰별 저장.

장비 연결은 prepare_capture 검사 이후 acquire를 명시적으로 호출할 때만 수행한다.
"""
from pathlib import Path
import numpy as np
import math
import xml.etree.ElementTree as ET
import hashlib
import json
from contextlib import AbstractContextManager
import os
import queue
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
import copy
import colorsys
from datetime import datetime, timezone
import importlib.metadata
import traceback


# ========================================================================
# 설정 읽기·검증
# ========================================================================

def finite_vector(values, count, label):
    if not isinstance(values, list) or len(values) != count or any(
            isinstance(x, bool) or not isinstance(x, (int, float)) for x in values):
        raise ValueError(f"{label}: 숫자 {count}개를 입력하세요.")
    result = np.asarray(values, dtype=float)
    if not np.isfinite(result).all():
        raise ValueError(f"{label}: NaN/inf는 사용할 수 없습니다.")
    return result


def load_config(path):
    import yaml

    path = Path(path).expanduser().resolve()
    cfg = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(cfg, dict):
        raise ValueError("scan 설정은 YAML 객체여야 합니다.")
    camera = cfg["camera"]
    camera.setdefault("capture_color", False)
    camera.setdefault("color_width", 0)
    camera.setdefault("color_height", 0)
    camera.setdefault("color_max_time_delta_ms", 50.0)
    camera.setdefault("capture_frames", 30)
    camera.setdefault("min_valid_frames", 3)
    if type(camera["capture_color"]) is not bool:
        raise ValueError("camera.capture_color는 true/false여야 합니다.")
    for name in ("control_python", "control_wrapper", "control_library", "xacro", "output_dir"):
        if cfg.get(name) is None and name.startswith("control_"):
            continue  # 새 제어 연결부가 지정될 때까지 실장비 실행을 차단한다.
        value = Path(cfg[name]).expanduser()
        cfg[name] = str((path.parent / value).resolve() if not value.is_absolute() else value.resolve())
    views = cfg.get("views")
    if not isinstance(views, list) or not views:
        raise ValueError("views는 촬영 시점이 1개 이상인 목록이어야 합니다.")
    view_names = set()
    for index, view in enumerate(views, start=1):
        if not isinstance(view, dict):
            raise ValueError(f"views의 {index}번째 항목에 name과 목표 자세를 지정하세요.")
        name = view.get("name")
        # 이름은 결과 폴더명으로 사용한다. 경로와 세션 파일 이름 충돌을 차단한다.
        if (not isinstance(name, str) or not name or len(name.encode("utf-8")) > 255
                or not all(char.isalnum() or char in "_-" for char in name)):
            raise ValueError(f"views의 {index}번째 name은 문자·숫자·밑줄(_)·하이픈(-)으로 "
                             "된 255바이트 이내의 비어 있지 않은 이름이어야 합니다.")
        if name in view_names:
            raise ValueError(f"views의 name이 중복됩니다: {name}. 시점마다 고유한 이름을 지정하세요.")
        view_names.add(name)
    validate_targets(cfg)
    for group, names in {
        "robot": ("connect_timeout_sec", "move_timeout_sec", "max_velocity_deg_s", "acceleration_deg_s2",
                  "target_tolerance_deg", "command_time_sec", "max_move_delta_deg", "settle_sec",
                  "settle_timeout_sec", "stationary_velocity_deg_s", "capture_drift_deg"),
        "camera": ("warmup_frames", "capture_frames", "min_valid_frames", "capture_timeout_sec",
                   "min_depth_mm", "max_depth_mm", "min_points", "color_max_time_delta_ms"),
        "reconstruction": ("voxel_size_mm",),
    }.items():
        for name in names:
            value = cfg[group][name]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value) or value <= 0:
                raise ValueError(f"{group}.{name}은 양의 유한한 숫자여야 합니다.")
    for group, names in {"robot": ("port", "rt_port", "connect_timeout_sec"),
                         "camera": ("width", "height", "fps", "warmup_frames", "capture_frames",
                                    "min_valid_frames", "min_points", "color_width", "color_height")}.items():
        for name in names:
            value = cfg[group][name]
            if type(value) is not int or value < 0:
                raise ValueError(f"{group}.{name}은 0 이상의 정수여야 합니다.")
    if camera["min_valid_frames"] > camera["capture_frames"]:
        raise ValueError("camera.min_valid_frames는 camera.capture_frames 이하여야 합니다.")
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
    configure_pose_source(cfg, path)
    return cfg


# ========================================================================
# URDF 좌표 변환
# ========================================================================

def vector(text, size=3):
    values = np.array([float(value) for value in text.split()])
    if values.shape != (size,) or not np.isfinite(values).all():
        raise ValueError(f"유효한 {size}개 숫자가 필요합니다: {text!r}")
    return values


def origin_matrix(origin):
    """URDF 고정축 RPY: Rz(yaw) @ Ry(pitch) @ Rx(roll), 위치는 m."""
    transform = np.eye(4)
    if origin is None:
        return transform
    roll, pitch, yaw = vector(origin.get("rpy", "0 0 0"))
    sr, sp, sy = np.sin([roll, pitch, yaw])
    cr, cp, cy = np.cos([roll, pitch, yaw])
    transform[:3, :3] = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    transform[:3, 3] = vector(origin.get("xyz", "0 0 0"))
    return transform


def joint_transform(joint, angle_deg=0.0):
    """부모 -> 관절 원점 -> 관절 로컬 축 회전을 합성한다."""
    name, kind = joint.get("name"), joint.get("type")
    transform = origin_matrix(joint.find("origin"))
    if kind == "fixed":
        return transform
    if kind not in ("revolute", "continuous"):
        raise ValueError(f"지원하지 않는 조인트 형식: {name} ({kind})")
    if joint.find("mimic") is not None:
        raise ValueError(f"mimic 조인트는 아직 지원하지 않습니다: {name}")
    angle_deg = float(angle_deg)
    if not np.isfinite(angle_deg):
        raise ValueError(f"관절각은 유한한 deg 값이어야 합니다: {name}")
    angle = np.deg2rad(angle_deg)
    limit = joint.find("limit")
    if kind == "revolute" and limit is not None:
        lower = float(limit.get("lower", "-inf"))
        upper = float(limit.get("upper", "inf"))
        if not lower <= angle <= upper:
            raise ValueError(
                f"관절각 범위 초과: {name}={angle_deg:g} deg "
                f"(허용: {np.rad2deg(lower):g} ~ {np.rad2deg(upper):g} deg)")
    axis_element = joint.find("axis")
    axis = vector(axis_element.get("xyz", "1 0 0") if axis_element is not None else "1 0 0")
    length = np.linalg.norm(axis)
    if not np.isfinite(length) or length < 1e-12:
        raise ValueError(f"관절 회전축은 0이 아닌 유한한 벡터여야 합니다: {name}")
    axis /= length
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    motion = np.eye(4)
    motion[:3, :3] = (np.cos(angle) * np.eye(3)
                      + (1 - np.cos(angle)) * np.outer(axis, axis)
                      + np.sin(angle) * skew)
    return transform @ motion


def link_transforms(robot, joint_angles_deg=None):
    """루트에서 각 링크까지의 변환을 계산한다. 잘못된 연결은 오류 처리."""
    links = {link.attrib["name"]: link for link in robot.findall("link")}
    if not links or len(links) != len(robot.findall("link")):
        raise ValueError("링크가 없거나 링크 이름이 중복됩니다. 매크로 호출이 있는 Xacro를 지정하세요.")
    joints = robot.findall("joint")
    names = [joint.get("name") for joint in joints]
    if any(not name for name in names) or len(names) != len(set(names)):
        raise ValueError("조인트 이름이 없거나 중복됩니다.")
    angles = {} if joint_angles_deg is None else joint_angles_deg
    movable = {joint.get("name") for joint in joints
               if joint.get("type") in ("revolute", "continuous")}
    unknown = set(angles) - movable
    if unknown:
        raise ValueError(f"관절각을 지정한 회전 조인트를 찾을 수 없습니다: {sorted(unknown)}")
    children, parents = {}, {}
    for joint in joints:
        parent = joint.find("parent").attrib["link"]
        child = joint.find("child").attrib["link"]
        if parent not in links or child not in links or child in parents:
            raise ValueError(f"잘못된 링크 연결 또는 부모 중복: {parent} -> {child}")
        parents[child] = parent
        local = joint_transform(joint, angles.get(joint.get("name"), 0.0))
        children.setdefault(parent, []).append((child, local))
    roots = set(links) - set(parents)
    if len(roots) != 1:
        raise ValueError(f"루트 링크가 하나여야 합니다: {sorted(roots)}")
    root = roots.pop()
    transforms = {root: np.eye(4)}
    queue = [root]
    for parent in queue:
        for child, local in children.get(parent, []):
            if child in transforms:
                raise ValueError(f"조인트 순환 연결: {child}")
            transforms[child] = transforms[parent] @ local
            queue.append(child)
    if len(transforms) != len(links):
        raise ValueError("루트에 연결되지 않았거나 순환 연결된 링크가 있습니다.")
    return root, links, transforms


# ========================================================================
# Cartesian 목표와 SDK IK
# ========================================================================

TARGET_FRAMES = ("link_6", "bracket_link", "camera_depth_optical_frame")
RPY_CONVENTION = "fixed_XYZ: Rz(yaw) @ Ry(pitch) @ Rx(roll)"
DEFAULTS = {"target_frame": "link_6"}


def vector6(values, label):
    if (not isinstance(values, list) or len(values) != 6
            or any(isinstance(v, bool) or not isinstance(v, (int, float))
                   or not math.isfinite(v) for v in values)):
        raise ValueError(f"{label}: 유한한 숫자 6개를 입력하세요.")
    return np.asarray(values, dtype=float)


def target_key(view):
    keys = [key for key in ("joint_deg", "pose_mm_deg") if key in view]
    if len(keys) != 1:
        raise ValueError(f"views.{view.get('name', '?')}: joint_deg 또는 pose_mm_deg 중 하나만 지정하세요.")
    return keys[0]


def validate_targets(cfg):
    settings = cfg.setdefault("cartesian", {})
    if not isinstance(settings, dict):
        raise ValueError("cartesian은 설정 항목이어야 합니다.")
    for key, value in DEFAULTS.items():
        settings.setdefault(key, value)
    if settings["target_frame"] not in TARGET_FRAMES:
        raise ValueError(f"cartesian.target_frame: {TARGET_FRAMES} 중 선택하세요.")
    for view in cfg["views"]:
        key = target_key(view)
        if view[key] is not None:
            vector6(view[key], f"views.{view['name']}.{key}")


def pose_matrix(values):
    """base_link 기준 목표 링크의 [mm, mm, mm, deg, deg, deg] → SE(3)."""
    values = vector6(values, "pose_mm_deg")
    return origin_matrix(ET.Element("origin", {
        "xyz": " ".join(map(str, values[:3] / 1000.0)),
        "rpy": " ".join(map(str, np.deg2rad(values[3:])))}))


def target_description(view, cfg):
    key = target_key(view)
    result = {"target_kind": key, key: view[key]}
    if key == "pose_mm_deg":
        result.update(reference_frame="base_link", target_frame=cfg["cartesian"]["target_frame"],
                      rpy_convention=RPY_CONVENTION)
    return result


def cartesian_error(view, cfg, frames):
    """촬영 후 선택한 TF로 계산한 실측 자세와 목표의 오차. 진단용이며 촬영 허용 기준은 아님."""
    if target_key(view) != "pose_mm_deg":
        return {}
    target = pose_matrix(view["pose_mm_deg"])
    actual = frames[cfg["cartesian"]["target_frame"]]
    cosine = np.clip((np.trace(target[:3, :3] @ actual[:3, :3].T) - 1) / 2, -1, 1)
    return {"actual_cartesian_position_error_mm": float(np.linalg.norm(target[:3, 3] - actual[:3, 3]) * 1000),
            "actual_cartesian_orientation_error_deg": float(np.rad2deg(np.arccos(cosine)))}


def sdk_pose_matrix(values):
    """두산 SDK의 [mm, mm, mm, A, B, C(deg)] / Euler ZYZ → SE(3)."""
    values = vector6(values, "SDK pose_mm_zyz_deg")
    a, b, c = np.deg2rad(values[3:])
    ca, cb, cc = np.cos([a, b, c])
    sa, sb, sc = np.sin([a, b, c])
    transform = np.eye(4)
    transform[:3, :3] = [
        [ca * cb * cc - sa * sc, -ca * cb * sc - sa * cc, ca * sb],
        [sa * cb * cc + ca * sc, -sa * cb * sc + ca * cc, sa * sb],
        [-sb * cc, sb * sc, cb],
    ]
    transform[:3, 3] = values[:3] / 1000.0
    return transform


def validate_transform(value):
    t = np.asarray(value, dtype=float)
    if (t.shape != (4, 4) or not np.isfinite(t).all()
            or not np.allclose(t[3], [0, 0, 0, 1], atol=1e-8, rtol=0)
            or not np.allclose(t[:3, :3].T @ t[:3, :3], np.eye(3), atol=1e-4, rtol=0)
            or abs(np.linalg.det(t[:3, :3]) - 1) > 1e-4):
        raise ValueError("유효한 4×4 SE(3) TF가 아닙니다. 위치 단위는 m여야 합니다.")
    return t


def matrix_to_sdk_pose(transform):
    """SE(3) → mm / Euler ZYZ. B=0,180도에서도 동등한 회전을 반환한다."""
    r = transform[:3, :3]
    sin_b = math.hypot(r[0, 2], r[1, 2])
    b = math.atan2(sin_b, r[2, 2])
    if sin_b > 1e-10:
        a, c = math.atan2(r[1, 2], r[0, 2]), math.atan2(r[2, 1], -r[2, 0])
    elif r[2, 2] >= 0:
        a, c = math.atan2(r[1, 0], r[0, 0]), 0.0
    else:
        a, c = math.atan2(-r[1, 0], -r[0, 0]), 0.0
    return np.concatenate((transform[:3, 3] * 1000.0, np.rad2deg([a, b, c]))).tolist()


class DoosanSDKIK:
    """플랜지 이후 URDF/보정 툴 TF를 적용하고 관절 해는 SDK에서 구한다."""
    def __init__(self, model_xml, settings, flange_target=None):
        cfg = {"cartesian": dict(settings), "views": []}
        validate_targets(cfg)
        self.settings = cfg["cartesian"]
        model = ET.fromstring(model_xml)
        parents = {joint.find("child").get("link"): joint for joint in model.findall("joint")}
        links = {link.get("name") for link in model.findall("link")}
        child = self.settings["target_frame"]
        if "link_6" not in links or child not in links:
            raise ValueError("SDK IK 모델에 플랜지 또는 목표 프레임이 없습니다.")
        self.flange_target = np.eye(4)
        visited = set()
        while child != "link_6":
            if child in visited or child not in parents or parents[child].get("type") != "fixed":
                raise ValueError("SDK IK 목표 프레임은 link_6 뒤의 고정 툴이어야 합니다.")
            visited.add(child)
            joint = parents[child]
            self.flange_target = origin_matrix(joint.find("origin")) @ self.flange_target
            child = joint.find("parent").get("link")
        self.tool_tf_source = "urdf"
        if flange_target is not None:
            if self.settings["target_frame"] != "camera_depth_optical_frame":
                raise ValueError("핸드아이 툴 TF는 Depth 광학 프레임 목표에만 적용할 수 있습니다.")
            self.flange_target = validate_transform(flange_target)
            self.tool_tf_source = "hand_eye"
        self.target_flange = np.linalg.inv(self.flange_target)

    def solve(self, robot, values, seed_deg):
        desired = pose_matrix(values)
        reference = vector6(seed_deg, "현재 실측 joint_deg").tolist()
        # SDK IK uses the controller's active TCP. Derive its fixed flange offset
        # from two SDK poses at rest; do not change the pendant's TCP selection.
        current = robot.read_actual_cartesian_state()
        base_flange = sdk_pose_matrix(current["flange_pose_mm_zyz_deg"])
        base_tcp = sdk_pose_matrix(current["tcp_pose_mm_zyz_deg"])
        flange_tcp = np.linalg.inv(base_flange) @ base_tcp
        desired_tcp = desired @ self.target_flange @ flange_tcp
        sdk_pose = matrix_to_sdk_pose(desired_tcp)
        solved = robot.solve_closest_ik(sdk_pose, reference)
        target = vector6(solved["target_joint_deg"], "SDK IK target_joint_deg").tolist()
        return {"target_joint_deg": target,
                "ik": {"method": "doosan_sdk_ikin", "seed_joint_deg": reference,
                       "solution_space": solved["solution_space"],
                       "target_frame": self.settings["target_frame"], "reference_frame": "base_link",
                       "rpy_convention": RPY_CONVENTION, "target_pose_mm_deg": list(values),
                       "T_base_target": desired.tolist(), "T_flange_active_tcp": flange_tcp.tolist(),
                       "tool_tf_source": self.tool_tf_source, "T_flange_target": self.flange_target.tolist(),
                       "sdk_target_pose_mm_zyz_deg": sdk_pose}}


# ========================================================================
# 촬영 자세·핸드아이 TF
# ========================================================================

DEPTH_FRAME = "camera_depth_optical_frame"
SDK_POSE_SOURCE = "Doosan SDK measured base->flange, mm/Euler ZYZ converted to m"


def configure_pose_source(cfg, config_path):
    settings = cfg.setdefault("pose_estimation", {})
    if not isinstance(settings, dict):
        raise ValueError("pose_estimation은 source와 hand_eye_calibration_file을 가진 설정이어야 합니다.")
    settings.setdefault("source", "urdf")
    settings.setdefault("hand_eye_calibration_file", None)
    if settings["source"] not in ("urdf", "hand_eye"):
        raise ValueError("pose_estimation.source는 urdf / hand_eye만 가능합니다.")
    value = settings["hand_eye_calibration_file"]
    if value is not None:
        if not isinstance(value, str) or not value.strip():
            raise ValueError("hand_eye_calibration_file은 JSON 파일 경로 또는 null이어야 합니다.")
        path = Path(value).expanduser()
        settings["hand_eye_calibration_file"] = str(
            (Path(config_path).parent / path).resolve() if not path.is_absolute() else path.resolve())


class ScanPoseSource:
    def __init__(self, cfg, model):
        self.source = cfg.get("pose_estimation", {}).get("source", "urdf")
        if self.source not in ("urdf", "hand_eye"):
            raise ValueError("pose_estimation.source는 urdf / hand_eye만 가능합니다.")
        self.model = model
        nominal = model.transforms([0.0] * 6)
        inv_flange = np.linalg.inv(nominal["link_6"])
        self.flange_bracket = validate_transform(inv_flange @ nominal["bracket_link"])
        self.flange_depth = validate_transform(inv_flange @ nominal[DEPTH_FRAME])
        self.metadata = {"source": self.source}
        self.report_bytes = None
        if self.source == "hand_eye":
            filename = cfg["pose_estimation"].get("hand_eye_calibration_file")
            if not filename:
                raise ValueError("hand_eye 모드에는 pose_estimation.hand_eye_calibration_file이 필요합니다.")
            self.report_bytes = Path(filename).read_bytes()
            report = json.loads(self.report_bytes)
            if not isinstance(report, dict):
                raise ValueError("핸드아이 결과는 calibration.json 객체여야 합니다.")
            if (report.get("status") != "computed_requires_physical_validation"
                    or report.get("quality_checks_passed") is not True):
                raise ValueError("핸드아이 계산 완료 및 품질 검사 통과 결과가 필요합니다. 실패/미완료 결과는 사용하지 않습니다.")
            if report.get("translation_unit") != "m" or report.get("robot_pose_source") != SDK_POSE_SOURCE:
                raise ValueError("SDK 실측 플랜지 기준·위치 단위 m인 핸드아이 결과가 필요합니다.")
            if "T_flange_depth" not in report:
                raise ValueError("핸드아이 결과에 T_flange_depth가 없습니다. RGB 결과만으로 Depth TF를 대체하지 않습니다.")
            # 점군에는 파일의 행렬을 그대로 적용한다(원본 SDK의 축 길이 오차도 비교 가능).
            # 로봇 IK 툴 TF에 대한 SE(3) 검사는 cartesian_scan에서 별도로 수행한다.
            tf = np.asarray(report["T_flange_depth"], dtype=float)
            if (tf.shape != (4, 4) or not np.isfinite(tf).all()
                    or not np.allclose(tf[3], [0, 0, 0, 1], atol=1e-8, rtol=0)):
                raise ValueError("T_flange_depth는 유한한 4×4 동차행렬이어야 합니다.")
            self.flange_depth = tf
            self.metadata.update(hand_eye_calibration_file=str(filename),
                source_sha256=hashlib.sha256(self.report_bytes).hexdigest(),
                calibration_method=report.get("method"),
                T_flange_depth=self.flange_depth.tolist())

    @property
    def method(self):
        return ("actual_joint_fk_and_xacro_tf_only" if self.source == "urdf"
                else "actual_sdk_flange_and_hand_eye_tf_only")

    @property
    def pose_source(self):
        return ("actual_joint_measurement_after_capture_plus_URDF_FK" if self.source == "urdf"
                else "actual_sdk_flange_after_capture_plus_hand_eye")

    def snapshot(self, robot):
        if self.source == "urdf":
            return robot.snapshot()
        state = robot.call("snapshot_cartesian")
        # 누락/비유한 SDK 자세를 URDF로 대체하지 않고 촬영을 중단한다.
        sdk_pose_matrix(state["flange_pose_mm_zyz_deg"])
        return state

    def frames(self, state):
        if self.source == "urdf":
            return self.model.transforms(state["joint_deg"])
        flange = sdk_pose_matrix(state["flange_pose_mm_zyz_deg"])
        return {"link_6": flange, "bracket_link": flange @ self.flange_bracket,
                DEPTH_FRAME: flange @ self.flange_depth}

    def save_report(self, output):
        if self.report_bytes is not None:
            # 선택한 원본과 동일한 바이트를 보존. 세션 중 원본이 수정돼도 TF는 바뀌지 않는다.
            (Path(output) / "hand_eye_calibration.json").write_bytes(self.report_bytes)

    def describe(self):
        print(f"정합 TF 모드: {self.source}")
        if self.source == "hand_eye":
            print("SDK 실측 base→flange × 보정 T_flange_depth 사용:",
                  self.metadata["hand_eye_calibration_file"])
            print("보정값의 카메라/브라켓 장착 상태가 현재와 같은지 확인하세요. 품질 검사는 실물 정확도 보증이 아닙니다.")
        else:
            print("실측 관절각 + URDF FK/고정 장착 TF 사용")
        print("T_flange_depth (Depth 좌표를 플랜지 좌표로 변환, 위치 m):")
        print(self.flange_depth)


# ========================================================================
# 로봇 제어 프로세스 연결
# ========================================================================

HERE = Path(__file__).resolve().parent
JOINT_NAMES = tuple(f"joint_{i}" for i in range(1, 7))
BASE_FRAME = "base_link"

class IKNoSolutionError(RuntimeError):
    """Explicit, recoverable no-solution response from the control worker."""
    def __init__(self, message, sdk_status):
        super().__init__(message)
        self.sdk_status = sdk_status

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
    return [cfg["control_python"], "-u", str(HERE / "robot_worker.py"),
            "--wrapper", cfg["control_wrapper"], "--library", cfg["control_library"], mode]


def ik_request(cfg, model, pose_source=None):
    if any(target_key(view) == "pose_mm_deg" for view in cfg["views"]):
        request = {"model_xml": model.xml, "cartesian": cfg["cartesian"]}
        if (cfg["cartesian"]["target_frame"] == DEPTH_FRAME
                and cfg.get("pose_estimation", {}).get("source", "urdf") == "hand_eye"):
            pose_source = pose_source or ScanPoseSource(cfg, model)
            request["T_flange_target"] = pose_source.flange_depth.tolist()
        return request
    return {}

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
            if reply.get("error_code") == "ik_no_solution" and reply.get("recoverable") is True:
                raise IKNoSolutionError(reply["error"], reply.get("sdk_status"))
            raise RuntimeError(reply["error"])
        return reply["result"]

    def move(self, joints):
        return self.call("move", joint_deg=joints,
                         timeout=self.cfg["robot"]["move_timeout_sec"] + self.cfg["robot"]["settle_timeout_sec"] + 15)

    def move_cartesian(self, pose, *, recover_ik_failure=False):
        return self.call("move_cartesian", pose_mm_deg=pose, recover_ik_failure=recover_ik_failure,
                         timeout=self.cfg["robot"]["move_timeout_sec"] + self.cfg["robot"]["settle_timeout_sec"] + 30)

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


# ========================================================================
# 카메라·RGB/Depth·중앙값 취득
# ========================================================================

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
    depth_frames: np.ndarray | None = None
    depth_valid_counts: np.ndarray | None = None


def fuse_depth_frames(depth_frames, min_valid_frames=3):
    """같은 자세의 Y16 프레임에서 0을 제외한 픽셀별 하위 중앙값과 유효 관측 수.

    짝수 개일 때도 두 값의 평균 대신 실제 측정값 하나를 선택한다.
    min_valid_frames회 미만 측정된 픽셀은 0이며 주변 픽셀 보간은 하지 않는다.
    """
    stack = np.asarray(depth_frames)
    if stack.dtype != np.uint16 or stack.ndim != 3 or min(stack.shape) == 0:
        raise ValueError("Depth 통합 입력은 비어 있지 않은 uint16 (프레임, 높이, 너비) 배열이어야 합니다.")
    if type(min_valid_frames) is not int or not 1 <= min_valid_frames <= stack.shape[0]:
        raise ValueError("min_valid_frames는 1 이상 수집 프레임 수 이하의 정수여야 합니다.")
    counts = np.count_nonzero(stack, axis=0).astype(np.uint32)
    ordered = np.sort(np.where(stack > 0, stack, np.iinfo(np.uint16).max), axis=0)
    rank = np.maximum(counts.astype(np.int64) - 1, 0) // 2
    fused = np.take_along_axis(ordered, rank[None], axis=0)[0]
    fused[counts < min_valid_frames] = 0
    return fused, counts


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
        capture_frames = settings.get("capture_frames", 30)
        min_valid_frames = settings.get("min_valid_frames", 3)
        # 각 이동이 끝난 뒤 새 스트림 시작. 이동 중 쌓인 이전 프레임 사용 방지.
        self.pipeline.start(self.config)
        try:
            deadline = time.monotonic() + settings["capture_timeout_sec"]
            count, seen_frames = 0, set()
            samples, source_frames = [], []
            sample_scale = None
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
                scale = float(depth.get_depth_scale())
                if not np.isfinite(scale) or scale <= 0:
                    raise ValueError("잘못된 Depth scale")
                if samples and (raw.shape != samples[0].shape or scale != sample_scale):
                    raise ValueError("촬영 중 Depth 해상도 또는 scale이 변경되었습니다. 서로 다른 측정값은 통합하지 않습니다.")
                sample_scale = scale
                samples.append(raw)
                source = {"frame_index": depth.get_index(), "device_timestamp_us": depth.get_timestamp_us(),
                          "sdk_system_timestamp_us": depth.get_system_timestamp_us(), "host_received_ns": received_ns}
                if color is not None:
                    source.update(color_frame_index=color.get_index(), color_device_timestamp_us=color.get_timestamp_us())
                source_frames.append(source)
                if len(samples) < capture_frames:
                    continue

                stack = np.stack(samples)
                raw, valid_counts = fuse_depth_frames(stack, min_valid_frames)
                # 보정값/좌표계/scale은 마지막 수신 프레임에서 유지하고 Depth 버퍼만 통합본으로 교체.
                # 원본 프레임은 수정하지 않으며 이후 점군과 RGB 정렬 모두 같은 통합 Depth를 사용한다.
                fused_depth = sdk.Frame.create_frame_from_other_frame(depth, True).as_depth_frame()
                fused_depth.set_value_scale(scale)
                fused_depth.update_data(raw.tobytes())
                fused_frames = sdk.Frame.create_frame_set()
                fused_frames.push_frame(fused_depth)
                point_filter = sdk.PointCloudFilter()
                point_filter.set_create_point_format(sdk.OBFormat.POINT)
                point_filter.set_coordinate_system(sdk.OBCoordinateSystemType.RIGHT_HAND)
                points = point_filter.process(fused_frames)
                if points is None:
                    raise RuntimeError("SDK Depth → point cloud 변환 실패")
                points = points.as_points_frame()
                xyz = points_in_meters(points)
                metadata = {
                    "device": self.device_info, "frame": DEPTH_FRAME, "alignment": "native_depth_no_color_alignment",
                    "axes": "x right, y down, z forward", "host_received_ns": received_ns,
                    "device_timestamp_us": depth.get_timestamp_us(),
                    "sdk_system_timestamp_us": depth.get_system_timestamp_us(), "frame_index": depth.get_index(),
                    "width": depth.get_width(), "height": depth.get_height(), "fps": self.profile.get_fps(),
                    "format": "Y16", "depth_scale_mm_per_unit": scale,
                    "point_scale_mm_per_unit": float(points.get_position_value_scale()),
                    **profile_calibration(self.profile), "discarded_warmup_frames": settings["warmup_frames"],
                    "depth_aggregation": {
                        "method": "nonzero_lower_median", "frame_count": len(samples),
                        "min_valid_frames": min_valid_frames,
                        "raw_frames_file": "depth_frames.npy", "valid_counts_file": "depth_valid_counts.npy",
                        "output_file": "depth_raw.npy", "reference_frame": "last_collected",
                        "depth_storage": "uint16, original raw units; pixels below min_valid_frames=0",
                        "valid_pixels": int(np.count_nonzero(raw)),
                        "rejected_low_observation_pixels": int(np.count_nonzero(
                            (valid_counts > 0) & (valid_counts < min_valid_frames))),
                        "filled_pixels_vs_reference": int(np.count_nonzero((stack[-1] == 0) & (raw > 0))),
                        "frames": source_frames,
                    },
                }
                result = CapturedView(raw, xyz, metadata, depth_frames=stack, depth_valid_counts=valid_counts)
                if color is not None:
                    # RGB는 평균내지 않는다. 정지한 장면의 마지막 동기 RGB 한 장을 원본으로 보존한다.
                    result.color_rgb, result.color_aligned_rgb = align_color_to_depth(sdk, fused_depth, color)
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
                        "selection": "last_collected_pair; aligned_using_aggregated_depth",
                    }
                print(f"Depth {len(samples)}프레임 통합: 유효 {np.count_nonzero(raw):,}픽셀, "
                      f"최소 {min_valid_frames}회 미달 제외 "
                      f"{metadata['depth_aggregation']['rejected_low_observation_pixels']:,}픽셀, "
                      f"마지막 프레임 대비 결손 보완 {metadata['depth_aggregation']['filled_pixels_vs_reference']:,}픽셀",
                      flush=True)
                return result
            stream = "Depth/RGB" if settings.get("capture_color", False) else "Depth"
            raise TimeoutError(f"유효한 새 {stream} 프레임 취득 시간 초과: "
                               f"초기 버림 {min(count, settings['warmup_frames'])}/{settings['warmup_frames']}, "
                               f"수집 {len(samples)}/{capture_frames}. 불완전한 묶음은 사용하지 않습니다.")
        finally:
            self.pipeline.stop()


# ========================================================================
# SDK 필터 취득
# ========================================================================

FILTER_ORDER = ("DecimationFilter", "ThresholdFilter", "NoiseRemovalFilter",
                "SpatialAdvancedFilter", "TemporalFilter", "HoleFillingFilter")
# (기본값, 최소, 최대, 정수 여부). 실제 SDK schema 범위는 장치 초기화 후 다시 확인.
PARAMETERS = {
    "DecimationFilter": {"decimate": (2, 1, 8, True)},
    "ThresholdFilter": {"min": (0, 0, 16000, True), "max": (16000, 0, 16000, True)},
    "NoiseRemovalFilter": {"min_diff": (256, 1, 10000, True), "max_size": (80, 1, 1000, True)},
    "SpatialAdvancedFilter": {"alpha": (0.5, 0.1, 1, False), "disp_diff": (160, 1, 10000, True),
                              "magnitude": (1, 1, 5, True), "radius": (1, 0, 8, True)},
    "TemporalFilter": {"diff_scale": (0.1, 0.1, 1, False), "weight": (0.4, 0.1, 1, False)},
    "HoleFillingFilter": {"hole_filling_mode": (0, 0, 2, True)},
}


def validate_filters(settings):
    if not isinstance(settings, dict) or set(settings) - {"warmup_frames", "filters"}:
        raise ValueError("depth_filters에는 warmup_frames와 filters만 지정하세요.")
    result = copy.deepcopy(settings)
    warmup = result.setdefault("warmup_frames", 30)
    if type(warmup) is not int or warmup < 0:
        raise ValueError("depth_filters.warmup_frames는 0 이상의 정수여야 합니다.")
    filters = result.setdefault("filters", {})
    if not isinstance(filters, dict) or set(filters) - set(FILTER_ORDER):
        raise ValueError("지원하는 필터: " + ", ".join(FILTER_ORDER))
    for name in FILTER_ORDER:
        item = filters.setdefault(name, {"enabled": False})
        if not isinstance(item, dict) or set(item) - {"enabled", *PARAMETERS[name]}:
            raise ValueError(f"{name}: 알 수 없는 설정 키입니다.")
        if type(item.setdefault("enabled", False)) is not bool:
            raise ValueError(f"{name}.enabled는 true/false여야 합니다.")
        for key, (default, low, high, integer) in PARAMETERS[name].items():
            value = item.setdefault(key, default)
            if (type(value) not in (int, float) or not np.isfinite(value)
                    or (integer and type(value) is not int) or not low <= value <= high):
                raise ValueError(f"{name}.{key}: {low}~{high} 범위의 {'정수' if integer else '숫자'}가 필요합니다.")
    threshold = filters["ThresholdFilter"]
    if threshold["min"] >= threshold["max"]:
        raise ValueError("ThresholdFilter.min은 max보다 작아야 합니다.")
    if filters["TemporalFilter"]["enabled"] and warmup < 1:
        raise ValueError("TemporalFilter 사용 시 depth_filters.warmup_frames를 1 이상으로 설정하세요.")
    return result

def depth_array(frame):
    return np.frombuffer(frame.get_data(), dtype=np.uint16).reshape(frame.get_height(), frame.get_width()).copy()


class FilterChain:
    """장치 초기화 후 생성. Frame 프로파일/단위를 SDK가 갱신한 상태로 유지한다."""
    def __init__(self, sdk, settings):
        self.sdk = sdk
        self.settings = validate_filters(settings)
        self.filters, self.metadata = [], []
        for name in FILTER_ORDER:
            requested = self.settings["filters"][name]
            if not requested["enabled"]:
                continue
            # Factory 경로는 장치 미초기화 시 OBError를 반환한다. 이 SDK 버전의
            # 일부 직접 생성자는 private filter 활성화 전 native crash를 일으킨다.
            filt = sdk.FilterFactory.create_filter(name)
            schemas = {p.name: p for p in filt.get_config_schema_vec()}
            values, defaults = {}, {}
            for key in PARAMETERS[name]:
                if key not in schemas:
                    raise ValueError(f"설치된 SDK의 {name}에 {key}가 없습니다.")
                value, schema = requested[key], schemas[key]
                if not schema.min <= value <= schema.max:
                    raise ValueError(f"{name}.{key}={value}: SDK 범위 {schema.min}~{schema.max} 밖입니다.")
                filt.set_config_value(key, value)
                actual = filt.get_config_value(key)
                if not np.isclose(actual, value, rtol=1e-6, atol=1e-6):
                    raise ValueError(f"{name}.{key} 설정 불일치: 요청 {value}, SDK {actual}")
                values[key], defaults[key] = actual, schema.default
            filt.enable(True)
            self.filters.append((name, filt, schemas))
            self.metadata.append({"name": name, "parameters": values, "sdk_defaults": defaults})
        self.last_stages = []

    def reset(self):
        for _, filt, _ in self.filters:
            filt.reset()
        self.last_stages = []

    def process(self, depth):
        # SDK 필터의 in-place 처리 가능성에 관계없이 비교용 원본을 보존한다.
        current = self.sdk.Frame.create_frame_from_other_frame(depth, True).as_depth_frame()
        stages = []
        for name, filt, schemas in self.filters:
            auto = {}
            if name == "NoiseRemovalFilter":
                for key, value in (("width", current.get_width()), ("height", current.get_height())):
                    if key in schemas:
                        if not schemas[key].min <= value <= schemas[key].max:
                            raise ValueError(f"NoiseRemovalFilter의 {key} 지원 범위를 벗어났습니다: {value}")
                        filt.set_config_value(key, value)
                        auto[key] = value
            out = filt.process(current)
            if out is None:
                self.last_stages = stages + [{"name": name, "output": None}]
                return None
            current = out.as_depth_frame()
            if current.get_format() != self.sdk.OBFormat.Y16:
                raise ValueError(f"{name} 출력이 Y16 Depth가 아닙니다.")
            profile = current.get_stream_profile().as_video_stream_profile()
            intrinsic = profile.get_intrinsic()
            if (intrinsic.width != current.get_width() or intrinsic.height != current.get_height()
                    or intrinsic.fx <= 0 or intrinsic.fy <= 0):
                raise ValueError(f"{name}: 출력 해상도와 내부 파라미터가 일치하지 않습니다.")
            stages.append({"name": name, "width": current.get_width(), "height": current.get_height(),
                           "valid_pixels": int(np.count_nonzero(depth_array(current))),
                           "automatic_parameters": auto})
        self.last_stages = stages
        return current


class FilteredDepthCamera(DepthCamera):
    def __init__(self, camera_settings, filter_settings):
        super().__init__(camera_settings)  # SDK 장치 초기화/필터 활성화를 먼저 수행.
        self.chain = FilterChain(self.sdk, filter_settings)

    def make_capture(self, depth, color, received_ns):
        sdk = self.sdk
        profile = depth.get_stream_profile().as_video_stream_profile()
        scale = float(depth.get_depth_scale())
        if not np.isfinite(scale) or scale <= 0:
            raise ValueError("잘못된 Depth scale")
        frames = sdk.Frame.create_frame_set()
        frames.push_frame(depth)
        point_filter = sdk.PointCloudFilter()
        point_filter.set_create_point_format(sdk.OBFormat.POINT)
        point_filter.set_coordinate_system(sdk.OBCoordinateSystemType.RIGHT_HAND)
        points = point_filter.process(frames)
        if points is None:
            raise RuntimeError("SDK Depth → point cloud 변환 실패")
        points = points.as_points_frame()
        raw, xyz = depth_array(depth), points_in_meters(points)
        if len(xyz) != raw.size:
            raise ValueError("필터 출력 Depth와 점군 픽셀 개수가 다릅니다.")
        metadata = {"device": self.device_info, "frame": DEPTH_FRAME,
                    "alignment": "native_depth_no_color_alignment", "axes": "x right, y down, z forward",
                    "host_received_ns": received_ns, "device_timestamp_us": depth.get_timestamp_us(),
                    "sdk_system_timestamp_us": depth.get_system_timestamp_us(), "frame_index": depth.get_index(),
                    "width": depth.get_width(), "height": depth.get_height(), "fps": profile.get_fps(),
                    "format": "Y16", "depth_scale_mm_per_unit": scale,
                    "point_scale_mm_per_unit": float(points.get_position_value_scale()),
                    **profile_calibration(profile)}
        result = CapturedView(raw, xyz, metadata)
        if color is not None:
            result.color_rgb, result.color_aligned_rgb = align_color_to_depth(sdk, depth, color)
            color_profile = color.get_stream_profile().as_video_stream_profile()
            ex = profile.get_extrinsic_to(color_profile)
            metadata["alignment"] = "color_to_depth_output_grid"
            metadata["color"] = {"width": color.get_width(), "height": color.get_height(),
                "fps": color_profile.get_fps(), "source_format": str(color.get_format()), "saved_format": "RGB8",
                "frame_index": color.get_index(), "device_timestamp_us": color.get_timestamp_us(),
                "sdk_system_timestamp_us": color.get_system_timestamp_us(),
                "depth_color_time_delta_us": abs(color.get_timestamp_us() - depth.get_timestamp_us()),
                **profile_calibration(color_profile), "alignment_target_distortion": True,
                "depth_to_color": {"rotation": np.asarray(ex.rot).reshape(3, 3).tolist(),
                                   "translation_mm": np.asarray(ex.transform).reshape(3).tolist()},
                "unmapped_color": "black", "selection": "same_pair_as_source_depth"}
        return result

    def capture(self):
        settings, sdk = self.settings, self.sdk
        self.chain.reset()  # 각 자세별 Temporal 이력 초기화.
        self.pipeline.start(self.config)
        try:
            deadline = time.monotonic() + settings["capture_timeout_sec"]
            count, processed_count, seen = 0, 0, set()
            while time.monotonic() < deadline:
                frames = self.pipeline.wait_for_frames(500)
                depth = frames.get_depth_frame() if frames is not None else None
                if depth is None:
                    continue
                identity = (depth.get_index(), depth.get_timestamp_us())
                if identity in seen:
                    continue
                color = frames.get_color_frame() if settings.get("capture_color", False) else None
                if settings.get("capture_color", False) and (color is None or abs(
                        color.get_timestamp_us() - depth.get_timestamp_us()) > settings["color_max_time_delta_ms"] * 1000):
                    continue
                if depth.get_format() != sdk.OBFormat.Y16:
                    raise ValueError("Y16 Depth만 지원합니다.")
                seen.add(identity)
                count += 1
                if count <= settings["warmup_frames"]:
                    continue
                received_ns = time.time_ns()
                filtered = self.chain.process(depth)
                processed_count += 1
                if filtered is None or processed_count <= self.chain.settings["warmup_frames"]:
                    continue
                result = self.make_capture(filtered, color, received_ns)
                result.unfiltered = self.make_capture(depth, color, received_ns)
                result.unfiltered.metadata["capture_mode"] = "sdk_output_before_test_filters"
                result.metadata.update(capture_mode="sdk_filter_chain", captured_frames=1,
                    discarded_warmup_frames=settings["warmup_frames"], filter_input_frames=processed_count,
                    filter_warmup_frames=self.chain.settings["warmup_frames"],
                    filter_chain=self.chain.metadata, filter_stages=self.chain.last_stages,
                    source_depth={"width": depth.get_width(), "height": depth.get_height(),
                                  "frame_index": depth.get_index(), "device_timestamp_us": depth.get_timestamp_us()},
                    temporal_history_reset_per_view=True)
                print(f"SDK 필터: {processed_count}프레임 처리 후 마지막 출력 저장 "
                      f"({depth.get_width()}×{depth.get_height()} → {filtered.get_width()}×{filtered.get_height()})", flush=True)
                return result
            raise TimeoutError(f"SDK 필터 출력 취득 시간 초과: 입력 {processed_count}프레임, 마지막 단계 {self.chain.last_stages}")
        finally:
            self.pipeline.stop()


# ========================================================================
# 뷰별 이동·촬영·저장
# ========================================================================

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


def save_depth_visualization(path, raw, scale_mm_per_unit, minimum_mm, maximum_mm):
    """저장한 Depth와 같은 픽셀의 거리 색상 PNG. live Depth와 같은 고정 범위/JET."""
    import cv2

    depth_mm = raw.astype(np.float32) * scale_mm_per_unit
    gray = np.clip((depth_mm - minimum_mm) / (maximum_mm - minimum_mm) * 255, 0, 255).astype(np.uint8)
    colored = cv2.applyColorMap(gray, cv2.COLORMAP_JET)
    colored[raw == 0] = 0
    if not cv2.imwrite(str(path), colored):
        raise OSError(f"Depth 시각화 이미지 저장 실패: {path}")
    return {"file": Path(path).name, "source_file": "depth_raw.npy", "colormap": "JET",
            "min_depth_mm": minimum_mm, "max_depth_mm": maximum_mm,
            "out_of_range": "clipped_to_display_range", "invalid_depth_color_rgb": [0, 0, 0]}


def capture_sequence(cfg, model, robot, camera, output, pose_source=None):
    """주입 가능한 robot/camera를 사용하여 순서와 저장 과정을 오프라인 테스트 가능."""
    import open3d as o3d

    pose_source = pose_source or ScanPoseSource(cfg, model)
    manifest = {"status": "in_progress", "created_at": datetime.now(timezone.utc).isoformat(),
                "units": {"point_cloud": "m", "joint": "deg"}, "reference_frame": BASE_FRAME,
                "method": pose_source.method, "pose_estimation": pose_source.metadata, "icp_applied": False,
                "settle_criterion": "stationary_actual_joints",
                "color_mode": "rgb" if cfg["camera"].get("capture_color", False) else "view_colors", "views": []}
    save_json(output / "manifest.json", manifest)
    save_json(output / "settings.json", cfg)
    pose_source.save_report(output)
    (output / "model.urdf").write_text(model.xml, encoding="utf-8")
    manifest["model_sha256"] = hashlib.sha256(model.xml.encode()).hexdigest()
    manifest["python"] = sys.executable
    manifest["packages"] = {name: importlib.metadata.version(name)
                            for name in ("numpy", "open3d", "pyorbbecsdk2", "xacro")}
    palette = view_colors(cfg["views"])
    try:
        for view in cfg["views"]:
            name = view["name"]
            directory = output / name
            directory.mkdir()
            entry = {"name": name, "status": "moving", "target_joint_deg": view.get("joint_deg"),
                     "command": target_description(view, cfg), "view_color_rgb": palette[name]}
            manifest["views"].append(entry)
            save_json(output / "manifest.json", manifest)
            print(f"[{name}] 이동 및 정지 확인", flush=True)
            entry["settled_state"] = (robot.move_cartesian(view["pose_mm_deg"])
                                      if target_key(view) == "pose_mm_deg" else robot.move(view["joint_deg"]))
            settled = entry["settled_state"]
            target = finite_vector(settled["target_joint_deg"], 6, "계산된 target_joint_deg").tolist()
            entry["target_joint_deg"] = target
            print(f"[{name}] 정지 확인: 실제 관절각 {settled['joint_deg']} deg, "
                  f"목표와 최대 차이 {settled['max_target_error_deg']:.4f} deg (기록용)", flush=True)
            save_json(output / "manifest.json", manifest)
            before = pose_source.snapshot(robot)
            # 이동 명령 종료와 실제 자세의 정지 유지가 확인된 뒤 새 스트림을 시작한다.
            captured = camera.capture()
            raw, xyz, calibration = captured.depth_raw, captured.xyz, captured.metadata
            after = None
            try:
                after = pose_source.snapshot(robot)  # 디스크 저장보다 먼저 촬영 직후 자세 조회.
            finally:
                # 직후 자세 조회가 실패해도 이미 받은 원본은 남긴다.
                np.save(directory / "depth_raw.npy", raw, allow_pickle=False)
                if hasattr(captured, "unfiltered"):
                    source = captured.unfiltered
                    np.save(directory / "depth_unfiltered.npy", source.depth_raw, allow_pickle=False)
                    save_json(directory / "camera_unfiltered.json", source.metadata)
                if captured.depth_frames is not None:
                    np.save(directory / "depth_frames.npy", captured.depth_frames, allow_pickle=False)
                    np.save(directory / "depth_valid_counts.npy", captured.depth_valid_counts, allow_pickle=False)
                if captured.color_rgb is not None:
                    save_rgb(directory / "color_rgb.png", captured.color_rgb)
                if captured.color_aligned_rgb is not None:
                    save_rgb(directory / "color_aligned_depth.png", captured.color_aligned_rgb)
                calibration["depth_visualization"] = save_depth_visualization(
                    directory / "depth_visualization.png", raw, calibration["depth_scale_mm_per_unit"],
                    cfg["camera"]["min_depth_mm"], cfg["camera"]["max_depth_mm"])
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
            target_error = (np.asarray(after["joint_deg"]) - target).tolist()
            max_target_error = max(abs(value) for value in target_error)
            frames = pose_source.frames(after)
            cartesian_diagnostics = cartesian_error(view, cfg, frames)
            from .registration import validate_transform
            transform = validate_transform(frames[DEPTH_FRAME])
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
            save_cloud(directory / "base.ply", base_points, palette[name], colors=colors)
            pose = {"before": before, "after": after, "accepted": True, "joint_drift_deg": drift,
                    "target_joint_deg": target, "target_error_deg": target_error,
                    "command": entry["command"], "ik": settled.get("ik"), **cartesian_diagnostics,
                    "max_target_error_deg": max_target_error,
                    "pose_source": pose_source.pose_source, "pose_estimation": pose_source.metadata,
                    "T_base_flange": frames["link_6"].tolist(), "T_base_bracket": frames["bracket_link"].tolist(),
                    "T_flange_depth": (np.linalg.inv(frames["link_6"]) @ transform).tolist(),
                    "T_base_depth": transform.tolist(), "translation_unit": "m"}
            save_json(directory / "pose.json", pose)
            entry.update(status="accepted", point_count=len(points), joint_drift_deg=drift,
                         target_error_deg=target_error, max_target_error_deg=max_target_error,
                         **cartesian_diagnostics)
            save_json(output / "manifest.json", manifest)
            print(f"[{name}] 저장 완료: {len(points):,} points", flush=True)
        manifest.update(status="complete", raw_point_count=sum(v["point_count"] for v in manifest["views"]),
                        reconstruction_pending=True)
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


# ========================================================================
# 단계 실행 진입점: 실장비 사전 검사 → 취득
# ========================================================================

def prepare_capture(settings):
    """실장비 연결 없이 모델/강체 TF/제어 프로토콜/패키지 검사를 수행."""
    from .registration import validate_transform

    cfg = settings["capture"]
    for name in ("control_python", "control_wrapper", "control_library", "xacro"):
        if not cfg.get(name) or not Path(cfg[name]).is_file():
            raise ValueError(f"{name}에 실제 새 제어 환경/모델 경로를 설정하세요: {cfg.get(name)}")
    if cfg.get("mounting_tf_confirmed") is not True:
        raise ValueError("실제 장착/안전한 전체 이동 경로 확인 후 mounting_tf_confirmed: true로 설정하세요.")
    model = ScanModel(cfg["xacro"])
    pose = ScanPoseSource(cfg, model)
    # 비강체 SDK extrinsic은 ICP/PGO에서 사용할 수 없다. 정규화해 숨기지 않는다.
    validate_transform(pose.flange_depth)
    for view in cfg["views"]:
        key = target_key(view)
        if view[key] is None:
            raise ValueError(f"{view['name']}: 목표 자세를 입력하세요.")
        if key == "joint_deg":
            model.transforms(view[key])
        else:
            pose_matrix(view[key])
    checked = subprocess.run(worker_command(cfg, "--check"),
        env=worker_environment(cfg["control_python"]),
        input=json.dumps(ik_request(cfg, model, pose)), capture_output=True, text=True, timeout=30)
    if checked.returncode:
        raise RuntimeError("새 제어 연결부 검사 실패 (장비 연결 전):\n" + checked.stdout + checked.stderr)
    import open3d  # noqa: F401
    import pyorbbecsdk  # noqa: F401 -- Context 생성하지 않음
    import cv2  # noqa: F401
    return model, pose


def acquire(settings, output, model, pose):
    cfg = settings["capture"]
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    camera = (FilteredDepthCamera(cfg["camera"], settings["filters"])
              if cfg["capture_mode"] == "sdk_filters" else DepthCamera(cfg["camera"]))
    with RobotClient(cfg, output) as robot:
        robot.call("initialize", settings=cfg["robot"], **ik_request(cfg, model, pose),
                   timeout=cfg["robot"]["connect_timeout_sec"] + 15)
        capture_sequence(cfg, model, robot, camera, output, pose)
    return output
