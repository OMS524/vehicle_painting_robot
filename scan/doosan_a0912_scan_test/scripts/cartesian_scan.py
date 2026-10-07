"""스캔 목표 자세 변환과 두산 SDK IK 연결. 생성 시 하드웨어에 연결하지 않는다."""
import math
import xml.etree.ElementTree as ET

import numpy as np

from visualize_xacro import origin_matrix

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
    """촬영 후 실제 관절 FK와 목표의 오차. 진단용이며 TF/촬영 허용 기준은 변경하지 않음."""
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
    """URDF는 플랜지 이후 고정 TF에만 사용하고 관절 해는 SDK에서 구한다."""
    def __init__(self, model_xml, settings):
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
                       "sdk_target_pose_mm_zyz_deg": sdk_pose}}
