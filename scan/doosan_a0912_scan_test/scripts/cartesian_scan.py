"""스캔 목표 자세 규약과 control 환경 전용 IK. 카메라/로봇 연결은 하지 않는다."""
import math
import xml.etree.ElementTree as ET

import numpy as np

from visualize_xacro import origin_matrix

JOINT_NAMES = tuple(f"joint_{i}" for i in range(1, 7))
TARGET_FRAMES = ("link_6", "bracket_link", "camera_depth_optical_frame")
RPY_CONVENTION = "fixed_XYZ: Rz(yaw) @ Ry(pitch) @ Rx(roll)"
DEFAULTS = {"target_frame": "link_6", "position_tolerance_mm": 0.1,
            "orientation_tolerance_deg": 0.1, "max_iterations": 300}


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
    for key in ("position_tolerance_mm", "orientation_tolerance_deg"):
        value = settings[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"cartesian.{key}는 양의 유한한 숫자여야 합니다.")
    iterations = settings["max_iterations"]
    if type(iterations) is not int or not 1 <= iterations <= 5000:
        raise ValueError("cartesian.max_iterations는 1~5000 정수여야 합니다.")
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


class CartesianIK:
    """현재 실측 관절을 seed로 쓰는 제한된 수치 IK. 자동 재시드/분기 전환 없음.

    Pinocchio는 control Python에서만 import한다. scan에서 사용한 확장 URDF를
    그대로 받아 TCP/브라켓 TF 중복 적용과 별도 URDF 모델 불일치를 방지한다.
    """
    def __init__(self, model_xml, settings):
        import pinocchio as pin

        cfg = {"cartesian": dict(settings), "views": []}
        validate_targets(cfg)
        self.settings, self.pin = cfg["cartesian"], pin
        self.model = pin.buildModelFromXML(model_xml)
        model = self.model
        if model.nq != 6 or model.nv != 6 or set(model.names[1:]) != set(JOINT_NAMES):
            raise ValueError("IK는 A0912의 joint_1~6 회전 조인트만 지원합니다.")
        self.indices = [model.joints[model.getJointId(name)].idx_q for name in JOINT_NAMES]
        for name in JOINT_NAMES:
            joint = model.joints[model.getJointId(name)]
            if joint.nq != 1 or joint.nv != 1 or joint.idx_q != joint.idx_v:
                raise ValueError(f"IK 관절 모델 오류: {name}")
        for name in ("base_link", self.settings["target_frame"]):
            if not model.existFrame(name):
                raise ValueError(f"IK 모델에 프레임이 없습니다: {name}")
        base_id = model.getFrameId("base_link")
        self.frame_id = model.getFrameId(self.settings["target_frame"])
        if model.frames[base_id].parentJoint != 0 or model.frames[self.frame_id].parentJoint != model.getJointId("joint_6"):
            raise ValueError("base_link는 고정, 목표 프레임은 joint_6 뒤의 고정 툴이어야 합니다.")
        self.data = model.createData()
        pin.framesForwardKinematics(model, self.data, np.zeros(6))
        self.world_base = self.data.oMf[base_id].copy()
        self.lower = np.asarray(model.lowerPositionLimit).copy()
        self.upper = np.asarray(model.upperPositionLimit).copy()
        if not np.isfinite([self.lower, self.upper]).all() or np.any(self.lower >= self.upper):
            raise ValueError("IK 모델의 유한한 관절 제한이 필요합니다.")

    def solve(self, values, seed_deg, max_delta_deg):
        pin, model, data = self.pin, self.model, self.data
        desired_base = pose_matrix(values)
        desired = self.world_base * pin.SE3(desired_base[:3, :3], desired_base[:3, 3])
        seed = np.deg2rad(vector6(seed_deg, "현재 실측 joint_deg"))
        q = np.empty(6)
        q[self.indices] = seed
        if np.any(q < self.lower) or np.any(q > self.upper):
            raise ValueError("현재 실측 관절각이 URDF 제한 밖입니다. 영점/모델을 확인하세요.")
        if isinstance(max_delta_deg, bool) or not math.isfinite(max_delta_deg) or max_delta_deg <= 0:
            raise ValueError("max_move_delta_deg는 양의 유한한 숫자여야 합니다.")
        delta = math.radians(max_delta_deg)
        lower, upper = np.maximum(self.lower, q - delta), np.minimum(self.upper, q + delta)
        pos_tol = self.settings["position_tolerance_mm"] / 1000
        rot_tol = math.radians(self.settings["orientation_tolerance_deg"])

        def evaluate(candidate):
            pin.framesForwardKinematics(model, data, candidate)
            actual = data.oMf[self.frame_id]
            relative = actual.actInv(desired)
            error = pin.log6(relative).vector.copy()
            position_error = float(np.linalg.norm(desired.translation - actual.translation))
            rotation_error = float(np.linalg.norm(pin.log3(relative.rotation)))
            return relative, error, position_error, rotation_error

        for iteration in range(self.settings["max_iterations"] + 1):
            relative, error, position_error, rotation_error = evaluate(q)
            if position_error <= pos_tol and rotation_error <= rot_tol:
                return {"target_joint_deg": np.rad2deg(q[self.indices]).tolist(),
                        "ik": {"method": "pinocchio_damped_least_squares", "seed_joint_deg": list(seed_deg),
                               "target_frame": self.settings["target_frame"], "reference_frame": "base_link",
                               "rpy_convention": RPY_CONVENTION, "target_pose_mm_deg": list(values),
                               "T_base_target": desired_base.tolist(), "iterations": iteration,
                               "position_error_mm": position_error * 1000,
                               "orientation_error_deg": math.degrees(rotation_error)}}
            if iteration == self.settings["max_iterations"]:
                break
            jacobian = pin.computeFrameJacobian(model, data, q, self.frame_id, pin.ReferenceFrame.LOCAL)
            jacobian = -pin.Jlog6(relative.inverse()) @ jacobian
            step = -jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + 1e-6 * np.eye(6), error)
            step *= min(1.0, math.radians(5) / max(float(np.max(np.abs(step))), 1e-12))
            accepted = False
            for scale in (1.0, 0.5, 0.25, 0.1):
                candidate = np.clip(q + scale * step, lower, upper)
                _, candidate_error, _, _ = evaluate(candidate)
                if np.linalg.norm(candidate_error) < np.linalg.norm(error):
                    q, accepted = candidate, True
                    break
            if not accepted:
                break
        raise ValueError("현재 자세 seed/관절 제한/최대 이동량 안에서 IK가 수렴하지 않았습니다. "
                         "이동하지 않습니다. 목표/시작 자세를 확인하세요. "
                         f"잔차: {position_error * 1000:.4f} mm, {math.degrees(rotation_error):.4f} deg")
