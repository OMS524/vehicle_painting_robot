#!/usr/bin/env python3

from __future__ import annotations

import argparse
import atexit
import csv
from dataclasses import dataclass
import math
from pathlib import Path
import signal
import sys
import time
import xml.etree.ElementTree as ET

import casadi as ca
import numpy as np


from motion_planner.configuration import repo_root

PROJECT_ROOT = repo_root()
ROOT = PROJECT_ROOT / "motion_planning"
DEFAULT_TRAJECTORY_CSV = [
    # PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_door_small/trajectory_000.csv",
    # PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_door_small/trajectory_001.csv",
    # PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_door_small/trajectory_002.csv",
    # PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_door_small/trajectory_003.csv",
    # PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_door_small/trajectory_004.csv",

    # PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_door_small/trajectory_005.csv",
    # PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_door_small/trajectory_006.csv",
    # PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_door_small/trajectory_007.csv",

    # PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_door_small/trajectory_008.csv",
    # PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_door_small/trajectory_009.csv",



    # PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_bumper_small/trajectory_000.csv",
    # PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_bumper_small/trajectory_001.csv",
    # PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_bumper_small/trajectory_002.csv",
    # PROJECT_ROOT / "painting_trajectory_planner/test/csv/car_bumper_small/trajectory_003.csv",
]
DEFAULT_URDF = (
    PROJECT_ROOT
    / "robot_controller/doosan_a0912_controller/urdf/a0912.white.urdf"
)
DEFAULT_OUTPUT_DIR = Path("/tmp/trajectory_nlp")

TCP_FRAME = "auto"  # tcp > spray_gun > link_6 중 URDF 에 있는 것 (resolve_tcp_frame)
TCP_SPRAY_AXIS = "z"
TCP_PATH_AXIS = "y"
COMMAND_TIME_SEC = 0.001
DONE_MARGIN_SEC = 2.0

INITIAL_JOINT_DEG = "90,0,90,-90,0,0"
POSITION_TOLERANCE_M = 0.002
ORIENTATION_TOLERANCE_DEG = 1.0
MAX_JOINT_STEP_DEG = 20.0
MAX_JOINT_VELOCITY_DEG_S = 70.0
MAX_JOINT_ACCELERATION_DEG_S2 = 500.0
MIN_MANIPULABILITY = 0.0

# 갠트리 직동축(prismatic) 한계. 회전축과 단위가 달라 따로 둔다.
# 6축 회전 전용 URDF 에서는 직동축이 없어 쓰이지 않는다.
# 기본값 "auto" = gantry_env yaml 의 motor_limits.gantry_velocity_m_s / gantry_acceleration_m_s2 (x7y4z3: 0.5 / 1.6,
# 2026-10-20 사용자: 모터 최고 0.8 m/s 라 0.5 로 계획). env 에 없거나 --gantry-env 가 없으면 아래 상수(종전 값).
MAX_GANTRY_VELOCITY_M_S = 1.0
MAX_GANTRY_ACCELERATION_M_S2 = 5.0
# 갠트리 전용 부드러움 항 (2026-10-20). 기존 velocity/acceleration 항은 8축을 똑같이 1e-3 으로 누르기 때문에 갠트리가
# row 길이의 85 % 를 혼자 달리고(후드 row 당 1.1~1.35 m, 최고 0.63 m/s) 전이마다 급가감속했다. 직동축 컬럼에만 추가로
# w·Σ(qd/qd_max)² + w·Σ(qdd/qdd_max)² 를 걸어 팔이 흡수할 수 있는 움직임은 팔이 하고, 갠트리는 느리고 일정하게 가게 한다.
# "auto" = gantry_env yaml nlp.gantry_velocity_weight / nlp.gantry_acceleration_weight (x7y4z3/spec_c: 1.0/1.0), 없으면 아래 상수
# 0 = 끔. legacy env 와 --gantry-env 없는 호출은 그래서 종전과 바이트 동일하고, 6축 URDF 는 직동축이 없어 항 자체가 생기지 않는다.
GANTRY_VELOCITY_WEIGHT = 0.0
GANTRY_ACCELERATION_WEIGHT = 0.0

# planner CSV 는 부품 스캔 좌표계 값이다. 로봇 월드로 옮기는 변환.
# 기본값 0 이면 예전처럼 스캔 좌표를 그대로 월드로 쓴다(= 배치 미지정).
PART_XYZ = "0,0,0"
PART_RPY = "0,0,0"

POSITION_WEIGHT = 1000.0
ORIENTATION_WEIGHT = 100.0
VELOCITY_WEIGHT = 1.0e-3
ACCELERATION_WEIGHT = 1.0e-4
CENTER_WEIGHT = 1.0e-3
SINGULARITY_WEIGHT = 1.0e-8
# 팔 특이점 회피. 2026-09-15 실측: 기본 설정으로 풀면 joint_3≈0(팔꿈치 완전 신전)·joint_5≈0(손목)에
# 해가 상시 머문다. 원인은 (1) center 항이 관절범위 중앙(=0°)으로 당기는데 그 자리가 특이점이고,
# (2) singularity 항이 8축 야코비안으로 계산돼 갠트리 직동축이 팔의 rank 손실을 가리기 때문.
# 처방: joint_3/joint_5 하한을 +25° 로 올려 0° 를 금지구역으로 만든다(중앙점도 80° 로 따라 옮겨진다).
# q3>0 은 천장장착 팔에서 팔꿈치가 위(천장)로 꺾이는 방향이다. 어깨 특이점(손목중심이 J1 축 위)은
# bound 로 못 막으므로 회전축 컬럼만 쓴 야코비안 trace((Ja Ja^T)^-1) 항을 함께 건다.
# 기본값 "auto" 는 갠트리(직동축)가 있는 천장장착 8축 모델에만 적용한다. 6축 단독(바닥 설치) URDF 는
# 팔꿈치 부호가 반대라 같은 bound 를 걸면 Infeasible 이 되고, 실기 6축 경로는 예전 출력과 바이트 동일해야
# 하므로 건드리지 않는다. 6축에도 걸고 싶으면 값을 명시할 것. 끄려면 --joint-bounds-deg "" --arm-singularity-weight 0.
ARM_JOINT_BOUNDS_DEG = "joint_3=25:135,joint_5=25:135"
ARM_SINGULARITY_WEIGHT = 1.0e-3
# 툴 간섭 항: TCP 프레임의 한 점(기본 = 스프레이건 페인트통 끝, tcp +x 0.30 m)이 지정 관절 원점(기본 joint_3=팔꿈치,
# joint_5=손목)에서 clearance 이상 떨어지도록 하는 소프트 페널티. 컵이 팔 쪽을 보며 0.2 m 까지 붙던 문제(후드 실측)를
# joint_6 bound 없이 기하로 푼다. 0 이면 끔. 툴 URDF(tcp/spray_gun 프레임) 에서만 의미가 있다.
TOOL_CLEARANCE_JOINTS = "joint_3,joint_5"
TOOL_TIP_OFFSET_M = "0.30,0,0"
TOOL_CLEARANCE_M = 0.35
TOOL_CLEARANCE_WEIGHT = 0.0
AUTO = "auto"

IPOPT_MAX_ITER = 800
IPOPT_TOL = 1.0e-5
IPOPT_PRINT_LEVEL = 5

JOINT_POSITION_GAIN = 4.0
PREPOSITION_MAX_JOINT_VELOCITY_DEG_S = 15.0
PREPOSITION_MAX_JOINT_ACCELERATION_DEG_S2 = 100.0


@dataclass
class TaskPoint:
    t: float
    s: float
    path_speed: float
    path_acceleration: float
    position: np.ndarray
    rotation: np.ndarray


@dataclass
class JointSpec:
    name: str
    joint_type: str
    parent: str
    child: str
    origin_xyz: np.ndarray
    origin_rpy: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float
    velocity: float


def parse_float_list(text: str, expected: int, name: str) -> list[float]:
    values = [float(part.strip()) for part in text.split(",") if part.strip()]
    if len(values) != expected:
        raise ValueError(f"{name} must contain {expected} comma-separated values")
    return values


def parse_signed_axis(axis: str) -> tuple[int, float]:
    value = str(axis).strip().lower()
    sign = 1.0
    if value.startswith("+"):
        value = value[1:]
    elif value.startswith("-"):
        value = value[1:]
        sign = -1.0

    indices = {"x": 0, "y": 1, "z": 2}
    if value not in indices:
        raise ValueError(f"invalid axis '{axis}'. Use one of x, y, z, -x, -y, -z")
    return indices[value], sign


def remap_planner_rotation_to_tcp(
    planner_rotation: np.ndarray,
    tcp_spray_axis: str,
    tcp_path_axis: str,
) -> np.ndarray:
    spray_index, spray_sign = parse_signed_axis(tcp_spray_axis)
    path_index, path_sign = parse_signed_axis(tcp_path_axis)
    if spray_index == path_index:
        raise ValueError("--tcp-spray-axis and --tcp-path-axis must use different axes")

    planner_rotation = np.asarray(planner_rotation, dtype=float).reshape(3, 3)
    planner_path_axis = planner_rotation[:, 0]
    planner_spray_axis = planner_rotation[:, 2]

    tcp_axes: list[np.ndarray | None] = [None, None, None]
    tcp_axes[spray_index] = spray_sign * planner_spray_axis

    path_axis = planner_path_axis - float(np.dot(planner_path_axis, tcp_axes[spray_index])) * tcp_axes[spray_index]
    path_norm = float(np.linalg.norm(path_axis))
    if path_norm <= 1.0e-9:
        path_axis = planner_rotation[:, 1]
        path_axis = path_axis - float(np.dot(path_axis, tcp_axes[spray_index])) * tcp_axes[spray_index]
        path_norm = float(np.linalg.norm(path_axis))
    if path_norm <= 1.0e-9:
        raise ValueError("could not build TCP target rotation from planner rotation")
    tcp_axes[path_index] = path_sign * (path_axis / path_norm)

    missing_index = next(index for index, axis in enumerate(tcp_axes) if axis is None)
    if missing_index == 0:
        tcp_axes[0] = np.cross(tcp_axes[1], tcp_axes[2])
    elif missing_index == 1:
        tcp_axes[1] = np.cross(tcp_axes[2], tcp_axes[0])
    else:
        tcp_axes[2] = np.cross(tcp_axes[0], tcp_axes[1])

    tcp_rotation = np.column_stack([axis / max(float(np.linalg.norm(axis)), 1.0e-12) for axis in tcp_axes])
    if float(np.linalg.det(tcp_rotation)) < 0.0:
        tcp_rotation[:, missing_index] *= -1.0
    return tcp_rotation


def worst_axis_detail(values: np.ndarray, limit: np.ndarray, prismatic: np.ndarray, unit: str) -> str:
    """한계 대비 최악 축을 각자 단위로(직동 m, 회전 deg) 표시한다."""
    if values.size == 0:
        return "n/a"
    peak = np.max(np.abs(values), axis=0)
    index = int(np.argmax(peak / np.maximum(limit, 1.0e-12)))
    scale = 1.0 if prismatic[index] else 180.0 / math.pi
    native = "m" if prismatic[index] else "deg"
    return (
        f"joint{index + 1} {peak[index] * scale:.3f}{native}{unit}"
        f"/{limit[index] * scale:.3f}{native}{unit}"
    )


def rpy_to_matrix_np(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = [float(v) for v in rpy]
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def quaternion_xyzw_to_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    q = np.array([qx, qy, qz, qw], dtype=float)
    norm = float(np.linalg.norm(q))
    if norm < 1.0e-12:
        return np.eye(3)
    qx, qy, qz, qw = q / norm
    return np.array(
        [
            [
                1.0 - 2.0 * (qy * qy + qz * qz),
                2.0 * (qx * qy - qw * qz),
                2.0 * (qx * qz + qw * qy),
            ],
            [
                2.0 * (qx * qy + qw * qz),
                1.0 - 2.0 * (qx * qx + qz * qz),
                2.0 * (qy * qz - qw * qx),
            ],
            [
                2.0 * (qx * qz - qw * qy),
                2.0 * (qy * qz + qw * qx),
                1.0 - 2.0 * (qx * qx + qy * qy),
            ],
        ],
        dtype=float,
    )


def parse_xyz(text: str | None, default: tuple[float, float, float]) -> np.ndarray:
    if not text:
        return np.array(default, dtype=float)
    values = [float(part) for part in text.split()]
    if len(values) != 3:
        raise ValueError(f"expected 3 values, got: {text}")
    return np.array(values, dtype=float)



# TCP 프레임 자동 선택 순서. 툴 장착 셀 URDF 의 "tcp"(스프레이건 노즐 팁, +z 분사) 가 최우선이고,
# 6축 단독 a0912.white.urdf 처럼 툴 프레임이 없는 모델은 종전대로 플랜지 link_6 이 된다.
TCP_FRAME_CANDIDATES = ("tcp", "spray_gun", "link_6")


def resolve_tcp_frame(urdf_path: Path, requested: str) -> str:
    """requested 가 'auto' 면 URDF 에 존재하는 첫 후보 링크를 고른다."""
    if requested != "auto":
        return requested
    links = {link.attrib.get("name", "") for link in ET.parse(urdf_path).getroot().findall("link")}
    for candidate in TCP_FRAME_CANDIDATES:
        if candidate in links:
            return candidate
    raise ValueError(f"URDF has none of the TCP candidate frames {TCP_FRAME_CANDIDATES}")

def parse_urdf_chain(urdf_path: Path, tcp_frame: str) -> list[JointSpec]:
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    joints_by_child: dict[str, JointSpec] = {}

    for joint in root.findall("joint"):
        name = joint.attrib.get("name", "")
        joint_type = joint.attrib.get("type", "fixed")
        parent_el = joint.find("parent")
        child_el = joint.find("child")
        if parent_el is None or child_el is None:
            continue

        origin_el = joint.find("origin")
        axis_el = joint.find("axis")
        limit_el = joint.find("limit")
        origin_xyz = parse_xyz(
            origin_el.attrib.get("xyz") if origin_el is not None else None,
            (0.0, 0.0, 0.0),
        )
        origin_rpy = parse_xyz(
            origin_el.attrib.get("rpy") if origin_el is not None else None,
            (0.0, 0.0, 0.0),
        )
        axis = parse_xyz(
            axis_el.attrib.get("xyz") if axis_el is not None else None,
            (0.0, 0.0, 1.0),
        )
        axis_norm = float(np.linalg.norm(axis))
        if axis_norm > 1.0e-12:
            axis = axis / axis_norm

        if joint_type == "continuous":
            lower, upper = -math.pi, math.pi
        elif limit_el is not None and joint_type != "fixed":
            lower = float(limit_el.attrib.get("lower", -math.pi))
            upper = float(limit_el.attrib.get("upper", math.pi))
        else:
            lower, upper = 0.0, 0.0
        velocity = (
            float(limit_el.attrib.get("velocity", math.inf))
            if limit_el is not None
            else math.inf
        )

        spec = JointSpec(
            name=name,
            joint_type=joint_type,
            parent=parent_el.attrib["link"],
            child=child_el.attrib["link"],
            origin_xyz=origin_xyz,
            origin_rpy=origin_rpy,
            axis=axis,
            lower=lower,
            upper=upper,
            velocity=velocity,
        )
        joints_by_child[spec.child] = spec

    chain: list[JointSpec] = []
    link = tcp_frame
    while link in joints_by_child:
        spec = joints_by_child[link]
        chain.append(spec)
        link = spec.parent
    chain.reverse()
    if not chain:
        raise ValueError(f"could not find a URDF chain ending at frame/link {tcp_frame}")
    return chain


def ca_cross(a, b):
    return ca.vertcat(
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def ca_skew(axis: np.ndarray):
    x, y, z = [float(v) for v in axis]
    return ca.DM([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def ca_axis_angle(axis: np.ndarray, q):
    k = ca_skew(axis)
    return ca.DM.eye(3) + ca.sin(q) * k + (1.0 - ca.cos(q)) * (k @ k)


def ca_rot_to_vec_row_major(rotation):
    return ca.vertcat(
        rotation[0, 0],
        rotation[0, 1],
        rotation[0, 2],
        rotation[1, 0],
        rotation[1, 1],
        rotation[1, 2],
        rotation[2, 0],
        rotation[2, 1],
        rotation[2, 2],
    )


def ca_rotation_from_row_major(values):
    return ca.vertcat(
        ca.horzcat(values[0], values[1], values[2]),
        ca.horzcat(values[3], values[4], values[5]),
        ca.horzcat(values[6], values[7], values[8]),
    )


def ca_orientation_cosine(current_rotation, target_rotation):
    relative_rotation = target_rotation.T @ current_rotation
    return 0.5 * (
        relative_rotation[0, 0]
        + relative_rotation[1, 1]
        + relative_rotation[2, 2]
        - 1.0
    )


class KinematicModel:
    def __init__(self, chain: list[JointSpec]) -> None:
        self.chain = chain
        self.active_joints = [joint for joint in chain if joint.joint_type != "fixed"]
        self.nq = len(self.active_joints)
        self.joint_names = [joint.name for joint in self.active_joints]
        self.lower = np.array([joint.lower for joint in self.active_joints], dtype=float)
        self.upper = np.array([joint.upper for joint in self.active_joints], dtype=float)
        self.velocity = np.array([joint.velocity for joint in self.active_joints], dtype=float)
        # 직동축 마스크. 회전축은 rad 계열, 직동축은 m 계열로 한계를 걸어야 한다.
        self.prismatic = np.array(
            [joint.joint_type == "prismatic" for joint in self.active_joints], dtype=bool
        )

    def motion_limits(self, args: argparse.Namespace) -> tuple[np.ndarray, np.ndarray]:
        """관절별 속도/가속도 한계를 SI 벡터로 만든다 (회전 rad 계열, 직동 m 계열).

        6축 회전 전용 모델에서는 예전 스칼라 값과 완전히 같은 벡터가 나온다.
        """
        velocity = np.where(
            self.prismatic,
            float(args.max_gantry_velocity_m_s),
            math.radians(float(args.max_joint_velocity_deg_s)),
        )
        acceleration = np.where(
            self.prismatic,
            float(args.max_gantry_acceleration_m_s2),
            math.radians(float(args.max_joint_acceleration_deg_s2)),
        )
        return velocity, acceleration

    def fk_symbolic(self, q):
        position, rotation, jacobian, _origins = self.fk_symbolic_full(q)
        return position, rotation, jacobian

    def fk_symbolic_full(self, q):
        """fk_symbolic + 활성 관절별 원점 위치 {joint_name: 3x1} (툴 간섭 항용)."""
        rotation = ca.DM.eye(3)
        position = ca.DM.zeros(3, 1)
        joint_origins = []
        joint_axes = []
        active_index = 0

        for joint in self.chain:
            position = position + rotation @ ca.DM(joint.origin_xyz.reshape(3, 1))
            rotation = rotation @ ca.DM(rpy_to_matrix_np(joint.origin_rpy))

            if joint.joint_type == "fixed":
                continue

            axis_local = ca.DM(joint.axis.reshape(3, 1))
            axis_world = rotation @ axis_local
            if joint.joint_type in ("revolute", "continuous"):
                joint_origins.append(position)
                joint_axes.append(axis_world)
                rotation = rotation @ ca_axis_angle(joint.axis, q[active_index])
            elif joint.joint_type == "prismatic":
                joint_origins.append(position)
                joint_axes.append(axis_world)
                position = position + axis_world * q[active_index]
            else:
                raise ValueError(f"unsupported joint type: {joint.joint_type}")
            active_index += 1

        columns = []
        for joint, origin, axis in zip(self.active_joints, joint_origins, joint_axes):
            if joint.joint_type in ("revolute", "continuous"):
                linear = ca_cross(axis, position - origin)
                angular = axis
            else:
                linear = axis
                angular = ca.DM.zeros(3, 1)
            columns.append(ca.vertcat(linear, angular))
        jacobian = ca.horzcat(*columns)
        origins = {joint.name: origin for joint, origin in zip(self.active_joints, joint_origins)}
        return position, rotation, jacobian, origins

    def tool_clearance_function(self, tip_offset: np.ndarray, joint_names: list[str]):
        """q -> 툴 팁 점과 지정 관절 원점 사이 거리 벡터 (평가용)."""
        q = ca.MX.sym("q", self.nq)
        position, rotation, _jacobian, origins = self.fk_symbolic_full(q)
        tip = position + rotation @ ca.DM(np.asarray(tip_offset, dtype=float).reshape(3, 1))
        distances = [ca.norm_2(tip - origins[name]) for name in joint_names]
        return ca.Function("tool_clearance", [q], [ca.vertcat(*distances)])

    def residual_function(self):
        q = ca.MX.sym("q", self.nq)
        target = ca.MX.sym("target", 12)
        position, rotation, _jacobian = self.fk_symbolic(q)
        target_position = target[0:3]
        target_rotation = ca_rotation_from_row_major(target[3:12])
        residual = ca.vertcat(
            position - target_position,
            ca.reshape(rotation - target_rotation, 9, 1),
        )
        residual_jacobian = ca.jacobian(residual, q)
        return ca.Function("residual_jacobian", [q, target], [residual, residual_jacobian])

    def fk_function(self):
        q = ca.MX.sym("q", self.nq)
        position, rotation, jacobian = self.fk_symbolic(q)
        return ca.Function(
            "fk",
            [q],
            [position, ca_rot_to_vec_row_major(rotation), jacobian],
        )


def load_task_points(
    csv_path: Path,
    stride: int,
    max_points: int,
    tcp_spray_axis: str,
    tcp_path_axis: str,
    part_rotation: np.ndarray | None = None,
    part_translation: np.ndarray | None = None,
) -> list[TaskPoint]:
    """planner CSV 를 읽어 로봇 월드 좌표계의 TaskPoint 로 만든다.

    CSV 의 position/orientation 은 **부품 스캔 좌표계** 값이다. part_rotation /
    part_translation 이 그 값을 도장셀 월드로 옮긴다(= 셀 안에서 부품을 어디에
    고정하는가). 지정하지 않으면 항등변환이라 예전 동작과 같다.
    """
    part_rotation = np.eye(3) if part_rotation is None else np.asarray(part_rotation, dtype=float)
    part_translation = (
        np.zeros(3) if part_translation is None else np.asarray(part_translation, dtype=float)
    )
    with csv_path.open("r", newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        required = [
            "t",
            "s",
            "position_x",
            "position_y",
            "position_z",
            "orientation_x",
            "orientation_y",
            "orientation_z",
            "orientation_w",
            "path_speed",
            "path_acceleration",
        ]
        missing = [name for name in required if name not in (reader.fieldnames or [])]
        if missing:
            raise ValueError(f"{csv_path} missing columns: {', '.join(missing)}")

        points = []
        first_t = None
        first_s = None
        previous_row_index = None
        for row in reader:
            row_index_text = str(row.get("row_index", "")).strip()
            if row_index_text:
                row_index = int(float(row_index_text))
                if previous_row_index is None:
                    previous_row_index = row_index
                elif row_index != previous_row_index:
                    raise ValueError(
                        f"{csv_path} contains multiple row_index values. "
                        "Optimize one trajectory_XXX.csv file at a time."
                    )

            t = float(row["t"])
            if first_t is None:
                first_t = t
            s = float(row["s"])
            if first_s is None:
                first_s = s
            position = np.array(
                [float(row["position_x"]), float(row["position_y"]), float(row["position_z"])],
                dtype=float,
            )
            position = part_rotation @ position + part_translation
            rotation = quaternion_xyzw_to_matrix(
                float(row["orientation_x"]),
                float(row["orientation_y"]),
                float(row["orientation_z"]),
                float(row["orientation_w"]),
            )
            rotation = part_rotation @ rotation
            rotation = remap_planner_rotation_to_tcp(
                rotation,
                tcp_spray_axis=tcp_spray_axis,
                tcp_path_axis=tcp_path_axis,
            )
            points.append(
                TaskPoint(
                    t=t - first_t,
                    s=s - first_s,
                    path_speed=float(row["path_speed"]),
                    path_acceleration=float(row["path_acceleration"]),
                    position=position,
                    rotation=rotation,
                )
            )

    if len(points) < 2:
        raise ValueError(f"{csv_path} needs at least two trajectory points")

    stride = max(1, int(stride))
    selected = points[::stride]
    if selected[-1] is not points[-1]:
        selected.append(points[-1])

    if max_points > 0 and len(selected) > max_points:
        indices = np.unique(np.rint(np.linspace(0, len(selected) - 1, max_points)).astype(int))
        selected = [selected[int(index)] for index in indices]
        if selected[-1] is not points[-1]:
            selected.append(points[-1])

    for prev, cur in zip(selected, selected[1:]):
        if cur.t <= prev.t:
            raise ValueError(f"{csv_path} has non-increasing time after downsampling")
        if cur.s <= prev.s:
            raise ValueError(f"{csv_path} has non-increasing arc length after downsampling")
    return selected


def target_vector(point: TaskPoint) -> np.ndarray:
    return np.concatenate([point.position, point.rotation.reshape(-1)])


def apply_joint_bound_overrides(model: KinematicModel, spec: str) -> None:
    """--joint-bounds-deg 문자열("name=lo:hi,...")로 model.lower/upper 를 덮어쓴다.

    URDF 한계보다 넓힐 수는 없고 좁히기만 한다. 회전축은 deg, 직동축은 m.
    """
    if not spec or not spec.strip():
        return
    for item in spec.split(","):
        item = item.strip()
        if not item:
            continue
        try:
            name, rng = item.split("=")
            lo_text, hi_text = rng.split(":")
            lo, hi = float(lo_text), float(hi_text)
        except ValueError as exc:
            raise ValueError(f"--joint-bounds-deg: bad item '{item}' (expected name=lo:hi)") from exc
        if name.strip() not in model.joint_names:
            raise ValueError(f"--joint-bounds-deg: unknown joint '{name}' (have {model.joint_names})")
        idx = model.joint_names.index(name.strip())
        if not model.prismatic[idx]:
            lo, hi = math.radians(lo), math.radians(hi)
        if lo >= hi:
            raise ValueError(f"--joint-bounds-deg: {name} lower {lo} >= upper {hi}")
        new_lo = max(lo, float(model.lower[idx]))
        new_hi = min(hi, float(model.upper[idx]))
        model.lower[idx] = new_lo
        model.upper[idx] = new_hi
        unit = "m" if model.prismatic[idx] else "deg"
        shown = (new_lo, new_hi) if model.prismatic[idx] else (math.degrees(new_lo), math.degrees(new_hi))
        print(f"[nlp] joint bound override {name.strip()}: [{shown[0]:.2f}, {shown[1]:.2f}] {unit}")


def seed_path(
    model: KinematicModel,
    points: list[TaskPoint],
    initial_q: np.ndarray,
    max_step_rad: float,
) -> np.ndarray:
    residual_fun = model.residual_function()
    q = np.clip(initial_q.astype(float), model.lower, model.upper)
    out = []
    damping = 1.0e-3

    for point_index, point in enumerate(points):
        target = target_vector(point)
        for _ in range(120):
            residual, jacobian = residual_fun(q, target)
            residual = np.asarray(residual, dtype=float).reshape(-1)
            jacobian = np.asarray(jacobian, dtype=float)
            if (
                np.linalg.norm(residual[:3]) < 2.0e-3
                and np.linalg.norm(residual[3:]) < 3.0e-2
            ):
                break
            lhs = jacobian.T @ jacobian + damping * damping * np.eye(model.nq)
            rhs = -jacobian.T @ residual
            try:
                dq = np.linalg.solve(lhs, rhs)
            except np.linalg.LinAlgError:
                dq = np.linalg.lstsq(lhs, rhs, rcond=None)[0]
            step_norm = float(np.linalg.norm(dq, ord=np.inf))
            if step_norm > max_step_rad:
                dq *= max_step_rad / step_norm
            q = np.clip(q + dq, model.lower, model.upper)

        residual, _jacobian = residual_fun(q, target)
        residual = np.asarray(residual, dtype=float).reshape(-1)
        if point_index % 25 == 0 or point_index == len(points) - 1:
            print(
                f"[nlp] seed {point_index + 1}/{len(points)} "
                f"pos_err={np.linalg.norm(residual[:3]):.5f}m "
                f"rot_resid={np.linalg.norm(residual[3:]):.5f}"
            )
        out.append(q.copy())
    return np.asarray(out, dtype=float)


def finite_difference(times: np.ndarray, positions: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    velocity = np.zeros_like(positions)
    acceleration = np.zeros_like(positions)

    for index in range(len(times)):
        lo = max(0, index - 1)
        hi = min(len(times) - 1, index + 1)
        dt = times[hi] - times[lo]
        if dt <= 0.0:
            raise ValueError("invalid trajectory time spacing")
        velocity[index] = (positions[hi] - positions[lo]) / dt

    for index in range(len(times)):
        lo = max(0, index - 1)
        hi = min(len(times) - 1, index + 1)
        dt = times[hi] - times[lo]
        if dt <= 0.0:
            raise ValueError("invalid trajectory time spacing")
        acceleration[index] = (velocity[hi] - velocity[lo]) / dt

    velocity[0] = 0.0
    velocity[-1] = 0.0
    acceleration[0] = 0.0
    acceleration[-1] = 0.0
    return velocity, acceleration


def derivative_by_axis(axis: np.ndarray, values: np.ndarray) -> np.ndarray:
    axis = np.asarray(axis, dtype=float).reshape(-1)
    values = np.asarray(values, dtype=float)
    if len(axis) != len(values):
        raise ValueError("axis and values length mismatch")
    if len(axis) < 2:
        raise ValueError("at least two samples are required")
    if np.any(np.diff(axis) <= 0.0):
        raise ValueError("axis must be strictly increasing")

    edge_order = 2 if len(axis) > 2 else 1
    derivative = np.zeros_like(values)
    for column in range(values.shape[1]):
        derivative[:, column] = np.gradient(values[:, column], axis, edge_order=edge_order)
    return derivative


def interpolate_columns(axis: np.ndarray, values: np.ndarray, samples: np.ndarray) -> np.ndarray:
    axis = np.asarray(axis, dtype=float).reshape(-1)
    values = np.asarray(values, dtype=float)
    samples = np.asarray(samples, dtype=float).reshape(-1)
    out = np.zeros((len(samples), values.shape[1]), dtype=float)
    for column in range(values.shape[1]):
        out[:, column] = np.interp(samples, axis, values[:, column])
    return out


def joint_motion_from_arc_profile(
    points: list[TaskPoint],
    q_rad: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    # 속도/가속도를 **시간 기반**으로 계산한다.
    #
    # 예전에는 호길이 연쇄법칙을 썼다:
    #     qd  = dq/ds · ṡ
    #     qdd = d²q/ds² · ṡ² + dq/ds · s̈
    # 이 식은 사다리꼴 프로파일의 양 끝(정지 구간)에서 **ds 가 붕괴**하면 무너진다.
    # ds 로 나누는 dq/ds 부터 병들고, d²q/ds² 는 1/ds² 로 더 심하게 폭주한다.
    #
    # 실측 (ES300h_hood row0, 갠트리 |a|max, 한계 5.0):
    #     stride   옛 식    시간기반
    #        8     0.512     0.497
    #        4     0.566     0.483
    #        2     0.761     0.485
    #        1     4.932     0.488     <- ds 중앙 1.00 mm, 끝점 부근 0.037 mm
    # 시간 기반은 stride 와 무관하게 일정하고 planner 의 max_acceleration=0.5 와 맞는다.
    #
    # 가속도만 시간 기반으로 바꾸는 것으로는 부족하다 — dq/ds 를 거쳐 만든 qd 를
    # 미분하면 그 spike 가 되살아난다(실측 5.418). 속도부터 시간 기반이어야 한다.
    #
    # 실기 명령 CSV(build_command_trajectory)는 자체 dq_ds 를 따로 계산하므로
    # 로봇에 나가는 값은 이 수정과 무관하다(6축 A/B 로 바이트 동일 확인).
    times = np.array([point.t for point in points], dtype=float)
    qd_rad = derivative_by_axis(times, q_rad)
    qdd_rad = derivative_by_axis(times, qd_rad)

    qd_rad[0] = 0.0
    qd_rad[-1] = 0.0
    qdd_rad[0] = 0.0
    qdd_rad[-1] = 0.0
    return qd_rad, qdd_rad


def natural_cubic_spline(axis: np.ndarray, values: np.ndarray):
    """열별 C2 natural cubic spline. samples -> (len(samples), columns) 를 돌려주는 함수를 만든다."""
    axis = np.asarray(axis, dtype=float).reshape(-1)
    values = np.asarray(values, dtype=float).reshape(len(axis), -1)
    count = len(axis)
    h = np.diff(axis)
    if count < 3:
        return lambda samples: interpolate_columns(axis, values, samples)
    system = np.zeros((count, count), dtype=float)
    rhs = np.zeros_like(values)
    system[0, 0] = system[-1, -1] = 1.0
    for i in range(1, count - 1):
        system[i, i - 1] = h[i - 1]
        system[i, i] = 2.0 * (h[i - 1] + h[i])
        system[i, i + 1] = h[i]
        rhs[i] = 6.0 * ((values[i + 1] - values[i]) / h[i] - (values[i] - values[i - 1]) / h[i - 1])
    second = np.linalg.solve(system, rhs)

    def evaluate(samples: np.ndarray) -> np.ndarray:
        samples = np.clip(np.asarray(samples, dtype=float).reshape(-1), axis[0], axis[-1])
        k = np.clip(np.searchsorted(axis, samples, side="right") - 1, 0, count - 2)
        hk = h[k].reshape(-1, 1)
        a = ((axis[k + 1] - samples).reshape(-1, 1)) / hk
        b = 1.0 - a
        return (
            a * values[k] + b * values[k + 1]
            + ((a ** 3 - a) * second[k] + (b ** 3 - b) * second[k + 1]) * hk ** 2 / 6.0
        )

    return evaluate


def matrix_to_quaternion_wxyz(rotation: np.ndarray) -> np.ndarray:
    # 대칭행렬 최대 고유벡터(Bar-Itzhack). trace 분기식과 달리 180 deg 근처에서도 안정하다.
    m = rotation
    k = np.array([
        [m[0, 0] + m[1, 1] + m[2, 2], m[2, 1] - m[1, 2], m[0, 2] - m[2, 0], m[1, 0] - m[0, 1]],
        [m[2, 1] - m[1, 2], m[0, 0] - m[1, 1] - m[2, 2], m[0, 1] + m[1, 0], m[0, 2] + m[2, 0]],
        [m[0, 2] - m[2, 0], m[0, 1] + m[1, 0], m[1, 1] - m[0, 0] - m[2, 2], m[1, 2] + m[2, 1]],
        [m[1, 0] - m[0, 1], m[0, 2] + m[2, 0], m[1, 2] + m[2, 1], m[2, 2] - m[0, 0] - m[1, 1]],
    ]) / 3.0
    _values, vectors = np.linalg.eigh(k)
    return vectors[:, -1]


def quaternions_wxyz_to_matrices(quaternions: np.ndarray) -> np.ndarray:
    w, x, y, z = quaternions.T
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], axis=1),
        np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], axis=1),
        np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], axis=1),
    ], axis=1)


def rotation_logs(rotations: np.ndarray) -> np.ndarray:
    """(N,3,3) 회전행렬의 회전벡터. 보정 잔차용이라 각도는 작다."""
    cos_angle = np.clip(0.5 * (np.trace(rotations, axis1=1, axis2=2) - 1.0), -1.0, 1.0)
    angle = np.arccos(cos_angle)
    vee = np.stack([
        rotations[:, 2, 1] - rotations[:, 1, 2],
        rotations[:, 0, 2] - rotations[:, 2, 0],
        rotations[:, 1, 0] - rotations[:, 0, 1],
    ], axis=1)
    scale = np.where(angle < 1.0e-6, 0.5, angle / (2.0 * np.sin(np.maximum(angle, 1.0e-6))))
    return vee * scale.reshape(-1, 1)


def tcp_exact_command_positions(
    model: KinematicModel,
    points: list[TaskPoint],
    q_rad: np.ndarray,
    command_arc: np.ndarray,
    iterations: int = 6,
) -> tuple[np.ndarray, float]:
    """명령 샘플마다 TCP 가 경로 위 호길이 s(t) 지점에 정확히 오도록 관절값을 보정한다.

    4 cm 노드 사이를 관절공간에서 선형보간하면 노드 평균속도는 맞아도 구간 안에서 TCP 속도가
    흔들린다(0.42 m/s 실측 -11 %..+17 %). 도장 품질상 row 안에서는 TCP 가 등속이어야 하므로
    목표 곡선을 NLP 해의 FK 로 만들고(노드의 경로방향 오차만 제거, 허용오차 안의 횡방향 오프셋과
    자세는 NLP 해 그대로), 그 곡선의 **실제 호길이**가 s(t) 가 되도록 샘플마다 최소노름 IK 로 맞춘다.
    돌려주는 두 번째 값은 남은 최대 위치 잔차 [m].
    """
    fk_fun = model.fk_function()
    source_arc = np.array([point.s for point in points], dtype=float)
    target_positions = np.array([point.position for point in points], dtype=float)
    count = len(points)

    node_positions = np.zeros((count, 3))
    node_rotations = np.zeros((count, 3, 3))
    for index, q in enumerate(q_rad):
        position, rotation_vec, _jac = fk_fun(q)
        node_positions[index] = np.asarray(position, dtype=float).reshape(3)
        node_rotations[index] = np.asarray(rotation_vec, dtype=float).reshape(3, 3)

    tangent = derivative_by_axis(source_arc, target_positions)
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1.0e-12)
    along = np.einsum("ij,ij->i", node_positions - target_positions, tangent)
    curve = natural_cubic_spline(source_arc, node_positions - along.reshape(-1, 1) * tangent)

    # 목표 곡선의 실제 호길이 L(u). 프로파일 s(t) 를 총길이 비로 옮겨 L 위의 등속/사다리꼴로 만든다.
    dense_u = np.unique(np.concatenate([np.linspace(a, b, 33) for a, b in zip(source_arc[:-1], source_arc[1:])]))
    dense_length = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(curve(dense_u), axis=0), axis=1))])
    span = max(source_arc[-1] - source_arc[0], 1.0e-12)
    command_length = (command_arc - source_arc[0]) * (dense_length[-1] / span)
    command_u = np.interp(command_length, dense_length, dense_u)

    goal_positions = curve(command_u)
    # 자세는 구간별 slerp 로 이으면 노드에서 각속도가 꺾여(C0) 손목축 가속도가 튄다(실측 joint_4 19,000 deg/s^2).
    # 쿼터니언을 부호 연속으로 맞춰 C2 spline + 정규화로 잇는다. 회전벡터 spline 은 범퍼처럼 row 하나가 부품을
    # 감아 돌 때(row 011 실측 250 deg) 0/pi 근처에서 축이 뒤집혀 쓸 수 없었다.
    node_quaternions = np.array([matrix_to_quaternion_wxyz(rotation) for rotation in node_rotations])
    for index in range(1, count):
        if np.dot(node_quaternions[index], node_quaternions[index - 1]) < 0.0:
            node_quaternions[index] = -node_quaternions[index]
    goal_quaternions = natural_cubic_spline(source_arc, node_quaternions)(command_u)
    goal_quaternions /= np.linalg.norm(goal_quaternions, axis=1, keepdims=True)
    goal_rotations = quaternions_wxyz_to_matrices(goal_quaternions)

    q_command = natural_cubic_spline(source_arc, q_rad)(command_u)
    samples = len(command_u)
    fk_map = fk_fun.map(samples)
    residual = 0.0
    for iteration in range(iterations + 1):
        positions, rotation_vecs, jacobians = fk_map(q_command.T)
        positions = np.asarray(positions, dtype=float).T
        rotation_vecs = np.asarray(rotation_vecs, dtype=float).T.reshape(samples, 3, 3)
        jacobians = np.asarray(jacobians, dtype=float).T.reshape(samples, model.nq, 6).transpose(0, 2, 1)
        position_error = goal_positions - positions
        residual = float(np.max(np.linalg.norm(position_error, axis=1)))
        if iteration == iterations or residual < 1.0e-9:
            break
        error = np.zeros((samples, 6))
        error[:, :3] = position_error
        error[:, 3:] = rotation_logs(goal_rotations @ rotation_vecs.transpose(0, 2, 1))
        gram = jacobians @ jacobians.transpose(0, 2, 1) + 1.0e-10 * np.eye(6)
        step = np.einsum("nij,nj->ni", jacobians.transpose(0, 2, 1), np.linalg.solve(gram, error[:, :, None])[:, :, 0])
        q_command = q_command + step
    return q_command, residual


def build_command_trajectory(
    points: list[TaskPoint],
    q_rad: np.ndarray,
    command_dt: float,
    model: KinematicModel | None = None,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    source_times = np.array([point.t for point in points], dtype=float)
    source_arc = np.array([point.s for point in points], dtype=float)
    source_speed = np.array([point.path_speed for point in points], dtype=float)

    dt = max(float(command_dt), 1.0e-4)
    duration = float(source_times[-1])
    command_times = np.arange(0.0, duration, dt, dtype=float)
    if command_times.size == 0 or not np.isclose(command_times[-1], duration, atol=1.0e-10):
        command_times = np.concatenate([command_times, [duration]])
    command_times = np.unique(command_times)

    command_arc = np.interp(command_times, source_times, source_arc)
    command_speed = np.interp(command_times, source_times, source_speed)
    dq_ds = derivative_by_axis(source_arc, q_rad)

    q_command = interpolate_columns(source_arc, q_rad, command_arc)
    dq_ds_command = interpolate_columns(source_arc, dq_ds, command_arc)
    qd_command = dq_ds_command * command_speed.reshape(-1, 1)
    if model is not None:
        # TCP 등속 보정(--command-interp tcp). qd 는 보정된 q 와 일치하도록 시간 미분으로 다시 만든다.
        # s(t) 를 노드 사이 선형보간하면 가감속 구간 속도가 계단이 되므로 노드 속도(path_speed)를 기울기로 쓰는
        # 3차 Hermite 로 잡는다 — 등속/등가속 구간은 정확히 재현되고 프로파일 꺾임 구간만 C1 로 근사된다.
        k = np.clip(np.searchsorted(source_times, command_times, side="right") - 1, 0, len(source_times) - 2)
        h = source_times[k + 1] - source_times[k]
        u = (command_times - source_times[k]) / h
        command_arc = (
            (2 * u ** 3 - 3 * u ** 2 + 1) * source_arc[k] + (u ** 3 - 2 * u ** 2 + u) * h * source_speed[k]
            + (-2 * u ** 3 + 3 * u ** 2) * source_arc[k + 1] + (u ** 3 - u ** 2) * h * source_speed[k + 1]
        )
        q_command, residual = tcp_exact_command_positions(model, points, q_rad, command_arc)
        qd_command = derivative_by_axis(command_times, q_command)
        print(f"[nlp] command TCP-exact refinement residual={residual * 1.0e6:.3f}um")
    qdd_command = finite_difference(command_times, qd_command)[0]
    qd_command[0] = 0.0
    qd_command[-1] = 0.0
    qdd_command[0] = 0.0
    qdd_command[-1] = 0.0
    return command_times, command_arc, q_command, qd_command, qdd_command


def optimize_trajectory(
    model: KinematicModel,
    points: list[TaskPoint],
    seed: np.ndarray,
    args: argparse.Namespace,
) -> np.ndarray:
    nq = model.nq
    count = len(points)
    times = np.array([point.t for point in points], dtype=float)
    if count < 2:
        raise ValueError("NLP needs at least two points")

    opti = ca.Opti()
    q_var = opti.variable(nq, count)
    objective = 0.0

    q_lower = ca.DM(model.lower)
    q_upper = ca.DM(model.upper)
    q_mid = ca.DM(0.5 * (model.lower + model.upper))
    q_range = ca.DM(np.maximum(model.upper - model.lower, 1.0e-6))

    urdf_velocity_limit = np.where(np.isfinite(model.velocity), model.velocity, np.inf)
    user_velocity_limit, acceleration_limit = model.motion_limits(args)
    velocity_limit = np.minimum(user_velocity_limit, urdf_velocity_limit)
    velocity_limit = np.where(np.isfinite(velocity_limit), velocity_limit, user_velocity_limit)
    # 노드(4 cm) 차분으로 거는 제약은 구간 평균이라, TCP 등속으로 촘촘히 만든 명령의 순간 최대값은 그보다 크다
    # (트렁크 row 004: 노드 70 deg/s·500 deg/s^2 -> 1 ms 명령 76.7·757). --node-limit-scale 로 NLP 쪽만 조인다.
    node_velocity_scale, node_acceleration_scale = parse_float_list(args.node_limit_scale, 2, "--node-limit-scale")
    v_limit_dm = ca.DM(velocity_limit * node_velocity_scale)
    a_limit_dm = ca.DM(acceleration_limit * node_acceleration_scale)
    # 갠트리 전용 부드러움 항 (직동축 컬럼만). 6축 모델은 빈 리스트라 항이 생기지 않는다.
    gantry_columns = [i for i in range(nq) if model.prismatic[i]]
    gantry_velocity_weight = float(args.gantry_velocity_weight)
    gantry_acceleration_weight = float(args.gantry_acceleration_weight)

    position_tol = float(args.position_tolerance)
    orientation_tol = math.radians(float(args.orientation_tolerance_deg))
    orientation_min_cos = math.cos(orientation_tol)

    tool_clearance_weight = float(args.tool_clearance_weight)
    if tool_clearance_weight > 0.0:
        tool_tip_offset = ca.DM(np.asarray(parse_float_list(args.tool_tip_offset, 3, "--tool-tip-offset")).reshape(3, 1))
        tool_clearance_joints = [name.strip() for name in args.tool_clearance_joints.split(",") if name.strip()]
        unknown = [name for name in tool_clearance_joints if name not in model.joint_names]
        if unknown:
            raise ValueError(f"--tool-clearance-joints: unknown joints {unknown} (have {model.joint_names})")
        clearance_sq = float(args.tool_clearance_m) ** 2

    for index, point in enumerate(points):
        qk = q_var[:, index]
        position, rotation, jacobian, joint_origins = model.fk_symbolic_full(qk)
        target_rotation = ca.DM(point.rotation)
        pos_err = position - ca.DM(point.position.reshape(3, 1))
        orientation_cos = ca_orientation_cosine(rotation, target_rotation)
        orientation_cost = 1.0 - orientation_cos

        opti.subject_to(opti.bounded(q_lower, qk, q_upper))
        opti.subject_to(opti.bounded(-position_tol, pos_err, position_tol))
        opti.subject_to(orientation_cos >= orientation_min_cos)

        objective += args.position_weight * ca.sumsqr(pos_err)
        objective += args.orientation_weight * orientation_cost
        objective += args.center_weight * ca.sumsqr((qk - q_mid) / q_range)

        if tool_clearance_weight > 0.0:
            tool_tip = position + rotation @ tool_tip_offset
            for name in tool_clearance_joints:
                # 부드러운 힌지 0.5*(x+sqrt(x^2+eps)): fmax 의 꺾임이 Ipopt 스텝 계산 오류(후드 row 025) 를 냈다.
                gap = clearance_sq - ca.sumsqr(tool_tip - joint_origins[name])
                violation = 0.5 * (gap + ca.sqrt(gap * gap + 1.0e-6))
                objective += tool_clearance_weight * violation ** 2

        if args.singularity_weight > 0.0:
            gram = jacobian @ jacobian.T
            regularized_gram = gram + 1.0e-8 * ca.DM.eye(6)
            objective += args.singularity_weight * ca.trace(
                ca.solve(regularized_gram, ca.DM.eye(6))
            )
        if args.arm_singularity_weight > 0.0:
            # 회전축(팔) 컬럼만. 갠트리 직동축을 넣으면 팔이 특이해도 6x8 은 full rank 라 항이 무력해진다.
            arm_columns = [i for i in range(nq) if not model.prismatic[i]]
            arm_jacobian = ca.horzcat(*[jacobian[:, i] for i in arm_columns])
            arm_gram = arm_jacobian @ arm_jacobian.T + 1.0e-6 * ca.DM.eye(6)
            objective += args.arm_singularity_weight * ca.trace(
                ca.solve(arm_gram, ca.DM.eye(6))
            )
        if args.min_manipulability > 0.0:
            raise ValueError(
                "--min-manipulability is disabled in this NLP path because "
                "CasADi's determinant node is not evaluable by this Ipopt build."
            )

    for index in range(count - 1):
        dt = times[index + 1] - times[index]
        if dt <= 0.0:
            raise ValueError("trajectory time must be strictly increasing")
        velocity = (q_var[:, index + 1] - q_var[:, index]) / dt
        opti.subject_to(opti.bounded(-v_limit_dm, velocity, v_limit_dm))
        objective += args.velocity_weight * ca.sumsqr(velocity / v_limit_dm)
        if gantry_columns and gantry_velocity_weight > 0.0:
            gantry_velocity = ca.vertcat(*[velocity[i] / v_limit_dm[i] for i in gantry_columns])
            objective += gantry_velocity_weight * ca.sumsqr(gantry_velocity)

    for index in range(1, count - 1):
        prev_dt = times[index] - times[index - 1]
        next_dt = times[index + 1] - times[index]
        prev_velocity = (q_var[:, index] - q_var[:, index - 1]) / prev_dt
        next_velocity = (q_var[:, index + 1] - q_var[:, index]) / next_dt
        acceleration = 2.0 * (next_velocity - prev_velocity) / (prev_dt + next_dt)
        opti.subject_to(opti.bounded(-a_limit_dm, acceleration, a_limit_dm))
        objective += args.acceleration_weight * ca.sumsqr(acceleration / a_limit_dm)
        if gantry_columns and gantry_acceleration_weight > 0.0:
            gantry_acceleration = ca.vertcat(*[acceleration[i] / a_limit_dm[i] for i in gantry_columns])
            objective += gantry_acceleration_weight * ca.sumsqr(gantry_acceleration)

    opti.minimize(objective)
    opti.set_initial(q_var, seed.T)
    opti.solver(
        "ipopt",
        {"expand": False},
        {
            "max_iter": int(args.ipopt_max_iter),
            "tol": float(args.ipopt_tol),
            "print_level": int(args.ipopt_print_level),
            "sb": "yes",
        },
    )

    print(f"[nlp] solving NLP points={count} joints={nq}")
    try:
        solution = opti.solve()
        q_solution = np.asarray(solution.value(q_var), dtype=float).T
    except RuntimeError:
        print("[nlp] solver failed")
        raise
    return q_solution


def evaluate_solution(
    model: KinematicModel,
    points: list[TaskPoint],
    q_solution: np.ndarray,
    args: argparse.Namespace | None = None,
) -> dict[str, float]:
    fk_fun = model.fk_function()
    clearance_fun = None
    if args is not None:
        names = [name.strip() for name in args.tool_clearance_joints.split(",") if name.strip()]
        if names and all(name in model.joint_names for name in names):
            clearance_fun = model.tool_clearance_function(
                parse_float_list(args.tool_tip_offset, 3, "--tool-tip-offset"), names)
    min_tool_clearance = math.inf
    max_pos_error = 0.0
    max_rot_error = 0.0
    min_manipulability = math.inf
    min_arm_sigma = math.inf
    arm_columns = np.flatnonzero(~model.prismatic)
    for point, q in zip(points, q_solution):
        position, rotation_vec, jacobian = fk_fun(q)
        position = np.asarray(position, dtype=float).reshape(3)
        rotation = np.asarray(rotation_vec, dtype=float).reshape(3, 3)
        jacobian = np.asarray(jacobian, dtype=float)
        max_pos_error = max(max_pos_error, float(np.linalg.norm(position - point.position)))
        orientation_cos = 0.5 * (float(np.trace(point.rotation.T @ rotation)) - 1.0)
        orientation_cos = float(np.clip(orientation_cos, -1.0, 1.0))
        max_rot_error = max(max_rot_error, math.degrees(math.acos(orientation_cos)))
        det_value = float(np.linalg.det(jacobian @ jacobian.T))
        min_manipulability = min(min_manipulability, math.sqrt(max(0.0, det_value)))
        # 팔(회전축)만의 최소 특이값. 8축 manipulability 는 갠트리가 팔 특이점을 가리므로 이걸 봐야 한다.
        arm_sigma = np.linalg.svd(jacobian[:, arm_columns], compute_uv=False)
        min_arm_sigma = min(min_arm_sigma, float(arm_sigma[-1]))
        if clearance_fun is not None:
            min_tool_clearance = min(min_tool_clearance, float(np.min(np.asarray(clearance_fun(q), dtype=float))))
    return {
        "max_position_error_m": max_pos_error,
        "max_orientation_error_deg": max_rot_error,
        "min_manipulability": min_manipulability,
        "min_arm_sigma": min_arm_sigma,
        "min_tool_clearance_m": min_tool_clearance,
    }


def write_joint_csv(
    output_csv: Path,
    times: np.ndarray,
    arc_lengths: np.ndarray,
    q_rad: np.ndarray,
    qd_rad: np.ndarray,
    qdd_rad: np.ndarray,
    prismatic: np.ndarray | None = None,
    joint_names: list[str] | None = None,
) -> None:
    """관절 궤적을 CSV로 저장한다.

    회전 전용(6축 실기) 모델이면 예전 그대로 전 축 deg 의 `q1..qN` 형식이다 —
    로봇 실행 경로가 이 형식을 읽으므로 바꾸면 안 된다.
    직동축이 섞인 8축 셀이면 deg 단일 단위를 쓸 수 없어 SI(회전 rad, 직동 m)로
    쓰고 컬럼 이름을 실제 관절 이름으로 준다. 미분값은 `qd_`/`qdd_` 접두어로
    구분해 paint_viz 재생 노드가 관절 컬럼과 헷갈리지 않게 한다
    (trajectory_ik_pinocchio.py 와 같은 규약).
    """
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    joint_count = q_rad.shape[1]
    if prismatic is None:
        prismatic = np.zeros(joint_count, dtype=bool)

    if bool(np.any(prismatic)):
        names = list(joint_names) if joint_names else [f"q{i + 1}" for i in range(joint_count)]
        header = ["t", "s"] + names
        header += [f"qd_{name}" for name in names]
        header += [f"qdd_{name}" for name in names]
        q_deg, qd_deg, qdd_deg = q_rad, qd_rad, qdd_rad
        _write_rows(output_csv, header, times, arc_lengths, q_deg, qd_deg, qdd_deg)
        return

    q_deg = np.degrees(q_rad)
    qd_deg = np.degrees(qd_rad)
    qdd_deg = np.degrees(qdd_rad)

    header = ["t", "s"]
    header += [f"q{index}" for index in range(1, q_rad.shape[1] + 1)]
    header += [f"qd{index}" for index in range(1, q_rad.shape[1] + 1)]
    header += [f"qdd{index}" for index in range(1, q_rad.shape[1] + 1)]

    _write_rows(output_csv, header, times, arc_lengths, q_deg, qd_deg, qdd_deg)


def _write_rows(output_csv, header, times, arc_lengths, q, qd, qdd) -> None:
    with output_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        for index, t in enumerate(times):
            writer.writerow(
                [f"{t:.9f}", f"{arc_lengths[index]:.9f}"]
                + [f"{value:.9f}" for value in q[index]]
                + [f"{value:.9f}" for value in qd[index]]
                + [f"{value:.9f}" for value in qdd[index]]
            )


def output_path_for(input_csv: Path, output: Path, multiple: bool) -> Path:
    output = output.expanduser().resolve()
    if multiple or output.suffix.lower() != ".csv":
        return output / f"{input_csv.stem}_nlp_joint.csv"
    return output


def command_output_path_for(input_csv: Path, output: Path, multiple: bool) -> Path:
    output = output.expanduser().resolve()
    if multiple or output.suffix.lower() != ".csv":
        return output / f"{input_csv.stem}_nlp_command.csv"
    return output.with_name(f"{output.stem}_command{output.suffix}")


def trajectory_duration_sec(csv_path: Path) -> float:
    with csv_path.open("r", newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        last_t = None
        for row in reader:
            value = str(row.get("t", "")).strip()
            if value:
                last_t = float(value)
    if last_t is None:
        raise ValueError(f"{csv_path} does not contain trajectory rows")
    return max(last_t, 0.0)


def run_robot(outputs: list[Path], args: argparse.Namespace) -> None:
    # Compatibility with the original --execute switch; execution remains in robot_controller.
    control_scripts = PROJECT_ROOT / "robot_controller/doosan_a0912_controller/test/scripts"
    sys.path.insert(0, str(control_scripts))
    from trajectory_execution import run_robot as execute
    execute(outputs, args)


# --- 갠트리 환경 (paint_viz config/gantry/<name>.yaml) ----------------------------------------
# --gantry-env 를 주면 URDF 기본값과 plan_metadata.json 의 환경 치수를 거기서 가져온다. paint_viz 가
# import 되지 않는 환경(실기 6축 전용 PC)에서는 yaml 경로를 직접 줄 수 있고, 없으면 None 으로 둔다.
def load_gantry_env_or_none(name_or_path: str | None):
    if not name_or_path:
        return None
    try:
        from motion_planner.configuration import load_gantry_env
        return load_gantry_env(name_or_path)
    except ImportError:
        path = Path(name_or_path).expanduser()
        if not path.is_file():
            raise RuntimeError(
                f"--gantry-env {name_or_path!r}: paint_viz 를 import 할 수 없고 yaml 경로도 아니다 "
                "(paint_cell_ws 를 source 하거나 yaml 경로를 직접 줄 것)")
        import yaml

        class _RawEnv:
            def __init__(self, data, path):
                self.data = data
                self.path = path
                self.name = str(data.get("name") or path.stem)

            def get(self, dotted, default=None):
                node = self.data
                for key in dotted.split("."):
                    if not isinstance(node, dict) or key not in node:
                        return default
                    node = node[key]
                return node

            def urdf_path(self, kind="primitive"):
                return path.parents[2] / "urdf" / str(self.data["urdf"][kind])

            def metadata(self):
                return {"name": self.name, "config_file": str(self.path), "raw": self.data}

        with open(path, "r", encoding="utf-8") as stream:
            loaded = yaml.safe_load(stream) or {}
        return _RawEnv(loaded.get("gantry_env", loaded), path)


def write_plan_metadata(output_dir: Path, script_name: str, args: argparse.Namespace, env,
                        urdf_path: Path, joint_names: list[str], inputs: list[Path],
                        outputs: list[Path], extra: dict | None = None) -> Path:
    """출력 디렉터리에 plan_metadata.json 을 쓴다 — 어떤 환경/URDF/배치로 만든 CSV 인지 추적용."""
    import datetime
    import json

    def jsonable(value):
        if isinstance(value, Path):
            return str(value)
        if isinstance(value, np.ndarray):
            return value.tolist()
        if isinstance(value, (list, tuple)):
            return [jsonable(v) for v in value]
        if isinstance(value, dict):
            return {str(k): jsonable(v) for k, v in value.items()}
        if isinstance(value, (np.floating, np.integer)):
            return value.item()
        return value

    skip = {"csv", "execute"}
    metadata = {
        "script": script_name,
        "generated_at": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        "gantry_env": env.metadata() if env is not None else None,
        "urdf": str(urdf_path),
        "tcp_frame": args.tcp_frame,
        "joint_names": list(joint_names),
        "part_xyz": [float(v) for v in str(args.part_xyz).split(",")],
        "part_rpy": [float(v) for v in str(args.part_rpy).split(",")],
        "initial_joint": args.initial_joint_deg,
        "inputs": [str(p) for p in inputs],
        "outputs": [str(p) for p in outputs],
        "args": {k: jsonable(v) for k, v in sorted(vars(args).items())
                 if k not in skip and not k.endswith("_obj") and not isinstance(v, np.ndarray)},
    }
    if extra:
        metadata.update(jsonable(extra))
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / "plan_metadata.json"
    with open(path, "w", encoding="utf-8") as stream:
        json.dump(metadata, stream, ensure_ascii=False, indent=2)
    return path


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Offline CasADi NLP trajectory optimization for A0912 Cartesian painting CSVs."
    )
    parser.add_argument("csv", nargs="*", type=Path, help="Input trajectory_XXX.csv file(s).")
    parser.add_argument("--urdf", type=Path, default=None,
                        help=f"URDF. 기본: --gantry-env 가 있으면 그 환경의 8축 URDF, 없으면 {DEFAULT_URDF.name} (6축)")
    parser.add_argument("--gantry-env", default=None,
                        help="갠트리 환경 이름(legacy|spec_c|x7y4z3) 또는 config/gantry/*.yaml 경로. "
                             "URDF 기본값과 plan_metadata.json 의 환경 정보에 쓴다")
    parser.add_argument("--tcp-frame", default=TCP_FRAME)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--max-points", type=int, default=0)
    parser.add_argument("--initial-joint-deg", default=INITIAL_JOINT_DEG)
    parser.add_argument("--position-tolerance", type=float, default=POSITION_TOLERANCE_M)
    parser.add_argument("--orientation-tolerance-deg", type=float, default=ORIENTATION_TOLERANCE_DEG)
    parser.add_argument("--tcp-spray-axis", default=TCP_SPRAY_AXIS)
    parser.add_argument("--tcp-path-axis", default=TCP_PATH_AXIS)
    parser.add_argument("--max-joint-step-deg", type=float, default=MAX_JOINT_STEP_DEG)
    parser.add_argument("--max-joint-velocity-deg-s", type=float, default=MAX_JOINT_VELOCITY_DEG_S)
    parser.add_argument("--max-joint-acceleration-deg-s2", type=float, default=MAX_JOINT_ACCELERATION_DEG_S2)
    parser.add_argument("--max-gantry-velocity-m-s", default=AUTO,
                        help="갠트리 직동축 속도 한계 [m/s] (8축 셀 URDF에서만 쓰임). "
                             f"auto = gantry_env motor_limits.gantry_velocity_m_s, 없으면 {MAX_GANTRY_VELOCITY_M_S}")
    parser.add_argument("--max-gantry-acceleration-m-s2", default=AUTO,
                        help="갠트리 직동축 가속도 한계 [m/s^2]. "
                             f"auto = gantry_env motor_limits.gantry_acceleration_m_s2, 없으면 {MAX_GANTRY_ACCELERATION_M_S2}")
    parser.add_argument("--gantry-velocity-weight", default=AUTO,
                        help="갠트리 직동축 속도 부드러움 가중치 (기존 --velocity-weight 에 더해 직동축에만 추가). "
                             f"auto = gantry_env nlp.gantry_velocity_weight, 없으면 {GANTRY_VELOCITY_WEIGHT}. 0 = 끔")
    parser.add_argument("--gantry-acceleration-weight", default=AUTO,
                        help="갠트리 직동축 가속도 부드러움 가중치. "
                             f"auto = gantry_env nlp.gantry_acceleration_weight, 없으면 {GANTRY_ACCELERATION_WEIGHT}. 0 = 끔")
    parser.add_argument("--part-xyz", default=PART_XYZ,
                        help="부품 스캔 좌표계를 로봇 월드로 옮기는 평행이동 'x,y,z' [m]")
    parser.add_argument("--part-rpy", default=PART_RPY,
                        help="부품 스캔 좌표계를 로봇 월드로 옮기는 고정축 회전 'r,p,y' [rad]")
    parser.add_argument("--min-manipulability", type=float, default=MIN_MANIPULABILITY)
    parser.add_argument("--position-weight", type=float, default=POSITION_WEIGHT)
    parser.add_argument("--orientation-weight", type=float, default=ORIENTATION_WEIGHT)
    parser.add_argument("--velocity-weight", type=float, default=VELOCITY_WEIGHT)
    parser.add_argument("--acceleration-weight", type=float, default=ACCELERATION_WEIGHT)
    parser.add_argument("--center-weight", type=float, default=CENTER_WEIGHT)
    parser.add_argument("--singularity-weight", type=float, default=SINGULARITY_WEIGHT)
    parser.add_argument("--arm-singularity-weight", default=AUTO,
                        help="trace((Ja Ja^T)^-1) on the revolute-only (arm) Jacobian columns. "
                             f"'auto' = {ARM_SINGULARITY_WEIGHT} for gantry models, 0 for the bare 6-axis arm. 0 disables.")
    parser.add_argument("--joint-bounds-deg", default=AUTO,
                        help='Override joint bounds, e.g. "joint_3=25:135,joint_5=25:135" '
                             f'(deg for revolute, m for prismatic). "auto" = "{ARM_JOINT_BOUNDS_DEG}" '
                             'for gantry models, none for the bare 6-axis arm. Empty string disables.')
    parser.add_argument("--tool-clearance-weight", type=float, default=TOOL_CLEARANCE_WEIGHT,
                        help="Soft penalty weight keeping the tool tip point away from the listed joint origins "
                             "(sum of max(0, c^2 - d^2)^2). 0 disables.")
    parser.add_argument("--tool-clearance-m", type=float, default=TOOL_CLEARANCE_M,
                        help="Required distance [m] between the tool tip point and each listed joint origin.")
    parser.add_argument("--tool-tip-offset", default=TOOL_TIP_OFFSET_M,
                        help='Tool tip point in the TCP frame, "x,y,z" [m]. Default = paint cup end of the spray gun.')
    parser.add_argument("--tool-clearance-joints", default=TOOL_CLEARANCE_JOINTS,
                        help="Comma-separated joint names whose origins the tool tip must clear (joint_3 = elbow, joint_5 = wrist).")
    parser.add_argument("--ipopt-max-iter", type=int, default=IPOPT_MAX_ITER)
    parser.add_argument("--ipopt-tol", type=float, default=IPOPT_TOL)
    parser.add_argument("--ipopt-print-level", type=int, default=IPOPT_PRINT_LEVEL)
    parser.add_argument("--execute", action="store_true", help="Send optimized joint CSV to the robot after solving.")
    parser.add_argument("--command-dt", type=float, default=COMMAND_TIME_SEC)
    parser.add_argument("--node-limit-scale", default=AUTO,
                        help="NLP 노드 속도/가속도 제약에 곱하는 배율 'v,a'. 촘촘한 명령의 순간 최대값이 "
                             "실제 한계 안에 들도록 NLP 만 조일 때 쓴다 (예: 0.85,0.65)")
    parser.add_argument("--command-interp", choices=("tcp", "linear"), default="tcp",
                        help="tcp: 명령 샘플마다 TCP 를 경로 호길이 s(t) 에 맞춰 row 안 TCP 등속 보장(기본). "
                             "linear: 예전처럼 노드 사이 관절공간 선형보간")
    parser.add_argument("--command-time-sec", type=float, default=COMMAND_TIME_SEC)
    parser.add_argument("--joint-position-gain", type=float, default=JOINT_POSITION_GAIN)
    parser.add_argument(
        "--preposition-max-joint-velocity-deg-s",
        type=float,
        default=PREPOSITION_MAX_JOINT_VELOCITY_DEG_S,
    )
    parser.add_argument(
        "--preposition-max-joint-acceleration-deg-s2",
        type=float,
        default=PREPOSITION_MAX_JOINT_ACCELERATION_DEG_S2,
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    input_csvs = [path.expanduser().resolve() for path in args.csv]
    if not input_csvs:
        input_csvs = [path.resolve() for path in DEFAULT_TRAJECTORY_CSV]
    for csv_path in input_csvs:
        if not csv_path.is_file():
            raise FileNotFoundError(csv_path)
    gantry_env = load_gantry_env_or_none(args.gantry_env)
    if args.urdf is None:
        args.urdf = Path(gantry_env.urdf_path("primitive")) if gantry_env is not None else DEFAULT_URDF
    urdf_path = args.urdf.expanduser().resolve()
    if not urdf_path.is_file():
        raise FileNotFoundError(urdf_path)
    if gantry_env is not None:
        print(f"[nlp] gantry env: {gantry_env.name} ({gantry_env.path})")
    parse_signed_axis(args.tcp_spray_axis)
    parse_signed_axis(args.tcp_path_axis)
    if parse_signed_axis(args.tcp_spray_axis)[0] == parse_signed_axis(args.tcp_path_axis)[0]:
        raise ValueError("--tcp-spray-axis and --tcp-path-axis must use different axes")

    args.tcp_frame = resolve_tcp_frame(urdf_path, args.tcp_frame)
    print(f"[nlp] TCP frame: {args.tcp_frame}")
    chain = parse_urdf_chain(urdf_path, args.tcp_frame)
    model = KinematicModel(chain)
    has_gantry = bool(np.any(model.prismatic))
    if args.joint_bounds_deg == AUTO:
        args.joint_bounds_deg = ARM_JOINT_BOUNDS_DEG if has_gantry else ""
    if args.arm_singularity_weight == AUTO:
        args.arm_singularity_weight = ARM_SINGULARITY_WEIGHT if has_gantry else 0.0
    args.arm_singularity_weight = float(args.arm_singularity_weight)
    apply_joint_bound_overrides(model, args.joint_bounds_deg)
    print(f"[nlp] arm singularity weight: {args.arm_singularity_weight:g}")

    def resolve_auto(value, env_key, fallback):
        """auto → gantry_env yaml 값, yaml 에 없으면 스크립트 상수. 명시값은 그대로 float."""
        if value != AUTO:
            return float(value), "cli"
        env_value = gantry_env.get(env_key) if gantry_env is not None else None
        if env_value is None:
            return float(fallback), "default"
        return float(env_value), "env"

    args.max_gantry_velocity_m_s, v_src = resolve_auto(
        args.max_gantry_velocity_m_s, "motor_limits.gantry_velocity_m_s", MAX_GANTRY_VELOCITY_M_S)
    args.max_gantry_acceleration_m_s2, a_src = resolve_auto(
        args.max_gantry_acceleration_m_s2, "motor_limits.gantry_acceleration_m_s2", MAX_GANTRY_ACCELERATION_M_S2)
    args.gantry_velocity_weight, wv_src = resolve_auto(
        args.gantry_velocity_weight, "nlp.gantry_velocity_weight", GANTRY_VELOCITY_WEIGHT)
    args.gantry_acceleration_weight, wa_src = resolve_auto(
        args.gantry_acceleration_weight, "nlp.gantry_acceleration_weight", GANTRY_ACCELERATION_WEIGHT)
    if args.node_limit_scale == AUTO:
        # 환경 yaml nlp.node_limit_scale (legacy "0.85,0.50", x7y4z3/spec_c "0.85,0.25"), 없으면 "1,1" (= 종전 기본값)
        env_scale = gantry_env.get("nlp.node_limit_scale") if gantry_env is not None else None
        args.node_limit_scale = "1,1" if env_scale is None else str(env_scale)
        print(f"[nlp] node limit scale: {args.node_limit_scale} ({'env' if env_scale is not None else 'default'})")
    if has_gantry:
        print(f"[nlp] gantry limits: v {args.max_gantry_velocity_m_s:g} m/s ({v_src}), "
              f"a {args.max_gantry_acceleration_m_s2:g} m/s^2 ({a_src}); gantry smoothness weights: "
              f"v {args.gantry_velocity_weight:g} ({wv_src}), a {args.gantry_acceleration_weight:g} ({wa_src})")
    if float(args.tool_clearance_weight) > 0.0:
        print(f"[nlp] tool clearance: tip {args.tool_tip_offset} (TCP frame) >= {args.tool_clearance_m} m "
              f"from {args.tool_clearance_joints}, weight {args.tool_clearance_weight:g}")
    if model.nq != 6 and not has_gantry:
        raise ValueError(
            f"expected a 6-joint arm or a gantry model with prismatic joints, "
            f"but URDF chain has {model.nq}: {model.joint_names}"
        )
    if args.execute and model.nq != 6:
        # DRFL 은 두산 6축만 명령한다. 갠트리 직동축은 별도 구동계라 이 스크립트로 못 움직인다.
        raise ValueError(
            f"--execute is only supported for the 6-joint arm (nq={model.nq}). "
            "The gantry prismatic axes need their own drive; this script cannot command them."
        )

    # 회전축은 deg, 직동축은 m 로 적는다.
    if (gantry_env is not None and model.nq != 6 and args.initial_joint_deg == INITIAL_JOINT_DEG
            and gantry_env.get("nlp.initial_joint")):
        # 6축 기본값을 8축 환경에 그대로 쓰면 개수가 안 맞는다 — env yaml 의 초기 자세(갠트리 중앙)를 쓴다
        args.initial_joint_deg = str(gantry_env.get("nlp.initial_joint"))
        print(f"[nlp] initial joint from gantry env: {args.initial_joint_deg}")
    initial_values = np.array(parse_float_list(args.initial_joint_deg, model.nq, "--initial-joint-deg"))
    initial_q = np.where(model.prismatic, initial_values, np.radians(initial_values))
    seed_initial_q = initial_q.copy()
    max_seed_step_rad = math.radians(max(1.0, min(args.max_joint_step_deg, 15.0)))

    part_translation = np.array(parse_float_list(args.part_xyz, 3, "--part-xyz"), dtype=float)
    part_rpy = parse_float_list(args.part_rpy, 3, "--part-rpy")
    part_rotation = rpy_to_matrix_np(np.array(part_rpy, dtype=float))

    print(f"[nlp] URDF: {urdf_path}")
    print(f"[nlp] joints: {', '.join(model.joint_names)}")
    print(f"[nlp] prismatic joints: {int(np.sum(model.prismatic))} of {model.nq}")
    print(f"[nlp] TCP spray axis: {args.tcp_spray_axis} path axis: {args.tcp_path_axis}")
    print(f"[nlp] part placement xyz={part_translation.tolist()} rpy={part_rpy}")
    print(f"[nlp] output: {args.output}")

    v_limit, a_limit = model.motion_limits(args)
    total_start = time.perf_counter()
    timing_rows: list[tuple[str, int, float, float, float]] = []
    command_outputs = []
    joint_outputs = []
    for csv_index, input_csv in enumerate(input_csvs, start=1):
        print(f"[nlp] input #{csv_index}/{len(input_csvs)}: {input_csv}")
        points = load_task_points(
            input_csv,
            stride=args.stride,
            max_points=args.max_points,
            tcp_spray_axis=args.tcp_spray_axis,
            tcp_path_axis=args.tcp_path_axis,
            part_rotation=part_rotation,
            part_translation=part_translation,
        )
        print(
            f"[nlp] loaded points={len(points)} duration={points[-1].t:.3f}s "
            f"stride={args.stride} max_points={args.max_points}"
        )

        seed_start = time.perf_counter()
        seed = seed_path(model, points, seed_initial_q, max_seed_step_rad)
        seed_elapsed = time.perf_counter() - seed_start
        nlp_start = time.perf_counter()
        q_solution = optimize_trajectory(model, points, seed, args)
        nlp_elapsed = time.perf_counter() - nlp_start
        timing_rows.append((input_csv.name, len(points), seed_elapsed, nlp_elapsed, points[-1].t))
        print(
            f"[nlp][time] seed {seed_elapsed:.2f}s + NLP {nlp_elapsed:.2f}s "
            f"= {seed_elapsed + nlp_elapsed:.2f}s for {len(points)} points "
            f"({1000.0 * (seed_elapsed + nlp_elapsed) / max(len(points), 1):.1f} ms/point), "
            f"path duration {points[-1].t:.2f}s"
        )
        seed_initial_q = q_solution[-1].copy()
        metrics = evaluate_solution(model, points, q_solution, args)
        times = np.array([point.t for point in points], dtype=float)
        arc_lengths = np.array([point.s for point in points], dtype=float)
        qd_solution, qdd_solution = joint_motion_from_arc_profile(points, q_solution)
        print(
            "[nlp] metrics "
            f"max_pos_err={metrics['max_position_error_m']:.6f}m "
            f"max_rot_err={metrics['max_orientation_error_deg']:.4f}deg "
            f"min_manip={metrics['min_manipulability']:.6e} "
            f"min_arm_sigma={metrics['min_arm_sigma']:.4f} "
            f"tool_clearance={metrics['min_tool_clearance_m']:.3f}m "
            f"qdot[{worst_axis_detail(qd_solution, v_limit, model.prismatic, '/s')}] "
            f"qddot[{worst_axis_detail(qdd_solution, a_limit, model.prismatic, '/s^2')}]"
        )

        output_csv = output_path_for(input_csv, args.output, multiple=len(input_csvs) > 1)
        write_joint_csv(output_csv, times, arc_lengths, q_solution, qd_solution, qdd_solution,
                        prismatic=model.prismatic, joint_names=model.joint_names)
        print(f"[nlp] saved optimized joint trajectory: {output_csv}")

        command_times, command_arc, command_q, command_qd, command_qdd = build_command_trajectory(
            points,
            q_solution,
            args.command_dt,
            model=model if args.command_interp == "tcp" else None,
        )
        command_csv = command_output_path_for(input_csv, args.output, multiple=len(input_csvs) > 1)
        write_joint_csv(command_csv, command_times, command_arc, command_q, command_qd, command_qdd,
                        prismatic=model.prismatic, joint_names=model.joint_names)
        command_v_ratio = float(np.max(np.abs(command_qd) / v_limit.reshape(1, -1)))
        command_a_ratio = float(np.max(np.abs(command_qdd) / a_limit.reshape(1, -1)))
        print(
            f"[nlp] command limits qdot[{worst_axis_detail(command_qd, v_limit, model.prismatic, '/s')}] "
            f"qddot[{worst_axis_detail(command_qdd, a_limit, model.prismatic, '/s^2')}]"
        )
        if command_v_ratio > 1.0 or command_a_ratio > 1.0:
            print(
                f"[nlp] WARNING command exceeds joint limits (velocity x{command_v_ratio:.2f}, "
                f"acceleration x{command_a_ratio:.2f}) - lower --node-limit-scale or raise the limits"
            )
        command_qd_deg = np.degrees(command_qd)
        command_qdd_deg = np.degrees(command_qdd)
        print(
            f"[nlp] saved {args.command_dt:.4f}s speedj command reference: {command_csv} "
            f"points={len(command_times)} max_qdot={np.max(np.abs(command_qd_deg)):.3f}deg/s "
            f"max_qddot={np.max(np.abs(command_qdd_deg)):.3f}deg/s^2"
        )
        command_outputs.append(command_csv)
        joint_outputs.append(output_csv)

    if timing_rows:
        total_elapsed = time.perf_counter() - total_start
        seed_total = sum(r[2] for r in timing_rows)
        nlp_total = sum(r[3] for r in timing_rows)
        point_total = sum(r[1] for r in timing_rows)
        path_total = sum(r[4] for r in timing_rows)
        plan_total = seed_total + nlp_total
        print("[nlp][time] planning cost")
        for name, count, seed_s, nlp_s, duration in timing_rows:
            print(
                f"[nlp][time]   {name:28s} points={count:5d} seed={seed_s:6.2f}s "
                f"NLP={nlp_s:8.2f}s total={seed_s + nlp_s:8.2f}s "
                f"({1000.0 * (seed_s + nlp_s) / max(count, 1):7.1f} ms/pt) "
                f"path={duration:6.2f}s ratio={(seed_s + nlp_s) / max(duration, 1.0e-9):7.2f}x"
            )
        print(
            f"[nlp][time]   {'TOTAL':28s} points={point_total:5d} seed={seed_total:6.2f}s "
            f"NLP={nlp_total:8.2f}s total={plan_total:8.2f}s "
            f"({1000.0 * plan_total / max(point_total, 1):7.1f} ms/pt) "
            f"path={path_total:6.2f}s ratio={plan_total / max(path_total, 1.0e-9):7.2f}x"
        )
        print(f"[nlp][time]   wall clock (I/O + 후처리 포함) {total_elapsed:.2f}s")

    metadata_path = write_plan_metadata(
        args.output, Path(__file__).name, args, gantry_env, urdf_path, model.joint_names,
        input_csvs, joint_outputs + command_outputs, extra={"joint_bounds_deg": args.joint_bounds_deg, "arm_singularity_weight": args.arm_singularity_weight,
               "gantry_limits": {"velocity_m_s": args.max_gantry_velocity_m_s,
                                 "acceleration_m_s2": args.max_gantry_acceleration_m_s2},
               "gantry_smoothness_weights": {"velocity": args.gantry_velocity_weight,
                                             "acceleration": args.gantry_acceleration_weight}})
    print(f"[nlp] saved plan metadata: {metadata_path}")

    if args.execute:
        run_robot(command_outputs, args)
    else:
        print("[nlp] dry run complete. Add --execute to track the optimized command CSV on the robot.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
