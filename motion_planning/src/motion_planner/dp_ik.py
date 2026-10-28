"""DP-IK 시각화 솔버 — 후보 생성 + DP 경로 선택을 8축 셀에서 돌린다.

`robot_controller/.../trajectory_ik_pinocchio.py` 의 IK 코어(다중 시드 DLS →
후보 필터링 → 점프/속도/가속도 실현성 DP)를 **그대로 옮겨** RViz 재생용 joint
CSV 를 만든다. 계산 결과가 원본과 달라지지 않도록 비용식·게이트·시드 오프셋·
가중치 기본값을 전부 동일하게 유지했다.

`viz_ik.py`(solve_viz_ik)와의 차이:

  - solve_viz_ik 는 이전 해를 warm start 로 쓰는 DLS IK **한 번**이다. 빠르지만
    관절 점프/속도/가속도를 전혀 보지 않는다.
  - 이쪽은 점마다 여러 분기해를 만들어 놓고 DP 로 **실현 가능한 경로**를 고른다.
    그래서 느리고, 게이트를 통과 못 하면 그 row 는 해가 없다.

원본과 다른 점(시각화 목적):

  1. 입력이 CSV 목록이 아니라 `config/scene.yaml` 의 `trajectory_dir` 전체다.
     부품 배치(part_xyz/part_rpy)도 같은 파일에서 읽어 RViz 표시와 어긋나지 않는다.
  2. 출력이 SI joint CSV(`t, s, <관절이름들>`) 하나뿐이다. 1 kHz 명령 CSV 와
     로봇 실행(`--execute`)은 없다. **실기 실행용이 아니다.**
  3. DP 가 막힌 row 에서 죽지 않는다(`--on-failure skip`). 풀린 row 만 저장하고
     막힌 지점을 요약에 남긴다 — 어디서 막히는지 보는 게 이 도구의 용도다.
  4. row 경계의 IK 분기 갈림(README "입력은 NLP 결과를 쓸 것" 참고)을 측정해
     요약에 남긴다.
"""

import argparse
import csv
import math
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pinocchio as pin

from motion_planner.configuration import add_gantry_env_argument, apply_gantry_env
from motion_planner.scene_config import load_scene_config, part_transform
from motion_planner.tcp_frame import resolve_tcp_frame
from motion_planner.data_io import find_trajectory_csv_files, read_trajectory_csv

DEFAULT_TCP_FRAME = "auto"  # tcp > spray_gun > link_6 중 URDF 에 있는 것 (paint_viz.tcp_frame)
DEFAULT_TCP_SPRAY_AXIS = "z"
DEFAULT_TCP_PATH_AXIS = "x"

# 8축(갠트리 2 + 팔 6) 기본 시드. 직동축은 m, 회전축은 deg. viz_ik.py 와 같은 값.
DEFAULT_INITIAL_JOINT = "0,0,0,0,90,0,90,0"

POSITION_TOLERANCE_M = 0.0001
ORIENTATION_TOLERANCE_DEG = 1.0
MAX_JOINT_STEP_DEG = 20.0
MAX_JOINT_VELOCITY_DEG_S = 70.0
MAX_JOINT_ACCELERATION_DEG_S2 = 500.0
MIN_MANIPULABILITY = 0.0
MIN_SINGULAR_VALUE = 0.01

# 갠트리 직동축 한계. 회전축과 단위가 달라 따로 둔다.
MAX_GANTRY_STEP_M = 0.05
MAX_GANTRY_VELOCITY_M_S = 1.0
MAX_GANTRY_ACCELERATION_M_S2 = 5.0
IK_DUPLICATE_CANDIDATE_M = 0.005

IK_MAX_ITER = 100
IK_DAMPING = 0.03
IK_MAX_STEP_DEG = 5.0
IK_POSITION_GAIN = 1.0
IK_ORIENTATION_GAIN = 1.0
IK_PROGRESS_INTERVAL = 25
IK_MAX_SEEDS_PER_POINT = 48
IK_MAX_CANDIDATES_PER_POINT = 12
IK_DUPLICATE_CANDIDATE_DEG = 0.5

# A0912 6축 팔의 손목/팔꿈치 뒤집기를 노린 시드 오프셋. 회전축에만 의미가 있다.
IK_SEED_OFFSETS_DEG = (
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    (0.0, 0.0, 0.0, 45.0, 0.0, -45.0),
    (0.0, 0.0, 0.0, -45.0, 0.0, 45.0),
    (0.0, 0.0, 0.0, 90.0, 0.0, -90.0),
    (0.0, 0.0, 0.0, -90.0, 0.0, 90.0),
    (0.0, 0.0, 0.0, 180.0, 0.0, -180.0),
    (0.0, 0.0, 0.0, -180.0, 0.0, 180.0),
    (0.0, 0.0, 0.0, 0.0, 20.0, 0.0),
    (0.0, 0.0, 0.0, 0.0, -20.0, 0.0),
    (0.0, 60.0, -60.0, 0.0, 0.0, 0.0),
    (0.0, -60.0, 60.0, 0.0, 0.0, 0.0),
    (180.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    (-180.0, 0.0, 0.0, 0.0, 0.0, 0.0),
)

DP_TRACKING_WEIGHT = 0.1
DP_JUMP_WEIGHT = 1.0
DP_VELOCITY_WEIGHT = 1.0
DP_ACCELERATION_WEIGHT = 1.0
DP_SINGULARITY_WEIGHT = 0.2
DP_LIMIT_MARGIN_WEIGHT = 0.02


@dataclass
class TaskPoint:
    t: float
    s: float
    position: np.ndarray
    rotation: np.ndarray


@dataclass
class IkPointResult:
    q: np.ndarray
    iterations: int
    position_error_m: float
    orientation_error_deg: float
    converged: bool


@dataclass
class IkCandidate:
    q: np.ndarray
    position_error_m: float
    orientation_error_deg: float
    min_sigma: float
    manipulability: float
    iterations: int
    seed_index: int


# ---------------------------------------------------------------- 좌표/자세 변환


def parse_signed_axis(axis):
    value = str(axis).strip().lower()
    sign = 1.0
    if value.startswith("+"):
        value = value[1:]
    elif value.startswith("-"):
        value = value[1:]
        sign = -1.0
    if value not in ("x", "y", "z"):
        raise ValueError(f"축 표기가 잘못됐다: {axis} (x, y, z, -x, -y, -z 중 하나)")
    return "xyz".index(value), sign


def quaternion_xyzw_to_matrix(quaternion):
    q = np.asarray(quaternion, dtype=float).reshape(4)
    norm = float(np.linalg.norm(q))
    if norm < 1.0e-12:
        return np.eye(3)
    x, y, z, w = q / norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
        [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
        [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
    ])


def remap_planner_rotation_to_tcp(planner_rotation, tcp_spray_axis, tcp_path_axis):
    """planner 회전(0열 = 경로 진행축, 2열 = 분사축)을 TCP 회전으로 옮긴다.

    trajectory_ik_pinocchio.remap_planner_rotation_to_tcp 와 같은 규약이다.
    """
    spray_index, spray_sign = parse_signed_axis(tcp_spray_axis)
    path_index, path_sign = parse_signed_axis(tcp_path_axis)
    if spray_index == path_index:
        raise ValueError("분사축과 경로축은 서로 달라야 한다")

    rotation = np.asarray(planner_rotation, dtype=float).reshape(3, 3)
    planner_path = rotation[:, 0]
    planner_spray = rotation[:, 2]

    axes = [None, None, None]
    axes[spray_index] = spray_sign * planner_spray

    projected = planner_path - float(np.dot(planner_path, axes[spray_index])) * axes[spray_index]
    norm = float(np.linalg.norm(projected))
    if norm <= 1.0e-9:
        projected = rotation[:, 1]
        projected = projected - float(np.dot(projected, axes[spray_index])) * axes[spray_index]
        norm = float(np.linalg.norm(projected))
    if norm <= 1.0e-9:
        raise ValueError("planner 회전에서 TCP 목표 회전을 만들 수 없다")
    axes[path_index] = path_sign * (projected / norm)

    missing = next(index for index, axis in enumerate(axes) if axis is None)
    if missing == 0:
        axes[0] = np.cross(axes[1], axes[2])
    elif missing == 1:
        axes[1] = np.cross(axes[2], axes[0])
    else:
        axes[2] = np.cross(axes[0], axes[1])

    tcp = np.column_stack([axis / max(float(np.linalg.norm(axis)), 1.0e-12) for axis in axes])
    if float(np.linalg.det(tcp)) < 0.0:
        tcp[:, missing] *= -1.0
    return tcp


def build_task_points(row, rotation, translation, args):
    """TrajectoryRow 를 로봇 월드 좌표계의 TaskPoint 목록으로 만든다.

    planner CSV 는 부품 스캔 좌표계 값이라 scene.yaml 의 배치 변환을 먼저 건다.
    호길이 s 는 CSV 에 없으므로 위치 차분 누적으로 만든다(viz_ik.py 와 같은 규약).
    """
    positions = np.asarray(row.positions, dtype=float)
    orientations = np.asarray(row.orientations, dtype=float)
    times = np.asarray(row.times, dtype=float)

    stride = max(1, int(args.stride))
    if stride > 1:
        keep = list(range(0, len(positions), stride))
        if keep[-1] != len(positions) - 1:
            keep.append(len(positions) - 1)
        positions = positions[keep]
        orientations = orientations[keep]
        times = times[keep]

    if len(positions) < 2:
        raise ValueError("점이 2개 미만이라 DP 를 돌릴 수 없다")
    if np.any(np.diff(times) <= 0.0):
        raise ValueError("다운샘플 후 시간 t 가 단조증가가 아니다")

    times = times - times[0]
    arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(positions, axis=0), axis=1))])

    points = []
    for index in range(len(positions)):
        world_position = rotation @ positions[index] + translation
        planner_rotation = rotation @ quaternion_xyzw_to_matrix(orientations[index])
        tcp_rotation = remap_planner_rotation_to_tcp(
            planner_rotation, args.tcp_spray_axis, args.tcp_path_axis
        )
        points.append(
            TaskPoint(
                t=float(times[index]),
                s=float(arc[index]),
                position=world_position,
                rotation=tcp_rotation,
            )
        )
    return points


# ------------------------------------------------------------------- 모델/한계


def create_model(urdf_path, tcp_frame):
    model = pin.buildModelFromUrdf(str(urdf_path))
    data = model.createData()
    _name, frame_id = resolve_tcp_frame(model, tcp_frame)
    if model.nq != model.nv:
        raise ValueError(f"nq == nv 인 모델만 다룬다. nq={model.nq} nv={model.nv}")
    return model, data, frame_id


def finite_limits(model):
    lower = np.asarray(model.lowerPositionLimit, dtype=float).copy()
    upper = np.asarray(model.upperPositionLimit, dtype=float).copy()
    lower = np.where(np.isfinite(lower), lower, -math.pi)
    upper = np.where(np.isfinite(upper), upper, math.pi)
    return lower, upper


def prismatic_joint_mask(model):
    """관절별 직동 여부. 갠트리 직동축은 m, 회전축은 rad 로 다뤄야 한다."""
    mask = np.zeros(model.nq, dtype=bool)
    for joint_index in range(1, model.njoints):
        joint = model.joints[joint_index]
        name = joint.shortname()
        is_prismatic = name.startswith("JointModelP") and not name.startswith("JointModelPlanar")
        mask[joint.idx_q : joint.idx_q + joint.nq] = is_prismatic
    return mask


def joint_names_of(model):
    return [model.names[index] for index in range(1, len(model.names))]


def attach_joint_motion_limits(model, args):
    """점프/속도/가속도/중복판정 한계를 관절별 SI 벡터로 만들어 args 에 붙인다.

    아래 함수들이 args 를 통째로 받는 원본 구조를 그대로 유지하려고 여기에 담는다.
    """
    prismatic = prismatic_joint_mask(model)
    args.joint_prismatic = prismatic
    args.joint_step_limit = np.where(
        prismatic, float(args.max_gantry_step_m), math.radians(float(args.max_joint_step_deg))
    )
    args.joint_velocity_limit = np.where(
        prismatic,
        float(args.max_gantry_velocity_m_s),
        math.radians(float(args.max_joint_velocity_deg_s)),
    )
    args.joint_acceleration_limit = np.where(
        prismatic,
        float(args.max_gantry_acceleration_m_s2),
        math.radians(float(args.max_joint_acceleration_deg_s2)),
    )
    args.joint_duplicate_threshold = np.where(
        prismatic,
        float(args.ik_duplicate_candidate_m),
        math.radians(max(float(args.ik_duplicate_candidate_deg), 1.0e-6)),
    )

    # 연속해 보존 개수. auto(-1)이면 여유자유도 모델에서만 켠다. 8축 셀은 항상 켜진다.
    # 이전 점의 **모든 후보**에 연속해를 남겨야 DP 그래프가 안 끊긴다.
    if int(args.ik_keep_continuations) < 0:
        redundant = model.nq > 6 or bool(np.any(prismatic))
        args.ik_keep_continuations = int(args.ik_max_candidates_per_point) if redundant else 0
    return args


def format_joint_vector(values, prismatic, suffix=""):
    """관절 벡터를 관절별 고유 단위로 찍는다 (직동 m, 회전 deg)."""
    parts = []
    for value, is_prismatic in zip(np.asarray(values).reshape(-1), prismatic):
        if is_prismatic:
            parts.append(f"{value:.4f}m{suffix}")
        else:
            parts.append(f"{math.degrees(value):.3f}deg{suffix}")
    return " ".join(parts)


def worst_limit_ratio(values, limit, prismatic, names, unit=""):
    """한계 대비 비율이 가장 큰 축을 (비율, 설명) 으로 돌려준다.

    축 번호 대신 실제 관절 이름을 쓴다. 8축 셀은 앞 2개가 갠트리라
    "joint8" 같은 번호 표기가 팔 관절 번호와 어긋난다.
    """
    values = np.asarray(values, dtype=float)
    if values.size == 0:
        return 0.0, "n/a"
    peak = np.max(np.abs(values), axis=0)
    ratios = peak / np.maximum(limit, 1.0e-12)
    index = int(np.argmax(ratios))
    scale = 1.0 if prismatic[index] else 180.0 / math.pi
    native = "m" if prismatic[index] else "deg"
    detail = (
        f"{names[index]} {peak[index] * scale:.3f}{native}{unit}"
        f"/{limit[index] * scale:.3f}{native}{unit} = {ratios[index]:.2f}x"
    )
    return float(ratios[index]), detail


# ----------------------------------------------------------------- 점별 DLS IK


def frame_pose_and_jacobian(model, data, frame_id, q):
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    pose = data.oMf[frame_id]
    jacobian = pin.computeFrameJacobian(
        model, data, q, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
    )
    return pose, np.asarray(jacobian, dtype=float)


def task_errors(point, pose):
    position_error = np.asarray(point.position - pose.translation, dtype=float).reshape(3)
    rotation_error = np.asarray(pin.log3(point.rotation @ pose.rotation.T), dtype=float).reshape(3)
    position_norm = float(np.linalg.norm(position_error))
    rotation_norm = float(np.linalg.norm(rotation_error))
    return position_error, rotation_error, position_norm, math.degrees(rotation_norm)


def solve_point_dls_ik(model, data, frame_id, point, q_seed, lower, upper, args):
    """시드 하나에서 damped least squares IK 를 푼다. 라인서치로 발산을 막는다."""
    q = np.clip(np.asarray(q_seed, dtype=float).copy(), lower, upper)
    damping = max(float(args.ik_damping), 1.0e-9)
    max_step_rad = math.radians(max(float(args.ik_max_step_deg), 0.1))
    position_tolerance = float(args.position_tolerance)
    orientation_tolerance_deg = float(args.orientation_tolerance_deg)

    best_q = q.copy()
    best_pos_error = math.inf
    best_rot_error_deg = math.inf

    for iteration in range(1, int(args.ik_max_iter) + 1):
        pose, jacobian = frame_pose_and_jacobian(model, data, frame_id, q)
        pos_error, rot_error, pos_norm, rot_deg = task_errors(point, pose)
        score = (
            pos_norm / max(position_tolerance, 1.0e-9)
            + math.radians(rot_deg) / max(math.radians(orientation_tolerance_deg), 1.0e-9)
        )
        if pos_norm < best_pos_error or (
            abs(pos_norm - best_pos_error) < 1.0e-12 and rot_deg < best_rot_error_deg
        ):
            best_q = q.copy()
            best_pos_error = pos_norm
            best_rot_error_deg = rot_deg
        if pos_norm <= position_tolerance and rot_deg <= orientation_tolerance_deg:
            return IkPointResult(
                q=q.copy(),
                iterations=iteration,
                position_error_m=pos_norm,
                orientation_error_deg=rot_deg,
                converged=True,
            )

        weighted_jacobian = jacobian.copy()
        weighted_jacobian[:3, :] *= float(args.ik_position_gain)
        weighted_jacobian[3:, :] *= float(args.ik_orientation_gain)
        weighted_error = np.concatenate([
            float(args.ik_position_gain) * pos_error,
            float(args.ik_orientation_gain) * rot_error,
        ])

        system = weighted_jacobian @ weighted_jacobian.T + (damping * damping) * np.eye(6)
        try:
            dq = weighted_jacobian.T @ np.linalg.solve(system, weighted_error)
        except np.linalg.LinAlgError:
            dq = weighted_jacobian.T @ np.linalg.lstsq(system, weighted_error, rcond=None)[0]

        max_abs_step = float(np.linalg.norm(dq, ord=np.inf))
        if max_abs_step > max_step_rad:
            dq *= max_step_rad / max_abs_step

        accepted = False
        for scale in (1.0, 0.5, 0.25, 0.1):
            candidate = np.clip(q + scale * dq, lower, upper)
            candidate_pose, _ = frame_pose_and_jacobian(model, data, frame_id, candidate)
            _, _, candidate_pos_norm, candidate_rot_deg = task_errors(point, candidate_pose)
            candidate_score = (
                candidate_pos_norm / max(position_tolerance, 1.0e-9)
                + math.radians(candidate_rot_deg)
                / max(math.radians(orientation_tolerance_deg), 1.0e-9)
            )
            if candidate_score <= score:
                q = candidate
                accepted = True
                break
        if not accepted:
            q = np.clip(q + 0.1 * dq, lower, upper)

    return IkPointResult(
        q=best_q,
        iterations=int(args.ik_max_iter),
        position_error_m=best_pos_error,
        orientation_error_deg=best_rot_error_deg,
        converged=False,
    )


# ------------------------------------------------------------------ 후보 생성


def q_is_duplicate(q, existing, threshold):
    """관절별 임계값(직동 m, 회전 rad) 안에 들어오면 같은 해로 본다."""
    return any(bool(np.all(np.abs(q - other) <= threshold)) for other in existing)


def add_unique_seed(seeds, q, lower, upper, threshold, max_count):
    if len(seeds) >= max_count:
        return
    q = np.clip(np.asarray(q, dtype=float).reshape(-1), lower, upper)
    if not q_is_duplicate(q, seeds, threshold):
        seeds.append(q)


def expand_seed_offset(offset, prismatic):
    """6축 회전 시드 오프셋(deg)을 모델 전체 관절 벡터(rad)로 편다.

    갠트리 직동축에는 0 을 넣는다 — 직동축은 여분 자유도라 warm start 로 충분하다.
    """
    full = np.zeros(prismatic.shape[0], dtype=float)
    revolute_indices = np.flatnonzero(~prismatic)
    count = min(len(offset), len(revolute_indices))
    full[revolute_indices[:count]] = np.radians(np.asarray(offset[:count], dtype=float))
    return full


def build_seed_bank(initial_q, previous_candidates, lower, upper, args):
    max_seeds = max(1, int(args.ik_max_seeds_per_point))
    threshold = args.joint_duplicate_threshold
    offsets = [expand_seed_offset(offset, args.joint_prismatic) for offset in IK_SEED_OFFSETS_DEG]
    seeds = []

    for candidate in previous_candidates:
        add_unique_seed(seeds, candidate.q, lower, upper, threshold, max_seeds)
    for offset in offsets:
        add_unique_seed(seeds, initial_q + offset, lower, upper, threshold, max_seeds)
    for candidate in previous_candidates[:3]:
        for offset in offsets:
            add_unique_seed(seeds, candidate.q + offset, lower, upper, threshold, max_seeds)
    return seeds


def candidate_quality_cost(candidate, model, args):
    """정적 품질 비용. 추종오차 / 특이점 / 관절한계 여유만 본다 (연속성은 DP 담당)."""
    position_ratio = candidate.position_error_m / max(float(args.position_tolerance), 1.0e-9)
    orientation_ratio = candidate.orientation_error_deg / max(
        float(args.orientation_tolerance_deg), 1.0e-9
    )
    singular_ratio = float(args.min_singular_value) / max(candidate.min_sigma, 1.0e-9)

    lower, upper = finite_limits(model)
    lower_margin = np.maximum(candidate.q - lower, 1.0e-6)
    upper_margin = np.maximum(upper - candidate.q, 1.0e-6)
    joint_range = np.maximum(upper - lower, 1.0e-6)
    limit_margin = np.minimum(lower_margin, upper_margin)
    limit_ratio = np.maximum(0.0, 0.05 * joint_range / limit_margin - 1.0)

    return (
        float(args.dp_tracking_weight)
        * (position_ratio * position_ratio + orientation_ratio * orientation_ratio)
        + float(args.dp_singularity_weight) * singular_ratio * singular_ratio
        + float(args.dp_limit_margin_weight) * float(np.sum(limit_ratio * limit_ratio))
    )


def make_candidate_from_result(model, data, frame_id, point, result, seed_index):
    pose, jacobian = frame_pose_and_jacobian(model, data, frame_id, result.q)
    _, _, pos_norm, rot_deg = task_errors(point, pose)
    singular_values = np.linalg.svd(jacobian, compute_uv=False)
    det_value = float(np.linalg.det(jacobian @ jacobian.T))
    return IkCandidate(
        q=result.q.copy(),
        position_error_m=pos_norm,
        orientation_error_deg=rot_deg,
        min_sigma=float(np.min(singular_values)),
        manipulability=math.sqrt(max(0.0, det_value)),
        iterations=result.iterations,
        seed_index=seed_index,
    )


def candidate_is_valid(candidate, args):
    if candidate.position_error_m > float(args.position_tolerance):
        return False
    if candidate.orientation_error_deg > float(args.orientation_tolerance_deg):
        return False
    if candidate.min_sigma < float(args.min_singular_value):
        return False
    if args.min_manipulability > 0.0 and candidate.manipulability < float(args.min_manipulability):
        return False
    return True


def generate_point_candidates(
    model, data, frame_id, point, initial_q, previous_candidates, lower, upper, args, dt=0.0
):
    seeds = build_seed_bank(initial_q, previous_candidates, lower, upper, args)
    duplicate_threshold = args.joint_duplicate_threshold
    candidates = []
    rejected = []
    candidate_qs = []

    for seed_index, seed in enumerate(seeds):
        result = solve_point_dls_ik(model, data, frame_id, point, seed, lower, upper, args)
        if not result.converged:
            continue
        candidate = make_candidate_from_result(model, data, frame_id, point, result, seed_index)
        if q_is_duplicate(candidate.q, candidate_qs, duplicate_threshold):
            continue
        candidate_qs.append(candidate.q)
        if candidate_is_valid(candidate, args):
            candidates.append(candidate)
        else:
            rejected.append(candidate)

    candidates.sort(key=lambda candidate: candidate_quality_cost(candidate, model, args))
    keep = max(1, int(args.ik_max_candidates_per_point))

    # 여유자유도(8축) 대응: candidate_quality_cost 는 이전 점과의 연속성을 보지 않는다.
    # 널스페이스가 2차원이라 DLS 가 갠트리를 제각각인 위치에 놓는데, 정적 품질 상위
    # N개만 남기면 매끄러운 연결해가 잘려 DP 가 "no feasible connection" 으로 막힌다.
    # 그래서 **연속 트랙을 먼저 채우고** 남는 자리만 품질 상위로 채운다.
    keep_continuations = int(getattr(args, "ik_keep_continuations", 0))
    selected = []
    chosen = set()
    if keep_continuations > 0 and previous_candidates and candidates:
        # 점프 한계와 속도*dt 중 **더 빡빡한 쪽**으로 거리를 재야 DP 를 통과한다.
        effective = args.joint_step_limit
        if dt > 0.0:
            effective = np.minimum(effective, args.joint_velocity_limit * dt)
        scale = np.maximum(effective, 1.0e-12)
        for reference in previous_candidates[:keep_continuations]:
            distances = [
                float(np.max(np.abs(candidate.q - reference.q) / scale)) for candidate in candidates
            ]
            nearest = candidates[int(np.argmin(distances))]
            if id(nearest) not in chosen:
                selected.append(nearest)
                chosen.add(id(nearest))

    for candidate in candidates:
        if len(selected) >= keep:
            break
        if id(candidate) not in chosen:
            selected.append(candidate)
            chosen.add(id(candidate))

    selected.sort(key=lambda candidate: candidate_quality_cost(candidate, model, args))
    if selected:
        return selected

    rejected.sort(
        key=lambda candidate: (
            candidate.position_error_m / max(float(args.position_tolerance), 1.0e-9)
            + candidate.orientation_error_deg / max(float(args.orientation_tolerance_deg), 1.0e-9)
            + float(args.min_singular_value) / max(candidate.min_sigma, 1.0e-9)
        )
    )
    if rejected:
        best = rejected[0]
        raise RuntimeError(
            "FK/특이점 필터를 통과한 후보가 없다: "
            f"pos_err={best.position_error_m:.6f}m "
            f"rot_err={best.orientation_error_deg:.4f}deg "
            f"min_sigma={best.min_sigma:.6e} "
            f"manip={best.manipulability:.6e} "
            f"rejected={len(rejected)} seeds={len(seeds)}"
        )
    raise RuntimeError(f"시드 {len(seeds)}개에서 수렴한 IK 해가 하나도 없다")


# ------------------------------------------------------------------ DP 경로 선택


def transition_cost(prev, cur, dt, args):
    dt = max(float(dt), 1.0e-9)
    dq = cur.q - prev.q
    velocity = dq / dt
    jump_ratio = dq / np.maximum(args.joint_step_limit, 1.0e-6)
    velocity_ratio = velocity / np.maximum(args.joint_velocity_limit, 1.0e-6)
    hard_velocity_excess = np.maximum(0.0, np.abs(velocity_ratio) - 1.0)
    return (
        float(args.dp_jump_weight) * float(np.sum(jump_ratio * jump_ratio))
        + float(args.dp_velocity_weight) * float(np.sum(velocity_ratio * velocity_ratio))
        + 1000.0 * float(np.sum(hard_velocity_excess * hard_velocity_excess))
    )


def transition_is_feasible(prev, cur, dt, args):
    dt = max(float(dt), 1.0e-9)
    dq = np.abs(cur.q - prev.q)
    return bool(
        np.all(dq <= args.joint_step_limit) and np.all(dq / dt <= args.joint_velocity_limit)
    )


def _acceleration(prev_prev, prev, cur, prev_dt, cur_dt):
    prev_dt = max(float(prev_dt), 1.0e-9)
    cur_dt = max(float(cur_dt), 1.0e-9)
    prev_velocity = (prev.q - prev_prev.q) / prev_dt
    cur_velocity = (cur.q - prev.q) / cur_dt
    return 2.0 * (cur_velocity - prev_velocity) / (prev_dt + cur_dt)


def acceleration_cost(prev_prev, prev, cur, prev_dt, cur_dt, args):
    acceleration = _acceleration(prev_prev, prev, cur, prev_dt, cur_dt)
    acceleration_ratio = acceleration / np.maximum(args.joint_acceleration_limit, 1.0e-6)
    hard_excess = np.maximum(0.0, np.abs(acceleration_ratio) - 1.0)
    return (
        float(args.dp_acceleration_weight) * float(np.sum(acceleration_ratio * acceleration_ratio))
        + 1000.0 * float(np.sum(hard_excess * hard_excess))
    )


def acceleration_is_feasible(prev_prev, prev, cur, prev_dt, cur_dt, args):
    acceleration = _acceleration(prev_prev, prev, cur, prev_dt, cur_dt)
    return bool(np.all(np.abs(acceleration) <= args.joint_acceleration_limit))


def select_candidate_path_dp(candidate_sets, points, model, args):
    """(이전, 현재) 후보쌍을 상태로 갖는 DP. 상태에 이전 점이 있어야 가속도를 본다."""
    count = len(candidate_sets)
    if count == 0:
        raise ValueError("후보 집합이 비었다")
    if count == 1:
        return [min(candidate_sets[0], key=lambda c: candidate_quality_cost(c, model, args))]

    node_costs = [
        [candidate_quality_cost(candidate, model, args) for candidate in candidates]
        for candidates in candidate_sets
    ]

    states = {}
    parents = {}
    first_dt = points[1].t - points[0].t
    for first_index, first in enumerate(candidate_sets[0]):
        for second_index, second in enumerate(candidate_sets[1]):
            if not transition_is_feasible(first, second, first_dt, args):
                continue
            states[(first_index, second_index)] = (
                node_costs[0][first_index]
                + node_costs[1][second_index]
                + transition_cost(first, second, first_dt, args)
            )
    if not states:
        raise RuntimeError("점 1-2 사이에 점프/속도 실현 가능한 연결이 없다")

    for point_index in range(2, count):
        prev_dt = points[point_index - 1].t - points[point_index - 2].t
        cur_dt = points[point_index].t - points[point_index - 1].t
        new_states = {}
        for (prev_prev_index, prev_index), old_cost in states.items():
            prev_prev = candidate_sets[point_index - 2][prev_prev_index]
            prev = candidate_sets[point_index - 1][prev_index]
            for cur_index, cur in enumerate(candidate_sets[point_index]):
                if not transition_is_feasible(prev, cur, cur_dt, args):
                    continue
                if not acceleration_is_feasible(prev_prev, prev, cur, prev_dt, cur_dt, args):
                    continue
                cost = (
                    old_cost
                    + node_costs[point_index][cur_index]
                    + transition_cost(prev, cur, cur_dt, args)
                    + acceleration_cost(prev_prev, prev, cur, prev_dt, cur_dt, args)
                )
                key = (prev_index, cur_index)
                if key not in new_states or cost < new_states[key]:
                    new_states[key] = cost
                    parents[(point_index, prev_index, cur_index)] = prev_prev_index
        states = new_states
        if not states:
            raise RuntimeError(
                f"점 {point_index + 1} 에서 점프/속도/가속도 실현 가능한 연결이 없다"
            )

    best_pair, best_cost = min(states.items(), key=lambda item: item[1])
    sequence_indices = [0] * count
    sequence_indices[count - 2], sequence_indices[count - 1] = best_pair
    prev_index, cur_index = best_pair
    for point_index in range(count - 1, 1, -1):
        prev_prev_index = parents[(point_index, prev_index, cur_index)]
        sequence_indices[point_index - 2] = prev_prev_index
        cur_index = prev_index
        prev_index = prev_prev_index

    return (
        [
            candidate_sets[point_index][candidate_index]
            for point_index, candidate_index in enumerate(sequence_indices)
        ],
        float(best_cost),
    )


def solve_dp_ik_path(model, data, frame_id, points, initial_q, args, label=""):
    """한 row 를 후보 생성 + DP 로 푼다. (q_path, 통계) 를 돌려준다."""
    lower, upper = finite_limits(model)
    initial_q = np.clip(np.asarray(initial_q, dtype=float), lower, upper)
    candidate_sets = []
    previous_candidates = []

    for index, point in enumerate(points):
        try:
            candidates = generate_point_candidates(
                model=model,
                data=data,
                frame_id=frame_id,
                point=point,
                initial_q=initial_q,
                previous_candidates=previous_candidates,
                lower=lower,
                upper=upper,
                args=args,
                dt=(point.t - points[index - 1].t) if index > 0 else 0.0,
            )
        except RuntimeError as error:
            raise RuntimeError(
                f"후보 생성 실패 (점 {index + 1}/{len(points)}): {error}"
            ) from error

        candidate_sets.append(candidates)
        previous_candidates = candidates
        if index % max(1, int(args.ik_progress_interval)) == 0 or index == len(points) - 1:
            best = candidates[0]
            print(
                f"[dp-ik] {label} 점 {index + 1}/{len(points)} "
                f"후보={len(candidates)} "
                f"pos_err={best.position_error_m:.6f}m "
                f"rot_err={best.orientation_error_deg:.4f}deg "
                f"sigma={best.min_sigma:.6e}",
                flush=True,
            )

    selected, best_cost = select_candidate_path_dp(candidate_sets, points, model, args)
    q_path = np.asarray([candidate.q for candidate in selected], dtype=float)
    counts = [len(candidates) for candidates in candidate_sets]
    stats = {
        "dp_cost": best_cost,
        "max_position_error_m": max(c.position_error_m for c in selected),
        "max_orientation_error_deg": max(c.orientation_error_deg for c in selected),
        "min_sigma": min(c.min_sigma for c in selected),
        "min_manipulability": min(c.manipulability for c in selected),
        "candidate_count_min": min(counts),
        "candidate_count_max": max(counts),
        "candidate_count_avg": sum(counts) / len(counts),
    }
    return q_path, stats


# ------------------------------------------------------------------- 결과 점검


def derivative_by_axis(axis, values):
    """불균등 격자에서 열 단위로 미분한다."""
    axis = np.asarray(axis, dtype=float).reshape(-1)
    values = np.asarray(values, dtype=float)
    if len(axis) < 2:
        raise ValueError("샘플이 2개 이상이어야 한다")
    edge_order = 2 if len(axis) > 2 else 1
    derivative = np.zeros_like(values)
    for column in range(values.shape[1]):
        derivative[:, column] = np.gradient(values[:, column], axis, edge_order=edge_order)
    return derivative


def joint_motion(points, q_path):
    """시간 기반 미분으로 q̇, q̈ 를 만든다 (trajectory_nlp.py 와 같은 규약).

    호길이 연쇄법칙은 사다리꼴 프로파일 양 끝에서 ds 가 붕괴해 가속도가 폭주한다.
    """
    times = np.array([point.t for point in points], dtype=float)
    qd = derivative_by_axis(times, q_path)
    qdd = derivative_by_axis(times, qd)
    qd[0] = 0.0
    qd[-1] = 0.0
    qdd[0] = 0.0
    qdd[-1] = 0.0
    return qd, qdd


# ---------------------------------------------------------------------- 출력


def write_joint_csv(path, times, arc_lengths, q_path, joint_names):
    """관절값을 SI 단위(회전 rad, 직동 m)로 저장한다.

    갠트리 직동축이 섞여 deg 단일 단위를 못 쓴다. viz_ik.py 와 같은 형식이라
    trajectory_player 가 그대로 읽는다.
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["t", "s"] + list(joint_names))
        for index, t in enumerate(times):
            writer.writerow(
                [f"{t:.9f}", f"{arc_lengths[index]:.9f}"]
                + [f"{value:.9f}" for value in q_path[index]]
            )


def write_summary_csv(path, rows):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    header = [
        "row", "source", "status", "points", "solve_sec",
        "max_pos_err_m", "max_rot_err_deg", "min_sigma",
        "candidate_min", "candidate_avg",
        "qdot_ratio", "qddot_ratio", "branch_jump", "message",
    ]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        for row in rows:
            writer.writerow([row.get(name, "") for name in header])


# ---------------------------------------------------------------------- CLI


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            "DP-IK(다중 시드 DLS + DP 경로선택) 시각화 솔버. "
            "trajectory_ik_pinocchio.py 의 IK 코어를 8축 셀에서 돌려 RViz 재생용 "
            "joint CSV 를 만든다. 실기 실행용이 아니다(1kHz 명령 CSV 를 만들지 않는다)."
        )
    )
    parser.add_argument("--scene", required=True, type=Path, help="config/scene.yaml 경로")
    parser.add_argument("--urdf", type=Path, default=None,
                        help="8축 셀 URDF. 생략하면 --gantry-env 의 urdf.primitive")
    add_gantry_env_argument(parser)
    parser.add_argument("--output", required=True, type=Path, help="joint CSV 출력 디렉터리")
    parser.add_argument("--tcp-frame", default=DEFAULT_TCP_FRAME)
    parser.add_argument("--tcp-spray-axis", default=DEFAULT_TCP_SPRAY_AXIS)
    parser.add_argument("--tcp-path-axis", default=DEFAULT_TCP_PATH_AXIS)
    parser.add_argument(
        "--initial-joint", default=DEFAULT_INITIAL_JOINT,
        help="초기 관절값. 회전축은 deg, 직동축은 m (URDF 관절 순서)",
    )
    parser.add_argument("--stride", type=int, default=1,
                        help="점 다운샘플 간격. DP 는 점당 비용이 커서 4 정도면 훨씬 빠르다")
    parser.add_argument("--max-rows", type=int, default=0, help="처리할 row 수 상한 (0이면 전체)")
    parser.add_argument(
        "--on-failure", choices=("skip", "stop"), default="skip",
        help="DP 가 막힌 row 처리. skip 이면 그 row 만 버리고 계속한다",
    )
    parser.add_argument(
        "--carry-seed", choices=("last", "initial"), default="last",
        help="다음 row 의 시드. last 는 직전 row 마지막 자세(원본과 동일), "
             "initial 은 매 row 를 --initial-joint 에서 다시 시작",
    )

    parser.add_argument("--position-tolerance", type=float, default=POSITION_TOLERANCE_M)
    parser.add_argument("--orientation-tolerance-deg", type=float, default=ORIENTATION_TOLERANCE_DEG)
    parser.add_argument("--max-joint-step-deg", type=float, default=MAX_JOINT_STEP_DEG)
    parser.add_argument("--max-joint-velocity-deg-s", type=float, default=MAX_JOINT_VELOCITY_DEG_S)
    parser.add_argument("--max-joint-acceleration-deg-s2", type=float,
                        default=MAX_JOINT_ACCELERATION_DEG_S2)
    parser.add_argument("--max-gantry-step-m", type=float, default=MAX_GANTRY_STEP_M)
    parser.add_argument("--max-gantry-velocity-m-s", type=float, default=MAX_GANTRY_VELOCITY_M_S)
    parser.add_argument("--max-gantry-acceleration-m-s2", type=float,
                        default=MAX_GANTRY_ACCELERATION_M_S2)
    parser.add_argument("--min-manipulability", type=float, default=MIN_MANIPULABILITY)
    parser.add_argument("--min-singular-value", type=float, default=MIN_SINGULAR_VALUE)

    parser.add_argument("--ik-max-iter", type=int, default=IK_MAX_ITER)
    parser.add_argument("--ik-damping", type=float, default=IK_DAMPING)
    parser.add_argument("--ik-max-step-deg", type=float, default=IK_MAX_STEP_DEG)
    parser.add_argument("--ik-position-gain", type=float, default=IK_POSITION_GAIN)
    parser.add_argument("--ik-orientation-gain", type=float, default=IK_ORIENTATION_GAIN)
    parser.add_argument("--ik-progress-interval", type=int, default=IK_PROGRESS_INTERVAL)
    parser.add_argument("--ik-max-seeds-per-point", type=int, default=IK_MAX_SEEDS_PER_POINT)
    parser.add_argument("--ik-max-candidates-per-point", type=int,
                        default=IK_MAX_CANDIDATES_PER_POINT)
    parser.add_argument("--ik-duplicate-candidate-deg", type=float,
                        default=IK_DUPLICATE_CANDIDATE_DEG)
    parser.add_argument("--ik-duplicate-candidate-m", type=float, default=IK_DUPLICATE_CANDIDATE_M)
    parser.add_argument("--ik-keep-continuations", type=int, default=-1,
                        help="연속해 보존 개수. -1(기본)이면 여유자유도 모델에서 자동으로 켠다")

    parser.add_argument("--dp-tracking-weight", type=float, default=DP_TRACKING_WEIGHT)
    parser.add_argument("--dp-jump-weight", type=float, default=DP_JUMP_WEIGHT)
    parser.add_argument("--dp-velocity-weight", type=float, default=DP_VELOCITY_WEIGHT)
    parser.add_argument("--dp-acceleration-weight", type=float, default=DP_ACCELERATION_WEIGHT)
    parser.add_argument("--dp-singularity-weight", type=float, default=DP_SINGULARITY_WEIGHT)
    parser.add_argument("--dp-limit-margin-weight", type=float, default=DP_LIMIT_MARGIN_WEIGHT)
    return parser.parse_args(argv)


def build_initial_q(model, joint_names, prismatic, text):
    """초기 관절값 문자열을 파싱한다 (회전 deg, 직동 m)."""
    lower, upper = finite_limits(model)
    text = str(text or "").strip()
    if not text:
        return np.clip(np.zeros(model.nq), lower, upper)
    values = [float(token) for token in text.split(",") if token.strip()]
    if len(values) != model.nq:
        raise ValueError(
            f"--initial-joint 는 {model.nq}개 값이어야 한다 (관절 순서: {', '.join(joint_names)})"
        )
    values = np.asarray(values, dtype=float)
    return np.clip(np.where(prismatic, values, np.radians(values)), lower, upper)


def main(argv=None):
    args = parse_args(argv)
    apply_gantry_env(args, "primitive")

    if parse_signed_axis(args.tcp_spray_axis)[0] == parse_signed_axis(args.tcp_path_axis)[0]:
        raise ValueError("--tcp-spray-axis 와 --tcp-path-axis 는 서로 다른 축이어야 한다")

    scene = load_scene_config(args.scene)
    rotation, translation = part_transform(scene)

    urdf_path = args.urdf.expanduser().resolve()
    if not urdf_path.is_file():
        raise FileNotFoundError(urdf_path)
    model, data, frame_id = create_model(urdf_path, args.tcp_frame)
    args.tcp_frame = model.frames[frame_id].name
    args = attach_joint_motion_limits(model, args)

    names = joint_names_of(model)
    initial_q = build_initial_q(model, names, args.joint_prismatic, args.initial_joint)

    files = find_trajectory_csv_files(scene["trajectory_dir"])
    max_rows = int(args.max_rows or scene.get("max_rows", 0) or 0)
    if max_rows:
        files = files[:max_rows]
    if not files:
        raise FileNotFoundError(f"trajectory CSV가 없다: {scene['trajectory_dir']}")

    print(f"[dp-ik] URDF: {urdf_path}")
    print(f"[dp-ik] 관절: {', '.join(names)}")
    print(f"[dp-ik] 직동축 {int(np.sum(args.joint_prismatic))} / {model.nq}")
    print(f"[dp-ik] TCP {args.tcp_frame} (분사축 {args.tcp_spray_axis}, 경로축 {args.tcp_path_axis})")
    print(f"[dp-ik] part_xyz={scene['part_xyz']} part_rpy={scene['part_rpy']}")
    print(f"[dp-ik] 연속해 보존={args.ik_keep_continuations} 후보상한={args.ik_max_candidates_per_point}")
    print(f"[dp-ik] row {len(files)}개, stride={args.stride}, 실패시={args.on_failure}")
    if args.stride == 1:
        print("[dp-ik] 참고: DP 는 점당 비용이 크다. 먼저 --stride 4 로 훑어보는 편이 빠르다.")
    print(f"[dp-ik] 출력: {args.output}")

    args.output.mkdir(parents=True, exist_ok=True)

    q_seed = initial_q.copy()
    previous_last_q = None
    summary_rows = []
    total_start = time.perf_counter()
    solved = 0
    total_points = 0

    for path in files:
        source = Path(path)
        label = source.stem
        try:
            row = read_trajectory_csv(path)
            points = build_task_points(row, rotation, translation, args)
        except (ValueError, OSError) as error:
            print(f"[dp-ik] {label}: 입력을 읽지 못했다 — {error}")
            summary_rows.append({
                "row": label, "source": str(source), "status": "input-error",
                "points": 0, "message": str(error),
            })
            if args.on_failure == "stop":
                raise
            continue

        start = time.perf_counter()
        try:
            q_path, stats = solve_dp_ik_path(
                model, data, frame_id, points, q_seed, args, label=label
            )
        except (RuntimeError, ValueError) as error:
            elapsed = time.perf_counter() - start
            print(f"[dp-ik] {label}: DP 실패 ({elapsed:.1f}s) — {error}")
            summary_rows.append({
                "row": label, "source": str(source), "status": "dp-failed",
                "points": len(points), "solve_sec": f"{elapsed:.2f}", "message": str(error),
            })
            if args.on_failure == "stop":
                raise
            continue
        elapsed = time.perf_counter() - start

        qd, qdd = joint_motion(points, q_path)
        velocity_ratio, velocity_detail = worst_limit_ratio(
            qd, args.joint_velocity_limit, args.joint_prismatic, names, "/s"
        )
        acceleration_ratio, acceleration_detail = worst_limit_ratio(
            qdd, args.joint_acceleration_limit, args.joint_prismatic, names, "/s^2"
        )

        # row 경계에서 IK 분기가 갈리면 재생이 순간이동처럼 보이고, 이어붙이기도 어렵다.
        branch_jump = ""
        if previous_last_q is not None:
            jump = np.abs(q_path[0] - previous_last_q)
            worst = int(np.argmax(jump / np.maximum(args.joint_step_limit, 1.0e-9)))
            scale = 1.0 if args.joint_prismatic[worst] else 180.0 / math.pi
            unit = "m" if args.joint_prismatic[worst] else "deg"
            branch_jump = f"{names[worst]} {jump[worst] * scale:.3f}{unit}"

        times = np.array([point.t for point in points], dtype=float)
        arc = np.array([point.s for point in points], dtype=float)
        output_csv = args.output / f"{label}_dp_joint.csv"
        write_joint_csv(output_csv, times, arc, q_path, names)

        solved += 1
        total_points += len(points)
        if args.carry_seed == "last":
            q_seed = q_path[-1].copy()
        previous_last_q = q_path[-1].copy()

        print(
            f"[dp-ik] {label} 완료 pts={len(points)} {elapsed:.1f}s "
            f"pos_err={stats['max_position_error_m']:.6f}m "
            f"rot_err={stats['max_orientation_error_deg']:.4f}deg "
            f"sigma={stats['min_sigma']:.4e} "
            f"후보 {stats['candidate_count_min']}~{stats['candidate_count_max']} "
            f"qdot[{velocity_detail}] qddot[{acceleration_detail}] -> {output_csv.name}",
            flush=True,
        )

        summary_rows.append({
            "row": label,
            "source": str(source),
            "status": "ok",
            "points": len(points),
            "solve_sec": f"{elapsed:.2f}",
            "max_pos_err_m": f"{stats['max_position_error_m']:.6f}",
            "max_rot_err_deg": f"{stats['max_orientation_error_deg']:.4f}",
            "min_sigma": f"{stats['min_sigma']:.6e}",
            "candidate_min": stats["candidate_count_min"],
            "candidate_avg": f"{stats['candidate_count_avg']:.2f}",
            "qdot_ratio": f"{velocity_ratio:.2f}",
            "qddot_ratio": f"{acceleration_ratio:.2f}",
            "branch_jump": branch_jump,
            "message": "",
        })

    summary_csv = args.output / "dp_ik_summary.csv"
    write_summary_csv(summary_csv, summary_rows)

    failed = [row for row in summary_rows if row["status"] != "ok"]
    total_elapsed = time.perf_counter() - total_start
    print(
        f"[dp-ik] 완료: row {solved}/{len(files)} 성공, 점 {total_points}개, "
        f"{total_elapsed:.1f}s ({1000.0 * total_elapsed / max(total_points, 1):.1f} ms/pt)"
    )
    print(f"[dp-ik] 요약 저장: {summary_csv}")
    for row in failed:
        print(f"[dp-ik] 실패 {row['row']}: {row['message']}")
    if failed:
        print(
            f"[dp-ik] 경고: {len(failed)}개 row 가 DP 게이트에서 막혔다. "
            "RViz 에는 성공한 row 만 나온다."
        )
    if solved == 0:
        print("[dp-ik] 오류: 저장된 joint CSV 가 없다. RViz 를 띄워도 로봇이 움직이지 않는다.")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
