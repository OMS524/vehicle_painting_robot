"""row 사이 전이(移動) 궤적을 만들어 전체 도장 시퀀스를 하나로 잇는다.

왜 필요한가
  planner 는 row 를 하나씩 만들고, IK/NLP 도 row 단위로 푼다. 그래서 row N 의
  마지막 자세와 row N+1 의 첫 자세 사이가 **비어 있다**. 재생 노드는 그 사이를
  잇지 않고 다음 row 첫 자세로 순간 이동한다(trajectory_player 의 row_gap_sec).
  실측(ES300h_hood, NLP): 경계 31곳에서 TCP 가 50~295 mm 떨어져 있고
  관절은 갠트리 최대 114 mm / 팔 최대 14 deg 를 순간에 건너뛴다.

  즉 "계획 경로를 로봇이 따라간다"가 row 안에서만 성립한다. 이 모듈이 그 사이를
  실제로 이동하는 궤적으로 채워 전체 시퀀스를 연속으로 만든다.

전이 형상
  분사는 꺼진 구간이므로 표면을 따라갈 이유가 없다. 대신 부품을 긁지 않도록
  **들어올렸다가(retract) 옮기고 다시 내린다(approach)**.

      p_end ──▲── p_end+n ────────→ p_start+n ──▼── p_start
             들어올림      수평 이동        내림

  들어올리는 방향 n (--retract-dir):
    spray   (기본) 양끝 TCP 분사축(+z)의 평균을 뒤집은 방향 = 표면에서 멀어지는 쪽.
            수평으로 놓인 부품(후드/트렁크/눕힌 펜더)에서는 월드 +Z 와 같고, 세워진
            부품(범퍼, 세운 펜더)에서는 수평으로 물러난다. 2026-09-15 부터 기본.
    world_z 월드 +Z 고정 (예전 방식. 세워진 부품에서는 표면을 따라 위로 긁는다).

  자세는 전체 구간에 걸쳐 R_end → R_start 로 지오데식 보간(slerp)한다.

시간 배분
  row 는 정지 상태에서 시작·종료하므로(planner 의 path_speed 가 양끝 0) 전이도
  rest-to-rest 여야 이음새에서 속도가 튀지 않는다. 그래서 경로 파라미터에
  smoothstep u(τ)=3τ²-2τ³ 을 씌운다 (τ=t/T, u'(0)=u'(1)=0).
  smoothstep 은 u' 최대 1.5/T, u'' 최대 6/T² 이므로 관절별 한계에서 T 를 잡는다.

      T >= 1.5 * G_j / v_j        (속도)
      T >= sqrt(6 * G_j / a_j)    (가속도)      G_j = max |dq_j/du|

  직동축은 m 계열, 회전축은 rad 계열로 한계가 다르므로 관절별로 따로 계산한다.

  2026-10-20 부터 들어올림 / 이동 / 내림 **세 구간에 각각** rest-to-rest 를 건다
  (time_parameterize_segments). 한 번에 씌우면 폴리라인 모서리의 가속도 스파이크 한 점이
  전이 전체 속도를 정해 버린다 (범퍼 1.4 m 전이가 40 s). 모서리에서 잠깐 멈추는 대신
  이동 구간이 한계까지 속도를 낸다.
"""

import argparse
import csv
import math
import shutil
from pathlib import Path

import numpy as np
import pinocchio as pin
from scipy.spatial import cKDTree

from motion_planner.configuration import add_gantry_env_argument, apply_gantry_env
from motion_planner.ply_io import read_surface_points
from motion_planner.scene_config import load_scene_config, part_transform
from motion_planner.tcp_frame import resolve_tcp_frame
from motion_planner.data_io import read_joint_csv
from motion_planner.kinematics import solve_point

# 리트랙트 높이 [m]. 예전 기본값은 0.10 이었는데 근거 없이 큰 값이었다.
#
# ES300h_hood 실측(2026-08-10): row N 끝 -> row N+1 시작을 **직선으로** 이었을 때
# 그 선분이 양끝보다 부품에 더 가까워지는 깊이 = 실제로 필요한 리트랙트 높이다.
#   전이 31곳 중 21곳이 0 mm, 최악(30->31)이 3.42 mm, 평균 0.28 mm.
# planner 가 인접 row 를 지그재그로 내놓아 전이 이동거리가 50~83 mm 뿐이고,
# 그 짧은 구간에서 후드 표면이 솟지 않기 때문이다.
#
# 0.10 은 필요량의 29배였고 대가가 컸다. 올림 100 + 이동 52 + 내림 100 이라
# 경로가 4.8배가 되는데, 전이 시간은 가속도 제한이 결정(경로 길이의 제곱근)한다.
# 스윕 실측 — 전이 총 시간 / 부품 최소여유 / traverse 구간 여유
# (여유는 TCP=link_6 원점에서 부품 점군까지의 거리다. collision_check 가 찍는
#  값은 로봇 충돌메시 **정점** 기준이라 더 작게 나온다 — 같은 조건에서 279.7 mm):
#     0mm 15.5s / 294.9mm / 295.6mm      30mm 35.9s / 294.9mm / 314.9mm
#    10mm 22.0s / 294.9mm / 302.6mm      50mm 48.8s / 294.9mm / 320.9mm
#    20mm 28.5s / 294.9mm / 309.7mm     100mm 78.5s / 294.9mm / 337.4mm
# **최소여유는 리트랙트와 무관하게 294.9 mm 로 고정**이다. 최소점은 항상 row
# 양끝에서 결정되므로 리트랙트를 올려도 안전 여유는 늘지 않고 traverse 구간
# 여유만 mm 당 약 1 초를 주고 사는 셈이다.
#
# 그래서 측정된 최악값 3.42 mm 의 약 9 배인 0.03 을 쓴다 (100mm 대비 42.6 s 절감).
#
# 주의 두 가지
#  - 이 값은 ES300h_hood 기하에서 나왔다. 곡률이 크거나 row 순서가 지그재그가
#    아닌 부품은 필요 높이를 다시 재야 한다. 위 "필요 높이" 계산은 부품 점군과
#    row 양끝 TCP 만 있으면 되므로 부품마다 다시 뽑을 수 있다.
#  - **분사건이 아직 URDF 에 없다.** 위 여유는 link_6 플랜지 기준이고 planner
#    offset_distance 가 0.3 m 라 사실상 "건 길이만큼의 스탠드오프"다. 건 형상이
#    들어오면 여유가 크게 줄므로 이 값을 재검토해야 한다.
DEFAULT_RETRACT_M = 0.03
# 적응형 리트랙트(--scene) 기본값. 필요 높이 * safety 를 [min, max] 로 자른다.
# safety 9.0 은 고정값 0.03 을 고른 근거(최악 3.42mm 의 약 9배)와 같은 기준이다.
DEFAULT_RETRACT_SAFETY = 9.0
DEFAULT_RETRACT_MIN_M = 0.005
DEFAULT_RETRACT_MAX_M = 0.10
DEFAULT_STEP_M = 0.01
DEFAULT_MAX_JOINT_VELOCITY_DEG_S = 70.0
DEFAULT_MAX_JOINT_ACCELERATION_DEG_S2 = 500.0
DEFAULT_MAX_GANTRY_VELOCITY_M_S = 1.0
DEFAULT_MAX_GANTRY_ACCELERATION_M_S2 = 5.0
SAFETY = 1.05


def prismatic_mask(model):
    """관절별 직동 여부. 한계 단위(m vs rad)를 가르는 기준이다."""
    mask = np.zeros(model.nq, dtype=bool)
    for index in range(1, model.njoints):
        joint = model.joints[index]
        name = joint.shortname()
        mask[joint.idx_q : joint.idx_q + joint.nq] = name.startswith("JointModelP")
    return mask


def apply_gantry_motor_limits(args, env):
    """--max-gantry-* 를 안 줬으면 gantry_env yaml motor_limits (없으면 모듈 기본값) 로 채운다."""
    velocity, acceleration = (env.motor_limits(DEFAULT_MAX_GANTRY_VELOCITY_M_S, DEFAULT_MAX_GANTRY_ACCELERATION_M_S2)
                              if env is not None else
                              (DEFAULT_MAX_GANTRY_VELOCITY_M_S, DEFAULT_MAX_GANTRY_ACCELERATION_M_S2))
    if args.max_gantry_velocity_m_s is None:
        args.max_gantry_velocity_m_s = velocity
    if args.max_gantry_acceleration_m_s2 is None:
        args.max_gantry_acceleration_m_s2 = acceleration


def limits_of(model, args):
    prismatic = prismatic_mask(model)
    velocity = np.where(
        prismatic, args.max_gantry_velocity_m_s, math.radians(args.max_joint_velocity_deg_s)
    )
    acceleration = np.where(
        prismatic,
        args.max_gantry_acceleration_m_s2,
        math.radians(args.max_joint_acceleration_deg_s2),
    )
    return prismatic, velocity, acceleration


def fk_pose(model, data, frame_id, q):
    pin.forwardKinematics(model, data, np.asarray(q, dtype=float))
    pin.updateFramePlacements(model, data)
    placement = data.oMf[frame_id]
    return placement.translation.copy(), placement.rotation.copy()


def slerp(rotation_a, rotation_b, ratio):
    """두 회전 사이의 지오데식 보간."""
    return rotation_a @ pin.exp3(ratio * pin.log3(rotation_a.T @ rotation_b))


def retract_direction(mode, rotation_end, rotation_start):
    """리트랙트 단위 방향. spray = 양끝 분사축(+z) 평균의 반대, world_z = 월드 +Z."""
    if mode == "world_z":
        return np.array([0.0, 0.0, 1.0])
    if mode != "spray":
        raise ValueError(f"--retract-dir 는 spray | world_z 중 하나여야 한다: {mode!r}")
    away = -(np.asarray(rotation_end)[:, 2] + np.asarray(rotation_start)[:, 2])
    norm = float(np.linalg.norm(away))
    if norm < 1.0e-6:  # 양끝 분사축이 정반대(있을 수 없지만) 면 월드 +Z 로
        return np.array([0.0, 0.0, 1.0])
    return away / norm


def retract_waypoints(p_end, p_start, retract, direction=(0.0, 0.0, 1.0)):
    """들어올림 -> 이동 -> 내림 폴리라인. direction(단위벡터) 으로 retract 만큼 들어올린다."""
    lift = np.asarray(direction, dtype=float) * float(retract)
    return [p_end, p_end + lift, p_start + lift, p_start]


def required_retract(part_tree, p_end, p_start, step):
    """이 전이에 **실제로 필요한** 리트랙트 높이 [m].

    두 끝점을 직선으로 이었을 때 그 선분이 양끝보다 부품에 더 가까워지는 깊이다.
    끝점 자체의 여유는 planner 의 offset 이 이미 보장하므로, 그보다 더 파고드는
    만큼만 들어올리면 원래 여유가 회복된다. 0 이면 직선이 이미 안전하다는 뜻이다.

    ES300h_hood 에서는 31곳 중 21곳이 0 이고 최악이 3.42 mm 였다 — 인접 row 사이
    표면이 솟지 않기 때문이다. 곡률이 큰 부품에서는 이 값이 커진다.
    """
    p_end = np.asarray(p_end, dtype=float)
    p_start = np.asarray(p_start, dtype=float)
    length = float(np.linalg.norm(p_start - p_end))
    count = max(2, int(math.ceil(length / max(step, 1.0e-4))) + 1)
    ratios = np.linspace(0.0, 1.0, count).reshape(-1, 1)
    line = p_end.reshape(1, 3) + ratios * (p_start - p_end).reshape(1, 3)

    distances = part_tree.query(line)[0]
    endpoint_clearance = float(min(distances[0], distances[-1]))
    return max(0.0, endpoint_clearance - float(distances.min()))


def load_part_tree(scene_path):
    """부품 점군을 world 좌표로 올려 KD-tree 를 만든다. 적응형 리트랙트에만 쓴다."""
    scene = load_scene_config(scene_path)
    rotation, translation = part_transform(scene)
    points, _colors = read_surface_points(scene["part_file"], 0)
    return cKDTree(np.asarray(points, dtype=float) @ rotation.T + translation)


def sample_polyline(points, step):
    """폴리라인을 step 간격으로 샘플하고 (위치, 0~1 진행률, 모서리 인덱스) 를 돌려준다.

    구간(선분)마다 따로 샘플하므로 **모서리가 항상 샘플 위에 놓인다**. 모서리 인덱스는
    내부 꼭짓점(길이 0 인 선분은 건너뜀)에 해당하는 샘플 번호로, 구간별 시간 배분에
    쓴다. 길이 0 인 선분만 있는 퇴화 경우는 양끝 두 점만 돌려준다.
    """
    points = [np.asarray(p, dtype=float) for p in points]
    lengths = [float(np.linalg.norm(points[i + 1] - points[i])) for i in range(len(points) - 1)]
    total = float(sum(lengths))
    if total < 1.0e-9:
        return [points[0], points[-1]], np.array([0.0, 1.0]), []

    positions = [points[0]]
    distances = [0.0]
    corners = []
    travelled = 0.0
    for index, length in enumerate(lengths):
        if length < 1.0e-12:
            continue
        if len(positions) > 1:
            corners.append(len(positions) - 1)   # 직전 선분의 끝 = 이 선분의 시작
        count = max(2, int(math.ceil(length / max(step, 1.0e-4))) + 1)
        for ratio in np.linspace(0.0, 1.0, count)[1:]:
            positions.append(points[index] + ratio * (points[index + 1] - points[index]))
            distances.append(travelled + ratio * length)
        travelled += length
    return positions, np.asarray(distances) / total, corners


def _ik_sweep(model, data, frame_id, targets, q_seed, args):
    """목표 자세열을 warm start 로 순차 IK. (q열, 최악 위치오차, 최악 자세오차)."""
    q = np.array(q_seed, dtype=float)
    out = [q.copy()]
    worst_position = 0.0
    worst_rotation = 0.0
    for target in targets[1:]:
        q, position_error, rotation_error = solve_point(
            model, data, frame_id, target, q,
            args.ik_max_iter, args.ik_damping, math.radians(args.ik_max_step_deg),
            args.position_tolerance, math.radians(args.orientation_tolerance_deg),
        )
        out.append(q.copy())
        worst_position = max(worst_position, position_error)
        worst_rotation = max(worst_rotation, math.degrees(rotation_error))
    return np.asarray(out), worst_position, worst_rotation


def solve_transition_path(model, data, frame_id, q_from, q_to, args, retract=None):
    """전이 구간의 관절 경로를 푼다. (q_path, 진단) 을 돌려준다.

    retract 를 주면 그 값을 쓰고, 없으면 args.retract(고정값)를 쓴다. 적응형에서
    전이마다 다른 높이를 넣기 위한 것이다.

    8축은 널스페이스가 2차원이라, 한쪽 끝에서만 warm start 로 풀면 같은 TCP 자세라도
    갠트리가 다른 위치로 수렴해 **다음 row 첫 자세와 이음새가 벌어진다**
    (실측: 그렇게 하면 잔차가 최대 16 deg 까지 남았다).

    그래서 양끝에서 각각 풀고 smoothstep 가중치로 섞는다. 가중치가 시작에서 0,
    끝에서 1 이므로 **양 끝단이 정확히 q_from / q_to 와 일치**한다. 섞인 해는
    Cartesian 오차가 조금 늘 수 있어 마지막에 IK 로 한 번 더 다듬는다(polish).
    """
    p_end, rotation_end = fk_pose(model, data, frame_id, q_from)
    p_start, rotation_start = fk_pose(model, data, frame_id, q_to)

    lift = args.retract if retract is None else retract
    direction = retract_direction(args.retract_dir, rotation_end, rotation_start)
    positions, ratios, corners = sample_polyline(
        retract_waypoints(p_end, p_start, lift, direction), args.step
    )
    targets = [
        pin.SE3(slerp(rotation_end, rotation_start, float(ratio)), np.asarray(position))
        for position, ratio in zip(positions, ratios)
    ]

    forward, forward_position, forward_rotation = _ik_sweep(
        model, data, frame_id, targets, q_from, args
    )
    backward_reversed, backward_position, backward_rotation = _ik_sweep(
        model, data, frame_id, targets[::-1], q_to, args
    )
    backward = backward_reversed[::-1]

    ratio = np.linspace(0.0, 1.0, len(targets)).reshape(-1, 1)
    weight = 3.0 * ratio**2 - 2.0 * ratio**3
    blended = forward + weight * (backward - forward)

    # 섞으면서 생긴 Cartesian 오차를 다듬는다. 양 끝점은 고정해야 이음새가 유지된다.
    polished = blended.copy()
    worst_position = 0.0
    worst_rotation = 0.0
    for index in range(1, len(targets) - 1):
        q, position_error, rotation_error = solve_point(
            model, data, frame_id, targets[index], blended[index],
            args.ik_polish_iter, args.ik_damping, math.radians(args.ik_max_step_deg),
            args.position_tolerance, math.radians(args.orientation_tolerance_deg),
        )
        polished[index] = q
        worst_position = max(worst_position, position_error)
        worst_rotation = max(worst_rotation, math.degrees(rotation_error))

    return polished, {
        "position_error": worst_position,
        "rotation_error_deg": worst_rotation,
        "seam_residual": np.zeros(model.nq),
        "sweep_position_error": max(forward_position, backward_position),
        "sweep_rotation_error_deg": max(forward_rotation, backward_rotation),
        "corner_indices": corners,   # 들어올림/이동/내림 경계 샘플 (구간별 시간 배분용)
    }


def _resample(q_path, total_time, control_dt):
    """smoothstep 시간법칙으로 control_dt 간격 샘플을 만든다. rest-to-rest 다."""
    count = len(q_path)
    u = np.linspace(0.0, 1.0, count)
    steps = max(int(math.ceil(total_time / control_dt)) + 1, 3)
    times = np.linspace(0.0, total_time, steps)
    tau = times / max(total_time, 1.0e-12)
    u_of_tau = 3.0 * tau**2 - 2.0 * tau**3

    q_timed = np.empty((steps, q_path.shape[1]))
    for joint in range(q_path.shape[1]):
        q_timed[:, joint] = np.interp(u_of_tau, u, q_path[:, joint])

    velocity = np.gradient(q_timed, times, axis=0, edge_order=2)
    acceleration = np.gradient(velocity, times, axis=0, edge_order=2)
    return times, q_timed, velocity, acceleration


def time_parameterize(q_path, velocity_limit, acceleration_limit, control_dt):
    """rest-to-rest 시간을 배분하되, **실제 샘플에서 한계를 재서** 만족할 때까지 늘린다.

    해석식(u'max=1.5/T, u''max=6/T²)만 믿으면 실제 샘플에서 한계를 넘는다. 경로가
    u 에 대해 균등하지 않고(폴리라인 꺾임), 유한차분 미분에도 오차가 있기 때문이다.
    실측 결과 해석식만으로는 가속도가 한계의 4.3배까지 나왔다. 그래서 만들고-재고-
    늘리는 방식으로 바꾼다. v ∝ 1/T, a ∝ 1/T² 이라 필요한 배율을 바로 계산할 수 있다.
    """
    count = len(q_path)
    if count < 2:
        zeros = np.zeros_like(q_path)
        return np.zeros(count), q_path.copy(), zeros, zeros

    u = np.linspace(0.0, 1.0, count)
    gradient = np.max(np.abs(np.diff(q_path, axis=0)), axis=0) / max(u[1] - u[0], 1.0e-12)

    total_time = 1.0e-2
    for index in range(len(gradient)):
        if gradient[index] <= 1.0e-12:
            continue
        total_time = max(total_time, 1.5 * gradient[index] / max(velocity_limit[index], 1.0e-12))
        total_time = max(
            total_time, math.sqrt(6.0 * gradient[index] / max(acceleration_limit[index], 1.0e-12))
        )

    for _ in range(12):
        times, q_timed, velocity, acceleration = _resample(q_path, total_time, control_dt)
        velocity_ratio = float(np.max(np.abs(velocity) / velocity_limit.reshape(1, -1)))
        acceleration_ratio = float(np.max(np.abs(acceleration) / acceleration_limit.reshape(1, -1)))
        if velocity_ratio <= 1.0 and acceleration_ratio <= 1.0:
            break
        total_time *= SAFETY * max(velocity_ratio, math.sqrt(max(acceleration_ratio, 1.0e-9)))

    return times, q_timed, velocity, acceleration


def time_parameterize_segments(q_path, breakpoints, velocity_limit, acceleration_limit, control_dt):
    """경로를 breakpoints(모서리 샘플 인덱스)에서 잘라 **구간별로** rest-to-rest 시간 배분한 뒤 이어붙인다.

    왜 구간별인가 (2026-10-20): 들어올림→이동→내림 폴리라인에 smoothstep 하나를 씌우면
    모서리에서 관절 속도 방향이 한 샘플 안에 뒤집혀 가속도 스파이크가 나고, 그 한 점을
    한계 아래로 누르느라 전이 **전체**가 기어간다. 범퍼 move 1->2 실측: 갠트리 1.4 m 이동에
    40.6 s, 최고 속도가 한계의 12 %, 샘플의 99.9 % 가 가속도 한계의 절반도 안 씀.
    가속도 한계를 5 → 1.6 m/s² 로 내리자 1/√a 로 늘어나 드러났다.
    구간별로 나누면 모서리에서 속도가 0 이라 꺾임이 문제가 안 되고, 이동 구간은 한계까지 쓴다.

    각 구간 길이는 control_dt 의 정수배로 올림해 전체 샘플 간격이 균일하다 (한계는 더 여유).
    이음새에서는 양쪽 모두 정지(v=a=0)라 중복 샘플 하나를 버리고 붙인다.
    돌려주는 것: (times, q, velocity, acceleration, segment_times)  — segment_times 는 구간별 길이 [s].
    """
    q_path = np.asarray(q_path, dtype=float)
    count = len(q_path)
    cuts = sorted({int(b) for b in breakpoints if 0 < int(b) < count - 1})
    bounds = [0] + cuts + [count - 1]
    times_all, q_all, v_all, a_all, segment_times = [], [], [], [], []
    offset = 0.0
    for start, stop in zip(bounds[:-1], bounds[1:]):
        if stop <= start:
            continue
        sub = q_path[start:stop + 1]
        times, q_timed, velocity, acceleration = time_parameterize(
            sub, velocity_limit, acceleration_limit, control_dt
        )
        if len(times) > 1:
            steps = int(math.ceil(times[-1] / control_dt - 1.0e-9))
            times, q_timed, velocity, acceleration = _resample(sub, steps * control_dt - 1.0e-9, control_dt)
            times = np.round(times, 6)
        if times_all:
            times, q_timed, velocity, acceleration = times[1:], q_timed[1:], velocity[1:], acceleration[1:]
        times_all.append(times + offset)
        q_all.append(q_timed)
        v_all.append(velocity)
        a_all.append(acceleration)
        segment_times.append(float(times[-1]) if len(times) else 0.0)
        offset = float(times_all[-1][-1]) if len(times_all[-1]) else offset
    if not times_all:
        times, q_timed, velocity, acceleration = time_parameterize(q_path, velocity_limit, acceleration_limit, control_dt)
        return times, q_timed, velocity, acceleration, [float(times[-1]) if len(times) else 0.0]
    return (np.concatenate(times_all), np.concatenate(q_all), np.concatenate(v_all),
            np.concatenate(a_all), segment_times)


def write_segment(path, times, q, velocity, acceleration, joint_names):
    """paint_viz / trajectory_ik_pinocchio 와 같은 SI + 관절이름 형식으로 쓴다."""
    path.parent.mkdir(parents=True, exist_ok=True)
    arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(q, axis=0), axis=1))])
    header = ["t", "s"] + list(joint_names)
    header += [f"qd_{name}" for name in joint_names]
    header += [f"qdd_{name}" for name in joint_names]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        for index, t in enumerate(times):
            writer.writerow(
                [f"{t:.9f}", f"{arc[index]:.9f}"]
                + [f"{value:.9f}" for value in q[index]]
                + [f"{value:.9f}" for value in velocity[index]]
                + [f"{value:.9f}" for value in acceleration[index]]
            )


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="row 사이 전이 궤적을 만들어 전체 도장 시퀀스를 연속으로 잇는다"
    )
    parser.add_argument("--joint-csv-dir", required=True, type=Path,
                        help="row별 joint CSV 디렉터리 (solve_viz_ik / trajectory_ik_pinocchio / trajectory_nlp 출력)")
    parser.add_argument("--pattern", default="*_joint.csv")
    parser.add_argument("--urdf", type=Path, default=None,
                        help="8축 셀 URDF. 생략하면 --gantry-env 의 urdf.primitive")
    add_gantry_env_argument(parser)
    parser.add_argument("--output", required=True, type=Path, help="row+전이를 순서대로 담을 디렉터리")
    parser.add_argument("--tcp-frame", default="auto",
                        help="TCP 프레임. auto = tcp > spray_gun > link_6 중 URDF 에 있는 것")
    parser.add_argument("--retract-dir", default="spray", choices=("spray", "world_z"),
                        help="리트랙트 방향. spray = 양끝 TCP 분사축 반대(표면에서 멀어짐, 세운 부품도 OK), "
                             "world_z = 월드 +Z 고정(예전 방식)")
    parser.add_argument("--retract", type=float, default=DEFAULT_RETRACT_M,
                        help="row 사이에서 들어올릴 높이 [m] (--scene 을 주면 무시)")
    parser.add_argument("--scene", type=Path, default=None,
                        help="config/scene.yaml. 주면 전이마다 필요한 높이를 부품 점군에서 "
                             "계산하는 **적응형 리트랙트**를 쓴다")
    parser.add_argument("--retract-safety", type=float, default=DEFAULT_RETRACT_SAFETY,
                        help="적응형에서 필요 높이에 곱할 안전계수")
    parser.add_argument("--retract-min", type=float, default=DEFAULT_RETRACT_MIN_M,
                        help="적응형 하한 [m]. 필요 높이가 0 이어도 이만큼은 들어올린다")
    parser.add_argument("--retract-max", type=float, default=DEFAULT_RETRACT_MAX_M,
                        help="적응형 상한 [m]")
    parser.add_argument("--step", type=float, default=DEFAULT_STEP_M,
                        help="전이 경로 데카르트 샘플 간격 [m]")
    parser.add_argument("--max-joint-velocity-deg-s", type=float,
                        default=DEFAULT_MAX_JOINT_VELOCITY_DEG_S)
    parser.add_argument("--max-joint-acceleration-deg-s2", type=float,
                        default=DEFAULT_MAX_JOINT_ACCELERATION_DEG_S2)
    parser.add_argument("--max-gantry-velocity-m-s", type=float, default=None,
                        help=f"갠트리 속도 한계 [m/s]. 기본: gantry_env motor_limits, 없으면 {DEFAULT_MAX_GANTRY_VELOCITY_M_S}")
    parser.add_argument("--max-gantry-acceleration-m-s2", type=float, default=None,
                        help=f"갠트리 가속도 한계 [m/s^2]. 기본: gantry_env motor_limits, 없으면 {DEFAULT_MAX_GANTRY_ACCELERATION_M_S2}")
    parser.add_argument("--position-tolerance", type=float, default=0.002)
    parser.add_argument("--orientation-tolerance-deg", type=float, default=2.0)
    parser.add_argument("--ik-max-iter", type=int, default=200)
    parser.add_argument("--ik-damping", type=float, default=0.05)
    parser.add_argument("--ik-max-step-deg", type=float, default=5.0)
    parser.add_argument("--ik-polish-iter", type=int, default=40,
                        help="양끝 블렌딩 후 Cartesian 오차를 다듬는 IK 반복 수")
    parser.add_argument("--control-dt", type=float, default=0.01,
                        help="전이 궤적 샘플 간격 [s] (planner control_dt 와 맞춤)")
    parser.add_argument("--seam-warn-deg", type=float, default=1.0,
                        help="이 값보다 큰 이음새 잔차가 남으면 경고 [deg / 직동은 mm]")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)

    files = sorted(Path(args.joint_csv_dir).glob(args.pattern))
    if len(files) < 2:
        raise FileNotFoundError(f"{args.joint_csv_dir} 에 {args.pattern} 이 2개 이상 있어야 한다")

    env = apply_gantry_env(args, "primitive")
    apply_gantry_motor_limits(args, env)
    model = pin.buildModelFromUrdf(str(args.urdf))
    data = model.createData()
    args.tcp_frame, frame_id = resolve_tcp_frame(model, args.tcp_frame)
    print(f"[stitch] URDF: {args.urdf}" + (f" (gantry_env {env.name})" if env else ""))
    print(f"[stitch] TCP frame: {args.tcp_frame}, 리트랙트 방향: {args.retract_dir}")
    prismatic, velocity_limit, acceleration_limit = limits_of(model, args)
    if prismatic.any():
        print(f"[stitch] 갠트리 한계 v {args.max_gantry_velocity_m_s:g} m/s, a {args.max_gantry_acceleration_m_s2:g} m/s^2")

    rows = []
    for path in files:
        times, joint_names, q = read_joint_csv(path)
        if q.shape[1] != model.nq:
            raise ValueError(
                f"{path.name} 의 관절 수 {q.shape[1]} 가 URDF nq={model.nq} 와 다르다"
            )
        rows.append((path, times, joint_names, q))
    joint_names = rows[0][2]

    output = Path(args.output)
    if output.exists():
        shutil.rmtree(output)
    output.mkdir(parents=True)

    part_tree = load_part_tree(args.scene) if args.scene else None

    print(f"[stitch] row {len(rows)}개, 전이 {len(rows) - 1}곳 생성")
    if part_tree is None:
        print(f"[stitch] 리트랙트 고정 {args.retract * 1000:.0f} mm, "
              f"샘플 간격 {args.step * 1000:.0f} mm")
    else:
        print(f"[stitch] 리트랙트 적응형 (안전계수 {args.retract_safety:.1f}, "
              f"{args.retract_min * 1000:.0f}~{args.retract_max * 1000:.0f} mm), "
              f"샘플 간격 {args.step * 1000:.0f} mm")

    sequence = 0
    total_transition_time = 0.0
    suspect_count = 0
    worst = {"pos": 0.0, "rot": 0.0, "seam": 0.0, "vel_ratio": 0.0, "acc_ratio": 0.0}

    for index, (path, times, _names, q) in enumerate(rows):
        target = output / f"seq_{sequence:04d}_row{index:03d}_joint.csv"
        shutil.copyfile(path, target)
        sequence += 1

        if index == len(rows) - 1:
            break

        q_from = q[-1]
        q_to = rows[index + 1][3][0]

        # planner 가 지그재그 방향을 잘못 매기면(최종 순서 기준 같은 방향 row 가 연달아 나옴) 다음 row 의
        # **반대쪽 끝**이 더 가깝다. 2026-10-06 ES300h_trunk 실측: 10->11, 12->13 전이가 1.5 m (정상 50~90 mm).
        # stitch 는 순서를 바꾸지 않으므로 여기서는 잡아내기만 한다 — 수정은 planner 쪽(4단계 _zigzag_flip_flags).
        p_from_tcp, _ = fk_pose(model, data, frame_id, q_from)
        p_next_start, _ = fk_pose(model, data, frame_id, q_to)
        p_next_end, _ = fk_pose(model, data, frame_id, rows[index + 1][3][-1])
        gap_start = float(np.linalg.norm(p_next_start - p_from_tcp))
        gap_end = float(np.linalg.norm(p_next_end - p_from_tcp))
        direction_suspect = gap_end < gap_start
        if direction_suspect:
            suspect_count += 1
            print(
                f"[stitch] 경고: move {index:2d}->{index + 1:2d} 다음 row 의 반대쪽 끝이 더 가깝다 "
                f"(시작까지 {gap_start * 1000:.0f} mm, 끝까지 {gap_end * 1000:.0f} mm). "
                f"planner 지그재그 방향을 확인할 것"
            )

        lift = args.retract
        if part_tree is not None:
            p_end, _ = fk_pose(model, data, frame_id, q_from)
            p_start, _ = fk_pose(model, data, frame_id, q_to)
            needed = required_retract(part_tree, p_end, p_start, args.step)
            lift = float(np.clip(needed * args.retract_safety,
                                 args.retract_min, args.retract_max))

        q_path, diagnostics = solve_transition_path(
            model, data, frame_id, q_from, q_to, args, retract=lift
        )
        move_times, q_timed, velocity, acceleration, segment_times = time_parameterize_segments(
            q_path, diagnostics["corner_indices"], velocity_limit, acceleration_limit, args.control_dt
        )

        write_segment(
            output / f"seq_{sequence:04d}_move{index:03d}_{index + 1:03d}_joint.csv",
            move_times, q_timed, velocity, acceleration, joint_names,
        )
        sequence += 1

        seam = np.maximum(
            np.abs(q_timed[0] - q_from), np.abs(q_timed[-1] - np.asarray(q_to, dtype=float))
        )
        seam_metric = float(np.max(np.where(prismatic, seam * 1000.0, np.degrees(seam))))
        velocity_ratio = float(np.max(np.abs(velocity) / velocity_limit.reshape(1, -1)))
        acceleration_ratio = float(np.max(np.abs(acceleration) / acceleration_limit.reshape(1, -1)))
        total_transition_time += float(move_times[-1])

        worst["pos"] = max(worst["pos"], diagnostics["position_error"])
        worst["rot"] = max(worst["rot"], diagnostics["rotation_error_deg"])
        worst["seam"] = max(worst["seam"], seam_metric)
        worst["vel_ratio"] = max(worst["vel_ratio"], velocity_ratio)
        worst["acc_ratio"] = max(worst["acc_ratio"], acceleration_ratio)

        flag = "  <-- 이음새 큼" if seam_metric > args.seam_warn_deg else ""
        print(
            f"[stitch] move {index:2d}->{index + 1:2d} pts={len(move_times):4d} "
            f"T={move_times[-1]:5.2f}s ({'+'.join(f'{t:.2f}' for t in segment_times)}) lift={lift * 1000:5.1f}mm "
            f"pos_err={diagnostics['position_error'] * 1000:5.2f}mm "
            f"seam={seam_metric:5.2f} v={velocity_ratio:4.2f} a={acceleration_ratio:4.2f}{flag}"
        )

    print(
        f"[stitch] 완료: 세그먼트 {sequence}개 -> {output}\n"
        f"[stitch] 전이 총 시간 {total_transition_time:.1f}s, "
        f"최악 추종오차 {worst['pos'] * 1000:.2f}mm / {worst['rot']:.2f}deg, "
        f"최악 이음새 {worst['seam']:.3f}, "
        f"속도 한계 대비 {worst['vel_ratio']:.2f}, 가속도 한계 대비 {worst['acc_ratio']:.2f}"
    )
    if worst["vel_ratio"] > 1.0 or worst["acc_ratio"] > 1.0:
        print("[stitch] 경고: 한계 초과 구간이 있다. --retract 를 줄이거나 한계를 확인할 것")
    if suspect_count:
        print(f"[stitch] 경고: row 방향 의심 전이 {suspect_count}곳. planner 지그재그(4단계) 결과를 확인할 것")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
