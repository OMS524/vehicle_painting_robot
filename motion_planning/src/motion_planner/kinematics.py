"""시각화 전용 IK 솔버.

robot_controller/.../trajectory_ik_pinocchio.py 는 실제 로봇 실행용이라
DP 기반 후보 선택 + 관절 점프/속도/가속도 실현성 게이트를 통과해야 하고,
현재 데이터에서는 이 게이트에서 멈춘다. 이 모듈은 RViz 애니메이션을 만들기 위한
가벼운 대안으로, 이전 해를 warm start로 쓰는 damped least squares IK만 수행한다.

  - 실기 실행용이 아니다. 관절 속도/가속도 한계를 보장하지 않는다.
  - 대신 점마다 위치/자세 추종 오차를 기록하고, 허용오차를 넘으면
    "미도달"로 표시해 요약에 남긴다.

출력 CSV는 trajectory_ik_pinocchio.py 의 joint CSV와 같은 형식이라
(t, s, q1..qN, qd1..qdN, qdd1..qddN, 단위 deg) 재생 노드가 그대로 읽는다.
"""

import argparse
import csv
import os
from pathlib import Path

import numpy as np
import pinocchio as pin

from motion_planner.configuration import add_gantry_env_argument, apply_gantry_env
from motion_planner.scene_config import load_scene_config, part_transform
from motion_planner.tcp_frame import resolve_tcp_frame
from motion_planner.data_io import find_trajectory_csv_files, read_trajectory_csv

DEFAULT_TCP_FRAME = "auto"  # tcp > spray_gun > link_6 중 URDF 에 있는 것 (paint_viz.tcp_frame)
# 8축(갠트리 2 + 팔 6) 기본 시드. 직동축은 m, 회전축은 deg.
DEFAULT_INITIAL_JOINT = "0,0,0,0,90,0,90,0"


def quaternion_xyzw_to_matrix(quaternion):
    """xyzw 쿼터니언을 회전행렬로 바꾼다."""
    q = np.asarray(quaternion, dtype=float)
    norm = float(np.linalg.norm(q))
    if norm < 1.0e-12:
        return np.eye(3)
    x, y, z, w = q / norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
        [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
        [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
    ])


def parse_signed_axis(axis):
    text = str(axis).strip().lower()
    sign = 1.0
    if text.startswith("-"):
        sign, text = -1.0, text[1:]
    elif text.startswith("+"):
        text = text[1:]
    if text not in ("x", "y", "z"):
        raise ValueError(f"축 표기가 잘못됐다: {axis}")
    return "xyz".index(text), sign


def remap_planner_rotation_to_tcp(planner_rotation, spray_axis="z", path_axis="x"):
    """planner 회전을 TCP 회전으로 옮긴다.

    trajectory_ik_pinocchio.remap_planner_rotation_to_tcp 와 같은 규약을 쓴다.
    planner 회전의 0열이 경로 진행축, 2열이 분사축이다.
    """
    spray_index, spray_sign = parse_signed_axis(spray_axis)
    path_index, path_sign = parse_signed_axis(path_axis)
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


def clamp_to_joint_limits(model, q):
    return np.clip(q, model.lowerPositionLimit, model.upperPositionLimit)


def build_seed(model, joint_names, initial_joint_text):
    """초기 관절값 문자열을 파싱한다.

    회전 관절은 deg, 직동 관절(갠트리)은 m 로 적는다. 값을 생략하면
    직동축은 0, 회전축은 URDF 한계 안의 0 으로 시작한다.
    """
    text = str(initial_joint_text or "").strip()
    if not text:
        return clamp_to_joint_limits(model, np.zeros(model.nq))

    values = [float(token) for token in text.split(",")]
    if len(values) != model.nq:
        raise ValueError(
            f"--initial-joint 는 {model.nq}개 값이어야 한다 (관절 순서: {', '.join(joint_names)})"
        )

    seed = np.zeros(model.nq)
    for index, name in enumerate(joint_names):
        joint = model.joints[model.getJointId(name)]
        is_prismatic = "P" in joint.shortname().replace("JointModel", "")[:2]
        seed[index] = values[index] if is_prismatic else np.radians(values[index])
    return clamp_to_joint_limits(model, seed)


def solve_point(model, data, frame_id, target, q_seed, max_iter, damping, max_step_rad,
                position_tolerance, rotation_tolerance_rad):
    """한 점에 대해 damped least squares IK를 푼다. (q, pos_err, rot_err)를 돌려준다."""
    q = np.array(q_seed, dtype=float)
    for _ in range(max_iter):
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        current = data.oMf[frame_id]
        error = pin.log6(current.actInv(target)).vector

        position_error = float(np.linalg.norm(target.translation - current.translation))
        rotation_error = float(np.linalg.norm(pin.log3(current.rotation.T @ target.rotation)))
        if position_error <= position_tolerance and rotation_error <= rotation_tolerance_rad:
            break

        jacobian = pin.computeFrameJacobian(model, data, q, frame_id, pin.ReferenceFrame.LOCAL)
        jjt = jacobian @ jacobian.T + (damping ** 2) * np.eye(6)
        dq = jacobian.T @ np.linalg.solve(jjt, error)

        step = float(np.max(np.abs(dq)))
        if step > max_step_rad:
            dq *= max_step_rad / step
        q = clamp_to_joint_limits(model, pin.integrate(model, q, dq))

    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    current = data.oMf[frame_id]
    position_error = float(np.linalg.norm(target.translation - current.translation))
    rotation_error = float(np.linalg.norm(pin.log3(current.rotation.T @ target.rotation)))
    return q, position_error, rotation_error


def solve_row(model, data, frame_id, row, rotation, translation, q_start, args):
    """trajectory row 하나를 풀어 (times, q_path, 통계)를 돌려준다."""
    q = np.array(q_start, dtype=float)
    q_path = []
    position_errors = []
    rotation_errors = []

    for index in range(len(row)):
        world_position = rotation @ row.positions[index] + translation
        planner_rotation = rotation @ quaternion_xyzw_to_matrix(row.orientations[index])
        tcp_rotation = remap_planner_rotation_to_tcp(
            planner_rotation, args.tcp_spray_axis, args.tcp_path_axis
        )
        target = pin.SE3(tcp_rotation, world_position)

        q, position_error, rotation_error = solve_point(
            model, data, frame_id, target, q,
            args.ik_max_iter, args.ik_damping, np.radians(args.ik_max_step_deg),
            args.position_tolerance, np.radians(args.orientation_tolerance_deg),
        )
        q_path.append(q.copy())
        position_errors.append(position_error)
        rotation_errors.append(rotation_error)

    q_path = np.array(q_path)
    position_errors = np.array(position_errors)
    rotation_errors = np.degrees(np.array(rotation_errors))
    unreached = int(np.sum(
        (position_errors > args.position_tolerance)
        | (rotation_errors > args.orientation_tolerance_deg)
    ))
    stats = {
        "points": len(q_path),
        "unreached": unreached,
        "max_position_error": float(position_errors.max()) if len(position_errors) else 0.0,
        "max_rotation_error_deg": float(rotation_errors.max()) if len(rotation_errors) else 0.0,
        # 직동축(m)과 회전축(rad)이 섞여 있어 단위 없는 최대 변화량으로만 본다.
        "max_joint_step": float(np.abs(np.diff(q_path, axis=0)).max()) if len(q_path) > 1 else 0.0,
    }
    return q_path, stats


def joint_names_of(model):
    """pinocchio 모델의 관절 이름을 URDF 순서대로 돌려준다(universe 제외)."""
    return [model.names[index] for index in range(1, len(model.names))]


def write_joint_csv(path, times, arc_lengths, q, joint_names):
    """관절값을 SI 단위(회전 rad, 직동 m)로 저장한다.

    갠트리 직동축이 섞이면 deg 단일 단위를 쓸 수 없어
    trajectory_ik_pinocchio.py 의 deg 형식 대신 SI + 관절이름 헤더를 쓴다.
    """
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["t", "s"] + list(joint_names))
        for index, t in enumerate(times):
            writer.writerow(
                [f"{t:.9f}", f"{arc_lengths[index]:.9f}"]
                + [f"{value:.9f}" for value in q[index]]
            )


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="RViz 애니메이션용 warm-start DLS IK (실기 실행용 아님)"
    )
    parser.add_argument("--scene", required=True, type=Path, help="config/scene.yaml 경로")
    parser.add_argument("--urdf", type=Path, default=None,
                        help="8축 셀 URDF. 생략하면 --gantry-env 의 urdf.primitive")
    add_gantry_env_argument(parser)
    parser.add_argument("--output", required=True, type=Path, help="joint CSV 출력 디렉터리")
    parser.add_argument("--tcp-frame", default=DEFAULT_TCP_FRAME)
    parser.add_argument("--tcp-spray-axis", default="z")
    parser.add_argument("--tcp-path-axis", default="x")
    parser.add_argument(
        "--initial-joint", default=DEFAULT_INITIAL_JOINT,
        help="초기 관절값. 회전축은 deg, 직동축은 m (URDF 관절 순서)",
    )
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--max-rows", type=int, default=0)
    parser.add_argument("--position-tolerance", type=float, default=0.002)
    parser.add_argument("--orientation-tolerance-deg", type=float, default=2.0)
    parser.add_argument("--ik-max-iter", type=int, default=200)
    parser.add_argument("--ik-damping", type=float, default=0.05)
    parser.add_argument("--ik-max-step-deg", type=float, default=5.0)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    apply_gantry_env(args, "primitive")

    scene = load_scene_config(args.scene)
    rotation, translation = part_transform(scene)

    model = pin.buildModelFromUrdf(str(args.urdf))
    data = model.createData()
    args.tcp_frame, frame_id = resolve_tcp_frame(model, args.tcp_frame)

    names = joint_names_of(model)
    seed = build_seed(model, names, args.initial_joint)

    trajectory_dir = scene["trajectory_dir"]
    files = find_trajectory_csv_files(trajectory_dir)
    if args.max_rows:
        files = files[: args.max_rows]
    if not files:
        raise FileNotFoundError(f"trajectory CSV가 없다: {trajectory_dir}")

    print(f"[viz-ik] URDF: {args.urdf}")
    print(f"[viz-ik] TCP frame: {args.tcp_frame} (분사축 {args.tcp_spray_axis}, 경로축 {args.tcp_path_axis})")
    print(f"[viz-ik] part_xyz={scene['part_xyz']} part_rpy={scene['part_rpy']}")
    print(f"[viz-ik] row {len(files)}개 처리")

    args.output.mkdir(parents=True, exist_ok=True)
    q_start = seed
    total_points = 0
    total_unreached = 0

    for order, path in enumerate(files):
        row = read_trajectory_csv(path)
        if args.stride > 1:
            row.positions = row.positions[:: args.stride]
            row.orientations = row.orientations[:: args.stride]
            row.times = row.times[:: args.stride]
            row.paint = row.paint[:: args.stride]

        q_path, stats = solve_row(model, data, frame_id, row, rotation, translation, q_start, args)
        q_start = q_path[-1]

        times = row.times
        arc_lengths = np.concatenate(
            [[0.0], np.cumsum(np.linalg.norm(np.diff(row.positions, axis=0), axis=1))]
        )
        output_csv = args.output / f"{Path(path).stem}_viz_joint.csv"
        write_joint_csv(output_csv, times, arc_lengths, q_path, names)

        total_points += stats["points"]
        total_unreached += stats["unreached"]
        print(
            f"[viz-ik] row {row.row_index:3d} pts={stats['points']:5d} "
            f"미도달={stats['unreached']:5d} "
            f"pos_err_max={stats['max_position_error']:.4f}m "
            f"rot_err_max={stats['max_rotation_error_deg']:.2f}deg "
            f"joint_step_max={stats['max_joint_step']:.3f} -> {output_csv.name}",
            flush=True,
        )

    reached = total_points - total_unreached
    ratio = 100.0 * reached / max(total_points, 1)
    print(f"[viz-ik] 완료: {reached}/{total_points} 점 추종 ({ratio:.1f}%)")
    if total_unreached:
        print("[viz-ik] 경고: 미도달 점이 있다. part_xyz/part_rpy 배치를 조정하거나 "
              "레일/갠트리 축을 추가해야 한다.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
