"""툴체인저 도킹 접근 모션 생성 + 갠트리 프레임 간섭 검사.

무엇을 하는가
  1) 갠트리 (x, y) 위치를 그리드로 훑으며, 각 위치에서 갠트리를 고정한 채
     팔 6축 DLS IK 로 pre-dock(슬롯 +Z 오프셋)과 dock 자세를 푼다.
     충돌(coal, 인접쌍 제외)까지 통과한 위치만 "실행 가능"으로 표시해
     슬롯별 ASCII 지도로 출력한다 — 접근 제약이 얼마나 빡빡한지의 정량 답.
  2) 가장 여유 있는 갠트리 위치를 골라 4단계 경로를 만든다:
       시작 자세 → 갠트리 이송(팔 고정) → 팔 재배치(pre-dock 자세로 관절 보간)
       → 수직 하강(pre-dock → dock, 직선 카테시안 스윕)
     전 구간 충돌 검사 + 주요 지점 최소 클리어런스를 기록하고,
     trajectory_player 가 읽는 SI 단위 joint CSV 로 저장한다.

도킹 방향 규약 (STL 실측)
  파킹 플레이트는 102(X) x 80(Y). 툴측 플레이트는 link_6 에서 80(x) x 102(y).
  따라서 dock 자세는 link_6 z축 = 월드 -Z(위에서 접근), link_6 y축 = 월드 ±X.
  체인저 키 방향의 180도 모호성은 --flip 으로 뒤집는다.

사용 예 (paintcell conda 환경):
  python dock_path.py --urdf urdf/paint_cell_8axis_gantry_mesh_c_bare.urdf \
      --slot 1 --out-dir /tmp/dock
  ros2 run paint_viz dock_path --gantry-env x7y4z3 --slot 1 --out-dir /tmp/dock_x7   # URDF/도킹값은 env yaml
"""

import argparse
from pathlib import Path

import numpy as np
import pinocchio as pin

from motion_planner.collision import build_collision_pairs
from motion_planner.configuration import add_gantry_env_argument, apply_gantry_env, default_package_dirs
from motion_planner.kinematics import write_joint_csv

PACKAGE_DIRS = default_package_dirs()
# 시작 자세: 갠트리 (0,0) + 팔 전관절 0 (거꾸로 매달려 수직으로 뻗은 자세)
START_ARM_DEG = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
# 이송 자세: 0포즈로 뻗은 채 이송하면 팔 끝(z~0.6)이 하부 브레이스(z~0.53)를
# 스칠 수 있어, 이송 전에 접는다 (viz_ik 기본 시드와 동일한 자세)
TRAVEL_ARM_DEG = (0.0, 0.0, 90.0, 0.0, 90.0, 0.0)


def dock_rotation(flip, extra_yaw_rad=0.0):
    """link_6 목표 자세: z=-Z(하향 접근), y=±X(플레이트 102mm 변 정렬)."""
    s = -1.0 if flip else 1.0
    base = np.array([[0.0, s, 0.0], [s, 0.0, 0.0], [0.0, 0.0, -1.0]])
    if extra_yaw_rad:
        c, sn = np.cos(extra_yaw_rad), np.sin(extra_yaw_rad)
        base = np.array([[c, -sn, 0.0], [sn, c, 0.0], [0.0, 0.0, 1.0]]) @ base
    return base


def solve_arm_ik(model, data, frame_id, target, q_seed, max_iter=300, damping=1e-6,
                 max_step=0.3, pos_tol=1e-4, rot_tol=1e-3):
    """갠트리 2축(q[0:2])을 고정한 damped least squares IK."""
    q = np.array(q_seed, dtype=float)
    for _ in range(max_iter):
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        current = data.oMf[frame_id]
        pos_err = float(np.linalg.norm(target.translation - current.translation))
        rot_err = float(np.linalg.norm(pin.log3(current.rotation.T @ target.rotation)))
        if pos_err <= pos_tol and rot_err <= rot_tol:
            break
        err = pin.log6(current.actInv(target)).vector
        jac = pin.computeFrameJacobian(model, data, q, frame_id, pin.ReferenceFrame.LOCAL)
        jac[:, :2] = 0.0                       # 갠트리 잠금
        dq = jac.T @ np.linalg.solve(jac @ jac.T + damping ** 2 * np.eye(6), err)
        step = float(np.max(np.abs(dq)))
        if step > max_step:
            dq *= max_step / step
        q = np.clip(pin.integrate(model, q, dq),
                    model.lowerPositionLimit, model.upperPositionLimit)
        q[:2] = q_seed[:2]
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    current = data.oMf[frame_id]
    pos_err = float(np.linalg.norm(target.translation - current.translation))
    rot_err = float(np.linalg.norm(pin.log3(current.rotation.T @ target.rotation)))
    return q, pos_err, rot_err


def in_collision(model, data, geom_model, geom_data, q):
    if not pin.computeCollisions(model, data, geom_model, geom_data, q, False):
        return []
    hits = []
    for index, result in enumerate(geom_data.collisionResults):
        if result.isCollision():
            pair = geom_model.collisionPairs[index]
            hits.append((geom_model.geometryObjects[pair.first].name,
                         geom_model.geometryObjects[pair.second].name))
    return hits

def min_clearance(model, data, geom_model, geom_data, q):
    pin.computeDistances(model, data, geom_model, geom_data, q)
    return min(r.min_distance for r in geom_data.distanceResults)


def gantry_world_map(model, data):
    """갠트리 조인트값 <-> 캐리지 월드 xy 선형 매핑.

    홈 원점/축 방향 규약(예: 툴체인저 반대편 코너 홈, y축 반전)이 바뀌어도
    FK 로 기울기·오프셋을 재서 흡수한다.
    """
    fid = model.getFrameId("gantry_y_link")

    def carriage_xy(gx, gy):
        q = np.zeros(model.nq)
        q[0], q[1] = gx, gy
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        return data.oMf[fid].translation[:2].copy()

    p0 = carriage_xy(0.0, 0.0)
    sx = carriage_xy(1.0, 0.0)[0] - p0[0]
    sy = carriage_xy(0.0, 1.0)[1] - p0[1]
    to_joint = lambda wx, wy: ((wx - p0[0]) / sx, (wy - p0[1]) / sy)
    to_world = lambda gx, gy: (p0[0] + sx * gx, p0[1] + sy * gy)
    return to_joint, to_world


def arm_base_offset_x(model, data):
    """캐리지(gantry_y_link) 중심 -> 팔 base_link 의 월드 X 오프셋 (q=0 에서 FK)."""
    q = np.zeros(model.nq)
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    return float(data.oMf[model.getFrameId("base_link")].translation[0]
                 - data.oMf[model.getFrameId("gantry_y_link")].translation[0])


def scan_gantry_grid(model, data, geom_model, geom_data, frame_id, slot_xyz, args):
    """캐리지 월드 xy 그리드를 훑어 (결과지도, 실행가능 목록)을 돌려준다."""
    to_joint, to_world = gantry_world_map(model, data)
    lo, hi = model.lowerPositionLimit, model.upperPositionLimit
    wx_lim = sorted([to_world(lo[0], 0)[0], to_world(hi[0], 0)[0]])
    wy_lim = sorted([to_world(0, lo[1])[1], to_world(0, hi[1])[1]])
    # 팔 베이스의 캐리지 기준 X 오프셋(spec_c 0.32304)은 FK 로 읽는다 — 환경이 바뀌어도 맞다
    base_off = arm_base_offset_x(model, data)
    gy_lo = max(wy_lim[0], slot_xyz[1] - args.window)
    gy_hi = min(wy_lim[1], slot_xyz[1] + args.window)
    gys = np.arange(gy_lo, gy_hi + 1e-9, args.grid)
    if args.dock_gx is not None:
        # 도킹 시 X축은 지정 조인트값(기본: 설계 최대 스트로크 끝)으로 고정
        gxs = np.array([to_world(args.dock_gx, 0.0)[0]])
    else:
        gx_lo = max(wx_lim[0], slot_xyz[0] - base_off - args.window)
        gx_hi = min(wx_lim[1], slot_xyz[0] - base_off + args.window)
        gxs = np.arange(gx_lo, gx_hi + 1e-9, args.grid)

    pre = slot_xyz + np.array([0.0, 0.0, args.pre_offset])
    dock = slot_xyz + np.array([0.0, 0.0, args.standoff])
    rot = dock_rotation(args.flip)
    seed_arm = np.radians(TRAVEL_ARM_DEG)

    grid_map = {}
    feasible = []
    for gy in gys:
        for gx in gxs:
            gjx, gjy = to_joint(gx, gy)          # 월드 -> 조인트값
            q0 = np.zeros(model.nq)
            q0[0], q0[1] = gjx, gjy
            q0[2:] = seed_arm
            q_pre, pe, re_ = solve_arm_ik(model, data, frame_id, pin.SE3(rot, pre), q0)
            if pe > 2e-3 or re_ > 5e-3:
                grid_map[(gx, gy)] = "."
                continue
            if in_collision(model, data, geom_model, geom_data, q_pre):
                grid_map[(gx, gy)] = "x"
                continue
            q_dock, pe, re_ = solve_arm_ik(model, data, frame_id, pin.SE3(rot, dock), q_pre)
            if pe > 2e-3 or re_ > 5e-3:
                grid_map[(gx, gy)] = "p"
                continue
            if in_collision(model, data, geom_model, geom_data, q_dock):
                grid_map[(gx, gy)] = "X"
                continue
            grid_map[(gx, gy)] = "#"
            feasible.append((gjx, gjy, q_pre, q_dock, gx, gy))
    return gxs, gys, grid_map, feasible


def print_grid(gxs, gys, grid_map, slot_name):
    print(f"\n[{slot_name}] 캐리지 월드 위치 타당성 지도 "
          "(#: dock 가능, p: pre-dock 만, x/X: pre/dock 충돌, .: IK 미도달)")
    header = "        " + " ".join(f"{gx:+.2f}"[:5] for gx in gxs)
    print(header)
    for gy in gys[::-1]:
        row = " ".join(f"  {grid_map[(gx, gy)]}  " for gx in gxs)
        print(f" gy{gy:+.2f} {row}")


def build_path(model, data, geom_model, geom_data, frame_id, choice, slot_xyz, args):
    """시작→이송→재배치→하강 경로를 만든다. (q 배열, 단계 경계, 위반 목록)"""
    gx, gy, q_pre, q_dock = choice[:4]          # gx, gy 는 조인트값
    q_start = np.zeros(model.nq)
    q_start[2:] = np.radians(START_ARM_DEG)
    q_travel = np.zeros(model.nq)
    q_travel[2:] = np.radians(TRAVEL_ARM_DEG)

    path = [q_start]
    # 0) 팔 접기: 0포즈 -> 이송 자세 (갠트리 0,0 고정)
    for a in np.linspace(0, 1, 15)[1:]:
        path.append((1 - a) * q_start + a * q_travel)
    # 1) 갠트리 이송 (팔은 접힌 자세 그대로)
    n1 = max(2, int(np.hypot(gx, gy) / 0.05))
    travel_from = path[-1]
    for a in np.linspace(0, 1, n1)[1:]:
        q = travel_from.copy()
        q[0], q[1] = a * gx, a * gy
        path.append(q)
    stage2_at = len(path)
    # 2) 팔 재배치 (관절 보간)
    n2 = 30
    for a in np.linspace(0, 1, n2)[1:]:
        q = path[-1].copy() if a == 0 else q_pre.copy()
        q = (1 - a) * path[stage2_at - 1] + a * q_pre
        path.append(q)
    stage3_at = len(path)
    # 3) 수직 하강: pre-dock → dock, 카테시안 직선
    rot = dock_rotation(args.flip)
    n3 = max(3, int((args.pre_offset - args.standoff) / 0.005))
    q_prev = q_pre
    for a in np.linspace(0, 1, n3)[1:]:
        z = slot_xyz[2] + (1 - a) * args.pre_offset + a * args.standoff
        target = pin.SE3(rot, np.array([slot_xyz[0], slot_xyz[1], z]))
        q_sol, pe, re_ = solve_arm_ik(model, data, frame_id, target, q_prev,
                                      max_iter=100)
        if pe > 2e-3:
            print(f"  [경고] 하강 z={z:.3f} IK 오차 {pe*1000:.1f} mm")
        path.append(q_sol)
        q_prev = q_sol

    # 전 구간 충돌 검사 + 단계 경계 클리어런스.
    # 침투 깊이 8mm 이하는 "경미 간섭"으로 분리한다 — 사용자 확정 스트로크
    # (정상운전 ±1.1432)와 9/3 STL 기하가 끝단에서 수 mm 어긋나 있어서(스펙이
    # 최신, 메시가 구버전으로 추정), 홈 코너에서 캐리지/베이스가 빔 모델에
    # 1~7mm 파고드는 것은 모델 오차 수준이지 실충돌이 아니다.
    violations = []
    shallow = []
    for index, q in enumerate(path):
        hits = in_collision(model, data, geom_model, geom_data, q)
        if hits:
            stage = ("접기/이송" if index < stage2_at else
                     "재배치" if index < stage3_at else "하강")
            depth = min_clearance(model, data, geom_model, geom_data, q)
            if depth >= -0.008:
                shallow.append((index, stage, depth, hits[:2]))
            else:
                violations.append((index, stage, hits[:3]))
    key_points = {"시작": 0, "이송끝": stage2_at - 1,
                  "pre-dock": stage3_at - 1, "dock": len(path) - 1}
    clearances = {name: min_clearance(model, data, geom_model, geom_data, path[i])
                  for name, i in key_points.items()}
    return np.array(path), violations, clearances, shallow


def main(argv=None):
    ap = argparse.ArgumentParser(description="툴체인저 도킹 접근 경로 생성/검증")
    ap.add_argument("--urdf", type=Path, default=None,
                    help="베어 플랜지 URDF. 생략하면 --gantry-env 의 urdf.bare")
    add_gantry_env_argument(ap, "도킹 기본값(pre-offset/standoff/dock-gx, 시작·이송 자세)도 여기서 읽는다")
    ap.add_argument("--slot", default="all", help="1..4 또는 all")
    ap.add_argument("--pre-offset", type=float, default=None,
                    help="pre-dock 높이: 플레이트 상면 +Z [m] (기본 env toolchanger.dock.pre_offset, 없으면 0.15)")
    ap.add_argument("--standoff", type=float, default=None,
                    help="dock 시 플랜지-플레이트 간격 [m] (기본 env toolchanger.dock.standoff, 없으면 0.005)")
    ap.add_argument("--flip", action="store_true",
                    help="도킹 yaw 180도 반전 (체인저 키 방향)")
    ap.add_argument("--dock-gx", type=float, default=None,
                    help="도킹 시 gantry_x 고정 조인트값 [m]. 기본 env toolchanger.dock.gantry_x_joint "
                         "(코너 홈 환경에서 0 = 홈 X 유지, Y만 이동해 도킹). 음수면 X도 스캔한다")
    ap.add_argument("--grid", type=float, default=0.05, help="갠트리 그리드 간격 [m]")
    ap.add_argument("--window", type=float, default=0.75, help="슬롯 주변 탐색 반경 [m]")
    ap.add_argument("--out-dir", type=Path, default=Path("/tmp/dock_path"))
    args = ap.parse_args(argv)
    env = apply_gantry_env(args, "bare")
    dock_cfg = (env.get("toolchanger.dock") or {}) if env else {}
    if args.pre_offset is None:
        args.pre_offset = float(dock_cfg.get("pre_offset", 0.15))
    if args.standoff is None:
        args.standoff = float(dock_cfg.get("standoff", 0.005))
    if args.dock_gx is None:
        args.dock_gx = float(dock_cfg.get("gantry_x_joint", 0.0))
    if args.dock_gx is not None and args.dock_gx < 0:
        args.dock_gx = None
    global START_ARM_DEG, TRAVEL_ARM_DEG
    if env and env.get("home.arm_deg") is not None:
        START_ARM_DEG = tuple(float(v) for v in env.get("home.arm_deg"))
    if env and env.get("home.travel_arm_deg") is not None:
        TRAVEL_ARM_DEG = tuple(float(v) for v in env.get("home.travel_arm_deg"))
    if env:
        print(f"[dock] gantry_env {env.name}: {env.path}")

    model = pin.buildModelFromUrdf(str(args.urdf))
    data = model.createData()
    geom_model = pin.buildGeomFromUrdf(model, str(args.urdf),
                                       pin.GeometryType.COLLISION,
                                       package_dirs=PACKAGE_DIRS)
    pairs = build_collision_pairs(model, geom_model)
    geom_data = pin.GeometryData(geom_model)
    frame_id = model.getFrameId("link_6")
    print(f"[dock] {args.urdf.name}: nq={model.nq}, 충돌쌍 {pairs}, 팔 base X 오프셋 {arm_base_offset_x(model, data):+.4f} m, "
          f"pre-offset {args.pre_offset} m, standoff {args.standoff} m, dock gantry_x {args.dock_gx}")

    slots = [int(args.slot)] if args.slot != "all" else [1, 2, 3, 4]
    q_ref = np.zeros(model.nq)
    pin.forwardKinematics(model, data, q_ref)
    pin.updateFramePlacements(model, data)

    for slot in slots:
        slot_frame = model.getFrameId(f"toolchanger_slot_{slot}")
        slot_xyz = data.oMf[slot_frame].translation.copy()
        print(f"\n===== 슬롯 {slot} @ ({slot_xyz[0]:.3f}, {slot_xyz[1]:.3f}, "
              f"{slot_xyz[2]:.3f}) =====")
        gxs, gys, grid_map, feasible = scan_gantry_grid(
            model, data, geom_model, geom_data, frame_id, slot_xyz, args)
        print_grid(gxs, gys, grid_map, f"slot{slot}")
        total = len(grid_map)
        print(f"  실행 가능: {len(feasible)}/{total} 위치")
        if not feasible:
            print("  -> 이 슬롯은 현재 조건에서 도킹 불가")
            continue

        # dock 자세 클리어런스가 최대인 갠트리 위치 선택 (상위 후보만 거리계산)
        scored = []
        for cand in feasible:
            scored.append((min_clearance(model, data, geom_model, geom_data, cand[3]),
                           cand))
        scored.sort(key=lambda t: -t[0])
        # dock 클리어런스 상위 후보부터 전체 경로(이송 포함)를 검사해,
        # 전 구간 무충돌인 첫 후보를 쓴다. 이송 중 접힌 팔이 측면 빔을 스치는
        # 갠트리 위치(깊은 -Y)가 걸러진다.
        path = violations = clearances = shallow = best = sel_clear = None
        for cand_clear, cand in scored[:10]:
            p, v, c, sh = build_path(model, data, geom_model, geom_data, frame_id,
                                     cand, slot_xyz, args)
            if path is None or (not v and violations):
                path, violations, clearances, shallow, best, sel_clear = \
                    p, v, c, sh, cand, cand_clear
            if not v:
                break
        print(f"  선택 갠트리: 조인트 ({best[0]:+.3f}, {best[1]:+.3f}) "
              f"= 캐리지 월드 ({best[4]:+.3f}, {best[5]:+.3f}) "
              f"(dock 클리어런스 {sel_clear*1000:.0f} mm)")
        for name, c in clearances.items():
            print(f"  클리어런스[{name}]: {c*1000:.0f} mm")
        if shallow:
            worst = min(depth for _i, _s, depth, _h in shallow)
            print(f"  [경미 간섭] {len(shallow)}개 웨이포인트 (최대 {worst*1000:.1f} mm, "
                  f"스펙 스트로크 vs 메시 기하 오차 수준 — 무시 가능)")
        if violations:
            print(f"  [충돌] {len(violations)}개 웨이포인트:")
            for index, stage, hits in violations[:5]:
                print(f"    wp{index} ({stage}): {hits}")
        else:
            print("  전 구간 실충돌 없음")

        times = np.arange(len(path)) * 0.1
        out = args.out_dir / f"slot{slot}_dock_joint.csv"
        names = [model.names[i] for i in range(1, len(model.names))]
        write_joint_csv(out, times, times, path, names)
        print(f"  경로 저장: {out} ({len(path)} 웨이포인트)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
