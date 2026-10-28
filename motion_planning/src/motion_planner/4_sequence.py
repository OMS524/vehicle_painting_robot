"""툴체인저 접근 + 도장 경로를 한 시퀀스로 잇는다 (build_sequence).

시퀀스 (config/sequence/<name>.yaml 의 segments 로 선언)
  home → dock_approach(슬롯 N, 빈 플랜지) → tool_attach → dock_depart
       → transit(도장 시작점 위로) → paint(stitched row+전이 CSV 그대로) → retreat
       → transit(툴체인저로) → dock_return → tool_detach → dock_depart(빈 플랜지) → home

설계 요점
  - segment 는 전부 rest-to-rest 다 (도장 row 는 planner 가 양끝 속도 0 으로 만들고, 전이·도킹 구간은
    smoothstep 시간법칙이라 양끝 속도 0). 그래서 경계에서는 **관절값 일치**만 보장하면 속도가 튀지 않는다.
    각 segment 는 이전 segment 의 마지막 q 에서 시작하고(warm start), 끝 q 는 다음 segment 의 첫 q 와
    정확히 같게 만든다(양끝 IK + smoothstep 블렌딩: stitch.solve_transition_path).
  - TCP 전환: 관절값 q 는 TCP 와 무관하므로 tool_attach/tool_detach 는 0 초 이벤트다. 그 뒤 segment 의
    목표만 tcp(노즐 팁) / link_6(빈 플랜지) 로 바뀐다. 충돌 검사는 이벤트 전후로 베어 URDF / 툴 URDF 를 나눠 쓰고,
    툴을 든 동안에는 그 슬롯의 파킹 플레이트 박스를 검사에서 뺀다(플레이트가 툴과 함께 움직이므로).
  - 회전은 내부에서 전부 회전행렬/log3 slerp 로 다루므로 쿼터니언 부호 문제가 없다. 통합 CSV 의 TCP 쿼터니언은
    이전 샘플과 내적이 음수면 부호를 뒤집어 반구 연속성을 맞춘다.
  - 긴 이동(툴체인저 ↔ 부품)은 "팔을 이송 자세로 접고 갠트리만 이동 → 팔 재배치(관절 보간) → 목표 위에서
    Cartesian 직선 하강" 으로 나눈다. 하강/상승 구간은 TCP 속도 한계(yaml speeds)로 시간을 배분한다.

출력 (--out-dir)
  seq_NNNN_<segment>_<stage>_joint.csv   stage 별 CSV (stitch 와 같은 t,s,q,qd,qdd 26열, SI). paint 는 stitched 파일 복사.
  sequence.json                           매니페스트: segment/stage 순서, 누적 시간, TCP 프레임, 툴 상태, 검증 결과.
  sequence_command.csv                    실행용 단일 1 ms CSV (t, s, q, qd, qdd, segment, tool, tcp_frame, TCP pose).

사용
  ros2 run paint_viz build_sequence --gantry-env x7y4z3 --part ES300h_hood --out-dir $NLP4CM/sequence_x7y4z3/ES300h_hood   (nlp4cm/sequence_all.sh 가 전 파츠)
  ros2 launch paint_viz paint_viz.launch.py gantry_env:=x7y4z3 joint_csv_dir:=<out-dir> joint_csv_pattern:="seq_*_joint.csv" \
      scene:=<scene yaml>
"""

import argparse
import csv
import json
import math
import shutil
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pinocchio as pin
import yaml

from motion_planner import dock_path as dock
from motion_planner.collision import build_collision_pairs
from motion_planner.configuration import (add_gantry_env_argument, default_package_dirs, expand_project_path,
                                  load_gantry_env)
from motion_planner.transition import (_resample, fk_pose, limits_of, prismatic_mask, solve_transition_path,
                              time_parameterize, write_segment)
from motion_planner.data_io import read_joint_csv

DEFAULT_SEQUENCE = "paint_with_tool"


# =============================================================================
# 설정
# =============================================================================
def sequence_config_dirs():
    from motion_planner.configuration import config_dirs
    return [d.parent / "sequence" for d in config_dirs()]


def load_sequence_config(name_or_path):
    path = Path(str(name_or_path)).expanduser()
    if not path.is_file():
        for directory in sequence_config_dirs():
            candidate = directory / f"{name_or_path}.yaml"
            if candidate.is_file():
                path = candidate
                break
    if not path.is_file():
        raise FileNotFoundError(f"sequence yaml 을 못 찾았다: {name_or_path} ({[str(d) for d in sequence_config_dirs()]})")
    with open(path, "r", encoding="utf-8") as stream:
        loaded = yaml.safe_load(stream) or {}
    cfg = loaded.get("sequence", loaded)
    cfg["_path"] = str(path.resolve())
    return cfg


# =============================================================================
# 모델 / 충돌
# =============================================================================
class CellModels:
    """운동학 모델 하나(툴 URDF) + 충돌 형상 두 벌(베어 / 툴)."""

    def __init__(self, env, package_dirs):
        self.env = env
        self.tool_urdf = env.urdf_path("collision")
        self.bare_urdf = env.urdf_path("bare")
        self.model = pin.buildModelFromUrdf(str(self.tool_urdf))
        self.data = self.model.createData()
        bare_model = pin.buildModelFromUrdf(str(self.bare_urdf))
        if bare_model.nq != self.model.nq or [bare_model.names[i] for i in range(bare_model.njoints)] != \
                [self.model.names[i] for i in range(self.model.njoints)]:
            raise ValueError("베어 URDF 와 툴 URDF 의 관절 구성이 다르다")
        self.names = [self.model.names[i] for i in range(1, self.model.njoints)]
        self.prismatic = prismatic_mask(self.model)
        self.geom = {}
        for key, urdf, model in (("bare", self.bare_urdf, bare_model), ("tool", self.tool_urdf, self.model)):
            geom_model = pin.buildGeomFromUrdf(model, str(urdf), pin.GeometryType.COLLISION,
                                               package_dirs=list(package_dirs))
            build_collision_pairs(model, geom_model)
            self.geom[key] = (model, model.createData(), geom_model)
        self.frame = {name: self.model.getFrameId(name) for name in ("link_6", "base_link", "gantry_y_link")}
        if self.model.existFrame("tcp"):
            self.frame["tcp"] = self.model.getFrameId("tcp")

    def frame_id(self, name):
        if name not in self.frame:
            if not self.model.existFrame(name):
                raise ValueError(f"URDF 에 프레임 {name} 이 없다")
            self.frame[name] = self.model.getFrameId(name)
        return self.frame[name]

    def slot_xyz(self, slot):
        q = np.zeros(self.model.nq)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.model.getFrameId(f"toolchanger_slot_{slot}")].translation.copy()

    def collision_set(self, tool_attached, carried_slot=None, dock_zone_slot=None, dock_zone_radius=0.3):
        """(model, data, geom_model, geom_data).

        툴을 들었으면 툴 형상을 쓰고 그 슬롯의 파킹 플레이트 박스는 뺀다(플레이트가 툴과 함께 움직인다).
        dock_zone_slot 을 주면(툴을 든 채 슬롯 바로 위에서 수직으로 오르내리는 구간·dwell) 툴 어셈블리 박스
        (tool_assy_*)와 슬롯 반경 안 스탠드 형상의 쌍도 뺀다 — 도킹 상태에서는 툴 몸체가 스탠드 브래킷 절개부
        안에 들어가 있어서 박스 근사끼리는 구조상 겹치는 것이 정상이다. 스프레이건(spray_gun_*)은 계속 검사한다.
        """
        model, data, geom_model = self.geom["tool" if tool_attached else "bare"]
        geom_model = geom_model.clone()
        drop = set()
        zone = set()
        zone_tool = set()
        if tool_attached and carried_slot is not None:
            slot = self.slot_xyz(carried_slot)
            plate = tuple(float(v) for v in (self.env.get("toolchanger.plate_size") or (0.102, 0.08, 0.044)))
            center = slot - np.array([0.0, 0.0, plate[2] / 2.0])
            for index, geom in enumerate(geom_model.geometryObjects):
                if geom.parentJoint == 0 and geom.name.startswith("world_base") and \
                        np.linalg.norm(geom.placement.translation - center) < 0.01:
                    drop.add(index)
        if tool_attached and dock_zone_slot is not None:
            slot = self.slot_xyz(dock_zone_slot)
            for index, geom in enumerate(geom_model.geometryObjects):
                if geom.parentJoint == 0 and geom.name.startswith("world_base") and \
                        np.linalg.norm(geom.placement.translation - slot) < dock_zone_radius:
                    zone.add(index)
                if geom.name.startswith("tool_assy"):   # 체인저 플레이트/어댑터/헤드 — 스탠드 브래킷 절개부를 지나간다
                    zone_tool.add(index)
        if drop or zone:
            keep = [pair for pair in geom_model.collisionPairs
                    if pair.first not in drop and pair.second not in drop
                    and not ({pair.first, pair.second} & zone and {pair.first, pair.second} & zone_tool)]
            geom_model.removeAllCollisionPairs()
            for pair in keep:
                geom_model.addCollisionPair(pair)
        return model, data, geom_model, pin.GeometryData(geom_model)


def collisions_at(cset, q):
    model, data, geom_model, geom_data = cset
    if not pin.computeCollisions(model, data, geom_model, geom_data, q, False):
        return []
    return [(geom_model.geometryObjects[geom_model.collisionPairs[i].first].name,
             geom_model.geometryObjects[geom_model.collisionPairs[i].second].name)
            for i, result in enumerate(geom_data.collisionResults) if result.isCollision()]


def check_path_collisions(cset, q_path, stride):
    hits = {}
    worst = 0.0
    model, data, geom_model, geom_data = cset
    for q in q_path[::stride]:
        pairs = collisions_at(cset, q)
        if not pairs:
            continue
        pin.computeDistances(model, data, geom_model, geom_data, q)
        worst = min(worst, min(r.min_distance for r in geom_data.distanceResults))
        for pair in pairs:
            hits[pair] = hits.get(pair, 0) + 1
    return hits, -worst


# =============================================================================
# 경로 조각 생성
# =============================================================================
def joint_interp(q_a, q_b, step_rad=math.radians(1.0), step_m=0.01, prismatic=None):
    q_a = np.asarray(q_a, dtype=float)
    q_b = np.asarray(q_b, dtype=float)
    delta = np.abs(q_b - q_a)
    if prismatic is None:
        prismatic = np.zeros(len(q_a), dtype=bool)
    count = int(max(np.where(prismatic, delta / step_m, delta / step_rad).max(), 1.0)) + 1
    count = max(count, 2)
    ratio = np.linspace(0.0, 1.0, count).reshape(-1, 1)
    return q_a + ratio * (q_b - q_a)


def lifted_pose(models, frame, q, lift):
    """q 에서의 frame 자세를 분사축(+z) 반대 방향으로 lift 만큼 띄운 SE3."""
    p, rot = fk_pose(models.model, models.data, models.frame_id(frame), q)
    return pin.SE3(rot, p + lift_direction(rot) * lift)


def solve_locked_gantry(models, frame, target, q_seed, tol=2.0e-3, rot_tol=5.0e-3):
    q, pe, re_ = dock.solve_arm_ik(models.model, models.data, models.frame_id(frame), target, q_seed)
    if pe > tol or re_ > rot_tol:
        raise RuntimeError(f"IK 실패 ({frame}, pos {pe * 1000:.1f} mm, rot {math.degrees(re_):.2f} deg)")
    return q


def lift_direction(rot):
    """도장 시작/끝에서 물러나는 단위 방향.

    분사축(+z)의 반대 방향을 기준으로, 표면이 세워져 있으면(반대 방향의 수평 성분이 45° 이상) **수평 성분만** 쓰고
    (갠트리 평행이동으로 정확히 실행되고, 팔이 끝까지 뻗은 하단 row 에서 더 내려가야 하는 일이 없다),
    표면이 누워 있으면 월드 +Z 로 들어올린다. 두 경우 모두 부품에서 멀어지는 방향이다.
    """
    away = -np.asarray(rot)[:, 2]
    horizontal = np.array([away[0], away[1], 0.0])
    if np.linalg.norm(horizontal) >= math.sin(math.radians(45.0)):
        return horizontal / np.linalg.norm(horizontal)
    return np.array([0.0, 0.0, 1.0])


def solve_lifted(models, frame, q, lift, ik_args):
    """q 에서 분사축 반대로 lift 만큼 띄운 자세의 관절해.

    띄우는 벡터의 수평(XY) 성분은 갠트리가 **그대로 평행이동**해 정확히 흡수하고(갠트리는 순수 병진이라 팔 자세를
    유지한 채 TCP 가 같은 벡터만큼 옮겨진다), 수직(Z) 성분만 갠트리를 잠근 팔 6축 IK 로 푼다. 팔이 끝까지 뻗은
    row(세운 펜더 하단 등)에서 수평으로 물러날 때 팔만으로는 도달 밖이 되는 문제를 피한다.
    """
    p, rot = fk_pose(models.model, models.data, models.frame_id(frame), q)
    lift_vec = lift_direction(rot) * lift
    target = pin.SE3(rot, p + lift_vec)
    to_joint, to_world = dock.gantry_world_map(models.model, models.data)
    q_seed = np.array(q, dtype=float)
    wx, wy = to_world(q_seed[0], q_seed[1])
    q_seed[0], q_seed[1] = to_joint(wx + lift_vec[0], wy + lift_vec[1])
    lo, hi = models.model.lowerPositionLimit[:2], models.model.upperPositionLimit[:2]
    if np.any(q_seed[:2] < lo - 1e-9) or np.any(q_seed[:2] > hi + 1e-9):
        q_seed[:2] = np.clip(q_seed[:2], lo, hi)   # 갠트리 끝이면 나머지는 팔이 맡는다
    return solve_locked_gantry(models, frame, target, q_seed,
                               tol=ik_args.position_tolerance, rot_tol=math.radians(ik_args.orientation_tolerance_deg))


def cartesian_line(models, frame, q_from, q_to, ik_args):
    """q_from → q_to 를 frame 의 Cartesian 직선(위치 선형 + slerp)으로 잇는다. 양끝 정확 일치."""
    q_path, diag = solve_transition_path(models.model, models.data, models.frame_id(frame),
                                         q_from, q_to, ik_args, retract=0.0)
    return q_path, diag


def timed(q_path, v_lim, a_lim, control_dt, scale=1.0, min_time=0.0):
    """rest-to-rest 시간 배분. scale 로 한계를 줄이고, min_time(TCP 속도 한계에서 계산)보다 짧으면 늘린다."""
    times, q_t, qd, qdd = time_parameterize(np.asarray(q_path), v_lim * scale, a_lim * scale, control_dt)
    if len(times) > 1 and times[-1] < min_time:
        times, q_t, qd, qdd = _resample(np.asarray(q_path), min_time, control_dt)
    return times, q_t, qd, qdd


def cartesian_length(models, frame, q_path):
    fid = models.frame_id(frame)
    points = np.array([fk_pose(models.model, models.data, fid, q)[0] for q in q_path])
    return float(np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1)))


# =============================================================================
# 빌더
# =============================================================================
class SequenceBuilder:
    def __init__(self, env, cfg, models, part, paint_dir, out_dir, verbose=True):
        self.env = env
        self.cfg = cfg
        self.models = models
        self.part = part
        self.paint_dir = paint_dir
        self.out = Path(out_dir)
        self.verbose = verbose
        lim = cfg.get("limits", {})
        # 갠트리 한계는 gantry_env yaml motor_limits 가 우선 (NLP/stitch 와 같은 값). sequence yaml 값은 env 에 없을 때만.
        gantry_v, gantry_a = env.motor_limits(float(lim.get("gantry_velocity_m_s", 1.0)),
                                              float(lim.get("gantry_acceleration_m_s2", 5.0)))
        args = SimpleNamespace(
            max_joint_velocity_deg_s=float(lim.get("joint_velocity_deg_s", 70.0)),
            max_joint_acceleration_deg_s2=float(lim.get("joint_acceleration_deg_s2", 500.0)),
            max_gantry_velocity_m_s=gantry_v,
            max_gantry_acceleration_m_s2=gantry_a,
        )
        self.prismatic, self.v_lim, self.a_lim = limits_of(models.model, args)
        if verbose:
            print(f"[sequence] 갠트리 한계 v {gantry_v:g} m/s, a {gantry_a:g} m/s^2 "
                  f"({'gantry_env motor_limits' if env.get('motor_limits') else 'sequence yaml limits'})")
        ik = cfg.get("ik", {})
        self.ik_args = SimpleNamespace(
            retract_dir="spray", retract=0.0, step=float(cfg.get("approach", {}).get("step_m", 0.01)),
            ik_max_iter=int(ik.get("max_iter", 200)), ik_damping=float(ik.get("damping", 0.05)),
            ik_max_step_deg=float(ik.get("max_step_deg", 5.0)),
            position_tolerance=float(ik.get("position_tolerance", 0.002)),
            orientation_tolerance_deg=float(ik.get("orientation_tolerance_deg", 2.0)),
            ik_polish_iter=int(ik.get("polish_iter", 40)),
        )
        self.control_dt = float(cfg.get("control_dt", 0.01))
        self.speeds = cfg.get("speeds", {})
        self.lift = float(cfg.get("approach", {}).get("lift_m", 0.15))
        self.collision_stride = int(cfg.get("collision_stride", 5))
        self.shallow_m = float(cfg.get("shallow_penetration_m", 0.008))
        self.dock_zone_slot = None   # 툴을 든 채 슬롯에 붙어 있는 stage 동안만 설정

        home = env.get("home") or {}
        self.q_home = np.zeros(models.model.nq)
        self.q_home[:2] = [float(v) for v in home.get("gantry", [0.0, 0.0])]
        self.q_home[2:] = np.radians([float(v) for v in home.get("arm_deg", [0.0] * 6)])
        self.travel_arm = np.radians([float(v) for v in home.get("travel_arm_deg", [0, 0, 90, 0, 90, 0])])
        self.dock_cfg = dict(env.get("toolchanger.dock") or {})
        self.dock_cfg.update(cfg.get("dock", {}))
        # 툴을 든 채 슬롯을 떠나거나 돌아올 때 수직으로 오르내리는 높이. 툴(건+컵 0.3 m)이 스탠드를 쓸지 않게
        # pre_offset(0.15) 보다 높게 잡는다.
        self.depart_lift = float(self.dock_cfg.get("depart_lift_m", 0.35))
        # 올라간 뒤 스탠드에서 수평으로 물러나는 벡터(월드). 건이 옆 슬롯 플레이트/상단 브래킷을 쓸지 않게 한 뒤 팔을 재배치한다.
        self.depart_retreat = np.array([float(v) for v in self.dock_cfg.get("depart_retreat_xyz", [0.0, 0.3, 0.0])])

        self.q = self.q_home.copy()
        self.tool_attached = False
        self.carried_slot = None
        self.sequence_index = 0
        self.t_cursor = 0.0
        self.records = []           # 매니페스트용
        self.dock_solutions = {}    # slot -> (gantry joints, q_pre, q_dock)
        self.paint_rows = None      # [(path, times, q)]

    # --- 로그 ------------------------------------------------------------
    def log(self, text):
        if self.verbose:
            print(f"[sequence] {text}")

    # --- 기록 --------------------------------------------------------------
    def emit(self, segment, stage, times, q, qd, qdd, tcp_frame, kind, source=None, extra=None):
        """stage CSV 를 쓰고 매니페스트에 기록한다. 경계 연속성은 여기서 self.q 와 비교해 잡는다."""
        q = np.asarray(q, dtype=float)
        jump = np.abs(q[0] - self.q)
        jump_metric = float(np.max(np.where(self.prismatic, jump * 1000.0, np.degrees(jump))))
        name = f"seq_{self.sequence_index:04d}_{segment}_{stage}_joint.csv"
        path = self.out / name
        if source is not None:
            shutil.copyfile(source, path)
        else:
            write_segment(path, times, q, qd, qdd, self.models.names)
        cset = self.models.collision_set(self.tool_attached, self.carried_slot, self.dock_zone_slot)
        hits, depth = check_path_collisions(cset, q, self.collision_stride)
        shallow = bool(hits) and depth <= self.shallow_m
        v_ratio = float(np.max(np.abs(qd) / self.v_lim.reshape(1, -1))) if len(times) > 1 else 0.0
        a_ratio = float(np.max(np.abs(qdd) / self.a_lim.reshape(1, -1))) if len(times) > 1 else 0.0
        lo, hi = self.models.model.lowerPositionLimit, self.models.model.upperPositionLimit
        limit_viol = int(np.sum((q < lo - 1e-9) | (q > hi + 1e-9)))
        duration = float(times[-1] - times[0]) if len(times) else 0.0
        record = {
            "index": self.sequence_index, "segment": segment, "stage": stage, "kind": kind, "file": name,
            "t_start": self.t_cursor, "duration": duration, "t_end": self.t_cursor + duration, "rows": int(len(q)),
            "tcp_frame": tcp_frame, "tool_attached": self.tool_attached,
            "collision_model": "tool" if self.tool_attached else "bare",
            "boundary_joint_jump": jump_metric,      # 직동 mm / 회전 deg 중 최대
            "velocity_ratio": v_ratio, "acceleration_ratio": a_ratio, "joint_limit_violations": limit_viol,
            "collisions": {f"{a}<->{b}": n for (a, b), n in hits.items()}, "max_penetration_m": depth,
            "collision_shallow": shallow, "dock_zone_exclusion": self.dock_zone_slot is not None,
        }
        if extra:
            record.update(extra)
        self.records.append(record)
        flag = ""
        if jump_metric > 1e-3:
            flag += "  <-- 경계 불연속"
        if hits:
            flag += (f"  <-- 경미 간섭 {len(hits)}쌍 (침투 {depth * 1000:.1f} mm)" if shallow
                     else f"  <-- 충돌 {len(hits)}쌍 (침투 {depth * 1000:.0f} mm)")
        if v_ratio > 1.0 or a_ratio > 1.0:
            flag += "  <-- 한계 초과"
        self.log(f"{name:46s} T={duration:7.2f}s rows={len(q):6d} tcp={tcp_frame:6s} tool={int(self.tool_attached)} "
                 f"jump={jump_metric:.4f} v={v_ratio:.2f} a={a_ratio:.2f}{flag}")
        self.sequence_index += 1
        self.t_cursor += duration
        self.q = q[-1].copy()

    def emit_joint_path(self, segment, stage, q_path, tcp_frame, scale=1.0, min_time=0.0, kind="joint"):
        times, q_t, qd, qdd = timed(q_path, self.v_lim, self.a_lim, self.control_dt, scale, min_time)
        self.emit(segment, stage, times, q_t, qd, qdd, tcp_frame, kind)

    def emit_cartesian(self, segment, stage, frame, q_to, tcp_speed, kind="cartesian"):
        q_path, diag = cartesian_line(self.models, frame, self.q, q_to, self.ik_args)
        length = cartesian_length(self.models, frame, q_path)
        min_time = 1.5 * length / max(float(tcp_speed), 1e-6)   # smoothstep 최대 속도 = 1.5 L/T
        times, q_t, qd, qdd = timed(q_path, self.v_lim, self.a_lim, self.control_dt, 1.0, min_time)
        self.emit(segment, stage, times, q_t, qd, qdd, frame, kind,
                  extra={"cartesian_length_m": length, "tcp_speed_limit_m_s": float(tcp_speed),
                         "ik_position_error_m": diag["position_error"],
                         "ik_rotation_error_deg": diag["rotation_error_deg"]})

    def emit_event(self, segment, kind):
        self.dock_zone_slot = self.carried_slot
        dwell = float(self.cfg.get("event_dwell_s", 1.0))
        steps = max(int(round(dwell / self.control_dt)) + 1, 2)
        times = np.linspace(0.0, dwell, steps)
        q = np.repeat(self.q.reshape(1, -1), steps, axis=0)
        zeros = np.zeros_like(q)
        self.emit(segment, "dwell", times, q, zeros, zeros, "link_6", kind)
        self.dock_zone_slot = None

    # --- 도킹 -------------------------------------------------------------
    def dock_solution(self, slot):
        """dock_path 와 같은 방식으로 갠트리 위치·pre-dock·dock 관절해를 고른다 (베어 형상으로 검사)."""
        if slot in self.dock_solutions:
            return self.dock_solutions[slot]
        models = self.models
        dock.TRAVEL_ARM_DEG = tuple(np.degrees(self.travel_arm))
        args = SimpleNamespace(
            pre_offset=float(self.dock_cfg.get("pre_offset", 0.15)),
            standoff=float(self.dock_cfg.get("standoff", 0.005)),
            flip=bool(self.dock_cfg.get("flip", False)),
            dock_gx=self.dock_cfg.get("gantry_x_joint", 0.0),
            grid=float(self.dock_cfg.get("grid", 0.05)), window=float(self.dock_cfg.get("window", 0.75)),
        )
        if args.dock_gx is not None and float(args.dock_gx) < 0:
            args.dock_gx = None
        slot_xyz = models.slot_xyz(slot)
        model, data, geom_model, geom_data = models.collision_set(False)
        fid = models.frame_id(str(self.dock_cfg.get("dock_frame", "link_6")))
        gxs, gys, grid_map, feasible = dock.scan_gantry_grid(model, data, geom_model, geom_data, fid, slot_xyz, args)
        if not feasible:
            raise RuntimeError(f"슬롯 {slot}: 도킹 가능한 갠트리 위치가 없다")
        scored = sorted(((dock.min_clearance(model, data, geom_model, geom_data, c[3]), c) for c in feasible),
                        key=lambda t: -t[0])
        chosen = None
        for clearance, cand in scored[:10]:
            gx, gy, q_pre, q_dock = cand[:4]
            # 접근 경로(접기 → 이송 → 재배치) 를 베어 형상으로 미리 검사해 무충돌 후보를 고른다
            q_travel = self.q_home.copy()
            q_travel[2:] = self.travel_arm
            q_travel_at = q_travel.copy()
            q_travel_at[:2] = [gx, gy]
            path = np.vstack([joint_interp(self.q_home, q_travel, prismatic=self.prismatic),
                              joint_interp(q_travel, q_travel_at, prismatic=self.prismatic),
                              joint_interp(q_travel_at, q_pre, prismatic=self.prismatic)])
            hits, depth = check_path_collisions((model, data, geom_model, geom_data), path, 2)
            deep = {k: v for k, v in hits.items()} if depth > 0.008 else {}
            if not deep:
                chosen = (gx, gy, q_pre, q_dock, clearance, hits, depth)
                break
        if chosen is None:
            raise RuntimeError(f"슬롯 {slot}: 접근 경로가 전부 충돌한다")
        gx, gy, q_pre, q_dock, clearance, hits, depth = chosen
        # 툴을 든 채 오르내릴 높은 지점 (같은 갠트리 위치, 같은 자세, slot + depart_lift)
        rot = dock.dock_rotation(args.flip)
        high = pin.SE3(rot, slot_xyz + np.array([0.0, 0.0, self.depart_lift]))
        dock_frame = str(self.dock_cfg.get("dock_frame", "link_6"))
        q_high = solve_locked_gantry(self.models, dock_frame, high, q_pre)
        away = pin.SE3(rot, high.translation + self.depart_retreat)
        q_away = solve_locked_gantry(self.models, dock_frame, away, q_high)
        self.log(f"슬롯 {slot} @ {np.round(slot_xyz, 4)}: 갠트리 조인트 ({gx:+.4f}, {gy:+.4f}), "
                 f"dock 클리어런스 {clearance * 1000:.0f} mm, 접근 경로 경미 간섭 {len(hits)}쌍 (최대 {depth * 1000:.1f} mm), "
                 f"이탈 높이 {self.depart_lift:.2f} m, 물러남 {self.depart_retreat.tolist()} m")
        self.dock_solutions[slot] = (np.array([gx, gy]), q_pre, q_dock, q_high, q_away)
        return self.dock_solutions[slot]

    def seg_dock_approach(self, seg):
        slot = int(seg.get("slot", self.cfg.get("slot", 1)))
        frame = seg.get("tcp_frame", "link_6")
        gantry, q_pre, q_dock, q_high, q_away = self.dock_solution(slot)
        name = seg["name"]
        scale = float(self.speeds.get("dock_approach_scale", 0.25))
        q_travel = self.q.copy()
        q_travel[2:] = self.travel_arm
        self.emit_joint_path(name, "fold", joint_interp(self.q, q_travel, prismatic=self.prismatic), frame, scale)
        q_travel_at = q_travel.copy()
        q_travel_at[:2] = gantry
        self.emit_joint_path(name, "travel", joint_interp(self.q, q_travel_at, prismatic=self.prismatic), frame,
                             float(self.speeds.get("transit_scale", 1.0)))
        self.emit_joint_path(name, "reposition", joint_interp(self.q, q_pre, prismatic=self.prismatic), frame, scale)
        self.emit_cartesian(name, "descend", frame, q_dock, float(self.speeds.get("dock_descend_tcp_m_s", 0.02)))

    def seg_dock_depart(self, seg):
        slot = int(seg.get("slot", self.carried_slot or self.cfg.get("slot", 1)))
        frame = seg.get("tcp_frame", "link_6")
        gantry, q_pre, q_dock, q_high, q_away = self.dock_solution(slot)
        name = seg["name"]
        ascend_speed = float(self.speeds.get("dock_ascend_tcp_m_s", 0.05))
        if self.tool_attached:
            # 툴을 든 채: 슬롯 바로 위로 높이 올라간 뒤(브래킷 절개부 통과 구간은 도킹 존 제외) 수평으로 물러난다
            self.dock_zone_slot = slot
            self.emit_cartesian(name, "ascend", frame, q_high, ascend_speed)
            self.dock_zone_slot = None
            self.emit_cartesian(name, "retreat", frame, q_away, ascend_speed)
        else:
            self.emit_cartesian(name, "ascend", frame, q_pre, ascend_speed)
        q_travel = self.q.copy()
        q_travel[2:] = self.travel_arm
        self.emit_joint_path(name, "reposition", joint_interp(self.q, q_travel, prismatic=self.prismatic), frame,
                             float(self.speeds.get("dock_approach_scale", 0.25)))

    def seg_dock_return(self, seg):
        slot = int(seg.get("slot", self.carried_slot or self.cfg.get("slot", 1)))
        frame = seg.get("tcp_frame", "link_6")
        gantry, q_pre, q_dock, q_high, q_away = self.dock_solution(slot)
        name = seg["name"]
        scale = float(self.speeds.get("dock_approach_scale", 0.25))
        q_top = q_away if self.tool_attached else q_pre
        if np.linalg.norm(self.q[:2] - gantry) > 1e-9:
            q_travel_at = self.q.copy()
            q_travel_at[:2] = gantry
            self.emit_joint_path(name, "travel", joint_interp(self.q, q_travel_at, prismatic=self.prismatic), frame,
                                 float(self.speeds.get("transit_scale", 1.0)))
        self.emit_joint_path(name, "reposition", joint_interp(self.q, q_top, prismatic=self.prismatic), frame, scale)
        descend_speed = float(self.speeds.get("dock_descend_tcp_m_s", 0.02))
        if self.tool_attached:
            self.emit_cartesian(name, "approach", frame, q_high, float(self.speeds.get("dock_ascend_tcp_m_s", 0.05)))
            self.dock_zone_slot = slot
            self.emit_cartesian(name, "descend", frame, q_dock, descend_speed)
            self.dock_zone_slot = None
        else:
            self.emit_cartesian(name, "descend", frame, q_dock, descend_speed)

    # --- 툴 이벤트 ------------------------------------------------------------
    def seg_tool_attach(self, seg):
        slot = int(seg.get("slot", self.cfg.get("slot", 1)))
        self.tool_attached = True
        self.carried_slot = slot
        self.emit_event(seg["name"], "event")

    def seg_tool_detach(self, seg):
        self.tool_attached = False
        self.carried_slot = None
        self.emit_event(seg["name"], "event")

    # --- 도장 ---------------------------------------------------------------
    def load_paint(self):
        if self.paint_rows is not None:
            return self.paint_rows
        pattern = self.cfg.get("paint", {}).get("pattern", "seq_*_joint.csv")
        files = sorted(Path(self.paint_dir).glob(pattern))
        if not files:
            raise FileNotFoundError(f"{self.paint_dir} 에 {pattern} 이 없다")
        rows = []
        for path in files:
            times, names, q = read_joint_csv(path)
            if list(names) != self.models.names:
                raise ValueError(f"{path.name} 관절 이름 {names} 이 URDF {self.models.names} 와 다르다")
            rows.append((path, times, q))
        self.paint_rows = rows
        return rows

    def paint_start_q(self):
        return self.load_paint()[0][2][0].copy()

    def seg_transit(self, seg):
        """현재 자세(이송 자세) → 목표 위 (관절 공간) → 목표 (Cartesian 하강)."""
        frame = seg.get("tcp_frame", "tcp" if self.tool_attached else "link_6")
        name = seg["name"]
        target = seg.get("target", "paint")
        if target == "paint":
            q_goal = self.paint_start_q()
            q_pre = solve_lifted(self.models, frame, q_goal, self.lift, self.ik_args)
        elif target == "dock":
            slot = int(seg.get("slot", self.carried_slot or self.cfg.get("slot", 1)))
            gantry = self.dock_solution(slot)[0]
            q_goal = None
            q_pre = self.q.copy()
            q_pre[:2] = gantry
        else:
            raise ValueError(f"transit target 은 paint | dock 중 하나: {target!r}")
        q_travel_at = self.q.copy()
        q_travel_at[2:] = self.travel_arm
        q_travel_at[:2] = q_pre[:2]
        self.emit_joint_path(name, "travel", joint_interp(self.q, q_travel_at, prismatic=self.prismatic), frame,
                             float(self.speeds.get("transit_scale", 1.0)))
        if q_goal is None:
            return
        self.emit_joint_path(name, "reposition", joint_interp(self.q, q_pre, prismatic=self.prismatic), frame,
                             float(self.speeds.get("reposition_scale", 0.5)))
        self.emit_cartesian(name, "approach", frame, q_goal, float(self.speeds.get("paint_approach_tcp_m_s", 0.1)))

    def seg_paint(self, seg):
        frame = seg.get("tcp_frame", "tcp")
        name = seg["name"]
        for index, (path, times, q) in enumerate(self.load_paint()):
            stage = path.stem.replace("_joint", "")
            stage = stage.split("_", 2)[-1] if stage.startswith("seq_") else stage   # seq_0001_move000_001 -> move000_001
            zeros = np.zeros_like(q)
            self.emit(name, f"{index:03d}_{stage}", times - times[0], q, zeros, zeros, frame, "paint", source=path)

    def seg_retreat(self, seg):
        frame = seg.get("tcp_frame", "tcp")
        name = seg["name"]
        q_pre = solve_lifted(self.models, frame, self.q, self.lift, self.ik_args)
        self.emit_cartesian(name, "lift", frame, q_pre, float(self.speeds.get("paint_approach_tcp_m_s", 0.1)))
        q_travel = self.q.copy()
        q_travel[2:] = self.travel_arm
        self.emit_joint_path(name, "reposition", joint_interp(self.q, q_travel, prismatic=self.prismatic), frame,
                             float(self.speeds.get("reposition_scale", 0.5)))

    def seg_home(self, seg):
        frame = seg.get("tcp_frame", "link_6")
        name = seg["name"]
        if self.sequence_index == 0:
            times = np.array([0.0])
            q = self.q_home.reshape(1, -1)
            zeros = np.zeros_like(q)
            self.emit(name, "start", times, q, zeros, zeros, frame, "home")
            return
        q_travel = self.q.copy()
        q_travel[2:] = self.travel_arm
        if np.linalg.norm(self.q[2:] - self.travel_arm) > 1e-9:
            self.emit_joint_path(name, "fold", joint_interp(self.q, q_travel, prismatic=self.prismatic), frame,
                                 float(self.speeds.get("reposition_scale", 0.5)))
        q_travel_home = q_travel.copy()
        q_travel_home[:2] = self.q_home[:2]
        self.emit_joint_path(name, "travel", joint_interp(self.q, q_travel_home, prismatic=self.prismatic), frame,
                             float(self.speeds.get("transit_scale", 1.0)))
        self.emit_joint_path(name, "unfold", joint_interp(self.q, self.q_home, prismatic=self.prismatic), frame,
                             float(self.speeds.get("dock_approach_scale", 0.25)))

    HANDLERS = {
        "home": seg_home, "dock_approach": seg_dock_approach, "tool_attach": seg_tool_attach,
        "dock_depart": seg_dock_depart, "transit": seg_transit, "paint": seg_paint, "retreat": seg_retreat,
        "dock_return": seg_dock_return, "tool_detach": seg_tool_detach,
    }

    def build(self):
        if self.out.exists():
            shutil.rmtree(self.out)
        self.out.mkdir(parents=True)
        for seg in self.cfg["segments"]:
            kind = seg["type"]
            if kind not in self.HANDLERS:
                raise ValueError(f"모르는 segment type {kind!r} (가능: {sorted(self.HANDLERS)})")
            seg.setdefault("name", kind)
            self.log(f"--- segment {seg['name']} ({kind})")
            self.HANDLERS[kind](self, seg)
        return self.records


# =============================================================================
# 통합 CSV / 매니페스트 / 검증
# =============================================================================
def resample_stage(t_src, q_src, t_dst):
    """stage 샘플(t_src, q_src)을 t_dst 격자로 다시 샘플한다.

    2026-10-20: 전이/도킹 stage 는 10 ms(control_dt) 인데 통합 CSV 는 1 ms 라, 선형보간(np.interp)으로 올리면 10 ms 마다
    속도가 계단으로 꺾여 통합 CSV 의 갠트리 가속도가 46 m/s^2 (stage 파일 자체는 ≤ 6.8) 로 튀었다(후드 실측). 원본보다
    촘촘한 격자로 갈 때만 C2 cubic spline 으로 보간한다 (양끝 1차 미분 0 = rest-to-rest 유지). 같은 간격(도장 row 1 ms)
    이나 샘플 3개 이하는 종전처럼 선형.
    """
    if len(t_src) >= 4 and (t_dst[1] - t_dst[0]) < 0.5 * (t_src[1] - t_src[0]):
        from scipy.interpolate import CubicSpline
        spline = CubicSpline(t_src, q_src, axis=0, bc_type="clamped")
        return spline(np.clip(t_dst, t_src[0], t_src[-1]))
    return np.column_stack([np.interp(t_dst, t_src, q_src[:, j]) for j in range(q_src.shape[1])])


def write_command_csv(models, out_dir, records, command_dt, frame_name="tcp"):
    """stage CSV 를 command_dt 간격으로 다시 샘플해 하나로 잇는다. TCP 쿼터니언은 반구 연속."""
    fid = models.frame_id(frame_name)
    rows_out = []
    t_offset = 0.0
    last_quat = None
    for rec in records:
        times, names, q = read_joint_csv(out_dir / rec["file"])
        t_local = times - times[0]
        if len(times) == 1 or t_local[-1] <= 0.0:
            grid = np.array([0.0])
            stage_duration = 0.0
        else:
            # stage 길이를 command_dt 의 정수배로 맞춘다(최대 ±dt/2 늘림/줄임, 속도 변화 < 0.1 %).
            # 그래야 통합 CSV 가 끝까지 균일 간격이고 stage 경계에 수 µs 짜리 샘플이 생기지 않는다.
            steps = max(int(round(t_local[-1] / command_dt)), 1)
            stage_duration = steps * command_dt
            grid = np.linspace(0.0, stage_duration, steps + 1)
            t_local = t_local * (stage_duration / t_local[-1])
        q_grid = resample_stage(t_local, q, grid)
        for k, t in enumerate(grid):
            if rows_out and k == 0:
                continue   # 경계 샘플 중복 제거 (이전 stage 마지막 == 이번 첫 샘플)
            p, rot = fk_pose(models.model, models.data, fid, q_grid[k])
            quat = pin.Quaternion(rot).coeffs()   # xyzw
            if last_quat is not None and float(np.dot(quat, last_quat)) < 0.0:
                quat = -quat
            last_quat = quat
            rows_out.append((t_offset + t, rec["index"], rec["segment"], int(rec["tool_attached"]), rec["tcp_frame"],
                             q_grid[k], p, quat))
        t_offset += stage_duration
    times = np.array([r[0] for r in rows_out])
    q_all = np.array([r[5] for r in rows_out])
    if len(times) > 2:
        qd = np.gradient(q_all, times, axis=0, edge_order=1)
        qdd = np.gradient(qd, times, axis=0, edge_order=1)
    else:
        qd = np.zeros_like(q_all)
        qdd = np.zeros_like(q_all)
    arc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(q_all, axis=0), axis=1))])
    path = out_dir / "sequence_command.csv"
    names = models.names
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["t", "s"] + names + [f"qd_{n}" for n in names] + [f"qdd_{n}" for n in names]
                        + ["segment_index", "segment", "tool_attached", "tcp_frame",
                           "tcp_x", "tcp_y", "tcp_z", "tcp_qx", "tcp_qy", "tcp_qz", "tcp_qw"])
        for i, r in enumerate(rows_out):
            writer.writerow([f"{times[i]:.6f}", f"{arc[i]:.9f}"] + [f"{v:.9f}" for v in q_all[i]]
                            + [f"{v:.9f}" for v in qd[i]] + [f"{v:.9f}" for v in qdd[i]]
                            + [r[1], r[2], r[3], r[4]] + [f"{v:.6f}" for v in r[6]] + [f"{v:.8f}" for v in r[7]])
    return path, len(rows_out), float(times[-1]) if len(times) else 0.0


def boundary_tcp_jumps(models, out_dir, records):
    """stage 경계에서 link_6 / tcp 위치·자세 점프 [m, deg]."""
    out = []
    prev_q = None
    for rec in records:
        _t, _n, q = read_joint_csv(out_dir / rec["file"])
        if prev_q is not None:
            worst = {}
            for frame in ("link_6", "tcp"):
                if frame not in models.frame and not models.model.existFrame(frame):
                    continue
                p0, r0 = fk_pose(models.model, models.data, models.frame_id(frame), prev_q)
                p1, r1 = fk_pose(models.model, models.data, models.frame_id(frame), q[0])
                worst[frame] = (float(np.linalg.norm(p1 - p0)), float(np.degrees(np.linalg.norm(pin.log3(r0.T @ r1)))))
            out.append((rec["file"], worst))
        prev_q = q[-1]
    return out


def main(argv=None):
    parser = argparse.ArgumentParser(description="툴체인저 접근 + 도장 경로 통합 시퀀스 생성")
    add_gantry_env_argument(parser, "URDF(툴/베어/충돌)·툴체인저·홈 자세는 여기서 읽는다")
    parser.add_argument("--sequence", default=DEFAULT_SEQUENCE, help="config/sequence/<name>.yaml 또는 경로")
    parser.add_argument("--part", required=True, help="도장 파츠 이름 (stitched 디렉터리 선택)")
    parser.add_argument("--paint-dir", type=Path, default=None,
                        help="stitched row+전이 joint CSV 디렉터리. 기본: yaml paint.dir 템플릿 ({env_suffix}, {part})")
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--slot", type=int, default=None, help="툴체인저 슬롯 (yaml slot 덮어쓰기)")
    parser.add_argument("--package-dir", action="append", default=None,
                        help="package:// 메시 해석용 share 디렉터리 (여러 번 가능). 기본: dsr_description2 + paint_viz 설치본")
    parser.add_argument("--command-dt", type=float, default=None, help="통합 CSV 샘플 간격 (기본 yaml command_dt=0.001)")
    parser.add_argument("--no-command-csv", action="store_true")
    args = parser.parse_args(argv)

    env = load_gantry_env(args.gantry_env)
    cfg = load_sequence_config(args.sequence)
    if args.slot is not None:
        cfg["slot"] = args.slot
    env_suffix = "" if env.name == "legacy" else f"_{env.name}"
    paint_dir = args.paint_dir or Path(expand_project_path(str(cfg.get("paint", {}).get("dir", "")).format(
        env_suffix=env_suffix, env=env.name, part=args.part)))
    if not paint_dir.is_dir():
        raise FileNotFoundError(f"도장 CSV 디렉터리가 없다: {paint_dir}")
    if not env.data.get("toolchanger"):
        raise ValueError(f"gantry_env {env.name} 에는 툴체인저가 없다 (spec_c | x7y4z3 를 쓸 것)")

    print(f"[sequence] gantry_env {env.name} ({env.path})")
    print(f"[sequence] sequence {cfg.get('name', args.sequence)} ({cfg['_path']}), 슬롯 {cfg.get('slot', 1)}")
    print(f"[sequence] 도장 CSV: {paint_dir}")
    models = CellModels(env, args.package_dir or default_package_dirs())
    print(f"[sequence] 운동학 URDF {models.tool_urdf.name}, 충돌: 베어 {models.bare_urdf.name} / 툴 {models.tool_urdf.name}")

    builder = SequenceBuilder(env, cfg, models, args.part, paint_dir, args.out_dir)
    records = builder.build()

    total = builder.t_cursor
    jumps = boundary_tcp_jumps(models, builder.out, records)
    worst_joint_jump = max(r["boundary_joint_jump"] for r in records[1:]) if len(records) > 1 else 0.0
    worst_tcp = max((v["link_6"][0] for _f, v in jumps if "link_6" in v), default=0.0)
    worst_tcp_rot = max((v["link_6"][1] for _f, v in jumps if "link_6" in v), default=0.0)
    collision_stages = [r["file"] for r in records if r["collisions"] and not r["collision_shallow"]]
    shallow_stages = [r["file"] for r in records if r["collisions"] and r["collision_shallow"]]
    limit_stages = [r["file"] for r in records if r["joint_limit_violations"]]
    over_stages = [r["file"] for r in records if r["velocity_ratio"] > 1.0 or r["acceleration_ratio"] > 1.0]

    command = None
    if not args.no_command_csv:
        command_dt = args.command_dt or float(cfg.get("command_dt", 0.001))
        frame = "tcp" if models.model.existFrame("tcp") else "link_6"
        command = write_command_csv(models, builder.out, records, command_dt, frame)
        print(f"[sequence] 통합 명령 CSV {command[0].name}: {command[1]} 행, {command[2]:.2f} s (dt {command_dt})")

    manifest = {
        "gantry_env": env.metadata(), "sequence": {k: v for k, v in cfg.items() if k != "_path"},
        "sequence_file": cfg["_path"], "part": args.part, "paint_dir": str(paint_dir),
        "urdf": {"kinematics": str(models.tool_urdf), "collision_bare": str(models.bare_urdf),
                 "collision_tool": str(models.tool_urdf)},
        "joint_names": models.names, "total_time_s": total,
        "stages": records,
        "boundary_tcp_jumps": [{"file": f, **{k: {"pos_m": v[0], "rot_deg": v[1]} for k, v in d.items()}} for f, d in jumps],
        "verification": {
            "worst_boundary_joint_jump": worst_joint_jump, "worst_boundary_link6_jump_m": worst_tcp,
            "worst_boundary_link6_jump_deg": worst_tcp_rot,
            "stages_with_collisions": collision_stages, "stages_with_shallow_interference": shallow_stages,
            "stages_with_joint_limit_violations": limit_stages,
            "stages_over_speed_limits": over_stages,
        },
        "command_csv": None if command is None else {"file": command[0].name, "rows": command[1], "duration_s": command[2]},
    }
    with (builder.out / "sequence.json").open("w", encoding="utf-8") as stream:
        json.dump(manifest, stream, ensure_ascii=False, indent=2, default=float)

    print(f"[sequence] 완료: stage {len(records)}개, 총 {total:.1f} s -> {builder.out}")
    print(f"[sequence] 검증: 경계 관절 점프 최대 {worst_joint_jump:.5f} (mm|deg), link_6 점프 최대 "
          f"{worst_tcp * 1000:.3f} mm / {worst_tcp_rot:.3f} deg, 충돌 stage {len(collision_stages)} "
          f"(경미 간섭 ≤{cfg.get('shallow_penetration_m', 0.008) * 1000:.0f} mm: {len(shallow_stages)}), "
          f"관절한계 위반 stage {len(limit_stages)}, 한계 초과 stage {len(over_stages)}")
    for f in shallow_stages:
        rec = next(r for r in records if r["file"] == f)
        print(f"[sequence]   경미 간섭 {f}: {rec['collisions']} (침투 {rec['max_penetration_m'] * 1000:.1f} mm)")
    for f in collision_stages:
        rec = next(r for r in records if r["file"] == f)
        print(f"[sequence]   충돌 {f}: {rec['collisions']} (침투 {rec['max_penetration_m'] * 1000:.1f} mm)")
    return 1 if (collision_stages or limit_stages or over_stages) else 0


if __name__ == "__main__":
    raise SystemExit(main())
