"""생성된 관절 궤적이 무언가와 부딪히는지 검사한다.

무엇을 보는가
  1) 자기 충돌 + 갠트리 프레임 간섭
     URDF 의 collision 형상으로 pinocchio/coal 충돌쌍 검사. 팔은 원본 A0912
     충돌 메시(a0912_collision), 프레임은 make_gantry_urdf 가 넣은 기둥/레일/빔/
     캐리지 박스다. **인접 링크 쌍은 제외**한다(설계상 맞닿아 있어 항상 충돌로
     잡힌다). 예: 캐리지-장착플레이트-base_link 는 같은 관절에 붙어 있다.

  2) 부품 간섭
     부품은 점군(PLY)이라 메시가 없다. 그래서 로봇 충돌 메시의 정점을 월드로
     옮겨 부품 점군과의 최근접 거리를 KD-tree 로 잰다. 점군 해상도만큼의
     오차가 있으므로 "여유(clearance)" 지표로 본다.

  3) 바닥 간섭
     로봇 충돌 정점의 최저 z.

왜 필요한가
  row 사이 전이 궤적(stitch_transitions)이 새로 생겼는데, 이 구간은 표면을
  따라가지 않고 들어올렸다 옮기는 경로다. 부품이나 프레임을 스치는지 확인해야
  "계획 경로를 로봇이 따라간다"를 시뮬레이션으로 주장할 수 있다.
"""

import argparse
from pathlib import Path

import numpy as np
import pinocchio as pin
from scipy.spatial import cKDTree

from motion_planner.configuration import add_gantry_env_argument, apply_gantry_env, default_package_dirs
from motion_planner.ply_io import read_surface_points
from motion_planner.scene_config import load_scene_config, part_transform
from motion_planner.data_io import read_joint_csv

DEFAULT_PACKAGE_DIRS = default_package_dirs()   # dsr_description2 + paint_viz share 의 부모 (ament index 로 해석)


def build_collision_pairs(model, geom_model):
    """인접하지 않은 형상 쌍만 **직접 추가**한다. 추가한 개수를 돌려준다.

    addAllCollisionPairs() 후 removeCollisionPair() 로 빼는 방식은 실제로 일부
    쌍이 남았다(레일-빔, 빔-캐리지가 전 샘플에서 충돌로 잡혔다). 그래서 처음부터
    허용 쌍만 넣는다 — 결과가 결정적이고 디버깅도 쉽다.

    제외 기준: 같은 관절에 붙은 형상, 그리고 부모-자식 관절에 붙은 형상.
    레일-빔, 빔-캐리지, 캐리지-base_link 처럼 설계상 맞닿는 구조물이 여기 해당한다.
    """
    geom_model.removeAllCollisionPairs()
    for first in range(geom_model.ngeoms):
        joint_first = geom_model.geometryObjects[first].parentJoint
        for second in range(first + 1, geom_model.ngeoms):
            joint_second = geom_model.geometryObjects[second].parentJoint
            if joint_first == joint_second:
                continue
            if model.parents[joint_first] == joint_second:
                continue
            if model.parents[joint_second] == joint_first:
                continue
            geom_model.addCollisionPair(pin.CollisionPair(first, second))
    return len(geom_model.collisionPairs)


def mesh_sample_points(geom_model, max_per_geom):
    """메시 형상마다 정점을 균등 추출한다. [(geom_index, 로컬점 Nx3), ...]"""
    samples = []
    for index, geom in enumerate(geom_model.geometryObjects):
        geometry = geom.geometry
        if not hasattr(geometry, "vertices"):
            continue  # Box/Cylinder(프레임)는 부품 여유 검사에서 제외 — 정적이라 따로 본다
        # coal 에서 vertices 는 **메서드**다 (속성처럼 쓰면 method 객체가 잡힌다).
        raw = geometry.vertices
        vertices = np.asarray(raw() if callable(raw) else raw, dtype=float).reshape(-1, 3)
        if len(vertices) > max_per_geom:
            step = int(np.ceil(len(vertices) / max_per_geom))
            vertices = vertices[::step]
        samples.append((index, vertices))
    return samples


def load_sequence(directory, pattern, expected_nq):
    files = sorted(Path(directory).glob(pattern))
    if not files:
        raise FileNotFoundError(f"{directory} 에 {pattern} 이 없다")
    segments = []
    for path in files:
        times, names, q = read_joint_csv(path)
        if q.shape[1] != expected_nq:
            raise ValueError(f"{path.name} 관절 수 {q.shape[1]} != URDF nq {expected_nq}")
        segments.append((path.name, times, q))
    return segments


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="관절 궤적의 충돌/간섭 검사")
    parser.add_argument("--joint-csv-dir", required=True, type=Path)
    parser.add_argument("--pattern", default="*_joint.csv")
    parser.add_argument("--urdf", type=Path, default=None,
                        help="collision 형상이 있는 URDF. 생략하면 --gantry-env 의 urdf.collision")
    add_gantry_env_argument(parser)
    parser.add_argument("--scene", type=Path, help="부품 파일/배치. 주면 부품 간섭도 검사")
    parser.add_argument("--package-dir", action="append", default=None,
                        help="package:// 해결용 share 디렉터리 (여러 번 가능). 기본: dsr_description2 + paint_viz 설치본")
    parser.add_argument("--stride", type=int, default=1, help="궤적 샘플 간격")
    parser.add_argument("--part-stride", type=int, default=3,
                        help="부품 간섭 검사는 이 간격으로 (느려서 따로 둔다)")
    parser.add_argument("--max-points-per-geom", type=int, default=60,
                        help="부품 여유 계산에 쓸 형상별 정점 수")
    parser.add_argument("--clearance-warn", type=float, default=0.02,
                        help="부품 여유가 이 값보다 작으면 경고 [m]")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    env = apply_gantry_env(args, "collision")

    model = pin.buildModelFromUrdf(str(args.urdf))
    data = model.createData()
    geom_model = pin.buildGeomFromUrdf(
        model, str(args.urdf), pin.GeometryType.COLLISION, package_dirs=args.package_dir or DEFAULT_PACKAGE_DIRS
    )
    pair_count = build_collision_pairs(model, geom_model)
    geom_data = pin.GeometryData(geom_model)

    segments = load_sequence(args.joint_csv_dir, args.pattern, model.nq)
    total_samples = sum(len(q[::args.stride]) for _n, _t, q in segments)
    print(f"[collision] URDF: {args.urdf}" + (f" (gantry_env {env.name})" if env else ""))
    print(f"[collision] 충돌 형상 {geom_model.ngeoms}개, 검사 쌍 {pair_count}개 (인접 쌍 제외)")
    print(f"[collision] 세그먼트 {len(segments)}개, 검사 샘플 {total_samples}개 (stride={args.stride})")

    # --- 1) 자기 충돌 + 프레임 간섭 -------------------------------------
    hits = {}
    checked = 0
    for name, _times, q_all in segments:
        for q in q_all[:: args.stride]:
            checked += 1
            if not pin.computeCollisions(model, data, geom_model, geom_data, q, False):
                continue
            for index, result in enumerate(geom_data.collisionResults):
                if not result.isCollision():
                    continue
                pair = geom_model.collisionPairs[index]
                key = (
                    geom_model.geometryObjects[pair.first].name,
                    geom_model.geometryObjects[pair.second].name,
                )
                entry = hits.setdefault(key, {"count": 0, "segments": set()})
                entry["count"] += 1
                entry["segments"].add(name)

    print(f"\n[collision] 1) 자기 충돌 / 프레임 간섭")
    if not hits:
        print(f"[collision]    충돌 없음 ({checked}개 샘플 전부 통과)")
    else:
        for (first, second), entry in sorted(hits.items(), key=lambda kv: -kv[1]["count"]):
            print(f"[collision]    {first} <-> {second}: {entry['count']}회, "
                  f"세그먼트 {len(entry['segments'])}개")

    # --- 2) 부품 간섭 + 3) 바닥 -----------------------------------------
    part_tree = None
    if args.scene:
        scene = load_scene_config(args.scene)
        rotation, translation = part_transform(scene)
        points = np.asarray(read_surface_points(str(scene["part_file"]))[0], dtype=float)
        world_points = (rotation @ points.T).T + translation
        part_tree = cKDTree(world_points)
        print(f"\n[collision] 2) 부품 간섭 — 점군 {len(world_points)}개 "
              f"(z {world_points[:, 2].min():.3f}~{world_points[:, 2].max():.3f} m)")

    samples = mesh_sample_points(geom_model, args.max_points_per_geom)
    sample_count = sum(len(v) for _i, v in samples)
    min_clearance = np.inf
    min_clearance_where = ""
    min_floor_z = np.inf
    min_floor_where = ""

    for name, _times, q_all in segments:
        for q in q_all[:: args.part_stride]:
            pin.forwardKinematics(model, data, q)
            pin.updateGeometryPlacements(model, data, geom_model, geom_data, q)
            world = []
            for index, vertices in samples:
                placement = geom_data.oMg[index]
                world.append((placement.rotation @ vertices.T).T + placement.translation)
            world = np.vstack(world)

            floor_z = float(world[:, 2].min())
            if floor_z < min_floor_z:
                min_floor_z, min_floor_where = floor_z, name

            if part_tree is not None:
                distance, _ = part_tree.query(world, k=1)
                clearance = float(distance.min())
                if clearance < min_clearance:
                    min_clearance, min_clearance_where = clearance, name

    if part_tree is not None:
        flag = "  <-- 경고" if min_clearance < args.clearance_warn else ""
        print(f"[collision]    로봇 정점 {sample_count}개 기준 최소 여유 "
              f"{min_clearance * 1000:.1f} mm  [{min_clearance_where}]{flag}")
        print(f"[collision]    (점군 해상도만큼의 오차가 있는 근사값이다)")

    print(f"\n[collision] 3) 바닥 간섭 — 로봇 최저점 z = {min_floor_z:.3f} m "
          f"[{min_floor_where}]" + ("  <-- 바닥 아래!" if min_floor_z < 0.0 else ""))

    failed = bool(hits) or min_floor_z < 0.0 or (
        part_tree is not None and min_clearance < args.clearance_warn
    )
    print(f"\n[collision] 결과: {'문제 있음' if failed else '이상 없음'}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
