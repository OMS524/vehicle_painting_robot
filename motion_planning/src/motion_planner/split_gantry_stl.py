"""model/stl/gantry.stl (2026-09-03 수정본)을 링크별 메시로 쪼개 8축 URDF 를 만든다.

입력 STL 은 갠트리 + A0912 팔(아래로 뻗은 자세)이 합쳐진 단일 바이너리 STL 이고
단위는 **인치**다 (500 mm 받침 플레이트가 19.685 로 나오는 것으로 확정).
STL 에는 부품 경계 정보가 없으므로 삼각형 연결성(connected components)으로
915개 부품을 복원한 뒤, 위치 규칙으로 4개 그룹에 배정한다:

  base     고정 프레임: 받침/기둥(1700 mm H형강)/하부 브레이스/장축 빔 2(3 m)/
           장축 레일 2(3 m)/랙기어/케이블 트레이/제어 캐비닛
  bridge   장축 이송부: 크로스빔(3 m, 단면 230x215)/엔드트럭/구동 모터/횡축 레일 2(2.4 m)
  carriage 횡축 이송부: 슬라이더 4/감속기/모터/로봇 어댑터 플레이트(220 mm)
  arm      A0912 팔 — **버린다**. URDF 의 팔은 dsr_description2 원본 메시로
           이미 관절화돼 있어서, 갠트리 메시에 팔이 박혀 있으면 이중으로 보인다.

좌표계: 기존 paint_cell_8axis URDF 관례(장축 = world X)를 유지하기 위해
메시를 Z 축 -90도 회전한다 (메시 Y = 장축 → world X). 원점은 레일 중심 XY,
바닥(방진 풋 하면) Z=0. 각 그룹 메시는 **자기 링크 프레임**으로 옮겨 저장하므로
URDF 의 visual origin 은 전부 0 이다.

조인트 0 = 브리지/캐리지가 레일 중앙에 온 자세. CAD 모델링 자세는
[report] 로 출력되는 초기값(gantry_x/gantry_y)으로 재현할 수 있다.

사용:
  python split_gantry_stl.py <gantry.stl> <template.urdf> <out_urdf> <mesh_dir>
메시 3개를 mesh_dir 에 쓰고, template(기존 paint_cell_8axis_mesh.urdf)의
갠트리 링크/조인트를 실측 메시판으로 갈아끼운 URDF 를 out_urdf 에 쓴다.
"""

import argparse
import json
import math
import re
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from motion_planner.configuration import repo_root

import numpy as np

IN2M = 0.0254

BASE, BRIDGE, CARRIAGE, ARM = 0, 1, 2, 3
GROUP_NAMES = {BASE: "base", BRIDGE: "bridge", CARRIAGE: "carriage", ARM: "arm"}


# --------------------------------------------------------------------------- #
# STL 입출력
# --------------------------------------------------------------------------- #
def read_binary_stl(path):
    with open(path, "rb") as f:
        f.read(80)
        n = struct.unpack("<I", f.read(4))[0]
        rec = np.frombuffer(
            f.read(n * 50),
            dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("attr", "<u2")]),
        )
    return rec["v"].reshape(-1, 3, 3).astype(np.float64)


def write_binary_stl(path, tris):
    e1 = tris[:, 1] - tris[:, 0]
    e2 = tris[:, 2] - tris[:, 0]
    nrm = np.cross(e1, e2)
    length = np.linalg.norm(nrm, axis=1, keepdims=True)
    nrm = np.where(length > 1e-30, nrm / np.maximum(length, 1e-30), 0.0)
    out = np.zeros(len(tris), dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("attr", "<u2")]))
    out["n"] = nrm.astype(np.float32)
    out["v"] = tris.reshape(-1, 9).astype(np.float32)
    with open(path, "wb") as f:
        f.write(b"paint_cell gantry split" + b" " * 57)
        f.write(struct.pack("<I", len(tris)))
        f.write(out.tobytes())


# --------------------------------------------------------------------------- #
# 연결 요소 분해
# --------------------------------------------------------------------------- #
def connected_components(tris):
    """공유 정점 기준 연결 요소. tri -> comp id 와 comp AABB/삼각형수를 돌려준다."""
    verts = tris.reshape(-1, 3)
    q = np.round(verts * 1e4).astype(np.int64)
    uniq, inv = np.unique(q, axis=0, return_inverse=True)
    faces = inv.reshape(-1, 3)

    parent = np.arange(len(uniq))

    def find(x):
        root = x.copy()
        while True:
            p = parent[root]
            if (p == root).all():
                break
            root = p
        return root

    edges = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [0, 2]]])
    for _ in range(200):
        r0, r1 = find(edges[:, 0]), find(edges[:, 1])
        diff = r0 != r1
        if not diff.any():
            break
        parent[np.maximum(r0[diff], r1[diff])] = np.minimum(r0[diff], r1[diff])

    root = find(np.arange(len(uniq)))
    _, comp_inv = np.unique(root, return_inverse=True)
    tri_comp = comp_inv[faces[:, 0]]

    nc = comp_inv.max() + 1
    vall = uniq.astype(np.float64) / 1e4
    mins = np.full((nc, 3), np.inf)
    maxs = np.full((nc, 3), -np.inf)
    for ax in range(3):
        np.minimum.at(mins[:, ax], comp_inv, vall[:, ax])
        np.maximum.at(maxs[:, ax], comp_inv, vall[:, ax])
    ntris = np.bincount(tri_comp, minlength=nc)
    return tri_comp, mins, maxs, ntris


def classify(mins, maxs):
    """부품 AABB 위치 규칙으로 base/bridge/carriage/arm 라벨을 매긴다 (인치).

    규칙 검증은 색상 투영 렌더로 했다. 핵심 판별 축:
      - 브리지 밴드: 모델링 자세에서 가동부 전체가 메시 Y -62..-20 에 있다.
      - 가동부(브리지/캐리지)는 전부 z_min > -5. 밴드에 걸리는 +Y 쪽 기둥·받침·
        하부 브레이스·제어 캐비닛은 이 규칙으로 base 로 돌아간다.
      - 캐리지와 팔은 메시 X 25..50 중앙 기둥에 있고, 어댑터 플레이트 하면
        (z=5.71)이 경계다. z 중심 4 를 문턱으로 쓰면 팔 베이스(중심 3.5)와
        어댑터(중심 6.1)가 갈린다.
    """
    ctr = (mins + maxs) / 2
    size = maxs - mins
    nc = len(mins)
    label = np.full(nc, BASE)

    in_band = (ctr[:, 1] > -62) & (ctr[:, 1] < -20)
    long_x = size[:, 0] > 60
    long_y = size[:, 1] > 60
    center_x = (ctr[:, 0] > 25) & (ctr[:, 0] < 50)

    label[in_band & ~long_y] = BRIDGE
    label[in_band & center_x & (ctr[:, 2] > 4) & ~long_x] = CARRIAGE
    label[in_band & center_x & (ctr[:, 2] <= 4) & ~long_x] = ARM
    label[long_y] = BASE
    label[(label != ARM) & (mins[:, 2] < -5.0)] = BASE
    # 우측 빔 하면의 800 mm 트레이 2개(고정 커버) — 밴드에 걸리지만 base 다.
    tray = in_band & (ctr[:, 0] > 95) & (ctr[:, 2] < 0) & (size[:, 1] > 25)
    label[tray] = BASE
    return label


# --------------------------------------------------------------------------- #
# 실측
# --------------------------------------------------------------------------- #
def measure(tris, tri_comp, mins, maxs, ntris, label):
    """메시(인치)에서 URDF 에 필요한 기준 프레임/스트로크/장착 프레임을 잰다."""
    ctr = (mins + maxs) / 2
    size = maxs - mins
    m = {}

    # 장축(메시 Y) 레일 2개: base 그룹에서 y 로 100in 이상 길고 단면이 1in 급.
    rail_sel = np.where(
        (label == BASE) & (size[:, 1] > 100) & (size[:, 0] < 2) & (size[:, 2] < 2)
    )[0]
    assert len(rail_sel) == 2, f"장축 레일이 2개가 아니다: {rail_sel}"
    m["origin_x"] = float(ctr[rail_sel, 0].mean())          # 레일 쌍의 중앙
    m["origin_y"] = float(ctr[rail_sel, 1].mean())
    m["long_rail_span"] = (float(mins[rail_sel, 1].min()), float(maxs[rail_sel, 1].max()))
    m["long_rail_x"] = sorted(float(x) for x in ctr[rail_sel, 0])

    # 바닥 = base 그룹 최저점 (방진 레벨링 풋 하면)
    m["floor_z"] = float(mins[label == BASE, 2].min())

    # 횡축(메시 X) 레일 2개: bridge 그룹, x 로 90in 급.
    xr = np.where((label == BRIDGE) & (size[:, 0] > 90)
                  & (size[:, 1] < 1.5) & (size[:, 2] < 1.5))[0]
    assert len(xr) == 2, f"횡축 레일이 2개가 아니다: {xr}"
    m["cross_rail_span"] = (float(mins[xr, 0].min()), float(maxs[xr, 0].max()))

    # 브리지 크로스빔: bridge 그룹에서 가장 긴 x 부품
    beam = np.where(label == BRIDGE)[0][np.argmax(size[label == BRIDGE, 0])]
    m["bridge_y"] = float(ctr[beam, 1])                     # 모델링 자세의 빔 중심
    m["bridge_z"] = float(ctr[beam, 2])                     # 링크 원점 높이

    # 브리지 스트로크: 장축 레일 근방(x ±3in)에 걸리고 레일 높이까지 내려오는
    # 브리지 부품(엔드트럭/피니언/모터)의 y 외곽 <-> 레일 끝.
    trucks = []
    for rx in m["long_rail_x"]:
        near = (label == BRIDGE) & (mins[:, 0] < rx + 3) & (maxs[:, 0] > rx - 3) \
            & (mins[:, 2] < 8.5)
        trucks.append(np.where(near)[0])
    truck_idx = np.unique(np.concatenate(trucks))
    assert len(truck_idx) > 0, "엔드트럭을 찾지 못했다"
    m["truck_span"] = (float(mins[truck_idx, 1].min()), float(maxs[truck_idx, 1].max()))

    # 캐리지 스트로크: 횡축 레일을 감싸는 슬라이더 블록(3.27 x 1.20 x 2.76)의
    # x 외곽 <-> 레일 끝. 레일과 같은 y/z 에 있는 것만 — 케이블체인 제외.
    rail_y = float(ctr[xr, 1].mean())
    slider = np.zeros(len(mins), dtype=bool)
    for rz in (float(ctr[i, 2]) for i in xr):
        slider |= (
            (label == CARRIAGE) & (np.abs(ctr[:, 2] - rz) < 1.0)
            & (np.abs(ctr[:, 1] - rail_y) < 2.0)
            & (size[:, 0] > 2) & (size[:, 0] < 5)
        )
    slider_idx = np.where(slider)[0]
    assert len(slider_idx) >= 2, "캐리지 슬라이더를 찾지 못했다"
    m["slider_span"] = (float(mins[slider_idx, 0].min()), float(maxs[slider_idx, 0].max()))
    m["carriage_x"] = float((m["slider_span"][0] + m["slider_span"][1]) / 2)

    # 로봇 어댑터 플레이트(220 mm 원판): carriage 그룹, xy 8~9in 원, 얇음.
    plate = np.where(
        (label == CARRIAGE) & (np.abs(size[:, 0] - 8.66) < 0.5)
        & (np.abs(size[:, 1] - 8.66) < 0.5) & (size[:, 2] < 1.5)
    )[0]
    assert len(plate) == 1, f"어댑터 플레이트가 1개가 아니다: {plate}"
    m["mount_xy"] = (float(ctr[plate[0], 0]), float(ctr[plate[0], 1]))
    m["mount_z"] = float(mins[plate[0], 2])                 # 플레이트 하면 = 팔 플랜지 상면

    # 팔 베이스 방향(yaw): 최상단(z max) 팔 부품 = 베이스 몸통. 그 AABB 중심이
    # 플랜지 중심에서 커넥터 쪽으로 치우친 것을 이용한다.
    arm_idx = np.where(label == ARM)[0]
    base_body = arm_idx[np.argmax(maxs[arm_idx, 2])]
    m["arm_base_bbox_ctr"] = (float(ctr[base_body, 0]), float(ctr[base_body, 1]))
    m["arm_base_size"] = tuple(float(s) for s in size[base_body])
    return m


def world_of(pts_in, m):
    """메시(인치) -> world(미터): 원점 이동 후 Rz(-90도), 인치->미터."""
    p = pts_in - np.array([m["origin_x"], m["origin_y"], m["floor_z"]])
    out = np.empty_like(p)
    out[..., 0] = p[..., 1]
    out[..., 1] = -p[..., 0]
    out[..., 2] = p[..., 2]
    return out * IN2M


def estimate_arm_yaw(m, dae_path):
    """dsr_description2 의 base_link 메시와 커넥터 돌출 방향을 맞춰 yaw 를 정한다.

    갠트리 메시의 팔은 뒤집혀(roll=pi) 달려 있으므로,
      world_offset ≈ Rz(yaw) @ Rx(pi) @ dae_offset
    을 만족하는 yaw 를 구하고 15도 격자로 반올림한다.
    """
    text = Path(dae_path).read_text()
    arrays = re.findall(r"<float_array[^>]*>([^<]+)</float_array>", text)
    pts = []
    for a in arrays:
        v = np.fromstring(a, sep=" ")
        if len(v) % 3 == 0 and len(v) >= 9:
            pts.append(v.reshape(-1, 3))
    dae = np.concatenate(pts) * 1e-3  # mm -> m (URDF 에서 scale 0.001 로 쓰는 메시)
    # 법선 배열이 섞여 있을 수 있어 |v|<=1.5 정육면체 안 좌표만 남기면 안 되고,
    # 위치 배열은 크기가 0.1~0.2 m 라 법선(단위구)과 겹친다. 대신 z 범위로 거른다:
    # base_link 메시는 z 0..0.22 m, 법선은 z -1..1 에 고르게 퍼져 있다.
    dae = dae[(dae[:, 2] > -1e-6) & (dae[:, 2] < 0.35)]
    dae_ctr = (dae.min(axis=0) + dae.max(axis=0)) / 2
    dae_size = dae.max(axis=0) - dae.min(axis=0)

    mesh_off_in = np.array(m["arm_base_bbox_ctr"]) - np.array(m["mount_xy"])
    world_off = np.array([mesh_off_in[1], -mesh_off_in[0]]) * IN2M  # Rz(-90)
    flip = np.array([dae_ctr[0], -dae_ctr[1]])                      # Rx(pi)

    report = {
        "dae_bbox_size": tuple(float(s) for s in dae_size),
        "mesh_base_size_m": tuple(s * IN2M for s in m["arm_base_size"][:2]),
        "world_offset": tuple(float(v) for v in world_off),
        "dae_offset_flipped": tuple(float(v) for v in flip),
    }
    if np.linalg.norm(world_off) < 0.003 or np.linalg.norm(flip) < 0.003:
        return None, report
    yaw = math.atan2(world_off[1], world_off[0]) - math.atan2(flip[1], flip[0])
    yaw = (yaw + math.pi) % (2 * math.pi) - math.pi
    snapped = round(yaw / (math.pi / 12)) * (math.pi / 12)
    report["yaw_raw_deg"] = math.degrees(yaw)
    return snapped, report


# --------------------------------------------------------------------------- #
# URDF 생성
# --------------------------------------------------------------------------- #
def _origin(parent, xyz=(0, 0, 0), rpy=(0, 0, 0)):
    o = ET.SubElement(parent, "origin")
    o.set("xyz", " ".join(f"{v:.6f}" for v in xyz))
    o.set("rpy", " ".join(f"{v:.6f}" for v in rpy))


def _mesh_link(root, name, mesh_uri, material, collision_boxes, inertial=True):
    link = ET.SubElement(root, "link")
    link.set("name", name)
    if inertial:
        inert = ET.SubElement(link, "inertial")
        ET.SubElement(inert, "mass").set("value", "1.0")
        _origin(inert)
        ine = ET.SubElement(inert, "inertia")
        for k, v in (("ixx", "0.01"), ("ixy", "0"), ("ixz", "0"),
                     ("iyy", "0.01"), ("iyz", "0"), ("izz", "0.01")):
            ine.set(k, v)
    vis = ET.SubElement(link, "visual")
    _origin(vis)
    geo = ET.SubElement(vis, "geometry")
    ET.SubElement(geo, "mesh").set("filename", mesh_uri)
    ET.SubElement(vis, "material").set("name", material)
    # 충돌은 1.1M 삼각형 메시 대신 주요 부품 AABB 박스 (hpp-fcl 부하 대책,
    # model/README.md 의 방침 그대로)
    for bmin, bmax in collision_boxes:
        col = ET.SubElement(link, "collision")
        c = (np.array(bmin) + np.array(bmax)) / 2
        s = np.array(bmax) - np.array(bmin)
        _origin(col, tuple(c))
        geo = ET.SubElement(col, "geometry")
        ET.SubElement(geo, "box").set("size", " ".join(f"{v:.6f}" for v in s))
    return link


def collision_boxes_for(group_tris_link, tri_comp_group, min_extent_m=0.25):
    """링크 프레임 삼각형에서 부품별 AABB 를 다시 계산해 큰 부품만 박스로 쓴다."""
    boxes = []
    for cid in np.unique(tri_comp_group):
        pts = group_tris_link[tri_comp_group == cid].reshape(-1, 3)
        bmin, bmax = pts.min(axis=0), pts.max(axis=0)
        if (bmax - bmin).max() >= min_extent_m:
            boxes.append((bmin, bmax))
    return boxes


def build_urdf(template, out_path, mesh_dir_uri, meas, yaw, limits, mount_in_carriage,
               bridge_z_w):
    tree = ET.parse(template)
    root = tree.getroot()

    # 기존 갠트리 링크/조인트 제거 (팔/툴은 그대로 둔다)
    for tag, names in (("link", {"world_base", "gantry_x_link", "gantry_y_link"}),
                       ("joint", {"gantry_x", "gantry_y", "gantry_to_arm"})):
        for el in list(root.findall(tag)):
            if el.get("name") in names:
                root.remove(el)

    _mesh_link(root, "world_base", f"{mesh_dir_uri}/gantry_base.stl", "cell_frame",
               limits["base_boxes"], inertial=False)
    _mesh_link(root, "gantry_x_link", f"{mesh_dir_uri}/gantry_bridge.stl", "cell_frame",
               limits["bridge_boxes"])
    _mesh_link(root, "gantry_y_link", f"{mesh_dir_uri}/gantry_carriage.stl", "cell_frame",
               limits["carriage_boxes"])

    jx = ET.SubElement(root, "joint")
    jx.set("name", "gantry_x")
    jx.set("type", "prismatic")
    ET.SubElement(jx, "parent").set("link", "world_base")
    ET.SubElement(jx, "child").set("link", "gantry_x_link")
    _origin(jx, (0, 0, bridge_z_w))
    ET.SubElement(jx, "axis").set("xyz", "1 0 0")
    lim = ET.SubElement(jx, "limit")
    lim.set("effort", "2000")
    lim.set("lower", f"{limits['x'][0]:.6f}")
    lim.set("upper", f"{limits['x'][1]:.6f}")
    lim.set("velocity", "1.5")

    jy = ET.SubElement(root, "joint")
    jy.set("name", "gantry_y")
    jy.set("type", "prismatic")
    ET.SubElement(jy, "parent").set("link", "gantry_x_link")
    ET.SubElement(jy, "child").set("link", "gantry_y_link")
    _origin(jy, (0, 0, 0))
    ET.SubElement(jy, "axis").set("xyz", "0 1 0")
    lim = ET.SubElement(jy, "limit")
    lim.set("effort", "2000")
    lim.set("lower", f"{limits['y'][0]:.6f}")
    lim.set("upper", f"{limits['y'][1]:.6f}")
    lim.set("velocity", "1.5")

    ja = ET.SubElement(root, "joint")
    ja.set("name", "gantry_to_arm")
    ja.set("type", "fixed")
    ET.SubElement(ja, "parent").set("link", "gantry_y_link")
    ET.SubElement(ja, "child").set("link", "base_link")
    _origin(ja, mount_in_carriage, (math.pi, 0.0, yaw))

    tree.write(out_path, encoding="utf-8", xml_declaration=True)


# --------------------------------------------------------------------------- #
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("stl", type=Path)
    ap.add_argument("template", type=Path, help="갠트리를 갈아끼울 기존 8축 URDF")
    ap.add_argument("out_urdf", type=Path)
    ap.add_argument("mesh_dir", type=Path)
    ap.add_argument("--dae", type=Path,
                    default=repo_root() / "description/paint_cell/a0912_meshes"
                    / "a0912_white/A0912_0_0.dae",
                    help="팔 yaw 추정용 base_link 메시")
    args = ap.parse_args(argv)

    tris = read_binary_stl(args.stl)
    print(f"[split] {args.stl.name}: {len(tris)} tris")
    tri_comp, mins, maxs, ntris = connected_components(tris)
    label = classify(mins, maxs)
    for g, name in GROUP_NAMES.items():
        sel = label == g
        print(f"[split] {name:8s}: parts={sel.sum():4d} tris={int(ntris[sel].sum()):7d}")

    meas = measure(tris, tri_comp, mins, maxs, ntris, label)

    yaw, yaw_report = estimate_arm_yaw(meas, args.dae)
    print(f"[yaw] {yaw_report}")
    if yaw is None:
        print("[yaw] 오프셋이 작아 추정 불가 — 이전 CAD 관례 pi/2 를 쓴다", file=sys.stderr)
        yaw = math.pi / 2

    # world 기준값 (미터)
    def w(x_in, y_in, z_in):
        return world_of(np.array([x_in, y_in, z_in]), meas)

    bridge_z_w = float(w(0, 0, meas["bridge_z"])[2])
    mount_w = w(meas["mount_xy"][0], meas["mount_xy"][1], meas["mount_z"])

    # 조인트 0 = 슬라이더군(엔드트럭/캐리지 슬라이더)이 레일 정중앙에 오는
    # 기계적 중립 자세. 스트로크는 (레일 길이 - 슬라이더군 길이)/2 로 대칭이다.
    # 모델링 자세 오프셋(bridge_x_w 등)은 메시를 링크 프레임으로 옮길 때와
    # CAD 자세 재현용 초기값 보고에 쓴다.
    t0, t1 = meas["truck_span"]                                 # 메시 y (장축)
    r0, r1 = meas["long_rail_span"]
    bridge_x_w = float(w(0, (t0 + t1) / 2, 0)[0])
    half_x = ((r1 - r0) - (t1 - t0)) / 2 * IN2M

    s0, s1 = meas["slider_span"]                                # 메시 x (횡축)
    c0, c1 = meas["cross_rail_span"]
    carriage_y_w = float(w((s0 + s1) / 2, 0, 0)[1])
    half_y = ((c1 - c0) - (s1 - s0)) / 2 * IN2M

    limits = {"x": (-half_x, half_x), "y": (-half_y, half_y)}

    # 메시를 링크 프레임으로 옮기며 내보낸다
    tris_w = world_of(tris, meas)
    args.mesh_dir.mkdir(parents=True, exist_ok=True)
    offsets = {
        BASE: np.zeros(3),
        BRIDGE: np.array([bridge_x_w, 0.0, bridge_z_w]),
        CARRIAGE: np.array([bridge_x_w, carriage_y_w, bridge_z_w]),
    }
    files = {BASE: "gantry_base.stl", BRIDGE: "gantry_bridge.stl",
             CARRIAGE: "gantry_carriage.stl"}
    for g, fname in files.items():
        sel = label[tri_comp] == g
        local = tris_w[sel] - offsets[g]
        write_binary_stl(args.mesh_dir / fname, local)
        boxes = collision_boxes_for(local, tri_comp[sel])
        limits[f"{GROUP_NAMES[g]}_boxes"] = boxes
        print(f"[mesh] {fname}: {sel.sum()} tris, collision boxes={len(boxes)}")

    mount_in_carriage = tuple(float(v) for v in (mount_w - offsets[CARRIAGE]))

    build_urdf(args.template, args.out_urdf, f"file://{args.mesh_dir.resolve()}",
               meas, yaw, limits, mount_in_carriage, bridge_z_w)

    # 실측 상수 기록 (검증/추적용, 인치 원시값 + 미터 결과값)
    dump = {
        "measured_in": {k: v for k, v in meas.items()},
        "bridge_z_w": bridge_z_w, "bridge_x_w_cad": bridge_x_w,
        "carriage_y_w_cad": carriage_y_w,
        "mount_in_carriage": list(mount_in_carriage),
        "mount_world_z": float(mount_w[2]),
        "arm_yaw": yaw, "half_x": half_x, "half_y": half_y,
    }
    (args.mesh_dir / "gantry_split_report.json").write_text(
        json.dumps(dump, indent=2, ensure_ascii=False))

    print(f"[report] 레일 높이/브리지 중심 z = {bridge_z_w:.3f} m")
    print(f"[report] 팔 장착면 높이 = {float(mount_w[2]):.3f} m, yaw = {math.degrees(yaw):.0f} deg")
    print(f"[report] 장착점(캐리지 프레임) = {mount_in_carriage}")
    print(f"[report] 스트로크: X ±{half_x:.3f} m, Y ±{half_y:.3f} m")
    print(f"[report] CAD 모델링 자세 재현: gantry_x={bridge_x_w:+.4f}, "
          f"gantry_y={carriage_y_w:+.4f}")
    print(f"[report] URDF: {args.out_urdf}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
