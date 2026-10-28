"""A0912 팔을 XY 갠트리에 **천장 장착(아래로 뻗는 자세)** 한 8축 도장셀 URDF를 만든다.

왜 이 구조인가
  - ES300h_hood 경로는 XY 평면으로 1.57 x 1.66 m 를 덮는데 A0912 도달 반경은
    약 1.40 m 다. 베이스를 고정하면 한 자세로 후드 전체를 칠할 수 없어
    갠트리 직동 2축을 얹어 8축(직동 2 + 회전 6)으로 만든다.
  - planner 경로의 분사축은 부품 좌표계 -Z(아래) 다. 건이 부품 위에서 아래로
    쏘는 경로이므로 팔은 갠트리에서 **아래로 내려오는** 배치여야 한다.
    그래서 `gantry_to_arm` 고정 조인트에 roll = pi 를 주어 팔 base_link 의
    +Z(팔이 뻗는 방향)가 월드 -Z 를 향하게 한다.

좌표계 / 링크 구조
  world_base (바닥 z=0, 프레임 중심이 원점. 고정: 기둥 4 + X 빔 2 + X 레일 2)
    └ gantry_x  (prismatic, X, ±x_travel/2)  → gantry_x_link  (Y 방향 이송 브리지)
        └ gantry_y  (prismatic, Y, ±y_travel/2) → gantry_y_link (캐리지 + 장착 플레이트)
            └ gantry_to_arm (fixed, rpy = pi 0 pi/2) → base_link → link_1..link_6

치수 출처 — model/gantry.stp (2026-07-13, Solid Edge)
  아래 상수는 전부 실 CAD 어셈블리에서 뽑은 실측값이다. 예전 가정값
  (스트로크 3.0 x 3.0 m, 장착 높이 2.2 m, 원통 레일)은 실물과 맞지 않았다.
  특히 Y 레일은 2.8 m 뿐이라 3.0 m 스트로크는 물리적으로 불가능했다.

  기준면(datum)
    - 바닥 z=0 = 방진 레벨링 풋 하면. CAD 상 좌측 열 -2316 mm, 우측 열 -2312 mm 로
      4 mm 어긋나 있어(어셈블리 모델링 오차) 평균 -2314 mm 를 바닥으로 잡았다.
      따라서 아래 높이값들은 ±2 mm 불확실성이 있다.
    - 프레임 중심 = X 레일 중심(CAD X +171.434), Y 중심(CAD Y 0).
      이 원점에서 기둥은 X ±1744.0 / Y ±1509.5 로 정확히 대칭이라 datum 이 맞다.

  브리지 중심(CAD Z=0)을 높이 기준으로 삼는다. 모든 고정 프레임 요소는
  브리지 중심에서 아래로 몇 mm 인지로 정의되므로 --mount-z 를 바꾸면
  기둥 길이가 따라 늘어난다(실제로 기둥을 잘라 맞추는 것과 같다).

주의
  팔 부분의 운동학은 원본 a0912 URDF 그대로다. 스트로크는 슬라이더 블록이
  레일 끝에 닿는 **기하학적 한계**이고 엔드스톱/케이블베어 여유는 반영돼 있지 않다.
"""

import argparse
import copy
import math
import xml.etree.ElementTree as ET
from pathlib import Path

from motion_planner.configuration import expand_project_path
from motion_planner.make_primitive_urdf import build_primitive_urdf

# --- 스트로크 [m] ------------------------------------------------------------
# 슬라이더 블록 외곽이 레일 끝에 닿을 때까지의 기하학적 한계.
#   X: HSR55 레일 5000 mm, 슬라이더군 외곽 ±304 mm  -> ±(2500 - 304) = ±2196
#   Y: HSR55 레일 2800 mm, 슬라이더군 외곽 ±328.95  -> ±(1400 - 329) = ±1071
# 랙기어 길이(X 5010.7, Y 2610.6)는 둘 다 이보다 넉넉해서 구속 조건이 아니다.
DEFAULT_X_TRAVEL = 4.392
DEFAULT_Y_TRAVEL = 2.142
# 팔 base_link 장착면의 바닥 기준 높이 [m]. CAD: 베이스 원점 -248, 바닥 -2314.
DEFAULT_MOUNT_Z = 2.066

# --- 브리지 중심(CAD Z=0) 기준 아래쪽 오프셋 [m] ------------------------------
# 팔 base_link 는 브리지 중심에서 248 mm 아래.
MOUNT_BELOW_BRIDGE = 0.248
# X 방향 박스빔(5300 x 200 x 250) 상면 -247, 중심 -372, 하면 -497.
X_BEAM_CENTER_BELOW_BRIDGE = 0.372
# X 레일(53 x 44) 중심 -207.
X_RAIL_CENTER_BELOW_BRIDGE = 0.207
# 기둥 상단 -533.
COLUMN_TOP_BELOW_BRIDGE = 0.533

# --- 고정 프레임 실측 치수 [m] -----------------------------------------------
# 기둥 H형강 244 x 175 x 1700, 프레임 중심 기준 X ±1744.0 / Y ±1509.5.
COLUMN_SIZE = (0.244, 0.175)
COLUMN_X = 1.744
COLUMN_Y = 1.5095
# 베이스 플레이트(500 x 500 x 25) + 방진 레벨링 풋을 한 덩어리 받침으로 합쳤다.
PEDESTAL_SIZE = (0.5, 0.5, 0.079)
# X 박스빔 2개: 길이 5300, 단면 200(Y) x 250(Z), Y ±1509.5.
X_BEAM_SIZE = (5.3, 0.2, 0.25)
# X 레일 2개: 길이 5000, 53(Y) x 44(Z), Y ±1479.5.
X_RAIL_SIZE = (5.0, 0.053, 0.044)
X_RAIL_Y = 1.4795

# --- 브리지(gantry_x_link) 실측 치수 [m] --------------------------------------
# single_box_beam_3500 은 이름과 달리 실제 3300 mm 다.
BRIDGE_BEAM_SIZE = (0.15, 3.3, 0.25)
# Y 레일 2개(상/하면): 2800 길이, 53(X) x 44(Z), 브리지 중심 기준 X +18.5, Z ±165.
BRIDGE_RAIL_SIZE = (0.053, 2.8, 0.044)
BRIDGE_RAIL_X = 0.0185
BRIDGE_RAIL_Z = 0.165

# --- 캐리지(gantry_y_link) 실측 치수 [m] --------------------------------------
# 상/하/측면 플레이트가 만드는 상자. 브리지를 감싸고 내려온다.
CARRIAGE_SIZE = (0.257, 0.65, 0.466)
# 캐리지 중심은 브리지 중심에서 X +28 mm 오프셋돼 있다.
CARRIAGE_X_OFFSET = 0.028
# 로봇 장착 플레이트 220 x 220 x 20, 캐리지 중심에서 Z -243.
MOUNT_PLATE = (0.22, 0.22, 0.02)
MOUNT_PLATE_Z = -0.243
# 캐리지 중심 -> 팔 base_link 원점. CAD (34.036, -109.185) vs 캐리지 (28, -109.2).
ARM_MOUNT_XY = (0.006036, 0.000015)
# 로봇은 캐리지에 90도 돌아 앉아 있다(CAD 배치행렬 rpy = 180, 0, 90).
ARM_MOUNT_YAW = math.pi / 2.0


def _sub_origin(parent, xyz=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)):
    origin = ET.SubElement(parent, "origin")
    origin.set("xyz", " ".join(f"{value:.6f}" for value in xyz))
    origin.set("rpy", " ".join(f"{value:.6f}" for value in rpy))
    return origin


def _add_prismatic_joint(root, name, parent_link, child_link, axis, lower, upper,
                         origin_xyz=(0.0, 0.0, 0.0)):
    joint = ET.SubElement(root, "joint")
    joint.set("name", name)
    joint.set("type", "prismatic")
    ET.SubElement(joint, "parent").set("link", parent_link)
    ET.SubElement(joint, "child").set("link", child_link)
    _sub_origin(joint, origin_xyz)
    ET.SubElement(joint, "axis").set("xyz", " ".join(str(value) for value in axis))
    limit = ET.SubElement(joint, "limit")
    limit.set("effort", "2000")
    limit.set("lower", f"{lower:.6f}")
    limit.set("upper", f"{upper:.6f}")
    limit.set("velocity", "1.5")
    return joint


def _as_collision(visual):
    """visual 엘리먼트를 같은 형상의 collision 으로 바꾼다(material 은 뺀다).

    갠트리 프레임에도 collision 이 있어야 팔과의 간섭을 검사할 수 있다.
    시각 형상과 같은 형상을 쓰므로 RViz 에 보이는 것이 곧 검사 대상이다.
    """
    collision = ET.Element("collision")
    for child in visual:
        if child.tag != "material":
            collision.append(copy.deepcopy(child))
    return collision


def _add_link(root, name, visuals=(), inertial=True):
    link = ET.SubElement(root, "link")
    link.set("name", name)
    if not inertial:
        # KDL은 루트 링크의 inertia를 지원하지 않아 robot_state_publisher가 경고를 낸다.
        for visual in visuals:
            link.append(visual)
        for visual in visuals:
            link.append(_as_collision(visual))
        return link
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "mass").set("value", "1.0")
    _sub_origin(inertial, (0.0, 0.0, 0.0))
    inertia = ET.SubElement(inertial, "inertia")
    for key, value in (("ixx", "0.01"), ("ixy", "0"), ("ixz", "0"),
                       ("iyy", "0.01"), ("iyz", "0"), ("izz", "0.01")):
        inertia.set(key, value)
    for visual in visuals:
        link.append(visual)
    for visual in visuals:
        link.append(_as_collision(visual))
    return link


def _box_visual(size, xyz, material="dr_gray"):
    visual = ET.Element("visual")
    _sub_origin(visual, xyz)
    geometry = ET.SubElement(visual, "geometry")
    box = ET.SubElement(geometry, "box")
    box.set("size", " ".join(f"{value:.6f}" for value in size))
    ET.SubElement(visual, "material").set("name", material)
    return visual


def _ensure_material(root, name, rgba):
    if root.find(f"./material[@name='{name}']") is not None:
        return
    material = ET.Element("material")
    material.set("name", name)
    ET.SubElement(material, "color").set("rgba", rgba)
    root.insert(0, material)


def build_gantry_urdf(arm_urdf, destination, x_travel=DEFAULT_X_TRAVEL,
                      y_travel=DEFAULT_Y_TRAVEL, mount_z=DEFAULT_MOUNT_Z):
    tree = ET.parse(arm_urdf)
    root = tree.getroot()
    root.set("name", "paint_cell_8axis")

    arm_root = "base_link"
    if root.find(f"./link[@name='{arm_root}']") is None:
        raise ValueError(f"{arm_urdf} 에 {arm_root} 링크가 없다")

    _ensure_material(root, "cell_frame", "0.25 0.27 0.30 1.0")
    _ensure_material(root, "cell_rail", "0.55 0.57 0.60 1.0")

    half_x = x_travel / 2.0
    half_y = y_travel / 2.0
    # 브리지 중심이 모든 높이의 기준. 팔 장착면이 그보다 248 mm 아래다.
    bridge_z = mount_z + MOUNT_BELOW_BRIDGE
    column_top_z = bridge_z - COLUMN_TOP_BELOW_BRIDGE
    column_height = column_top_z - PEDESTAL_SIZE[2]

    if column_height <= 0.0:
        raise ValueError(
            f"--mount-z 가 너무 낮다 (기둥 길이 {column_height:.3f} m). "
            f"실측 장착 높이는 {DEFAULT_MOUNT_Z:.3f} m 다"
        )

    # --- 고정 프레임: 받침 4 + 기둥 4 + X 박스빔 2 + X 레일 2 -------------
    frame_visuals = []
    for sign_x in (-1.0, 1.0):
        for sign_y in (-1.0, 1.0):
            frame_visuals.append(_box_visual(
                PEDESTAL_SIZE,
                (sign_x * COLUMN_X, sign_y * COLUMN_Y, PEDESTAL_SIZE[2] / 2.0),
                material="cell_frame",
            ))
            frame_visuals.append(_box_visual(
                (COLUMN_SIZE[0], COLUMN_SIZE[1], column_height),
                (sign_x * COLUMN_X, sign_y * COLUMN_Y,
                 PEDESTAL_SIZE[2] + column_height / 2.0),
                material="cell_frame",
            ))
    for sign_y in (-1.0, 1.0):
        frame_visuals.append(_box_visual(
            X_BEAM_SIZE,
            (0.0, sign_y * COLUMN_Y, bridge_z - X_BEAM_CENTER_BELOW_BRIDGE),
            material="cell_frame",
        ))
        frame_visuals.append(_box_visual(
            X_RAIL_SIZE,
            (0.0, sign_y * X_RAIL_Y, bridge_z - X_RAIL_CENTER_BELOW_BRIDGE),
            material="cell_rail",
        ))
    _add_link(root, "world_base", inertial=False, visuals=frame_visuals)

    # --- X 이송: Y 방향 브리지가 X 레일 위를 달린다 -----------------------
    bridge_visuals = [_box_visual(BRIDGE_BEAM_SIZE, (0.0, 0.0, 0.0),
                                  material="cell_frame")]
    for sign_z in (-1.0, 1.0):
        bridge_visuals.append(_box_visual(
            BRIDGE_RAIL_SIZE,
            (BRIDGE_RAIL_X, 0.0, sign_z * BRIDGE_RAIL_Z),
            material="cell_rail",
        ))
    _add_link(root, "gantry_x_link", visuals=bridge_visuals)
    _add_prismatic_joint(root, "gantry_x", "world_base", "gantry_x_link",
                         (1, 0, 0), -half_x, half_x, (0.0, 0.0, bridge_z))

    # --- Y 이송: 캐리지가 브리지를 감싸고 달린다. 팔 장착 플레이트는 그 아래 ---
    _add_link(root, "gantry_y_link", visuals=[
        _box_visual(CARRIAGE_SIZE, (0.0, 0.0, 0.0), material="cell_frame"),
        _box_visual(MOUNT_PLATE, (0.0, 0.0, MOUNT_PLATE_Z), material="cell_rail"),
    ])
    _add_prismatic_joint(root, "gantry_y", "gantry_x_link", "gantry_y_link",
                         (0, 1, 0), -half_y, half_y,
                         (CARRIAGE_X_OFFSET, 0.0, 0.0))

    # --- 팔 장착: roll = pi 로 뒤집어 아래로 뻗게 하고, yaw = pi/2 로 돌린다 ---
    mount = ET.SubElement(root, "joint")
    mount.set("name", "gantry_to_arm")
    mount.set("type", "fixed")
    ET.SubElement(mount, "parent").set("link", "gantry_y_link")
    ET.SubElement(mount, "child").set("link", arm_root)
    _sub_origin(mount,
                (ARM_MOUNT_XY[0], ARM_MOUNT_XY[1], -MOUNT_BELOW_BRIDGE),
                (math.pi, 0.0, ARM_MOUNT_YAW))

    destination = Path(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    tree.write(destination, encoding="utf-8", xml_declaration=True)
    return destination


# =============================================================================
# 환경 설정(config/gantry/<env>.yaml) 기반 생성 — gantry_env 참조
# =============================================================================
# 위의 build_gantry_urdf 는 인자 없는 기본 호출이 기존 paint_cell_8axis.urdf 와 바이트 동일해야
# 하므로 건드리지 않는다. 아래는 yaml 하나로 프레임 치수·조인트 원점/부호·팔 오프셋·툴체인저
# 슬롯까지 정하는 생성 경로다. `--env legacy` 는 위 기본 경로와 같은 갠트리 부분을 만든다(회귀 검증).


def _box_collision(size, xyz):
    collision = ET.Element("collision")
    _sub_origin(collision, xyz)
    geometry = ET.SubElement(collision, "geometry")
    ET.SubElement(geometry, "box").set("size", " ".join(f"{value:.6f}" for value in size))
    return collision


def _add_link_with_mass(root, name, visuals, mass, main_box_size, com=(0.0, 0.0, 0.0)):
    """질량·박스 관성을 명시한 링크 (spec_c 스타일: 브리지 180 kg / 캐리지 160 kg)."""
    link = ET.SubElement(root, "link")
    link.set("name", name)
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "mass").set("value", f"{mass:.1f}")
    _sub_origin(inertial, com)
    sx, sy, sz = main_box_size
    ixx = mass / 12.0 * (sy * sy + sz * sz)
    iyy = mass / 12.0 * (sx * sx + sz * sz)
    izz = mass / 12.0 * (sx * sx + sy * sy)
    inertia = ET.SubElement(inertial, "inertia")
    for key, value in (("ixx", f"{ixx:.2f}"), ("ixy", "0"), ("ixz", "0"),
                       ("iyy", f"{iyy:.2f}"), ("iyz", "0"), ("izz", f"{izz:.2f}")):
        inertia.set(key, value)
    for visual in visuals:
        link.append(visual)
    for visual in visuals:
        link.append(_as_collision(visual))
    return link


def _legacy_stp_frames(env, bridge_z):
    """legacy(gantry.stp) 스타일: yaml frame 값으로 build_gantry_urdf 와 같은 박스를 만든다.

    (world_base visuals, bridge visuals, carriage visuals, gantry_y origin xyz, 캐리지 질량 없음)
    """
    f = env["frame"]
    column_size = tuple(f["column_size"])
    pedestal = tuple(f["pedestal_size"])
    column_x, column_y = float(f["column_x"]), float(f["column_y"])
    column_top_z = bridge_z - float(f["column_top_below_bridge"])
    column_height = column_top_z - pedestal[2]
    if column_height <= 0.0:
        raise ValueError(f"레일 높이 {bridge_z:.3f} m 가 너무 낮다 (기둥 길이 {column_height:.3f} m)")

    frame_visuals = []
    for sign_x in (-1.0, 1.0):
        for sign_y in (-1.0, 1.0):
            frame_visuals.append(_box_visual(
                pedestal, (sign_x * column_x, sign_y * column_y, pedestal[2] / 2.0),
                material="cell_frame"))
            frame_visuals.append(_box_visual(
                (column_size[0], column_size[1], column_height),
                (sign_x * column_x, sign_y * column_y, pedestal[2] + column_height / 2.0),
                material="cell_frame"))
    for sign_y in (-1.0, 1.0):
        frame_visuals.append(_box_visual(
            tuple(f["x_beam_size"]),
            (0.0, sign_y * column_y, bridge_z - float(f["x_beam_center_below_bridge"])),
            material="cell_frame"))
        frame_visuals.append(_box_visual(
            tuple(f["x_rail_size"]),
            (0.0, sign_y * float(f["x_rail_y"]), bridge_z - float(f["x_rail_center_below_bridge"])),
            material="cell_rail"))

    bridge_visuals = [_box_visual(tuple(f["bridge_beam_size"]), (0.0, 0.0, 0.0), material="cell_frame")]
    for sign_z in (-1.0, 1.0):
        bridge_visuals.append(_box_visual(
            tuple(f["bridge_rail_size"]),
            (float(f["bridge_rail_x"]), 0.0, sign_z * float(f["bridge_rail_z"])),
            material="cell_rail"))

    carriage_visuals = [
        _box_visual(tuple(f["carriage_size"]), (0.0, 0.0, 0.0), material="cell_frame"),
        _box_visual(tuple(f["mount_plate"]), (0.0, 0.0, float(f["mount_plate_z"])), material="cell_rail"),
    ]
    return frame_visuals, bridge_visuals, carriage_visuals


def _spec_c_box_frames(env, bridge_z):
    """spec_c(gantry_base.stl 실측 부재) 스타일: 단면/오프셋은 yaml, 길이는 레일 길이에서 계산."""
    f = env["frame"]
    rail_x = float(env.rail["x_length"])
    rail_y = float(env.rail["y_length"])
    col = f["column"]
    col_sx, col_sy = (float(v) for v in col["section"])
    pedestal = tuple(float(v) for v in f["pedestal"]["size"])
    bridge_len = rail_y + float(f["bridge_beam"]["length_extra"])
    cx = rail_x / 2.0 - float(col["inset_x"])
    cy = bridge_len / 2.0 - float(col["inset_y"])
    beam_sec = tuple(float(v) for v in f["x_beam"]["section"])
    beam_z = bridge_z - float(f["x_beam"]["below_bridge"])
    beam_bottom = beam_z - beam_sec[1] / 2.0
    column_height = beam_bottom - pedestal[2]
    if column_height <= 0.0:
        raise ValueError(f"레일 높이 {bridge_z:.3f} m 가 너무 낮다 (기둥 길이 {column_height:.3f} m)")
    rail_sec = tuple(float(v) for v in f["x_rail"]["section"])
    rail_y_pos = cy - float(f["x_rail"]["inset_y"])
    rail_z = bridge_z - float(f["x_rail"]["below_bridge"])

    frame_visuals = []
    for sign_x in (-1.0, 1.0):
        for sign_y in (-1.0, 1.0):
            frame_visuals.append(_box_visual(
                pedestal, (sign_x * cx, sign_y * cy, pedestal[2] / 2.0), material="cell_frame"))
            frame_visuals.append(_box_visual(
                (col_sx, col_sy, column_height),
                (sign_x * cx, sign_y * cy, pedestal[2] + column_height / 2.0), material="cell_frame"))
    for sign_y in (-1.0, 1.0):
        frame_visuals.append(_box_visual(
            (rail_x, beam_sec[0], beam_sec[1]), (0.0, sign_y * cy, beam_z), material="cell_frame"))
        frame_visuals.append(_box_visual(
            (rail_x, rail_sec[0], rail_sec[1]), (0.0, sign_y * rail_y_pos, rail_z), material="cell_rail"))
    brace = f.get("lower_brace")
    if brace:
        b_sec = tuple(float(v) for v in brace["section"])
        b_z = float(brace["z"])
        for sign_y in (-1.0, 1.0):   # X 방향 브레이스 (기둥 사이)
            frame_visuals.append(_box_visual(
                (2.0 * cx - col_sx, b_sec[0], b_sec[1]), (0.0, sign_y * cy, b_z), material="cell_frame"))
        for sign_x in (-1.0, 1.0):   # Y 방향 브레이스
            frame_visuals.append(_box_visual(
                (b_sec[0], 2.0 * cy - col_sy, b_sec[1]), (sign_x * cx, 0.0, b_z), material="cell_frame"))

    bb = f["bridge_beam"]
    bb_sec = tuple(float(v) for v in bb["section"])
    bridge_main = (bb_sec[0], bridge_len, bb_sec[1])
    bridge_visuals = [_box_visual(bridge_main, (float(bb["center_x"]), 0.0, 0.0), material="cell_frame")]
    br = f["bridge_rail"]
    br_sec = tuple(float(v) for v in br["section"])
    for z in br["z"]:
        bridge_visuals.append(_box_visual(
            (br_sec[0], rail_y, br_sec[1]), (float(br["center_x"]), 0.0, float(z)), material="cell_rail"))
    plate = f.get("bridge_end_plate")
    if plate:
        for sign_y in (-1.0, 1.0):
            bridge_visuals.append(_box_visual(
                tuple(float(v) for v in plate["size"]),
                (float(plate["center_x"]), sign_y * cy, float(plate["z"])), material="cell_frame"))

    carriage_visuals = [
        _box_visual(tuple(float(v) for v in box["size"]), tuple(float(v) for v in box["center"]),
                    material="cell_frame")
        for box in f["carriage_boxes"]
    ]
    return frame_visuals, bridge_visuals, carriage_visuals, bridge_main


def _append_tool_links(root):
    """tool_assy / spray_gun / tcp 링크 + collision 박스 (paint_cell_8axis.urdf 수작업 블록과 같은 운동학).

    메시는 package://paint_viz/urdf/<stl> 로 참조한다 (setup.py 가 urdf/*.stl 을 share 에 설치하고,
    pinocchio 쪽은 gantry_env.default_package_dirs() 가 해석). 절대경로를 쓰면 다른 PC 에서 깨진다."""
    from motion_planner.make_dock_urdfs import SPRAY_GUN_BOXES, TOOL_ASSY_BOXES

    def mesh_visual(filename):
        visual = ET.Element("visual")
        origin = ET.SubElement(visual, "origin")
        origin.set("xyz", "0 0 0")
        origin.set("rpy", "0 0 0")
        geometry = ET.SubElement(visual, "geometry")
        ET.SubElement(geometry, "mesh").set("filename", f"package://paint_viz/urdf/{filename}")
        ET.SubElement(visual, "material").set("name", "dr_gray")
        return visual

    def fixed_joint(name, parent, child, xyz):
        joint = ET.SubElement(root, "joint")
        joint.set("name", name)
        joint.set("type", "fixed")
        ET.SubElement(joint, "parent").set("link", parent)
        ET.SubElement(joint, "child").set("link", child)
        _sub_origin(joint, xyz)

    _ensure_material(root, "dr_gray", "0.6 0.6 0.6 1.0")
    tool = ET.SubElement(root, "link")
    tool.set("name", "tool_assy")
    tool.append(mesh_visual("tool_assy.stl"))
    for center, size in TOOL_ASSY_BOXES:
        tool.append(_box_collision(size, center))
    fixed_joint("flange_to_tool", "link_6", "tool_assy", (0.0, 0.0, 0.0))
    gun = ET.SubElement(root, "link")
    gun.set("name", "spray_gun")
    gun.append(mesh_visual("spray_gun.stl"))
    for center, size in SPRAY_GUN_BOXES:
        gun.append(_box_collision(size, center))
    # 노즐 팁 = link_6 기준 (0.1292, -0.0607, 0.2969), +z 분사 방향 (paint_cell_8axis.urdf 와 동일)
    fixed_joint("tool_to_gun", "link_6", "spray_gun", TOOL_TO_GUN_XYZ)
    ET.SubElement(root, "link").set("name", "tcp")
    fixed_joint("spray_gun_to_tcp", "spray_gun", "tcp", (0.0, 0.0, 0.0))


# link_6 -> spray_gun(노즐 팁) 오프셋. 기존 툴 URDF 들과 동일 (tool_to_gun 조인트).
TOOL_TO_GUN_XYZ = (0.1292, -0.0607, 0.2969)


def _append_slot_frames(root, env):
    for name, x, y, z, _occupied in env.toolchanger_slots():
        ET.SubElement(root, "link").set("name", name)
        joint = ET.SubElement(root, "joint")
        joint.set("name", f"world_base_to_{name}")
        joint.set("type", "fixed")
        ET.SubElement(joint, "parent").set("link", "world_base")
        ET.SubElement(joint, "child").set("link", name)
        _sub_origin(joint, (x, y, z))


def _strip_tool_links(root):
    """tool_assy / spray_gun / tcp 를 빼서 베어 플랜지판으로 만든다 (make_dock_urdfs 의 _bare 와 동일 규약)."""
    for tag, attr, names in (("link", "name", ("tool_assy", "spray_gun", "tcp")),
                             ("joint", "name", ("flange_to_tool", "tool_to_gun", "spray_gun_to_tcp"))):
        for element in list(root.findall(tag)):
            if element.get(attr) in names:
                root.remove(element)


def build_env_urdf(env, arm_urdf, destination, with_tool=True, bare_destination=None):
    """gantry_env 설정으로 8축 셀 URDF 를 만든다. (툴 포함본, 선택적으로 베어본도)"""
    tree = ET.parse(arm_urdf)
    root = tree.getroot()
    root.set("name", f"paint_cell_8axis_{env.name}")
    if root.find("./link[@name='base_link']") is None:
        raise ValueError(f"{arm_urdf} 에 base_link 링크가 없다")
    _ensure_material(root, "cell_frame", "0.25 0.27 0.30 1.0")
    _ensure_material(root, "cell_rail", "0.55 0.57 0.60 1.0")

    bridge_z = env.bridge_z
    style = str(env.get("frame.style", "legacy_stp"))
    if style == "legacy_stp":
        frame_visuals, bridge_visuals, carriage_visuals = _legacy_stp_frames(env, bridge_z)
        bridge_main = None
    elif style == "spec_c_boxes":
        frame_visuals, bridge_visuals, carriage_visuals, bridge_main = _spec_c_box_frames(env, bridge_z)
    else:
        raise ValueError(f"frame.style={style!r} 는 legacy_stp | spec_c_boxes 중 하나여야 한다")

    # 툴체인저 스탠드/파킹 플레이트 collision (visual 없이 collision 만 — 메시 환경의 gantry_base.stl 과 같은 역할)
    extra_collisions = []
    if env.data.get("toolchanger"):
        plate = tuple(float(v) for v in env.get("toolchanger.plate_size"))
        for _name, x, y, top_z, occupied in env.toolchanger_slots():
            if occupied:
                extra_collisions.append(_box_collision(plate, (x, y, top_z - plate[2] / 2.0)))
        for center, size in env.toolchanger_stand_boxes():
            # 스탠드는 visual 로 넣는다 (_add_link 가 visual 마다 collision 을 같이 만든다 — 중복 추가 금지)
            frame_visuals.append(_box_visual(size, center, material="cell_rail"))
    world = _add_link(root, "world_base", inertial=False, visuals=frame_visuals)
    for collision in extra_collisions:
        world.append(collision)

    mass = env.get("frame.mass")
    jx, jy = env.joint("gantry_x"), env.joint("gantry_y")
    if mass and bridge_main is not None:
        _add_link_with_mass(root, "gantry_x_link", bridge_visuals, float(mass["bridge"]), bridge_main,
                            com=(float(env.get("frame.bridge_beam.center_x", 0.0)), 0.0, 0.0))
    else:
        _add_link(root, "gantry_x_link", visuals=bridge_visuals)
    _add_prismatic_joint(root, "gantry_x", "world_base", "gantry_x_link",
                         tuple(int(v) for v in jx["axis"]), float(jx["lower"]), float(jx["upper"]),
                         tuple(float(v) for v in jx["origin_xyz"]))
    if mass and bridge_main is not None:
        main_box = env["frame"]["carriage_boxes"][0]
        _add_link_with_mass(root, "gantry_y_link", carriage_visuals, float(mass["carriage"]),
                            tuple(float(v) for v in main_box["size"]),
                            com=tuple(float(v) for v in main_box["center"]))
    else:
        _add_link(root, "gantry_y_link", visuals=carriage_visuals)
    _add_prismatic_joint(root, "gantry_y", "gantry_x_link", "gantry_y_link",
                         tuple(int(v) for v in jy["axis"]), float(jy["lower"]), float(jy["upper"]),
                         tuple(float(v) for v in jy["origin_xyz"]))

    mount = ET.SubElement(root, "joint")
    mount.set("name", "gantry_to_arm")
    mount.set("type", "fixed")
    ET.SubElement(mount, "parent").set("link", "gantry_y_link")
    ET.SubElement(mount, "child").set("link", "base_link")
    _sub_origin(mount, tuple(float(v) for v in env["arm_mount"]["xyz"]),
                tuple(float(v) for v in env["arm_mount"]["rpy"]))

    destination = Path(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    if with_tool:
        _append_tool_links(root)
    _append_slot_frames(root, env)
    tree.write(destination, encoding="utf-8", xml_declaration=True)
    outputs = [destination]
    if bare_destination is not None:
        _strip_tool_links(root)
        tree.write(bare_destination, encoding="utf-8", xml_declaration=True)
        outputs.append(Path(bare_destination))
    return outputs


def _main_env(args):
    from motion_planner.configuration import load_gantry_env

    env = load_gantry_env(args.env)
    arm_source = args.arm_urdf or Path(expand_project_path(env.get("urdf.arm_source")))
    if not arm_source.is_file():
        raise FileNotFoundError(f"팔 URDF 가 없다: {arm_source} (--arm-urdf 로 지정)")
    out_dir = args.output_dir or env.urdf_path("primitive", must_exist=False).parent
    out_dir.mkdir(parents=True, exist_ok=True)
    if not env.get("urdf.generated") and not args.force:
        raise SystemExit(
            f"[gantry-urdf] 환경 '{env.name}' 은 urdf.generated=false (기존 파일 사용). "
            f"그래도 만들려면 --force 와 --output-dir 를 같이 줄 것")

    outputs = []
    # 프리미티브 팔 본
    primitive_arm = out_dir / f"{env.urdf_name('primitive').rsplit('.', 1)[0]}_arm_primitive.urdf"
    arm_primitive = build_primitive_urdf(arm_source, primitive_arm)
    outputs += build_env_urdf(env, arm_primitive, out_dir / env.urdf_name("primitive"),
                              with_tool=not args.no_tool)
    # 원본 메시 팔 본 (+ 베어 플랜지판)
    mesh_name = env.get("urdf.mesh")
    if mesh_name and not args.primitive_only:
        bare_name = env.get("urdf.bare")
        outputs += build_env_urdf(env, arm_source, out_dir / mesh_name, with_tool=not args.no_tool,
                                  bare_destination=(out_dir / bare_name) if bare_name else None)

    travel = env.world_travel_range()
    for path in outputs:
        print(f"[gantry-urdf] 생성 완료: {path}")
    print(f"[gantry-urdf] 환경 {env.name}: 레일 {env.rail['x_length']} x {env.rail['y_length']} m, "
          f"스트로크 X {env.stroke['x']} / Y {env.stroke['y']} m, "
          f"캐리지 월드 X {travel['x'][0]:+.4f}~{travel['x'][1]:+.4f} / Y {travel['y'][0]:+.4f}~{travel['y'][1]:+.4f}")
    print(f"[gantry-urdf] 브리지 중심 {env.bridge_z:.4f} m, 팔 장착면 {env.mount_z:.4f} m, 원점 규약 {env.convention}")
    slots = env.toolchanger_slots()
    if slots:
        print("[gantry-urdf] 툴체인저 슬롯: " + ", ".join(
            f"{name}=({x:.4f}, {y:.4f}, {z:.4f}){'' if occ else ' 비어있음'}" for name, x, y, z, occ in slots))
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="A0912 + XY 갠트리(천장 장착, 아래로 뻗음) 8축 URDF 생성"
    )
    parser.add_argument("--env", default=None,
                        help="갠트리 환경 이름/yaml (config/gantry). 주면 아래 --x-travel/--y-travel/--mount-z 대신 "
                             "yaml 값으로 프리미티브본·메시본·베어본을 한 번에 만든다 (예: --env x7y4z3)")
    parser.add_argument("--output-dir", type=Path, default=None,
                        help="--env 전용: 출력 디렉터리 (기본 패키지 urdf/)")
    parser.add_argument("--no-tool", action="store_true", help="--env 전용: 툴(tool_assy/spray_gun/tcp) 링크 생략")
    parser.add_argument("--primitive-only", action="store_true", help="--env 전용: 메시본/베어본 생략")
    parser.add_argument("--force", action="store_true",
                        help="--env 전용: urdf.generated=false 인 환경도 생성 (기존 파일 덮어쓰기 방지용 --output-dir 와 함께)")
    parser.add_argument("--arm-urdf", type=Path, default=None,
                        help="원본 a0912 URDF (--env 를 주면 yaml urdf.arm_source 가 기본)")
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--x-travel", type=float, default=DEFAULT_X_TRAVEL,
                        help=f"갠트리 X 스트로크 [m] (기본 {DEFAULT_X_TRAVEL}, CAD 실측)")
    parser.add_argument("--y-travel", type=float, default=DEFAULT_Y_TRAVEL,
                        help=f"갠트리 Y 스트로크 [m] (기본 {DEFAULT_Y_TRAVEL}, CAD 실측)")
    parser.add_argument("--mount-z", type=float, default=DEFAULT_MOUNT_Z,
                        help="팔 base_link 장착면의 바닥 기준 높이 [m] "
                             f"(기본 {DEFAULT_MOUNT_Z}, CAD 실측)")
    parser.add_argument("--keep-meshes", action="store_true",
                        help="dsr_description2 가 설치돼 있으면 원본 메시를 그대로 쓴다")
    args = parser.parse_args(argv)

    if args.env:
        return _main_env(args)
    if args.arm_urdf is None or args.output is None:
        parser.error("--env 없이 쓰려면 --arm-urdf 와 --output 이 필요하다")

    arm_urdf = args.arm_urdf
    if not args.keep_meshes:
        # dsr_description2 메시가 없으면 RViz에서 팔이 보이지 않으므로 먼저 프리미티브로 바꾼다.
        primitive = args.output.with_name(args.output.stem + "_arm_primitive.urdf")
        arm_urdf = build_primitive_urdf(args.arm_urdf, primitive)

    output = build_gantry_urdf(arm_urdf, args.output, args.x_travel, args.y_travel, args.mount_z)
    bridge_z = args.mount_z + MOUNT_BELOW_BRIDGE
    print(f"[gantry-urdf] 생성 완료: {output}")
    print(f"[gantry-urdf] 스트로크: X {args.x_travel:.3f} m, Y {args.y_travel:.3f} m "
          f"(레일 {X_RAIL_SIZE[0]:.2f} x {BRIDGE_RAIL_SIZE[1]:.2f} m)")
    print(f"[gantry-urdf] 브리지 중심 {bridge_z:.3f} m, 팔 장착면 {args.mount_z:.3f} m, "
          f"팔은 아래(-Z)로 뻗고 yaw {math.degrees(ARM_MOUNT_YAW):.0f}도 회전")
    print(f"[gantry-urdf] 축 구성: gantry_x, gantry_y (직동) + joint_1..joint_6 (회전) = 8축")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
