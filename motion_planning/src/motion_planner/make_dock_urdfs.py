"""툴체인저 도킹 검증용 URDF 2종을 만든다.

입력: paint_cell_8axis_gantry_mesh_c.urdf (툴 장착 최종본)
출력:
  1) 입력 파일 자체에 (in-place):
     - tool_assy / spray_gun collision 박스 — 지금은 visual 만 있어 충돌 검사에서
       플랜지 툴이 통째로 빠진다. STL z-슬라이스 실측으로 잡은 근사 박스를 넣는다.
     - toolchanger_slot_1~4 고정 프레임 (world_base 기준, 플레이트 상면 중심, +Z 위)
     - 파킹된 툴측 플레이트 3장 collision 박스 (world_base)
  2) *_bare.urdf: 위에서 tool_assy/spray_gun/tcp 링크·조인트를 뺀 베어 플랜지판.
     툴 교체 직전(빈 플랜지) 접근 모션 검증용.

슬롯 좌표는 gantry.stl(2026-09-03) 실측값이다 — split_gantry_stl 분석 참조.
플레이트: 102(X) x 80(Y) x 44(Z) mm. 하단 2슬롯 상면 z=0.9152, 상단 z=1.2422.
상단-2(x 1.5311)는 비어 있다(플레이트 collision 없음, 프레임만 둔다).

툴측 플레이트는 link_6 프레임에서 80(x) x 102(y) — 도킹 시 link_6 y축이
월드 X축과 정렬되어야 한다 (dock_path.py 의 --flip 으로 180도 반전 선택).
"""

import argparse
import re
import sys
from pathlib import Path
from motion_planner.configuration import urdf_dirs

# (name, x, y, plate_top_z, occupied)
SLOTS = [
    ("toolchanger_slot_1", 1.3311, -1.3019, 0.9152, True),
    ("toolchanger_slot_2", 1.5311, -1.3019, 0.9152, True),
    ("toolchanger_slot_3", 1.3311, -1.5019, 1.2422, True),
    ("toolchanger_slot_4", 1.5311, -1.5019, 1.2422, False),
]
PLATE_SIZE = (0.102, 0.080, 0.044)

# link_6 프레임 tool_assy 근사 박스 (center, size) — STL z-슬라이스 실측
TOOL_ASSY_BOXES = [
    ((-0.010, -0.012, 0.030), (0.092, 0.104, 0.062)),   # 체인저 플레이트/어댑터
    ((-0.014, 0.007, 0.083), (0.082, 0.086, 0.066)),    # 지지 칼럼
    ((0.021, -0.024, 0.175), (0.152, 0.178, 0.122)),    # 매니폴드/헤드
    ((0.065, -0.054, 0.262), (0.099, 0.070, 0.058)),    # 상부 브래킷
]
# spray_gun 링크 프레임 근사 박스
SPRAY_GUN_BOXES = [
    ((0.088, 0.019, -0.030), (0.301, 0.111, 0.127)),    # 건 본체+핸들
    ((-0.012, 0.017, -0.140), (0.138, 0.026, 0.102)),   # 하부 튜브
]


def box_collision_xml(center, size):
    cx, cy, cz = center
    sx, sy, sz = size
    return (f'<collision><origin xyz="{cx:.6f} {cy:.6f} {cz:.6f}" rpy="0 0 0" />'
            f'<geometry><box size="{sx:.6f} {sy:.6f} {sz:.6f}" /></geometry></collision>')


def slot_frames_xml():
    parts = []
    for name, x, y, z, _occ in SLOTS:
        parts.append(
            f'<link name="{name}" />'
            f'<joint name="world_base_to_{name}" type="fixed">'
            f'<parent link="world_base" /><child link="{name}" />'
            f'<origin xyz="{x:.4f} {y:.4f} {z:.4f}" rpy="0 0 0" /></joint>'
        )
    return "".join(parts)


def parked_plate_boxes_xml():
    parts = []
    for _name, x, y, top_z, occupied in SLOTS:
        if not occupied:
            continue
        parts.append(box_collision_xml((x, y, top_z - PLATE_SIZE[2] / 2), PLATE_SIZE))
    return "".join(parts)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("urdf", type=Path, nargs="?",
                    default=urdf_dirs()[0] / "paint_cell_8axis_gantry_mesh_c.urdf")
    args = ap.parse_args(argv)

    text = args.urdf.read_text()
    if "toolchanger_slot_1" in text:
        print(f"[dock-urdf] {args.urdf.name} 에 이미 슬롯 프레임이 있다 — 중복 실행 방지, 중단",
              file=sys.stderr)
        return 1

    # 1) 파킹 플레이트 박스: world_base 링크의 마지막 collision 뒤에 삽입
    marker = '</link><link name="gantry_x_link">'
    assert text.count(marker) == 1
    text = text.replace(marker, parked_plate_boxes_xml() + marker)

    # 2) 툴 collision: visual 만 있는 두 링크에 박스 추가
    tool_marker = '</visual></link><joint name="flange_to_tool"'
    assert text.count(tool_marker) == 1
    text = text.replace(
        tool_marker,
        "</visual>" + "".join(box_collision_xml(*b) for b in TOOL_ASSY_BOXES)
        + '</link><joint name="flange_to_tool"')
    gun_marker = '</visual></link><joint name="tool_to_gun"'
    assert text.count(gun_marker) == 1
    text = text.replace(
        gun_marker,
        "</visual>" + "".join(box_collision_xml(*b) for b in SPRAY_GUN_BOXES)
        + '</link><joint name="tool_to_gun"')

    # 3) 슬롯 프레임: </robot> 앞에 삽입
    text = text.replace("</robot>", slot_frames_xml() + "</robot>")
    args.urdf.write_text(text)
    print(f"[dock-urdf] {args.urdf.name}: 툴 collision {len(TOOL_ASSY_BOXES)+len(SPRAY_GUN_BOXES)}박스, "
          f"파킹 플레이트 3박스, 슬롯 프레임 4개 추가")

    # 4) 베어 플랜지판: 툴 링크/조인트 제거
    bare = text
    for pattern in (
        r'<link name="tool_assy">.*?</link><joint name="flange_to_tool"[^>]*>.*?</joint>',
        r'<link name="spray_gun">.*?</link><joint name="tool_to_gun"[^>]*>.*?</joint>',
        # tcp 프레임(노즐 팁)은 spray_gun 에 붙어 있으므로 건과 함께 뺀다 — 베어 플랜지의 TCP 는 link_6
        r'<link name="tcp" /><joint name="spray_gun_to_tcp"[^>]*>.*?</joint>',
    ):
        bare, n = re.subn(pattern, "", bare, flags=re.S)
        assert n == 1, pattern
    bare_path = args.urdf.with_name(args.urdf.stem + "_bare.urdf")
    bare_path.write_text(bare)
    print(f"[dock-urdf] {bare_path.name}: 베어 플랜지판 생성 (tool_assy/spray_gun 제거)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
