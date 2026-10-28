"""메시 없는 URDF를 만든다.

a0912.white.urdf 는 시각 형상을 package://dsr_description2/meshes/... 로 참조하는데
이 저장소에도, 시스템 /opt/ros/humble 에도 dsr_description2 패키지가 없다.
그대로 RViz에 올리면 RobotModel이 메시를 못 찾아 로봇이 보이지 않는다.

이 스크립트는 원본 URDF의 관절 구조(origin/axis/limit)는 그대로 두고
visual/collision 형상만 실린더+구로 바꾼 URDF를 만든다. 운동학은 원본과 동일하므로
관절값 재생 결과도 같고, 링크가 막대 모형으로 보일 뿐이다.

dsr_description2 를 설치했다면 이 파일 대신 원본 URDF를 쓰면 된다.
"""

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

LINK_RADIUS = 0.035
JOINT_RADIUS = 0.05


def _rpy_from_direction(direction):
    """z축을 direction 방향으로 돌리는 고정축 rpy를 구한다."""
    norm = float(np.linalg.norm(direction))
    if norm < 1.0e-9:
        return (0.0, 0.0, 0.0)
    unit = np.asarray(direction, dtype=float) / norm
    # z축 기준: pitch로 기울이고 yaw로 돌린다.
    pitch = float(np.arccos(np.clip(unit[2], -1.0, 1.0)))
    yaw = float(np.arctan2(unit[1], unit[0]))
    return (0.0, pitch, yaw)


def _origin_xyz(joint):
    origin = joint.find("origin")
    if origin is None:
        return np.zeros(3)
    return np.array([float(value) for value in origin.get("xyz", "0 0 0").split()])


def _make_visual(geometry, xyz, rpy, material_name):
    visual = ET.Element("visual")
    origin = ET.SubElement(visual, "origin")
    origin.set("xyz", " ".join(f"{value:.6f}" for value in xyz))
    origin.set("rpy", " ".join(f"{value:.6f}" for value in rpy))
    geometry_element = ET.SubElement(visual, "geometry")
    geometry_element.append(geometry)
    material = ET.SubElement(visual, "material")
    material.set("name", material_name)
    return visual


def _cylinder(length, radius):
    element = ET.Element("cylinder")
    element.set("length", f"{length:.6f}")
    element.set("radius", f"{radius:.6f}")
    return element


def _sphere(radius):
    element = ET.Element("sphere")
    element.set("radius", f"{radius:.6f}")
    return element


def build_primitive_urdf(source, destination):
    tree = ET.parse(source)
    root = tree.getroot()

    links = {link.get("name"): link for link in root.findall("link")}
    children_of = {}
    for joint in root.findall("joint"):
        parent = joint.find("parent").get("link")
        children_of.setdefault(parent, []).append(joint)

    # 원본 시각/충돌 형상 제거 (관성은 그대로 둔다)
    for link in links.values():
        for tag in ("visual", "collision"):
            for element in link.findall(tag):
                link.remove(element)

    for name, link in links.items():
        for joint in children_of.get(name, []):
            offset = _origin_xyz(joint)
            length = float(np.linalg.norm(offset))
            if length < 1.0e-4:
                continue
            link.append(
                _make_visual(
                    _cylinder(length, LINK_RADIUS),
                    offset / 2.0,
                    _rpy_from_direction(offset),
                    "dr_white",
                )
            )
            link.append(
                _make_visual(_sphere(JOINT_RADIUS), offset, (0.0, 0.0, 0.0), "dr_gray")
            )
        # 링크 자체 원점 표시 (말단 링크도 보이도록)
        link.append(_make_visual(_sphere(JOINT_RADIUS * 0.6), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), "dr_gray"))

    destination = Path(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    tree.write(destination, encoding="utf-8", xml_declaration=True)
    return destination


def main(argv=None):
    parser = argparse.ArgumentParser(description="메시 참조를 실린더/구로 바꾼 URDF 생성")
    parser.add_argument("--source", required=True, type=Path, help="원본 URDF")
    parser.add_argument("--output", required=True, type=Path, help="출력 URDF")
    args = parser.parse_args(argv)

    output = build_primitive_urdf(args.source, args.output)
    print(f"[primitive-urdf] 생성 완료: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
