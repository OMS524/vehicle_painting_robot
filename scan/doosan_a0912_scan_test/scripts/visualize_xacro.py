#!/usr/bin/env python3
"""고정/회전 조인트 Xacro/URDF 모델을 지정한 관절각으로 Open3D에 표시한다.

ROS 노드/카메라/로봇을 실행하지 않는다. 입력 파일을 수정하지 않는다.
의존성: python -m pip install xacro numpy open3d trimesh pycollada
"""
import argparse
import sys
from pathlib import Path
from urllib.parse import unquote, urlparse
import xml.etree.ElementTree as ET

import numpy as np

# ==================== 사용자 파라미터 ====================
# 상대 경로를 쓰면 이 Python 파일이 있는 scripts 디렉토리 기준이다.
XACRO_PATH = Path(__file__).resolve().parents[1] / "description/xacro/a0912_to_tool_scan.xacro"
# 회전 관절각(deg). 지정하지 않은 관절은 0도. 실제 로봇을 움직이지 않는다.
JOINT_ANGLES_DEG = {
    # "joint_2": -30.0,
    # "joint_3": 60.0,
}
FRAME_SIZE_MM = 25.0
# None이면 모든 링크 좌표계를 표시한다. 각 축 색상: X=빨강, Y=초록, Z=파랑.
FRAME_NAMES = ("base_link", "link_6", "bracket_link", "camera_base_link", "camera_depth_optical_frame")
BACKGROUND_COLOR = (0.12, 0.14, 0.17, 1.0)
WINDOW_WIDTH, WINDOW_HEIGHT = 1280, 900
# ========================================================


def vector(text, size=3):
    values = np.array([float(value) for value in text.split()])
    if values.shape != (size,) or not np.isfinite(values).all():
        raise ValueError(f"유효한 {size}개 숫자가 필요합니다: {text!r}")
    return values


def origin_matrix(origin):
    """URDF 고정축 RPY: Rz(yaw) @ Ry(pitch) @ Rx(roll), 위치는 m."""
    transform = np.eye(4)
    if origin is None:
        return transform
    roll, pitch, yaw = vector(origin.get("rpy", "0 0 0"))
    sr, sp, sy = np.sin([roll, pitch, yaw])
    cr, cp, cy = np.cos([roll, pitch, yaw])
    transform[:3, :3] = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    transform[:3, 3] = vector(origin.get("xyz", "0 0 0"))
    return transform


def joint_transform(joint, angle_deg=0.0):
    """부모 -> 관절 원점 -> 관절 로컬 축 회전을 합성한다."""
    name, kind = joint.get("name"), joint.get("type")
    transform = origin_matrix(joint.find("origin"))
    if kind == "fixed":
        return transform
    if kind not in ("revolute", "continuous"):
        raise ValueError(f"지원하지 않는 조인트 형식: {name} ({kind})")
    if joint.find("mimic") is not None:
        raise ValueError(f"mimic 조인트는 아직 지원하지 않습니다: {name}")
    angle_deg = float(angle_deg)
    if not np.isfinite(angle_deg):
        raise ValueError(f"관절각은 유한한 deg 값이어야 합니다: {name}")
    angle = np.deg2rad(angle_deg)
    limit = joint.find("limit")
    if kind == "revolute" and limit is not None:
        lower = float(limit.get("lower", "-inf"))
        upper = float(limit.get("upper", "inf"))
        if not lower <= angle <= upper:
            raise ValueError(
                f"관절각 범위 초과: {name}={angle_deg:g} deg "
                f"(허용: {np.rad2deg(lower):g} ~ {np.rad2deg(upper):g} deg)")
    axis_element = joint.find("axis")
    axis = vector(axis_element.get("xyz", "1 0 0") if axis_element is not None else "1 0 0")
    length = np.linalg.norm(axis)
    if not np.isfinite(length) or length < 1e-12:
        raise ValueError(f"관절 회전축은 0이 아닌 유한한 벡터여야 합니다: {name}")
    axis /= length
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    motion = np.eye(4)
    motion[:3, :3] = (np.cos(angle) * np.eye(3)
                      + (1 - np.cos(angle)) * np.outer(axis, axis)
                      + np.sin(angle) * skew)
    return transform @ motion


def link_transforms(robot, joint_angles_deg=None):
    """루트에서 각 링크까지의 변환을 계산한다. 잘못된 연결은 오류 처리."""
    links = {link.attrib["name"]: link for link in robot.findall("link")}
    if not links or len(links) != len(robot.findall("link")):
        raise ValueError("링크가 없거나 링크 이름이 중복됩니다. 매크로 호출이 있는 Xacro를 지정하세요.")
    joints = robot.findall("joint")
    names = [joint.get("name") for joint in joints]
    if any(not name for name in names) or len(names) != len(set(names)):
        raise ValueError("조인트 이름이 없거나 중복됩니다.")
    angles = {} if joint_angles_deg is None else joint_angles_deg
    movable = {joint.get("name") for joint in joints
               if joint.get("type") in ("revolute", "continuous")}
    unknown = set(angles) - movable
    if unknown:
        raise ValueError(f"관절각을 지정한 회전 조인트를 찾을 수 없습니다: {sorted(unknown)}")
    children, parents = {}, {}
    for joint in joints:
        parent = joint.find("parent").attrib["link"]
        child = joint.find("child").attrib["link"]
        if parent not in links or child not in links or child in parents:
            raise ValueError(f"잘못된 링크 연결 또는 부모 중복: {parent} -> {child}")
        parents[child] = parent
        local = joint_transform(joint, angles.get(joint.get("name"), 0.0))
        children.setdefault(parent, []).append((child, local))
    roots = set(links) - set(parents)
    if len(roots) != 1:
        raise ValueError(f"루트 링크가 하나여야 합니다: {sorted(roots)}")
    root = roots.pop()
    transforms = {root: np.eye(4)}
    queue = [root]
    for parent in queue:
        for child, local in children.get(parent, []):
            if child in transforms:
                raise ValueError(f"조인트 순환 연결: {child}")
            transforms[child] = transforms[parent] @ local
            queue.append(child)
    if len(transforms) != len(links):
        raise ValueError("루트에 연결되지 않았거나 순환 연결된 링크가 있습니다.")
    return root, links, transforms


def mesh_path(filename, directory):
    if filename.startswith("file://"):
        uri = urlparse(filename)
        if uri.netloc not in ("", "localhost"):
            raise ValueError("원격 file:// 주소는 지원하지 않습니다.")
        path = Path(unquote(uri.path))
    elif "://" in filename:
        raise ValueError(f"메쉬는 로컬 절대/상대 경로로 지정하세요: {filename}")
    else:
        path = Path(filename).expanduser()
    path = (directory / path).resolve() if not path.is_absolute() else path.resolve()
    if not path.is_file():
        raise FileNotFoundError(f"메쉬 파일을 찾을 수 없습니다: {path}")
    return path


def read_mesh(source):
    import open3d as o3d

    if source.suffix.lower() != ".dae":
        return o3d.io.read_triangle_mesh(str(source))
    # Open3D가 직접 읽지 못하는 COLLADA는 scene node 변환까지 적용해 불러온다.
    # 원본 메쉬 단위는 유지하며 URDF의 scale은 load_model에서 한 번만 적용한다.
    import trimesh

    data = trimesh.load_mesh(str(source), process=False, ignore_broken=False)
    mesh = o3d.geometry.TriangleMesh(
        o3d.utility.Vector3dVector(np.asarray(data.vertices)),
        o3d.utility.Vector3iVector(np.asarray(data.faces)),
    )
    mesh.vertex_normals = o3d.utility.Vector3dVector(np.array(data.vertex_normals, copy=True))
    return mesh


def load_model(path, joint_angles_deg=None):
    import open3d as o3d
    import xacro

    # include/매크로/수식은 직접 해석하지 않고 Xacro 라이브러리로 확장한다.
    robot = ET.fromstring(xacro.process_file(str(path)).toxml())
    root, links, transforms = link_transforms(robot, joint_angles_deg)
    materials = {item.get("name"): item for item in robot.findall("material")}
    meshes = []
    for name, link in links.items():
        for index, visual in enumerate(link.findall("visual")):
            spec = visual.find("geometry/mesh")
            if spec is None:
                raise ValueError(f"현재는 mesh visual만 지원합니다: {name}")
            source = mesh_path(spec.attrib["filename"], path.parent)
            mesh = read_mesh(source)
            if not mesh.has_triangles():
                raise ValueError(f"삼각형 메쉬를 읽지 못했습니다: {source}")
            scale = vector(spec.get("scale", "1 1 1"))
            if np.any(scale <= 0):
                raise ValueError(f"메쉬 배율은 양수여야 합니다: {source}")
            mesh.vertices = o3d.utility.Vector3dVector(np.asarray(mesh.vertices) * scale)
            mesh.transform(transforms[name] @ origin_matrix(visual.find("origin")))
            # DAE의 원본 법선은 보존하고 STL은 분리 정점으로 날카로운 모서리를 유지한다.
            if not mesh.has_vertex_normals():
                mesh.compute_vertex_normals()
            color = np.array([0.75, 0.78, 0.83])
            material = visual.find("material")
            if material is not None:
                if material.find("color") is None:
                    material = materials.get(material.get("name"), material)
                if material.find("color") is not None:
                    color = vector(material.find("color").attrib["rgba"], 4)[:3]
            mesh.paint_uniform_color(color)
            meshes.append({"name": f"{name}/visual_{index}", "geometry": mesh, "group": "Meshes"})
    if not meshes:
        raise ValueError("시각화할 visual 메쉬가 없습니다.")
    return root, transforms, meshes


def show_model(path, root, transforms, meshes, frame_size_mm, show_frames):
    import open3d as o3d

    geometries = list(meshes)
    labels = []
    if show_frames:
        names = list(transforms) if FRAME_NAMES is None else list(FRAME_NAMES)
        if root not in names:
            names.insert(0, root)
        for name in names:
            if name not in transforms:
                continue
            frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=frame_size_mm / 1000)
            frame.transform(transforms[name])
            geometries.append({"name": name, "geometry": frame, "group": "Frames"})
            labels.append((transforms[name][:3, 3], name))

    def add_labels(window):
        for position, name in labels:
            window.add_3d_label(position, name)

    bounds = np.array([item["geometry"].get_axis_aligned_bounding_box().get_box_points()
                       for item in geometries]).reshape(-1, 3)
    center = (bounds.min(axis=0) + bounds.max(axis=0)) / 2
    extent = max(float(np.ptp(bounds, axis=0).max()), 0.01)
    o3d.visualization.draw(
        geometries, title=f"Xacro Viewer - {path.name}",
        width=WINDOW_WIDTH, height=WINDOW_HEIGHT,
        bg_color=BACKGROUND_COLOR, show_skybox=False, show_ui=True,
        lookat=center, eye=center + extent * np.array([1.3, 1.6, 1.0]),
        up=[0, 0, 1], field_of_view=50, on_init=add_labels,
    )


def joint_angle_arg(text):
    name, separator, value = text.partition("=")
    try:
        angle = float(value)
        if not separator or not name.strip() or not np.isfinite(angle):
            raise ValueError
    except ValueError:
        raise argparse.ArgumentTypeError("관절각은 joint_2=-30처럼 이름=각도(deg)로 지정하세요.")
    return name.strip(), angle


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--xacro", type=Path, default=XACRO_PATH, help="표시할 Xacro/URDF 경로")
    parser.add_argument("--joint", action="append", type=joint_angle_arg, default=[],
                        metavar="NAME=DEG", help="회전 관절각 지정. 예: --joint joint_2=-30 (반복 가능)")
    parser.add_argument("--frame-size-mm", type=float, default=FRAME_SIZE_MM)
    parser.add_argument("--no-frames", action="store_true", help="좌표계 숨기기")
    parser.add_argument("--check", action="store_true", help="GUI 없이 메쉬·연결·TF 확인")
    args = parser.parse_args()
    path = args.xacro.expanduser()
    if not path.is_absolute():
        path = Path(__file__).resolve().parent / path
    path = path.resolve()
    try:
        if not path.is_file():
            raise FileNotFoundError(f"Xacro 파일을 찾을 수 없습니다: {path}")
        if not np.isfinite(args.frame_size_mm) or args.frame_size_mm <= 0:
            raise ValueError("좌표계 크기는 0보다 큰 유한한 mm 값이어야 합니다.")
        joint_angles = dict(JOINT_ANGLES_DEG)
        joint_angles.update(args.joint)
        root, transforms, meshes = load_model(path, joint_angles)
        print(f"Xacro: {path}\n기준 좌표계: {root}\n메쉬: {len(meshes)}개", flush=True)
        print(f"표시 관절각(deg): {joint_angles or '{}'} / 미지정 관절은 0도", flush=True)
        for name, transform in transforms.items():
            xyz = ", ".join(f"{value:.3f}" for value in transform[:3, 3] * 1000)
            print(f"  {name}: XYZ [{xyz}] mm")
        if args.check:
            print("검증 완료: 메쉬 로딩과 고정/회전 조인트 TF 계산 성공.")
        else:
            print("좌표축 X=빨강, Y=초록, Z=파랑. 마우스로 회전/확대하고 오른쪽 목록에서 메쉬를 숨길 수 있습니다.", flush=True)
            show_model(path, root, transforms, meshes, args.frame_size_mm, not args.no_frames)
    except ImportError as exc:
        print(f"의존성 로딩 오류: {exc}\n현재 Python 환경에서 python -m pip install xacro numpy open3d trimesh pycollada", file=sys.stderr)
        return 1
    except Exception as exc:
        print(f"시각화 오류: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
