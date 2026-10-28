"""의존성 없는 최소 PLY 리더.

open3d/trimesh를 conda 환경에 추가로 넣지 않기 위해 numpy만으로
binary_little_endian / ascii PLY의 vertex element(x,y,z[,rgb])만 읽는다.
face element는 시각화에 point cloud만 쓰므로 건너뛴다.
"""

import numpy as np

_PLY_NUMPY_TYPE = {
    "char": "i1", "int8": "i1",
    "uchar": "u1", "uint8": "u1",
    "short": "i2", "int16": "i2",
    "ushort": "u2", "uint16": "u2",
    "int": "i4", "int32": "i4",
    "uint": "u4", "uint32": "u4",
    "float": "f4", "float32": "f4",
    "double": "f8", "float64": "f8",
}


def _parse_header(stream):
    """(elements, format, header_end_offset)를 돌려준다."""
    if stream.readline().strip() != b"ply":
        raise ValueError("PLY magic 'ply' 가 없다")
    fmt = None
    elements = []
    while True:
        line = stream.readline()
        if not line:
            raise ValueError("end_header 를 만나기 전에 파일이 끝났다")
        parts = line.split()
        if not parts:
            continue
        keyword = parts[0]
        if keyword == b"format":
            fmt = parts[1].decode()
        elif keyword == b"comment":
            continue
        elif keyword == b"element":
            elements.append({"name": parts[1].decode(), "count": int(parts[2]), "properties": []})
        elif keyword == b"property":
            if not elements:
                raise ValueError("element 선언 없이 property가 나왔다")
            if parts[1] == b"list":
                elements[-1]["properties"].append(
                    {"list": True,
                     "count_type": parts[2].decode(),
                     "item_type": parts[3].decode(),
                     "name": parts[4].decode()}
                )
            else:
                elements[-1]["properties"].append(
                    {"list": False, "type": parts[1].decode(), "name": parts[2].decode()}
                )
        elif keyword == b"end_header":
            return elements, fmt, stream.tell()


def _vertex_dtype(properties, byte_order):
    fields = []
    for prop in properties:
        if prop["list"]:
            raise ValueError("vertex element의 list property는 지원하지 않는다")
        fields.append((prop["name"], byte_order + _PLY_NUMPY_TYPE[prop["type"]]))
    return np.dtype(fields)


def read_ply_points(path, max_points=0):
    """PLY vertex를 (points Nx3 float32, colors Nx3 uint8 또는 None)로 읽는다.

    max_points > 0 이면 균등 간격으로 다운샘플링한다(RViz 부하 조절용).
    """
    with open(path, "rb") as stream:
        elements, fmt, offset = _parse_header(stream)
        vertex = next((e for e in elements if e["name"] == "vertex"), None)
        if vertex is None:
            raise ValueError(f"vertex element가 없다: {path}")

        if fmt == "ascii":
            names = [p["name"] for p in vertex["properties"]]
            raw = np.loadtxt(stream, max_rows=vertex["count"])
            raw = np.atleast_2d(raw)
            columns = {name: raw[:, i] for i, name in enumerate(names)}
        else:
            byte_order = "<" if fmt == "binary_little_endian" else ">"
            dtype = _vertex_dtype(vertex["properties"], byte_order)
            stream.seek(offset)
            raw = np.fromfile(stream, dtype=dtype, count=vertex["count"])
            if raw.size != vertex["count"]:
                raise ValueError(f"vertex를 {vertex['count']}개 읽지 못했다: {path}")
            columns = {name: raw[name] for name in dtype.names}

    missing = [axis for axis in ("x", "y", "z") if axis not in columns]
    if missing:
        raise ValueError(f"vertex에 {', '.join(missing)} property가 없다: {path}")

    points = np.column_stack([columns["x"], columns["y"], columns["z"]]).astype(np.float32)

    colors = None
    if all(channel in columns for channel in ("red", "green", "blue")):
        colors = np.column_stack(
            [columns["red"], columns["green"], columns["blue"]]
        ).astype(np.uint8)

    if max_points and len(points) > max_points:
        step = int(np.ceil(len(points) / max_points))
        points = points[::step]
        if colors is not None:
            colors = colors[::step]
    return points, colors


def read_csv_points(path, max_points=0):
    """x,y,z 헤더를 가진 표면 점군 CSV를 읽는다."""
    points = np.loadtxt(path, delimiter=",", skiprows=1, usecols=(0, 1, 2)).astype(np.float32)
    points = np.atleast_2d(points)
    if max_points and len(points) > max_points:
        step = int(np.ceil(len(points) / max_points))
        points = points[::step]
    return points, None


def read_surface_points(path, max_points=0):
    """확장자를 보고 PLY/CSV 로더를 고른다."""
    suffix = str(path).lower()
    if suffix.endswith(".ply"):
        return read_ply_points(path, max_points=max_points)
    if suffix.endswith(".csv"):
        return read_csv_points(path, max_points=max_points)
    raise ValueError(f"지원하지 않는 표면 데이터 확장자: {path}")
