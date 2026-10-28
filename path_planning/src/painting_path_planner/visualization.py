#!/usr/bin/env python3

import argparse
import csv
import json
import math
import tempfile
import webbrowser
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import open3d as o3d
import plotly.graph_objects as go


PLANNER_ROOT = Path(__file__).resolve().parents[2]
DATA_DIR = PLANNER_ROOT / "test" / "data"
CSV_DIR = PLANNER_ROOT / "test" / "csv"
REFERENCE_DIR = DATA_DIR / "reference_path_requirements"

DEFAULT_INPUT_PATHS = (
    # REFERENCE_DIR / "right_rear_door" / "metallic_default" / "model-final-new.ply",
    # REFERENCE_DIR / "right_rear_door" / "metallic_default" / "right_rear_door_1_WaterBasedFullWetBaseCoat.10",
    # REFERENCE_DIR / "right_rear_door" / "metallic_default" / "right_rear_door_1_WaterBasedFullWetClearCoat.12",
    # REFERENCE_DIR / "right_rear_door" / "metallic_default" / "right_rear_door_1_WaterBasedSemiWetClearCoat.13",

    # REFERENCE_DIR / "right_rear_door" / "metallic_wet" / "model-final-new.ply",
    # REFERENCE_DIR / "right_rear_door" / "metallic_wet" / "right_rear_door_1_WaterBasedFullWetBaseCoat.10",
    # REFERENCE_DIR / "right_rear_door" / "metallic_wet" / "right_rear_door_1_WaterBasedFullWetClearCoat.12",
    # REFERENCE_DIR / "right_rear_door" / "metallic_wet" / "right_rear_door_1_WaterBasedSemiWetClearCoat.13",

    # REFERENCE_DIR / "right_rear_door" / "solid_pearl_default" / "model-final-new.ply",
    # REFERENCE_DIR / "right_rear_door" / "solid_pearl_default" / "right_rear_door_1_WaterBasedFullWetBaseCoat.10",
    # REFERENCE_DIR / "right_rear_door" / "solid_pearl_default" / "right_rear_door_1_WaterBasedFullWetClearCoat.12",
    # REFERENCE_DIR / "right_rear_door" / "solid_pearl_default" / "right_rear_door_1_WaterBasedSemiWetClearCoat.13",
    # REFERENCE_DIR / "right_rear_door" / "solid_pearl_default" / "right_rear_door_1_WaterBasedMistBaseCoat.14",

    # REFERENCE_DIR / "right_rear_door" / "solid_pearl_wet" / "model-final-new.ply",
    # REFERENCE_DIR / "right_rear_door" / "solid_pearl_wet" / "right_rear_door_1_WaterBasedFullWetBaseCoat.10",
    # REFERENCE_DIR / "right_rear_door" / "solid_pearl_wet" / "right_rear_door_1_WaterBasedFullWetClearCoat.12",
    # REFERENCE_DIR / "right_rear_door" / "solid_pearl_wet" / "right_rear_door_1_WaterBasedSemiWetClearCoat.13",
    # REFERENCE_DIR / "right_rear_door" / "solid_pearl_wet" / "right_rear_door_1_WaterBasedMistBaseCoat.14",



    REFERENCE_DIR / "trunk" / "ES300h rear hood.ply",
    REFERENCE_DIR / "trunk" / "rear_hood_1_WaterBasedFullWetBaseCoat.10",
    REFERENCE_DIR / "trunk" / "rear_hood_1_WaterBasedFullWetClearCoat.12",
    REFERENCE_DIR / "trunk" / "rear_hood_1_WaterBasedMistBaseCoat.14",
)

FILE_COLOR_PALETTE = (
    "rgb(242, 51, 41)",
    "rgb(26, 158, 242)",
    "rgb(26, 199, 92)",
    "rgb(242, 184, 31)",
    "rgb(184, 97, 242)",
    "rgb(242, 107, 194)",
    "rgb(31, 209, 209)",
)

POSITION_COLUMN_CANDIDATES = (
    ("x", "y", "z"),
    ("position_x", "position_y", "position_z"),
    ("px", "py", "pz"),
)

RGB_COLUMN_CANDIDATES = (
    ("r", "g", "b"),
    ("red", "green", "blue"),
)

JSON_SUFFIXES = (".json", ".10", ".12", ".13", ".14")


@dataclass(slots=True)
class LoadedData:
    path: Path
    file_type: str
    points: np.ndarray
    colors: list[str]
    skipped_rows: int = 0
    row_ids: list[str] | None = None


def _normalized_field_map(fieldnames: list[str]) -> dict[str, str]:
    return {field.strip().lower(): field for field in fieldnames if field is not None}


def _find_columns(
    fieldnames: list[str],
    candidates: tuple[tuple[str, str, str], ...],
) -> tuple[str, str, str] | None:
    normalized = _normalized_field_map(fieldnames)
    for candidate in candidates:
        if all(name in normalized for name in candidate):
            return tuple(normalized[name] for name in candidate)
    return None


def _float_or_none(value: str | None) -> float | None:
    if value is None:
        return None
    text = value.strip()
    if not text:
        return None
    try:
        number = float(text)
    except ValueError:
        return None
    if not math.isfinite(number):
        return None
    return number


def _bool_from_text(value: str | None) -> bool | None:
    if value is None:
        return None
    text = value.strip().lower()
    if text in ("1", "true", "t", "yes", "y"):
        return True
    if text in ("0", "false", "f", "no", "n"):
        return False
    return None


def _rgb_to_255(value: float) -> int:
    if value <= 1.0:
        value *= 255.0
    return int(np.clip(round(value), 0, 255))


def _rgb_string(values) -> str:
    arr = np.asarray(values, dtype=float).reshape(3)
    return f"rgb({_rgb_to_255(arr[0])}, {_rgb_to_255(arr[1])}, {_rgb_to_255(arr[2])})"


def _path_color(path_value: str) -> str:
    try:
        index = int(float(path_value))
    except (TypeError, ValueError):
        index = abs(hash(path_value)) % 360

    hue = (index * 0.61803398875) % 1.0
    saturation = 0.75
    value = 0.95

    i = int(hue * 6.0)
    f = hue * 6.0 - i
    p = value * (1.0 - saturation)
    q = value * (1.0 - f * saturation)
    t = value * (1.0 - (1.0 - f) * saturation)
    i %= 6

    if i == 0:
        rgb = (value, t, p)
    elif i == 1:
        rgb = (q, value, p)
    elif i == 2:
        rgb = (p, value, t)
    elif i == 3:
        rgb = (p, q, value)
    elif i == 4:
        rgb = (t, p, value)
    else:
        rgb = (value, p, q)
    return _rgb_string(rgb)


def load_csv_data(csv_path: Path) -> LoadedData:
    with csv_path.open("r", newline="", encoding="utf-8-sig") as csv_file:
        reader = csv.DictReader(csv_file)
        if not reader.fieldnames:
            raise ValueError("CSV 헤더가 없습니다.")

        position_columns = _find_columns(reader.fieldnames, POSITION_COLUMN_CANDIDATES)
        if position_columns is None:
            raise ValueError(
                "CSV에서 좌표 컬럼을 찾지 못했습니다. "
                "x,y,z 또는 position_x,position_y,position_z 컬럼이 필요합니다."
            )

        rgb_columns = _find_columns(reader.fieldnames, RGB_COLUMN_CANDIDATES)
        normalized = _normalized_field_map(reader.fieldnames)
        path_col = normalized.get("path_index") or normalized.get("row_index")
        paint_col = normalized.get("paint")

        points: list[list[float]] = []
        colors: list[str] = []
        row_ids: list[str] = []
        skipped_rows = 0

        x_col, y_col, z_col = position_columns
        for row in reader:
            x = _float_or_none(row.get(x_col))
            y = _float_or_none(row.get(y_col))
            z = _float_or_none(row.get(z_col))
            if x is None or y is None or z is None:
                skipped_rows += 1
                continue

            points.append([x, y, z])
            if path_col is not None:
                row_ids.append(str(row.get(path_col, "")))

            color = None
            if paint_col is not None:
                paint = _bool_from_text(row.get(paint_col))
                if paint is True:
                    color = "rgb(0, 255, 0)"
                elif paint is False:
                    color = "rgb(90, 96, 106)"

            if color is None and path_col is not None:
                color = _path_color(row.get(path_col, "0"))

            if color is None and rgb_columns is not None:
                r_col, g_col, b_col = rgb_columns
                r = _float_or_none(row.get(r_col))
                g = _float_or_none(row.get(g_col))
                b = _float_or_none(row.get(b_col))
                if r is not None and g is not None and b is not None:
                    color = f"rgb({_rgb_to_255(r)}, {_rgb_to_255(g)}, {_rgb_to_255(b)})"

            colors.append(color or "rgb(0, 184, 140)")

    if not points:
        raise ValueError("시각화할 수 있는 좌표 데이터가 없습니다.")

    return LoadedData(
        path=csv_path,
        file_type="csv",
        points=np.asarray(points, dtype=np.float64),
        colors=colors,
        skipped_rows=skipped_rows,
        row_ids=row_ids if path_col is not None else None,
    )


def load_ply_data(ply_path: Path) -> LoadedData:
    point_cloud = o3d.io.read_point_cloud(str(ply_path))
    if point_cloud.is_empty():
        raise ValueError(f"{ply_path}에서 포인트 클라우드를 읽지 못했습니다.")

    points = np.asarray(point_cloud.points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) == 0:
        raise ValueError(f"{ply_path} must contain vertex x,y,z data")

    if point_cloud.has_colors():
        colors = [_rgb_string(color) for color in np.asarray(point_cloud.colors)]
    else:
        colors = ["rgb(0, 184, 140)"] * len(points)

    return LoadedData(
        path=ply_path,
        file_type="ply",
        points=points,
        colors=colors,
    )


def _read_json_file(json_path: Path):
    try:
        return json.loads(json_path.read_text(encoding="utf-8"))
    except UnicodeDecodeError:
        return json.loads(json_path.read_text(encoding="cp949"))


def _json_point_color(point: dict) -> str:
    is_swift = point.get("is_swift")
    try:
        is_swift_enabled = bool(int(is_swift))
    except (TypeError, ValueError):
        is_swift_enabled = bool(is_swift)
    if is_swift_enabled:
        return "rgb(255, 0, 255)"

    trigger = str(point.get("gun_trigger", "")).strip().lower()
    if trigger == "open":
        return "rgb(0, 255, 0)"
    if trigger == "close":
        return "rgb(255, 80, 40)"

    speed = point.get("speed")
    try:
        if speed is not None and float(speed) > 0.0:
            return "rgb(35, 180, 255)"
    except (TypeError, ValueError):
        pass
    return "rgb(110, 116, 128)"


def load_json_data(json_path: Path) -> LoadedData:
    raw = _read_json_file(json_path)
    if isinstance(raw, dict):
        trace = raw.get("trace")
    elif isinstance(raw, list):
        trace = raw
    else:
        trace = None

    if not isinstance(trace, list):
        raise ValueError("JSON에서 trace 배열을 찾지 못했습니다.")

    points: list[list[float]] = []
    colors: list[str] = []
    row_ids: list[str] = []
    skipped_rows = 0

    for point in trace:
        if not isinstance(point, dict):
            skipped_rows += 1
            continue

        pos = point.get("pos")
        if not isinstance(pos, (list, tuple)) or len(pos) < 3:
            skipped_rows += 1
            continue

        xyz = []
        for value in pos[:3]:
            try:
                number = float(value)
            except (TypeError, ValueError):
                number = math.nan
            xyz.append(number)

        if not all(math.isfinite(value) for value in xyz):
            skipped_rows += 1
            continue

        points.append(xyz)
        row_ids.append(str(point.get("row_ind", 0)))
        colors.append(_json_point_color(point))

    if not points:
        raise ValueError("JSON에서 시각화할 수 있는 pos 좌표가 없습니다.")

    return LoadedData(
        path=json_path,
        file_type="json",
        points=np.asarray(points, dtype=np.float64),
        colors=colors,
        skipped_rows=skipped_rows,
        row_ids=row_ids,
    )


def load_data(input_path: Path) -> LoadedData:
    suffix = input_path.suffix.lower()
    if suffix == ".csv":
        return load_csv_data(input_path)
    if suffix == ".ply":
        return load_ply_data(input_path)
    if suffix in JSON_SUFFIXES:
        return load_json_data(input_path)
    raise ValueError(
        f"지원하지 않는 파일 형식입니다: {suffix}. CSV, PLY, JSON(.json/.10/.12/.13/.14)만 지원합니다."
    )


def _voxel_downsample(data: LoadedData, voxel_size: float) -> LoadedData:
    if voxel_size <= 0.0:
        return data

    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(data.points)
    point_cloud = point_cloud.voxel_down_sample(float(voxel_size))
    points = np.asarray(point_cloud.points, dtype=np.float64)
    if len(points) == 0:
        raise ValueError(f"voxel_size={voxel_size} 적용 후 포인트가 없어졌습니다.")

    # Downsampling changes point order and drops metadata, so use one neutral color.
    return LoadedData(
        path=data.path,
        file_type=data.file_type,
        points=points,
        colors=["rgb(0, 184, 140)"] * len(points),
        skipped_rows=data.skipped_rows,
        row_ids=None,
    )


def _paint_uniform_color(data: LoadedData, color: str) -> LoadedData:
    return LoadedData(
        path=data.path,
        file_type=data.file_type,
        points=data.points,
        colors=[color] * len(data.points),
        skipped_rows=data.skipped_rows,
        row_ids=data.row_ids,
    )


def _print_summary(data: LoadedData) -> None:
    points = data.points
    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    center = (mins + maxs) * 0.5
    span = maxs - mins

    print(f"[surface_data_debugger] file={data.path}")
    print(
        f"[surface_data_debugger] type={data.file_type} "
        f"points={len(points)} skipped_rows={data.skipped_rows} "
        f"has_color={bool(data.colors)}"
    )
    print(
        "[surface_data_debugger] bounds "
        f"x=[{mins[0]:.3f}, {maxs[0]:.3f}] "
        f"y=[{mins[1]:.3f}, {maxs[1]:.3f}] "
        f"z=[{mins[2]:.3f}, {maxs[2]:.3f}]"
    )
    print(
        "[surface_data_debugger] center "
        f"({center[0]:.3f}, {center[1]:.3f}, {center[2]:.3f}) "
        f"span=({span[0]:.3f}, {span[1]:.3f}, {span[2]:.3f})"
    )


def _line_segments_by_row(data: LoadedData) -> tuple[list, list, list]:
    if data.row_ids is None or len(data.row_ids) != len(data.points):
        return [], [], []

    xs, ys, zs = [], [], []
    current_row = None
    for point, row_id in zip(data.points, data.row_ids):
        if current_row is not None and row_id != current_row:
            xs.append(None)
            ys.append(None)
            zs.append(None)
        current_row = row_id
        xs.append(float(point[0]))
        ys.append(float(point[1]))
        zs.append(float(point[2]))
    return xs, ys, zs


def _add_axis_traces(fig: go.Figure, axis_size: float) -> None:
    size = float(axis_size)
    if size <= 0.0:
        return
    axes = (
        ("X axis", [0, size], [0, 0], [0, 0], "rgb(240, 80, 80)"),
        ("Y axis", [0, 0], [0, size], [0, 0], "rgb(80, 220, 120)"),
        ("Z axis", [0, 0], [0, 0], [0, size], "rgb(100, 150, 255)"),
    )
    for name, x, y, z, color in axes:
        fig.add_trace(
            go.Scatter3d(
                x=x,
                y=y,
                z=z,
                mode="lines",
                name=name,
                line={"color": color, "width": 5},
                showlegend=False,
            )
        )


def write_plotly_html(
    loaded: list[LoadedData],
    output_path: Path,
    point_size: float,
    line_width: float,
    show_axis: bool,
    axis_size: float,
) -> Path:
    fig = go.Figure()

    for index, data in enumerate(loaded):
        points = data.points
        name = f"{index + 1}. {data.path.name} ({len(points)})"
        fig.add_trace(
            go.Scatter3d(
                x=points[:, 0],
                y=points[:, 1],
                z=points[:, 2],
                mode="markers",
                name=name,
                marker={
                    "size": point_size,
                    "color": data.colors,
                    "opacity": 0.85,
                },
            )
        )

        xs, ys, zs = _line_segments_by_row(data)
        if xs:
            fig.add_trace(
                go.Scatter3d(
                    x=xs,
                    y=ys,
                    z=zs,
                    mode="lines",
                    name=f"{index + 1}. {data.path.name} lines",
                    line={
                        "color": FILE_COLOR_PALETTE[index % len(FILE_COLOR_PALETTE)],
                        "width": line_width,
                    },
                )
            )

    if show_axis:
        _add_axis_traces(fig, axis_size)

    axis_style = {
        "showbackground": False,
        "showgrid": False,
        "showline": True,
        "showspikes": False,
        "zeroline": False,
        "color": "rgb(160, 190, 230)",
        "tickfont": {"color": "rgb(120, 160, 210)"},
        "linecolor": "rgba(160, 190, 230, 0.35)",
    }
    fig.update_layout(
        title="surface_data_debugger",
        scene={
            "aspectmode": "data",
            "bgcolor": "rgb(24, 26, 30)",
            "xaxis": {"title": "X", **axis_style},
            "yaxis": {"title": "Y", **axis_style},
            "zaxis": {"title": "Z", **axis_style},
        },
        legend={
            "itemsizing": "constant",
            "x": 0.02,
            "y": 0.98,
            "bgcolor": "rgba(20, 20, 20, 0.45)",
            "font": {"color": "white"},
        },
        paper_bgcolor="rgb(24, 26, 30)",
        plot_bgcolor="rgb(24, 26, 30)",
        margin={"l": 0, "r": 0, "t": 42, "b": 0},
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.write_html(
        output_path,
        include_plotlyjs=True,
        full_html=True,
        config={"displaylogo": False},
    )
    return output_path


DEBUG_HTML_NAME = "trajectory_debug.html"
DEBUG_DIRECTION_MAX_SEGMENTS = 1000
DEBUG_DIRECTION_LENGTH = 0.05
DEBUG_SLICE_PLANE_STRIDE = 1
DEBUG_SLICE_PLANE_OPACITY = 0.18
DEBUG_SLICE_PLANE_VISIBLE = "legendonly"


def load_csv_surface_data(csv_path: Path) -> tuple[np.ndarray, list[str]]:
    points = np.loadtxt(csv_path, delimiter=",", skiprows=1, usecols=(0, 1, 2))
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f"{csv_path} must contain x,y,z columns")
    return points, ["rgb(0, 184, 140)"] * len(points)


def _surface_rgb_string(values) -> str:
    rgb = np.asarray(values, dtype=float).reshape(3)
    if np.max(rgb) <= 1.0:
        rgb = rgb * 255.0
    rgb = np.clip(np.rint(rgb), 0, 255).astype(int)
    return f"rgb({rgb[0]}, {rgb[1]}, {rgb[2]})"


def load_ply_surface_data(ply_path: Path) -> tuple[np.ndarray, list[str]]:
    point_cloud = o3d.io.read_point_cloud(str(ply_path))
    if point_cloud.is_empty():
        raise ValueError(f"{ply_path}에서 포인트 클라우드를 읽지 못했습니다.")

    points = np.asarray(point_cloud.points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f"{ply_path} must contain vertex x,y,z data")
    if point_cloud.has_colors():
        colors = [_surface_rgb_string(color) for color in np.asarray(point_cloud.colors)]
    else:
        colors = ["rgb(0, 184, 140)"] * len(points)
    return points, colors


def load_surface_data(input_path: Path) -> tuple[np.ndarray, list[str]]:
    suffix = input_path.suffix.lower()
    if suffix == ".csv":
        return load_csv_surface_data(input_path)
    if suffix == ".ply":
        return load_ply_surface_data(input_path)
    raise ValueError(f"지원하지 않는 표면 데이터 형식입니다: {suffix}. CSV 또는 PLY만 지원합니다.")


def _points_array(values) -> np.ndarray:
    points = np.asarray(values, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) == 0:
        return np.empty((0, 3), dtype=float)
    return points


def _rows_to_line_xyz(rows) -> tuple[list, list, list]:
    xs, ys, zs = [], [], []
    for row in rows or []:
        points = _points_array(row)
        if len(points) == 0:
            continue
        xs.extend(points[:, 0].tolist())
        ys.extend(points[:, 1].tolist())
        zs.extend(points[:, 2].tolist())
        xs.append(None)
        ys.append(None)
        zs.append(None)
    return xs, ys, zs


def _masked_points(rows, mask_rows) -> np.ndarray:
    selected = []
    for row, mask in zip(rows or [], mask_rows or []):
        points = _points_array(row)
        mask = np.asarray(mask, dtype=bool).reshape(-1)
        if len(points) == 0 or len(mask) != len(points):
            continue
        selected.append(points[mask])
    if not selected:
        return np.empty((0, 3), dtype=float)
    return np.vstack(selected)


def _direction_segments(position_rows, direction_rows) -> tuple[list, list, list]:
    segments = []
    for positions, directions in zip(position_rows or [], direction_rows or []):
        points = _points_array(positions)
        dirs = _points_array(directions)
        if len(points) == 0 or len(points) != len(dirs):
            continue
        segments.extend(zip(points, dirs))

    if not segments:
        return [], [], []
    if len(segments) > DEBUG_DIRECTION_MAX_SEGMENTS:
        indices = np.linspace(0, len(segments) - 1, DEBUG_DIRECTION_MAX_SEGMENTS, dtype=int)
        segments = [segments[index] for index in indices]

    xs, ys, zs = [], [], []
    for point, direction in segments:
        direction_norm = np.linalg.norm(direction)
        if direction_norm <= 1e-12:
            continue
        end = point + DEBUG_DIRECTION_LENGTH * direction / direction_norm
        xs.extend([point[0], end[0], None])
        ys.extend([point[1], end[1], None])
        zs.extend([point[2], end[2], None])
    return xs, ys, zs


def _add_marker_trace(
    fig: go.Figure,
    name: str,
    points,
    color,
    size: float,
    opacity: float = 1.0,
    visible=True,
) -> None:
    points = _points_array(points)
    if len(points) == 0:
        return
    fig.add_trace(
        go.Scatter3d(
            x=points[:, 0],
            y=points[:, 1],
            z=points[:, 2],
            mode="markers",
            name=name,
            marker={"size": size, "color": color, "opacity": opacity},
            visible=visible,
        )
    )


def _add_line_trace(
    fig: go.Figure,
    name: str,
    rows,
    color: str,
    width: float,
    visible=True,
) -> None:
    xs, ys, zs = _rows_to_line_xyz(rows)
    if not xs:
        return
    fig.add_trace(
        go.Scatter3d(
            x=xs,
            y=ys,
            z=zs,
            mode="lines",
            name=name,
            line={"color": color, "width": width},
            visible=visible,
        )
    )


def _add_slice_plane_mesh(
    fig: go.Figure,
    slice_profiles,
    point_cloud_xyz: np.ndarray,
    visible=True,
) -> None:
    profiles = list(slice_profiles or [])
    cloud = _points_array(point_cloud_xyz)
    if not profiles or len(cloud) == 0:
        return

    xs, ys, zs = [], [], []
    tri_i, tri_j, tri_k = [], [], []
    vertex_offset = 0

    for profile in profiles[:: max(int(DEBUG_SLICE_PLANE_STRIDE), 1)]:
        origin = np.asarray(profile.get("slice_plane_origin_world", []), dtype=float).reshape(-1)
        row_axis = np.asarray(profile.get("slice_plane_row_axis_world", []), dtype=float).reshape(-1)
        other_axis = np.asarray(profile.get("slice_plane_other_axis_world", []), dtype=float).reshape(-1)
        if origin.size != 3 or row_axis.size != 3 or other_axis.size != 3:
            continue

        row_axis = row_axis / max(float(np.linalg.norm(row_axis)), 1e-12)
        other_axis = other_axis / max(float(np.linalg.norm(other_axis)), 1e-12)
        if not np.isfinite(origin).all() or not np.isfinite(row_axis).all() or not np.isfinite(other_axis).all():
            continue

        slice_points = _points_array(profile.get("slice_points_world", []))
        row_source = slice_points if len(slice_points) >= 2 else cloud
        row_values = row_source @ row_axis
        other_values = cloud @ other_axis
        row_half = max(0.5 * float(np.ptp(row_values)), 0.03)
        other_half = max(0.55 * float(np.ptp(other_values)), 0.05)

        corners = np.asarray(
            [
                origin - row_half * row_axis - other_half * other_axis,
                origin + row_half * row_axis - other_half * other_axis,
                origin + row_half * row_axis + other_half * other_axis,
                origin - row_half * row_axis + other_half * other_axis,
            ],
            dtype=float,
        )
        xs.extend(corners[:, 0].tolist())
        ys.extend(corners[:, 1].tolist())
        zs.extend(corners[:, 2].tolist())
        tri_i.extend([vertex_offset, vertex_offset])
        tri_j.extend([vertex_offset + 1, vertex_offset + 2])
        tri_k.extend([vertex_offset + 2, vertex_offset + 3])
        vertex_offset += 4

    if not xs:
        return

    fig.add_trace(
        go.Mesh3d(
            x=xs,
            y=ys,
            z=zs,
            i=tri_i,
            j=tri_j,
            k=tri_k,
            name="local slicing planes",
            color="rgb(255, 210, 40)",
            opacity=DEBUG_SLICE_PLANE_OPACITY,
            visible=visible,
            showlegend=True,
            hoverinfo="skip",
        )
    )


def write_debug_plotly_html(
    case: dict,
    point_cloud_xyz: np.ndarray,
    surface_colors: list[str],
    result: dict,
) -> Path:
    fig = go.Figure()

    _add_marker_trace(
        fig,
        name=f"surface points ({len(point_cloud_xyz)})",
        points=point_cloud_xyz,
        color=surface_colors,
        size=2.0,
        opacity=0.85,
    )

    slice_points = [
        profile.get("slice_points_world", [])
        for profile in result.get("slice_profiles", [])
    ]
    guide_curve_rows = []
    guide_hull_rows = []
    guide_strip_points = []
    seen_guide = set()
    seen_guide_hull = set()
    seen_guide_strip = set()
    for profile in result.get("slice_profiles", []):
        guide = _points_array(profile.get("guide_curve_points_world", []))
        if len(guide) > 0:
            key = tuple(np.round(guide.reshape(-1), 6).tolist())
            if key not in seen_guide:
                seen_guide.add(key)
                guide_curve_rows.append(guide)

        guide_hull = _points_array(profile.get("guide_hull_points_world", []))
        if len(guide_hull) > 0:
            key = tuple(np.round(guide_hull.reshape(-1), 6).tolist())
            if key not in seen_guide_hull:
                seen_guide_hull.add(key)
                guide_hull_rows.append(guide_hull)

        guide_strip = _points_array(profile.get("guide_strip_points_world", []))
        if len(guide_strip) > 0:
            key = tuple(np.round(guide_strip.reshape(-1), 6).tolist())
            if key not in seen_guide_strip:
                seen_guide_strip.add(key)
                guide_strip_points.append(guide_strip)

    _add_marker_trace(
        fig,
        name="slice raw points",
        points=np.vstack([_points_array(row) for row in slice_points if len(_points_array(row)) > 0])
        if slice_points
        else np.empty((0, 3)),
        color="rgba(255, 70, 210, 0.95)",
        size=2.1,
        opacity=0.95,
        visible=True,
    )
    _add_line_trace(
        fig,
        name="guide curve",
        rows=guide_curve_rows,
        color="rgb(255, 120, 0)",
        width=6.0,
        visible=True,
    )
    _add_line_trace(
        fig,
        name="guide upper hull",
        rows=guide_hull_rows,
        color="rgb(255, 230, 0)",
        width=4.0,
        visible="legendonly",
    )
    _add_marker_trace(
        fig,
        name="guide strip points",
        points=np.vstack(guide_strip_points) if guide_strip_points else np.empty((0, 3)),
        color="rgba(255, 170, 0, 0.8)",
        size=2.2,
        opacity=0.8,
        visible="legendonly",
    )

    _add_slice_plane_mesh(
        fig,
        result.get("slice_profiles", []),
        point_cloud_xyz,
        visible=DEBUG_SLICE_PLANE_VISIBLE,
    )

    _add_line_trace(
        fig,
        name="correction components",
        rows=result.get("correction_observed_path_rows_world", []),
        color="rgba(160, 160, 160, 0.75)",
        width=2.0,
        visible="legendonly",
    )
    _add_line_trace(
        fig,
        name="correction connected component",
        rows=result.get("correction_connected_component_rows_world", []),
        color="rgba(80, 255, 180, 0.95)",
        width=4.0,
        visible="legendonly",
    )
    _add_line_trace(
        fig,
        name="correction convex hull",
        rows=result.get("correction_convex_hull_rows_world", []),
        color="rgba(255, 255, 255, 0.70)",
        width=3.0,
        visible="legendonly",
    )
    _add_line_trace(
        fig,
        name="correction selected hull",
        rows=result.get("correction_selected_hull_rows_world", []),
        color="rgba(255, 120, 60, 0.85)",
        width=3.0,
        visible="legendonly",
    )
    _add_marker_trace(
        fig,
        name="correction supported samples",
        points=np.vstack(
            [
                _points_array(row)
                for row in result.get("correction_supported_sample_rows_world", [])
                if len(_points_array(row)) > 0
            ]
        )
        if result.get("correction_supported_sample_rows_world", [])
        else np.empty((0, 3)),
        color="cyan",
        size=2.0,
        visible="legendonly",
    )
    _add_marker_trace(
        fig,
        name="correction endpoints",
        points=result.get("correction_endpoint_points_world", []),
        color="red",
        size=5.0,
    )
    _add_line_trace(
        fig,
        name="corrected rows",
        rows=result.get("corrected_rows", []),
        color="deepskyblue",
        width=3.0,
    )
    _add_line_trace(
        fig,
        name="offset rows",
        rows=result.get("offset_rows", []),
        color="orange",
        width=3.0,
    )
    _add_marker_trace(
        fig,
        name="extension points",
        points=result.get("paint_spline_extension_points_world", []),
        color="purple",
        size=2.4,
        visible="legendonly",
    )
    _add_line_trace(
        fig,
        name="paint trajectory",
        rows=result.get("paint_trajectory_position_rows", []),
        color="yellow",
        width=4.0,
    )
    _add_marker_trace(
        fig,
        name="paint ON points",
        points=_masked_points(
            result.get("paint_trajectory_position_rows", []),
            result.get("paint_trajectory_paint_mask_rows", []),
        ),
        color="lime",
        size=2.4,
    )

    dir_x, dir_y, dir_z = _direction_segments(
        result.get("paint_trajectory_position_rows", []),
        result.get("paint_spline_direction_rows_world", []),
    )
    if dir_x:
        fig.add_trace(
            go.Scatter3d(
                x=dir_x,
                y=dir_y,
                z=dir_z,
                mode="lines",
                name="spray directions",
                line={"color": "magenta", "width": 2.0},
                visible="legendonly",
            )
        )

    axis_style = {
        "showbackground": False,
        "showgrid": False,
        "showline": True,
        "showspikes": False,
        "zeroline": False,
        "color": "rgb(160, 190, 230)",
        "tickfont": {"color": "rgb(120, 160, 210)"},
        "linecolor": "rgba(160, 190, 230, 0.35)",
    }

    fig.update_layout(
        title=f"{case['name']} painting trajectory debug",
        scene={
            "aspectmode": "data",
            "bgcolor": "rgb(24, 26, 30)",
            "xaxis": {"title": "X", **axis_style},
            "yaxis": {"title": "Y", **axis_style},
            "zaxis": {"title": "Z", **axis_style},
        },
        legend={
            "itemsizing": "constant",
            "x": 0.02,
            "y": 0.98,
            "bgcolor": "rgba(20, 20, 20, 0.45)",
            "font": {"color": "white"},
        },
        paper_bgcolor="rgb(24, 26, 30)",
        plot_bgcolor="rgb(24, 26, 30)",
        margin={"l": 0, "r": 0, "t": 42, "b": 0},
    )

    html_path = case["output_dir"] / DEBUG_HTML_NAME
    fig.write_html(
        html_path,
        include_plotlyjs=True,
        full_html=True,
        config={"displaylogo": False},
    )
    return html_path



def _default_output_path(input_paths: list[Path]) -> Path:
    if len(input_paths) == 1:
        return Path(tempfile.gettempdir()) / f"{input_paths[0].stem}_data_checker.html"
    return Path(tempfile.gettempdir()) / "data_checker_view.html"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="CSV/PLY/JSON 포인트 데이터를 Plotly HTML로 확인합니다."
    )
    parser.add_argument(
        "input_paths",
        nargs="*",
        type=Path,
        help=(
            "시각화할 CSV/PLY/JSON 파일 경로들. 생략하면 기본값 사용: "
            + ", ".join(str(path) for path in DEFAULT_INPUT_PATHS)
        ),
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="저장할 HTML 경로. 생략하면 /tmp에 저장합니다.",
    )
    parser.add_argument(
        "--open",
        action="store_true",
        help="HTML 생성 후 브라우저로 엽니다.",
    )
    parser.add_argument(
        "--voxel-size",
        type=float,
        default=0.0,
        help="0보다 크면 voxel downsample을 적용합니다. 좌표 단위 기준입니다.",
    )
    parser.add_argument(
        "--point-size",
        type=float,
        default=2.0,
        help="Plotly에서 표시할 포인트 크기입니다.",
    )
    parser.add_argument(
        "--line-width",
        type=float,
        default=4.0,
        help="row_index/path_index가 있는 CSV line trace 두께입니다.",
    )
    parser.add_argument(
        "--axis-size",
        type=float,
        default=1.0,
        help="원점 좌표축 크기입니다.",
    )
    parser.add_argument(
        "--no-axis",
        action="store_true",
        help="원점 좌표축을 표시하지 않습니다.",
    )
    parser.add_argument(
        "--color-by-file",
        action="store_true",
        help="파일별로 강제 색상을 입힙니다. 원본 색상/paint 색상은 무시됩니다.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    input_paths = args.input_paths if args.input_paths else list(DEFAULT_INPUT_PATHS)
    input_paths = [input_path.expanduser().resolve() for input_path in input_paths]

    loaded = []
    for index, input_path in enumerate(input_paths):
        if not input_path.exists():
            raise FileNotFoundError(input_path)

        data = load_data(input_path)
        data = _voxel_downsample(data, args.voxel_size)
        if args.color_by_file:
            color = FILE_COLOR_PALETTE[index % len(FILE_COLOR_PALETTE)]
            data = _paint_uniform_color(data, color)
            print(f"[data_checker] file_color #{index + 1} {color}")
        _print_summary(data)
        loaded.append(data)

    output_path = args.output.expanduser().resolve() if args.output else _default_output_path(input_paths)
    html_path = write_plotly_html(
        loaded=loaded,
        output_path=output_path,
        point_size=args.point_size,
        line_width=args.line_width,
        show_axis=not args.no_axis,
        axis_size=args.axis_size,
    )
    print(f"[data_checker] loaded_files={len(loaded)}")
    print(f"[data_checker] html={html_path}")

    if args.open:
        webbrowser.open(html_path.as_uri())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
