#!/usr/bin/env python3

import numpy as np
from scipy.interpolate import make_interp_spline

INTERPOLATING_BSPLINE_DEGREE = 3


def _safe_normalize(v):
    arr = np.asarray(v, float).reshape(-1)
    norm = float(np.linalg.norm(arr))
    if norm < 1e-12:
        return np.zeros(3, dtype=float)
    return arr / norm


def _safe_normalize_rows(v):
    arr = np.asarray(v, float)
    if arr.ndim != 2 or arr.shape[1] != 3 or len(arr) == 0:
        return np.empty((0, 3), float)
    norms = np.linalg.norm(arr, axis=1, keepdims=True)
    return arr / np.maximum(norms, 1e-12)


def _safe_normalize_2d(v):
    arr = np.asarray(v, float).reshape(-1)
    norm = float(np.linalg.norm(arr))
    if norm < 1e-12:
        return np.zeros(2, dtype=float)
    return arr / norm


def _polyline_arclength(points):
    pts = np.asarray(points, float)
    if pts.ndim != 2 or pts.shape[1] != 3 or len(pts) == 0:
        return np.zeros(0, dtype=float)
    if len(pts) == 1:
        return np.zeros(1, dtype=float)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(seg)])


def _sample_arclength_grid(total_length, spacing):
    length = float(total_length)
    step = max(float(spacing), 1e-4)
    if length <= 1e-9:
        return np.array([0.0], dtype=float)
    grid = np.arange(0.0, length, step, dtype=float)
    if grid.size == 0 or not np.isclose(grid[-1], length, atol=1e-9):
        grid = np.concatenate([grid, [length]])
    return np.unique(grid)


def _sample_uniform_arclength_grid(total_length, target_spacing):
    """Sample fixed arc-length steps without appending a short remainder."""
    length = max(float(total_length), 0.0)
    spacing = max(float(target_spacing), 1e-4)
    if length <= 1e-9:
        return np.array([0.0], dtype=float)

    segment_count = int(np.floor((length + 1e-9) / spacing))
    if segment_count <= 0:
        return np.array([0.0], dtype=float)
    grid = np.arange(segment_count + 1, dtype=float) * spacing
    if np.isclose(grid[-1], length, atol=1e-9, rtol=0.0):
        grid[-1] = length
    return grid


def _sample_line_segment(start_point, end_point, spacing):
    start = np.asarray(start_point, float).reshape(3)
    end = np.asarray(end_point, float).reshape(3)
    delta = end - start
    length = float(np.linalg.norm(delta))
    if length <= 1e-9:
        return start.reshape(1, 3).copy()

    fractions = _sample_arclength_grid(length, spacing) / length
    return start + fractions.reshape(-1, 1) * delta.reshape(1, 3)


def _false_runs(mask):
    arr = np.asarray(mask, dtype=bool).reshape(-1)
    if len(arr) == 0:
        return
    idx = 0
    while idx < len(arr):
        if arr[idx]:
            idx += 1
            continue
        start = idx
        while idx + 1 < len(arr) and not arr[idx + 1]:
            idx += 1
        yield start, idx
        idx += 1


def _true_runs(mask):
    arr = np.asarray(mask, dtype=bool).reshape(-1)
    if len(arr) == 0:
        return
    idx = 0
    while idx < len(arr):
        if not arr[idx]:
            idx += 1
            continue
        start = idx
        while idx + 1 < len(arr) and arr[idx + 1]:
            idx += 1
        yield start, idx
        idx += 1


def _spline_scan_config(spline_start_side):
    side = str(spline_start_side).strip().lower()
    aliases = {
        "top": "top",
        "from_top": "top",
        "top_to_bottom": "top",
        "bottom": "bottom",
        "from_bottom": "bottom",
        "bottom_to_top": "bottom",
        "left": "left",
        "from_left": "left",
        "left_to_right": "left",
        "right": "right",
        "from_right": "right",
        "right_to_left": "right",
    }
    side = aliases.get(side, side)
    if side == "top":
        return {"slice_axis": 1, "row_axis": 0}
    if side == "bottom":
        return {"slice_axis": 1, "row_axis": 0}
    if side == "left":
        return {"slice_axis": 0, "row_axis": 1}
    if side == "right":
        return {"slice_axis": 0, "row_axis": 1}
    raise ValueError("spline_start_side must be one of: top, bottom, left, right")


def _polyline_tangents(points):
    pts = np.asarray(points, float)
    if pts.ndim != 2 or pts.shape[1] != 3 or len(pts) == 0:
        return np.empty((0, 3), float)
    if len(pts) == 1:
        return np.array([[1.0, 0.0, 0.0]], dtype=float)

    tangents = _safe_normalize_rows(np.gradient(pts, axis=0, edge_order=1))
    for idx in range(1, len(tangents)):
        if float(np.dot(tangents[idx], tangents[idx - 1])) < 0.0:
            tangents[idx] = -tangents[idx]

    chord = pts[-1] - pts[0]
    chord_norm = float(np.linalg.norm(chord))
    if chord_norm > 1e-9:
        chord_dir = chord / chord_norm
        tangent_ref = np.mean(tangents, axis=0)
        if float(np.linalg.norm(tangent_ref)) <= 1e-9:
            tangent_ref = tangents[len(tangents) // 2]
        tangent_ref = _safe_normalize(tangent_ref)
        if float(np.dot(tangent_ref, chord_dir)) < 0.0:
            tangents = -tangents

    return tangents


def _slerp_unit_vectors(v0, v1, fraction):
    a = _safe_normalize(v0)
    b = _safe_normalize(v1)
    alpha = float(np.clip(fraction, 0.0, 1.0))
    dot = float(np.clip(np.dot(a, b), -1.0, 1.0))

    if dot > 0.9995:
        return _safe_normalize((1.0 - alpha) * a + alpha * b)
    if dot < -0.9995:
        blended = (1.0 - alpha) * a + alpha * b
        if float(np.linalg.norm(blended)) > 1e-9:
            return _safe_normalize(blended)
        return a.copy() if alpha < 0.5 else b.copy()

    theta = float(np.arccos(dot))
    sin_theta = float(np.sin(theta))
    coeff0 = np.sin((1.0 - alpha) * theta) / sin_theta
    coeff1 = np.sin(alpha * theta) / sin_theta
    return _safe_normalize(coeff0 * a + coeff1 * b)


def _rotation_matrix_to_quaternion_xyzw(rotation):
    mat = np.asarray(rotation, float).reshape(3, 3)
    trace = float(np.trace(mat))
    if trace > 0.0:
        s = 2.0 * np.sqrt(trace + 1.0)
        qx = (mat[2, 1] - mat[1, 2]) / s
        qy = (mat[0, 2] - mat[2, 0]) / s
        qz = (mat[1, 0] - mat[0, 1]) / s
        qw = 0.25 * s
    elif mat[0, 0] > mat[1, 1] and mat[0, 0] > mat[2, 2]:
        s = 2.0 * np.sqrt(1.0 + mat[0, 0] - mat[1, 1] - mat[2, 2])
        qx = 0.25 * s
        qy = (mat[0, 1] + mat[1, 0]) / s
        qz = (mat[0, 2] + mat[2, 0]) / s
        qw = (mat[2, 1] - mat[1, 2]) / s
    elif mat[1, 1] > mat[2, 2]:
        s = 2.0 * np.sqrt(1.0 + mat[1, 1] - mat[0, 0] - mat[2, 2])
        qx = (mat[0, 1] + mat[1, 0]) / s
        qy = 0.25 * s
        qz = (mat[1, 2] + mat[2, 1]) / s
        qw = (mat[0, 2] - mat[2, 0]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + mat[2, 2] - mat[0, 0] - mat[1, 1])
        qx = (mat[0, 2] + mat[2, 0]) / s
        qy = (mat[1, 2] + mat[2, 1]) / s
        qz = 0.25 * s
        qw = (mat[1, 0] - mat[0, 1]) / s

    quat = np.array([qx, qy, qz, qw], dtype=float)
    norm = float(np.linalg.norm(quat))
    if norm <= 1e-12:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
    return quat / norm


def _orientations_from_tangent_and_direction(tangents, directions):
    tangents = _safe_normalize_rows(tangents)
    directions = _safe_normalize_rows(directions)
    if tangents.shape != directions.shape or len(tangents) == 0:
        return np.empty((0, 4), float)

    quats = np.zeros((len(tangents), 4), dtype=float)
    reference_x = None
    for idx, (tangent, direction) in enumerate(zip(tangents, directions)):
        z_axis = _safe_normalize(direction)
        if float(np.linalg.norm(z_axis)) <= 1e-9:
            z_axis = np.array([0.0, 0.0, 1.0], dtype=float)

        x_axis = tangent - float(np.dot(tangent, z_axis)) * z_axis
        if float(np.linalg.norm(x_axis)) <= 1e-9 and reference_x is not None:
            x_axis = reference_x - float(np.dot(reference_x, z_axis)) * z_axis
        if float(np.linalg.norm(x_axis)) <= 1e-9:
            fallback = np.array([1.0, 0.0, 0.0], dtype=float)
            if abs(float(np.dot(fallback, z_axis))) > 0.9:
                fallback = np.array([0.0, 1.0, 0.0], dtype=float)
            x_axis = fallback - float(np.dot(fallback, z_axis)) * z_axis
        x_axis = _safe_normalize(x_axis)

        y_axis = _safe_normalize(np.cross(z_axis, x_axis))
        x_axis = _safe_normalize(np.cross(y_axis, z_axis))
        reference_x = x_axis.copy()

        rotation = np.column_stack([x_axis, y_axis, z_axis])
        quat = _rotation_matrix_to_quaternion_xyzw(rotation)
        if idx > 0 and float(np.dot(quat, quats[idx - 1])) < 0.0:
            quat = -quat
        quats[idx] = quat

    return quats


def _interpolate_points_at_arc(dense_arc, dense_points, target_arc):
    return np.column_stack(
        [
            np.interp(target_arc, dense_arc, dense_points[:, 0]),
            np.interp(target_arc, dense_arc, dense_points[:, 1]),
            np.interp(target_arc, dense_arc, dense_points[:, 2]),
        ]
    )


def _tangents_at_arc(dense_arc, dense_points, target_arc):
    dense_tangents = _polyline_tangents(dense_points)
    if dense_tangents.shape != dense_points.shape:
        return np.empty((0, 3), float)
    tangents = np.column_stack(
        [
            np.interp(target_arc, dense_arc, dense_tangents[:, 0]),
            np.interp(target_arc, dense_arc, dense_tangents[:, 1]),
            np.interp(target_arc, dense_arc, dense_tangents[:, 2]),
        ]
    )
    return _safe_normalize_rows(tangents)


def _build_interpolating_spline(points_world, directions_world):
    pts = np.asarray(points_world, float)
    dirs = np.asarray(directions_world, float)
    if (
        pts.ndim != 2
        or pts.shape[1] != 3
        or dirs.ndim != 2
        or dirs.shape != pts.shape
        or len(pts) < 2
    ):
        return None

    arc = _polyline_arclength(pts)
    keep = np.concatenate([[True], np.diff(arc) > 1e-9])
    pts = pts[keep]
    dirs = _safe_normalize_rows(dirs[keep])
    arc = arc[keep]
    if len(pts) < 2 or float(arc[-1]) <= 1e-9:
        return None

    k = int(max(1, min(INTERPOLATING_BSPLINE_DEGREE, len(pts) - 1)))
    spline = make_interp_spline(arc, pts, k=k, axis=0)
    return {
        "anchor_points": pts,
        "anchor_directions": dirs,
        "anchor_arc": arc,
        "spline": spline,
        "degree": k,
    }


def _dense_spline_core_polyline(built, point_spacing):
    anchor_points = built["anchor_points"]
    anchor_arc = built["anchor_arc"]
    spline = built["spline"]

    dense_parts = [anchor_points[0].reshape(1, 3)]
    core_anchor_lengths = [0.0]
    spacing = max(float(point_spacing), 1e-4)

    for idx in range(len(anchor_points) - 1):
        start_arc = float(anchor_arc[idx])
        end_arc = float(anchor_arc[idx + 1])
        param_span = max(end_arc - start_arc, 1e-9)
        dense_count = max(64, int(np.ceil(param_span / spacing)) * 32)
        dense_arc_param = np.linspace(start_arc, end_arc, dense_count, dtype=float)
        segment_points = np.asarray(spline(dense_arc_param), float)
        segment_points[0] = anchor_points[idx]
        segment_points[-1] = anchor_points[idx + 1]

        segment_arc = _polyline_arclength(segment_points)
        segment_length = float(segment_arc[-1]) if len(segment_arc) else 0.0
        if segment_length <= 1e-9:
            continue

        dense_parts.append(segment_points[1:])
        core_anchor_lengths.append(core_anchor_lengths[-1] + segment_length)

    if len(core_anchor_lengths) != len(anchor_points):
        return None, None

    dense_points = np.vstack(dense_parts)
    if len(dense_points) < 2:
        return None, None
    return dense_points, np.asarray(core_anchor_lengths, dtype=float)


def _spline_endpoint_tangents(built):
    anchor_points = built["anchor_points"]
    anchor_arc = built["anchor_arc"]
    spline = built["spline"]

    start_tangent = np.zeros(3, dtype=float)
    end_tangent = np.zeros(3, dtype=float)
    try:
        derivative = spline.derivative()
        start_tangent = _safe_normalize(derivative(float(anchor_arc[0])))
        end_tangent = _safe_normalize(derivative(float(anchor_arc[-1])))
    except Exception:
        pass

    if float(np.linalg.norm(start_tangent)) <= 1e-9:
        start_tangent = _safe_normalize(anchor_points[1] - anchor_points[0])
    if float(np.linalg.norm(end_tangent)) <= 1e-9:
        end_tangent = _safe_normalize(anchor_points[-1] - anchor_points[-2])
    return start_tangent, end_tangent


def _interpolate_anchor_direction(anchor_dirs, core_anchor_lengths, core_distance):
    dirs = _safe_normalize_rows(anchor_dirs)
    lengths = np.asarray(core_anchor_lengths, float).reshape(-1)
    if len(dirs) == 0:
        return np.array([0.0, 0.0, -1.0], dtype=float)
    if len(dirs) == 1 or len(lengths) != len(dirs):
        return dirs[0].copy()

    distance = float(np.clip(core_distance, 0.0, lengths[-1]))
    if distance >= lengths[-1] - 1e-9:
        return dirs[-1].copy()

    idx = int(np.searchsorted(lengths, distance, side="right") - 1)
    idx = int(np.clip(idx, 0, len(dirs) - 2))
    segment_length = float(lengths[idx + 1] - lengths[idx])
    if segment_length <= 1e-9:
        fraction = 0.0
    else:
        fraction = (distance - float(lengths[idx])) / segment_length
    return _slerp_unit_vectors(dirs[idx], dirs[idx + 1], fraction)


def _sample_extended_spline_row_uniform(
    points_world,
    directions_world,
    point_spacing,
    final_point_spacing,
    extension_length,
):
    built = _build_interpolating_spline(points_world, directions_world)
    if built is None:
        return None

    anchor_points = built["anchor_points"]
    anchor_dirs = built["anchor_directions"]
    spacing = max(float(point_spacing), 1e-4)
    output_spacing = max(float(final_point_spacing), 1e-4)
    extension_length = max(float(extension_length), 0.0)

    dense_core_points, core_anchor_lengths = _dense_spline_core_polyline(
        built,
        point_spacing=spacing,
    )
    if dense_core_points is None or core_anchor_lengths is None:
        return None

    start_tangent, end_tangent = _spline_endpoint_tangents(built)
    start_extension_point = None
    end_extension_point = None
    start_extension_length = 0.0
    end_extension_length = 0.0
    extension_rows = []
    extension_points = []

    if extension_length > 1e-9 and float(np.linalg.norm(start_tangent)) > 1e-9:
        start_extension_point = anchor_points[0] - extension_length * start_tangent
        start_extension_length = float(
            np.linalg.norm(anchor_points[0] - start_extension_point)
        )
        extension_rows.append(
            _sample_line_segment(start_extension_point, anchor_points[0], spacing)
        )
        extension_points.append(start_extension_point.reshape(1, 3))

    if extension_length > 1e-9 and float(np.linalg.norm(end_tangent)) > 1e-9:
        end_extension_point = anchor_points[-1] + extension_length * end_tangent
        end_extension_length = float(
            np.linalg.norm(end_extension_point - anchor_points[-1])
        )
        extension_rows.append(
            _sample_line_segment(anchor_points[-1], end_extension_point, spacing)
        )
        extension_points.append(end_extension_point.reshape(1, 3))

    dense_parts = []
    if start_extension_point is not None and start_extension_length > 1e-9:
        dense_parts.append(np.vstack([start_extension_point, anchor_points[0]])[:-1])
    dense_parts.append(dense_core_points)
    if end_extension_point is not None and end_extension_length > 1e-9:
        dense_parts.append(np.vstack([anchor_points[-1], end_extension_point])[1:])

    dense_points = np.vstack(dense_parts)
    dense_arc = _polyline_arclength(dense_points)
    keep = np.concatenate([[True], np.diff(dense_arc) > 1e-9])
    dense_points = dense_points[keep]
    dense_arc = dense_arc[keep]
    if len(dense_points) < 2 or float(dense_arc[-1]) <= 1e-9:
        return None

    total_length = float(dense_arc[-1])
    target_arc = _sample_uniform_arclength_grid(total_length, output_spacing)
    if len(target_arc) < 2:
        return None

    sampled_points = _interpolate_points_at_arc(dense_arc, dense_points, target_arc)
    sampled_points[0] = dense_points[0]
    if np.isclose(target_arc[-1], total_length, atol=1e-9, rtol=0.0):
        sampled_points[-1] = dense_points[-1]

    core_start_arc = start_extension_length
    core_end_arc = start_extension_length + float(core_anchor_lengths[-1])
    eps = max(1e-8, spacing * 1e-6)
    base_paint_mask = (
        (target_arc >= core_start_arc - eps)
        & (target_arc <= core_end_arc + eps)
    )

    sampled_dirs = np.zeros_like(sampled_points)
    for idx, distance in enumerate(target_arc):
        if distance < core_start_arc - eps:
            sampled_dirs[idx] = anchor_dirs[0]
        elif distance > core_end_arc + eps:
            sampled_dirs[idx] = anchor_dirs[-1]
        else:
            sampled_dirs[idx] = _interpolate_anchor_direction(
                anchor_dirs,
                core_anchor_lengths,
                core_distance=float(distance - core_start_arc),
            )
    sampled_dirs = _safe_normalize_rows(sampled_dirs)
    if sampled_dirs.shape != sampled_points.shape:
        return None

    anchor_mask = np.zeros(len(target_arc), dtype=bool)
    for anchor_distance in core_anchor_lengths:
        target_distance = core_start_arc + float(anchor_distance)
        anchor_mask |= np.isclose(
            target_arc,
            target_distance,
            atol=max(1e-7, spacing * 1e-4),
            rtol=0.0,
        )
    anchor_mask &= base_paint_mask

    tangents = _tangents_at_arc(dense_arc, dense_points, target_arc)
    if tangents.shape != sampled_points.shape:
        return None

    orientations = _orientations_from_tangent_and_direction(
        tangents,
        sampled_dirs,
    )
    if orientations.shape != (len(sampled_points), 4):
        return None

    core_points = sampled_points[base_paint_mask]
    return {
        "points": sampled_points,
        "core_points": core_points,
        "arc_lengths": target_arc,
        "directions": sampled_dirs,
        "tangents": tangents,
        "orientations": orientations,
        "anchor_mask": anchor_mask,
        "base_paint_mask": base_paint_mask,
        "extension_rows": extension_rows,
        "extension_points": extension_points,
        "spline_degree": int(built["degree"]),
    }


def _reverse_sampled_row_for_zigzag(sampled):
    total_length = float(np.asarray(sampled["arc_lengths"], float).reshape(-1)[-1])
    reversed_sampled = dict(sampled)

    reversed_sampled["points"] = np.asarray(sampled["points"], float)[::-1].copy()
    reversed_sampled["core_points"] = reversed_sampled["points"][
        np.asarray(sampled["base_paint_mask"], dtype=bool)[::-1]
    ].copy()
    reversed_sampled["directions"] = np.asarray(sampled["directions"], float)[::-1].copy()
    reversed_sampled["tangents"] = -np.asarray(sampled["tangents"], float)[::-1].copy()
    reversed_sampled["orientations"] = np.asarray(sampled["orientations"], float)[::-1].copy()
    reversed_sampled["arc_lengths"] = (
        total_length - np.asarray(sampled["arc_lengths"], float).reshape(-1)[::-1]
    )
    reversed_sampled["anchor_mask"] = np.asarray(sampled["anchor_mask"], dtype=bool)[
        ::-1
    ].copy()
    reversed_sampled["base_paint_mask"] = np.asarray(
        sampled["base_paint_mask"], dtype=bool
    )[::-1].copy()
    reversed_sampled["extension_rows"] = [
        np.asarray(row, float)[::-1].copy()
        for row in reversed(list(sampled["extension_rows"]))
    ]
    reversed_sampled["extension_points"] = [
        np.asarray(point, float).copy()
        for point in reversed(list(sampled["extension_points"]))
    ]
    return reversed_sampled


def _false_run_length(points, start, end):
    pts = np.asarray(points, float)
    if pts.ndim != 2 or pts.shape[1] != 3 or len(pts) == 0:
        return 0.0
    lo = int(max(0, start))
    hi = int(min(len(pts) - 1, end))
    if lo > hi:
        return 0.0

    length = 0.0
    if lo > 0:
        length += 0.5 * float(np.linalg.norm(pts[lo] - pts[lo - 1]))
    if hi > lo:
        length += float(np.sum(np.linalg.norm(np.diff(pts[lo : hi + 1], axis=0), axis=1)))
    if hi < len(pts) - 1:
        length += 0.5 * float(np.linalg.norm(pts[hi + 1] - pts[hi]))
    return length


def _fill_short_false_runs(mask, points, min_false_length):
    arr = np.asarray(mask, dtype=bool).reshape(-1).copy()
    threshold = max(float(min_false_length), 0.0)
    if threshold <= 0.0 or len(arr) == 0:
        return arr

    for start, end in _false_runs(arr):
        if _false_run_length(points, start, end) < threshold:
            arr[start : end + 1] = True
    return arr


def _fill_short_true_islands(mask, points, min_true_length):
    arr = np.asarray(mask, dtype=bool).reshape(-1).copy()
    threshold = max(float(min_true_length), 0.0)
    if threshold <= 0.0 or len(arr) == 0:
        return arr

    for start, end in _true_runs(arr):
        if start == 0 or end == len(arr) - 1:
            continue
        if arr[start - 1] or arr[end + 1]:
            continue
        if _false_run_length(points, start, end) < threshold:
            arr[start : end + 1] = False
    return arr


def _filter_core_surface_hit_mask(
    raw_surface_hit_mask,
    points,
    base_paint_mask,
    min_false_length,
    min_true_length,
):
    raw_mask = np.asarray(raw_surface_hit_mask, dtype=bool).reshape(-1)
    pts = np.asarray(points, float)
    base_mask = np.asarray(base_paint_mask, dtype=bool).reshape(-1)
    if len(raw_mask) == 0 or len(raw_mask) != len(base_mask) or len(pts) != len(base_mask):
        return raw_mask.copy()

    filtered = np.zeros(len(raw_mask), dtype=bool)
    core_indices = np.flatnonzero(base_mask)
    if len(core_indices) == 0:
        return filtered

    start = int(core_indices[0])
    end = int(core_indices[-1]) + 1
    core_mask = raw_mask[start:end]
    core_points = pts[start:end]
    core_mask = _fill_short_false_runs(
        core_mask,
        core_points,
        min_false_length=min_false_length,
    )
    core_mask = _fill_short_true_islands(
        core_mask,
        core_points,
        min_true_length=min_true_length,
    )
    filtered[start:end] = core_mask
    return filtered


def _nearest_slice_profile(slice_profiles, slice_position):
    best_profile = None
    best_error = np.inf
    for profile in slice_profiles:
        points = np.asarray(profile.get("slice_points_work", []), float)
        if points.ndim != 2 or points.shape[1] != 3 or len(points) == 0:
            continue
        error = abs(float(profile.get("slice_position", 0.0)) - float(slice_position))
        if error < best_error:
            best_error = error
            best_profile = profile
    return best_profile


def _slice_profile_for_offset_row(
    slice_profiles,
    offset_row_profile,
    offset_row_index,
    fallback_slice_position,
):
    source = dict(offset_row_profile or {})
    if "row_index" in source:
        source_row_index = int(source["row_index"])
        for profile in slice_profiles:
            if int(profile.get("row_index", -1)) == source_row_index:
                return profile

    if 0 <= int(offset_row_index) < len(slice_profiles):
        profile = slice_profiles[int(offset_row_index)]
        if str(profile.get("slicing_method", "axis")).strip().lower() != "axis":
            return profile

    return _nearest_slice_profile(slice_profiles, fallback_slice_position)


def _paint_surface_hit_mask_2d(
    sample_points_world,
    directions_world,
    slice_profile,
    r_work,
    origin,
    check_distance,
    half_width,
):
    points_world = np.asarray(sample_points_world, float)
    dirs_world = np.asarray(directions_world, float)
    profile = dict(slice_profile or {})
    observed_work = np.asarray(profile.get("slice_points_work", []), float)
    if (
        points_world.ndim != 2
        or points_world.shape[1] != 3
        or dirs_world.shape != points_world.shape
        or observed_work.ndim != 2
        or observed_work.shape[1] != 3
        or len(points_world) == 0
        or len(observed_work) == 0
    ):
        return np.ones(len(points_world), dtype=bool)

    sample_points_work = (points_world - origin.reshape(1, 3)) @ r_work
    directions_work = _safe_normalize_rows(dirs_world @ r_work)

    plane_origin_work = np.asarray(
        profile.get("slice_plane_origin_work", []),
        dtype=float,
    ).reshape(-1)
    plane_row_axis_work = _safe_normalize(
        profile.get("slice_plane_row_axis_work", [])
    )
    plane_other_axis_work = _safe_normalize(
        profile.get("slice_plane_other_axis_work", [])
    )
    if (
        plane_origin_work.size != 3
        or float(np.linalg.norm(plane_row_axis_work)) <= 1e-9
        or float(np.linalg.norm(plane_other_axis_work)) <= 1e-9
    ):
        return np.ones(len(points_world), dtype=bool)

    plane_other_axis_work = plane_other_axis_work - float(
        np.dot(plane_other_axis_work, plane_row_axis_work)
    ) * plane_row_axis_work
    plane_other_axis_work = _safe_normalize(plane_other_axis_work)
    if float(np.linalg.norm(plane_other_axis_work)) <= 1e-9:
        return np.ones(len(points_world), dtype=bool)

    sample_relative_work = sample_points_work - plane_origin_work.reshape(1, 3)
    observed_relative_work = observed_work - plane_origin_work.reshape(1, 3)
    sample_uw = np.column_stack(
        [
            sample_relative_work @ plane_row_axis_work,
            sample_relative_work @ plane_other_axis_work,
        ]
    )
    observed_uw = np.column_stack(
        [
            observed_relative_work @ plane_row_axis_work,
            observed_relative_work @ plane_other_axis_work,
        ]
    )

    axial_limit = max(float(check_distance), 0.0)
    lateral_limit = max(float(half_width), 0.0)

    hit_mask = np.zeros(len(sample_uw), dtype=bool)
    direction_uw = np.column_stack(
        [
            directions_work @ plane_row_axis_work,
            directions_work @ plane_other_axis_work,
        ]
    )
    for idx, (sample, direction_work) in enumerate(zip(sample_uw, direction_uw)):
        axis = _safe_normalize_2d(direction_work)
        if float(np.linalg.norm(axis)) <= 1e-9:
            continue
        perp = np.array([-axis[1], axis[0]], dtype=float)
        rel = observed_uw - sample[None, :]
        axial = rel @ axis
        lateral = np.abs(rel @ perp)
        hit_mask[idx] = bool(
            np.any(
                (axial >= 0.0)
                & (axial <= axial_limit)
                & (lateral <= lateral_limit)
            )
        )
    return hit_mask


def generate_paint_spline_rows(
    offset_result,
    paint_spline_point_spacing=0.01,
    final_point_spacing=0.04,
    raster_zigzag=True,
    endpoint_extension_length=0.0,
    paint_check_distance=0.22,
    paint_check_half_width=0.005,
    paint_min_false_length=0.02,
    paint_min_true_length=0.0,
):
    result = dict(offset_result)

    offset_rows = list(result.get("offset_rows", []))
    offset_normals = list(result.get("offset_row_normals_world", []))
    offset_row_profiles = list(result.get("offset_row_profiles", []))
    slice_profiles = list(result.get("slice_profiles", []))
    default_direction = _safe_normalize(result.get("view_dir_world", [0.0, 0.0, -1.0]))
    r_work = np.asarray(result.get("work_frame_world", []), float)
    origin = np.asarray(result.get("work_origin_world", []), float).reshape(-1)
    cfg = _spline_scan_config(result.get("spline_start_side", "top"))
    row_axis_idx = int(cfg["row_axis"])
    slice_axis_idx = int(cfg["slice_axis"])

    core_rows = []
    full_rows = []
    extension_rows = []
    extension_points = []
    direction_rows = []
    orientation_rows = []
    tangent_rows = []
    arc_length_rows = []
    anchor_mask_rows = []
    base_paint_mask_rows = []
    raw_surface_hit_mask_rows = []
    surface_hit_mask_rows = []
    paint_mask_rows = []
    row_slice_positions = []
    paint_spline_profiles = []

    for row_idx, row in enumerate(offset_rows):
        anchor_points = np.asarray(row, float)
        if anchor_points.ndim != 2 or anchor_points.shape[1] != 3 or len(anchor_points) < 2:
            continue

        if row_idx < len(offset_normals):
            anchor_dirs = -np.asarray(offset_normals[row_idx], float)
        else:
            anchor_dirs = np.repeat(default_direction.reshape(1, 3), len(anchor_points), axis=0)
        anchor_dirs = _safe_normalize_rows(anchor_dirs)
        if anchor_dirs.shape != anchor_points.shape:
            anchor_dirs = np.repeat(default_direction.reshape(1, 3), len(anchor_points), axis=0)

        trajectory_row_index = len(full_rows)
        row_should_zigzag = bool(raster_zigzag) and trajectory_row_index % 2 == 1

        sampled = _sample_extended_spline_row_uniform(
            anchor_points,
            anchor_dirs,
            point_spacing=paint_spline_point_spacing,
            final_point_spacing=final_point_spacing,
            extension_length=endpoint_extension_length,
        )
        if sampled is None:
            continue
        if row_should_zigzag:
            sampled = _reverse_sampled_row_for_zigzag(sampled)

        full_row = sampled["points"]
        core_row = sampled["core_points"]
        full_dirs = sampled["directions"]
        full_tangents = sampled["tangents"]
        full_orientations = sampled["orientations"]
        full_arc_lengths = np.asarray(sampled["arc_lengths"], float).reshape(-1)
        full_anchor_mask = sampled["anchor_mask"]
        base_paint_mask = sampled["base_paint_mask"]

        if len(core_row) > 0:
            core_rows.append(core_row)
        extension_rows.extend(sampled["extension_rows"])
        extension_points.extend(sampled["extension_points"])

        raw_surface_hit_mask = np.ones(len(full_row), dtype=bool)
        row_slice_position = np.nan
        source_slice_profile = None
        if r_work.shape == (3, 3) and origin.size == 3 and slice_profiles:
            full_row_work = (full_row - origin.reshape(1, 3)) @ r_work
            row_slice_position = float(np.median(full_row_work[:, slice_axis_idx]))
            offset_row_profile = (
                offset_row_profiles[row_idx]
                if row_idx < len(offset_row_profiles)
                else None
            )
            source_slice_profile = _slice_profile_for_offset_row(
                slice_profiles,
                offset_row_profile=offset_row_profile,
                offset_row_index=row_idx,
                fallback_slice_position=row_slice_position,
            )
            if source_slice_profile is not None:
                raw_surface_hit_mask = _paint_surface_hit_mask_2d(
                    full_row,
                    full_dirs,
                    source_slice_profile,
                    r_work=r_work,
                    origin=origin,
                    check_distance=paint_check_distance,
                    half_width=paint_check_half_width,
                )
        surface_hit_mask = _filter_core_surface_hit_mask(
            raw_surface_hit_mask,
            full_row,
            base_paint_mask,
            min_false_length=paint_min_false_length,
            min_true_length=paint_min_true_length,
        )
        final_paint_mask = (
            np.asarray(base_paint_mask, dtype=bool)
            & np.asarray(surface_hit_mask, dtype=bool)
        )

        full_rows.append(full_row)
        direction_rows.append(full_dirs)
        tangent_rows.append(full_tangents)
        orientation_rows.append(full_orientations)
        arc_length_rows.append(full_arc_lengths)
        anchor_mask_rows.append(full_anchor_mask)
        base_paint_mask_rows.append(np.asarray(base_paint_mask, dtype=bool))
        raw_surface_hit_mask_rows.append(np.asarray(raw_surface_hit_mask, dtype=bool))
        surface_hit_mask_rows.append(np.asarray(surface_hit_mask, dtype=bool))
        paint_mask_rows.append(final_paint_mask)
        row_slice_positions.append(row_slice_position)
        paint_spline_profiles.append(
            {
                "points_world": full_row,
                "directions_world": full_dirs,
                "tangents_world": full_tangents,
                "orientations_xyzw": full_orientations,
                "arc_lengths": full_arc_lengths.copy(),
                "anchor_mask": full_anchor_mask.copy(),
                "base_paint_mask": np.asarray(base_paint_mask, dtype=bool).copy(),
                "raw_surface_hit_mask": np.asarray(raw_surface_hit_mask, dtype=bool).copy(),
                "surface_hit_mask": np.asarray(surface_hit_mask, dtype=bool).copy(),
                "paint_mask": final_paint_mask.copy(),
                "slice_position": row_slice_position,
                "source_slice_row_index": (
                    int(source_slice_profile.get("row_index", row_idx))
                    if source_slice_profile is not None
                    else None
                ),
                "source_slice_position": (
                    float(source_slice_profile.get("slice_position", row_slice_position))
                    if source_slice_profile is not None
                    else np.nan
                ),
            }
        )

    if full_rows:
        result["paint_spline_points_world"] = np.vstack(full_rows)
    else:
        result["paint_spline_points_world"] = np.empty((0, 3), float)

    if core_rows:
        result["paint_spline_core_points_world"] = np.vstack(core_rows)
    else:
        result["paint_spline_core_points_world"] = np.empty((0, 3), float)

    if extension_points:
        result["paint_spline_extension_points_world"] = np.vstack(extension_points)
    else:
        result["paint_spline_extension_points_world"] = np.empty((0, 3), float)

    result["paint_spline_rows"] = full_rows
    result["paint_spline_core_rows"] = core_rows
    result["paint_spline_extension_rows"] = extension_rows
    result["paint_spline_direction_rows_world"] = direction_rows
    result["paint_spline_tangent_rows_world"] = tangent_rows
    result["paint_spline_orientation_rows_xyzw"] = orientation_rows
    result["paint_spline_arc_length_rows"] = arc_length_rows
    result["paint_spline_anchor_mask_rows"] = anchor_mask_rows
    result["paint_spline_base_paint_mask_rows"] = base_paint_mask_rows
    result["paint_spline_raw_surface_hit_mask_rows"] = raw_surface_hit_mask_rows
    result["paint_spline_surface_hit_mask_rows"] = surface_hit_mask_rows
    result["paint_spline_paint_mask_rows"] = paint_mask_rows
    result["paint_spline_row_slice_positions"] = row_slice_positions
    result["paint_spline_profiles"] = paint_spline_profiles
    result["paint_spline_point_spacing"] = float(paint_spline_point_spacing)
    result["paint_spline_final_point_spacing"] = float(final_point_spacing)
    result["paint_spline_raster_zigzag"] = bool(raster_zigzag)
    result["paint_spline_paint_check_distance"] = float(paint_check_distance)
    result["paint_spline_paint_check_half_width"] = float(paint_check_half_width)
    result["paint_spline_paint_min_false_length"] = float(paint_min_false_length)
    result["paint_spline_paint_min_true_length"] = float(paint_min_true_length)
    result["paint_spline_degree"] = int(INTERPOLATING_BSPLINE_DEGREE)
    result["endpoint_extension_length"] = float(endpoint_extension_length)

    result["paint_trajectory_position_rows"] = full_rows
    result["paint_trajectory_orientation_rows_xyzw"] = orientation_rows
    result["paint_trajectory_arc_length_rows"] = arc_length_rows
    result["paint_trajectory_paint_mask_rows"] = paint_mask_rows
    return result


__all__ = ["generate_paint_spline_rows"]
