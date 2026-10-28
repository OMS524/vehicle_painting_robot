#!/usr/bin/env python3

import numpy as np


def _safe_normalize(v):
    arr = np.asarray(v, float).reshape(-1)
    norm = float(np.linalg.norm(arr))
    if norm < 1e-12:
        return np.array([0.0, 0.0, 1.0], dtype=float)
    return arr / norm


def _safe_normalize_rows(v):
    arr = np.asarray(v, float)
    if arr.ndim != 2 or arr.shape[1] != 3 or len(arr) == 0:
        return np.empty((0, 3), float)
    norms = np.linalg.norm(arr, axis=1, keepdims=True)
    return arr / np.maximum(norms, 1e-12)


def _safe_normalize_rows_2d(v):
    arr = np.asarray(v, float)
    if arr.ndim != 2 or arr.shape[1] != 2 or len(arr) == 0:
        return np.empty((0, 2), float)
    norms = np.linalg.norm(arr, axis=1, keepdims=True)
    return arr / np.maximum(norms, 1e-12)


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


def _polyline_arclength(points):
    pts = np.asarray(points, float)
    if pts.ndim != 2 or pts.shape[1] != 3 or len(pts) == 0:
        return np.zeros(0, dtype=float)
    if len(pts) == 1:
        return np.zeros(1, dtype=float)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(seg)])


def _sample_polyline_uniform(points, spacing):
    pts = np.asarray(points, float)
    if pts.ndim != 2 or pts.shape[1] != 3 or len(pts) < 2:
        return None

    arc = _polyline_arclength(pts)
    keep = np.concatenate([[True], np.diff(arc) > 1e-9])
    pts = pts[keep]
    arc = arc[keep]
    if len(pts) < 2 or float(arc[-1]) <= 1e-9:
        return None

    spacing = max(float(spacing), 1e-4)
    sample_arc = np.arange(0.0, float(arc[-1]), spacing, dtype=float)
    if sample_arc.size == 0 or not np.isclose(sample_arc[-1], arc[-1], atol=1e-9):
        sample_arc = np.concatenate([sample_arc, [float(arc[-1])]])
    sample_arc = np.unique(sample_arc)

    sampled = np.column_stack(
        [
            np.interp(sample_arc, arc, pts[:, 0]),
            np.interp(sample_arc, arc, pts[:, 1]),
            np.interp(sample_arc, arc, pts[:, 2]),
        ]
    )
    return sampled


def _row_tangents_2d(points_2d):
    pts = np.asarray(points_2d, float)
    if pts.ndim != 2 or pts.shape[1] != 2 or len(pts) == 0:
        return np.empty((0, 2), float)
    return _safe_normalize_rows_2d(np.gradient(pts, axis=0, edge_order=1))


def _orient_tangents_along_row(points_2d, tangents_2d):
    pts = np.asarray(points_2d, float)
    tangents = _safe_normalize_rows_2d(tangents_2d)
    if pts.ndim != 2 or pts.shape[1] != 2 or len(pts) == 0 or len(tangents) != len(pts):
        return np.empty((0, 2), float)
    if len(tangents) <= 1:
        return tangents

    oriented = tangents.copy()

    # 1) Local consistency: neighboring tangents should not point opposite ways.
    for idx in range(1, len(oriented)):
        if float(np.dot(oriented[idx], oriented[idx - 1])) < 0.0:
            oriented[idx] = -oriented[idx]

    # 2) Global consistency: align the overall tangent direction with the chord.
    chord = pts[-1] - pts[0]
    chord_norm = float(np.linalg.norm(chord))
    if chord_norm > 1e-9:
        chord_dir = chord / chord_norm
        tangent_ref = np.mean(oriented, axis=0)
        if float(np.linalg.norm(tangent_ref)) <= 1e-9:
            tangent_ref = oriented[len(oriented) // 2]
        tangent_ref = _safe_normalize(tangent_ref)
        if float(np.dot(tangent_ref, chord_dir)) < 0.0:
            oriented = -oriented

    return oriented


def _offset_vectors_from_slice_plane_row(
    sampled_points_work,
    plane_origin_work,
    plane_row_axis_work,
    plane_other_axis_work,
    row_axis_idx,
):
    """Compute v1-style row normals in each slice profile's local plane."""
    pts = np.asarray(sampled_points_work, float)
    if pts.ndim != 2 or pts.shape[1] != 3 or len(pts) == 0:
        return np.empty((0, 3), float)

    fallback_row_axis = np.zeros(3, dtype=float)
    fallback_row_axis[int(row_axis_idx)] = 1.0
    outward_axis = np.array([0.0, 0.0, 1.0], dtype=float)

    plane_row_axis = np.asarray(plane_row_axis_work, float).reshape(-1)
    if plane_row_axis.size != 3 or np.linalg.norm(plane_row_axis) <= 1e-12:
        plane_row_axis = fallback_row_axis
    plane_row_axis = _safe_normalize(plane_row_axis)

    plane_other_axis = np.asarray(plane_other_axis_work, float).reshape(-1)
    if plane_other_axis.size != 3 or np.linalg.norm(plane_other_axis) <= 1e-12:
        plane_other_axis = outward_axis.copy()
    plane_other_axis = (
        plane_other_axis
        - float(np.dot(plane_other_axis, plane_row_axis)) * plane_row_axis
    )
    if np.linalg.norm(plane_other_axis) <= 1e-12:
        plane_other_axis = (
            outward_axis
            - float(np.dot(outward_axis, plane_row_axis)) * plane_row_axis
        )
    if np.linalg.norm(plane_other_axis) <= 1e-12:
        return np.empty((0, 3), float)
    plane_other_axis = _safe_normalize(plane_other_axis)
    if float(np.dot(plane_other_axis, outward_axis)) < 0.0:
        plane_other_axis *= -1.0

    plane_origin = np.asarray(plane_origin_work, float).reshape(-1)
    if plane_origin.size != 3 or not np.all(np.isfinite(plane_origin)):
        plane_origin = np.mean(pts, axis=0)
    relative = pts - plane_origin.reshape(1, 3)
    pts_uw = np.column_stack(
        [
            relative @ plane_row_axis,
            relative @ plane_other_axis,
        ]
    )
    tangents_uw = _row_tangents_2d(pts_uw)
    tangents_uw = _orient_tangents_along_row(pts_uw, tangents_uw)
    if len(tangents_uw) != len(pts_uw):
        return np.empty((0, 3), float)

    offset_vectors = np.zeros((len(tangents_uw), 3), dtype=float)
    if len(tangents_uw) == 0:
        return offset_vectors

    chord = pts_uw[-1] - pts_uw[0]
    chord_u = float(chord[0])

    # Desired normal convention in (u, w):
    # - left -> right row  : n = (-t_w,  t_u)
    # - right -> left row : n = ( t_w, -t_u)
    if chord_u >= 0.0:
        normals_uw = np.column_stack([-tangents_uw[:, 1], tangents_uw[:, 0]])
    else:
        normals_uw = np.column_stack([tangents_uw[:, 1], -tangents_uw[:, 0]])
    normals_uw = _safe_normalize_rows_2d(normals_uw)

    offset_vectors = (
        normals_uw[:, 0:1] * plane_row_axis.reshape(1, 3)
        + normals_uw[:, 1:2] * plane_other_axis.reshape(1, 3)
    )
    offset_vectors = _safe_normalize_rows(offset_vectors)

    return offset_vectors


def generate_offset_rows(
    correction_result,
    offset_point_spacing=0.02,
    offset_distance=0.05,
):
    result = dict(correction_result)

    corrected_profiles = list(result.get("corrected_row_profiles", []))
    r_work = np.asarray(result.get("work_frame_world", []), float)
    origin = np.asarray(result.get("work_origin_world", []), float).reshape(-1)

    offset_rows = []
    offset_normals_world = []
    offset_row_profiles = []

    if r_work.shape != (3, 3) or origin.size != 3 or not corrected_profiles:
        result["offset_points_world"] = np.empty((0, 3), float)
        result["offset_rows"] = []
        result["offset_row_normals_world"] = []
        result["offset_row_profiles"] = []
        result["offset_point_spacing"] = float(offset_point_spacing)
        result["offset_distance"] = float(offset_distance)
        return result

    cfg = _spline_scan_config(result.get("spline_start_side", "top"))
    row_axis_idx = int(cfg["row_axis"])
    work_to_world = lambda x: np.asarray(x, float) @ r_work.T + origin
    dir_to_world = lambda x: np.asarray(x, float) @ r_work.T

    for profile in corrected_profiles:
        pts_work = np.asarray(profile.get("path_points_work", []), float)
        if pts_work.ndim != 2 or pts_work.shape[1] != 3 or len(pts_work) < 2:
            continue

        sampled_pts_work = _sample_polyline_uniform(
            pts_work,
            spacing=offset_point_spacing,
        )
        if sampled_pts_work is None or len(sampled_pts_work) < 2:
            continue

        offset_vecs_work = _offset_vectors_from_slice_plane_row(
            sampled_pts_work,
            plane_origin_work=profile.get("slice_plane_origin_work", []),
            plane_row_axis_work=profile.get("slice_plane_row_axis_work", []),
            plane_other_axis_work=profile.get("slice_plane_other_axis_work", []),
            row_axis_idx=row_axis_idx,
        )
        if len(offset_vecs_work) != len(sampled_pts_work):
            continue

        sampled_pts_world = work_to_world(sampled_pts_work)
        offset_vecs_world = _safe_normalize_rows(dir_to_world(offset_vecs_work))
        original_offset_pts_world = sampled_pts_world + float(offset_distance) * offset_vecs_world

        offset_rows.append(original_offset_pts_world)
        offset_normals_world.append(offset_vecs_world)
        offset_row_profiles.append(
            {
                "row_index": int(profile.get("row_index", len(offset_row_profiles))),
                "slice_position": float(
                    profile.get("slice_position", len(offset_row_profiles))
                ),
                "slice_plane_origin_work": np.asarray(
                    profile.get("slice_plane_origin_work", []),
                    dtype=float,
                ).copy(),
                "slice_plane_normal_work": np.asarray(
                    profile.get("slice_plane_normal_work", []),
                    dtype=float,
                ).copy(),
                "slice_plane_row_axis_work": np.asarray(
                    profile.get("slice_plane_row_axis_work", []),
                    dtype=float,
                ).copy(),
                "slice_plane_other_axis_work": np.asarray(
                    profile.get("slice_plane_other_axis_work", []),
                    dtype=float,
                ).copy(),
            }
        )

    if offset_rows:
        result["offset_points_world"] = np.vstack(offset_rows)
    else:
        result["offset_points_world"] = np.empty((0, 3), float)

    result["offset_rows"] = offset_rows
    result["offset_row_normals_world"] = offset_normals_world
    result["offset_row_profiles"] = offset_row_profiles
    result["offset_point_spacing"] = float(offset_point_spacing)
    result["offset_distance"] = float(offset_distance)
    return result


__all__ = ["generate_offset_rows"]
