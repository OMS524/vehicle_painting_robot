import numpy as np
from scipy.spatial import cKDTree

from . import correction

_select_guide_hull_chain = correction._select_hull_chain_from_endpoints


def _safe_normalize(v):
    arr = np.asarray(v, dtype=float)
    norm = float(np.linalg.norm(arr))
    if norm <= 1e-12:
        return np.zeros_like(arr, dtype=float)
    return arr / norm


def _orient_vectors_consistently(vectors, reference=None):
    values = np.asarray(vectors, dtype=float)
    if values.ndim != 2 or values.shape[1] != 3 or len(values) == 0:
        return np.empty((0, 3), dtype=float)

    oriented = np.zeros_like(values, dtype=float)
    reference_vec = None
    if reference is not None:
        reference_vec = _safe_normalize(reference)
        if np.linalg.norm(reference_vec) <= 1e-12:
            reference_vec = None

    previous = reference_vec
    for idx, value in enumerate(values):
        current = _safe_normalize(value)
        if np.linalg.norm(current) <= 1e-12:
            if previous is not None:
                current = previous.copy()
            elif reference_vec is not None:
                current = reference_vec.copy()
            else:
                current = np.array([0.0, 0.0, 1.0], dtype=float)

        if previous is not None and float(np.dot(current, previous)) < 0.0:
            current *= -1.0

        oriented[idx] = current
        previous = current

    return oriented


def _spline_scan_config(spline_start_side):
    side = str(spline_start_side).strip().lower()
    if side == "top":
        return {"side": side, "slice_axis": 1, "row_axis": 0, "descending": True}
    if side == "bottom":
        return {"side": side, "slice_axis": 1, "row_axis": 0, "descending": False}
    if side == "left":
        return {"side": side, "slice_axis": 0, "row_axis": 1, "descending": False}
    if side == "right":
        return {"side": side, "slice_axis": 0, "row_axis": 1, "descending": True}
    raise ValueError("spline_start_side must be one of: top, bottom, left, right")


def _normalize_slicing_method(slicing_method):
    method = str(slicing_method).strip().lower()
    if method == "axis":
        return "axis"
    if method == "surface_adaptive":
        return "surface_adaptive"
    raise ValueError(
        "slicing_method must be one of: axis, surface_adaptive"
    )


def _build_work_frame(points_world, paint_dir_world, scan_dir_world):
    p = np.asarray(points_world, dtype=float)
    origin = np.mean(p, axis=0)

    view_dir = _safe_normalize(paint_dir_world)
    if np.linalg.norm(view_dir) <= 1e-12:
        view_dir = np.array([0.0, 0.0, -1.0], dtype=float)

    surface_dir = _safe_normalize(-view_dir)
    row_dir = _safe_normalize(scan_dir_world)
    if np.linalg.norm(row_dir) <= 1e-12:
        row_dir = np.array([1.0, 0.0, 0.0], dtype=float)

    x_axis = row_dir - float(np.dot(row_dir, surface_dir)) * surface_dir
    if np.linalg.norm(x_axis) <= 1e-12:
        tmp = np.array([1.0, 0.0, 0.0], dtype=float)
        if abs(float(np.dot(tmp, surface_dir))) > 0.9:
            tmp = np.array([0.0, 1.0, 0.0], dtype=float)
        x_axis = tmp - float(np.dot(tmp, surface_dir)) * surface_dir
    x_axis = _safe_normalize(x_axis)

    y_axis = _safe_normalize(np.cross(surface_dir, x_axis))
    x_axis = _safe_normalize(np.cross(y_axis, surface_dir))
    r_work = np.column_stack([x_axis, y_axis, surface_dir])
    return origin, r_work, view_dir


def _axis_unit(axis_idx):
    axis = np.zeros(3, dtype=float)
    axis[int(axis_idx)] = 1.0
    return axis


def _slice_normal_direction(slice_axis_idx, descending):
    direction = _axis_unit(slice_axis_idx)
    if bool(descending):
        direction *= -1.0
    return direction


def _estimate_surface_normals_work(points_work, neighbor_count):
    pts = np.asarray(points_work, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 3 or len(pts) == 0:
        return np.empty((0, 3), dtype=float)
    if len(pts) < 3:
        return np.repeat(np.array([[0.0, 0.0, 1.0]], dtype=float), len(pts), axis=0)

    k = min(max(int(neighbor_count), 3), len(pts))
    tree = cKDTree(pts)
    _, indices = tree.query(pts, k=k)
    indices = np.asarray(indices, dtype=int)
    if indices.ndim == 1:
        indices = indices.reshape(-1, 1)

    normals = np.zeros_like(pts)
    for idx, neigh in enumerate(indices):
        local = pts[neigh]
        local = local - np.mean(local, axis=0, keepdims=True)
        try:
            _, _, vh = np.linalg.svd(local, full_matrices=False)
            normal = vh[-1]
        except np.linalg.LinAlgError:
            normal = np.array([0.0, 0.0, 1.0], dtype=float)
        normal = _safe_normalize(normal)
        normals[idx] = normal
    return normals


def _project_to_tangent_plane(vector, normal, fallback=None):
    normal = _safe_normalize(normal)
    v = np.asarray(vector, dtype=float)
    v = v - float(np.dot(v, normal)) * normal
    if np.linalg.norm(v) <= 1e-12:
        if fallback is None:
            fallback = np.array([1.0, 0.0, 0.0], dtype=float)
        v = np.asarray(fallback, dtype=float)
        v = v - float(np.dot(v, normal)) * normal
    if np.linalg.norm(v) <= 1e-12:
        v = np.array([1.0, 0.0, 0.0], dtype=float)
        v = v - float(np.dot(v, normal)) * normal
    return _safe_normalize(v)


def _surface_row_axis_from_guide(surface_normal, guide_tangent, row_axis_idx):
    surface_normal = _safe_normalize(surface_normal)
    guide_tangent = _project_to_tangent_plane(guide_tangent, surface_normal, fallback=_axis_unit(row_axis_idx))
    row_axis = np.cross(surface_normal, guide_tangent)
    row_axis = _project_to_tangent_plane(row_axis, surface_normal, fallback=_axis_unit(row_axis_idx))
    if float(np.dot(row_axis, _axis_unit(row_axis_idx))) < 0.0:
        row_axis *= -1.0
    return row_axis


def _select_longest_supported_strip(
    points_work,
    slice_axis_idx,
    row_axis_idx,
    strip_half_width,
    slice_bin_width,
    min_bin_points,
):
    pts = np.asarray(points_work, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 3 or len(pts) < 2:
        return None

    row_coords = pts[:, int(row_axis_idx)]
    slice_coords = pts[:, int(slice_axis_idx)]
    finite_mask = np.isfinite(row_coords) & np.isfinite(slice_coords)
    if np.count_nonzero(finite_mask) < 2:
        return None

    row_width = max(float(strip_half_width) * 2.0, 1e-4)
    slice_width = max(float(slice_bin_width), row_width, 1e-4)
    row_min = float(np.min(row_coords[finite_mask]))
    row_max = float(np.max(row_coords[finite_mask]))
    slice_min = float(np.min(slice_coords[finite_mask]))
    slice_max = float(np.max(slice_coords[finite_mask]))
    row_bin_count = max(int(np.floor((row_max - row_min) / row_width)) + 1, 1)
    slice_bin_count = max(int(np.floor((slice_max - slice_min) / slice_width)) + 1, 1)

    row_ids = np.zeros(len(pts), dtype=int)
    slice_ids = np.zeros(len(pts), dtype=int)
    row_ids[finite_mask] = np.floor(
        (row_coords[finite_mask] - row_min) / row_width
    ).astype(int)
    slice_ids[finite_mask] = np.floor(
        (slice_coords[finite_mask] - slice_min) / slice_width
    ).astype(int)
    row_ids[finite_mask] = np.clip(row_ids[finite_mask], 0, row_bin_count - 1)
    slice_ids[finite_mask] = np.clip(
        slice_ids[finite_mask], 0, slice_bin_count - 1
    )
    occupancy_counts = np.zeros((row_bin_count, slice_bin_count), dtype=int)
    np.add.at(occupancy_counts, (row_ids[finite_mask], slice_ids[finite_mask]), 1)

    threshold = max(int(min_bin_points), 1)
    occupied = occupancy_counts >= threshold
    median_row = float(np.median(row_coords[finite_mask]))
    best_score = None
    selected = None
    for row_bin in range(row_bin_count):
        neighbor_lo = max(0, row_bin - 1)
        neighbor_hi = min(row_bin_count, row_bin + 2)
        neighbor_count = neighbor_hi - neighbor_lo
        required_neighbors = min(2, neighbor_count)
        stable_cells = (
            np.count_nonzero(occupied[neighbor_lo:neighbor_hi], axis=0)
            >= required_neighbors
        )
        supported_bins = np.flatnonzero(stable_cells)
        if len(supported_bins) < 2:
            continue

        lower_bin = int(supported_bins[0])
        upper_bin = int(supported_bins[-1])
        span_bins = upper_bin - lower_bin
        point_support = int(
            np.sum(
                occupancy_counts[
                    neighbor_lo:neighbor_hi,
                    lower_bin : upper_bin + 1,
                ]
            )
        )
        stable_cell_count = int(
            np.count_nonzero(stable_cells[lower_bin : upper_bin + 1])
        )
        row_center = row_min + (float(row_bin) + 0.5) * row_width
        score = (
            span_bins,
            point_support,
            stable_cell_count,
            -abs(row_center - median_row),
        )
        if best_score is None or score > best_score:
            best_score = score
            selected = {
                "row_bin": int(row_bin),
                "neighbor_lo": int(neighbor_lo),
                "neighbor_hi": int(neighbor_hi),
                "lower_bin": lower_bin,
                "upper_bin": upper_bin,
                "threshold": int(threshold),
                "stable_cells": stable_cells,
            }

    if selected is None:
        return None

    neighbor_lo = selected["neighbor_lo"]
    neighbor_hi = selected["neighbor_hi"]
    lower_bin = selected["lower_bin"]
    upper_bin = selected["upper_bin"]
    threshold = selected["threshold"]
    occupied = occupancy_counts >= threshold

    lower_supported_rows = np.flatnonzero(occupied[neighbor_lo:neighbor_hi, lower_bin]) + neighbor_lo
    upper_supported_rows = np.flatnonzero(occupied[neighbor_lo:neighbor_hi, upper_bin]) + neighbor_lo
    lower_mask = np.isin(row_ids, lower_supported_rows) & (slice_ids == lower_bin) & finite_mask
    upper_mask = np.isin(row_ids, upper_supported_rows) & (slice_ids == upper_bin) & finite_mask
    if not np.any(lower_mask) or not np.any(upper_mask):
        return None

    supported_min = float(np.min(slice_coords[lower_mask]))
    supported_max = float(np.max(slice_coords[upper_mask]))
    strip_mask = (
        finite_mask
        & (row_ids >= neighbor_lo)
        & (row_ids < neighbor_hi)
        & (slice_coords >= supported_min)
        & (slice_coords <= supported_max)
    )
    strip_indices = np.flatnonzero(strip_mask).astype(int)
    if len(strip_indices) < 2:
        return None

    return {
        "indices": strip_indices,
        "center_value": float(np.median(row_coords[strip_indices])),
    }


def _longest_strip_hull_guide_curve(
    points_work,
    slice_axis_idx,
    row_axis_idx,
    descending,
    strip_half_width,
    guide_spacing,
    min_bin_points,
):
    pts = np.asarray(points_work, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 3 or len(pts) < 2:
        return None

    base_half_width = max(float(strip_half_width), 1e-6)
    strip_selection = _select_longest_supported_strip(
        pts,
        slice_axis_idx=slice_axis_idx,
        row_axis_idx=row_axis_idx,
        strip_half_width=base_half_width,
        slice_bin_width=max(float(guide_spacing), base_half_width * 2.0),
        min_bin_points=min_bin_points,
    )
    if strip_selection is None:
        return None

    strip_indices = np.asarray(strip_selection["indices"], dtype=int)
    strip_points = pts[strip_indices]
    center_value = float(strip_selection["center_value"])
    guide_2d_points = np.column_stack([
        strip_points[:, int(slice_axis_idx)],
        strip_points[:, 2],
    ])

    lower_slice = float(np.min(guide_2d_points[:, 0]))
    upper_slice = float(np.max(guide_2d_points[:, 0]))
    lower_candidates = guide_2d_points[np.isclose(guide_2d_points[:, 0], lower_slice)]
    upper_candidates = guide_2d_points[np.isclose(guide_2d_points[:, 0], upper_slice)]
    lower_endpoint = lower_candidates[int(np.argmax(lower_candidates[:, 1]))]
    upper_endpoint = upper_candidates[int(np.argmax(upper_candidates[:, 1]))]

    hull_chain = _select_guide_hull_chain(
        guide_2d_points,
        endpoint_a=lower_endpoint,
        endpoint_b=upper_endpoint,
    )
    if hull_chain is None or len(hull_chain) < 2:
        return None
    hull_chain = np.asarray(hull_chain, dtype=float)

    if bool(descending):
        if hull_chain[0, 0] < hull_chain[-1, 0]:
            hull_chain = hull_chain[::-1].copy()
    elif hull_chain[0, 0] > hull_chain[-1, 0]:
        hull_chain = hull_chain[::-1].copy()

    hull_arc = np.concatenate(
        [
            [0.0],
            np.cumsum(np.linalg.norm(np.diff(hull_chain, axis=0), axis=1)),
        ]
    )
    keep_hull = np.concatenate([[True], np.diff(hull_arc) > 1e-9])
    hull_chain = hull_chain[keep_hull]
    hull_arc = hull_arc[keep_hull]
    if len(hull_chain) < 2 or float(hull_arc[-1]) <= 1e-9:
        return None

    guide_step = max(float(guide_spacing), 1e-4)
    total_length = float(hull_arc[-1])
    # The visualized guide always covers the complete upper hull, independent
    # of where slicing starts.
    sample_arc = np.arange(0.0, total_length, guide_step, dtype=float)
    if sample_arc.size == 0 or not np.isclose(sample_arc[-1], total_length, atol=1e-9):
        sample_arc = np.concatenate([sample_arc, [total_length]])
    sampled_2d = np.column_stack(
        [
            np.interp(sample_arc, hull_arc, hull_chain[:, 0]),
            np.interp(sample_arc, hull_arc, hull_chain[:, 1]),
        ]
    )
    if len(sampled_2d) < 1:
        return None

    hull_chain_points = np.zeros((len(hull_chain), 3), dtype=float)
    hull_chain_points[:, int(row_axis_idx)] = center_value
    hull_chain_points[:, int(slice_axis_idx)] = hull_chain[:, 0]
    hull_chain_points[:, 2] = hull_chain[:, 1]

    query_points = np.zeros((len(sampled_2d), 3), dtype=float)
    query_points[:, int(row_axis_idx)] = center_value
    query_points[:, int(slice_axis_idx)] = sampled_2d[:, 0]
    query_points[:, 2] = sampled_2d[:, 1]

    guide_points = query_points.copy()

    keep = np.concatenate([
        [True],
        np.linalg.norm(np.diff(guide_points[:, [slice_axis_idx, 2]], axis=0), axis=1) > 1e-6,
    ])
    guide_points = guide_points[keep]
    if len(guide_points) < 1:
        return None

    return {
        "points": guide_points,
        "strip_points": strip_points,
        "hull_chain_points": hull_chain_points,
        "hull_chain_2d": hull_chain,
        "hull_chain_arc": hull_arc,
        "center_value": center_value,
    }


def _world_direction(work_to_world, origin_work, direction_work):
    origin_world = work_to_world(np.asarray(origin_work, dtype=float).reshape(1, 3))[0]
    tip_world = work_to_world((np.asarray(origin_work, dtype=float) + np.asarray(direction_work, dtype=float)).reshape(1, 3))[0]
    return _safe_normalize(tip_world - origin_world)


def _make_profile(
    row_index,
    slice_position,
    slice_points_work,
    plane_origin_work,
    plane_normal_work,
    plane_row_axis_work,
    plane_other_axis_work,
    work_to_world,
    slicing_method,
    extra=None,
):
    plane_origin_work = np.asarray(plane_origin_work, dtype=float)
    plane_normal_work = _safe_normalize(plane_normal_work)
    plane_row_axis_work = _safe_normalize(plane_row_axis_work)
    plane_other_axis_work = _safe_normalize(plane_other_axis_work)
    plane_origin_world = work_to_world(plane_origin_work.reshape(1, 3))[0]
    profile = {
        "row_index": int(row_index),
        "slice_position": float(slice_position),
        "slice_points_work": np.asarray(slice_points_work, dtype=float).copy(),
        "slice_points_world": work_to_world(slice_points_work),
        "slice_plane_origin_work": plane_origin_work.copy(),
        "slice_plane_normal_work": plane_normal_work.copy(),
        "slice_plane_row_axis_work": plane_row_axis_work.copy(),
        "slice_plane_other_axis_work": plane_other_axis_work.copy(),
        "slice_plane_origin_world": plane_origin_world,
        "slice_plane_row_axis_world": _world_direction(work_to_world, plane_origin_work, plane_row_axis_work),
        "slice_plane_other_axis_world": _world_direction(work_to_world, plane_origin_work, plane_other_axis_work),
        "slicing_method": str(slicing_method),
    }
    if extra:
        profile.update(extra)
    return profile


def _vector_angle_degrees(vector_a, vector_b):
    a = _safe_normalize(vector_a)
    b = _safe_normalize(vector_b)
    if np.linalg.norm(a) <= 1e-12 or np.linalg.norm(b) <= 1e-12:
        return 0.0
    cosine = float(np.clip(np.dot(a, b), -1.0, 1.0))
    return float(np.degrees(np.arccos(cosine)))


def _slerp_unit_vector(vector_a, vector_b, fraction):
    a = _safe_normalize(vector_a)
    b = _safe_normalize(vector_b)
    t = float(np.clip(fraction, 0.0, 1.0))
    if np.linalg.norm(a) <= 1e-12:
        return b
    if np.linalg.norm(b) <= 1e-12:
        return a

    cosine = float(np.clip(np.dot(a, b), -1.0, 1.0))
    if cosine >= 1.0 - 1e-10:
        return _safe_normalize((1.0 - t) * a + t * b)

    if cosine <= -1.0 + 1e-10:
        basis = np.zeros(3, dtype=float)
        basis[int(np.argmin(np.abs(a)))] = 1.0
        orthogonal = _safe_normalize(basis - float(np.dot(basis, a)) * a)
        return _safe_normalize(
            np.cos(np.pi * t) * a + np.sin(np.pi * t) * orthogonal
        )

    angle = float(np.arccos(cosine))
    sine = float(np.sin(angle))
    return _safe_normalize(
        (np.sin((1.0 - t) * angle) / sine) * a
        + (np.sin(t * angle) / sine) * b
    )


def _plane_slab_indices(
    points_work,
    plane_origin_work,
    plane_normal_work,
    band_half_width,
    min_band_points,
):
    pts = np.asarray(points_work, dtype=float)
    origin = np.asarray(plane_origin_work, dtype=float).reshape(3)
    normal = _safe_normalize(plane_normal_work)
    if (
        pts.ndim != 2
        or pts.shape[1] != 3
        or len(pts) == 0
        or np.linalg.norm(normal) <= 1e-12
    ):
        return np.empty(0, dtype=int)

    signed_distance = (pts - origin.reshape(1, 3)) @ normal
    required_points = max(int(min_band_points), 2)
    best_indices = np.empty(0, dtype=int)
    for width_scale in (1.0, 1.5, 2.0, 3.0, 4.0, 6.0, 8.0):
        width = max(float(band_half_width), 1e-6) * width_scale
        raw_indices = np.flatnonzero(np.abs(signed_distance) <= width).astype(int)
        if len(raw_indices) > len(best_indices):
            best_indices = raw_indices
        if len(raw_indices) >= required_points:
            break
    return best_indices


def _curvature_subdivision_count(angle_degrees, target_angle_degrees):
    angle = max(float(angle_degrees), 0.0)
    target = float(target_angle_degrees)
    if target <= 0.0 or angle < target:
        return 1

    ratio = angle / target
    nearest_integer = round(ratio)
    if abs(ratio - nearest_integer) <= 1e-9:
        ratio = float(nearest_integer)
    return max(int(np.floor(ratio)), 2)


def _insert_curvature_slice_profiles(
    base_profiles,
    points_work,
    work_to_world,
    band_half_width,
    min_band_points,
    target_angle_degrees,
):
    profiles = [dict(profile) for profile in base_profiles]

    if len(profiles) < 2 or float(target_angle_degrees) <= 0.0:
        for row_index, profile in enumerate(profiles):
            profile["row_index"] = int(row_index)
        return profiles

    densified_profiles = []
    for pair_index in range(len(profiles) - 1):
        current = profiles[pair_index]
        following = profiles[pair_index + 1]
        densified_profiles.append(current)

        current_normal = _safe_normalize(
            current.get("slice_plane_normal_work", [])
        )
        following_normal = _safe_normalize(
            following.get("slice_plane_normal_work", [])
        )
        angle_degrees = _vector_angle_degrees(
            current_normal, following_normal
        )
        subdivision_count = _curvature_subdivision_count(
            angle_degrees, target_angle_degrees
        )
        if subdivision_count <= 1:
            continue

        current_origin = np.asarray(
            current.get("slice_plane_origin_work", []), dtype=float
        ).reshape(-1)
        following_origin = np.asarray(
            following.get("slice_plane_origin_work", []), dtype=float
        ).reshape(-1)
        if current_origin.size != 3 or following_origin.size != 3:
            continue

        current_row_axis = _safe_normalize(
            current.get("slice_plane_row_axis_work", [])
        )
        following_row_axis = _safe_normalize(
            following.get("slice_plane_row_axis_work", [])
        )
        current_other_axis = _safe_normalize(
            current.get("slice_plane_other_axis_work", [])
        )
        following_other_axis = _safe_normalize(
            following.get("slice_plane_other_axis_work", [])
        )
        current_position = float(current.get("slice_position", pair_index))
        following_position = float(
            following.get("slice_position", pair_index + 1)
        )

        for subdivision_index in range(1, subdivision_count):
            fraction = float(subdivision_index) / float(subdivision_count)
            plane_origin_work = (
                (1.0 - fraction) * current_origin
                + fraction * following_origin
            )
            plane_normal_work = _slerp_unit_vector(
                current_normal, following_normal, fraction
            )

            row_axis_hint = _slerp_unit_vector(
                current_row_axis, following_row_axis, fraction
            )
            plane_row_axis_work = (
                row_axis_hint
                - float(np.dot(row_axis_hint, plane_normal_work))
                * plane_normal_work
            )
            if np.linalg.norm(plane_row_axis_work) <= 1e-9:
                plane_row_axis_work = (
                    current_row_axis
                    - float(np.dot(current_row_axis, plane_normal_work))
                    * plane_normal_work
                )
            if np.linalg.norm(plane_row_axis_work) <= 1e-9:
                plane_row_axis_work = (
                    following_row_axis
                    - float(np.dot(following_row_axis, plane_normal_work))
                    * plane_normal_work
                )
            if np.linalg.norm(plane_row_axis_work) <= 1e-9:
                continue
            plane_row_axis_work = _safe_normalize(plane_row_axis_work)

            other_axis_hint = _slerp_unit_vector(
                current_other_axis, following_other_axis, fraction
            )
            plane_other_axis_work = _safe_normalize(
                np.cross(plane_normal_work, plane_row_axis_work)
            )
            if np.linalg.norm(plane_other_axis_work) <= 1e-9:
                continue
            if float(np.dot(plane_other_axis_work, other_axis_hint)) < 0.0:
                plane_other_axis_work *= -1.0

            slice_indices = _plane_slab_indices(
                points_work,
                plane_origin_work=plane_origin_work,
                plane_normal_work=plane_normal_work,
                band_half_width=band_half_width,
                min_band_points=min_band_points,
            )
            if len(slice_indices) < max(int(min_band_points), 2):
                continue

            slice_position = (
                (1.0 - fraction) * current_position
                + fraction * following_position
            )
            densified_profiles.append(
                _make_profile(
                    row_index=-1,
                    slice_position=slice_position,
                    slice_points_work=np.asarray(points_work, dtype=float)[
                        slice_indices
                    ].copy(),
                    plane_origin_work=plane_origin_work,
                    plane_normal_work=plane_normal_work,
                    plane_row_axis_work=plane_row_axis_work,
                    plane_other_axis_work=plane_other_axis_work,
                    work_to_world=work_to_world,
                    slicing_method="surface_adaptive",
                )
            )

    densified_profiles.append(profiles[-1])
    for row_index, profile in enumerate(densified_profiles):
        profile["row_index"] = int(row_index)
    return densified_profiles


def _generate_axis_slice_profiles(pw, work_to_world, slice_cfg, spline_spacing, spline_half_width, spline_start_offset):
    slice_axis_idx = int(slice_cfg["slice_axis"])
    row_axis_idx = int(slice_cfg["row_axis"])
    descending = bool(slice_cfg["descending"])

    slice_min = float(np.min(pw[:, slice_axis_idx]))
    slice_max = float(np.max(pw[:, slice_axis_idx]))
    if not np.isfinite([slice_min, slice_max]).all() or slice_max < slice_min:
        return []

    if descending:
        first_slice = slice_max - float(spline_start_offset)
        slice_positions = np.arange(first_slice, slice_min - 1e-12, -float(spline_spacing))
        plane_normal_work = _slice_normal_direction(slice_axis_idx, descending=True)
    else:
        first_slice = slice_min + float(spline_start_offset)
        slice_positions = np.arange(first_slice, slice_max + 1e-12, float(spline_spacing))
        plane_normal_work = _slice_normal_direction(slice_axis_idx, descending=False)

    row_axis_work = _axis_unit(row_axis_idx)
    plane_other_axis_work = _safe_normalize(np.cross(plane_normal_work, row_axis_work))

    profiles = []
    for row_index, slice_center in enumerate(slice_positions):
        indices = np.flatnonzero(np.abs(pw[:, slice_axis_idx] - float(slice_center)) <= float(spline_half_width)).astype(int)
        if len(indices) < 2:
            continue
        slice_points_work = pw[indices].copy()
        slice_points_work[:, slice_axis_idx] = float(slice_center)
        plane_origin_work = np.mean(slice_points_work, axis=0)
        profiles.append(
            _make_profile(
                row_index=row_index,
                slice_position=float(slice_center),
                slice_points_work=slice_points_work,
                plane_origin_work=plane_origin_work,
                plane_normal_work=plane_normal_work,
                plane_row_axis_work=row_axis_work,
                plane_other_axis_work=plane_other_axis_work,
                work_to_world=work_to_world,
                slicing_method="axis",
            )
        )
    return profiles


def _generate_surface_adaptive_slice_profiles(
    pw,
    work_to_world,
    slice_cfg,
    spline_spacing,
    spline_half_width,
    spline_start_offset,
    geodesic_normal_neighbor_count,
    geodesic_band_half_width,
    guide_extension_min_bin_points,
    geodesic_min_band_points,
    curvature_slice_angle_deg,
):
    slice_axis_idx = int(slice_cfg["slice_axis"])
    row_axis_idx = int(slice_cfg["row_axis"])
    descending = bool(slice_cfg["descending"])

    normals_work = _estimate_surface_normals_work(pw, neighbor_count=geodesic_normal_neighbor_count)
    surface_tree = cKDTree(pw)

    band_half_width = float(geodesic_band_half_width)
    if band_half_width <= 0.0:
        band_half_width = float(spline_half_width)
    guide = _longest_strip_hull_guide_curve(
        pw,
        slice_axis_idx=slice_axis_idx,
        row_axis_idx=row_axis_idx,
        descending=descending,
        strip_half_width=band_half_width,
        guide_spacing=spline_spacing,
        min_bin_points=guide_extension_min_bin_points,
    )
    if guide is None:
        return []

    guide_curve_points = np.asarray(guide.get("points", []), dtype=float)
    hull_chain = np.asarray(guide.get("hull_chain_2d", []), dtype=float)
    hull_arc = np.asarray(guide.get("hull_chain_arc", []), dtype=float)
    if (
        guide_curve_points.ndim != 2
        or guide_curve_points.shape[1] != 3
        or len(guide_curve_points) < 2
        or hull_chain.ndim != 2
        or hull_chain.shape[1] != 2
        or len(hull_chain) < 2
        or hull_arc.shape != (len(hull_chain),)
    ):
        return []

    guide_step = max(float(spline_spacing), 1e-4)
    start_offset = max(float(spline_start_offset), 0.0)
    total_length = float(hull_arc[-1])
    if total_length < start_offset:
        return []
    # Slice origins are sampled separately from the full guide curve.
    remaining_length = max(total_length - start_offset, 0.0)
    tolerance = max(1e-9, guide_step * 1e-6)
    sample_count = int(np.floor((remaining_length + tolerance) / guide_step)) + 1
    slice_arc = start_offset + np.arange(sample_count, dtype=float) * guide_step
    slice_arc = slice_arc[slice_arc <= total_length + tolerance]
    slice_arc = np.clip(slice_arc, start_offset, total_length)
    slice_points_2d = np.column_stack(
        [
            np.interp(slice_arc, hull_arc, hull_chain[:, 0]),
            np.interp(slice_arc, hull_arc, hull_chain[:, 1]),
        ]
    )

    slice_guide_points = np.zeros((len(slice_points_2d), 3), dtype=float)
    slice_guide_points[:, row_axis_idx] = float(guide.get("center_value", 0.0))
    slice_guide_points[:, slice_axis_idx] = slice_points_2d[:, 0]
    slice_guide_points[:, 2] = slice_points_2d[:, 1]
    _, slice_guide_indices = surface_tree.query(slice_guide_points, k=1)
    slice_guide_indices = np.asarray(slice_guide_indices, dtype=int).reshape(-1)
    slice_guide_normals = normals_work[slice_guide_indices]
    slice_guide_normals = _orient_vectors_consistently(
        slice_guide_normals,
        reference=np.array([0.0, 0.0, 1.0], dtype=float),
    )

    desired_axis = _slice_normal_direction(slice_axis_idx, descending)
    scan_row_axis = _axis_unit(row_axis_idx)
    outward_reference = _axis_unit(2)
    slice_guide_tangents = []
    for idx in range(len(slice_guide_points)):
        if len(slice_guide_points) == 1:
            tangent_source = desired_axis
        elif idx == 0:
            tangent_source = slice_guide_points[1] - slice_guide_points[0]
        elif idx == len(slice_guide_points) - 1:
            tangent_source = slice_guide_points[-1] - slice_guide_points[-2]
        else:
            tangent_source = slice_guide_points[idx + 1] - slice_guide_points[idx - 1]
        tangent = _safe_normalize(tangent_source)
        if np.linalg.norm(tangent) <= 1e-12:
            tangent = desired_axis.copy()
        if float(np.dot(tangent, desired_axis)) < 0.0:
            tangent *= -1.0
        slice_guide_tangents.append(tangent)
    slice_guide_tangents = np.asarray(slice_guide_tangents, dtype=float)

    min_band_points = max(int(geodesic_min_band_points), 2)
    profiles = []
    for row_index, (origin_work, plane_normal_work, surface_normal_work) in enumerate(
        zip(slice_guide_points, slice_guide_tangents, slice_guide_normals)
    ):
        origin_work = np.asarray(origin_work, dtype=float)
        plane_normal_work = _safe_normalize(plane_normal_work)
        fallback_surface_normal_work = _safe_normalize(surface_normal_work)
        plane_row_axis_work = scan_row_axis - float(np.dot(scan_row_axis, plane_normal_work)) * plane_normal_work
        if np.linalg.norm(plane_row_axis_work) <= 1e-9:
            plane_row_axis_work = _surface_row_axis_from_guide(
                fallback_surface_normal_work,
                plane_normal_work,
                row_axis_idx=row_axis_idx,
            )
        else:
            plane_row_axis_work = _safe_normalize(plane_row_axis_work)

        plane_other_axis_work = np.cross(plane_normal_work, plane_row_axis_work)
        if np.linalg.norm(plane_other_axis_work) <= 1e-9:
            plane_other_axis_work = fallback_surface_normal_work
        else:
            plane_other_axis_work = _safe_normalize(plane_other_axis_work)
        if float(np.dot(plane_other_axis_work, outward_reference)) < 0.0:
            plane_other_axis_work *= -1.0

        best_indices = _plane_slab_indices(
            pw,
            plane_origin_work=origin_work,
            plane_normal_work=plane_normal_work,
            band_half_width=band_half_width,
            min_band_points=min_band_points,
        )
        if len(best_indices) < min_band_points:
            continue

        extra = {
            "guide_curve_points_world": work_to_world(guide_curve_points),
            "guide_hull_points_world": work_to_world(np.asarray(guide.get("hull_chain_points", []), dtype=float))
            if len(np.asarray(guide.get("hull_chain_points", []), dtype=float))
            else np.empty((0, 3), dtype=float),
            "guide_strip_points_world": work_to_world(np.asarray(guide.get("strip_points", []), dtype=float))
            if len(np.asarray(guide.get("strip_points", []), dtype=float))
            else np.empty((0, 3), dtype=float),
        }
        profiles.append(
            _make_profile(
                row_index=row_index,
                slice_position=float(row_index),
                slice_points_work=pw[best_indices].copy(),
                plane_origin_work=origin_work,
                plane_normal_work=plane_normal_work,
                plane_row_axis_work=plane_row_axis_work,
                plane_other_axis_work=plane_other_axis_work,
                work_to_world=work_to_world,
                slicing_method="surface_adaptive",
                extra=extra,
            )
        )
    return _insert_curvature_slice_profiles(
        profiles,
        points_work=pw,
        work_to_world=work_to_world,
        band_half_width=band_half_width,
        min_band_points=min_band_points,
        target_angle_degrees=curvature_slice_angle_deg,
    )


def generate_surface_spline_rows(
    point_cloud,
    paint_dir,
    scan_dir,
    spline_spacing=0.06,
    spline_half_width=0.002,
    spline_start_side="top",
    spline_start_offset=0.0,
    slicing_method="axis",
    geodesic_normal_neighbor_count=20,
    geodesic_band_half_width=0.0,
    guide_extension_min_bin_points=8,
    geodesic_min_band_points=8,
    curvature_slice_angle_deg=20.0,
):
    method = _normalize_slicing_method(slicing_method)

    def _empty_result():
        return {
            "work_frame_world": None,
            "work_origin_world": None,
            "view_dir_world": None,
            "slicing_method": method,
            "spline_start_side": str(spline_start_side),
            "slice_profiles": [],
        }

    points = np.asarray(point_cloud, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 2:
        return _empty_result()

    origin, r_work, view_dir = _build_work_frame(points, paint_dir, scan_dir)
    work_to_world = lambda x: np.asarray(x, dtype=float) @ r_work.T + origin
    points_work = (points - origin) @ r_work

    slice_cfg = _spline_scan_config(spline_start_side)
    spline_spacing = max(float(spline_spacing), 1e-4)
    spline_half_width = max(float(spline_half_width), 1e-6)
    spline_start_offset = max(float(spline_start_offset), 0.0)
    if method == "axis":
        slice_profiles = _generate_axis_slice_profiles(
            points_work,
            work_to_world=work_to_world,
            slice_cfg=slice_cfg,
            spline_spacing=spline_spacing,
            spline_half_width=spline_half_width,
            spline_start_offset=spline_start_offset,
        )
    else:
        slice_profiles = _generate_surface_adaptive_slice_profiles(
            points_work,
            work_to_world=work_to_world,
            slice_cfg=slice_cfg,
            spline_spacing=spline_spacing,
            spline_half_width=spline_half_width,
            spline_start_offset=spline_start_offset,
            geodesic_normal_neighbor_count=geodesic_normal_neighbor_count,
            geodesic_band_half_width=geodesic_band_half_width,
            guide_extension_min_bin_points=guide_extension_min_bin_points,
            geodesic_min_band_points=geodesic_min_band_points,
            curvature_slice_angle_deg=curvature_slice_angle_deg,
        )
    return {
        "work_frame_world": r_work,
        "work_origin_world": origin,
        "view_dir_world": view_dir,
        "slicing_method": method,
        "spline_start_side": str(slice_cfg["side"]),
        "slice_profiles": slice_profiles,
    }


__all__ = ["generate_surface_spline_rows"]
