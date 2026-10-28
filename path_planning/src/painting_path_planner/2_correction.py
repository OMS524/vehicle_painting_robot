#!/usr/bin/env python3

import heapq

import numpy as np

try:
    from scipy.linalg import solveh_banded
except Exception:  # pragma: no cover - fallback when SciPy linalg is unavailable
    solveh_banded = None


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
        return {"side": side, "slice_axis": 1, "row_axis": 0, "descending": True}
    if side == "bottom":
        return {"side": side, "slice_axis": 1, "row_axis": 0, "descending": False}
    if side == "left":
        return {"side": side, "slice_axis": 0, "row_axis": 1, "descending": False}
    if side == "right":
        return {"side": side, "slice_axis": 0, "row_axis": 1, "descending": True}
    raise ValueError(
        "spline_start_side must be one of: top, bottom, left, right"
    )


def _cross_2d(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


def _preserve_chain_endpoints(chain_uw, endpoint_a, endpoint_b):
    chain = np.asarray(chain_uw, float)
    if chain.ndim != 2 or chain.shape[1] != 2 or len(chain) < 2:
        return np.empty((0, 2), float)

    a = np.asarray(endpoint_a, float).reshape(2)
    b = np.asarray(endpoint_b, float).reshape(2)

    preserved = chain.copy()
    preserved[0] = a
    preserved[-1] = b
    return preserved


def _quantize_unique_points_2d(points_uw, step):
    pts = np.asarray(points_uw, float)
    if pts.ndim != 2 or pts.shape[1] != 2 or len(pts) == 0:
        return np.empty((0, 2), float)

    step = max(float(step), 1e-6)
    origin = np.min(pts, axis=0)
    keys = np.rint((pts - origin[None, :]) / step).astype(int)
    _, unique_idx = np.unique(keys, axis=0, return_index=True)
    unique_idx = np.sort(unique_idx)
    return pts[unique_idx]


def _graph_components(adjacency):
    n = len(adjacency)
    if n == 0:
        return []

    visited = np.zeros(n, dtype=bool)
    components = []
    for start in range(n):
        if visited[start]:
            continue
        stack = [start]
        visited[start] = True
        comp = []
        while stack:
            node = stack.pop()
            comp.append(node)
            for nei, _ in adjacency[node]:
                if visited[nei]:
                    continue
                visited[nei] = True
                stack.append(nei)
        components.append(np.asarray(comp, dtype=int))
    return components


def _dijkstra_shortest_paths(adjacency, start_idx):
    n = len(adjacency)
    dist = np.full(n, np.inf, dtype=float)
    prev = np.full(n, -1, dtype=int)
    dist[int(start_idx)] = 0.0
    heap = [(0.0, int(start_idx))]

    while heap:
        cur_dist, node = heapq.heappop(heap)
        if cur_dist > dist[node] + 1e-12:
            continue
        for nei, weight in adjacency[node]:
            cand = cur_dist + float(weight)
            if cand + 1e-12 < dist[nei]:
                dist[nei] = cand
                prev[nei] = node
                heapq.heappush(heap, (cand, nei))
    return dist, prev


def _component_longest_path(points_uw, adjacency, component_ids):
    component = np.asarray(component_ids, dtype=int).reshape(-1)
    if len(component) < 2:
        return np.empty((0, 2), float)

    comp_set = set(int(v) for v in component)
    sub_adj = [
        [(nei, w) for nei, w in edges if nei in comp_set] if idx in comp_set else []
        for idx, edges in enumerate(adjacency)
    ]

    seed = int(component[int(np.argmin(points_uw[component, 0]))])
    dist0, _ = _dijkstra_shortest_paths(sub_adj, seed)
    finite0 = np.isfinite(dist0)
    if not np.any(finite0):
        path = points_uw[component]
        order = np.argsort(path[:, 0], kind="mergesort")
        return path[order]

    a = int(np.nanargmax(np.where(finite0, dist0, -np.inf)))
    dist_a, prev = _dijkstra_shortest_paths(sub_adj, a)
    finite_a = np.isfinite(dist_a)
    if not np.any(finite_a):
        path = points_uw[component]
        order = np.argsort(path[:, 0], kind="mergesort")
        return path[order]

    b = int(np.nanargmax(np.where(finite_a, dist_a, -np.inf)))
    path_ids = [b]
    cur = b
    while cur != a and cur >= 0:
        cur = int(prev[cur])
        if cur < 0:
            break
        path_ids.append(cur)
    if path_ids[-1] != a:
        path = points_uw[component]
        order = np.argsort(path[:, 0], kind="mergesort")
        return path[order]

    path_ids.reverse()
    return points_uw[np.asarray(path_ids, dtype=int)]


def _polyline_arclength_2d(points_uw):
    pts = np.asarray(points_uw, float)
    if pts.ndim != 2 or pts.shape[1] != 2 or len(pts) == 0:
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


def _resample_polyline_linear_2d(points_uw, point_spacing):
    pts = np.asarray(points_uw, float)
    if pts.ndim != 2 or pts.shape[1] != 2 or len(pts) < 2:
        return None

    arc = _polyline_arclength_2d(pts)
    keep = np.concatenate([[True], np.diff(arc) > 1e-9])
    pts = pts[keep]
    arc = arc[keep]
    if len(pts) < 2:
        return None
    if float(arc[-1]) <= 1e-9:
        return pts.copy()

    sample_arc = _sample_arclength_grid(arc[-1], point_spacing)
    sampled = np.column_stack(
        [
            np.interp(sample_arc, arc, pts[:, 0]),
            np.interp(sample_arc, arc, pts[:, 1]),
        ]
    )
    sampled[0] = pts[0]
    sampled[-1] = pts[-1]
    return sampled


def _compute_local_tangent_vectors_2d(sampled_uw):
    samples = np.asarray(sampled_uw, float)
    if samples.ndim != 2 or samples.shape[1] != 2 or len(samples) == 0:
        return np.empty((0, 2), float)

    n = len(samples)
    tangents = np.zeros((n, 2), dtype=float)
    for idx in range(n):
        if n == 1:
            vec = np.array([1.0, 0.0], dtype=float)
        elif idx == 0:
            vec = samples[1] - samples[0]
        elif idx == n - 1:
            vec = samples[-1] - samples[-2]
        else:
            vec = samples[idx + 1] - samples[idx - 1]

        norm = float(np.linalg.norm(vec))
        if norm <= 1e-12:
            if idx > 0:
                vec = samples[idx] - samples[idx - 1]
                norm = float(np.linalg.norm(vec))
            if norm <= 1e-12 and idx + 1 < n:
                vec = samples[idx + 1] - samples[idx]
                norm = float(np.linalg.norm(vec))
            if norm <= 1e-12:
                vec = np.array([1.0, 0.0], dtype=float)
                norm = 1.0

        tangents[idx] = vec / norm
    return tangents


def _compute_tangent_window_support_distances_2d(
    sampled_uw,
    observed_points_uw,
    tangent_half_width,
):
    """Compute nearest raw-point distance inside a local tangent-direction window."""
    samples = np.asarray(sampled_uw, float)
    observed = np.asarray(observed_points_uw, float)
    if samples.ndim != 2 or samples.shape[1] != 2 or len(samples) == 0:
        return np.full(0, np.inf, dtype=float)
    if observed.ndim != 2 or observed.shape[1] != 2 or len(observed) == 0:
        return np.full(len(samples), np.inf, dtype=float)

    tangents = _compute_local_tangent_vectors_2d(samples)
    half_width = max(float(tangent_half_width), 1e-6)

    nearest_distances = np.full(len(samples), np.inf, dtype=float)
    for idx, sample in enumerate(samples):
        tangent = tangents[idx]
        local_offsets = observed - sample[None, :]
        tangent_coords = np.abs(local_offsets @ tangent)
        candidate_mask = tangent_coords <= half_width
        if not np.any(candidate_mask):
            continue

        candidate_points = observed[candidate_mask]
        nearest_dist = float(
            np.min(np.linalg.norm(candidate_points - sample[None, :], axis=1))
        )
        nearest_distances[idx] = nearest_dist

    return nearest_distances


def _false_runs(mask):
    runs = []
    start = None
    for idx, value in enumerate(np.asarray(mask, dtype=bool)):
        if not value and start is None:
            start = idx
        elif value and start is not None:
            runs.append((start, idx - 1))
            start = None
    if start is not None:
        runs.append((start, len(mask) - 1))
    return runs


def _build_anchor_mask_from_support(
    support_mask, transition_count, endpoint_transition_count=0
):
    support_mask = np.asarray(support_mask, dtype=bool).reshape(-1)
    anchor_mask = support_mask.copy()
    n = len(anchor_mask)
    if n == 0:
        return anchor_mask

    k = max(int(transition_count), 0)
    for start, end in _false_runs(support_mask):
        lo = max(0, start - k)
        hi = min(n - 1, end + k)
        anchor_mask[lo : hi + 1] = False

    endpoint_k = min(max(int(endpoint_transition_count), 0), n)
    if endpoint_k > 0:
        anchor_mask[:endpoint_k] = False
        anchor_mask[n - endpoint_k :] = False
    else:
        anchor_mask[0] = True
        anchor_mask[-1] = True
    return anchor_mask


def _solve_weighted_bending_axis(
    target,
    weights,
    edge_weights,
    lambda_data,
    lambda_stretch,
    lambda_bend,
    lambda_end,
    regularizer,
):
    target = np.asarray(target, float).reshape(-1)
    weights = np.asarray(weights, float).reshape(-1)
    edge_weights = np.asarray(edge_weights, float).reshape(-1)
    n = len(target)
    if n == 0:
        return np.zeros(0, dtype=float)
    if n == 1:
        return target.copy()

    if len(weights) != n:
        weights = np.zeros(n, dtype=float)
    if len(edge_weights) != n - 1:
        edge_weights = np.zeros(max(n - 1, 0), dtype=float)

    main = np.zeros(n, dtype=float)
    off1 = np.zeros(max(n - 1, 0), dtype=float)
    off2 = np.zeros(max(n - 2, 0), dtype=float)
    rhs = np.zeros(n, dtype=float)

    reg = max(float(regularizer), 0.0)
    if reg > 0.0:
        main += reg

    if np.any(weights > 0.0) and float(lambda_data) > 0.0:
        w = np.clip(weights, 0.0, None)
        lam = float(lambda_data)
        main += lam * w
        rhs += lam * w * target

    if float(lambda_end) > 0.0:
        lam = float(lambda_end)
        main[0] += lam
        main[-1] += lam
        rhs[0] += lam * target[0]
        rhs[-1] += lam * target[-1]

    if n >= 2 and np.any(edge_weights > 0.0) and float(lambda_stretch) > 0.0:
        lam = float(lambda_stretch)
        e = np.clip(edge_weights, 0.0, None)
        target_delta = target[1:] - target[:-1]
        main[:-1] += lam * e
        main[1:] += lam * e
        off1 += -lam * e
        rhs[:-1] += -lam * e * target_delta
        rhs[1:] += lam * e * target_delta

    if n >= 3 and float(lambda_bend) > 0.0:
        lam = float(lambda_bend)
        for idx in range(n - 2):
            main[idx] += lam
            main[idx + 1] += 4.0 * lam
            main[idx + 2] += lam
            off1[idx] += -2.0 * lam
            off1[idx + 1] += -2.0 * lam
            off2[idx] += lam

    if solveh_banded is not None:
        ab = np.zeros((3, n), dtype=float)
        ab[2, :] = main
        if n >= 2:
            ab[1, 1:] = off1
        if n >= 3:
            ab[0, 2:] = off2
        try:
            return solveh_banded(ab, rhs, lower=False, check_finite=False)
        except Exception:
            pass

    system = np.diag(main)
    if n >= 2:
        idx = np.arange(n - 1)
        system[idx, idx + 1] = off1
        system[idx + 1, idx] = off1
    if n >= 3:
        idx = np.arange(n - 2)
        system[idx, idx + 2] = off2
        system[idx + 2, idx] = off2

    try:
        return np.linalg.solve(system, rhs)
    except np.linalg.LinAlgError:
        return np.linalg.lstsq(system, rhs, rcond=None)[0]


def _solve_hull_regularized_curve_2d(
    chain_uw,
    observed_points_uw,
    point_spacing,
    support_tangent_half_width=0.0025,
    support_anchor_max_distance=0.0025,
    transition_count=3,
    endpoint_transition_count=0,
    lambda_data=40.0,
    lambda_stretch=6.0,
    lambda_bend=1.0,
    lambda_end=200.0,
    regularizer=1e-9,
):
    base = _resample_polyline_linear_2d(chain_uw, point_spacing)
    if base is None or len(base) < 2:
        return None, None

    base = np.asarray(base, float)
    n = len(base)
    if n < 3:
        return base.copy(), np.empty((0, 2), float)

    support_distances = _compute_tangent_window_support_distances_2d(
        base,
        observed_points_uw,
        tangent_half_width=float(support_tangent_half_width),
    )
    support_mask = support_distances <= float(support_anchor_max_distance)
    supported = base[support_mask]
    anchor_mask = _build_anchor_mask_from_support(
        support_mask,
        int(transition_count),
        int(endpoint_transition_count),
    )
    weights = anchor_mask.astype(float)
    edge_weights = np.minimum(weights[:-1], weights[1:])

    corrected = np.zeros_like(base)
    corrected[:, 0] = _solve_weighted_bending_axis(
        base[:, 0],
        weights,
        edge_weights,
        lambda_data=lambda_data,
        lambda_stretch=lambda_stretch,
        lambda_bend=lambda_bend,
        lambda_end=lambda_end,
        regularizer=regularizer,
    )
    corrected[:, 1] = _solve_weighted_bending_axis(
        base[:, 1],
        weights,
        edge_weights,
        lambda_data=lambda_data,
        lambda_stretch=lambda_stretch,
        lambda_bend=lambda_bend,
        lambda_end=lambda_end,
        regularizer=regularizer,
    )

    return corrected, supported.copy()


def _orient_path_from_endpoint(path_uw, endpoint_idx):
    path = np.asarray(path_uw, float)
    if path.ndim != 2 or path.shape[1] != 2 or len(path) == 0:
        return np.empty((0, 2), float)
    return path.copy() if int(endpoint_idx) == 0 else path[::-1].copy()


def _left_top_endpoint_index(path_uw):
    path = np.asarray(path_uw, float)
    if path.ndim != 2 or path.shape[1] != 2 or len(path) == 0:
        return 0
    first_key = (float(path[0, 0]), float(path[0, 1]))
    last_key = (float(path[-1, 0]), float(path[-1, 1]))
    return 0 if first_key <= last_key else -1


def _component_left_top_order_key(path_uw):
    path = np.asarray(path_uw, float)
    endpoint_idx = _left_top_endpoint_index(path)
    endpoint = path[endpoint_idx]
    center = np.mean(path, axis=0)
    arc = _polyline_arclength_2d(path)
    arc_length = float(arc[-1]) if len(arc) else 0.0
    return (
        float(endpoint[0]),
        float(endpoint[1]),
        float(center[0]),
        float(center[1]),
        -arc_length,
    )


def _merge_component_paths_by_left_top_order(
    component_paths,
    backtrack_tolerance=0.0,
):
    paths = [
        np.asarray(path, float).copy()
        for path in component_paths
        if np.asarray(path, float).ndim == 2 and np.asarray(path, float).shape[1] == 2 and len(path) >= 2
    ]
    if not paths:
        return np.empty((0, 2), float)

    paths.sort(key=_component_left_top_order_key)
    start_endpoint_idx = _left_top_endpoint_index(paths[0])
    merged = _orient_path_from_endpoint(paths[0], start_endpoint_idx)

    for path in paths[1:]:
        current_end = merged[-1]
        first_dist = float(np.linalg.norm(path[0] - current_end))
        last_dist = float(np.linalg.norm(path[-1] - current_end))
        next_endpoint_idx = 0 if first_dist <= last_dist else -1
        next_path = _orient_path_from_endpoint(path, next_endpoint_idx)
        backtrack = float(current_end[0] - next_path[0, 0])
        if backtrack > max(float(backtrack_tolerance), 0.0):
            continue
        merged = np.vstack([merged, next_path])

    return merged


def _extract_surface_polyline_info(
    points_uw,
    endpoint_quantize_step=0.00125,
    endpoint_graph_neighbor_count=6,
    endpoint_component_min_arc_length=0.00375,
    endpoint_component_merge_backtrack_tolerance=0.0,
):
    pts = _quantize_unique_points_2d(
        points_uw,
        float(endpoint_quantize_step),
    )
    if len(pts) < 2:
        return {"path": np.empty((0, 2), float), "components": []}
    if len(pts) == 2:
        order = np.argsort(pts[:, 0], kind="mergesort")
        ordered = pts[order]
        return {"path": ordered, "components": [ordered]}

    diffs = pts[:, None, :] - pts[None, :, :]
    d2 = np.sum(diffs * diffs, axis=2)
    np.fill_diagonal(d2, np.inf)
    neighbor_count = min(max(int(endpoint_graph_neighbor_count), 1), len(pts) - 1)

    adjacency = [[] for _ in range(len(pts))]
    for idx in range(len(pts)):
        neigh = np.argpartition(d2[idx], neighbor_count)[:neighbor_count]
        for j in neigh:
            dist = float(np.sqrt(d2[idx, j]))
            if not np.isfinite(dist):
                continue
            adjacency[idx].append((int(j), dist))
            adjacency[int(j)].append((int(idx), dist))

    components = _graph_components(adjacency)
    if not components:
        order = np.argsort(pts[:, 0], kind="mergesort")
        ordered = pts[order]
        return {"path": ordered, "components": [ordered]}

    component_paths = []
    min_arc = max(float(endpoint_component_min_arc_length), 1e-4)
    for component in components:
        path = _component_longest_path(pts, adjacency, component)
        if len(path) < 2:
            continue
        arc = _polyline_arclength_2d(path)
        if len(arc) < 2 or float(arc[-1]) < min_arc:
            continue
        if path[0, 0] > path[-1, 0]:
            path = path[::-1]
        component_paths.append(path)

    if not component_paths:
        order = np.argsort(pts[:, 0], kind="mergesort")
        ordered = pts[order]
        return {"path": ordered, "components": [ordered]}

    merged = _merge_component_paths_by_left_top_order(
        component_paths,
        backtrack_tolerance=endpoint_component_merge_backtrack_tolerance,
    )

    return {"path": merged, "components": component_paths}


def _convex_hull_2d(points_uw):
    pts = np.asarray(points_uw, float)
    if pts.ndim != 2 or pts.shape[1] != 2 or len(pts) == 0:
        return np.empty((0, 2), float)

    pts = np.unique(pts, axis=0)
    if len(pts) <= 2:
        order = np.argsort(pts[:, 0], kind="mergesort")
        return pts[order]

    order = np.lexsort((pts[:, 1], pts[:, 0]))
    pts = pts[order]

    lower = []
    for point in pts:
        while len(lower) >= 2 and _cross_2d(lower[-2], lower[-1], point) <= 0.0:
            lower.pop()
        lower.append(point.copy())

    upper = []
    for point in pts[::-1]:
        while len(upper) >= 2 and _cross_2d(upper[-2], upper[-1], point) <= 0.0:
            upper.pop()
        upper.append(point.copy())

    return np.asarray(lower[:-1] + upper[:-1], float)


def _cyclic_path(points, start_idx, end_idx, step):
    path = []
    idx = int(start_idx)
    n = len(points)
    while True:
        path.append(points[idx])
        if idx == int(end_idx):
            break
        idx = (idx + step) % n
    return np.asarray(path, float)


def _polyline_chord_relative_w_score(path_uw, endpoint_a, endpoint_b):
    path = np.asarray(path_uw, float)
    if path.ndim != 2 or path.shape[1] != 2 or len(path) < 2:
        return -np.inf

    a = np.asarray(endpoint_a, float).reshape(2)
    b = np.asarray(endpoint_b, float).reshape(2)
    du = float(b[0] - a[0])
    if abs(du) <= 1e-12:
        return -np.inf

    # Use the endpoint chord as a baseline in the (u, w) plane, and compare
    # how much each hull candidate stays above that line in the w direction.
    chord_fraction = (path[:, 0] - a[0]) / du
    line_w = a[1] + chord_fraction * (b[1] - a[1])
    relative_w = path[:, 1] - line_w
    arc = _polyline_arclength_2d(path)
    positive_area = float(np.trapz(np.maximum(relative_w, 0.0), arc))
    negative_area = float(np.trapz(np.maximum(-relative_w, 0.0), arc))
    mean_relative = float(np.mean(relative_w))
    max_relative = float(np.max(relative_w))
    return positive_area - 2.0 * negative_area + 0.1 * max_relative + 0.01 * mean_relative


def _select_hull_chain_from_endpoints(points_uw, endpoint_a, endpoint_b):
    pts = np.asarray(points_uw, float)
    if pts.ndim != 2 or pts.shape[1] != 2 or len(pts) < 2:
        return np.empty((0, 2), float)

    hull = _convex_hull_2d(pts)
    if len(hull) < 2:
        return np.empty((0, 2), float)

    a = np.asarray(endpoint_a, float).reshape(2)
    b = np.asarray(endpoint_b, float).reshape(2)
    a_idx = int(np.argmin(np.linalg.norm(hull - a[None, :], axis=1)))
    b_idx = int(np.argmin(np.linalg.norm(hull - b[None, :], axis=1)))
    if a_idx == b_idx:
        return np.empty((0, 2), float)

    path_a = _cyclic_path(hull, a_idx, b_idx, 1)
    path_b = _cyclic_path(hull, a_idx, b_idx, -1)
    score_a = _polyline_chord_relative_w_score(path_a, a, b)
    score_b = _polyline_chord_relative_w_score(path_b, a, b)
    chain = path_a if score_a >= score_b else path_b
    return _preserve_chain_endpoints(chain, a, b)


def _correct_row_profile(
    slice_points_work,
    row_axis_idx,
    row_point_spacing,
    correction_kwargs=None,
):
    pts = np.asarray(slice_points_work, float)
    if pts.ndim != 2 or pts.shape[1] != 3 or len(pts) < 2:
        return None

    points_uw = np.column_stack([pts[:, row_axis_idx], pts[:, 2]])
    full_hull_uw = _convex_hull_2d(points_uw)
    correction_cfg = dict(correction_kwargs or {})
    solver_cfg = {
        "support_tangent_half_width": float(
            correction_cfg.get("support_tangent_half_width", 0.0025)
        ),
        "support_anchor_max_distance": float(
            correction_cfg.get("support_anchor_max_distance", 0.0025)
        ),
        "transition_count": int(correction_cfg.get("transition_count", 3)),
        "endpoint_transition_count": int(
            correction_cfg.get("endpoint_transition_count", 0)
        ),
        "lambda_data": float(correction_cfg.get("lambda_data", 40.0)),
        "lambda_stretch": float(correction_cfg.get("lambda_stretch", 6.0)),
        "lambda_bend": float(correction_cfg.get("lambda_bend", 1.0)),
        "lambda_end": float(correction_cfg.get("lambda_end", 200.0)),
        "regularizer": float(correction_cfg.get("regularizer", 1e-9)),
    }
    observed_info = _extract_surface_polyline_info(
        points_uw,
        endpoint_quantize_step=float(
            correction_cfg.get("endpoint_quantize_step", 0.00125)
        ),
        endpoint_graph_neighbor_count=int(
            correction_cfg.get("endpoint_graph_neighbor_count", 6)
        ),
        endpoint_component_min_arc_length=float(
            correction_cfg.get("endpoint_component_min_arc_length", 0.00375)
        ),
        endpoint_component_merge_backtrack_tolerance=float(
            correction_cfg.get("endpoint_component_merge_backtrack_tolerance", 0.0)
        ),
    )
    observed_path_uw = np.asarray(observed_info.get("path", []), float)
    observed_component_paths_uw = [
        np.asarray(path, float)
        for path in observed_info.get("components", [])
        if np.asarray(path, float).ndim == 2 and np.asarray(path, float).shape[1] == 2 and len(path) >= 2
    ]
    if len(observed_path_uw) < 2:
        return None

    chain_uw = _select_hull_chain_from_endpoints(
        points_uw,
        endpoint_a=observed_path_uw[0],
        endpoint_b=observed_path_uw[-1],
    )
    if len(chain_uw) < 2:
        return None

    corrected_chain_uw, supported_samples_uw = _solve_hull_regularized_curve_2d(
        chain_uw,
        observed_points_uw=points_uw,
        point_spacing=row_point_spacing,
        **solver_cfg,
    )
    if corrected_chain_uw is None or len(corrected_chain_uw) < 2:
        corrected_chain_uw = np.asarray(chain_uw, float).copy()
    if supported_samples_uw is None:
        supported_samples_uw = np.empty((0, 2), float)

    row_grid = np.asarray(corrected_chain_uw[:, 0], float)
    corrected = np.asarray(corrected_chain_uw[:, 1], float)
    if row_grid.ndim != 1 or corrected.ndim != 1 or len(row_grid) < 2:
        return None

    return {
        "row_grid": row_grid,
        "corrected_w": corrected,
        "connected_component_path_uw": observed_path_uw.copy(),
        "selected_hull_chain_uw": chain_uw.copy(),
        "corrected_chain_uw": corrected_chain_uw.copy(),
        "supported_samples_uw": np.asarray(supported_samples_uw, float).copy(),
        "full_hull_uw": full_hull_uw,
        "observed_component_paths_uw": observed_component_paths_uw,
        "endpoint_points_uw": np.vstack([observed_path_uw[0], observed_path_uw[-1]]),
    }



def correct_surface_spline_rows(
    extraction_result,
    row_point_spacing=0.0025,
    endpoint_quantize_step=0.00125,
    endpoint_graph_neighbor_count=6,
    endpoint_component_min_arc_length=0.00375,
    endpoint_component_merge_backtrack_tolerance=0.0,
    support_tangent_half_width=0.0025,
    support_anchor_max_distance=0.0025,
    transition_count=3,
    endpoint_transition_count=0,
    lambda_data=40.0,
    lambda_stretch=6.0,
    lambda_bend=1.0,
    lambda_end=200.0,
    regularizer=1e-9,
):
    result = dict(extraction_result)

    r_work = np.asarray(result.get("work_frame_world"), float)
    origin = np.asarray(result.get("work_origin_world"), float)
    slice_profiles = result.get("slice_profiles", [])
    spline_start_side = str(result.get("spline_start_side", "top"))

    if r_work.shape != (3, 3) or origin.shape != (3,):
        result["corrected_rows"] = []
        result["corrected_row_profiles"] = []
        return result

    cfg = _spline_scan_config(spline_start_side)
    row_axis_idx = int(cfg["row_axis"])
    slice_axis_idx = int(cfg["slice_axis"])
    descending = bool(cfg["descending"])
    slicing_method = str(result.get("slicing_method", "axis")).strip().lower()

    if not slice_profiles:
        result["corrected_rows"] = []
        result["corrected_row_profiles"] = []
        return result

    def _normalize(v, fallback):
        arr = np.asarray(v, dtype=float).reshape(-1)
        if arr.size != 3 or not np.all(np.isfinite(arr)):
            arr = np.asarray(fallback, dtype=float).reshape(3)
        norm = float(np.linalg.norm(arr))
        if norm <= 1e-12:
            arr = np.asarray(fallback, dtype=float).reshape(3)
            norm = float(np.linalg.norm(arr))
        if norm <= 1e-12:
            return np.array([1.0, 0.0, 0.0], dtype=float)
        return arr / norm

    def _axis_unit(axis_idx):
        axis = np.zeros(3, dtype=float)
        axis[int(axis_idx)] = 1.0
        return axis

    def _profile_plane(profile, points_work):
        default_origin = np.mean(points_work, axis=0)
        if "slice_position" in profile:
            try:
                default_origin[int(slice_axis_idx)] = float(profile.get("slice_position"))
            except (TypeError, ValueError):
                pass

        plane_origin = np.asarray(profile.get("slice_plane_origin_work", default_origin), dtype=float).reshape(-1)
        if plane_origin.size != 3 or not np.all(np.isfinite(plane_origin)):
            plane_origin = default_origin

        row_axis = _normalize(profile.get("slice_plane_row_axis_work", _axis_unit(row_axis_idx)), _axis_unit(row_axis_idx))
        other_axis = _normalize(profile.get("slice_plane_other_axis_work", _axis_unit(2)), _axis_unit(2))
        other_axis = other_axis - float(np.dot(other_axis, row_axis)) * row_axis
        other_axis = _normalize(other_axis, _axis_unit(2))
        if abs(float(np.dot(other_axis, row_axis))) > 0.1:
            other_axis = _axis_unit(2) - float(np.dot(_axis_unit(2), row_axis)) * row_axis
            other_axis = _normalize(other_axis, _axis_unit(2))
        return plane_origin, row_axis, other_axis

    def _to_plane_uw(points_work, plane_origin, row_axis, other_axis):
        rel = np.asarray(points_work, dtype=float) - plane_origin.reshape(1, 3)
        return np.column_stack([rel @ row_axis, rel @ other_axis])

    def _from_plane_uw(points_uw, plane_origin, row_axis, other_axis):
        pts = np.asarray(points_uw, dtype=float)
        if pts.ndim != 2 or pts.shape[1] != 2:
            return np.empty((0, 3), dtype=float)
        return (
            plane_origin.reshape(1, 3)
            + pts[:, 0:1] * row_axis.reshape(1, 3)
            + pts[:, 1:2] * other_axis.reshape(1, 3)
        )

    corrected_profiles_work = []
    endpoint_points_work = []
    full_hull_rows_work = []
    selected_hull_rows_work = []
    supported_sample_rows_work = []
    observed_path_rows_work = []
    connected_component_rows_work = []

    correction_kwargs = {
        "row_point_spacing": float(row_point_spacing),
        "endpoint_quantize_step": float(endpoint_quantize_step),
        "endpoint_graph_neighbor_count": int(endpoint_graph_neighbor_count),
        "endpoint_component_min_arc_length": float(endpoint_component_min_arc_length),
        "endpoint_component_merge_backtrack_tolerance": float(endpoint_component_merge_backtrack_tolerance),
        "support_tangent_half_width": float(support_tangent_half_width),
        "support_anchor_max_distance": float(support_anchor_max_distance),
        "transition_count": int(transition_count),
        "endpoint_transition_count": int(endpoint_transition_count),
        "lambda_data": float(lambda_data),
        "lambda_stretch": float(lambda_stretch),
        "lambda_bend": float(lambda_bend),
        "lambda_end": float(lambda_end),
        "regularizer": float(regularizer),
    }

    if slicing_method == "axis":
        ordered_slice_profiles = sorted(
            slice_profiles,
            key=lambda p: float(p.get("slice_position", 0.0)),
            reverse=descending,
        )
    else:
        ordered_slice_profiles = sorted(
            slice_profiles,
            key=lambda p: (int(p.get("row_index", 0)), float(p.get("slice_position", 0.0))),
        )

    for default_row_index, slice_profile in enumerate(ordered_slice_profiles):
        slice_position = float(slice_profile.get("slice_position", default_row_index))
        row_index = int(slice_profile.get("row_index", default_row_index))
        slice_points_work = np.asarray(slice_profile.get("slice_points_work", []), float)
        if slice_points_work.ndim != 2 or slice_points_work.shape[1] != 3 or len(slice_points_work) < 2:
            continue

        plane_origin, plane_row_axis, plane_other_axis = _profile_plane(slice_profile, slice_points_work)
        points_uw = _to_plane_uw(slice_points_work, plane_origin, plane_row_axis, plane_other_axis)
        local_points = np.zeros((len(points_uw), 3), dtype=float)
        local_points[:, 0] = points_uw[:, 0]
        local_points[:, 2] = points_uw[:, 1]

        corrected_row = _correct_row_profile(
            slice_points_work=local_points,
            row_axis_idx=0,
            row_point_spacing=float(row_point_spacing),
            correction_kwargs=correction_kwargs,
        )
        if corrected_row is None:
            continue

        corrected_uw = np.column_stack([corrected_row["row_grid"], corrected_row["corrected_w"]])
        corrected_work = _from_plane_uw(corrected_uw, plane_origin, plane_row_axis, plane_other_axis)
        if len(corrected_work) < 2:
            continue

        corrected_profiles_work.append(
            {
                "row_index": int(row_index),
                "slice_position": float(slice_position),
                "corrected_row_work": corrected_work,
                "slice_plane_origin_work": plane_origin.copy(),
                "slice_plane_row_axis_work": plane_row_axis.copy(),
                "slice_plane_other_axis_work": plane_other_axis.copy(),
                "slice_plane_normal_work": np.asarray(
                    slice_profile.get("slice_plane_normal_work", np.cross(plane_row_axis, plane_other_axis)),
                    dtype=float,
                ).copy(),
            }
        )

        selected_hull_uw = np.asarray(corrected_row["selected_hull_chain_uw"], float)
        if selected_hull_uw.ndim == 2 and selected_hull_uw.shape[1] == 2 and len(selected_hull_uw) > 0:
            selected_hull_rows_work.append(_from_plane_uw(selected_hull_uw, plane_origin, plane_row_axis, plane_other_axis))

        supported_samples_uw = np.asarray(corrected_row.get("supported_samples_uw", []), float)
        if supported_samples_uw.ndim == 2 and supported_samples_uw.shape[1] == 2 and len(supported_samples_uw) > 0:
            supported_sample_rows_work.append(_from_plane_uw(supported_samples_uw, plane_origin, plane_row_axis, plane_other_axis))

        endpoint_uw = np.asarray(corrected_row.get("endpoint_points_uw", []), float)
        if endpoint_uw.ndim == 2 and endpoint_uw.shape == (2, 2):
            endpoint_points_work.append(_from_plane_uw(endpoint_uw, plane_origin, plane_row_axis, plane_other_axis))

        connected_component_uw = np.asarray(corrected_row.get("connected_component_path_uw", []), float)
        if connected_component_uw.ndim == 2 and connected_component_uw.shape[1] == 2 and len(connected_component_uw) >= 2:
            connected_component_rows_work.append(_from_plane_uw(connected_component_uw, plane_origin, plane_row_axis, plane_other_axis))

        for observed_component_uw in corrected_row.get("observed_component_paths_uw", []) or []:
            observed_component_uw = np.asarray(observed_component_uw, float)
            if observed_component_uw.ndim != 2 or observed_component_uw.shape[1] != 2 or len(observed_component_uw) < 2:
                continue
            observed_path_rows_work.append(_from_plane_uw(observed_component_uw, plane_origin, plane_row_axis, plane_other_axis))

        full_hull_uw = np.asarray(corrected_row.get("full_hull_uw", []), float)
        if full_hull_uw.ndim == 2 and full_hull_uw.shape[1] == 2 and len(full_hull_uw) >= 2:
            full_hull_closed = full_hull_uw
            if not np.allclose(full_hull_closed[0], full_hull_closed[-1], atol=1e-12, rtol=0.0):
                full_hull_closed = np.vstack([full_hull_closed, full_hull_closed[0]])
            full_hull_rows_work.append(_from_plane_uw(full_hull_closed, plane_origin, plane_row_axis, plane_other_axis))

    work_to_world = lambda x: np.asarray(x, float) @ r_work.T + origin
    corrected_rows = []
    corrected_profiles = []
    for corrected in corrected_profiles_work:
        corrected_row_world = work_to_world(corrected["corrected_row_work"])
        corrected_rows.append(corrected_row_world)
        corrected_profiles.append(
            {
                "row_index": int(corrected.get("row_index", len(corrected_profiles))),
                "slice_position": float(corrected["slice_position"]),
                "path_points_work": corrected["corrected_row_work"],
                "slice_plane_origin_work": corrected["slice_plane_origin_work"],
                "slice_plane_normal_work": corrected["slice_plane_normal_work"],
                "slice_plane_row_axis_work": corrected["slice_plane_row_axis_work"],
                "slice_plane_other_axis_work": corrected["slice_plane_other_axis_work"],
            }
        )

    result["correction_endpoint_points_world"] = (
        work_to_world(np.vstack(endpoint_points_work)) if endpoint_points_work else np.empty((0, 3), float)
    )
    result["correction_convex_hull_rows_world"] = [work_to_world(row) for row in full_hull_rows_work]
    result["correction_selected_hull_rows_world"] = [work_to_world(row) for row in selected_hull_rows_work]
    result["correction_supported_sample_rows_world"] = [work_to_world(row) for row in supported_sample_rows_work]
    result["correction_observed_path_rows_world"] = [work_to_world(row) for row in observed_path_rows_work]
    result["correction_connected_component_rows_world"] = [work_to_world(row) for row in connected_component_rows_work]
    result["corrected_rows"] = corrected_rows
    result["corrected_row_profiles"] = corrected_profiles
    return result


__all__ = ["correct_surface_spline_rows"]
