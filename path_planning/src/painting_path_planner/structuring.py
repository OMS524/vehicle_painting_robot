#!/usr/bin/env python3

from dataclasses import dataclass, field

import numpy as np


Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]


@dataclass(slots=True)
class TrajectoryPoint:
    s: float
    position: Vec3
    orientation: Quat
    paint: bool


@dataclass(slots=True)
class TrajectoryRow:
    row_index: int
    points: list[TrajectoryPoint] = field(default_factory=list)


@dataclass(slots=True)
class PaintTrajectory:
    frame_id: str
    rows: list[TrajectoryRow] = field(default_factory=list)


PaintingTrajectory = PaintTrajectory


def _as_vec3(values) -> Vec3:
    arr = np.asarray(values, float).reshape(3)
    return (float(arr[0]), float(arr[1]), float(arr[2]))


def _as_quat(values) -> Quat:
    arr = np.asarray(values, float).reshape(4)
    norm = float(np.linalg.norm(arr))
    if norm <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    arr = arr / norm
    return (float(arr[0]), float(arr[1]), float(arr[2]), float(arr[3]))


def _polyline_arclength(points):
    pts = np.asarray(points, float)
    if pts.ndim != 2 or pts.shape[1] != 3 or len(pts) == 0:
        return np.zeros(0, dtype=float)
    if len(pts) == 1:
        return np.zeros(1, dtype=float)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(seg)])


def _array_rows(source, primary_key, fallback_key=None):
    rows = source.get(primary_key, None)
    if rows is None and fallback_key is not None:
        rows = source.get(fallback_key, None)
    if rows is None:
        return []
    return list(rows)


def _continuous_quaternions(orientations):
    quats = np.asarray(orientations, float).copy()
    if quats.ndim != 2 or quats.shape[1] != 4:
        return quats
    for idx in range(len(quats)):
        norm = float(np.linalg.norm(quats[idx]))
        if norm <= 1e-12:
            quats[idx] = np.array([0.0, 0.0, 0.0, 1.0], dtype=float)
        else:
            quats[idx] = quats[idx] / norm
        if idx > 0 and float(np.dot(quats[idx], quats[idx - 1])) < 0.0:
            quats[idx] = -quats[idx]
    return quats


def _row_arrays(paint_spline_result, row_index):
    position_rows = _array_rows(
        paint_spline_result,
        "paint_trajectory_position_rows",
        "paint_spline_rows",
    )
    orientation_rows = _array_rows(
        paint_spline_result,
        "paint_trajectory_orientation_rows_xyzw",
        "paint_spline_orientation_rows_xyzw",
    )
    paint_mask_rows = _array_rows(
        paint_spline_result,
        "paint_trajectory_paint_mask_rows",
        "paint_spline_paint_mask_rows",
    )

    positions = np.asarray(position_rows[row_index], float)
    orientations = _continuous_quaternions(np.asarray(orientation_rows[row_index], float))
    paint_mask = np.asarray(paint_mask_rows[row_index], dtype=bool).reshape(-1)
    if positions.ndim != 2 or positions.shape[1] != 3 or len(positions) == 0:
        return None
    if orientations.ndim != 2 or orientations.shape != (len(positions), 4):
        return None
    if len(paint_mask) != len(positions):
        return None

    count = len(positions)
    arc_rows = _array_rows(
        paint_spline_result,
        "paint_trajectory_arc_length_rows",
        "paint_spline_arc_length_rows",
    )
    arcs = (
        np.asarray(arc_rows[row_index], float).reshape(-1)
        if row_index < len(arc_rows)
        else _polyline_arclength(positions)
    )
    if len(arcs) != count:
        return None

    return {
        "arcs": arcs,
        "positions": positions,
        "orientations": orientations,
        "paint_mask": paint_mask,
    }


def structure_painting_trajectory(
    paint_spline_result,
    frame_id="world",
):
    position_rows = _array_rows(
        paint_spline_result,
        "paint_trajectory_position_rows",
        "paint_spline_rows",
    )
    orientation_rows = _array_rows(
        paint_spline_result,
        "paint_trajectory_orientation_rows_xyzw",
        "paint_spline_orientation_rows_xyzw",
    )
    paint_mask_rows = _array_rows(
        paint_spline_result,
        "paint_trajectory_paint_mask_rows",
        "paint_spline_paint_mask_rows",
    )
    row_count = min(len(position_rows), len(orientation_rows), len(paint_mask_rows))

    rows = []
    # 상위 단계에서 시작 방향·보정 슬라이스 삽입·지그재그가 이미 적용되었다.
    # 경로 좌표의 중앙값으로 재정렬하면 가이드라인 순서와 지그재그가 깨진다.
    for trajectory_row_index in range(row_count):
        arrays = _row_arrays(
            paint_spline_result,
            trajectory_row_index,
        )
        if arrays is None:
            continue

        points = [
            TrajectoryPoint(
                s=float(s),
                position=_as_vec3(position),
                orientation=_as_quat(orientation),
                paint=bool(paint),
            )
            for (
                s,
                position,
                orientation,
                paint,
            ) in zip(
                arrays["arcs"],
                arrays["positions"],
                arrays["orientations"],
                arrays["paint_mask"],
            )
        ]
        rows.append(TrajectoryRow(row_index=trajectory_row_index, points=points))

    return PaintTrajectory(
        frame_id=str(frame_id),
        rows=rows,
    )


__all__ = [
    "Vec3",
    "Quat",
    "TrajectoryPoint",
    "TrajectoryRow",
    "PaintTrajectory",
    "PaintingTrajectory",
    "structure_painting_trajectory",
]
