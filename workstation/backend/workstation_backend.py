#!/usr/bin/env python3
"""Painting Process Workstation backend adapter.

The adapter intentionally lives outside the path planner package. It calls the
existing planner as-is, persists offset anchors for an editing session, and
invokes only the existing spline/structuring stages after anchor edits.
"""

from __future__ import annotations

import argparse
import json
import pickle
import shutil
import sys
import traceback
from datetime import datetime
from pathlib import Path
from typing import Any

import numpy as np
import open3d as o3d
import yaml


STATE_FILE_NAME = "planner_state.pkl"
CONTROL_POINTS_FILE_NAME = "edited_control_points.json"
GENERATED_DIR_NAME = "generated"


def _read_request() -> dict[str, Any]:
    source = sys.stdin.read()
    if not source.strip():
        raise ValueError("백엔드 요청 데이터가 없습니다.")
    request = json.loads(source)
    if not isinstance(request, dict):
        raise ValueError("백엔드 요청은 JSON 객체여야 합니다.")
    return request


def _require_directory(value: Any, label: str) -> Path:
    path = Path(str(value)).expanduser().resolve()
    if not path.is_dir():
        raise NotADirectoryError(f"{label} 디렉터리를 찾을 수 없습니다: {path}")
    return path


def _require_file(value: Any, label: str) -> Path:
    path = Path(str(value)).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"{label} 파일을 찾을 수 없습니다: {path}")
    return path


def _configure_planner_imports(planner_root: Path):
    scripts_dir = planner_root / "scripts"
    if not scripts_dir.is_dir():
        raise NotADirectoryError(f"경로 생성 스크립트 디렉터리가 없습니다: {scripts_dir}")
    scripts_source = str(scripts_dir)
    if scripts_source not in sys.path:
        sys.path.insert(0, scripts_source)

    from painting_trajectory_planner import (  # pylint: disable=import-outside-toplevel
        PaintingTrajectoryConfig,
        _structure_and_export_painting_trajectory,
        generate_painting_trajectory_debug,
    )
    from painting_trajectory_planner_4_spline import (  # pylint: disable=import-outside-toplevel
        generate_paint_spline_rows,
    )

    return (
        PaintingTrajectoryConfig,
        generate_painting_trajectory_debug,
        generate_paint_spline_rows,
        _structure_and_export_painting_trajectory,
    )


def _load_surface_points(scan_path: Path) -> np.ndarray:
    suffix = scan_path.suffix.lower()
    if suffix == ".ply":
        cloud = o3d.io.read_point_cloud(str(scan_path))
        if cloud.is_empty():
            raise ValueError(f"스캔 PLY에서 포인트를 읽지 못했습니다: {scan_path}")
        points = np.asarray(cloud.points, dtype=float)
    elif suffix == ".csv":
        try:
            points = np.loadtxt(scan_path, delimiter=",", skiprows=1, usecols=(0, 1, 2))
        except ValueError:
            points = np.loadtxt(scan_path, delimiter=",", usecols=(0, 1, 2))
        points = np.asarray(points, dtype=float)
    else:
        raise ValueError("스캔 데이터는 PLY 또는 XYZ CSV 형식이어야 합니다.")

    if points.ndim != 2 or points.shape[1] != 3 or len(points) == 0:
        raise ValueError("스캔 데이터에 유효한 XYZ 포인트가 없습니다.")
    if not np.isfinite(points).all():
        raise ValueError("스캔 데이터에 NaN 또는 무한대 좌표가 있습니다.")
    return points


def _vector3(value: Any, label: str) -> np.ndarray:
    vector = np.asarray(value, dtype=float).reshape(-1)
    if vector.size != 3 or not np.isfinite(vector).all():
        raise ValueError(f"{label}은 유한한 숫자 3개여야 합니다.")
    norm = float(np.linalg.norm(vector))
    if norm <= 1.0e-12:
        raise ValueError(f"{label}의 크기는 0일 수 없습니다.")
    return vector / norm


def _plane_for_row(state: dict[str, Any], row_index: int) -> tuple[np.ndarray, np.ndarray]:
    profiles = list(state.get("offset_row_profiles", []))
    profile = profiles[row_index] if row_index < len(profiles) else {}
    work_frame = np.asarray(state.get("work_frame_world", []), dtype=float)
    work_origin = np.asarray(state.get("work_origin_world", []), dtype=float).reshape(-1)
    if work_frame.shape != (3, 3) or work_origin.size != 3:
        raise ValueError("작업 좌표계 정보가 없어 슬라이싱 평면을 계산할 수 없습니다.")

    plane_origin_work = np.asarray(
        profile.get("slice_plane_origin_work", []), dtype=float
    ).reshape(-1)
    plane_normal_work = np.asarray(
        profile.get("slice_plane_normal_work", []), dtype=float
    ).reshape(-1)
    if plane_origin_work.size != 3 or plane_normal_work.size != 3:
        raise ValueError(f"row {row_index}의 슬라이싱 평면 정보가 없습니다.")
    plane_origin_world = plane_origin_work @ work_frame.T + work_origin
    plane_normal_world = _vector3(plane_normal_work @ work_frame.T, "슬라이싱 평면 법선")
    return plane_origin_world, plane_normal_world


def _structured_source_row_indices(state: dict[str, Any]) -> list[int]:
    trajectory_rows = list(state.get("paint_spline_rows", []))
    row_count = len(trajectory_rows)
    slice_positions = np.asarray(
        state.get("paint_spline_row_slice_positions", []), dtype=float
    ).reshape(-1)
    if row_count == 0 or len(slice_positions) != row_count:
        return list(range(row_count))

    start_side = str(state.get("spline_start_side", "top")).strip().lower()
    descending = start_side in {"top", "from_top", "top_to_bottom", "right", "from_right", "right_to_left"}
    finite = np.isfinite(slice_positions)
    return sorted(
        range(row_count),
        key=lambda index: (
            not bool(finite[index]),
            -float(slice_positions[index])
            if descending
            else float(slice_positions[index]),
            index,
        ),
    )


def _trajectory_offset_row_indices(state: dict[str, Any]) -> list[int]:
    offset_rows = list(state.get("offset_rows", []))
    trajectory_rows = list(state.get("paint_spline_rows", []))
    source_indices = []
    for offset_row_index, values in enumerate(offset_rows):
        points = np.asarray(values, dtype=float)
        if points.ndim == 2 and points.shape[1] == 3 and len(points) >= 2:
            source_indices.append(offset_row_index)
    if len(source_indices) != len(trajectory_rows):
        raise ValueError(
            "최종 경로 row와 원본 오프셋 row의 대응 관계가 일치하지 않습니다: "
            f"trajectory={len(trajectory_rows)}, offset={len(source_indices)}"
        )
    return [
        source_indices[source_row_index]
        for source_row_index in _structured_source_row_indices(state)
    ]


def _editable_core_rows(state: dict[str, Any]) -> list[dict[str, Any]]:
    point_rows = list(state.get("paint_spline_rows", []))
    orientation_rows = list(state.get("paint_spline_orientation_rows_xyzw", []))
    base_mask_rows = list(state.get("paint_spline_base_paint_mask_rows", []))
    paint_mask_rows = list(state.get("paint_spline_paint_mask_rows", []))
    row_count = len(point_rows)
    if not (
        len(orientation_rows) == row_count
        and len(base_mask_rows) == row_count
        and len(paint_mask_rows) == row_count
    ):
        raise ValueError("최종 경로의 위치, 회전 또는 paint mask row 개수가 일치하지 않습니다.")

    source_indices = _structured_source_row_indices(state)
    offset_indices = _trajectory_offset_row_indices(state)
    editable_rows: list[dict[str, Any]] = []
    for trajectory_row_index, source_row_index in enumerate(source_indices):
        points = np.asarray(point_rows[source_row_index], dtype=float)
        orientations = np.asarray(
            orientation_rows[source_row_index], dtype=float
        )
        base_mask = np.asarray(
            base_mask_rows[source_row_index], dtype=bool
        ).reshape(-1)
        paint_mask = np.asarray(
            paint_mask_rows[source_row_index], dtype=bool
        ).reshape(-1)
        if (
            points.ndim != 2
            or points.shape[1] != 3
            or orientations.shape != (len(points), 4)
            or len(base_mask) != len(points)
            or len(paint_mask) != len(points)
            or not np.isfinite(points).all()
            or not np.isfinite(orientations).all()
        ):
            raise ValueError(f"최종 경로 row {trajectory_row_index} 데이터가 올바르지 않습니다.")
        core_indices = np.flatnonzero(base_mask)
        if len(core_indices) == 0:
            raise ValueError(
                f"최종 경로 row {trajectory_row_index}에 편집 가능한 코어 포인트가 없습니다."
            )
        editable_rows.append(
            {
                "trajectory_row_index": trajectory_row_index,
                "source_row_index": source_row_index,
                "offset_row_index": offset_indices[trajectory_row_index],
                "points": points,
                "orientations": orientations,
                "paint_mask": paint_mask,
                "core_indices": core_indices,
            }
        )
    return editable_rows


def _control_points_from_state(state: dict[str, Any]) -> list[dict[str, Any]]:
    controls: list[dict[str, Any]] = []
    for editable_row in _editable_core_rows(state):
        row_index = int(editable_row["trajectory_row_index"])
        offset_row_index = int(editable_row["offset_row_index"])
        points = editable_row["points"]
        orientations = editable_row["orientations"]
        paint_mask = editable_row["paint_mask"]
        plane_origin, plane_normal = _plane_for_row(state, offset_row_index)
        for point_index in editable_row["core_indices"]:
            position = points[point_index]
            orientation = orientations[point_index]
            controls.append(
                {
                    "id": f"control-{row_index}-{point_index}",
                    "rowIndex": row_index,
                    "pointIndex": int(point_index),
                    "position": position.tolist(),
                    "orientation": orientation.tolist(),
                    "paint": bool(paint_mask[point_index]),
                    "planeOrigin": plane_origin.tolist(),
                    "planeNormal": plane_normal.tolist(),
                }
            )
    return controls


def _write_control_points(session_dir: Path, controls: list[dict[str, Any]]) -> None:
    (session_dir / CONTROL_POINTS_FILE_NAME).write_text(
        json.dumps({"points": controls}, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def _trajectory_summary(painting_trajectory: Any, control_count: int) -> dict[str, Any]:
    rows = list(painting_trajectory.rows)
    return {
        "rowCount": len(rows),
        "pointCount": sum(len(row.points) for row in rows),
        "controlPointCount": int(control_count),
    }


def _generated_dir(session_dir: Path) -> Path:
    output_dir = session_dir / GENERATED_DIR_NAME
    output_dir.mkdir(parents=True, exist_ok=True)
    for stale_path in output_dir.glob("trajectory_*.csv"):
        stale_path.unlink()
    return output_dir


def _structuring_overrides(output_dir: Path) -> dict[str, Any]:
    return {
        "save_csv": True,
        "csv_output_dir": str(output_dir),
        "painting_trajectory_csv_name": "painting_trajectory.csv",
        "trajectory_csv_name_format": "trajectory_{row_index:03d}.csv",
    }


def generate(request: dict[str, Any]) -> dict[str, Any]:
    planner_root = _require_directory(request.get("plannerRoot"), "경로 생성기")
    session_dir = _require_directory(request.get("sessionDir"), "작업 세션")
    scan_path = _require_file(request.get("scanPath"), "스캔 데이터")
    paint_dir = _vector3(request.get("paintDir"), "도장 방향 벡터")
    scan_dir = _vector3(request.get("scanDir"), "스캔 방향 벡터")
    if abs(float(np.dot(paint_dir, scan_dir))) > 0.999:
        raise ValueError("도장 방향과 스캔 방향은 서로 평행할 수 없습니다.")

    (
        PaintingTrajectoryConfig,
        generate_debug,
        _,
        _,
    ) = _configure_planner_imports(planner_root)
    config_source = request.get("parameters")
    if not isinstance(config_source, dict):
        raise ValueError("경로 생성 파라미터가 JSON 객체가 아닙니다.")
    config = PaintingTrajectoryConfig.from_dict(config_source)
    points = _load_surface_points(scan_path)
    output_dir = _generated_dir(session_dir)

    result = generate_debug(
        points,
        config=config,
        extraction_kwargs={
            "paint_dir": paint_dir.tolist(),
            "scan_dir": scan_dir.tolist(),
        },
        structuring_kwargs=_structuring_overrides(output_dir),
    )
    painting_trajectory = result.pop("painting_trajectory")
    result.pop("painting_trajectory_csv_files", None)
    result["_workstation_config"] = config.to_dict()
    result["_workstation_paint_dir"] = paint_dir.tolist()
    result["_workstation_scan_dir"] = scan_dir.tolist()
    result["_workstation_scan_path"] = str(scan_path)
    controls = _control_points_from_state(result)
    result["_workstation_edit_controls"] = controls
    result["_workstation_edit_offset_row_indices"] = (
        _trajectory_offset_row_indices(result)
    )
    result["_workstation_edit_source_row_indices"] = (
        _structured_source_row_indices(result)
    )
    with (session_dir / STATE_FILE_NAME).open("wb") as stream:
        pickle.dump(result, stream, protocol=pickle.HIGHEST_PROTOCOL)
    _write_control_points(session_dir, controls)
    parameter_document = {
        "project": {
            "name": scan_path.stem,
            "paint_dir": paint_dir.tolist(),
            "scan_dir": scan_dir.tolist(),
        },
        "painting_trajectory": config.to_dict(),
    }
    (session_dir / "painting_trajectory.yaml").write_text(
        yaml.safe_dump(parameter_document, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )
    return _trajectory_summary(painting_trajectory, len(controls))


def _quaternion_spray_direction(value: Any) -> np.ndarray:
    quaternion = np.asarray(value, dtype=float).reshape(-1)
    if quaternion.size != 4 or not np.isfinite(quaternion).all():
        raise ValueError("제어점 회전은 XYZW 쿼터니언 숫자 4개여야 합니다.")
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1.0e-12:
        raise ValueError("제어점 회전 쿼터니언의 크기는 0일 수 없습니다.")
    x, y, z, w = quaternion / norm
    direction = np.array(
        [
            2.0 * (x * z + w * y),
            2.0 * (y * z - w * x),
            1.0 - 2.0 * (x * x + y * y),
        ],
        dtype=float,
    )
    return _vector3(direction, "제어점 분사 방향")


def _apply_control_points(
    state: dict[str, Any], controls_source: Any, deleted_rows_source: Any = None,
    deleted_points_source: Any = None,
) -> list[dict[str, Any]]:
    if not isinstance(controls_source, list):
        raise ValueError("수정 제어점 목록이 배열이 아닙니다.")
    source_by_key: dict[tuple[int, int], dict[str, Any]] = {}
    for item in controls_source:
        if not isinstance(item, dict):
            raise ValueError("수정 제어점 항목이 객체가 아닙니다.")
        key = (int(item.get("rowIndex", -1)), int(item.get("pointIndex", -1)))
        if key in source_by_key:
            raise ValueError(f"중복된 제어점이 있습니다: row={key[0]}, point={key[1]}")
        source_by_key[key] = item

    baseline_controls = state.get("_workstation_edit_controls")
    if not isinstance(baseline_controls, list) or not baseline_controls:
        baseline_controls = _control_points_from_state(state)
    baseline_by_key = {
        (int(item["rowIndex"]), int(item["pointIndex"])): item
        for item in baseline_controls
    }
    if deleted_rows_source is None:
        deleted_rows_source = []
    if not isinstance(deleted_rows_source, list):
        raise ValueError("삭제 경로 row 목록이 배열이 아닙니다.")
    try:
        deleted_rows = {int(value) for value in deleted_rows_source}
    except (TypeError, ValueError) as error:
        raise ValueError("삭제 경로 row 번호가 올바르지 않습니다.") from error
    if any(row_index < 0 for row_index in deleted_rows):
        raise ValueError("삭제 경로 row 번호는 0 이상이어야 합니다.")
    available_rows = {key[0] for key in baseline_by_key}
    unknown_deleted_rows = deleted_rows - available_rows
    if unknown_deleted_rows:
        raise ValueError(
            f"삭제할 수 없는 경로 row가 있습니다: {sorted(unknown_deleted_rows)}"
        )

    if deleted_points_source is None:
        deleted_points_source = []
    if (not isinstance(deleted_points_source, list)
            or any(not isinstance(value, str) for value in deleted_points_source)):
        raise ValueError("삭제 포인트 ID 목록은 문자열 배열이어야 합니다.")
    if len(set(deleted_points_source)) != len(deleted_points_source):
        raise ValueError("삭제 포인트 ID가 중복되었습니다.")
    baseline_key_by_id = {str(item["id"]): key for key, item in baseline_by_key.items()}
    unknown_deleted_points = set(deleted_points_source) - baseline_key_by_id.keys()
    if unknown_deleted_points:
        raise ValueError(f"삭제할 수 없는 포인트 ID가 있습니다: {sorted(unknown_deleted_points)}")
    deleted_keys = {baseline_key_by_id[value] for value in deleted_points_source}
    changed_rows = {key[0] for key in deleted_keys} - deleted_rows
    expected_keys = {
        key for key in baseline_by_key if key[0] not in deleted_rows and key not in deleted_keys
    }
    for row_index in changed_rows:
        if sum(key[0] == row_index for key in expected_keys) < 2:
            raise ValueError(f"row {row_index}에는 최소 2개의 편집 포인트가 남아야 합니다. 경로 전체 삭제를 사용하세요.")
    actual_keys = set(source_by_key)
    if actual_keys != expected_keys:
        raise ValueError(
            f"제어점 구성이 일치하지 않습니다: expected={len(expected_keys)}, "
            f"actual={len(actual_keys)}, missing={len(expected_keys - actual_keys)}, "
            f"extra={len(actual_keys - expected_keys)}"
        )

    new_rows = [np.asarray(row, dtype=float).copy() for row in state.get("offset_rows", [])]
    new_normals = [
        np.asarray(row, dtype=float).copy()
        for row in state.get("offset_row_normals_world", [])
    ]
    if len(new_normals) != len(new_rows):
        raise ValueError("오프셋 위치 row와 법선 row 개수가 일치하지 않습니다.")

    offset_indices_source = state.get("_workstation_edit_offset_row_indices")
    if isinstance(offset_indices_source, list):
        offset_indices = [int(value) for value in offset_indices_source]
    else:
        offset_indices = _trajectory_offset_row_indices(state)
    source_indices_source = state.get("_workstation_edit_source_row_indices")
    if isinstance(source_indices_source, list):
        source_indices = [int(value) for value in source_indices_source]
    else:
        source_indices = _structured_source_row_indices(state)

    keys_by_row: dict[int, list[tuple[int, int]]] = {}
    for key in expected_keys:
        keys_by_row.setdefault(key[0], []).append(key)
    for row_keys in keys_by_row.values():
        row_keys.sort(key=lambda key: key[1])

    raster_zigzag = bool(state.get("paint_spline_raster_zigzag", False))
    normalized_by_key: dict[tuple[int, int], dict[str, Any]] = {}
    for row_index in sorted(keys_by_row):
        if row_index >= len(offset_indices):
            raise ValueError(f"row {row_index}의 오프셋 입력 row 정보가 없습니다.")
        if row_index >= len(source_indices):
            raise ValueError(f"row {row_index}의 원본 spline row 정보가 없습니다.")
        offset_row_index = offset_indices[row_index]
        source_row_index = source_indices[row_index]
        row_keys = keys_by_row[row_index]
        plane_origin, plane_normal = _plane_for_row(state, offset_row_index)
        positions = np.zeros((len(row_keys), 3), dtype=float)
        normals = np.zeros_like(positions)
        baseline_positions = np.vstack(
            [np.asarray(baseline_by_key[key]["position"], dtype=float) for key in row_keys]
        )
        baseline_directions = np.vstack(
            [
                _quaternion_spray_direction(baseline_by_key[key]["orientation"])
                for key in row_keys
            ]
        )
        for core_index, key in enumerate(row_keys):
            item = source_by_key[key]
            position = np.asarray(item.get("position"), dtype=float).reshape(-1)
            if position.size != 3 or not np.isfinite(position).all():
                raise ValueError(f"{key} 위치가 유효하지 않습니다.")
            position = position - float(np.dot(position - plane_origin, plane_normal)) * plane_normal
            orientation = np.asarray(item.get("orientation"), dtype=float).reshape(-1)
            direction = _quaternion_spray_direction(orientation)
            orientation = orientation / max(float(np.linalg.norm(orientation)), 1.0e-12)
            positions[core_index] = position
            normals[core_index] = -direction
            baseline = baseline_by_key[key]
            normalized_by_key[key] = {
                "id": str(baseline["id"]),
                "rowIndex": key[0],
                "pointIndex": key[1],
                "position": position.tolist(),
                "orientation": orientation.tolist(),
                "paint": bool(baseline.get("paint", True)),
                "planeOrigin": plane_origin.tolist(),
                "planeNormal": plane_normal.tolist(),
            }

        position_changed = bool(
            np.any(np.linalg.norm(positions - baseline_positions, axis=1) > 1.0e-9)
        )
        direction_changed = bool(
            np.any(
                np.linalg.norm((-normals) - baseline_directions, axis=1) > 1.0e-9
            )
        )
        points_deleted = row_index in changed_rows
        if not position_changed and not direction_changed and not points_deleted:
            continue

        original_row = new_rows[offset_row_index]
        original_normals = new_normals[offset_row_index]
        if len(row_keys) == 1:
            positions = original_row + (positions[0] - baseline_positions[0])
            normals = np.repeat(normals[0].reshape(1, 3), len(original_row), axis=0)
        else:
            reversed_row = raster_zigzag and source_row_index % 2 == 1
            if reversed_row:
                positions = positions[::-1].copy()
                normals = normals[::-1].copy()
            # 마지막 편집점 뒤의 원본 끝점은 기존 동작대로 보존하되,
            # 끝 편집점을 삭제한 경우에는 복원하지 않는다. 지그재그 row의
            # 원본 끝점은 화면상 첫 편집점 쪽이므로 source row 순서를 따른다.
            original_keys = [key for key in baseline_by_key if key[0] == row_index]
            tail_key = min(original_keys) if reversed_row else max(original_keys)
            if (tail_key not in deleted_keys
                    and float(np.linalg.norm(positions[-1] - original_row[-1])) > 1.0e-9):
                positions = np.vstack([positions, original_row[-1]])
                normals = np.vstack([normals, original_normals[-1]])
        new_rows[offset_row_index] = positions
        new_normals[offset_row_index] = normals

    for row_index in sorted(deleted_rows):
        if row_index >= len(offset_indices):
            raise ValueError(f"row {row_index}의 오프셋 입력 row 정보가 없습니다.")
        offset_row_index = offset_indices[row_index]
        new_rows[offset_row_index] = np.empty((0, 3), dtype=float)
        new_normals[offset_row_index] = np.empty((0, 3), dtype=float)

    state["offset_rows"] = new_rows
    state["offset_row_normals_world"] = new_normals
    populated_rows = [row for row in new_rows if len(row)]
    state["offset_points_world"] = (
        np.vstack(populated_rows)
        if populated_rows
        else np.empty((0, 3), dtype=float)
    )
    normalized_controls = [
        normalized_by_key[(int(item["rowIndex"]), int(item["pointIndex"]))]
        for item in baseline_controls
        if (int(item["rowIndex"]), int(item["pointIndex"])) in normalized_by_key
    ]
    state["_workstation_edit_controls"] = normalized_controls
    state["_workstation_edit_offset_row_indices"] = offset_indices
    state["_workstation_edit_source_row_indices"] = source_indices
    return normalized_controls


def regenerate(request: dict[str, Any]) -> dict[str, Any]:
    planner_root = _require_directory(request.get("plannerRoot"), "경로 생성기")
    session_dir = _require_directory(request.get("sessionDir"), "작업 세션")
    state_path = _require_file(session_dir / STATE_FILE_NAME, "경로 생성 상태")
    (
        PaintingTrajectoryConfig,
        _,
        generate_spline,
        structure_and_export,
    ) = _configure_planner_imports(planner_root)
    with state_path.open("rb") as stream:
        state = pickle.load(stream)
    if not isinstance(state, dict):
        raise ValueError("저장된 경로 생성 상태가 올바르지 않습니다.")

    deleted_rows_source = request.get("deletedRowIndices", [])
    controls = _apply_control_points(
        state,
        request.get("controlPoints"),
        deleted_rows_source,
        request.get("deletedPointIds", []),
    )
    config = PaintingTrajectoryConfig.from_dict(state.get("_workstation_config"))
    regenerated = generate_spline(state, **config.spline.to_kwargs())
    # The submitted controls are spline inputs. Rebuild the edit controls from
    # the regenerated, uniformly sampled trajectory so the pink points and
    # direction glyphs shown by the editor match the exported 40 mm path.
    controls = _control_points_from_state(regenerated)
    regenerated["_workstation_edit_controls"] = controls
    regenerated["_workstation_edit_offset_row_indices"] = (
        _trajectory_offset_row_indices(regenerated)
    )
    regenerated["_workstation_edit_source_row_indices"] = (
        _structured_source_row_indices(regenerated)
    )
    output_dir = _generated_dir(session_dir)
    painting_trajectory, _ = structure_and_export(
        regenerated,
        config.structuring,
        structuring_kwargs=_structuring_overrides(output_dir),
    )
    with state_path.open("wb") as stream:
        pickle.dump(regenerated, stream, protocol=pickle.HIGHEST_PROTOCOL)
    _write_control_points(session_dir, controls)
    return _trajectory_summary(painting_trajectory, len(controls))


def complete(request: dict[str, Any]) -> dict[str, Any]:
    planner_root = _require_directory(request.get("plannerRoot"), "경로 생성기")
    session_dir = _require_directory(request.get("sessionDir"), "작업 세션")
    generated_dir = _require_directory(session_dir / GENERATED_DIR_NAME, "생성 경로")
    full_trajectory = _require_file(
        generated_dir / "painting_trajectory.csv", "전체 도장 경로"
    )
    scan_path = _require_file(request.get("scanPath"), "스캔 데이터")
    log_root = planner_root / "log"
    log_root.mkdir(parents=True, exist_ok=True)

    timestamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
    output_dir = log_root / timestamp
    collision_index = 1
    while output_dir.exists():
        output_dir = log_root / f"{timestamp}_{collision_index:02d}"
        collision_index += 1
    output_dir.mkdir(parents=True)

    for path in generated_dir.glob("*.csv"):
        shutil.copy2(path, output_dir / path.name)
    shutil.copy2(scan_path, output_dir / scan_path.name)
    for file_name in ("painting_trajectory.yaml", CONTROL_POINTS_FILE_NAME):
        source = session_dir / file_name
        if source.is_file():
            shutil.copy2(source, output_dir / file_name)

    summary = {
        "completedAt": datetime.now().astimezone().isoformat(timespec="seconds"),
        "scanFile": scan_path.name,
        "trajectoryFile": full_trajectory.name,
        "outputDirectory": str(output_dir),
    }
    (output_dir / "workstation_job.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return summary


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Painting Process Workstation backend")
    parser.add_argument("action", choices=("generate", "regenerate", "complete"))
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    request = _read_request()
    handlers = {
        "generate": generate,
        "regenerate": regenerate,
        "complete": complete,
    }
    response = handlers[args.action](request)
    json.dump(response, sys.stdout, ensure_ascii=False, allow_nan=False)
    sys.stdout.write("\n")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception:
        traceback.print_exc(file=sys.stderr)
        raise SystemExit(1)
