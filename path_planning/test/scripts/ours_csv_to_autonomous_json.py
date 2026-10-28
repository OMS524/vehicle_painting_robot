#!/usr/bin/env python3

"""Convert our painting trajectory CSV files to Autonomous-style JSON files."""

from __future__ import annotations

import csv
import json
import math
from collections import defaultdict
from pathlib import Path
from typing import Any


# 변환할 CSV와 저장할 JSON 경로를 한 쌍씩 추가합니다.
CONVERSION_JOBS: list[dict[str, Any]] = [
    {
        "csv_path": Path(
            "/home/oms/vehicle_painting_robot/path_planning/test/csv/"
            "ES300h_front_bumper/painting_trajectory.csv"
        ),
        "output_path": Path(
            "/home/oms/temporary/ES300h_front_bumper.json"
        ),
    },
    {
        "csv_path": Path(
            "/home/oms/vehicle_painting_robot/path_planning/test/csv/"
            "ES300h_hood/painting_trajectory.csv"
        ),
        "output_path": Path(
            "/home/oms/temporary/ES300h_hood.json"
        ),
    },
    {
        "csv_path": Path(
            "/home/oms/vehicle_painting_robot/path_planning/test/csv/"
            "ES300h_left_fender/painting_trajectory.csv"
        ),
        "output_path": Path(
            "/home/oms/temporary/ES300h_left_fender.json"
        ),
    },
    {
        "csv_path": Path(
            "/home/oms/vehicle_painting_robot/path_planning/test/csv/"
            "ES300h_trunk/painting_trajectory.csv"
        ),
        "output_path": Path(
            "/home/oms/temporary/ES300h_trunk.json"
        ),
    },
    {
        "csv_path": Path(
            "/home/oms/vehicle_painting_robot/path_planning/test/csv/"
            "NX350h_right_fender/painting_trajectory.csv"
        ),
        "output_path": Path(
            "/home/oms/temporary/NX350h_right_fender.json"
        ),
    },
    {
        "csv_path": Path(
            "/home/oms/vehicle_painting_robot/path_planning/test/csv/"
            "Tesla_Model_Y_front_bumper/painting_trajectory.csv"
        ),
        "output_path": Path(
            "/home/oms/temporary/Tesla_Model_Y_front_bumper.json"
        ),
    },
]


# 계약사 JSON 포인트에 고정으로 기록할 값입니다.
DEFAULT_FIXED_VALUES: dict[str, int | float] = {
    "gun_power": 1.0,
    "gun_trigger_delay": 0,
    "is_swift": 0,
    "speed": 0.8,
}

TOP_LEVEL_TYPE = ""
TOP_LEVEL_VERSION = "v1"

REQUIRED_CSV_COLUMNS = {
    "row_index",
    "point_index",
    "position_x",
    "position_y",
    "position_z",
    "orientation_x",
    "orientation_y",
    "orientation_z",
    "orientation_w",
    "paint",
}


def _parse_int(value: str, field_name: str) -> int:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field_name} 값이 숫자가 아닙니다: {value!r}") from exc
    if not math.isfinite(number) or not number.is_integer():
        raise ValueError(f"{field_name} 값이 정수가 아닙니다: {value!r}")
    return int(number)


def _parse_float(value: str, field_name: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field_name} 값이 숫자가 아닙니다: {value!r}") from exc
    if not math.isfinite(number):
        raise ValueError(f"{field_name} 값이 유한한 숫자가 아닙니다: {value!r}")
    return number


def _parse_paint(value: str) -> bool:
    normalized = str(value).strip().lower()
    if normalized in {"1", "1.0", "true", "t", "yes", "y"}:
        return True
    if normalized in {"0", "0.0", "false", "f", "no", "n"}:
        return False
    raise ValueError(f"paint 값은 0/1 또는 false/true여야 합니다: {value!r}")


def _quaternion_to_autonomous_axes(
    quaternion_xyzw: tuple[float, float, float, float],
) -> tuple[list[float], list[float], list[float]]:
    """Return front, left, up from our normalized XYZW quaternion.

    Our trajectory orientation uses local X as the row/tangent axis,
    local Y as the remaining orthogonal axis, and local Z as the spray
    direction. Autonomous JSON names these axes left, up, and front.
    """

    x, y, z, w = quaternion_xyzw
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        raise ValueError("크기가 0인 쿼터니언은 방향 벡터로 변환할 수 없습니다.")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm

    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z

    rotation = (
        (
            1.0 - 2.0 * (yy + zz),
            2.0 * (xy - wz),
            2.0 * (xz + wy),
        ),
        (
            2.0 * (xy + wz),
            1.0 - 2.0 * (xx + zz),
            2.0 * (yz - wx),
        ),
        (
            2.0 * (xz - wy),
            2.0 * (yz + wx),
            1.0 - 2.0 * (xx + yy),
        ),
    )

    # Rotation matrix columns are the local X, Y, and Z axes in world space.
    left = [rotation[0][0], rotation[1][0], rotation[2][0]]
    up = [rotation[0][1], rotation[1][1], rotation[2][1]]
    front = [rotation[0][2], rotation[1][2], rotation[2][2]]
    return front, left, up


def _gun_triggers_for_row(paint_mask: list[bool]) -> list[str]:
    """Convert per-point paint states to open/close/nothing events.

    The first painted point of each continuous paint run receives ``open``.
    The last painted point receives ``close``, matching the reference JSON
    layout. A one-point paint run cannot contain both events and is rejected.
    """

    triggers = ["nothing"] * len(paint_mask)
    for index, is_painting in enumerate(paint_mask):
        if not is_painting:
            continue

        previous_is_painting = index > 0 and paint_mask[index - 1]
        next_is_painting = index + 1 < len(paint_mask) and paint_mask[index + 1]
        starts_run = not previous_is_painting
        ends_run = not next_is_painting

        if starts_run and ends_run:
            raise ValueError(
                "포인트 하나로만 구성된 paint=True 구간은 동일 포인트에 "
                "open과 close를 함께 저장할 수 없습니다."
            )
        if starts_run:
            triggers[index] = "open"
        elif ends_run:
            triggers[index] = "close"

    return triggers


def _read_csv_rows(csv_path: Path) -> dict[int, list[dict[str, Any]]]:
    rows_by_index: dict[int, list[dict[str, Any]]] = defaultdict(list)

    with csv_path.open("r", encoding="utf-8-sig", newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        field_names = set(reader.fieldnames or [])
        missing_columns = sorted(REQUIRED_CSV_COLUMNS - field_names)
        if missing_columns:
            raise ValueError(
                f"{csv_path}: 필수 CSV 열이 없습니다: {', '.join(missing_columns)}"
            )

        for csv_line_number, raw in enumerate(reader, start=2):
            try:
                row_index = _parse_int(raw["row_index"], "row_index")
                point_index = _parse_int(raw["point_index"], "point_index")
                point = {
                    "point_index": point_index,
                    "pos": [
                        _parse_float(raw["position_x"], "position_x"),
                        _parse_float(raw["position_y"], "position_y"),
                        _parse_float(raw["position_z"], "position_z"),
                    ],
                    "quaternion": (
                        _parse_float(raw["orientation_x"], "orientation_x"),
                        _parse_float(raw["orientation_y"], "orientation_y"),
                        _parse_float(raw["orientation_z"], "orientation_z"),
                        _parse_float(raw["orientation_w"], "orientation_w"),
                    ),
                    "paint": _parse_paint(raw["paint"]),
                }
            except ValueError as exc:
                raise ValueError(f"{csv_path}:{csv_line_number}: {exc}") from exc
            rows_by_index[row_index].append(point)

    if not rows_by_index:
        raise ValueError(f"{csv_path}: 변환할 경로 포인트가 없습니다.")

    for row_index, points in rows_by_index.items():
        points.sort(key=lambda point: point["point_index"])
        point_indices = [point["point_index"] for point in points]
        if len(point_indices) != len(set(point_indices)):
            raise ValueError(
                f"{csv_path}: row_index={row_index}에 중복 point_index가 있습니다."
            )

    return dict(sorted(rows_by_index.items()))


def convert_csv_to_autonomous_json(
    csv_path: str | Path,
    output_path: str | Path,
    *,
    fixed_values: dict[str, int | float] | None = None,
) -> dict[str, Any]:
    csv_path = Path(csv_path).expanduser().resolve()
    output_path = Path(output_path).expanduser().resolve()
    if not csv_path.is_file():
        raise FileNotFoundError(f"CSV 파일을 찾을 수 없습니다: {csv_path}")

    point_fixed_values = dict(DEFAULT_FIXED_VALUES)
    if fixed_values:
        unknown_keys = sorted(set(fixed_values) - set(DEFAULT_FIXED_VALUES))
        if unknown_keys:
            raise ValueError(
                "지원하지 않는 고정값 필드입니다: " + ", ".join(unknown_keys)
            )
        point_fixed_values.update(fixed_values)

    rows_by_index = _read_csv_rows(csv_path)
    trace: list[dict[str, Any]] = []

    for row_index, points in rows_by_index.items():
        triggers = _gun_triggers_for_row([point["paint"] for point in points])
        for point, gun_trigger in zip(points, triggers):
            front, left, up = _quaternion_to_autonomous_axes(point["quaternion"])
            trace.append(
                {
                    "front": front,
                    "gun_power": point_fixed_values["gun_power"],
                    "gun_trigger": gun_trigger,
                    "gun_trigger_delay": point_fixed_values["gun_trigger_delay"],
                    "id": len(trace),
                    "is_swift": point_fixed_values["is_swift"],
                    "left": left,
                    "pos": point["pos"],
                    "row_ind": row_index,
                    "speed": point_fixed_values["speed"],
                    "up": up,
                }
            )

    output = {
        "trace": trace,
        "type": TOP_LEVEL_TYPE,
        "version": TOP_LEVEL_VERSION,
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as json_file:
        json.dump(output, json_file, ensure_ascii=False, indent=2, allow_nan=False)
        json_file.write("\n")

    return {
        "csv_path": csv_path,
        "output_path": output_path,
        "row_count": len(rows_by_index),
        "point_count": len(trace),
    }


def main() -> None:
    if not CONVERSION_JOBS:
        raise SystemExit(
            "CONVERSION_JOBS에 csv_path와 output_path를 하나 이상 설정하세요."
        )

    for job in CONVERSION_JOBS:
        if "csv_path" not in job or "output_path" not in job:
            raise ValueError(
                "각 CONVERSION_JOBS 항목에는 csv_path와 output_path가 필요합니다."
            )
        result = convert_csv_to_autonomous_json(
            job["csv_path"],
            job["output_path"],
            fixed_values=job.get("fixed_values"),
        )
        print(
            f"[converted] {result['csv_path']} -> {result['output_path']} "
            f"(rows={result['row_count']}, points={result['point_count']})"
        )


if __name__ == "__main__":
    main()
