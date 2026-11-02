"""Validate saved planner CSVs without changing samples, order or spray flags."""
import csv
import io
import math
from pathlib import Path
import shutil
import json

REQUIRED = ["row_index", "point_index", "s", "position_x", "position_y", "position_z",
            "orientation_x", "orientation_y", "orientation_z", "orientation_w", "paint"]


def validate_csv(source: bytes) -> dict:
    reader = csv.DictReader(io.StringIO(source.decode("utf-8-sig")))
    if not reader.fieldnames or not set(REQUIRED + ["frame_id"]).issubset(reader.fieldnames):
        raise ValueError("도장 경로 CSV에 필수 열이 없습니다: " + ", ".join(REQUIRED + ["frame_id"]))
    if len(set(reader.fieldnames)) != len(reader.fieldnames):
        raise ValueError("CSV 헤더에 중복된 열 이름이 있습니다.")
    seen = set()
    rows = []
    previous_row, previous_point, previous_s = None, -1, -1.0
    count = 0
    row_counts = {}
    for line, item in enumerate(reader, 2):
        try:
            if None in item:
                raise ValueError("헤더보다 많은 열이 있습니다")
            row, point = int(item["row_index"]), int(item["point_index"])
            values = [float(item[key]) for key in REQUIRED[2:-1]]
            if row < 0 or point < 0 or not all(math.isfinite(v) for v in values):
                raise ValueError("음수 인덱스 또는 비유한 값")
            if row != previous_row:
                if row in seen or (previous_row is not None and row < previous_row):
                    raise ValueError("row_index는 연속된 묶음의 오름차순이어야 합니다")
                seen.add(row)
                rows.append(row)
                previous_point, previous_s = -1, -1.0
            if point <= previous_point or values[0] < previous_s or values[0] < 0:
                raise ValueError("point_index와 s 순서가 올바르지 않습니다")
            if abs(math.sqrt(sum(v * v for v in values[4:8])) - 1.0) > 1e-3:
                raise ValueError("orientation은 단위 XYZW 쿼터니언이어야 합니다")
            if not (item.get("frame_id") or "").strip():
                raise ValueError("frame_id가 비어 있습니다")
            if (item["paint"] or "").strip().lower() not in ("0", "1", "true", "false"):
                raise ValueError("paint는 0/1 또는 true/false여야 합니다")
            previous_row, previous_point, previous_s = row, point, values[0]
            count += 1
            row_counts[row] = row_counts.get(row, 0) + 1
        except (ValueError, TypeError, KeyError) as error:
            raise ValueError(f"CSV {line}행이 올바르지 않습니다: {error}") from error
    if not count:
        raise ValueError("도장 경로 CSV에 포인트가 없습니다.")
    if any(n < 2 for n in row_counts.values()):
        raise ValueError("각 경로 행에는 최소 2개의 포인트가 필요합니다.")
    return {"rowCount": len(rows), "pointCount": count, "controlPointCount": 0}


def import_trajectory(request: dict) -> dict:
    session = Path(request["sessionDir"])
    source = Path(request["csvPath"])
    summary = validate_csv(source.read_bytes())
    generated = session / "generated"
    generated.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, generated / "painting_trajectory.csv")
    (session / "edited_control_points.json").write_text('{"points": []}\n')
    (session / "imported_trajectory.json").write_text(json.dumps({
        "fileName": request["fileName"], "mode": "csv_review_only",
        "note": "원본 CSV 보존. 슬라이싱 평면/코어 마스크 부재로 포인트 편집 비활성화.",
        **summary,
    }, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return summary
