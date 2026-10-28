#!/usr/bin/env python3
# 실행: conda activate vehicle_painting_robot_planner
# python path_planning/test/scripts/test.py

import sys
import webbrowser
from pathlib import Path



PLANNER_ROOT = Path(__file__).resolve().parents[2]
SCRIPTS_DIR = PLANNER_ROOT / "scripts"
DATA_DIR = PLANNER_ROOT / "test" / "data"
CSV_DIR = PLANNER_ROOT / "test" / "csv"

if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

from path_planning import generate_painting_trajectory_debug  # noqa: E402
from painting_path_planner.visualization import load_surface_data, write_debug_plotly_html


SAVE_DEBUG_HTML = True
OPEN_DEBUG_HTML = True
BUMPER_PAINT_DIR = [0.0, 0.0, -1.0]
BUMPER_SCAN_DIR = [1.0, 0.0, 0.0]

DOOR_PAINT_DIR = [0.0, 1.0, 0.0]
DOOR_SCAN_DIR = [1.0, 0.0, 0.0]

# 각 case의 slicing_method는 "axis" 또는 "surface_adaptive"로 지정합니다.
CASES = [
    # {
    #     "name": "car_bumper_small",
    #     "input_csv": DATA_DIR / "car_bumper_small_surface_points.csv",
    #     "output_dir": CSV_DIR / "car_bumper_small",
    #     "paint_dir": BUMPER_PAINT_DIR,
    #     "scan_dir": BUMPER_SCAN_DIR,
    # },
    # {
    #     "name": "car_door_small",
    #     "input_csv": DATA_DIR / "car_door_small_surface_points.csv",
    #     "output_dir": CSV_DIR / "car_door_small",
    #     "paint_dir": DOOR_PAINT_DIR,
    #     "scan_dir": DOOR_SCAN_DIR,
    # },



    {
        "name": "ES300h_front_bumper",
        "input_csv": DATA_DIR / "ES300h_front_bumper.ply",
        "output_dir": CSV_DIR / "ES300h_front_bumper",
        "paint_dir": [0.0, 0.0, -1.0],
        "scan_dir": [0.0, 1.0, 0.0],
        "slicing_method": "axis",
    },
    {
        "name": "ES300h_hood",
        "input_csv": DATA_DIR / "ES300h_hood.ply",
        "output_dir": CSV_DIR / "ES300h_hood",
        "paint_dir": [0.0, 0.0, -1.0],
        "scan_dir": [-1.0, 0.0, 0.0],
        "slicing_method": "surface_adaptive",
    },
    {
        "name": "ES300h_left_fender",
        "input_csv": DATA_DIR / "ES300h_left_fender.ply",
        "output_dir": CSV_DIR / "ES300h_left_fender",
        "paint_dir": [0.0, -1.0, 0.0],
        "scan_dir": [-1.0, 0.0, 0.0],
        "slicing_method": "surface_adaptive",
    },
    {
        "name": "ES300h_trunk",
        "input_csv": DATA_DIR / "ES300h_trunk.ply",
        "output_dir": CSV_DIR / "ES300h_trunk",
        "paint_dir": [0.0, 0.0, -1.0],
        "scan_dir": [0.0, 1.0, 0.0],
        "slicing_method": "surface_adaptive",
    },
    {
        "name": "NX350h_right_fender",
        "input_csv": DATA_DIR / "NX350h_right_fender.ply",
        "output_dir": CSV_DIR / "NX350h_right_fender",
        "paint_dir": [0.0, -1.0, 0.0],
        "scan_dir": [-1.0, 0.0, 0.0],
        "slicing_method": "surface_adaptive",
    },
    {
        "name": "Tesla_Model_Y_front_bumper",
        "input_csv": DATA_DIR / "Tesla_Model_Y_front_bumper.ply",
        "output_dir": CSV_DIR / "Tesla_Model_Y_front_bumper",
        "paint_dir": [0.0, 0.0, -1.0],
        "scan_dir": [0.0, 1.0, 0.0],
        "slicing_method": "axis",
    },


    # {
    #     "name": "front_hood",
    #     "input_csv": DATA_DIR / "reference_path_requirements" / "front_hood" / "solid_pearl_default" / "model-final-new.ply",
    #     "output_dir": DATA_DIR / "reference_path_requirements" / "front_hood" / "solid_pearl_default" / "test",
    #     "paint_dir": [0.0, 0.0, -1.0],
    #     "scan_dir": [0.0, 1.0, 0.0],
    # },
    # {
    #     "name": "right_front_door",
    #     "input_csv": DATA_DIR / "reference_path_requirements" / "right_front_door" / "solid_pearl_default" / "model-final-new.ply",
    #     "output_dir": DATA_DIR / "reference_path_requirements" / "right_front_door" / "solid_pearl_default" / "test",
    #     "paint_dir": [0.0, 1.0, 0.0],
    #     "scan_dir": [1.0, 0.0, 0.0],
    # },
    # {
    #     "name": "right_front_fender",
    #     "input_csv": DATA_DIR / "reference_path_requirements" / "right_front_fender" / "solid_pearl_default" / "model-final-new.ply",
    #     "output_dir": DATA_DIR / "reference_path_requirements" / "right_front_fender" / "solid_pearl_default" / "test",
    #     "paint_dir": [0.0, 1.0, 0.0],
    #     "scan_dir": [1.0, 0.0, 0.0],
    # },
    # {
    #     "name": "right_rear_door",
    #     "input_csv": DATA_DIR / "reference_path_requirements" / "right_rear_door" / "solid_pearl_default" / "model-final-new.ply",
    #     "output_dir": DATA_DIR / "reference_path_requirements" / "right_rear_door" / "solid_pearl_default" / "test",
    #     "paint_dir": [0.0, -1.0, 0.0],
    #     "scan_dir": [-1.0, 0.0, 0.0],
    # },
    # {
    #     "name": "trunk",
    #     "input_csv": DATA_DIR / "reference_path_requirements" / "trunk" / "ES300h rear hood.ply",
    #     "output_dir": DATA_DIR / "reference_path_requirements" / "trunk" / "test",
    #     "paint_dir": [0.0, 0.0, -1.0],
    #     "scan_dir": [0.0, 1.0, 0.0],
    # },
]


def generate_case(case: dict) -> dict:
    point_cloud_xyz, surface_colors = load_surface_data(case["input_csv"])
    case["output_dir"].mkdir(parents=True, exist_ok=True)
    for stale_csv in case["output_dir"].glob("trajectory_*.csv"):
        stale_csv.unlink()

    result = generate_painting_trajectory_debug(
        point_cloud_xyz,
        extraction_kwargs={
            "paint_dir": case["paint_dir"],
            "scan_dir": case["scan_dir"],
            "slicing_method": case["slicing_method"],
        },
        structuring_kwargs={
            "save_csv": True,
            "csv_output_dir": str(case["output_dir"]),
            "painting_trajectory_csv_name": "painting_trajectory.csv",
            "trajectory_csv_name_format": "trajectory_{row_index:03d}.csv",
        },
    )

    row_count = len(result["painting_trajectory"].rows)
    point_count = sum(len(row.points) for row in result["painting_trajectory"].rows)
    print(
        f"[{case['name']}] points={len(point_cloud_xyz)} "
        f"mode={case['slicing_method']} rows={row_count} "
        f"trajectory_points={point_count}"
    )
    print(f"[{case['name']}] csv={result['painting_trajectory_csv_files']}")
    if SAVE_DEBUG_HTML:
        debug_html_path = write_debug_plotly_html(
            case,
            point_cloud_xyz,
            surface_colors,
            result,
        )
        print(f"[{case['name']}] debug_html={debug_html_path}")
        if OPEN_DEBUG_HTML:
            webbrowser.open(debug_html_path.as_uri())
    return result


def main() -> int:
    for case in CASES:
        generate_case(case)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
