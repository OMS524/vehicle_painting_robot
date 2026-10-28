#!/usr/bin/env python3
# 실행: conda activate vehicle_painting_robot_path_planning
# python path_planning/scripts/visualize_path.py <점군.ply> <경로.csv> --open
"""저장된 PLY/CSV/JSON을 시각화합니다. 경로를 다시 생성하지 않습니다."""

from pathlib import Path
import sys

SOURCE_DIR = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SOURCE_DIR))

from painting_path_planner.visualization import main

if __name__ == "__main__":
    raise SystemExit(main())
