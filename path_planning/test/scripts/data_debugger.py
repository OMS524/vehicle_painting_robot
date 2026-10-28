#!/usr/bin/env python3
# 실행: conda activate vehicle_painting_robot_planner
# python path_planning/test/scripts/data_debugger.py <데이터.ply> --open
"""기존 테스트 진입점. 공통 시각화는 src의 visualization.py를 사용합니다."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
from painting_path_planner.visualization import main

if __name__ == "__main__":
    raise SystemExit(main())
