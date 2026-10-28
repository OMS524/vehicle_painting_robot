#!/usr/bin/env python3
# conda activate vehicle_painting_robot_scan
# python -s scan_planning/scripts/run_segmentation.py --help
"""segmentation 단계 실행. 공통 상위 제어기 scan_planning.py의 동일 기능을 사용한다."""
from scan_planning import main

if __name__ == "__main__":
    raise SystemExit(main(default_stage="segmentation"))
