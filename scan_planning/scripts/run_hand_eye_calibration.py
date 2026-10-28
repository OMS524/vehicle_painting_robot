#!/usr/bin/env python3
# conda activate vehicle_painting_robot_scan
# python -s scan_planning/scripts/run_hand_eye_calibration.py --check
# RGB/C2D 선택: src/hand_eye_calibration/hand_eye_calibration.yaml의 camera.image_source
# 카메라만 미리보기: --preview / 실제 이동·촬영·계산: --execute
"""src/hand_eye_calibration의 독립 보정 도구 진입점."""
from pathlib import Path
import runpy
import sys

if __name__ == "__main__":
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
    runpy.run_module("hand_eye_calibration.hand_eye_calibration", run_name="__main__")
