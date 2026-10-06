#!/usr/bin/env python3
"""Generalized ICP (GICP): 양쪽 점군의 국부 표면 구조를 사용."""
from pathlib import Path

from icp_common import Settings, main

# ======================== 사용자가 수정할 설정 ========================
HERE = Path(__file__).resolve().parent
CAPTURE_DIR = HERE.parents[1] / "doosan_a0912_scan_test/log/20260923_220542_796308"
OUTPUT_ROOT = HERE.parent / "log"
REFERENCE_VIEW = None  # 첫 뷰 고정. 다른 기준 뷰 예: "view_02"
SETTINGS = Settings(
    scales=((10.0, 30.0, 60), (5.0, 15.0, 40), (2.0, 6.0, 30)),
    evaluation_voxel_mm=5.0,
    evaluation_distance_mm=10.0,
    output_voxel_mm=2.0,
    generalized_epsilon=0.001,
    roi_min_mm=None,  # 정합/평가용 base 좌표 ROI [x, y, z], mm. 출력 전체 점은 유지.
    roi_max_mm=None,
)
# =====================================================================

if __name__ == "__main__":
    raise SystemExit(main("generalized", CAPTURE_DIR, OUTPUT_ROOT, REFERENCE_VIEW, SETTINGS))
