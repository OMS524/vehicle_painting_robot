#!/usr/bin/env python3
"""Colored ICP: 형상과 실제 RGB를 함께 사용. 뷰 구분색은 사용 불가."""
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
    colored_lambda_geometric=0.968,  # 형상 오차 가중치. 나머지는 색상 오차.
    roi_min_mm=None,  # 정합/평가용 base 좌표 ROI [x, y, z], mm. 출력 전체 점은 유지.
    roi_max_mm=None,
)
# =====================================================================

if __name__ == "__main__":
    raise SystemExit(main("colored", CAPTURE_DIR, OUTPUT_ROOT, REFERENCE_VIEW, SETTINGS))
