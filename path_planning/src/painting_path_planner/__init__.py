"""도장 경로 알고리즘. 숫자 접두어 파일은 importlib로 불러옵니다."""

from importlib import import_module

# 1_slicing이 2_correction의 hull 계산을 사용하므로 correction을 먼저 로드합니다.
correction = import_module(".2_correction", __name__)
slicing = import_module(".1_slicing", __name__)
offset = import_module(".3_offset", __name__)
spline = import_module(".4_spline", __name__)
structuring = import_module(".structuring", __name__)

generate_surface_spline_rows = slicing.generate_surface_spline_rows
correct_surface_spline_rows = correction.correct_surface_spline_rows
generate_offset_rows = offset.generate_offset_rows
generate_paint_spline_rows = spline.generate_paint_spline_rows
structure_painting_trajectory = structuring.structure_painting_trajectory
