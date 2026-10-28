#!/usr/bin/env python3
# 실행: conda activate vehicle_painting_robot_planner
# python -s -B path_planning/test/scripts/test_row_order.py
"""보정 슬라이스 삽입 순서와 지그재그/CSV 순서 회귀 검사. 파일 출력 없음."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
import path_planning as planner
from painting_path_planner import slicing as extraction
from painting_path_planner.structuring import structure_painting_trajectory


def spline_result(side="top", method="surface_adaptive", fallback=False):
    # 기울어진 슬라이싱의 실제 회귀 사례처럼 좌표 중앙값 순서가 뒤집힌 행.
    medians = [0.3, 0.2, 0.1, 0.15, 0.0]
    positions = []
    for i, value in enumerate(medians):
        row = np.column_stack((np.linspace(0, 1, 3), [value] * 3, [i * .01] * 3))
        positions.append(row[::-1].copy() if i % 2 else row)
    orientations = [np.tile([0., 0., 0., 1.], (3, 1)) for _ in positions]
    masks = [np.array([False, True, i % 2 == 0]) for i in range(len(positions))]
    result = {
        "spline_start_side": side, "slicing_method": method,
        "paint_spline_row_slice_positions": medians,
    }
    prefix = "paint_spline" if fallback else "paint_trajectory"
    result[prefix + ("_rows" if fallback else "_position_rows")] = positions
    result[prefix + "_orientation_rows_xyzw"] = orientations
    result[prefix + "_paint_mask_rows"] = masks
    result[prefix + "_arc_length_rows"] = [np.array([0., .5, 1.]) for _ in positions]
    return result, positions, orientations, masks


class RowOrderTests(unittest.TestCase):
    def test_structure_preserves_rows_points_and_zigzag_for_all_modes_and_sides(self):
        for method in ("axis", "surface_adaptive"):
            for side in ("top", "bottom", "left", "right"):
                for fallback in (False, True):
                    with self.subTest(method=method, side=side, fallback=fallback):
                        data, positions, quats, masks = spline_result(side, method, fallback)
                        trajectory = structure_painting_trajectory(data)
                        self.assertEqual([r.row_index for r in trajectory.rows], list(range(5)))
                        for i, row in enumerate(trajectory.rows):
                            np.testing.assert_array_equal([p.position for p in row.points], positions[i])
                            np.testing.assert_array_equal([p.orientation for p in row.points], quats[i])
                            np.testing.assert_array_equal([p.paint for p in row.points], masks[i])
                            np.testing.assert_array_equal([p.s for p in row.points], [0., .5, 1.])
                        directions = [np.sign(row.points[-1].position[0] - row.points[0].position[0])
                                      for row in trajectory.rows]
                        self.assertEqual(directions, [1, -1, 1, -1, 1])

    def test_missing_equal_or_nonfinite_medians_do_not_reorder(self):
        for values in ([], [0.] * 5, [0.3, np.nan, 0.1, 0.15, np.inf]):
            with self.subTest(values=values):
                data, positions, _, _ = spline_result()
                data["paint_spline_row_slice_positions"] = values
                trajectory = structure_painting_trajectory(data)
                for row, expected in zip(trajectory.rows, positions):
                    np.testing.assert_array_equal([p.position for p in row.points], expected)

    def test_curvature_profiles_are_inserted_between_original_profiles(self):
        points = np.array([[0., 0., 0.], [1., 0., 0.]])
        identity = lambda p: np.asarray(p).copy()
        for angle, expected in ((40., [0., .5, 1.]), (70., [0., 1 / 3, 2 / 3, 1.])):
            with self.subTest(angle=angle):
                profiles = []
                for i, degrees in enumerate((0., angle)):
                    theta = np.deg2rad(degrees)
                    normal = np.array([0., -np.cos(theta), np.sin(theta)])
                    profiles.append(extraction._make_profile(
                        i, float(i), points, [0., -float(i), 0.], normal,
                        [1., 0., 0.], np.cross(normal, [1., 0., 0.]),
                        identity, "surface_adaptive"))
                with patch.object(extraction, "_plane_slab_indices", return_value=np.arange(2)):
                    result = extraction._insert_curvature_slice_profiles(
                        profiles, points, identity, .005, 2, 20.)
                self.assertEqual([p["row_index"] for p in result], list(range(len(expected))))
                np.testing.assert_allclose([p["slice_position"] for p in result], expected)
                np.testing.assert_allclose([p["slice_plane_origin_work"][1] for p in result],
                                           -np.asarray(expected))

    def test_csv_export_preserves_the_structured_sequence(self):
        data, _, _, _ = spline_result()
        trajectory = structure_painting_trajectory(data)
        writes = {}
        def capture_write(path, rows):
            writes[Path(path).name] = list(rows)
        with patch.object(planner.os, "makedirs"), patch.object(planner, "_write_csv", side_effect=capture_write):
            planner.save_painting_trajectory_csv_files(trajectory, csv_output_dir="/tmp/row-order-regression")
        expected = []
        for row in trajectory.rows:
            encoded = [planner._csv_row(trajectory.frame_id, row.row_index, i, p)
                       for i, p in enumerate(row.points)]
            self.assertEqual(writes[f"trajectory_{row.row_index:03d}.csv"], encoded)
            expected.extend(encoded)
        self.assertEqual(writes["painting_trajectory.csv"], expected)


if __name__ == "__main__":
    unittest.main(verbosity=2)
