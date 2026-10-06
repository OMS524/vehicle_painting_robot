"""하드웨어 없는 편집/재생성/CSV 저장 테스트. planner 환경에서 실행."""
import copy
import csv
import json
from pathlib import Path
import pickle
import tempfile
import unittest

import numpy as np

import workstation_backend as backend

PLANNER = Path(__file__).resolve().parents[2] / "painting_trajectory_planner/path_planner"
Config, _, spline, _ = backend._configure_planner_imports(PLANNER)


def make_state(zigzag=False, descending=False):
    rows = [np.array([[x, y, 0.0] for x in [0, .04, .08, .12, .16, .18]]) for y in [0, .1]]
    settings = backend.yaml.safe_load((PLANNER / "config/painting_trajectory.yaml").read_text())["painting_trajectory"]
    settings["spline"].update(raster_zigzag=zigzag, endpoint_extension_length=0.0, final_point_spacing=.04)
    config = Config.from_dict(settings)
    state = spline({
        "offset_rows": rows,
        "offset_row_normals_world": [np.tile([0, 0, 1], (len(row), 1)) for row in rows],
        "offset_row_profiles": [{"slice_plane_origin_work": [0, y, 0],
                                 "slice_plane_normal_work": [0, 1, 0]} for y in [0, .1]],
        "work_frame_world": np.eye(3), "work_origin_world": np.zeros(3),
        "spline_start_side": "top" if descending else "bottom",
    }, **config.spline.to_kwargs())
    # Represent the sorted trajectory-to-source mapping as in real sliced data.
    state["paint_spline_row_slice_positions"] = [0, .1]
    state["_workstation_config"] = config.to_dict()
    state["_workstation_edit_controls"] = backend._control_points_from_state(state)
    state["_workstation_edit_offset_row_indices"] = backend._trajectory_offset_row_indices(state)
    state["_workstation_edit_source_row_indices"] = backend._structured_source_row_indices(state)
    return state


def controls(state):
    return copy.deepcopy(state["_workstation_edit_controls"])


def delete(state, points, rows=()):
    ids = [point["id"] for point in points]
    kept = [p for p in controls(state) if p["id"] not in ids and p["rowIndex"] not in rows]
    return backend._apply_control_points(state, kept, list(rows), ids)


class PointDeletionTests(unittest.TestCase):
    def test_noop_preserves_offset_rows(self):
        state = make_state()
        before = [row.copy() for row in state["offset_rows"]]
        backend._apply_control_points(state, controls(state))
        for old, new in zip(before, state["offset_rows"]):
            np.testing.assert_array_equal(old, new)

    def test_interior_delete_rebuilds_even_without_movement(self):
        state = make_state()
        point = controls(state)[2]
        before_other = state["offset_rows"][1].copy()
        result = delete(state, [point])
        self.assertNotIn(point["id"], [p["id"] for p in result])
        self.assertFalse(np.any(np.linalg.norm(state["offset_rows"][0] - point["position"], axis=1) < 1e-9))
        np.testing.assert_allclose(state["offset_rows"][0][-1], [.18, 0, 0])
        np.testing.assert_array_equal(state["offset_rows"][1], before_other)

    def test_endpoint_deletion_respects_zigzag_and_sorted_rows(self):
        for zigzag in [False, True]:
            for descending in [False, True]:
                for row_index in [0, 1]:
                    for endpoint in [0, -1]:
                        with self.subTest(zigzag=zigzag, descending=descending, row=row_index, endpoint=endpoint):
                            state = make_state(zigzag, descending)
                            row_controls = [p for p in controls(state) if p["rowIndex"] == row_index]
                            point = row_controls[endpoint]
                            offset_index = state["_workstation_edit_offset_row_indices"][row_index]
                            deleted_x = point["position"][0]
                            delete(state, [point])
                            xs = state["offset_rows"][offset_index][:, 0]
                            self.assertTrue(np.all(np.diff(xs) > 0))
                            if deleted_x > .1:
                                self.assertAlmostEqual(xs.max(), .12)
                            else:
                                self.assertAlmostEqual(xs.min(), .04)
                                self.assertAlmostEqual(xs.max(), .18)

    def test_multiple_delete_minimum_two(self):
        state = make_state()
        first_row = [p for p in controls(state) if p["rowIndex"] == 0]
        result = delete(state, first_row[1:-1])
        self.assertEqual(len([p for p in result if p["rowIndex"] == 0]), 2)
        state = make_state()
        before = state["offset_rows"][0].copy()
        with self.assertRaisesRegex(ValueError, "최소 2"):
            delete(state, first_row[:-1])
        np.testing.assert_array_equal(before, state["offset_rows"][0])

    def test_deleted_point_and_whole_row_can_overlap(self):
        state = make_state()
        all_controls = controls(state)
        result = delete(state, [all_controls[1], all_controls[-1]], rows=[1])
        self.assertEqual({p["rowIndex"] for p in result}, {0})
        self.assertEqual(len(state["offset_rows"][1]), 0)

    def test_undeclared_missing_and_contradictory_points_rejected(self):
        state = make_state()
        points = controls(state)
        with self.assertRaisesRegex(ValueError, "구성이 일치"):
            backend._apply_control_points(state, points[1:])
        with self.assertRaisesRegex(ValueError, "구성이 일치"):
            backend._apply_control_points(state, points, [], [points[0]["id"]])

    def test_invalid_deletion_ids_rejected(self):
        for ids in ["control-0-0", [1], ["missing"], ["control-0-0", "control-0-0"]]:
            state = make_state()
            with self.subTest(ids=ids), self.assertRaises(ValueError):
                backend._apply_control_points(state, controls(state), [], ids)

    def test_delete_and_move_still_projects_onto_plane(self):
        state = make_state()
        points = controls(state)
        removed = points.pop(2)
        points[1]["position"] = [.05, .8, 0]
        result = backend._apply_control_points(state, points, [], [removed["id"]])
        moved = next(p for p in result if p["id"] == points[1]["id"])
        np.testing.assert_allclose(moved["position"], [.05, 0, 0])

    def test_regenerate_twice_and_save_matches_edit_controls(self):
        with tempfile.TemporaryDirectory() as tmp:
            session = Path(tmp) / "session"
            session.mkdir()
            scan = session / "scan.csv"
            np.savetxt(scan, [[0, 0, 0], [.1, 0, 0]], delimiter=",")
            (session / "painting_trajectory.yaml").write_text("painting_trajectory: {}\n")
            state = make_state(zigzag=True)
            with (session / backend.STATE_FILE_NAME).open("wb") as stream:
                pickle.dump(state, stream)
            for iteration in range(2):
                points = controls(state)
                removed = next(p for p in reversed(points) if p["rowIndex"] == 0)
                response = backend.regenerate({
                    "plannerRoot": str(PLANNER), "sessionDir": str(session),
                    "controlPoints": [p for p in points if p["id"] != removed["id"]],
                    "deletedRowIndices": [], "deletedPointIds": [removed["id"]],
                })
                with (session / backend.STATE_FILE_NAME).open("rb") as stream:
                    state = pickle.load(stream)
                updated = json.loads((session / backend.CONTROL_POINTS_FILE_NAME).read_text())["points"]
                with (session / "generated/painting_trajectory.csv").open() as stream:
                    csv_points = list(csv.DictReader(stream))
                lookup = {(int(p["row_index"]), int(p["point_index"])): p for p in csv_points}
                self.assertEqual(response["pointCount"], len(csv_points))
                self.assertEqual(response["controlPointCount"], len(updated))
                for point in updated:
                    exported = lookup[(point["rowIndex"], point["pointIndex"])]
                    np.testing.assert_allclose(point["position"], [float(exported[f"position_{a}"]) for a in "xyz"], atol=1e-8)
                xs = [p["position"][0] for p in updated if p["rowIndex"] == 0]
                self.assertLess(max(xs), removed["position"][0] - .01)
                np.testing.assert_allclose(np.diff(xs), .04, atol=1e-8)
            destination = Path(tmp) / "planner"
            destination.mkdir()
            saved = backend.complete({"plannerRoot": str(destination), "sessionDir": str(session), "scanPath": str(scan)})
            self.assertEqual((Path(saved["outputDirectory"]) / "painting_trajectory.csv").read_bytes(),
                             (session / "generated/painting_trajectory.csv").read_bytes())


if __name__ == "__main__":
    unittest.main()
