"""Offline wrapper tests. Never initialize/connect/move the robot.

Run in vehicle_painting_robot_control:
python -B -s -m unittest discover -s robot_controller/doosan_a0912_controller/test/tests -v
"""
import ctypes
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import MagicMock, patch


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("scan_wrapper", ROOT / "scripts/joint_target_test.py")
wrapper = importlib.util.module_from_spec(spec)
spec.loader.exec_module(wrapper)


def set_int(pointer, value):
    ctypes.cast(pointer, ctypes.POINTER(ctypes.c_int))[0] = value


class WrapperTests(unittest.TestCase):
    def setUp(self):
        self.lib = MagicMock()
        self.lib.doosan_controller_create.return_value = 123
        with patch.object(wrapper.ctypes, "CDLL", return_value=self.lib), patch.object(Path, "is_file", return_value=True):
            self.robot = wrapper.DoosanRobot(library_path="/tmp/mock-controller.so")
        self.addCleanup(self.robot.shutdown)

    def test_constructor_accepts_existing_default(self):
        with patch.object(wrapper.ctypes, "CDLL", return_value=self.lib), patch.object(Path, "is_file", return_value=True):
            robot = wrapper.DoosanRobot()
            robot.shutdown()

    def test_joint_state_and_motion_flag(self):
        for busy in (0, 1):
            def query(handle, pos, vel, running):
                pos[:] = [1, 2, 3, 4, 5, 6]
                vel[:] = [0.5] * 6
                set_int(running, busy)
                return 1
            self.lib.doosan_controller_read_actual_joint_state.side_effect = query
            self.assertEqual(self.robot.read_actual_joint_state(), {
                "joint_deg": [1, 2, 3, 4, 5, 6], "joint_velocity_deg_s": [0.5] * 6,
                "motion_running": bool(busy)})

    def test_cartesian_units_and_fields(self):
        def query(handle, flange, tcp):
            flange[:] = [100, 200, 300, 90, 45, -90]
            tcp[:] = [110, 220, 330, 90, 45, -90]
            return 1
        self.lib.doosan_controller_read_actual_cartesian_state.side_effect = query
        self.assertEqual(self.robot.read_actual_cartesian_state(), {
            "flange_pose_mm_zyz_deg": [100, 200, 300, 90, 45, -90],
            "tcp_pose_mm_zyz_deg": [110, 220, 330, 90, 45, -90]})

    def test_query_failures_do_not_return_zero_states(self):
        self.lib.doosan_controller_read_actual_joint_state.return_value = 0
        self.lib.doosan_controller_read_actual_cartesian_state.return_value = 0
        with self.assertRaises(RuntimeError):
            self.robot.read_actual_joint_state()
        with self.assertRaises(RuntimeError):
            self.robot.read_actual_cartesian_state()

    def test_nonfinite_measurements_rejected(self):
        def joints(handle, pos, vel, running):
            pos[0] = float("nan")
            set_int(running, 0)
            return 1
        self.lib.doosan_controller_read_actual_joint_state.side_effect = joints
        with self.assertRaises(RuntimeError):
            self.robot.read_actual_joint_state()
        def cartesian(handle, flange, tcp):
            tcp[5] = float("inf")
            return 1
        self.lib.doosan_controller_read_actual_cartesian_state.side_effect = cartesian
        with self.assertRaises(RuntimeError):
            self.robot.read_actual_cartesian_state()

    def test_ik_success_does_not_move_or_initialize(self):
        def solve(handle, pose, reference, target, solution, status):
            self.assertEqual(list(pose), [100, 200, 300, 90, 45, -90])
            self.assertEqual(list(reference), [0, 1, 2, 3, 4, 5])
            target[:] = [10, 20, 30, 40, 50, 60]
            set_int(solution, 3)
            set_int(status, 0)
            return 1
        self.lib.doosan_controller_solve_closest_ik.side_effect = solve
        self.assertEqual(self.robot.solve_closest_ik([100, 200, 300, 90, 45, -90], list(range(6))), {
            "target_joint_deg": [10, 20, 30, 40, 50, 60], "solution_space": 3, "sdk_status": 0})
        called = [call[0] for call in self.lib.mock_calls]
        self.assertEqual(called, ["doosan_controller_create", "doosan_controller_solve_closest_ik"])

    def test_ik_error_categories(self):
        for code, sdk_status, error in ((0, 1, wrapper.IKNoSolutionError),
                                        (0, 2, wrapper.IKNoSolutionError),
                                        (-1, -1, RuntimeError), (0, -1, RuntimeError)):
            def solve(handle, pose, reference, target, solution, status):
                set_int(status, sdk_status)
                return code
            self.lib.doosan_controller_solve_closest_ik.side_effect = solve
            with self.assertRaises(error) as caught:
                self.robot.solve_closest_ik([0] * 6, [0] * 6)
            self.assertIs(type(caught.exception), error)
            if error is wrapper.IKNoSolutionError:
                self.assertEqual(caught.exception.sdk_status, sdk_status)

    def test_invalid_ik_vectors_rejected_before_sdk(self):
        invalid = ([], [0] * 5, [0] * 7, [float("nan")] * 6, [float("inf")] * 6,
                   [1e100] * 6, [True] * 6, ["1"] * 6, None)
        for values in invalid:
            with self.subTest(values=values):
                with self.assertRaises(ValueError):
                    self.robot.solve_closest_ik(values, [0] * 6)
                with self.assertRaises(ValueError):
                    self.robot.solve_closest_ik([0] * 6, values)
        self.lib.doosan_controller_solve_closest_ik.assert_not_called()

    def test_invalid_ik_outputs_rejected(self):
        for sol, value in ((8, 0), (-1, 0), (0, float("nan"))):
            def solve(handle, pose, reference, target, solution, status):
                target[0] = value
                set_int(solution, sol)
                set_int(status, 0)
                return 1
            self.lib.doosan_controller_solve_closest_ik.side_effect = solve
            with self.assertRaises(RuntimeError):
                self.robot.solve_closest_ik([0] * 6, [0] * 6)

    def test_queries_after_shutdown_rejected(self):
        self.robot.shutdown()
        for query in (self.robot.read_actual_joint_state, self.robot.read_actual_cartesian_state,
                      lambda: self.robot.solve_closest_ik([0] * 6, [0] * 6)):
            with self.assertRaises(RuntimeError):
                query()


@unittest.skipUnless(wrapper.LIB_PATH.is_file(), "build native library first")
class NativeDisconnectedTests(unittest.TestCase):
    def test_native_queries_require_connection(self):
        # Constructor allocates a controller only. No initialize(), network or servo command.
        robot = wrapper.DoosanRobot()
        try:
            for query in (robot.read_actual_joint_state, robot.read_actual_cartesian_state,
                          lambda: robot.solve_closest_ik([0] * 6, [0] * 6)):
                with self.assertRaises(RuntimeError) as caught:
                    query()
                self.assertNotIsInstance(caught.exception, wrapper.IKNoSolutionError)
        finally:
            robot.shutdown()

    def test_null_c_handles_fail_without_overwriting_output(self):
        robot = wrapper.DoosanRobot()
        try:
            a, b = wrapper.Float6(*([123] * 6)), wrapper.Float6(*([456] * 6))
            running = ctypes.c_int(99)
            self.assertEqual(robot._lib.doosan_controller_read_actual_joint_state(None, a, b, ctypes.byref(running)), 0)
            self.assertEqual(robot._lib.doosan_controller_read_actual_cartesian_state(None, a, b), 0)
            solution, status = ctypes.c_int(99), ctypes.c_int(99)
            self.assertEqual(robot._lib.doosan_controller_solve_closest_ik(
                None, a, b, a, ctypes.byref(solution), ctypes.byref(status)), -1)
            self.assertEqual(status.value, -1)
            self.assertEqual(solution.value, 99)
            self.assertEqual(running.value, 99)
            self.assertEqual(list(a), [123] * 6)
            self.assertEqual(list(b), [456] * 6)
        finally:
            robot.shutdown()


if __name__ == "__main__":
    unittest.main()
