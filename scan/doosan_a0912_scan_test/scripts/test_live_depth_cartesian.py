"""Offline regression tests for retrying live IK without restarting hardware."""
from concurrent.futures import Future
import contextlib
import ctypes
import io
import json
from pathlib import Path
import queue
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

import live_depth_cartesian as live
import robot_scan_worker as worker
from three_view_scan import IKNoSolutionError, RobotClient

ROOT = Path(__file__).resolve().parents[3]
WRAPPER = worker.load_wrapper(ROOT / "robot_control/doosan_a0912_controller/test/scripts/joint_target_test.py")
STATE = {"joint_deg": [0.0] * 6, "joint_velocity_deg_s": [0.0] * 6, "motion_running": False}
TARGET = [10.0] * 6
SETTLED = {**STATE, "target_joint_deg": TARGET, "max_target_error_deg": 0.1}
SETTINGS = {"ip": "unused", "port": 0, "rt_port": 0, "connect_timeout_sec": 1,
            "stationary_velocity_deg_s": 0.05, "capture_drift_deg": 0.05,
            "move_timeout_sec": 1, "settle_timeout_sec": 1}


class WrapperTests(unittest.TestCase):
    def test_native_ik_rejection_is_distinct_from_sdk_query_failure(self):
        fp = ctypes.POINTER(ctypes.c_float)
        signature = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, fp, fp, fp,
                                     ctypes.POINTER(ctypes.c_int))
        robot = WRAPPER.DoosanRobot.__new__(WRAPPER.DoosanRobot)
        robot._handle, robot._closed = 1, False
        for code in (-2, -3, -1):
            with self.subTest(code=code):
                def solve(handle, pose, reference, joint, solution):
                    solution[0] = code
                    return 0
                robot._lib = SimpleNamespace(doosan_controller_solve_closest_ik=signature(solve))
                with self.assertRaises(RuntimeError) as caught:
                    robot.solve_closest_ik([0] * 6, [0] * 6)
                error = caught.exception
                if code in (-2, -3):
                    self.assertIsInstance(error, WRAPPER.IKNoSolutionError)
                    self.assertEqual(error.sdk_status, -1 - code)
                else:
                    self.assertNotIsInstance(error, WRAPPER.IKNoSolutionError)


class WorkerTests(unittest.TestCase):
    def run_requests(self, error, recover):
        robot = Mock()
        robot.read_actual_joint_state.return_value = STATE
        wrapper = SimpleNamespace(DoosanRobot=Mock(return_value=robot), IKNoSolutionError=WRAPPER.IKNoSolutionError)
        ik = Mock()
        ik.solve.side_effect = [error, {"target_joint_deg": TARGET, "ik": {"method": "doosan_sdk_ikin"}}]
        requests = [
            {"id": 1, "op": "initialize", "settings": SETTINGS},
            {"id": 2, "op": "move_cartesian", "pose_mm_deg": [0] * 6, "recover_ik_failure": recover},
            {"id": 3, "op": "move_cartesian", "pose_mm_deg": [1] * 6, "recover_ik_failure": recover},
            {"id": 4, "op": "close"},
        ]
        protocol = io.StringIO()
        # Record whether the worker has shut down when each reply is emitted.
        shutdown_counts = []
        protocol.flush = lambda: shutdown_counts.append(robot.shutdown.call_count)
        with patch.object(worker.sys, "stdin", io.StringIO("\n".join(map(json.dumps, requests)))), \
             patch.object(worker.signal, "signal"), patch.object(worker, "make_ik", return_value=ik), \
             patch.object(worker, "move_and_settle", return_value=dict(SETTLED)) as move, \
             contextlib.redirect_stderr(io.StringIO()):
            worker.serve(wrapper, "unused", protocol)
        return [json.loads(line) for line in protocol.getvalue().splitlines()], move, robot, shutdown_counts

    def test_live_ik_failure_then_success_keeps_same_worker(self):
        for sdk_status in (1, 2):
            with self.subTest(sdk_status=sdk_status):
                replies, move, robot, counts = self.run_requests(WRAPPER.IKNoSolutionError(sdk_status), True)
                self.assertEqual([r["id"] for r in replies], [1, 2, 3, 4])
                self.assertEqual(replies[1]["error_code"], "ik_no_solution")
                self.assertIs(replies[1]["recoverable"], True)
                self.assertEqual(replies[1]["sdk_status"], sdk_status)
                self.assertEqual(replies[2]["result"]["target_joint_deg"], TARGET)
                move.assert_called_once_with(robot, TARGET, SETTINGS)
                robot.initialize.assert_called_once()
                self.assertEqual(counts, [0, 0, 0, 1])
                robot.shutdown.assert_called_once()

    def test_automatic_scan_still_stops_on_ik_failure(self):
        for recover in (False, "true"):
            replies, move, robot, _ = self.run_requests(WRAPPER.IKNoSolutionError(1), recover)
            self.assertEqual([r["id"] for r in replies], [1, 2])
            self.assertNotIn("recoverable", replies[-1])
            move.assert_not_called()
            robot.shutdown.assert_called_once()

    def test_other_sdk_errors_are_not_treated_as_no_solution(self):
        replies, move, robot, _ = self.run_requests(RuntimeError("SDK IK failed: connection lost"), True)
        self.assertEqual([r["id"] for r in replies], [1, 2])
        self.assertNotIn("recoverable", replies[-1])
        move.assert_not_called()
        robot.shutdown.assert_called_once()


class ClientTests(unittest.TestCase):
    def test_typed_error_and_next_reply_keep_protocol_sequence(self):
        client = RobotClient.__new__(RobotClient)
        client.cfg, client.sequence = {"robot": SETTINGS}, 0
        client.process = SimpleNamespace(stdin=io.StringIO())
        client.responses = queue.Queue()
        client.responses.put(json.dumps({"id": 1, "error": "no solution", "error_code": "ik_no_solution",
                                         "sdk_status": 1, "recoverable": True}))
        client.responses.put(json.dumps({"id": 2, "result": SETTLED}))
        with self.assertRaises(IKNoSolutionError) as caught:
            client.move_cartesian([0] * 6, recover_ik_failure=True)
        self.assertEqual(caught.exception.sdk_status, 1)
        self.assertEqual(client.move_cartesian([1] * 6, recover_ik_failure=True), SETTLED)
        requests = [json.loads(line) for line in client.process.stdin.getvalue().splitlines()]
        self.assertEqual([r["id"] for r in requests], [1, 2])
        self.assertTrue(all(r["recover_ik_failure"] is True for r in requests))


class LiveLoopTests(unittest.TestCase):
    def run_session(self, first_error):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        cfg = {"output_dir": directory.name, "mounting_tf_confirmed": True, "robot": SETTINGS,
               "camera": {"min_depth_mm": 200, "max_depth_mm": 3000},
               "cartesian": {"target_frame": "link_6"}}
        terminal = SimpleNamespace(lines=queue.Queue(), stop=threading.Event(), start=Mock())
        robot = Mock()
        robot.process.poll.return_value = None
        robot.call.return_value = STATE
        robot.move_cartesian.side_effect = [first_error, SETTLED]
        exits = []

        class RobotContext:
            def __enter__(self):
                return robot

            def __exit__(self, exc_type, exc, tb):
                exits.append(exc_type)

        pool = Mock()

        def submit(fn, *args, **kwargs):
            future = Future()
            try:
                future.set_result(fn(*args, **kwargs))
            except Exception as error:
                future.set_exception(error)
            return future

        pool.submit.side_effect = submit
        cv = Mock()
        cv.getWindowProperty.return_value = 1
        cv.namedWindow.return_value = None
        camera = Mock(device_info={"name": "fake camera"})
        camera.profile.get_width.return_value = 2
        camera.profile.get_height.return_value = 2
        camera.profile.get_fps.return_value = 30
        latest = Mock()
        latest.snapshot.side_effect = lambda: (object(), time.monotonic(), 1)
        tick = 0

        def wait_key(_delay):
            nonlocal tick
            tick += 1
            # Inputs arriving before a completion notification must be discarded.
            commands = {1: "MOVE", 2: "999,0,0,0,0,0", 3: "500,0,900,160,0,-90",
                        4: "999,0,0,0,0,0", 5: "400,0,700,160,0,-90"}
            if tick in commands:
                terminal.lines.put((time.monotonic(), commands[tick]))
            return ord("q") if tick >= 7 else -1

        cv.waitKey.side_effect = wait_key
        stdout = io.StringIO()
        with patch.dict("sys.modules", {"cv2": cv}), patch.dict(live.os.environ, {"DISPLAY": ":fake"}), \
             patch.object(live.sys.stdin, "isatty", return_value=True), \
             patch.object(live, "TerminalInput", return_value=terminal), \
             patch.object(live, "DepthCamera", return_value=camera), \
             patch.object(live, "LatestDepth", return_value=latest), \
             patch.object(live, "RobotClient", return_value=RobotContext()), \
             patch.object(live, "ThreadPoolExecutor", return_value=pool), \
             patch.object(live, "decode_depth", return_value=np.ones((2, 2))), \
             patch.object(live, "depth_image", return_value=np.zeros((2, 2, 3), dtype=np.uint8)), \
             contextlib.redirect_stdout(stdout):
            caught = None
            try:
                live.run_live(cfg, SimpleNamespace(xml="fake model"))
            except Exception as error:
                caught = error
        events = [json.loads(line) for line in next(Path(directory.name).rglob("events.jsonl")).read_text().splitlines()]
        camera.pipeline.start.assert_called_once()
        camera.pipeline.stop.assert_called_once()
        return caught, robot, events, exits, stdout.getvalue()

    def test_gui_continues_after_ik_failure_then_moves_without_rearming(self):
        caught, robot, events, exits, output = self.run_session(IKNoSolutionError("no solution", 1))
        self.assertIsNone(caught)
        robot.call.assert_called_once()
        self.assertEqual(robot.move_cartesian.call_count, 2)
        self.assertTrue(all(c.kwargs == {"recover_ik_failure": True} for c in robot.move_cartesian.call_args_list))
        self.assertEqual([e["event"] for e in events], ["initialize_requested", "initialize_complete",
                         "input_discarded", "move_requested", "move_rejected", "input_discarded",
                         "move_requested", "move_complete", "exit_requested"])
        self.assertEqual(exits, [None])
        self.assertIn("IK 실패", output)
        self.assertIn("다음 x,y,z,roll,pitch,yaw 입력 대기", output)

    def test_gui_still_exits_on_motion_or_connection_error(self):
        for error in (RuntimeError("connection lost"), TimeoutError("motion timed out")):
            with self.subTest(error=error):
                caught, robot, events, exits, _ = self.run_session(error)
                self.assertIs(caught, error)
                robot.move_cartesian.assert_called_once()
                self.assertNotIn("move_rejected", [e["event"] for e in events])
                self.assertEqual(exits, [type(error)])


if __name__ == "__main__":
    unittest.main()
