"""하드웨어 없이 정지 판정과 실제 관절각 기반 정합을 검증한다."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import yaml

import robot_scan_worker as worker
import three_view_scan as scan


def fixture_config():
    """사용자가 수정 중인 실행 YAML에 의존하지 않는 합성 테스트 설정."""
    return {
        "xacro": str(scan.HERE.parent / "description/xacro/a0912_to_tool_scan.xacro"),
        "control_python": "unused-python", "control_wrapper": "unused-wrapper",
        "control_library": "unused-library", "output_dir": "unused-output",
        "robot": {"port": 12345, "rt_port": 12347, "connect_timeout_sec": 15,
                  "move_timeout_sec": 120, "max_velocity_deg_s": 10.0, "acceleration_deg_s2": 10.0,
                  "target_tolerance_deg": 0.1, "command_time_sec": 0.01, "max_move_delta_deg": 100.0,
                  "settle_sec": 1.0, "settle_timeout_sec": 10.0,
                  "stationary_velocity_deg_s": 0.05, "capture_drift_deg": 0.05},
        "views": [{"name": name, "joint_deg": [j1, -13.0, 95.0, 0.0, 125.0, -90.0]}
                  for name, j1 in (("front", 100.0), ("left", 145.0), ("right", 55.0))],
        "camera": {"width": 0, "height": 0, "fps": 0, "warmup_frames": 30,
                   "capture_timeout_sec": 20.0, "min_depth_mm": 200.0,
                   "max_depth_mm": 3000.0, "min_points": 1000},
        "reconstruction": {"voxel_size_mm": 2.0, "roi_min_mm": None, "roi_max_mm": None},
    }


class Clock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


def state(joints=None, velocity=0.0, running=False):
    return {"joint_deg": list(joints if joints is not None else [0.0] * 6),
            "joint_velocity_deg_s": [velocity] * 6, "motion_running": running}


class SampleRobot:
    def __init__(self, sample):
        self.sample = sample

    def read_actual_joint_state(self):
        return self.sample()


class SimulatedRobot:
    def __init__(self, settings, initial, offset):
        self.settings = settings
        self.joints = list(initial)
        self.offset = offset
        self.commands = []

    def read_actual_joint_state(self):
        return state(self.joints)

    snapshot = read_actual_joint_state

    def velocity_control_to_position(self, target, **kwargs):
        self.commands.append((list(target), kwargs))
        self.joints = (np.asarray(target) + self.offset).tolist()

    def wait_until_motion_done(self, timeout):
        pass

    def move(self, target):
        return worker.move_and_settle(self, target, self.settings)


class SettlingTests(unittest.TestCase):
    def setUp(self):
        self.settings = fixture_config()["robot"]
        self.clock = Clock()
        self.timer = patch.object(worker, "time", self.clock)
        self.timer.start()
        self.addCleanup(self.timer.stop)

    def test_stationary_pose_accepts_target_error_as_diagnostic_only(self):
        for offset in (0.194679, 0.666634, 5.0):
            with self.subTest(offset=offset):
                robot = SampleRobot(lambda: state([offset] * 6))
                start = self.clock.now
                result = worker.wait_settled(robot, [0.0] * 6, self.settings)
                self.assertGreaterEqual(self.clock.now - start, self.settings["settle_sec"])
                self.assertEqual(result["joint_deg"], [offset] * 6)
                self.assertEqual(result["target_error_deg"], [offset] * 6)
                self.assertEqual(result["max_target_error_deg"], offset)

    def test_running_or_measured_speed_still_prevents_capture(self):
        for sample in (state(running=True), state(velocity=0.1)):
            with self.subTest(sample=sample):
                with self.assertRaisesRegex(TimeoutError, "마지막 실제 상태"):
                    worker.wait_settled(SampleRobot(lambda: sample), [0.0] * 6, self.settings)

    def test_cumulative_joint_drift_prevents_capture_even_with_zero_reported_speed(self):
        robot = SampleRobot(lambda: state([self.clock.now * 0.4] * 6))
        with self.assertRaises(TimeoutError):
            worker.wait_settled(robot, [0.0] * 6, self.settings)

    def test_motion_resets_required_stationary_interval(self):
        robot = SampleRobot(lambda: state(running=0.3 <= self.clock.now < 0.6))
        worker.wait_settled(robot, [0.0] * 6, self.settings)
        self.assertGreaterEqual(self.clock.now, 0.6 + self.settings["settle_sec"])

    def test_invalid_measurements_fail_instead_of_appearing_stationary(self):
        for key in ("joint_deg", "joint_velocity_deg_s"):
            for values in ([float("nan")] * 6, [float("inf")] * 6, [0.0] * 5, [False] * 6):
                with self.subTest(key=key, values=values):
                    bad = state()
                    bad[key] = values
                    with self.assertRaisesRegex(RuntimeError, "유효하지 않은 실제 관절 상태"):
                        worker.wait_settled(SampleRobot(lambda: bad), [0.0] * 6, self.settings)

    def test_move_returns_actual_pose_without_correction_motion(self):
        robot = SimulatedRobot(self.settings, [0.0] * 6, np.full(6, 0.194679))
        target = [45.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        result = robot.move(target)
        self.assertEqual(len(robot.commands), 1)
        self.assertEqual(robot.commands[0][0], target)
        self.assertEqual(robot.commands[0][1]["max_velocity"], [self.settings["max_velocity_deg_s"]] * 6)
        self.assertGreater(result["max_target_error_deg"], self.settings["target_tolerance_deg"])

    def test_motion_timeout_still_aborts(self):
        robot = SimulatedRobot(self.settings, [0.0] * 6, np.zeros(6))
        with patch.object(robot, "wait_until_motion_done", side_effect=TimeoutError("motion timeout")):
            with self.assertRaisesRegex(TimeoutError, "motion timeout"):
                robot.move([10.0] * 6)


class CaptureTests(unittest.TestCase):
    def test_rgb_survives_invalid_depth_distance_roi_filtering_and_ply_merge(self):
        import cv2
        import open3d as o3d

        cfg = fixture_config()
        cfg["views"] = [{"name": f"view_{index:02d}", "joint_deg": view["joint_deg"]}
                        for index, view in enumerate((cfg["views"] * 2)[:5], start=1)]
        cfg["camera"].update(capture_color=True, min_points=1)
        cfg["reconstruction"].update(roi_min_mm=[-100, -100, 500], roi_max_mm=[100, 100, 1500])
        robot = SimulatedRobot(cfg["robot"], cfg["views"][0]["joint_deg"], np.zeros(6))
        xyz = np.array([[np.nan, 0, 1], [0, 0, -1], [0, 0, 0.1], [0, 0, 1], [0, 0, 2], [0, 0, 4]])
        rgb = (np.arange(18) * 13).astype(np.uint8).reshape(2, 3, 3)

        class Model:
            xml = "<robot/>"

            def transforms(self, joints):
                return {key: np.eye(4) for key in ("link_6", "bracket_link", scan.DEPTH_FRAME)}

        class Camera:
            def capture(self):
                return scan.CapturedView(np.ones((2, 3), np.uint16), xyz, {}, rgb, rgb)

        with tempfile.TemporaryDirectory() as directory, patch.object(worker, "time", Clock()):
            output = Path(directory)
            scan.capture_sequence(cfg, Model(), robot, Camera(), output)
            for view in cfg["views"]:
                folder = output / view["name"]
                cloud = o3d.io.read_point_cloud(str(folder / "base.ply"))
                np.testing.assert_allclose(np.asarray(cloud.points), [[0, 0, 1]])
                np.testing.assert_allclose(np.asarray(cloud.colors), rgb.reshape(-1, 3)[[3]] / 255.0)
                raw = o3d.io.read_point_cloud(str(folder / "camera_raw.ply"))
                np.testing.assert_allclose(np.asarray(raw.colors), rgb.reshape(-1, 3)[2:] / 255.0)
                saved_rgb = cv2.cvtColor(cv2.imread(str(folder / "color_rgb.png")), cv2.COLOR_BGR2RGB)
                np.testing.assert_array_equal(saved_rgb, rgb)
            merged = o3d.io.read_point_cloud(str(output / "merged.ply"))
            np.testing.assert_allclose(np.asarray(merged.colors), rgb.reshape(-1, 3)[[3]] / 255.0)
            self.assertTrue((output / "merged_view_colors.ply").is_file())
            manifest = json.loads((output / "manifest.json").read_text())
            self.assertEqual(manifest["color_mode"], "rgb")
            self.assertEqual(manifest["status"], "complete")
            self.assertEqual(manifest["raw_point_count"], 5)
            self.assertEqual([v["name"] for v in manifest["views"]], [v["name"] for v in cfg["views"]])
            comparison = o3d.io.read_point_cloud(str(output / "merged_view_colors.ply"))
            expected_color = np.mean([v["view_color_rgb"] for v in manifest["views"]], axis=0)
            np.testing.assert_allclose(np.asarray(comparison.colors), [expected_color], atol=1 / 255)

    def test_six_views_reconstruct_same_surface_from_actual_not_target_joints(self):
        import open3d as o3d

        cfg = fixture_config()
        # A small synthetic surface; no device discovery or real robot wrapper.
        cfg["camera"]["min_points"] = 1000
        cfg["reconstruction"]["roi_min_mm"] = None
        cfg["reconstruction"]["roi_max_mm"] = None
        model = scan.ScanModel(cfg["xacro"])
        offset = np.array([0.7, -0.2, 0.3, 0.0, 0.1, -0.4])
        robot = SimulatedRobot(cfg["robot"], cfg["views"][0]["joint_deg"], offset)
        x, y = np.meshgrid(np.linspace(-0.05, 0.05, 32), np.linspace(-0.05, 0.05, 32))
        local = np.column_stack((x.ravel(), y.ravel(), np.ones(x.size)))
        front_actual = (np.asarray(cfg["views"][0]["joint_deg"]) + offset).tolist()
        surface = scan.transform_points(local, model.transforms(front_actual)[scan.DEPTH_FRAME])
        front, left, right = cfg["views"]
        cfg["views"] = [left, dict(front, name="view_04"), front,
                        dict(right, name="right-detail"), right, dict(front, name="추가_뷰")]

        class Camera:
            def capture(self):
                transform = model.transforms(robot.joints)[scan.DEPTH_FRAME]
                points = scan.transform_points(surface, np.linalg.inv(transform))
                return scan.CapturedView(np.full((32, 32), 1000, dtype=np.uint16), points, {"synthetic": True})

        with tempfile.TemporaryDirectory(prefix="multi_view_actual_pose_") as directory:
            output = Path(directory)
            config_path = output / "input.yaml"
            config_path.write_text(yaml.safe_dump(cfg, allow_unicode=True), encoding="utf-8")
            cfg = scan.load_config(config_path)
            with patch.object(worker, "time", Clock()):
                scan.capture_sequence(cfg, model, robot, Camera(), output)
            manifest = json.loads((output / "manifest.json").read_text())
            self.assertEqual(manifest["status"], "complete")
            self.assertEqual(manifest["raw_point_count"], 6 * len(surface))
            self.assertEqual(len(robot.commands), 6)
            self.assertEqual([entry["name"] for entry in manifest["views"]],
                             ["left", "view_04", "front", "right-detail", "right", "추가_뷰"])
            self.assertEqual(len({tuple(entry["view_color_rgb"]) for entry in manifest["views"]}), 6)
            self.assertEqual([command[0] for command in robot.commands], [view["joint_deg"] for view in cfg["views"]])
            for entry, view in zip(manifest["views"], cfg["views"]):
                folder = output / view["name"]
                pose = json.loads((folder / "pose.json").read_text())
                self.assertTrue(pose["accepted"])
                np.testing.assert_allclose(pose["target_error_deg"], offset, atol=1e-12)
                self.assertGreater(entry["max_target_error_deg"], cfg["robot"]["target_tolerance_deg"])
                cloud = o3d.io.read_point_cloud(str(folder / "base.ply"))
                np.testing.assert_allclose(np.asarray(cloud.points), surface, atol=1e-9)
                np.testing.assert_allclose(np.asarray(cloud.colors),
                                           np.tile(entry["view_color_rgb"], (len(surface), 1)), atol=1 / 255)
                if view["name"] in scan.VIEW_COLORS:
                    self.assertEqual(entry["view_color_rgb"], scan.VIEW_COLORS[view["name"]])
                # The commanded pose would place the surface incorrectly.
                target_tf = model.transforms(view["joint_deg"])[scan.DEPTH_FRAME]
                self.assertFalse(np.allclose(pose["T_base_depth"], target_tf))
            merged = o3d.io.read_point_cloud(str(output / "merged.ply"))
            self.assertFalse(merged.is_empty())

    def test_motion_during_capture_is_still_rejected(self):
        settings = fixture_config()["robot"]
        before = state()
        for after in (state(running=True), state(velocity=0.1), state(joints=[0.1] * 6)):
            with self.subTest(after=after):
                with self.assertRaises(RuntimeError):
                    scan.validate_capture(copy.deepcopy(before), after, settings)


class ConfigTests(unittest.TestCase):
    def test_any_positive_view_count_and_order_are_preserved(self):
        for names in (["left", "front", "right"], ["front", "left"], ["single"],
                      ["view_05", "front", "추가_뷰", "left_high", "right-detail", "view_06"]):
            with self.subTest(names=names), tempfile.TemporaryDirectory() as directory:
                cfg = fixture_config()
                cfg["views"] = [{"name": name, "joint_deg": [0.0] * 6} for name in names]
                path = Path(directory) / "config.yaml"
                path.write_text(yaml.safe_dump(cfg))
                loaded = scan.load_config(path)
                self.assertEqual([view["name"] for view in loaded["views"]], names)

    def test_invalid_view_definitions_are_rejected_before_capture(self):
        valid = {"name": "view_01", "joint_deg": [0.0] * 6}
        invalid = [[], None, {}, [None], ["front"], [valid, valid],
                   [{"joint_deg": [0.0] * 6}], [{"name": "missing_joints"}]]
        for name in (None, 123, "", "..", "../outside", "/tmp/view", "nested/view", "nested\\view",
                     "has space", "manifest.json", "x" * 256):
            invalid.append([dict(valid, name=name)])
        for joints in ([0.0] * 5, [False] * 6, [float("nan")] * 6, "0, 0, 0, 0, 0, 0"):
            invalid.append([dict(valid, joint_deg=joints)])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.yaml"
            for views in invalid:
                with self.subTest(views=views):
                    cfg = fixture_config()
                    cfg["views"] = views
                    path.write_text(yaml.safe_dump(cfg), encoding="utf-8")
                    with self.assertRaisesRegex(ValueError, "views"):
                        scan.load_config(path)

    def test_unfilled_additional_pose_blocks_execution_preflight(self):
        cfg = fixture_config()
        cfg["views"].append({"name": "view_04", "joint_deg": None})
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.yaml"
            path.write_text(yaml.safe_dump(cfg), encoding="utf-8")
            cfg = scan.load_config(path)
            for key in ("control_python", "control_wrapper", "control_library"):
                cfg[key] = str(path)
            model = scan.ScanModel(cfg["xacro"])
            with patch.object(scan.subprocess, "run") as launch:
                with self.assertRaisesRegex(ValueError, "view_04"):
                    scan.preflight(cfg, model, execute=True)
                launch.assert_not_called()


class SDKColorTests(unittest.TestCase):
    """실제 SDK의 메모리 프레임만 사용한다. Context/Pipeline 장치를 만들지 않는다."""
    @staticmethod
    def memory_frames(color_format=None, color_size=8):
        import pyorbbecsdk as sdk

        frames, profiles = [], []
        for kind, fmt, size in ((sdk.OBFrameType.DEPTH_FRAME, sdk.OBFormat.Y16, 4),
                                (sdk.OBFrameType.COLOR_FRAME, color_format or sdk.OBFormat.RGB, color_size)):
            frame = sdk.Frame.create_video_frame(kind, fmt, size, size)
            profile = frame.get_stream_profile().as_video_stream_profile()
            intrinsic = sdk.OBCameraIntrinsic()
            for key, value in dict(width=size, height=size, fx=25.0 * size, fy=25.0 * size,
                                   cx=size / 4, cy=size / 4).items():
                setattr(intrinsic, key, value)
            profile.set_intrinsic(intrinsic)
            distortion = sdk.OBCameraDistortion()
            for key in ("k1", "k2", "k3", "k4", "k5", "k6", "p1", "p2"):
                setattr(distortion, key, 0.0)
            distortion.model = sdk.OBCameraDistortionModel.BROWN_CONRADY_K6
            profile.set_distortion(distortion)
            frames.append(frame)
            profiles.append(profile)
        depth = frames[0].as_depth_frame()
        depth.set_value_scale(0.5)
        depth.update_data(np.full((4, 4), 2000, np.uint16).tobytes())
        extrinsic = sdk.OBExtrinsic()
        extrinsic.rot = np.eye(3, dtype=np.float32)
        extrinsic.transform = np.array([5, 0, 0], np.float32)
        profiles[0].bind_extrinsic_to(profiles[1], extrinsic)
        return sdk, depth, frames[1].as_color_frame(), profiles

    def test_sdk_color_alignment_uses_calibrated_offset_and_keeps_native_depth(self):
        sdk, depth, color, _ = self.memory_frames()
        rgb = np.zeros((8, 8, 3), np.uint8)
        rgb[:, :, 0] = np.arange(8) * 20
        rgb[:, :, 1] = np.arange(8)[:, None] * 20
        rgb[:, :, 2] = 100
        color.update_data(rgb.tobytes())
        original_depth = depth.get_data().copy()
        source, aligned = scan.align_color_to_depth(sdk, depth, color)
        # 5 mm baseline at 1 m = +1 color pixel. Resolutions differ by 2x.
        np.testing.assert_array_equal(source, rgb)
        np.testing.assert_array_equal(aligned, rgb[::2, 1::2])
        np.testing.assert_array_equal(depth.get_data(), original_depth)

    def test_mjpg_decode_retains_rgb_channels_and_alignment_calibration(self):
        import cv2

        import pyorbbecsdk as sdk
        sdk, depth, color, _ = self.memory_frames(sdk.OBFormat.MJPG, 32)
        rgb = np.full((32, 32, 3), [230, 80, 20], np.uint8)
        ok, encoded = cv2.imencode(".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        self.assertTrue(ok)
        payload = np.zeros(color.get_data_size(), np.uint8)
        self.assertGreaterEqual(payload.size, encoded.size)
        payload[:encoded.size] = encoded.ravel()
        color.update_data(payload.tobytes())
        source, aligned = scan.align_color_to_depth(sdk, depth, color)
        np.testing.assert_allclose(source.mean(axis=(0, 1)), [230, 80, 20], atol=3)
        np.testing.assert_allclose(aligned.mean(axis=(0, 1)), [230, 80, 20], atol=3)

    def test_depth_camera_capture_keeps_point_units_with_rgb_enabled(self):
        sdk, depth, color, profiles = self.memory_frames()
        color.update_data(np.full((8, 8, 3), [220, 60, 15], np.uint8).tobytes())
        frames = sdk.Frame.create_frame_set()
        frames.push_frame(depth)
        frames.push_frame(color)

        class Pipeline:
            running = False

            def start(self, config):
                self.running = True

            def stop(self):
                self.running = False

            def wait_for_frames(self, timeout):
                return frames

        camera = scan.DepthCamera.__new__(scan.DepthCamera)
        camera.sdk = sdk
        camera.settings = dict(fixture_config()["camera"], capture_color=True, warmup_frames=0,
                               color_max_time_delta_ms=50)
        camera.profile, camera.color_profile = profiles
        camera.pipeline, camera.config = Pipeline(), None
        camera.device_info = {"name": "SYNTHETIC"}
        result = camera.capture()
        self.assertFalse(camera.pipeline.running)
        np.testing.assert_allclose(result.xyz[:, 2], 1.0)
        np.testing.assert_array_equal(result.color_aligned_rgb[0, 0], [220, 60, 15])
        self.assertEqual(result.metadata["frame"], scan.DEPTH_FRAME)
        self.assertEqual(result.metadata["alignment"], "color_to_native_depth")
        self.assertEqual(result.metadata["color"]["depth_to_color"]["translation_mm"], [5.0, 0.0, 0.0])
        for missing_color in (True, False):
            bad_frames = sdk.Frame.create_frame_set()
            bad_frames.push_frame(depth)
            if not missing_color:
                sdk.Frame.set_frame_device_timestamp_us(color, 100_000)
                bad_frames.push_frame(color)
            clock = Clock()

            def wait_for_bad_frames(timeout):
                clock.sleep(timeout / 1000)
                return bad_frames

            with self.subTest(missing_color=missing_color), patch.object(scan, "time", clock), \
                    patch.object(camera.pipeline, "wait_for_frames", wait_for_bad_frames):
                with self.assertRaisesRegex(TimeoutError, "Depth/RGB"):
                    camera.capture()
                self.assertFalse(camera.pipeline.running)


if __name__ == "__main__":
    unittest.main()
