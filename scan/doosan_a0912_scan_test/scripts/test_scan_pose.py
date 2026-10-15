"""TF 선택/IK/저장 회귀 검사. 실제 로봇·카메라 객체를 만들지 않는다."""
import copy
from contextlib import redirect_stdout
import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import numpy as np

import multi_view_scan as multi
import multi_view_scan_single_frame as single
from cartesian_scan import DoosanSDKIK, pose_matrix, sdk_pose_matrix, validate_transform
from robot_scan_worker import make_ik
from scan_pose import DEPTH_FRAME, SDK_POSE_SOURCE, ScanPoseSource, configure_pose_source


class ScanPoseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.base_cfg = multi.load_config(multi.DEFAULT_CONFIG)
        cls.model = multi.ScanModel(cls.base_cfg["xacro"])
        cls.hand_eye_tf = pose_matrix([35, -22, 65, 12, -8, 30])

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="scan_pose_test_")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.report_file = self.root / "calibration.json"
        self.report = {
            "status": "computed_requires_physical_validation", "quality_checks_passed": True,
            "translation_unit": "m", "robot_pose_source": SDK_POSE_SOURCE,
            "method": "TSAI", "T_flange_depth": self.hand_eye_tf.tolist(),
        }
        self.write_report(self.report)

    def write_report(self, report):
        self.report_file.write_text(json.dumps(report), encoding="utf-8")

    def config(self, source="urdf", module=multi):
        cfg = module.load_config(module.DEFAULT_CONFIG)
        cfg["pose_estimation"] = {
            "source": source, "hand_eye_calibration_file": str(self.report_file)}
        cfg["views"] = [{"name": "view_01", "joint_deg": [0.0] * 6},
                        {"name": "view_02", "joint_deg": [0.0] * 6}]
        cfg["camera"].update(capture_color=True, min_depth_mm=100, max_depth_mm=2000, min_points=1)
        cfg["reconstruction"].update(roi_min_mm=None, roi_max_mm=None)
        return cfg

    @staticmethod
    def state(index=0):
        return {"joint_deg": [float(index * 5), 0.0, 0.0, 0.0, 0.0, 0.0],
                "joint_velocity_deg_s": [0.0] * 6, "motion_running": False,
                "flange_pose_mm_zyz_deg": [500.0 + index * 20, 100.0, 600.0, 20.0, 30.0, 40.0]}

    def test_config_defaults_relative_paths_and_invalid_modes(self):
        cfg = {}
        configure_pose_source(cfg, self.root / "settings.yaml")
        self.assertEqual(cfg["pose_estimation"]["source"], "urdf")
        cfg["pose_estimation"].update(source="hand_eye", hand_eye_calibration_file="calibration.json")
        configure_pose_source(cfg, self.root / "settings.yaml")
        self.assertEqual(cfg["pose_estimation"]["hand_eye_calibration_file"], str(self.report_file))
        for value in (None, "wrong", {"source": "automatic"}, {"source": "urdf", "hand_eye_calibration_file": ""}):
            with self.subTest(value=value), self.assertRaises(ValueError):
                configure_pose_source({"pose_estimation": value}, self.root / "settings.yaml")

    def test_report_validation_and_no_fallback(self):
        for changes in ({"status": "failed"}, {"quality_checks_passed": False},
                        {"translation_unit": "mm"}, {"robot_pose_source": "URDF FK"},
                        {"T_flange_depth": None}, {"T_flange_depth": np.zeros((4, 4)).tolist()}):
            with self.subTest(changes=changes):
                self.write_report({**self.report, **changes})
                with self.assertRaises(ValueError):
                    ScanPoseSource(self.config("hand_eye"), self.model)
        for invalid in ({k: v for k, v in self.report.items() if k != "T_flange_depth"}, []):
            self.write_report(invalid)
            with self.assertRaises(ValueError):
                ScanPoseSource(self.config("hand_eye"), self.model)
        cfg = self.config("hand_eye")
        cfg["pose_estimation"]["hand_eye_calibration_file"] = None
        with self.assertRaises(ValueError):
            ScanPoseSource(cfg, self.model)
        self.report_file.unlink()
        with self.assertRaises(FileNotFoundError):
            ScanPoseSource(self.config("hand_eye"), self.model)
        # URDF 모드는 사용하지 않는 보정 파일이 없어도 기존 동작 유지.
        self.assertEqual(ScanPoseSource(self.config(), self.model).source, "urdf")

    def test_reject_nonrigid_nonfinite_transforms(self):
        for broken in (np.diag([-1, 1, 1, 1]), np.diag([2, 1, 1, 1]),
                       np.full((4, 4), np.nan), np.eye(3), np.diag([1, 1, 1, 2])):
            with self.subTest(matrix=broken), self.assertRaises(ValueError):
                validate_transform(broken)

    def test_source_composition_and_snapshot_command(self):
        state = self.state(1)
        robot = Mock()
        robot.snapshot.return_value = state
        robot.call.return_value = state
        for source in ("urdf", "hand_eye"):
            provider = ScanPoseSource(self.config(source), self.model)
            self.assertEqual(provider.snapshot(robot), state)
            actual = provider.frames(state)
            if source == "urdf":
                expected = self.model.transforms(state["joint_deg"])
            else:
                flange = sdk_pose_matrix(state["flange_pose_mm_zyz_deg"])
                expected = {"link_6": flange, "bracket_link": flange @ provider.flange_bracket,
                            DEPTH_FRAME: flange @ self.hand_eye_tf}
            for frame, matrix in expected.items():
                np.testing.assert_allclose(actual[frame], matrix, atol=1e-12)
        robot.snapshot.assert_called_once_with()
        robot.call.assert_called_once_with("snapshot_cartesian")
        robot.call.return_value = {"joint_deg": state["joint_deg"]}
        with self.assertRaises(KeyError):
            provider.snapshot(robot)
        robot.snapshot.assert_called_once_with()  # 실패해도 URDF snapshot으로 대체하지 않음.

    def test_ik_override_only_for_depth_target(self):
        for source in ("urdf", "hand_eye"):
            for frame in ("link_6", "bracket_link", DEPTH_FRAME):
                with self.subTest(source=source, frame=frame):
                    cfg = self.config(source)
                    cfg["views"] = [{"name": "target", "pose_mm_deg": [100, 200, 500, 10, 20, 30]}]
                    cfg["cartesian"]["target_frame"] = frame
                    request = multi.ik_request(cfg, self.model)
                    expected_override = source == "hand_eye" and frame == DEPTH_FRAME
                    self.assertEqual("T_flange_target" in request, expected_override)
                    ik = make_ik(request)
                    self.assertEqual(ik.tool_tf_source, "hand_eye" if expected_override else "urdf")
                    nominal = DoosanSDKIK(self.model.xml, cfg["cartesian"])
                    expected_tool = self.hand_eye_tf if expected_override else nominal.flange_target
                    np.testing.assert_allclose(ik.flange_target, expected_tool)
                    robot = Mock()
                    flange = [100.0, -200.0, 400.0, 30.0, 45.0, 20.0]
                    tcp = [120.0, -180.0, 460.0, 40.0, 50.0, 25.0]
                    robot.read_actual_cartesian_state.return_value = {
                        "flange_pose_mm_zyz_deg": flange, "tcp_pose_mm_zyz_deg": tcp}
                    robot.solve_closest_ik.return_value = {"target_joint_deg": [0.0] * 6, "solution_space": 0}
                    desired = cfg["views"][0]["pose_mm_deg"]
                    result = ik.solve(robot, desired, [0.0] * 6)
                    expected = (pose_matrix(desired) @ np.linalg.inv(expected_tool)
                                @ np.linalg.inv(sdk_pose_matrix(flange)) @ sdk_pose_matrix(tcp))
                    np.testing.assert_allclose(
                        sdk_pose_matrix(result["ik"]["sdk_target_pose_mm_zyz_deg"]), expected, atol=1e-10)
        # 캘리브레이션 자신은 link_6 목표: 기존 scan YAML이 hand_eye여도 보정 파일 불필요.
        cfg["cartesian"]["target_frame"] = "link_6"
        self.report_file.unlink()
        self.assertNotIn("T_flange_target", multi.ik_request(cfg, self.model))
        self.assertEqual(multi.ik_request(self.config("hand_eye"), self.model), {})

    def run_capture(self, module, source, fail_after_capture=False):
        cfg = self.config(source, module)
        provider = ScanPoseSource(cfg, self.model)
        output = self.root / f"{module.__name__}_{source}"
        output.mkdir()
        robot = Mock()
        states = [self.state(0), self.state(1)]
        robot.move.side_effect = [
            {**state, "target_joint_deg": [0.0] * 6, "max_target_error_deg": float(i * 5)}
            for i, state in enumerate(states)]
        snapshots = [states[0], states[0], states[1], states[1]]
        if fail_after_capture:
            snapshots[1] = RuntimeError("synthetic SDK snapshot failure")
        if source == "urdf":
            robot.snapshot.side_effect = snapshots
        else:
            robot.call.side_effect = snapshots
        # 32점: ICP 입력 로더의 최소 점 개수도 만족. RGB/Depth 원본 함께 보존 검사.
        xyz = np.column_stack((np.arange(32) * .01, np.zeros(32), np.full(32, 1.0)))
        raw = np.full((4, 8), 1000, dtype=np.uint16)
        rgb = np.full((4, 8, 3), [60, 120, 180], dtype=np.uint8)
        captured = module.CapturedView(raw, xyz, {"depth_scale_mm_per_unit": 1.0},
                                       color_rgb=rgb, color_aligned_rgb=rgb)
        if module is multi:
            captured.depth_frames = np.stack([raw] * 3)
            captured.depth_valid_counts = np.full(raw.shape, 3, dtype=np.uint32)
        camera = Mock()
        camera.capture.side_effect = lambda: copy.deepcopy(captured)
        # 읽은 결과를 고정: 실행 중 원본 파일이 바뀌어도 사용 TF/보존 원본이 변하면 안 됨.
        original = self.report_file.read_bytes()
        self.write_report({"status": "modified_after_loading"})
        with redirect_stdout(io.StringIO()):
            if fail_after_capture:
                with self.assertRaisesRegex(RuntimeError, "synthetic SDK"):
                    module.capture_sequence(cfg, self.model, robot, camera, output, provider)
            else:
                module.capture_sequence(cfg, self.model, robot, camera, output, provider)
        return output, provider, states, xyz, raw, original

    def test_both_capture_versions_both_sources(self):
        import open3d as o3d

        for module in (multi, single):
            for source in ("urdf", "hand_eye"):
                with self.subTest(module=module.__name__, source=source):
                    self.write_report(self.report)
                    output, provider, states, xyz, raw, original = self.run_capture(module, source)
                    manifest = json.loads((output / "manifest.json").read_text())
                    self.assertEqual(manifest["status"], "complete")
                    self.assertEqual(manifest["method"], provider.method)
                    self.assertEqual(manifest["pose_estimation"]["source"], source)
                    self.assertFalse(manifest["icp_applied"])
                    self.assertEqual([v["name"] for v in manifest["views"]], ["view_01", "view_02"])
                    for i, state in enumerate(states, 1):
                        folder = output / f"view_{i:02d}"
                        pose = json.loads((folder / "pose.json").read_text())
                        expected = provider.frames(state)[DEPTH_FRAME]
                        np.testing.assert_allclose(pose["T_base_depth"], expected)
                        np.testing.assert_allclose(pose["T_flange_depth"], provider.flange_depth, atol=1e-12)
                        np.testing.assert_array_equal(np.load(folder / "depth_raw.npy"), raw)
                        points = o3d.io.read_point_cloud(str(folder / "base.ply"))
                        np.testing.assert_allclose(np.asarray(points.points), multi.transform_points(xyz, expected))
                        np.testing.assert_allclose(np.asarray(points.colors), np.tile([60, 120, 180], (32, 1)) / 255)
                        self.assertEqual((folder / "depth_frames.npy").exists(), module is multi)
                        self.assertTrue((folder / "depth_visualization.png").is_file())
                        self.assertTrue(pose["accepted"])
                    self.assertEqual((output / "hand_eye_calibration.json").exists(), source == "hand_eye")
                    if source == "hand_eye":
                        self.assertEqual((output / "hand_eye_calibration.json").read_bytes(), original)
                        self.assertEqual(provider.metadata["source_sha256"], hashlib.sha256(original).hexdigest())
                    if module is single:
                        settings = json.loads((output / "settings.json").read_text())
                        self.assertEqual(settings["camera"]["capture_frames"], 1)
                        self.assertEqual(manifest["capture_mode"], "single_frame")

    def test_failed_sdk_snapshot_preserves_raw_but_never_merges(self):
        for module in (multi, single):
            with self.subTest(module=module.__name__):
                self.write_report(self.report)
                output, *_ = self.run_capture(module, "hand_eye", fail_after_capture=True)
                self.assertEqual(json.loads((output / "manifest.json").read_text())["status"], "failed")
                self.assertFalse(json.loads((output / "view_01/pose.json").read_text())["accepted"])
                self.assertTrue((output / "view_01/depth_raw.npy").is_file())
                self.assertFalse((output / "merged.ply").exists())

    def test_invalid_report_blocks_before_preflight_subprocess(self):
        self.write_report({"status": "failed"})
        for module in (multi, single):
            with self.subTest(module=module.__name__), patch.object(module.subprocess, "run") as run:
                with self.assertRaises(ValueError):
                    module.preflight(self.config("hand_eye", module), self.model)
                run.assert_not_called()


if __name__ == "__main__":
    unittest.main()
