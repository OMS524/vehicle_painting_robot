"""저장 로그 기반 Depth TF 비교 검사. 로봇/카메라를 연결하지 않는다."""
from contextlib import redirect_stdout
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

import numpy as np

import hand_eye_calibration as hand_eye
from scan_pose import DEPTH_FRAME, SDK_POSE_SOURCE, ScanPoseSource


class DepthTfVariantsTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="depth_tf_variants_test_")
        self.addCleanup(self.temp.cleanup)
        self.session = Path(self.temp.name)
        self.flange_color = hand_eye.pose_matrix([30, -40, 20, 10, 80, -30])
        self.raw = hand_eye.pose_matrix([-33, -1, 2, -6, 1, 2])
        self.raw[:3, 0] *= 0.994
        self.camera = {"T_color_depth_sdk_raw": self.raw.tolist(), "T_color_depth": np.eye(4).tolist()}
        self.report = {"status": "computed_requires_physical_validation", "translation_unit": "m",
                       "source_session": "/old/moved/session", "quality_checks_passed": True,
                       "robot_pose_source": SDK_POSE_SOURCE, "method": "TSAI", "excluded": ["pose_03"],
                       "samples": [{"name": "pose_01", "T_base_flange": np.eye(4).tolist()}],
                       "board_consistency": [{"name": "pose_01", "translation_mm": 0.2}],
                       "leave_one_out": [{"omitted": "pose_01", "translation_mm": 0.01}],
                       "excitation": {"rotation_axis_ratio": 0.3},
                       "T_flange_color": self.flange_color.tolist(),
                       "warnings": ["Depth TF 미출력: 원본 검사 실패", "기존 참고 사항"],
                       "notes": ["RGB 보정 참고", "T_flange_color는 RGB 광학 프레임. Depth는 SDK Depth->RGB TF가 유효할 때만 계산합니다."]}
        self.calibration_file = self.session / "calibration_20261015_203609" / "calibration.json"
        self.write(self.calibration_file, self.report)
        self.write(self.session / "camera.json", self.camera)

    @staticmethod
    def write(path, value):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(value), encoding="utf-8")

    def test_raw_preserved_and_composition_direction(self):
        original = self.raw.copy()
        variants = hand_eye.depth_tf_variants(self.flange_color, self.raw)
        raw = variants["raw"]
        np.testing.assert_array_equal(raw["T_color_depth"], original)
        np.testing.assert_array_equal(self.raw, original)
        # 변환 순서와 단위 확인: 한 점을 Depth→RGB→flange 순서로 옮긴 값과 비교한다.
        point = np.array([0.3, -0.2, 1.0, 1.0])
        actual = np.asarray(raw["T_flange_depth"]) @ point
        np.testing.assert_allclose(actual, self.flange_color @ (original @ point), atol=1e-12)
        self.assertFalse(raw["diagnostics"]["T_color_depth"]["passes_se3_tolerance"])
        with self.assertRaises(ValueError):
            hand_eye.validate_transform(raw["T_flange_depth"])

    def test_selected_column_only_and_no_orthogonalization(self):
        self.raw[:3, 1] += 0.2 * self.raw[:3, 0]  # 길이 보정만으로 제거되지 않는 비직교 오차.
        for axis, index in zip("xyz", range(3)):
            with self.subTest(axis=axis):
                result = hand_eye.depth_tf_variants(self.flange_color, self.raw, axis)["unit_axis"]
                corrected = np.asarray(result["T_color_depth"])
                self.assertAlmostEqual(np.linalg.norm(corrected[:3, index]), 1.0, places=14)
                np.testing.assert_allclose(np.cross(corrected[:3, index], self.raw[:3, index]), 0, atol=1e-15)
                unchanged = [i for i in range(4) if i != index]
                np.testing.assert_array_equal(corrected[:, unchanged], self.raw[:, unchanged])
                self.assertFalse(result["diagnostics"]["T_color_depth"]["passes_se3_tolerance"])
                self.assertFalse(result["normalization"]["orthogonalization_applied"])

    def test_export_preserves_inputs_and_uses_raw_before_fallback(self):
        before = {p: p.read_bytes() for p in (self.calibration_file, self.session / "camera.json")}
        outputs = []
        for path in (self.session, self.calibration_file.parent, self.calibration_file):
            with redirect_stdout(io.StringIO()):
                output = hand_eye.export_depth_variants(path)
            outputs.append(output)
            self.assertEqual(output.parent, self.session)
            raw = hand_eye.read_json(output / "raw/calibration.json")
            unit = hand_eye.read_json(output / "unit_axis/calibration.json")
            np.testing.assert_array_equal(raw["T_color_depth"], self.raw)
            for result in (raw, unit):
                for key, value in self.report.items():
                    if key not in ("warnings", "notes"):
                        self.assertEqual(result[key], value)
                self.assertNotIn("schema", result)
                self.assertEqual(result["depth_tf_export"]["source_calibration_file"], str(self.calibration_file))
                self.assertEqual(result["depth_tf_export"]["source_warnings"], self.report["warnings"])
                self.assertEqual(result["depth_tf_export"]["source_notes"], self.report["notes"])
                self.assertIn("기존 참고 사항", result["warnings"])
                self.assertFalse(any(w.startswith("Depth TF 미출력:") for w in result["warnings"]))
            self.assertAlmostEqual(unit["depth_tf_export"]["diagnostics"]["T_color_depth"]["axis_lengths_xyz"][0], 1.0)
            self.assertTrue(unit["depth_tf_export"]["diagnostics"]["T_flange_depth"]["passes_se3_tolerance"])
        self.assertEqual(len(set(outputs)), 3)
        self.assertEqual(hand_eye.depth_variant_sources(outputs[0] / "raw/calibration.json")[0], self.session)
        for p, data in before.items():
            self.assertEqual(p.read_bytes(), data)

    def test_full_reports_load_through_existing_hand_eye_fields(self):
        with redirect_stdout(io.StringIO()):
            output = hand_eye.export_depth_variants(self.session)
        model = Mock()
        model.transforms.return_value = {key: np.eye(4) for key in ("link_6", "bracket_link", DEPTH_FRAME)}
        state = {"flange_pose_mm_zyz_deg": [500, 20, 600, 20, 30, -10]}
        for variant in ("raw", "unit_axis"):
            filename = output / variant / "calibration.json"
            result = hand_eye.read_json(filename)
            cfg = {"pose_estimation": {"source": "hand_eye", "hand_eye_calibration_file": str(filename)}}
            provider = ScanPoseSource(cfg, model)
            np.testing.assert_array_equal(provider.flange_depth, result["T_flange_depth"])
            np.testing.assert_allclose(provider.frames(state)[DEPTH_FRAME],
                                       hand_eye.sdk_pose_matrix(state["flange_pose_mm_zyz_deg"]) @ provider.flange_depth)
            # 추가 메타데이터 없이도 기존 calibration.json 필드만으로 읽을 수 있어야 한다.
            result.pop("depth_tf_export")
            self.write(filename, result)
            np.testing.assert_array_equal(ScanPoseSource(cfg, model).flange_depth, provider.flange_depth)
            for field, bad in (("status", "failed"), ("quality_checks_passed", False),
                               ("robot_pose_source", "URDF FK"), ("translation_unit", "mm"),
                               ("T_flange_depth", np.full((4, 4), np.nan).tolist())):
                self.write(filename, {**result, field: bad})
                with self.subTest(variant=variant, field=field), self.assertRaises(ValueError):
                    ScanPoseSource(cfg, model)

    def test_export_does_not_upgrade_failed_rgb_quality(self):
        self.write(self.calibration_file, {**self.report, "quality_checks_passed": False})
        with redirect_stdout(io.StringIO()):
            output = hand_eye.export_depth_variants(self.session)
        for variant in ("raw", "unit_axis"):
            self.assertFalse(hand_eye.read_json(output / variant / "calibration.json")["quality_checks_passed"])

    def test_session_selects_latest_completed_result(self):
        newer = self.calibration_file.parent.with_name("calibration_20261015_204000") / "calibration.json"
        self.write(newer, self.report)
        failed = self.calibration_file.parent.with_name("calibration_20261015_205000") / "calibration.json"
        self.write(failed, {**self.report, "status": "failed"})
        self.assertEqual(hand_eye.depth_variant_sources(self.session)[1], newer)

    def test_missing_extrinsic_and_invalid_matrices_fail_before_writing(self):
        self.write(self.session / "camera.json", {})
        with self.assertRaisesRegex(ValueError, "행렬이 없습니다"):
            hand_eye.export_depth_variants(self.session)
        self.assertFalse(list(self.session.glob("depth_tf_variants_*")))
        for invalid in (np.eye(3), np.full((4, 4), np.nan), np.diag([1, 1, 1, 2]), np.diag([0, 1, 1, 1])):
            with self.subTest(value=invalid), self.assertRaises(ValueError):
                hand_eye.depth_tf_variants(self.flange_color, invalid)

    def test_cli_is_offline_and_does_not_load_current_config_or_resolve_rgb_again(self):
        args = ["hand_eye_calibration.py", "--export-depth-variants", str(self.session), "--normalize-axis", "x"]
        with patch("sys.argv", args), redirect_stdout(io.StringIO()):
            with patch.object(hand_eye, "load_settings", side_effect=AssertionError("config read")), \
                 patch.object(hand_eye, "solve_session", side_effect=AssertionError("RGB re-solve")), \
                 patch.object(hand_eye, "ColorCamera", side_effect=AssertionError("camera connection")), \
                 patch.object(hand_eye.scan, "RobotClient", side_effect=AssertionError("robot connection")):
                self.assertEqual(hand_eye.main(), 0)
        self.assertEqual(len(list(self.session.glob("depth_tf_variants_*/*/calibration.json"))), 2)


if __name__ == "__main__":
    unittest.main()
