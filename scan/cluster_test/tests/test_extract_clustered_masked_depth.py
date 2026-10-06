"""SAM 점군 전용 입력과 기존 코드/입력 보존 검사. 하드웨어를 사용하지 않음."""
from contextlib import redirect_stdout
import copy
import io
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import open3d as o3d

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import extract_clustered_masked_depth as app
from test_extract_clustered_part import cloud, pose


class MaskedDepthTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / "masked"
        self.root.mkdir()
        grid = np.array([[x, y, 2] for x in np.arange(-3, 4) * .004 for y in np.arange(-3, 4) * .004])
        self.points = np.vstack([grid, grid + [1, 0, 0], [5, 5, 5]])
        o3d.io.write_point_cloud(str(self.root / "merged.ply"), cloud(self.points))
        o3d.io.write_point_cloud(str(self.root / "merged_raw.ply"), cloud(np.vstack([self.points, grid])))
        self.manifest = {"status": "complete", "reference_frame": "base_link", "point_cloud_unit": "m",
                         "method": "saved_T_base_depth_no_ICP", "merged_points": 99,
                         "merged_raw_points": 148, "views": []}
        for i, origin in enumerate(([-1, 0, 0], [1, 0, 0]), 1):
            directory = self.root / f"{i:02d}_view_{i:02d}"
            directory.mkdir()
            self.manifest["views"].append({"output": str(directory), "source_view": "/unavailable/original"})
            t = pose(np.array(origin, float), [0, 0, 2])
            app.cluster.write_json(directory / "pose.json", {
                "accepted": True, "translation_unit": "m", "T_base_depth": t.tolist()})
        app.cluster.write_json(self.root / "manifest.json", self.manifest)
        self.output = Path(self.temp.name) / "outputs"
        settings = patch.multiple(app, VOXEL_SIZE_MM=1.0, DBSCAN_EPS_MM=9.0, DBSCAN_MIN_POINTS=3,
                                  MIN_CLUSTER_VOXELS=10, SELECT_CLUSTER_IDS=None,
                                  TARGET_POINT_BASE_MM=None, POINT_CLOUD_FILENAME="merged.ply")
        settings.start()
        self.addCleanup(settings.stop)

    def run_app(self, **kwargs):
        with redirect_stdout(io.StringIO()):
            return app.run(self.root, output_root=self.output, **kwargs)

    def test_end_to_end_preserves_input_and_existing_module(self):
        before = {p: app.cluster.sha256(p) for p in self.root.rglob("*") if p.is_file()}
        old_globals = dict(vars(app.cluster))
        output = self.run_app()
        report = app.cluster.read_json(output / "summary.json")
        self.assertEqual(report["status"], "complete")
        self.assertEqual(report["selected_points"], 49)
        self.assertEqual(report["sources"]["format"], "sam_masked_depth")
        labels = np.load(output / "cluster_labels.npy", allow_pickle=False)
        selected = np.isin(labels, report["selected_cluster_ids"])
        result = o3d.io.read_point_cloud(str(output / "part.ply"))
        source = o3d.io.read_point_cloud(str(self.root / "merged.ply"))
        np.testing.assert_array_equal(np.asarray(result.points), np.asarray(source.points)[selected])
        np.testing.assert_array_equal(np.asarray(result.colors), np.asarray(source.colors)[selected])
        self.assertEqual(before, {p: app.cluster.sha256(p) for p in before})
        for key, value in old_globals.items():
            self.assertIs(vars(app.cluster)[key], value)
        for filename in ("clusters_colored.ply", "selection_preview.ply"):
            self.assertTrue((output / filename).is_file())

    def test_raw_cloud_uses_raw_count(self):
        with patch.object(app, "POINT_CLOUD_FILENAME", "merged_raw.ply"):
            result, _, _, sources = app.load_masked_capture(self.root, o3d)
            self.assertEqual(len(result.points), 148)
            self.assertEqual(Path(sources["cloud"]["path"]).name, "merged_raw.ply")
            self.manifest["merged_raw_points"] = 99
            app.cluster.write_json(self.root / "manifest.json", self.manifest)
            with self.assertRaises(ValueError):
                app.load_masked_capture(self.root, o3d)

    def test_relocated_folder_uses_local_poses(self):
        moved = self.root.with_name("moved")
        self.root.rename(moved)
        _, origins, _, sources = app.load_masked_capture(moved, o3d)
        self.assertEqual(len(origins), 2)
        self.assertTrue(all(Path(source["path"]).parent.parent == moved for source in sources["poses"]))

    def test_invalid_manifest(self):
        for key, value in [("status", "failed"), ("reference_frame", "camera"),
                           ("point_cloud_unit", "mm"), ("method", "unknown"), ("icp_applied", True),
                           ("merged_points", 1), ("merged_points", None), ("views", [])]:
            with self.subTest(key=key, value=value):
                changed = copy.deepcopy(self.manifest)
                changed[key] = value
                app.cluster.write_json(self.root / "manifest.json", changed)
                with self.assertRaises(ValueError):
                    app.load_masked_capture(self.root, o3d)

    def test_invalid_or_missing_pose(self):
        path = self.root / "01_view_01/pose.json"
        original = app.cluster.read_json(path)
        for key, value in [("accepted", False), ("translation_unit", "mm"), ("T_base_depth", [[1]])]:
            changed = dict(original, **{key: value})
            app.cluster.write_json(path, changed)
            with self.assertRaises(ValueError):
                app.load_masked_capture(self.root, o3d)
        path.unlink()
        with self.assertRaises(FileNotFoundError):
            app.load_masked_capture(self.root, o3d)

    def test_duplicate_and_escaping_view(self):
        self.manifest["views"].append(copy.deepcopy(self.manifest["views"][0]))
        app.cluster.write_json(self.root / "manifest.json", self.manifest)
        with self.assertRaises(ValueError):
            app.load_masked_capture(self.root, o3d)
        self.manifest["views"] = [{"output": "../.."}]
        app.cluster.write_json(self.root / "manifest.json", self.manifest)
        with self.assertRaises(ValueError):
            app.load_masked_capture(self.root, o3d)

    def test_check_does_not_save(self):
        self.assertIsNone(self.run_app(check_only=True))
        self.assertFalse(self.output.exists())

    def test_parallel_rays_save_diagnostics_only(self):
        for path in self.root.glob("*/pose.json"):
            data = app.cluster.read_json(path)
            t = np.array(data["T_base_depth"])
            t[:3, :3] = np.eye(3)
            data["T_base_depth"] = t.tolist()
            app.cluster.write_json(path, data)
        output = self.run_app()
        self.assertEqual(app.cluster.read_json(output / "summary.json")["status"], "needs_selection")
        self.assertFalse((output / "part.ply").exists())

    def test_manual_multiple_clusters_and_viewer(self):
        output = self.run_app(manual_ids=[0, 1])
        report = app.cluster.read_json(output / "summary.json")
        self.assertEqual(report["selected_points"], 98)
        self.assertEqual(report["selection_method"], "manual_ids")
        with patch.object(o3d.visualization, "draw_geometries") as draw:
            app.cluster.view_result(output)
            self.assertEqual(len(draw.call_args.args[0]), 3)


if __name__ == "__main__":
    unittest.main()
