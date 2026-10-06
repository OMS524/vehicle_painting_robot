"""스캔 환경에서 실행하는 하드웨어 없는 DBSCAN/시선/파일 보존 회귀 테스트."""
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
import extract_clustered_part as app


def cloud(points):
    result = o3d.geometry.PointCloud()
    result.points = o3d.utility.Vector3dVector(np.asarray(points, float))
    result.colors = o3d.utility.Vector3dVector(np.tile([0.2, 0.6, 0.8], (len(points), 1)))
    return result


def pose(origin, target):
    z = np.asarray(target, float) - origin
    z /= np.linalg.norm(z)
    x = np.cross([0, 1, 0], z)
    x /= np.linalg.norm(x)
    t = np.eye(4)
    t[:3, :3] = np.column_stack([x, np.cross(z, x), z])
    t[:3, 3] = origin
    return t


class GeometryTests(unittest.TestCase):
    def test_intersection(self):
        origins = np.array([[-1, 0, 0], [1, 0, 0], [0, -1, 0]], float)
        expected = np.array([0.1, 0.2, 2])
        target, info = app.estimate_target(origins, expected - origins, 1)
        np.testing.assert_allclose(target, expected, atol=1e-10)
        self.assertLess(info['rms_mm'], 1e-8)

    def test_rigid_transform_invariance(self):
        origins = np.array([[-1, 0, 0], [1, 0, 0]], float)
        target = np.array([0, 0, 2.0])
        r = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        offset = np.array([2, 3, -1])
        result, _ = app.estimate_target(origins @ r.T + offset, (target - origins) @ r.T, 1)
        np.testing.assert_allclose(result, target @ r.T + offset, atol=1e-10)

    def test_bad_rays(self):
        o = np.array([[-1, 0, 0], [1, 0, 0]], float)
        with self.assertRaises(ValueError):
            app.estimate_target(o, [[0, 0, 1], [0, 0, 1]], 100)
        with self.assertRaises(ValueError):
            app.estimate_target(o, o - [0, 0, 2], 100)
        with self.assertRaises(ValueError):
            app.estimate_target(o[:1], [[0, 0, 1]], 100)
        with self.assertRaises(ValueError):
            app.estimate_target(o, [[0, 0, 0], [0, 0, 1]], 100)

    def test_skew_rays_rejected(self):
        with self.assertRaisesRegex(ValueError, 'RMS'):
            app.estimate_target([[-1, 0, 0], [0, -1, 1]], [[1, 0, 0], [0, 1, 0]], 100)

    def test_invalid_transform(self):
        np.testing.assert_array_equal(app.validate_transform(np.eye(4)), np.eye(4))
        t = np.eye(4)
        t[0, 0] = -1
        with self.assertRaises(ValueError):
            app.validate_transform(t)

    def test_cluster_mapping_and_hole(self):
        angles = np.linspace(0, 2 * np.pi, 180, endpoint=False)
        ring = np.column_stack([.1 * np.cos(angles), .1 * np.sin(angles), np.zeros(180)])
        points = np.vstack([ring, ring + [1, 0, 0], ring, [5, 5, 5], [np.nan, 0, 0]])
        p = cloud(points)
        _, _, labels, records = app.cluster_cloud(p, o3d, voxel_mm=1, eps_mm=9, min_points=3, min_cluster_voxels=10)
        self.assertEqual(len(records), 2)
        self.assertEqual(labels[-2], -1)
        self.assertEqual(labels[-1], -2)
        np.testing.assert_array_equal(labels[:180], labels[360:540])
        self.assertNotEqual(labels[0], labels[180])
        selected = p.select_by_index(np.flatnonzero(labels == labels[0]).tolist())
        self.assertEqual(len(selected.points), 360)
        self.assertGreater(np.linalg.norm(np.asarray(selected.points), axis=1).min(), .09)

    def test_noise_only(self):
        _, _, labels, records = app.cluster_cloud(cloud([[0, 0, 0], [1, 0, 0]]), o3d,
                                                 voxel_mm=1, eps_mm=5, min_points=3, min_cluster_voxels=1)
        self.assertEqual(records, [])
        np.testing.assert_array_equal(labels, [-1, -1])

    def test_empty_cloud(self):
        with self.assertRaises(ValueError):
            app.cluster_cloud(cloud(np.empty((0, 3))), o3d, voxel_mm=1, eps_mm=5,
                              min_points=3, min_cluster_voxels=1)

    def test_selection_and_overrides(self):
        records = [{'id': 0, 'eligible': True, 'voxel_count': 100, 'center_base_mm': [0, 0, 0]},
                   {'id': 1, 'eligible': True, 'voxel_count': 500, 'center_base_mm': [1000, 0, 0]},
                   {'id': 2, 'eligible': False, 'voxel_count': 3, 'center_base_mm': [50, 0, 0]}]
        self.assertEqual(app.choose_clusters(records, np.array([.05, 0, 0]), None, 100), [0])
        self.assertEqual(app.choose_clusters(records, np.array([5, 0, 0]), None, 100), [])
        self.assertEqual(app.choose_clusters(records, None, None, 100), [])
        self.assertEqual(app.choose_clusters(records, None, [0, 2], 100), [0, 2])
        for ids in ([], [8], [-1], [True], [0, 0]):
            with self.assertRaises(ValueError):
                app.choose_clusters(records, None, ids, 100)


class FileTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name) / 'capture'
        self.root.mkdir()
        grid = np.array([[x, y, 2] for x in np.arange(-3, 4) * .004 for y in np.arange(-3, 4) * .004])
        self.points = np.vstack([grid, grid + [1, 0, 0], [5, 5, 5]])
        o3d.io.write_point_cloud(str(self.root / 'merged.ply'), cloud(self.points))
        manifest = {'status': 'complete', 'reference_frame': 'base_link', 'units': {'point_cloud': 'm'},
                    'icp_applied': False, 'point_count': len(self.points), 'views': []}
        for i, origin in enumerate(([-1, 0, 0], [1, 0, 0]), 1):
            name = f'view_{i:02d}'
            (self.root / name).mkdir()
            t = pose(np.array(origin, float), [0, 0, 2])
            app.write_json(self.root / name / 'pose.json', {'accepted': True, 'translation_unit': 'm', 'T_base_depth': t.tolist()})
            manifest['views'].append({'name': name, 'status': 'accepted'})
        app.write_json(self.root / 'manifest.json', manifest)
        self.output = Path(self.temp.name) / 'outputs'
        self.settings = patch.multiple(app, VOXEL_SIZE_MM=1.0, DBSCAN_EPS_MM=9.0,
                                       DBSCAN_MIN_POINTS=3, MIN_CLUSTER_VOXELS=10,
                                       SELECT_CLUSTER_IDS=None, TARGET_POINT_BASE_MM=None)
        self.settings.start()
        self.addCleanup(self.settings.stop)

    def run_app(self, **kwargs):
        with redirect_stdout(io.StringIO()):
            return app.run(self.root, output_root=self.output, **kwargs)

    def test_end_to_end_preserves_input_xyz_rgb(self):
        before = {p: app.sha256(p) for p in self.root.rglob('*') if p.is_file()}
        output = self.run_app()
        report = app.read_json(output / 'summary.json')
        self.assertEqual(report['status'], 'complete')
        self.assertEqual(report['selected_points'], 49)
        labels = np.load(output / 'cluster_labels.npy')
        selected = np.isin(labels, report['selected_cluster_ids'])
        result = o3d.io.read_point_cloud(str(output / 'part.ply'))
        source = o3d.io.read_point_cloud(str(self.root / 'merged.ply'))
        np.testing.assert_array_equal(np.asarray(result.points), np.asarray(source.points)[selected])
        np.testing.assert_array_equal(np.asarray(result.colors), np.asarray(source.colors)[selected])
        self.assertEqual(before, {p: app.sha256(p) for p in before})
        self.assertTrue((output / 'clusters_colored.ply').is_file())

    def test_check_does_not_create_outputs(self):
        self.assertIsNone(self.run_app(check_only=True))
        self.assertFalse(self.output.exists())

    def test_invalid_manifest_rejected(self):
        path = self.root / 'manifest.json'
        original = app.read_json(path)
        for key, value in [('status', 'running'), ('units', {'point_cloud': 'mm'}), ('icp_applied', True), ('point_count', 1)]:
            changed = copy.deepcopy(original)
            changed[key] = value
            app.write_json(path, changed)
            with self.assertRaises(ValueError):
                app.load_capture(self.root, o3d)
        app.write_json(path, original)

    def test_unreliable_target_keeps_diagnostics_only(self):
        for path in self.root.glob('view_*/pose.json'):
            data = app.read_json(path)
            t = np.array(data['T_base_depth'])
            t[:3, :3] = np.eye(3)
            data['T_base_depth'] = t.tolist()
            app.write_json(path, data)
        output = self.run_app()
        self.assertEqual(app.read_json(output / 'summary.json')['status'], 'needs_selection')
        self.assertFalse((output / 'part.ply').exists())
        self.assertTrue((output / 'clusters_colored.ply').exists())

    def test_visualization_geometry_without_window(self):
        output = self.run_app()
        with patch.object(o3d.visualization, 'draw_geometries') as draw:
            app.view_result(output)
            self.assertEqual(len(draw.call_args.args[0]), 3)
            self.assertIsInstance(draw.call_args.args[0][-1], o3d.geometry.LineSet)


if __name__ == '__main__':
    unittest.main()
