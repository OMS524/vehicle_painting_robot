"""SDK scan IK regression tests. No robot/camera connection or motion."""
import ctypes
import importlib.util
from pathlib import Path
import unittest
from unittest.mock import Mock, patch

import numpy as np
from scipy.spatial.transform import Rotation

import cartesian_scan as cs
import robot_scan_worker as worker
import three_view_scan as multi
import three_view_scan_single_frame as single

HERE = Path(__file__).resolve().parent


def transform(xyz_mm, rotation):
    result = np.eye(4)
    result[:3, :3] = rotation.as_matrix()
    result[:3, 3] = np.asarray(xyz_mm) / 1000
    return result


def measured_robot(flange, tcp):
    robot = Mock()
    robot.read_actual_cartesian_state.return_value = {
        'flange_pose_mm_zyz_deg': flange,
        'tcp_pose_mm_zyz_deg': tcp,
    }
    robot.solve_closest_ik.return_value = {'target_joint_deg': [10, 20, 30, 40, 50, 60],
                                         'solution_space': 3}
    return robot


class SdkPoseTests(unittest.TestCase):
    def test_sdk_zyz_rotation_and_mm_against_scipy(self):
        for angles in ([23, 57, -82], [-30, 0, 17], [43, 180, -71], [90, 90, -90]):
            with self.subTest(angles=angles):
                expected = transform([210, -350, 680], Rotation.from_euler('ZYZ', angles, degrees=True))
                np.testing.assert_allclose(cs.sdk_pose_matrix([210, -350, 680, *angles]), expected, atol=1e-12)

    def test_rpy_to_sdk_round_trip_including_both_zyz_singularities(self):
        rng = np.random.default_rng(34)
        cases = [[0, 0, 0], [0, 0, 90], [180, 0, 37], [0, 180, -41],
                 [90, 0, 0], [0, 90, 0], [180 - 1e-7, 0, 29]]
        cases.extend(rng.uniform(-180, 180, (80, 3)).tolist())
        for angles in cases:
            expected = transform([100, -200, 300], Rotation.from_euler('xyz', angles, degrees=True))
            sdk = cs.matrix_to_sdk_pose(expected)
            actual = transform(sdk[:3], Rotation.from_euler('ZYZ', sdk[3:], degrees=True))
            np.testing.assert_allclose(actual, expected, atol=1e-9)


class AdapterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = multi.load_config(multi.DEFAULT_CONFIG)
        cls.model = multi.ScanModel(cls.cfg['xacro'])

    def test_nonzero_active_tcp_and_each_target_frame(self):
        flange_pose = [250, -125, 710, 32, 81, -63]
        tcp_pose = [310, -48, 735, -12, 46, 72]
        current_flange = transform(flange_pose[:3], Rotation.from_euler('ZYZ', flange_pose[3:], degrees=True))
        current_tcp = transform(tcp_pose[:3], Rotation.from_euler('ZYZ', tcp_pose[3:], degrees=True))
        flange_tcp = np.linalg.inv(current_flange) @ current_tcp
        requested = [410, 270, 680, -47, 23, 105]
        desired = transform(requested[:3], Rotation.from_euler('xyz', requested[3:], degrees=True))
        frames = self.model.transforms([12, -20, 75, 28, 65, -35])
        for name in cs.TARGET_FRAMES:
            with self.subTest(frame=name):
                robot = measured_robot(flange_pose, tcp_pose)
                ik = cs.DoosanSDKIK(self.model.xml, {'target_frame': name})
                result = ik.solve(robot, requested, [1, 2, 3, 4, 5, 6])
                sdk_pose, reference = robot.solve_closest_ik.call_args.args
                # Check the SDK target yields the original requested link pose,
                # accounting for the real controller TCP and the Xacro tool chain.
                sdk_target = transform(sdk_pose[:3], Rotation.from_euler('ZYZ', sdk_pose[3:], degrees=True))
                flange_target = np.linalg.inv(frames['link_6']) @ frames[name]
                recovered = sdk_target @ np.linalg.inv(flange_tcp) @ flange_target
                np.testing.assert_allclose(recovered, desired, atol=1e-10)
                self.assertEqual(reference, [1, 2, 3, 4, 5, 6])
                self.assertEqual(result['target_joint_deg'], [10, 20, 30, 40, 50, 60])
                self.assertEqual(result['ik']['method'], 'doosan_sdk_ikin')
                self.assertEqual(result['ik']['solution_space'], 3)
                self.assertNotIn('iterations', result['ik'])

    def test_zero_tcp_flange_target_and_sdk_failure_propagation(self):
        robot = measured_robot([0]*6, [0]*6)
        ik = cs.DoosanSDKIK(self.model.xml, {'target_frame': 'link_6'})
        ik.solve(robot, [0, 500, 500, 0, 0, 90], [0]*6)
        np.testing.assert_allclose(robot.solve_closest_ik.call_args.args[0], [0, 500, 500, 90, 0, 0])
        robot.solve_closest_ik.side_effect = RuntimeError('SDK IK failed')
        with self.assertRaisesRegex(RuntimeError, 'SDK IK failed'):
            ik.solve(robot, [0]*6, [0]*6)
        robot.velocity_control_to_position.assert_not_called()

    def test_missing_tcp_data_is_not_replaced_with_zero(self):
        robot = measured_robot([0]*6, [0]*6)
        robot.read_actual_cartesian_state.side_effect = RuntimeError('unavailable')
        ik = cs.DoosanSDKIK(self.model.xml, {'target_frame': 'link_6'})
        with self.assertRaisesRegex(RuntimeError, 'unavailable'):
            ik.solve(robot, [0]*6, [0]*6)
        robot.solve_closest_ik.assert_not_called()

    def test_both_scan_modes_select_same_backend_and_single_stays_single(self):
        with patch.dict('sys.modules', {'pinocchio': None}):
            for module in (multi, single):
                cfg = module.load_config(module.DEFAULT_CONFIG)
                cfg['views'] = [{'name': 'test_view', 'pose_mm_deg': [0, 500, 500, 0, 0, 0]}]
                ik = worker.make_ik(module.ik_request(cfg, self.model))
                self.assertIsInstance(ik, cs.DoosanSDKIK)
                if module is single:
                    self.assertEqual(cfg['camera']['capture_frames'], 1)
                    self.assertEqual(cfg['camera']['min_valid_frames'], 1)
                cfg['views'] = [{'name': 'test_view', 'joint_deg': [0]*6}]
                self.assertEqual(module.ik_request(cfg, self.model), {})

    def test_worker_routes_sdk_result_to_existing_motion(self):
        settings = self.cfg['robot']
        state = {'joint_deg': [1, 2, 3, 4, 5, 6], 'joint_velocity_deg_s': [0]*6, 'motion_running': False}
        robot, ik = Mock(), Mock()
        robot.read_actual_joint_state.return_value = state
        ik.solve.return_value = {'target_joint_deg': [10, 20, 30, 40, 50, 60], 'ik': {'method': 'doosan_sdk_ikin'}}
        with patch.object(worker, 'move_and_settle', return_value={}) as move:
            worker.move_cartesian_and_settle(robot, [0]*6, settings, ik)
            ik.solve.assert_called_once_with(robot, [0]*6, state['joint_deg'])
            move.assert_called_once_with(robot, [10, 20, 30, 40, 50, 60], settings)
        ik.solve.side_effect = RuntimeError('SDK IK failed')
        with patch.object(worker, 'move_and_settle') as move:
            with self.assertRaisesRegex(RuntimeError, 'SDK IK failed'):
                worker.move_cartesian_and_settle(robot, [0]*6, settings, ik)
            move.assert_not_called()


class CtypesTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cfg = multi.load_config(multi.DEFAULT_CONFIG)
        spec = importlib.util.spec_from_file_location('scan_test_wrapper', cfg['control_wrapper'])
        cls.wrapper = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.wrapper)

    def test_pointer_signatures_and_outputs(self):
        fp = ctypes.POINTER(ctypes.c_float)
        read_type = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, fp, fp)
        solve_type = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, fp, fp, fp, ctypes.POINTER(ctypes.c_int))
        received = []
        def read(handle, flange, tcp):
            for i in range(6):
                flange[i], tcp[i] = i + 1, i + 11
            return 1
        def solve(handle, pose, reference, joint, solution):
            received.append(([pose[i] for i in range(6)], [reference[i] for i in range(6)]))
            for i in range(6):
                joint[i] = i + 20
            solution[0] = 6
            return 1
        robot = self.wrapper.DoosanRobot.__new__(self.wrapper.DoosanRobot)
        robot._handle, robot._closed = 1, False
        robot._lib = Mock()
        robot._lib.doosan_controller_read_actual_cartesian_state = read_type(read)
        robot._lib.doosan_controller_solve_closest_ik = solve_type(solve)
        poses = robot.read_actual_cartesian_state()
        self.assertEqual(poses['flange_pose_mm_zyz_deg'], [1, 2, 3, 4, 5, 6])
        self.assertEqual(poses['tcp_pose_mm_zyz_deg'], [11, 12, 13, 14, 15, 16])
        result = robot.solve_closest_ik([100, 200, 300, 40, 50, 60], [1, 2, 3, 4, 5, 6])
        self.assertEqual(received, [([100, 200, 300, 40, 50, 60], [1, 2, 3, 4, 5, 6])])
        self.assertEqual(result, {'target_joint_deg': [20, 21, 22, 23, 24, 25], 'solution_space': 6})
        robot._lib.doosan_controller_solve_closest_ik = solve_type(lambda *args: 0)
        with self.assertRaisesRegex(RuntimeError, 'SDK IK failed'):
            robot.solve_closest_ik([0]*6, [0]*6)


if __name__ == '__main__':
    unittest.main()
