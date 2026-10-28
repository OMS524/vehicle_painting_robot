#!/usr/bin/env python3
"""Offline import regression tests; no ROS nodes, cameras or robot connections.

conda activate vehicle_painting_robot_motion_planning
python -B -s motion_planning/test/test_lazy_imports.py
"""
import os
from pathlib import Path
import subprocess
import sys
import textwrap
import unittest


ROOT = Path(__file__).resolve().parents[1]
PREAMBLE = f"""
import importlib
import importlib.abc
from pathlib import Path
import sys
sys.path.insert(0, {str(ROOT / 'src')!r})
ROOT = Path({str(ROOT)!r})
"""
BLOCK_NUMERICAL = """
class BlockNumerical(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in ('casadi', 'pinocchio'):
            raise ModuleNotFoundError('Blocked numerical dependency: ' + fullname)
sys.meta_path.insert(0, BlockNumerical())
"""


class LazyImports(unittest.TestCase):
    def run_code(self, code, *, block_numerical=False):
        # Each check gets a fresh module cache; the developer's ROS/PYTHONPATH
        # settings must not accidentally supply dependencies to these tests.
        env = os.environ.copy()
        env.pop('PYTHONPATH', None)
        env.pop('PYTHONHOME', None)
        env['PYTHONDONTWRITEBYTECODE'] = '1'
        code = PREAMBLE + (BLOCK_NUMERICAL if block_numerical else '') + textwrap.dedent(code)
        result = subprocess.run([sys.executable, '-B', '-s', '-c', code],
                                cwd=ROOT, env=env, capture_output=True, text=True, timeout=45)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

    def test_configuration_without_numerical_dependencies(self):
        self.run_code("""
            from motion_planner.configuration import load_gantry_env
            from motion_planner.scene_config import load_scene_config
            env = load_gantry_env('x7y4z3')
            assert env.urdf_path('mesh').is_file()
            scene = load_scene_config(ROOT / 'config/parts/scene_ES300h_hood.yaml')
            assert Path(scene['part_file']).is_file()
            assert 'casadi' not in sys.modules
            assert 'pinocchio' not in sys.modules
        """, block_numerical=True)

    def test_time_profile_alias_without_optimizers(self):
        self.run_code("""
            from motion_planner import time_profile
            from motion_planner.time_profile import trapezoid
            assert time_profile.trapezoid is trapezoid
            assert importlib.import_module('motion_planner.1_time_profile') is time_profile
            assert importlib.import_module('motion_planner.time_profile') is time_profile
            at, duration = trapezoid(1.0)
            assert duration > 0 and at(0.0) == (0.0, 0.0, time_profile.A)
            assert 'casadi' not in sys.modules
            assert 'pinocchio' not in sys.modules
        """, block_numerical=True)

    def test_missing_dependency_fails_when_optimizer_is_used(self):
        self.run_code("""
            from motion_planner.configuration import load_gantry_env
            from motion_planner import optimization
            try:
                optimization.main
            except ModuleNotFoundError as error:
                assert 'Blocked numerical dependency: casadi' in str(error), str(error)
            else:
                raise AssertionError('Using optimization must still require CasADi')
        """, block_numerical=True)

    def test_stage_aliases_and_entry_points(self):
        for name, filename in (('optimization', '2_optimization'),
                               ('transition', '3_transition'), ('sequence', '4_sequence')):
            with self.subTest(stage=name):
                self.run_code(f"""
                    from motion_planner import {name} as stage
                    assert callable(stage.main)
                    assert stage is importlib.import_module('motion_planner.{name}')
                    assert stage is importlib.import_module('motion_planner.{filename}')
                    assert Path(stage.__file__).name == '{filename}.py'
                    sys.argv = ['{name}', '--help']
                    try:
                        result = stage.main()
                    except SystemExit as error:
                        assert error.code == 0, error.code
                    else:
                        assert result in (None, 0), result
                """)

    def test_direct_submodule_and_ros_compatibility_imports(self):
        self.run_code("""
            from motion_planner.transition import solve_transition_path
            from motion_planner.sequence import main
            from motion_planner import transition, sequence
            assert transition.solve_transition_path is solve_transition_path
            assert sequence.main is main
            assert 'casadi' not in sys.modules
            sys.path.insert(0, str(ROOT / 'ros2_ws/src/paint_viz'))
            from paint_viz import stitch, sequence as ros_sequence
            assert stitch.main is transition.main
            assert ros_sequence.main is sequence.main
        """)


if __name__ == '__main__':
    unittest.main(verbosity=2)
