#!/usr/bin/env python3
"""Offline preservation checks against the pre-move snapshot (no robot/ROS node).

conda activate vehicle_painting_robot_control
python -B -s motion_planning/test/check_restructure.py
"""
import ast
import copy
import hashlib
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
BACKUP = ROOT / "backup/20261028_before_directory_restructure"
OLD = BACKUP / "motion_planning"
CORE = ROOT / "src/motion_planner"
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "scripts"))
import yaml

RENAMES = {
    "stitch": "3_transition", "sequence": "4_sequence", "collision_check": "collision",
    "viz_ik": "kinematics", "trajectory_io": "data_io", "gantry_env": "configuration",
}
ALIASES = dict(RENAMES, stitch="transition", sequence="sequence")
PATH_FUNCTIONS = {
    "repo_root", "_source_root", "_share_root", "config_dirs", "_workspace_source_root",
    "nlp4cm_root", "expand_project_path", "urdf_dirs", "default_package_dirs",
}


def parse(path):
    return ast.parse(path.read_text())


class Normalize(ast.NodeTransformer):
    def visit_Constant(self, node):
        # Directory relocation only; keep algorithm constants/expressions checked.
        if isinstance(node.value, str):
            node.value = node.value.replace("robot_control/", "robot_controller/")
        return node

    def visit_ImportFrom(self, node):
        if node.module and node.module.startswith("paint_viz"):
            fields = node.module.split(".")
            node.module = "motion_planner" + ("." + ALIASES.get(fields[1], fields[1]) if len(fields) > 1 else "")
        return node

    def generic_visit(self, node):
        super().generic_visit(node)
        if isinstance(node, (ast.Module, ast.FunctionDef, ast.ClassDef)) and node.body:
            first = node.body[0]
            if isinstance(first, ast.Expr) and isinstance(first.value, ast.Constant) and isinstance(first.value.value, str):
                node.body.pop(0)
        return node


def normalized(tree, skip_functions=(), skip_assignments=(), skip_imports=()):
    tree = Normalize().visit(copy.deepcopy(tree))
    tree.body = [n for n in tree.body if not (
        isinstance(n, (ast.FunctionDef, ast.ClassDef)) and n.name in skip_functions
        or isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id in skip_assignments for t in n.targets)
        or isinstance(n, ast.ImportFrom) and n.module in skip_imports)]
    return ast.dump(tree, include_attributes=False)


class Preservation(unittest.TestCase):
    def test_core_code_and_defaults(self):
        count = 0
        for path in (OLD / "paint_viz/paint_viz").glob("*.py"):
            if path.stem in ("__init__", "scene_publisher", "trajectory_player", "cell_pose_info"):
                continue
            target = CORE / (RENAMES.get(path.stem, path.stem) + ".py")
            old, new = path.read_text(), target.read_text()
            if path.stem == "make_dock_urdfs":
                old = old.replace('Path(__file__).resolve().parents[1]\n                    / "urdf/paint_cell_8axis_gantry_mesh_c.urdf"',
                                  'urdf_dirs()[0] / "paint_cell_8axis_gantry_mesh_c.urdf"')
            if path.stem == "split_gantry_stl":
                old = old.replace('Path.home() / "paint_cell_ws/src/dsr_description2/meshes"',
                                  'repo_root() / "description/paint_cell/a0912_meshes"')
            skips = PATH_FUNCTIONS if path.stem == "gantry_env" else ()
            # Added imports in the two asset-location helper scripts only.
            imports = ("motion_planner.configuration",) if path.stem in ("make_dock_urdfs", "split_gantry_stl") else ()
            with self.subTest(module=path.stem):
                self.assertEqual(normalized(ast.parse(old), skips, skip_imports=imports),
                                 normalized(ast.parse(new), skips, skip_imports=imports))
            count += 1
        self.assertEqual(count, 15)

    def test_optimizer_code_and_defaults(self):
        old = parse(BACKUP / "robot_control_scripts/trajectory_nlp.py")
        new = parse(CORE / "2_optimization.py")
        opts = dict(skip_functions=("run_robot",), skip_assignments=("ROOT", "PROJECT_ROOT"),
                    skip_imports=("motion_planner.configuration",))
        self.assertEqual(normalized(old, **opts), normalized(new, **opts))
        # Execution code was moved, not rewritten; it is NOT invoked in this test.
        execution = parse(ROOT.parent / "robot_controller/doosan_a0912_controller/test/scripts/trajectory_execution.py")
        for name in ("run_robot", "trajectory_duration_sec"):
            before = next(n for n in old.body if isinstance(n, ast.FunctionDef) and n.name == name)
            after = next(n for n in execution.body if isinstance(n, ast.FunctionDef) and n.name == name)
            self.assertEqual(ast.dump(before), ast.dump(after))

    def test_time_profile_unchanged(self):
        self.assertEqual((OLD / "nlp4cm/add_time_profile.py").read_bytes(), (CORE / "1_time_profile.py").read_bytes())

    def test_ros_nodes_unchanged(self):
        for name in ("scene_publisher", "trajectory_player", "cell_pose_info"):
            self.assertEqual(normalized(parse(OLD / f"paint_viz/paint_viz/{name}.py")),
                             normalized(parse(ROOT / f"ros2_ws/src/paint_viz/paint_viz/{name}.py")))

    def test_assets_and_results_byte_identical(self):
        pairs = [
            (OLD / "paint_viz/urdf", ROOT.parent / "description/paint_cell/urdf"),
            (OLD / "dsr_description2/meshes", ROOT.parent / "description/paint_cell/a0912_meshes"),
            (OLD / "nlp4cm/input", ROOT / "test/input"),
        ]
        for old, new in (("out", "optimization"), ("stitched", "transition"), ("sequence", "sequence"),
                         ("collision", "collision"), ("log", "logs")):
            pairs.append((OLD / f"nlp4cm/{old}_x7y4z3", ROOT / f"log/imported_x7y4z3/{new}"))
        count = 0
        for before, after in pairs:
            self.assertTrue(before.is_dir(), str(before))
            originals = {p.relative_to(before) for p in before.rglob("*") if p.is_file()}
            moved = {p.relative_to(after) for p in after.rglob("*") if p.is_file()}
            self.assertEqual(originals, moved, str(before))
            for relative in originals:
                self.assertEqual(hashlib.sha256((before / relative).read_bytes()).digest(),
                                 hashlib.sha256((after / relative).read_bytes()).digest(), str(relative))
                count += 1
        print(f"\nByte-identical assets/input/results: {count} files")

    def test_config_values_preserved(self):
        for path in (OLD / "paint_viz/config/gantry").glob("*.yaml"):
            before = yaml.safe_load(path.read_text().replace("/robot_control/", "/robot_controller/"))
            self.assertEqual(before, yaml.safe_load((ROOT / "config/gantry" / path.name).read_text()))
        old = yaml.safe_load((OLD / "paint_viz/config/sequence/paint_with_tool.yaml").read_text())
        new = yaml.safe_load((ROOT / "config/sequence/paint_with_tool.yaml").read_text())
        old["sequence"]["paint"]["dir"] = new["sequence"]["paint"]["dir"]
        self.assertEqual(old, new)
        for path in (OLD / "nlp4cm/viz").glob("scene_*.yaml"):
            before = yaml.safe_load(path.read_text())["scene"]
            after = yaml.safe_load((ROOT / "config/parts" / path.name).read_text())["scene"]
            for key in ("part_file", "trajectory_dir"):
                before[key] = after[key]
            self.assertEqual(before, after)

    def test_offline_paths_and_commands(self):
        from motion_planner.configuration import load_gantry_env, default_package_dirs
        from motion_planning import read_config, make_commands, STAGES
        cfg = read_config(ROOT / "config/motion_planning.yaml")
        for name in ("legacy", "spec_c", "x7y4z3"):
            env = load_gantry_env(name)
            for kind in ("primitive", "mesh", "collision"):
                self.assertTrue(env.urdf_path(kind).is_file())
        env = load_gantry_env("x7y4z3")
        for scene in (ROOT / "config/parts").glob("scene_*.yaml"):
            part = scene.stem.removeprefix("scene_")
            _, data, commands = make_commands(cfg, part, ROOT / "log/<check>", ROOT / "log/imported_x7y4z3", env, STAGES)
            self.assertTrue(Path(data["part_file"]).is_file(), data["part_file"])
            self.assertEqual([s for s, _ in commands], list(STAGES))
            self.assertFalse(any("--execute" in cmd for _, cmd in commands))
        self.assertTrue(Path(default_package_dirs()[0]).is_dir())


if __name__ == "__main__":
    unittest.main(verbosity=2)
