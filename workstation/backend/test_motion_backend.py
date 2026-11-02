"""Offline adapter tests. Never run an optimizer, ROS, camera, or robot.

vehicle_painting_robot_motion_planning Python:
python -s -m unittest discover -s workstation/backend -p 'test_motion*.py' -v
"""
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import yaml
import motion_backend as adapter

REPO = Path(__file__).resolve().parents[2]
MOTION = REPO / "motion_planning"


class MotionTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="workstation_motion_test_")
        self.addCleanup(self.temporary.cleanup)
        self.base = Path(self.temporary.name)
        self.source, self.job = self.base / "source", self.base / "job"
        self.source.mkdir()
        self.job.mkdir()
        self.csv = self.source / "painting_trajectory.csv"
        fields = ["row_index", "point_index", "frame_id", "s", "position_x", "position_y", "position_z",
                  "orientation_x", "orientation_y", "orientation_z", "orientation_w", "paint"]
        with self.csv.open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(fields)
            for row in [3, 1]:  # geometric/index sorting must not reorder the path.
                for i in range(3):
                    writer.writerow([row, i, "world", .04 * i, 1 + .04 * i, row * .1, -.4, 0, 0, 0, 1, i != 0])
        self.request = {"motionRoot": str(MOTION), "jobDir": str(self.job), "sourceDirectory": str(self.source),
                        "modelKey": adapter.model(MOTION)["modelKey"],
                        "xyz": [-1.2, -.4, 1.8], "rpy": [.1, -.2, .3], "velocity": .42, "acceleration": 1.2}

    def test_authoritative_csv_order_and_unchanged_original(self):
        before = self.csv.read_bytes()
        (self.source / "trajectory_999.csv").write_text("stale unrelated row")
        command = adapter.prepare(self.request)
        self.assertEqual(before, self.csv.read_bytes())
        files = sorted((self.job / "input").glob("*.csv"))
        self.assertEqual(len(files), 2)
        actual = []
        for file in files:
            with file.open() as stream:
                actual.extend(csv.DictReader(stream))
        with self.csv.open() as stream:
            self.assertEqual(actual, list(csv.DictReader(stream)))
        self.assertIn("--input-dir", command)
        self.assertNotIn("--execute", command)

    def test_no_hidden_offset_and_only_expected_stages(self):
        source_config = MOTION / "config/gantry/x7y4z3.yaml"
        before = hashlib.sha256(source_config.read_bytes()).hexdigest()
        command = adapter.prepare(self.request)
        sys.path.insert(0, str(MOTION / "scripts"))
        import motion_planning as entry
        cfg = entry.read_config(self.job / "config/motion_planning.yaml")
        from motion_planner.configuration import load_gantry_env
        env = load_gantry_env(cfg["gantry_env"])
        _, _, scene, xyz, rpy = entry.part_settings(cfg, adapter.PART, env)
        self.assertEqual(scene["part_xyz"], self.request["xyz"])
        self.assertEqual(scene["part_rpy"], self.request["rpy"])
        self.assertEqual(scene["part_file"], "")
        self.assertEqual(cfg["stages"], ["time_profile", "optimization"])
        transition = adapter.transition_command(self.request, self.job / "results/example")
        self.assertNotIn("--scene", transition)
        self.assertNotIn("--execute", transition)
        self.assertIn("--tcp-frame", transition)
        result = subprocess.run([*command, "--check"], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("No output files or hardware connection", result.stdout)
        self.assertFalse((self.job / "results").exists())
        self.assertEqual(before, hashlib.sha256(source_config.read_bytes()).hexdigest())

    def test_invalid_inputs_fail_before_writing(self):
        for key, value in [("xyz", [0, 0, float("nan")]), ("rpy", [0, 1]),
                           ("velocity", 0), ("acceleration", float("inf"))]:
            with self.subTest(key=key), self.assertRaises(ValueError):
                adapter.prepare({**self.request, key: value})
        self.assertFalse((self.job / "input").exists())

    def test_failed_planner_cannot_publish_success(self):
        with patch.object(adapter.subprocess, "run", side_effect=subprocess.CalledProcessError(1, "planner")):
            with self.assertRaises(subprocess.CalledProcessError):
                adapter.run(self.request)
        self.assertFalse((self.job / "result.json").exists())

    def test_changed_model_rejected(self):
        with self.assertRaisesRegex(ValueError, "변경"):
            adapter.prepare({**self.request, "modelKey": "stale"})
        self.assertFalse((self.job / "input").exists())

    def test_time_profile_preserves_pose_and_row_order(self):
        adapter.prepare(self.request)
        output = self.job / "timed"
        result = subprocess.run([sys.executable, "-s", str(MOTION / "src/motion_planner/1_time_profile.py"),
                                 str(self.job / "input"), str(output), ".42", "1.2"], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        for source in (self.job / "input").glob("*.csv"):
            with source.open() as a, (output / source.name).open() as b:
                for raw, timed in zip(csv.DictReader(a), csv.DictReader(b)):
                    for key in ["row_index", "position_x", "position_y", "position_z",
                                "orientation_x", "orientation_y", "orientation_z", "orientation_w"]:
                        self.assertEqual(float(raw[key]), float(timed[key]))
                    self.assertIn("t", timed)

    def test_model_assets_and_eight_joint_initial_posture(self):
        data = adapter.model(MOTION)
        self.assertEqual(data["root"], "world_base")
        self.assertEqual(len([j for j in data["joints"] if j["type"] != "fixed"]), 8)
        for link in data["links"]:
            for visual in link["visuals"]:
                shape = visual["geometry"]
                if shape["type"] == "mesh":
                    self.assertTrue((REPO / "description" / shape["asset"]).is_file())
        self.assertAlmostEqual(next(j["position"] for j in data["joints"] if j["name"] == "joint_1"), math.pi / 2)

    def test_threejs_link_transforms_match_pinocchio(self):
        """Run the actual TypeScript FK (without visuals/DOM) and compare every link."""
        import pinocchio as pin
        data = adapter.model(MOTION)
        for link in data["links"]:
            link["visuals"] = []
        node = Path.home() / ".local/node-v22.17.0-linux-x64/bin/node"
        if not node.is_file():
            self.skipTest("Node 22 is needed for TypeScript numerical test")
        code = """
import {readFileSync} from 'node:fs';
import {buildRobot, setPose} from './src/lib/motion.ts';
import * as THREE from 'three';
const data = JSON.parse(readFileSync(0, 'utf8'));
const root = await buildRobot(data);
const result = {};
for (const link of data.links) result[link.name] = root.getObjectByName(link.name).matrixWorld.toArray();
const placement = new THREE.Group();
setPose(placement, [-1.2,-.4,1.8], [.1,-.2,.3]); placement.updateMatrixWorld(true);
result.placement = placement.matrixWorld.toArray();
console.log(JSON.stringify(result));
"""
        proc = subprocess.run([str(node), "--experimental-strip-types", "--input-type=module", "-e", code],
                              cwd=REPO / "workstation", input=json.dumps(data), capture_output=True, text=True)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        matrices = json.loads(proc.stdout)
        model = pin.buildModelFromUrdf(data["urdfPath"])
        q = pin.neutral(model)
        for joint in data["joints"]:
            if joint["type"] == "fixed":
                continue
            index = model.getJointId(joint["name"])
            q[model.joints[index].idx_q] = joint["position"]
        state = model.createData()
        pin.forwardKinematics(model, state, q)
        pin.updateFramePlacements(model, state)
        for link in data["links"]:
            expected = state.oMf[model.getFrameId(link["name"])].homogeneous
            actual = np.array(matrices[link["name"]]).reshape(4, 4).T
            np.testing.assert_allclose(actual, expected, atol=1e-10)
        from motion_planner.scene_config import rotation_from_rpy
        actual = np.array(matrices["placement"]).reshape(4, 4).T
        np.testing.assert_allclose(actual[:3, :3], rotation_from_rpy(self.request["rpy"]), atol=1e-12)
        np.testing.assert_allclose(actual[:3, 3], self.request["xyz"])


if __name__ == "__main__":
    unittest.main()
