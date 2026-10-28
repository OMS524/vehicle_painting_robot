#!/usr/bin/env python3
"""Replay generated CSVs in RViz; no robot connection.

source motion_planning/scripts/activate_ros2.sh
python -s motion_planning/scripts/visualize_motion.py --part ES300h_hood
python -s motion_planning/scripts/visualize_motion.py --run-dir motion_planning/log/<run> --part ES300h_hood
"""
import argparse
import json
from pathlib import Path
import shlex
import shutil
import subprocess
import sys
import tempfile

from motion_planning import ROOT, read_config, part_settings
from motion_planner.configuration import load_gantry_env
import yaml


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--run-dir", type=Path, default=ROOT / "log/imported_x7y4z3")
    ap.add_argument("--part", default="ES300h_hood")
    ap.add_argument("--stage", choices=("auto", "optimization", "transition", "sequence"), default="auto")
    ap.add_argument("--gantry-env", default=None)
    ap.add_argument("--speed", type=float, default=2.0)
    ap.add_argument("--check", action="store_true", help="Print paths only; no GUI or files")
    args = ap.parse_args(argv)
    run = args.run_dir.expanduser().resolve()
    if Path(args.part).name != args.part or args.part in (".", ".."):
        ap.error("part must be a single directory name")
    stage = args.stage
    if stage == "auto":
        # Same preference as the old play.sh: transition only if newer than NLP.
        opt = list((run / "optimization" / args.part).glob("*_nlp_joint.csv"))
        trans = list((run / "transition" / args.part).glob("*_joint.csv"))
        stage = "transition" if trans and (not opt or max(p.stat().st_mtime for p in trans) > max(p.stat().st_mtime for p in opt)) else "optimization"
    directory = run / stage / args.part
    pattern = "seq_*_joint.csv" if stage == "sequence" else "*_joint.csv"
    if not any(directory.glob(pattern)):
        ap.error(f"No trajectory CSV: {directory}/{pattern}")
    cfg = read_config(ROOT / "config/motion_planning.yaml")
    manifest = json.loads((run / "run.json").read_text()) if (run / "run.json").is_file() else {}
    saved_env = run / "config/gantry.yaml"
    env = load_gantry_env(args.gantry_env or (saved_env if saved_env.is_file() else manifest.get("gantry_env", cfg["gantry_env"])))
    scene_path = run / "scenes" / f"{args.part}.yaml"
    scene = None
    if not scene_path.is_file():
        _, planning, scene, _, _ = part_settings(cfg, args.part, env)
        input_part = planning.get("input_part", args.part)
        candidates = [run / "input" / args.part, cfg["fallback_input_root"] / input_part,
                      cfg["planner_csv_root"] / input_part]
        scene["trajectory_dir"] = str(next((p for p in candidates if p.is_dir()), candidates[-1]))

    ros2 = shutil.which("ros2")

    def command(path):
        # The system ros2 script has a /usr/bin/python3 shebang. Use this script's
        # Python so launch-file imports see the same Conda CasADi/Pinocchio.
        return [sys.executable, "-s", ros2 or "ros2", "launch", "paint_viz", "paint_viz.launch.py", f"gantry_env:={env.path}",
                f"scene:={path}", f"joint_csv_dir:={directory}", f"joint_csv_pattern:={pattern}",
                f"speed_scale:={args.speed}"]

    if args.check:
        print(shlex.join(command(scene_path if scene is None else "<temporary-scene.yaml>")))
        if scene is not None:
            print(yaml.safe_dump({"scene": scene}, sort_keys=False))
        return 0
    if ros2 is None:
        ap.error("ros2를 찾지 못했습니다. source motion_planning/scripts/activate_ros2.sh를 먼저 실행하세요.")
    if scene is None:
        return subprocess.call(command(scene_path))
    with tempfile.TemporaryDirectory(prefix="motion_scene_") as tmp:
        path = Path(tmp) / "scene.yaml"
        path.write_text(yaml.safe_dump({"scene": scene}, sort_keys=False))
        return subprocess.call(command(path))


if __name__ == "__main__":
    raise SystemExit(main())
