#!/usr/bin/env python3
"""Offline orchestration only. Numerical work is delegated to the unchanged four stages.

conda activate vehicle_painting_robot_control
python -s motion_planning/scripts/motion_planning.py --check
python -s motion_planning/scripts/motion_planning.py --run --part ES300h_hood
python -s motion_planning/scripts/motion_planning.py --run --stage all --part ES300h_hood
"""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
import yaml
from motion_planner.configuration import load_gantry_env, expand_project_path

STAGES = ("time_profile", "optimization", "transition", "collision", "sequence")
MODULES = {"optimization": "optimization", "transition": "transition", "collision": "collision", "sequence": "sequence"}


def read_config(path):
    path = Path(path).expanduser().resolve()
    cfg = yaml.safe_load(path.read_text())
    for key in ("parts_dir", "planner_csv_root", "fallback_input_root", "output_root"):
        cfg[key] = (path.parent / cfg[key]).resolve()
    return cfg


def part_settings(cfg, part, env):
    if Path(part).name != part or part in (".", ".."):
        raise ValueError("part must be a single directory name")
    path = cfg["parts_dir"] / f"scene_{part}.yaml"
    doc = yaml.safe_load(path.read_text())
    planning = doc.get("planning", {})
    scene = dict(doc["scene"])
    xyz, rpy = env.part_pose(part, scene["part_xyz"], scene.get("part_rpy", [0, 0, 0]))
    # Preserve env.sh's serialized precision and part-specific override behavior.
    scene["part_xyz"] = [round(v, 6) for v in xyz]
    scene["part_rpy"] = [round(v, 7) for v in rpy]
    scene["gantry_env"] = env.name
    scene["part_file"] = expand_project_path(scene["part_file"])
    return path, planning, scene, ",".join(f"{v:.6g}" for v in xyz), ",".join(f"{v:.7g}" for v in rpy)


def module_command(name, args):
    # Module aliases refer to numbered files; invoking main avoids runpy's duplicate import.
    return [sys.executable, "-s", "-c",
            f"from motion_planner import {name}; raise SystemExit({name}.main())", *map(str, args)]


def make_commands(cfg, part, run, source_run, env, stages, input_dir=None):
    config_path, planning, scene, xyz, rpy = part_settings(cfg, part, env)
    source_part = planning.get("input_part", part)
    raw = Path(input_dir).expanduser().resolve() if input_dir else cfg["planner_csv_root"] / source_part
    reuse_prepared = False
    if not input_dir and not any(raw.glob("trajectory_[0-9]*.csv")):
        raw = cfg["fallback_input_root"] / source_part
        reuse_prepared = True
    if "time_profile" in stages and not any(raw.glob("trajectory_[0-9]*.csv")):
        raise FileNotFoundError(f"No row CSV input: {raw}")
    def upstream(stage):
        return (run if stage in stages else source_run) / stage / part
    prepared = run / "input" / part if "time_profile" in stages else source_run / "input" / part
    if "time_profile" not in stages and not prepared.is_dir():
        prepared = raw
    scene["trajectory_dir"] = str(prepared)
    scene_path = run / "scenes" / f"{part}.yaml"
    commands = []
    if "time_profile" in stages:
        if reuse_prepared:
            # Original prepare_input reused saved time profiles when planner data was absent.
            commands.append(("time_profile", [sys.executable, "-s", "-c",
                "from pathlib import Path; import shutil,sys; src,dst=map(Path,sys.argv[1:]); "
                "dst.mkdir(parents=True,exist_ok=True); "
                "[shutil.copy2(p,dst/p.name) for p in sorted(src.glob('trajectory_[0-9]*.csv'))]",
                str(raw), str(prepared)]))
        else:
            commands.append(("time_profile", [sys.executable, "-s", str(ROOT / "src/motion_planner/1_time_profile.py"),
                str(raw), str(prepared), str(cfg["time_profile"]["velocity_m_s"]), str(cfg["time_profile"]["acceleration_m_s2"])]))
    if "optimization" in stages:
        # Filenames are unchanged by the time-profile stage, so this works before it runs.
        files = sorted((raw if "time_profile" in stages else prepared).glob("trajectory_[0-9]*.csv"))
        if not files:
            raise FileNotFoundError(f"No optimization input: {prepared}")
        opt = planning.get("optimization_arguments", cfg["optimization_arguments"])
        args = [*(prepared / p.name for p in files), "--urdf", env.urdf_path("primitive"),
                "--gantry-env", env.path, "--output", run / "optimization" / part,
                *opt, "--initial-joint-deg", env.get("nlp.initial_joint"),
                "--part-xyz=" + xyz, "--part-rpy=" + rpy, *planning.get("extra_optimization_arguments", [])]
        commands.append(("optimization", module_command("optimization", args)))
    if "transition" in stages:
        commands.append(("transition", module_command("transition", [
            "--joint-csv-dir", upstream("optimization"), "--pattern", "*_nlp_command.csv",
            "--urdf", env.urdf_path("primitive"), "--gantry-env", env.path,
            "--scene", scene_path, "--output", run / "transition" / part,
            *cfg["transition_arguments"], *planning.get("transition_arguments", [])])))
    if "collision" in stages:
        directory = upstream("transition")
        pattern = "*_joint.csv"
        if "transition" not in stages and not directory.is_dir():
            directory, pattern = upstream("optimization"), "*_nlp_joint.csv"
        commands.append(("collision", module_command("collision", [
            "--joint-csv-dir", directory, "--pattern", pattern,
            "--urdf", env.urdf_path("collision"), "--gantry-env", env.path,
            "--scene", scene_path, *cfg["collision_arguments"]])))
    if "sequence" in stages:
        commands.append(("sequence", module_command("sequence", [
            "--gantry-env", env.path, "--sequence", cfg["sequence"], "--part", part,
            "--paint-dir", upstream("transition"), "--out-dir", run / "sequence" / part])))
    for _, command in commands:
        if any(str(v).split("=", 1)[0] == "--execute" for v in command):
            raise ValueError("This entry point only generates files; --execute belongs to robot_controller")
    return config_path, scene, commands


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--config", type=Path, default=ROOT / "config/motion_planning.yaml")
    ap.add_argument("--part", action="append", help="Repeat to select several parts")
    ap.add_argument("--gantry-env", default=None)
    ap.add_argument("--stage", choices=("configured", "all", *STAGES), default="configured")
    ap.add_argument("--source-run", type=Path, default=ROOT / "log/imported_x7y4z3",
                    help="Existing output used by individual downstream stages; never overwritten")
    ap.add_argument("--input-dir", type=Path, help="Explicit row-CSV directory for one part")
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--check", action="store_true", help="Print resolved commands; no output/hardware")
    mode.add_argument("--run", action="store_true", help="Generate files, never command hardware")
    args = ap.parse_args(argv)
    cfg = read_config(args.config)
    env = load_gantry_env(args.gantry_env or cfg["gantry_env"])
    parts = args.part or cfg["parts"]
    if args.input_dir and len(parts) != 1:
        ap.error("--input-dir requires one --part")
    stages = list(STAGES) if args.stage == "all" else (cfg["stages"] if args.stage == "configured" else [args.stage])
    if not stages or any(s not in STAGES for s in stages):
        ap.error("Unknown or empty stage list")
    run = cfg["output_root"] / (datetime.now().strftime("%Y%m%d_%H%M%S_%f") if args.run else "<new_run>")
    source = args.source_run.expanduser().resolve()
    plans = [(part, *make_commands(cfg, part, run, source, env, stages, args.input_dir)) for part in parts]
    for part, _, _, commands in plans:
        print(f"[{part}] {env.name}")
        for stage, command in commands:
            print(stage + ": " + shlex.join(list(map(str, command))))
    if not args.run:
        print("Check complete. No output files or hardware connection.")
        return 0
    run.mkdir(parents=True, exist_ok=False)
    (run / "config/parts").mkdir(parents=True)
    (run / "scenes").mkdir()
    (run / "logs").mkdir()
    shutil.copy2(args.config, run / "config/motion_planning.yaml")
    shutil.copy2(env.path, run / "config/gantry.yaml")
    sequence_file = ROOT / "config/sequence" / f"{cfg['sequence']}.yaml"
    if sequence_file.is_file():
        shutil.copy2(sequence_file, run / "config/sequence.yaml")
    child_env = os.environ.copy()
    child_env["PYTHONPATH"] = str(ROOT / "src") + os.pathsep + child_env.get("PYTHONPATH", "")
    child_env["MOTION_RUN"] = str(run)
    manifest = {"gantry_env": env.name, "parts": parts, "source_run": str(source), "stages": []}
    try:
        for part, config_path, scene, commands in plans:
            shutil.copy2(config_path, run / "config/parts" / config_path.name)
            (run / "scenes" / f"{part}.yaml").write_text(yaml.safe_dump({"scene": scene}, sort_keys=False))
            for stage, command in commands:
                log = run / "logs" / f"{part}_{stage}.log"
                if stage == "transition":
                    input_path = Path(command[command.index("--joint-csv-dir") + 1])
                    if len(list(input_path.glob("*_nlp_command.csv"))) < 2:
                        # Same single-row skip as run_all.sh's stitch helper.
                        log.write_text("Transition skipped: fewer than two command rows.\n")
                        manifest["stages"].append({"part": part, "stage": stage, "status": "skipped", "command": command})
                        continue
                print(f"Running {part}/{stage}: {log}", flush=True)
                with log.open("w") as stream:
                    proc = subprocess.run(command, env=child_env, stdout=stream, stderr=subprocess.STDOUT)
                manifest["stages"].append({"part": part, "stage": stage, "command": command, "returncode": proc.returncode})
                if proc.returncode:
                    raise RuntimeError(f"{part}/{stage} failed ({proc.returncode}). See {log}")
    finally:
        (run / "run.json").write_text(json.dumps(manifest, indent=2))
        print(f"Results: {run}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
