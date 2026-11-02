#!/usr/bin/env python3
"""Workstation adapter only: URDF preview and offline motion-planner jobs.

Called by Tauri with the motion-planning Python environment. No robot SDK is
imported. Original path CSVs, planner algorithms and shared YAMLs are not edited.
Tests: python -s -m unittest discover -s workstation/backend -p 'test_motion*.py'
"""
import copy
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import traceback
import xml.etree.ElementTree as ET

import yaml

PART = "workstation_path"


def configuration(root):
    root = Path(root).resolve()
    os.environ["PAINT_CELL_REPO"] = str(root.parent)
    sys.path.insert(0, str(root / "src"))
    from motion_planner.configuration import load_gantry_env
    config = yaml.safe_load((root / "config/motion_planning.yaml").read_text())
    return config, load_gantry_env(config["gantry_env"])


def vector(text, default="0 0 0"):
    return [float(v) for v in (text or default).split()]


def origin(element):
    node = element.find("origin")
    return {"xyz": vector(node.get("xyz") if node is not None else None),
            "rpy": vector(node.get("rpy") if node is not None else None)}


def model(root):
    config, env = configuration(root)
    description = Path(root).parent / "description"
    urdf = env.urdf_path("mesh").resolve()
    robot = ET.parse(urdf).getroot()
    materials = {m.get("name"): vector(m.find("color").get("rgba"))
                 for m in robot.findall("material") if m.find("color") is not None}
    links = []
    for link in robot.findall("link"):
        visuals = []
        for visual in link.findall("visual"):
            geometry = visual.find("geometry")[0]
            shape = {"type": geometry.tag, **geometry.attrib}
            if geometry.tag == "mesh":
                name = geometry.get("filename")
                if name.startswith("package://dsr_description2/meshes/"):
                    path = description / "paint_cell/a0912_meshes" / name.split("/meshes/", 1)[1]
                elif name.startswith("package://paint_viz/urdf/"):
                    path = description / "paint_cell/urdf" / name.split("/urdf/", 1)[1]
                elif name.startswith("package://"):
                    raise ValueError(f"지원하지 않는 메시 패키지: {name}")
                else:
                    path = urdf.parent / name.removeprefix("file://")
                path = path.resolve(strict=True)
                shape = {"type": "mesh", "asset": str(path.relative_to(description.resolve())),
                         "scale": vector(geometry.get("scale"), "1 1 1")}
            elif geometry.tag == "box":
                shape["size"] = vector(geometry.get("size"))
            elif geometry.tag not in ("sphere", "cylinder"):
                raise ValueError(f"지원하지 않는 URDF geometry: {geometry.tag}")
            material = visual.find("material")
            color = None
            if material is not None:
                node = material.find("color")
                color = vector(node.get("rgba")) if node is not None else materials.get(material.get("name"))
            visuals.append({**origin(visual), "geometry": shape, "color": color})
        links.append({"name": link.get("name"), "visuals": visuals})
    joints = []
    values = [float(v) for v in env.get("nlp.initial_joint").split(",")]
    if len(values) != 8:
        raise ValueError("8축 초기 자세가 필요합니다.")
    positions = dict(zip(["gantry_x", "gantry_y"] + [f"joint_{i}" for i in range(1, 7)],
                         values[:2] + [math.radians(v) for v in values[2:]]))
    for joint in robot.findall("joint"):
        axis = joint.find("axis")
        kind = joint.get("type")
        if kind not in ("fixed", "prismatic", "revolute", "continuous"):
            raise ValueError(f"지원하지 않는 조인트: {kind}")
        if kind != "fixed" and joint.get("name") not in positions:
            raise ValueError(f"초기 자세가 없는 조인트: {joint.get('name')}")
        joints.append({"name": joint.get("name"), "type": kind, **origin(joint),
                       "parent": joint.find("parent").get("link"),
                       "child": joint.find("child").get("link"),
                       "axis": vector(axis.get("xyz") if axis is not None else None, "1 0 0"),
                       "position": positions.get(joint.get("name"), 0)})
    roots = {link["name"] for link in links} - {joint["child"] for joint in joints}
    if roots != {"world_base"}:
        raise ValueError(f"world_base 루트가 필요합니다: {roots}")
    return {"root": "world_base", "name": env.name, "title": env.get("title", env.name),
            "modelKey": model_key(root, env),
            "urdfPath": str(urdf), "motionRoot": str(root), "python": sys.executable,
            "links": links, "joints": joints, "initialJoint": values,
            "velocity": config["time_profile"]["velocity_m_s"],
            "acceleration": config["time_profile"]["acceleration_m_s2"]}


def model_key(root, env):
    digest = hashlib.sha256()
    for path in [Path(root) / "config/motion_planning.yaml", env.path,
                 env.urdf_path("mesh"), env.urdf_path("primitive")]:
        digest.update(Path(path).read_bytes())
    return digest.hexdigest()


def finite_vector(value):
    if not isinstance(value, list) or len(value) != 3 or not all(math.isfinite(float(v)) for v in value):
        raise ValueError("위치와 RPY는 유한한 숫자 3개여야 합니다.")
    return [float(v) for v in value]


def prepare(request):
    root, job = Path(request["motionRoot"]), Path(request["jobDir"])
    source = Path(request["sourceDirectory"])
    config, env = configuration(root)
    if request.get("modelKey") != model_key(root, env):
        raise ValueError("미리보기 이후 모션 설정/URDF가 변경되었습니다. 4단계 모델을 다시 불러오세요.")
    xyz = finite_vector(request["xyz"])
    rpy = finite_vector(request["rpy"])
    velocity, acceleration = float(request["velocity"]), float(request["acceleration"])
    if not all(math.isfinite(v) and v > 0 for v in (velocity, acceleration)):
        raise ValueError("속도와 가속도는 0보다 큰 유한한 값이어야 합니다.")
    # The combined, finalized CSV is authoritative, not old individual row files.
    source_csv = source / "painting_trajectory.csv"
    with source_csv.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        fields = reader.fieldnames
        required = ["row_index", "point_index", "frame_id", "paint", "s", *[f"position_{v}" for v in "xyz"],
                    *[f"orientation_{v}" for v in "xyzw"]]
        if not fields or any(key not in fields for key in required):
            raise ValueError("완료 경로 CSV에 필수 위치/자세/s/row_index 열이 없습니다.")
        rows = {}
        for point in reader:
            index = int(point["row_index"])
            if not all(math.isfinite(float(point[key])) for key in required[4:]):
                raise ValueError(f"Row {index}: 유효하지 않은 경로 값")
            if sum(float(point[f"orientation_{v}"]) ** 2 for v in "xyzw") < 1e-12:
                raise ValueError(f"Row {index}: 길이가 0인 자세 quaternion")
            rows.setdefault(index, []).append(point)
    if not rows or any(len(points) < 2 for points in rows.values()):
        raise ValueError("각 경로에는 최소 2개의 포인트가 필요합니다.")
    for points in rows.values():
        if any(float(b["s"]) < float(a["s"]) for a, b in zip(points, points[1:])):
            raise ValueError("경로의 s 값이 감소합니다. 경로를 다시 생성하세요.")
    input_dir, config_dir = job / "input", job / "config"
    input_dir.mkdir()
    (config_dir / "parts").mkdir(parents=True)
    shutil.copy2(source_csv, job / "source_trajectory.csv")
    for ordinal, points in enumerate(rows.values()):
        with (input_dir / f"trajectory_{ordinal:03d}.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(points)
    config = copy.deepcopy(config)
    config.update(parts=[PART], parts_dir=str(config_dir / "parts"),
                  planner_csv_root=str(input_dir), fallback_input_root=str(input_dir),
                  output_root=str(job / "results"), stages=["time_profile", "optimization"])
    config["time_profile"] = {"velocity_m_s": velocity, "acceleration_m_s2": acceleration}
    gantry = copy.deepcopy(env.data)
    # UI placement is already in world_base. Do NOT add legacy part offsets.
    gantry["part_placement"] = {"xyz_offset": [0, 0, 0], "overrides": {}}
    gantry_file = config_dir / "gantry.yaml"
    gantry_file.write_text(yaml.safe_dump({"gantry_env": gantry}, sort_keys=False))
    config["gantry_env"] = str(gantry_file)
    config_file = config_dir / "motion_planning.yaml"
    config_file.write_text(yaml.safe_dump(config, sort_keys=False))
    scene = {"part_file": "", "part_xyz": xyz, "part_rpy": rpy}
    (config_dir / "parts" / f"scene_{PART}.yaml").write_text(yaml.safe_dump({"scene": scene}))
    command = [sys.executable, "-s", str(root / "scripts/motion_planning.py"),
               "--config", str(config_file), "--part", PART, "--input-dir", str(input_dir)]
    return command


def run(request):
    job = Path(request["jobDir"])
    command = prepare(request)
    print("모션 계획: 시간 프로파일 → 최적화 → 경로 간 전이 (하드웨어 연결 없음)", flush=True)
    subprocess.run([*command, "--run"], check=True)
    manifests = list((job / "results").glob("*/run.json"))
    if len(manifests) != 1:
        raise RuntimeError("현재 작업의 run.json을 식별하지 못했습니다.")
    run_dir = manifests[0].parent
    manifest = json.loads(manifests[0].read_text())
    if not manifest["stages"] or any(s.get("returncode", 0) for s in manifest["stages"]):
        raise RuntimeError("실패한 모션 계획 단계가 있습니다.")
    if not any((run_dir / "optimization" / PART).glob("*_nlp_joint.csv")):
        raise RuntimeError("최적화된 관절 경로가 없습니다.")
    run_transition(request, run_dir, manifest)
    result = {"runDirectory": str(run_dir), "part": PART, "sourceDirectory": request["sourceDirectory"],
              "xyz": request["xyz"], "rpy": request["rpy"],
              "collisionChecked": False}
    (job / "result.json").write_text(json.dumps(result, indent=2))


def transition_command(request, run_dir):
    """Use the existing fixed-retract mode: omit --scene, do not fake a surface.

    The shared orchestrator always passes --scene and consequently requires a
    part cloud. This adapter invokes the same unchanged transition module with
    its public no-scene mode (default retract 0.03 m).
    """
    job = Path(request["jobDir"])
    config = yaml.safe_load((job / "config/motion_planning.yaml").read_text())
    from motion_planner.configuration import load_gantry_env
    env = load_gantry_env(job / "config/gantry.yaml")
    args = config.get("transition_arguments", [])
    if any(str(arg).split("=", 1)[0] in ("--execute", "--scene") for arg in args):
        raise ValueError("부품 없는 워크스테이션 전이에 --scene/--execute를 사용할 수 없습니다.")
    return [sys.executable, "-s", "-c",
            "from motion_planner import transition; raise SystemExit(transition.main())",
            "--joint-csv-dir", str(run_dir / "optimization" / PART), "--pattern", "*_nlp_command.csv",
            "--urdf", str(env.urdf_path("primitive")), "--gantry-env", str(env.path),
            "--output", str(run_dir / "transition" / PART), *map(str, args)]


def run_transition(request, run_dir, manifest):
    command = transition_command(request, run_dir)
    record = {"part": PART, "stage": "transition", "mode": "fixed_retract_no_surface", "command": command}
    log = run_dir / "logs" / f"{PART}_transition.log"
    try:
        if len(list((run_dir / "optimization" / PART).glob("*_nlp_command.csv"))) < 2:
            record["status"] = "skipped"
            log.write_text("Transition skipped: fewer than two command rows.\n")
            return
        print(f"Running {PART}/transition (고정 후퇴 거리, 부품 점군 없음): {log}", flush=True)
        child_env = os.environ.copy()
        child_env["PYTHONPATH"] = str(Path(request["motionRoot"]) / "src") + os.pathsep + child_env.get("PYTHONPATH", "")
        with log.open("w") as stream:
            process = subprocess.run(command, env=child_env, stdout=stream, stderr=subprocess.STDOUT)
        record["returncode"] = process.returncode
        if process.returncode:
            raise RuntimeError(f"전이 생성 실패 ({process.returncode}): {log}")
    finally:
        manifest["stages"].append(record)
        (run_dir / "run.json").write_text(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    try:
        if sys.argv[1] == "model":
            print(json.dumps(model(json.load(sys.stdin)["motionRoot"])))
        elif sys.argv[1] == "run":
            run(json.loads(Path(sys.argv[2]).read_text()))
        else:
            raise ValueError("지원하지 않는 작업")
    except Exception:
        traceback.print_exc()
        sys.exit(1)
