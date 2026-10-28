"""갠트리 환경(gantry_env) 설정 로더.

왜 있는가
  도장셀 코드는 갠트리 스트로크·장착 높이·조인트 원점 규약·툴체인저 슬롯 좌표를
  여러 파일(URDF 생성기 상수, 셸 스크립트, launch 기본값, dock_path 상수)에 따로
  들고 있었다. 새 갠트리(예: 레일 7 x 4 m, 높이 3 m)를 추가하려면 그 전부를 손봐야
  했고, 기존 환경과 섞여 회귀가 생기기 쉬웠다.

  이 모듈은 `config/gantry/<name>.yaml` 한 파일이 환경 하나를 정의하게 하고,
  모든 스크립트/launch 가 `--gantry-env <name>` (또는 launch 인자 `gantry_env:=`)
  하나로 URDF 경로, 조인트 범위, 툴체인저 위치, 부품 배치 오프셋을 받아가게 한다.

환경 이름
  legacy   gantry.stp 실측 (스트로크 4.392 x 2.142, 장착면 2.066 m, 원점 = 중앙, 축 정방향).
           NLP 도장 파이프라인(nlp4cm/run_all.sh 등)의 기본값. 기존 결과와 동일.
  spec_c   사용자 확정 스펙 (레일 3.0 x 2.4 m, 스트로크 2.8264 x 2.2864, 원점 = 코너,
           두 축 반전, 툴체인저 4슬롯). cell_view.launch / dock_path 의 기본값.
  x7y4z3   신규 레일 7 x 4 m, 레일 높이 3 m. spec_c 규약(코너 원점, 축 반전, 팔 오프셋,
           툴체인저 상대 위치)을 그대로 물려받고 치수만 바뀐다. URDF 는 make_gantry_urdf --env 로 생성.

yaml 스키마 (요약 — 각 yaml 의 주석 참조)
  gantry_env:
    name, title, convention(center|corner)
    rail: {x_length, y_length, z_height}        레일 길이·브리지 중심 높이 [m]
    stroke: {x, y}                              조인트 범위 길이 [m]
    joints: {gantry_x: {axis, lower, upper, origin_xyz}, gantry_y: {...}}
    mount_z, arm_mount: {xyz, rpy}              팔 base_link 장착
    urdf: {generated, primitive, mesh, bare, collision, arm_source}
    frame: {...}                                생성형 환경의 프레임 박스 치수
    toolchanger: {anchor, slots, stand_boxes, plate_size} | null
    home: {gantry, arm_deg}
    nlp: {initial_joint, gantry_velocity_weight, gantry_acceleration_weight}
    motor_limits: {gantry_velocity_m_s, gantry_acceleration_m_s2}   갠트리 계획 한계 (NLP/stitch/sequence 공통)
    part_placement: {xyz_offset}                legacy 월드 기준 part_xyz 에 더할 값

사용
  from motion_planner.configuration import load_gantry_env
  env = load_gantry_env("x7y4z3")          # 이름 또는 yaml 경로
  env.urdf_path("primitive")               # Path
  env.part_xyz([-0.751, -1.316, 1.879])    # 오프셋 적용
  CLI:  python -m paint_viz.gantry_env --name x7y4z3 --get urdf.primitive
"""

import argparse
import json
import os
import sys
from pathlib import Path

import yaml

DEFAULT_ENV = "legacy"
ENV_VAR = "GANTRY_ENV"
_PACKAGE_NAME = "paint_viz"


def repo_root():
    """Resolve the checkout independent of ROS workspace/package nesting."""
    override = os.environ.get("PAINT_CELL_REPO")
    if override:
        return Path(override).expanduser().resolve()
    for parent in Path(__file__).resolve().parents:
        if (parent / "motion_planning/config").is_dir():
            return parent
    raise RuntimeError("Set PAINT_CELL_REPO to the vehicle_painting_robot checkout")


def _source_root():
    return repo_root() / "motion_planning"


def _share_root():
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory(_PACKAGE_NAME))
    except (ImportError, LookupError):
        return None


def config_dirs():
    dirs = []
    extra = os.environ.get("GANTRY_ENV_CONFIG_DIR")
    if extra:
        dirs.append(Path(extra))
    dirs.append(_source_root() / "config/gantry")
    return dirs


def nlp4cm_root():
    """Compatibility name for old callers; new runs use MOTION_RUN."""
    return Path(os.environ.get("NLP4CM", str(_source_root() / "log/imported_x7y4z3")))


def expand_project_path(value):
    from string import Template
    mapping = dict(os.environ)
    repo = repo_root()
    mp = _source_root()
    mapping.setdefault("REPO", str(repo))
    mapping.setdefault("PAINT_CELL_REPO", str(repo))
    mapping.setdefault("MOTION_ROOT", str(mp))
    mapping.setdefault("MOTION_RUN", str(mp / "log/imported_x7y4z3"))
    mapping.setdefault("DESCRIPTION", str(repo / "description"))
    mapping.setdefault("NLP4CM", str(nlp4cm_root()))
    mapping.setdefault("PAINT_CELL_WS", str(mp / "ros2_ws"))
    return os.path.expanduser(Template(str(value)).safe_substitute(mapping))


def urdf_dirs():
    return [repo_root() / "description/paint_cell/urdf"]


def list_envs():
    names = {}
    for directory in config_dirs():
        if not directory.is_dir():
            continue
        for path in sorted(directory.glob("*.yaml")):
            names.setdefault(path.stem, path)
    return names


def resolve_env_file(name_or_path):
    candidate = Path(os.path.expanduser(str(name_or_path)))
    if candidate.suffix in (".yaml", ".yml") or candidate.is_file():
        if not candidate.is_file():
            raise FileNotFoundError(f"gantry_env yaml 이 없다: {candidate}")
        return candidate.resolve()
    for directory in config_dirs():
        path = directory / f"{name_or_path}.yaml"
        if path.is_file():
            return path.resolve()
    known = ", ".join(sorted(list_envs())) or "(없음)"
    raise FileNotFoundError(
        f"gantry_env '{name_or_path}' 를 {[str(d) for d in config_dirs()]} 에서 못 찾았다. 알려진 환경: {known}")


class GantryEnv:
    def __init__(self, data, path):
        self.data = data
        self.path = Path(path)
        self.name = str(data.get("name") or self.path.stem)

    # --- 조회 ----------------------------------------------------------
    def get(self, dotted, default=None):
        node = self.data
        for key in dotted.split("."):
            if not isinstance(node, dict) or key not in node:
                return default
            node = node[key]
        return node

    def __getitem__(self, key):
        return self.data[key]

    @property
    def convention(self):
        return str(self.data.get("convention", "center"))

    @property
    def rail(self):
        return self.data["rail"]

    @property
    def stroke(self):
        return self.data["stroke"]

    @property
    def bridge_z(self):
        return float(self.rail["z_height"])

    @property
    def mount_z(self):
        return float(self.data["mount_z"])

    def joint(self, name):
        return self.data["joints"][name]

    def joint_limits(self):
        """{joint: (lower, upper)} — URDF 와 동일해야 한다 (검증 항목)."""
        return {name: (float(spec["lower"]), float(spec["upper"]))
                for name, spec in self.data["joints"].items()}

    def gantry_center_joint(self):
        """브리지/캐리지가 갠트리 중앙에 오는 조인트값 (limit 중간)."""
        return [0.5 * (lo + hi) for lo, hi in
                (self.joint_limits()["gantry_x"], self.joint_limits()["gantry_y"])]

    def world_travel_range(self):
        """캐리지 중심의 월드 xy 이동 범위 {axis: (min, max)} — 조인트 원점·부호에서 계산."""
        out = {}
        for axis_index, name in ((0, "gantry_x"), (1, "gantry_y")):
            spec = self.joint(name)
            origin = float(spec["origin_xyz"][axis_index])
            sign = float(spec["axis"][axis_index])
            ends = sorted([origin + sign * float(spec["lower"]), origin + sign * float(spec["upper"])])
            out["xy"[axis_index]] = (ends[0], ends[1])
        # gantry_y 원점은 gantry_x_link 기준이라 월드 y 에는 그대로 더해진다 (x 원점의 y 성분 = 0 전제)
        return out

    # --- URDF ------------------------------------------------------------
    def urdf_name(self, kind="primitive"):
        urdf = self.data.get("urdf", {})
        name = urdf.get(kind)
        if not name:
            raise KeyError(f"gantry_env '{self.name}' 에 urdf.{kind} 가 없다 (있는 것: {sorted(k for k, v in urdf.items() if v)})")
        return str(name)

    def urdf_path(self, kind="primitive", must_exist=True):
        name = expand_project_path(self.urdf_name(kind))
        if os.path.isabs(name):
            path = Path(name)
            if must_exist and not path.is_file():
                raise FileNotFoundError(path)
            return path
        for directory in urdf_dirs():
            path = directory / name
            if path.is_file():
                return path
        if must_exist:
            hint = " (make_gantry_urdf --env 로 먼저 생성할 것)" if self.get("urdf.generated") else ""
            raise FileNotFoundError(f"gantry_env '{self.name}' URDF {name} 이 {[str(d) for d in urdf_dirs()]} 에 없다{hint}")
        return (urdf_dirs()[0] if urdf_dirs() else Path.cwd()) / name

    # --- 툴체인저 ----------------------------------------------------------
    def toolchanger_anchor(self):
        """슬롯/스탠드 상대좌표의 기준점 (월드 x, y, z). 없으면 None.

        x/y 는 캐리지 월드 이동범위 끝(x_max|x_min|y_max|y_min) 또는 숫자, z 는 mount_z|bridge_z|floor 또는 숫자.
        툴체인저를 갠트리에 상대적으로(코너 + 팔 장착면 기준 높이) 두면 환경이 바뀌어도 도킹 기하가 유지된다.
        """
        tc = self.data.get("toolchanger")
        if not tc:
            return None
        anchor = tc.get("anchor", {})
        travel = self.world_travel_range()
        refs = {"x_max": travel["x"][1], "x_min": travel["x"][0],
                "y_max": travel["y"][1], "y_min": travel["y"][0],
                "mount_z": self.mount_z, "bridge_z": self.bridge_z, "floor": 0.0}

        def pick(key):
            value = anchor.get(key, 0.0)
            if isinstance(value, str):
                if value not in refs:
                    raise ValueError(f"toolchanger.anchor.{key}={value!r} 는 {sorted(refs)} 중 하나여야 한다")
                return refs[value]
            return float(value)
        return (pick("x"), pick("y"), pick("z"))

    def toolchanger_slots(self):
        """[(name, world_x, world_y, plate_top_z, occupied)] — make_dock_urdfs.SLOTS 와 같은 형식."""
        tc = self.data.get("toolchanger")
        if not tc:
            return []
        ax, ay, az = self.toolchanger_anchor()
        slots = []
        for slot in tc.get("slots", []):
            slots.append((str(slot["name"]), ax + float(slot["dx"]), ay + float(slot["dy"]),
                          az + float(slot["dz"]), bool(slot.get("occupied", True))))
        return slots

    def toolchanger_stand_boxes(self):
        """[(center_xyz, size_xyz)] 월드 좌표."""
        tc = self.data.get("toolchanger")
        if not tc:
            return []
        ax, ay, az = self.toolchanger_anchor()
        boxes = []
        for box in tc.get("stand_boxes", []):
            dx, dy, dz = (float(v) for v in box["center"])
            boxes.append(((ax + dx, ay + dy, az + dz), tuple(float(v) for v in box["size"])))
        riser = tc.get("stand_riser")
        if riser and boxes:
            # 스탠드 최하단 박스 아래를 바닥까지 받치는 기둥 (장착면 기준으로 띄운 환경에서 스탠드가 공중에 뜨지 않게)
            bottom = min(c[2] - sz[2] / 2.0 for c, sz in boxes)
            if bottom > 0.0:
                sx, sy = (float(v) for v in riser["size_xy"])
                boxes.append(((ax + float(riser["dx"]), ay + float(riser["dy"]), bottom / 2.0), (sx, sy, bottom)))
        return boxes

    # --- 부품 배치 --------------------------------------------------------
    def part_xyz(self, legacy_xyz):
        """legacy 월드 기준으로 튜닝된 part_xyz 를 이 환경의 월드 좌표로 옮긴다."""
        offset = self.get("part_placement.xyz_offset", [0.0, 0.0, 0.0])
        return [float(a) + float(b) for a, b in zip(legacy_xyz, offset)]

    def part_pose(self, part, legacy_xyz, legacy_rpy):
        """파츠별 배치 (xyz, rpy). part_placement.overrides.<part> 가 있으면 그 값(이 환경 월드 좌표 그대로),
        없으면 legacy 값 + xyz_offset. (예: x7y4z3 펜더는 팔꿈치-브리지 간섭 때문에 yaw +90° 로 돌려 둔다)"""
        override = (self.get("part_placement.overrides") or {}).get(part)
        if override:
            return ([float(v) for v in override["xyz"]],
                    [float(v) for v in override.get("rpy", legacy_rpy)])
        return self.part_xyz(legacy_xyz), [float(v) for v in legacy_rpy]

    # --- 메타데이터 ---------------------------------------------------------
    def motor_limits(self, default_velocity, default_acceleration):
        """갠트리 직동축 계획 한계 (v [m/s], a [m/s^2]). yaml motor_limits 가 없으면 호출자 기본값.

        2026-10-20: 실제 갠트리 모터 최고 0.8 m/s → 계획은 0.5 m/s (x7y4z3/spec_c yaml). NLP·stitch·sequence 가
        모두 여기서 읽어 세 곳의 한계가 어긋나지 않게 한다.
        """
        velocity = self.get("motor_limits.gantry_velocity_m_s")
        acceleration = self.get("motor_limits.gantry_acceleration_m_s2")
        return (float(default_velocity if velocity is None else velocity),
                float(default_acceleration if acceleration is None else acceleration))

    def metadata(self):
        """plan_metadata.json 에 넣을 요약."""
        travel = self.world_travel_range()
        return {
            "name": self.name,
            "config_file": str(self.path),
            "convention": self.convention,
            "rail_m": {k: float(v) for k, v in self.rail.items()},
            "stroke_m": {k: float(v) for k, v in self.stroke.items()},
            "bridge_z_m": self.bridge_z,
            "mount_z_m": self.mount_z,
            "joint_limits": {k: list(v) for k, v in self.joint_limits().items()},
            "world_travel_m": {k: list(v) for k, v in travel.items()},
            "arm_mount": self.data.get("arm_mount"),
            "toolchanger_slots": [list(s) for s in self.toolchanger_slots()],
            "part_xyz_offset": self.get("part_placement.xyz_offset", [0.0, 0.0, 0.0]),
            "motor_limits": self.get("motor_limits"),
        }


def load_gantry_env(name_or_path=None):
    """이름/경로 → GantryEnv. None 이면 $GANTRY_ENV, 그것도 없으면 legacy."""
    if name_or_path is None or str(name_or_path).strip() == "":
        name_or_path = os.environ.get(ENV_VAR, DEFAULT_ENV)
    path = resolve_env_file(name_or_path)
    with open(path, "r", encoding="utf-8") as stream:
        loaded = yaml.safe_load(stream) or {}
    data = loaded.get("gantry_env", loaded)
    if not isinstance(data, dict) or "joints" not in data:
        raise ValueError(f"{path}: gantry_env 매핑(joints 포함)이 없다")
    return GantryEnv(data, path)


def add_gantry_env_argument(parser, help_suffix=""):
    """argparse 에 --gantry-env 를 붙인다 (모든 스크립트가 같은 문구를 쓰도록)."""
    parser.add_argument(
        "--gantry-env", default=None,
        help="갠트리 환경 이름(legacy|spec_c|x7y4z3) 또는 config/gantry/*.yaml 경로. "
             f"기본: $GANTRY_ENV 또는 '{DEFAULT_ENV}'. {help_suffix}".strip())


def apply_gantry_env(args, urdf_kind="primitive"):
    """argparse 결과에 환경을 적용한다.

    --urdf 를 명시하면 그대로 쓰고(기존 동작), 없으면 --gantry-env(또는 $GANTRY_ENV, legacy)의
    urdf.<urdf_kind> 를 쓴다. --gantry-env 를 줬거나 --urdf 가 비어 있으면 GantryEnv 를 돌려주고
    args.gantry_env_obj 에도 넣는다(메타데이터 기록용). 둘 다 없으면 None.
    """
    requested = getattr(args, "gantry_env", None)
    urdf = getattr(args, "urdf", None)
    env = None
    if requested or urdf is None:
        env = load_gantry_env(requested)
    if urdf is None:
        args.urdf = env.urdf_path(urdf_kind)
    args.gantry_env_obj = env
    return env


def default_package_dirs():
    """Resolve package:// meshes without requiring ROS; source assets are canonical."""
    ws = Path(os.environ.get("PAINT_CELL_WS", str(_source_root() / "ros2_ws"))).expanduser()
    return [str(_source_root() / "ros2_ws/src")] + [
        str(ws / "install" / pkg / "share") for pkg in ("dsr_description2", _PACKAGE_NAME)]


def default_package_dir():
    """호환용: 첫 package_dir. 새 코드는 default_package_dirs() 를 쓸 것."""
    return default_package_dirs()[0]


def main(argv=None):
    parser = argparse.ArgumentParser(description="갠트리 환경 설정 조회")
    parser.add_argument("--name", default=None, help="환경 이름 또는 yaml 경로 (기본 $GANTRY_ENV / legacy)")
    parser.add_argument("--get", action="append", default=[],
                        help="점 표기 키 (예: urdf.primitive, mount_z, stroke.x). 여러 번 가능. "
                             "urdf.* 는 절대 경로로 해석한다")
    parser.add_argument("--part-xyz", default=None, help="legacy 기준 'x,y,z' 를 이 환경 좌표로 변환해 출력")
    parser.add_argument("--part-pose", default=None, metavar="PART",
                        help="파츠 이름. --part-xyz/--part-rpy(legacy 기준)를 받아 이 환경의 xyz, rpy 를 두 줄로 출력 (overrides 반영)")
    parser.add_argument("--part-rpy", default="0,0,0")
    parser.add_argument("--json", action="store_true", help="metadata JSON 출력")
    parser.add_argument("--list", action="store_true", help="알려진 환경 목록")
    args = parser.parse_args(argv)

    if args.list:
        for name, path in list_envs().items():
            print(f"{name}\t{path}")
        return 0
    env = load_gantry_env(args.name)
    if args.json:
        print(json.dumps(env.metadata(), ensure_ascii=False, indent=2))
    for key in args.get:
        if key.startswith("urdf."):
            print(env.urdf_path(key.split(".", 1)[1], must_exist=False))
        elif key == "nlp.initial_joint":
            print(env.get(key))
        else:
            value = env.get(key)
            if value is None:
                print(f"[gantry-env] '{key}' 가 {env.path} 에 없다", file=sys.stderr)
                return 1
            print(",".join(str(v) for v in value) if isinstance(value, list) else value)
    if args.part_pose:
        xyz = [float(v) for v in (args.part_xyz or "0,0,0").split(",")]
        rpy = [float(v) for v in args.part_rpy.split(",")]
        pose_xyz, pose_rpy = env.part_pose(args.part_pose, xyz, rpy)
        print(",".join(f"{v:.6g}" for v in pose_xyz))
        print(",".join(f"{v:.7g}" for v in pose_rpy))
    elif args.part_xyz:
        xyz = [float(v) for v in args.part_xyz.split(",")]
        print(",".join(f"{v:.6g}" for v in env.part_xyz(xyz)))
    if not (args.json or args.get or args.part_xyz or args.part_pose):
        print(f"{env.name}\t{env.path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
