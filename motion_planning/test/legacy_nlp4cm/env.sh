#!/bin/bash
# nlp4cm 공통 환경 — 모든 스크립트가 `source "$(dirname "$0")/env.sh"` 로 읽는다. 경로는 전부 이 파일 위치에서 계산한다.
#
#   REPO        vehicle_painting_robot 저장소 루트 (= 이 파일의 두 단계 위)
#   PLANNER_CSV planner 출력: $REPO/painting_trajectory_planner/path_planner/test/csv/<part>/trajectory_NNN.csv  ← 두 파이프라인의 공유 지점
#   PLANNER_DATA 부품 점군:  $REPO/painting_trajectory_planner/path_planner/test/data/<part>.ply
#   IN          NLP 입력 = planner CSV + 시간 프로파일(t, path_speed, path_acceleration). prepare_input 이 자동 생성/갱신
#   OUT/STITCHED/COLLISION/LOG/SEQ   갠트리 환경별 산출물 (legacy 는 접미사 없음, 그 외 _<env>)
#   SCRIPT      trajectory_nlp.py (robot_controller/doosan_a0912_controller/test/scripts)
#   WS          colcon 워크스페이스 ($PAINT_CELL_WS, 기본 ~/paint_cell_ws)
#
#   GANTRY_ENV=x7y4z3 (기본)  → out_x7y4z3/, log_x7y4z3/, stitched_x7y4z3/, sequence_x7y4z3/  (현재 셀: 7 x 4 x 3 m 갠트리)
#   GANTRY_ENV=legacy         → out/, log/, ... (gantry.stp 실측 구형 환경, 회귀 비교용)
#   환경 정의는 paint_viz/config/gantry/<name>.yaml 하나뿐이다 (ros2 run paint_viz gantry_env --list).
NLP="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$NLP/../.." && pwd)"
export PAINT_CELL_REPO="${PAINT_CELL_REPO:-$REPO}"
export NLP4CM="$NLP"
WS="${PAINT_CELL_WS:-$HOME/paint_cell_ws}"

# conda(paintcell) + ROS2 + 워크스페이스 활성화. 이미 활성화돼 있으면 건너뛴다.
if ! python -c "import paint_viz, pinocchio, casadi" >/dev/null 2>&1; then
  source "$NLP/../activate_env.sh" >/dev/null || { echo "[env.sh] 환경 활성화 실패 (motion_planning/activate_env.sh)" >&2; return 1 2>/dev/null || exit 1; }
fi
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"

GANTRY_ENV=${GANTRY_ENV:-x7y4z3}
if [ "$GANTRY_ENV" = "legacy" ]; then ENV_SUFFIX=""; else ENV_SUFFIX="_$GANTRY_ENV"; fi
PLANNER_CSV=$REPO/painting_trajectory_planner/path_planner/test/csv
PLANNER_DATA=$REPO/painting_trajectory_planner/path_planner/test/data
IN=$NLP/input
OUT=$NLP/out$ENV_SUFFIX
LOG=$NLP/log$ENV_SUFFIX
STITCHED=$NLP/stitched$ENV_SUFFIX
COLLISION=$NLP/collision$ENV_SUFFIX
SEQ=$NLP/sequence$ENV_SUFFIX
VIZ=$NLP/viz
SCRIPT=$REPO/robot_controller/doosan_a0912_controller/test/scripts/trajectory_nlp.py
# planner CSV 에 붙이는 TCP 시간 프로파일 (add_time_profile.py): 등속 0.42 m/s, 가감속 1.2 m/s^2 (2026-09-22 요구)
TIME_PROFILE_V=${TIME_PROFILE_V:-0.42}
TIME_PROFILE_A=${TIME_PROFILE_A:-1.2}

_genv() { python -m paint_viz.gantry_env --name "$GANTRY_ENV" "$@"; }
URDF=$(_genv --get urdf.primitive)        || { echo "[env.sh] gantry_env '$GANTRY_ENV' 조회 실패" >&2; return 1 2>/dev/null || exit 1; }
URDF_COLL=$(_genv --get urdf.collision)
INIT_JOINT=$(_genv --get nlp.initial_joint)   # trajectory_nlp --initial-joint-deg (직동축 m, 회전축 deg)
export NLP REPO WS GANTRY_ENV ENV_SUFFIX PLANNER_CSV PLANNER_DATA IN OUT LOG STITCHED COLLISION SEQ VIZ SCRIPT URDF URDF_COLL INIT_JOINT TIME_PROFILE_V TIME_PROFILE_A

# planner CSV(test/csv/<part>) → NLP 입력(input/<part>). planner 쪽이 더 새로우면 다시 만든다.
# planner 스키마에는 t/path_speed 가 없어 add_time_profile.py 가 사다리꼴 프로파일을 붙인다 (내용은 그 외 동일).
# planner 출력이 없고 input 만 있으면(저장소에 들어 있는 입력) 그대로 쓴다.
prepare_input() {
  local part=$1 src=$PLANNER_CSV/$1 dst=$IN/$1 newest_src newest_dst
  if [ ! -d "$src" ]; then
    if [ -d "$dst" ]; then return 0; fi
    echo "[env.sh] $part: planner CSV 가 없다 ($src). planner 를 먼저 돌리거나 input/$part 를 넣을 것" >&2; return 1
  fi
  newest_src=$(ls -t "$src"/trajectory_*.csv 2>/dev/null | head -1)
  newest_dst=$(ls -t "$dst"/trajectory_*.csv 2>/dev/null | head -1)
  if [ -z "$newest_src" ]; then echo "[env.sh] $part: $src 에 trajectory_*.csv 가 없다" >&2; return 1; fi
  if [ -z "$newest_dst" ] || [ "$newest_src" -nt "$newest_dst" ]; then
    echo "[env.sh] $part: planner CSV -> 시간 프로파일(V=$TIME_PROFILE_V, A=$TIME_PROFILE_A) -> input/$part"
    rm -rf "$dst"; python "$NLP/add_time_profile.py" "$src" "$dst" "$TIME_PROFILE_V" "$TIME_PROFILE_A" || return 1
  fi
}

# legacy 월드 기준으로 튜닝된 part_xyz "x,y,z" 를 현재 환경 좌표로 바꾼다 (env yaml part_placement.xyz_offset).
place_xyz() { _genv --part-xyz="$1"; }
# 파츠별 배치: place_pose <part> <legacy xyz> <legacy rpy> → PART_XYZ / PART_RPY 변수 설정 (env yaml overrides 반영)
place_pose() {
  local out; out=$(_genv --part-pose "$1" --part-xyz="$2" --part-rpy="$3") || return 1
  PART_XYZ=$(echo "$out" | sed -n 1p); PART_RPY=$(echo "$out" | sed -n 2p)
}

# RViz/stitch/collision 용 scene yaml. legacy 는 viz/scene_<part>.yaml 그대로,
# 다른 환경은 part_xyz 만 변환한 viz/env_<env>/scene_<part>.yaml 을 만들어 그 경로를 돌려준다 (git 제외, 매번 생성).
# 파일 안의 $REPO/$NLP4CM 은 paint_viz.scene_config.load_scene_config 가 읽을 때 치환한다.
scene_file() {
  local part=$1 src=$VIZ/scene_$1.yaml
  if [ "$GANTRY_ENV" = "legacy" ]; then echo "$src"; return; fi
  local dst=$VIZ/env_$GANTRY_ENV/scene_$part.yaml
  mkdir -p "$(dirname "$dst")"
  python - "$src" "$dst" "$GANTRY_ENV" "$part" <<'PY'
import sys, yaml
from paint_viz.gantry_env import load_gantry_env
src, dst, name, part = sys.argv[1:5]
env = load_gantry_env(name)
doc = yaml.safe_load(open(src))
scene = doc.get("scene", doc)
xyz, rpy = env.part_pose(part, scene["part_xyz"], scene.get("part_rpy", [0.0, 0.0, 0.0]))
scene["part_xyz"] = [round(v, 6) for v in xyz]
scene["part_rpy"] = [round(v, 7) for v in rpy]
scene["gantry_env"] = env.name
with open(dst, "w", encoding="utf-8") as f:
    f.write(f"# {src} 에서 gantry_env {env.name} 용으로 자동 생성 (part_xyz 에 part_placement.xyz_offset 적용). 직접 편집하지 말 것.\n")
    yaml.safe_dump({"scene": scene}, f, allow_unicode=True, sort_keys=False)
PY
  echo "$dst"
}
