#!/bin/bash
# 4cm waypoint NLP 결과를 RViz2로 재생.
#   ./play.sh <part> [speed_scale]            (GANTRY_ENV=legacy ./play.sh ES300h_hood 로 환경 선택, 기본 x7y4z3)
#   part: ES300h_hood | ES300h_front_bumper | NX350h_right_fender | NX350h_right_fender_flat | ES300h_trunk | ...
#   (out/<part> 와 viz/scene_<part>.yaml 이 있는 이름이면 무엇이든)
# stitched/<part> 가 out/<part> 보다 새로우면 row+전이 이어붙인 궤적을, 아니면(재계획 후 아직 stitch 안 함) out/ 을 쓴다.
# TCP 자취는 launch 기본 auto = tcp(노즐 팁) 프레임. URDF 는 launch 가 gantry_env 에서 고른다 (urdf.mesh).
part=${1:-ES300h_hood}
speed=${2:-2.0}   # 경로 속도 0.42 m/s 기준 (0.1 m/s 시절 8.0 과 같은 화면 속도)
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/../env.sh" || exit 1   # 환경 활성화 + 경로
if [ ! -d "$OUT/$part" ] || [ ! -f "$VIZ/scene_$part.yaml" ]; then
  echo "사용법: $0 <part> [배속]  — $OUT/<part> 와 viz/scene_<part>.yaml 이 있어야 한다" >&2
  ls -d $OUT/*/ 2>/dev/null | xargs -n1 basename >&2
  exit 1
fi
csv_dir=$OUT/$part
if [ -d "$STITCHED/$part" ]; then
  newest_out=$(ls -t $OUT/$part/*_nlp_joint.csv 2>/dev/null | head -1)
  newest_st=$(ls -t $STITCHED/$part/*_joint.csv 2>/dev/null | head -1)
  if [ -n "$newest_st" ] && [ "$newest_st" -nt "$newest_out" ]; then
    csv_dir=$STITCHED/$part   # row+전이 이어붙인 궤적 (워프 없음)
  else
    echo "[play] $STITCHED/$part 가 out 보다 오래됐다 — out 을 재생한다 (stitch_all.sh 다시 돌릴 것)" >&2
  fi
fi
echo "gantry_env=$GANTRY_ENV joint_csv_dir=$csv_dir"
exec ros2 launch paint_viz paint_viz.launch.py \
  gantry_env:=$GANTRY_ENV \
  scene:="$(scene_file $part)" \
  joint_csv_dir:=$csv_dir \
  speed_scale:=$speed
