#!/bin/bash
# 우측 펜더를 눕혀서(도장면 위로, Rx+90) NLP IK. 배치는 후드와 같은 작업 높이대.
#   ./run_fender_flat.sh                       (기본 x7y4z3: out_x7y4z3/NX350h_right_fender_flat, stitched_x7y4z3/...)
#   GANTRY_ENV=legacy ./run_fender_flat.sh
# TCP/툴 간섭 플래그는 run_all.sh 와 동일하게 유지할 것 (tcp 프레임 + --tool-clearance-weight 100).
# 2026-10-20: run_all.sh 와 같이 배치는 place_pose(env xyz_offset / overrides 반영), node-limit-scale 은 env yaml
#   nlp.node_limit_scale 기본(legacy 0.85,0.50 = 종전 하드코딩 값 / x7y4z3 0.85,0.25). 갠트리 한계·부드러움 가중치도 yaml.
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh" || exit 1   # 환경 활성화 + 경로
part=NX350h_right_fender_flat
prepare_input NX350h_right_fender || exit 1      # 입력은 세운 펜더와 같은 planner CSV (배치만 다름)
mkdir -p $OUT/$part $LOG
place_pose "$part" "-0.966,-0.744,1.151" "1.5707963,0,0" || exit 1
echo "===== $part (gantry_env $GANTRY_ENV, part_xyz $PART_XYZ rpy $PART_RPY) ====="

python $SCRIPT $IN/NX350h_right_fender/trajectory_*.csv \
  --urdf $URDF --gantry-env $GANTRY_ENV --output $OUT/$part --stride 1 \
  --tcp-frame tcp --tool-clearance-weight 100 \
  --initial-joint-deg "$INIT_JOINT" \
  --part-xyz="$PART_XYZ" --part-rpy="$PART_RPY" \
  --ipopt-print-level 0 "$@" \
  > $LOG/$part.log 2>&1
echo "fender_flat exit=$?"
# row 사이 전이 이어붙이기 (모든 모션 생성에 포함)
rm -rf "${STITCHED:?}"/"${part:?}"
ros2 run paint_viz stitch_transitions \
  --joint-csv-dir $OUT/$part --pattern "*_nlp_command.csv" --urdf $URDF --gantry-env $GANTRY_ENV --tcp-frame tcp \
  --scene "$(scene_file $part)" \
  --output $STITCHED/$part \
  > $LOG/${part}_stitch.log 2>&1
echo "fender_flat stitch exit=$? ($(grep -o '전이 총 시간 [0-9.]*s' $LOG/${part}_stitch.log), $(grep -o '최악 추종오차 [^,]*' $LOG/${part}_stitch.log))"
