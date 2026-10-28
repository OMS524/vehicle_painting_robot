#!/bin/bash
# 눕힌 펜더 + 실제 툴(Spraygun_full_assy) 장착 기준 NLP.
# TCP = tcp 프레임 (= spray_gun 원점, 노즐 팁, +z 분사) — link_6 이 아니다.
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh" || exit 1   # 환경 활성화 + 경로
mkdir -p $OUT/NX350h_right_fender_flat_tool

python $SCRIPT $IN/NX350h_right_fender/trajectory_*.csv \
  --urdf $URDF --gantry-env $GANTRY_ENV --output $OUT/NX350h_right_fender_flat_tool --stride 1 \
  --tcp-frame tcp \
  --initial-joint-deg "$INIT_JOINT" \
  --part-xyz="$(place_xyz -0.966,-0.744,1.151)" --part-rpy="1.5707963,0,0" \
  --ipopt-print-level 0 \
  > $LOG/NX350h_right_fender_flat_tool.log 2>&1
echo "fender_flat_tool exit=$?"
