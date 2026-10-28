#!/bin/bash
# 우측 펜더 단독 NLP (run_all.sh 의 NX350h_right_fender 와 같은 배치). GANTRY_ENV 로 환경 선택 (기본 legacy).
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh" || exit 1   # 환경 활성화 + 경로
mkdir -p "$OUT" "$LOG"
place_pose NX350h_right_fender "-0.990,0.051,2.280" "0,0,0" || exit 1
python $SCRIPT $IN/NX350h_right_fender/trajectory_*.csv \
  --urdf $URDF --gantry-env $GANTRY_ENV \
  --output $OUT/NX350h_right_fender --stride 1 \
  --initial-joint-deg "$INIT_JOINT" \
  --part-xyz="$PART_XYZ" --part-rpy="$PART_RPY" \
  --ipopt-max-iter 4000 --ipopt-print-level 0 \
  > $LOG/NX350h_right_fender.log 2>&1
echo "fender exit=$?"
