#!/bin/bash
# stitched row+전이 → 툴체인저 포함 통합 시퀀스 (홈 → 장착 → 도장 → 반납 → 홈). 파츠별 $SEQ/<part>/ 에
# seq_NNNN_*_joint.csv, sequence.json, sequence_command.csv(1 ms, git 제외) 를 만든다.
#   ./sequence_all.sh                              (기본 x7y4z3, 5 파츠)
#   PARTS="ES300h_hood" ./sequence_all.sh          (일부만)
# 선행: run_all.sh (+ run_fender_flat.sh) 로 stitched_<env>/<part> 가 있어야 한다. 설정: paint_viz/config/sequence/paint_with_tool.yaml
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh" || exit 1
PARTS=${PARTS:-"ES300h_hood ES300h_trunk ES300h_front_bumper NX350h_right_fender NX350h_right_fender_flat"}
mkdir -p "$SEQ"
for part in $PARTS; do
  if [ ! -d "$STITCHED/$part" ]; then echo "$part skip ($STITCHED/$part 없음 — run_all.sh 먼저)"; continue; fi
  ros2 run paint_viz build_sequence --gantry-env "$GANTRY_ENV" --part "$part" --out-dir "$SEQ/$part" "$@" > "$SEQ/$part.log" 2>&1
  echo "$part exit=$? ($(grep -o '총 [0-9.]* s' "$SEQ/$part.log" | head -1); $(grep -o '검증: .*' "$SEQ/$part.log" | cut -c1-120))"
done
echo ALL_DONE
