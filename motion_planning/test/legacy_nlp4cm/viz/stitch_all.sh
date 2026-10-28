#!/bin/bash
# 4cm NLP joint CSV -> row+전이 이어붙인 궤적 (재-stitch 전용; run_all.sh / run_fender_flat.sh 가 NLP 직후 자동으로 같은 일을 한다).
# 리트랙트 방향은 분사축 반대(stitch 기본 --retract-dir spray) 라 세운 front_bumper 도 잇는다.
# TCP 는 run_all.sh 와 같은 tcp 프레임(노즐 팁). out/ 을 tcp 로 다시 풀지 않은 상태에서 돌리면
# link_6 기준 궤적을 노즐 기준으로 잇게 되므로 반드시 run_all.sh 먼저.
# 갠트리 환경: GANTRY_ENV=x7y4z3 ./stitch_all.sh (기본 x7y4z3, legacy 는 GANTRY_ENV=legacy). URDF/디렉터리는 env.sh 가 정한다.
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/../env.sh" || exit 1   # 환경 활성화 + 경로
PARTS=${PARTS:-"ES300h_hood ES300h_trunk ES300h_front_bumper NX350h_right_fender NX350h_right_fender_flat"}
for part in $PARTS; do
  echo "===== $part ====="
  tcp=tcp
  # 트렁크 move 10->11 은 TCP 가 트렁크 리드 전체를 가로지르는데(y +0.80 -> -0.73, 리드 최고점 z 1.228)
  # 기본 리트랙트 상한 100 mm 로는 부품과 0.4 mm 까지 붙는다(baseline 도 동일). 400 mm 로 올려야 여유 71.6 mm.
  extra=""   # 2026-10-06: 트렁크 --retract-max 0.4 예외 제거. planner 4단계 지그재그 방향 수정으로 move 10->11/12->13 이 1.5 m -> 0.13 m, 기본 리트랙트로 여유 126.5 mm
  if [ ! -d $OUT/$part ]; then echo "$part skip ($OUT/$part 없음)"; continue; fi
  rm -rf "${STITCHED:?}"/"${part:?}"
  ros2 run paint_viz stitch_transitions \
    --joint-csv-dir $OUT/$part --pattern "*_nlp_command.csv" \
    --urdf $URDF --gantry-env $GANTRY_ENV --tcp-frame $tcp $extra \
    --scene "$(scene_file $part)" \
    --output $STITCHED/$part
  echo "$part exit=$?"
done
echo ALL_DONE
