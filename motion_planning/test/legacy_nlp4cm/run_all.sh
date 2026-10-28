#!/bin/bash
# TCP = tcp 프레임(스프레이건 노즐 팁, link_6 에서 +z 0.297 m). 2026-09-15 이전 결과(baseline_20260915/)는
# link_6 을 TCP 로 풀어 건이 부품 속으로 0.3 m 들어가 있었다. stitch/collision 도 같은 프레임을 쓴다.
# 페인트통(tcp +x 0.30 m 지점)이 팔꿈치(joint_3)/손목(joint_5) 원점에서 0.35 m 이상 떨어지도록 --tool-clearance-weight.
# 경로축 -y 뒤집기만으로는 center 항이 joint_6→0° 로 당겨 팔이 joint_1 을 돌려 컵을 다시 로봇 쪽으로 가져오고,
# joint_6=90:270 bound 는 수평 분사(범퍼/펜더) 에서 row 000 부터 Infeasible 이라 기하 페널티로 바꿨다(2026-09-15 실측).
#
# 갠트리 환경 (2026-10-13): `GANTRY_ENV=<name> ./run_all.sh` 처럼 환경 변수 하나로 URDF·출력 디렉터리(out_<env>/)·
# 초기 관절·부품 배치(z 오프셋)가 바뀐다. 기본 x7y4z3 (현재 셀, 2026-10-26 부터; 종전 기본 legacy 는 GANTRY_ENV=legacy).
# 정의는 env.sh 와 paint_viz/config/gantry/*.yaml. 입력은 planner 출력(path_planner/test/csv/<part>)을 env.sh prepare_input 이 가져온다.
# 갠트리 속도 (2026-10-20): 모터 최고 0.8 m/s → 계획 0.5 m/s, 가속도 1.6 m/s^2, 갠트리 부드러움 가중치 0.01, 노드 스케일 0.85,0.25 는
# 전부 env yaml(x7y4z3/spec_c: motor_limits, nlp.*) 에서 온다. trajectory_nlp/stitch/sequence 가 같은 값을 읽는다.
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/env.sh" || exit 1   # 환경 활성화 + 경로 (REPO/PLANNER_CSV/IN/OUT/...)
mkdir -p "$OUT" "$LOG"
echo "gantry_env=$GANTRY_ENV urdf=$URDF out=$OUT init_joint=$INIT_JOINT"

# NLP 가 끝나면 바로 row 사이 전이를 이어 stitched/<part> 를 만든다 (2026-09-15 부터 모든 모션 생성에 포함).
# 리트랙트는 분사축 반대 방향(--retract-dir spray 기본) 이라 세운 범퍼도 잇는다. 부품 점군 기반 적응형 높이.
# (2026-10-06) 예전 트렁크 전용 --retract-max 0.4 는 planner 지그재그 방향 오류(move 10->11, 12->13 이 1.5 m) 때문이었고, 4단계 수정으로 불필요해졌다.
stitch() {
  part=$1; shift
  # row 구간은 4 cm 노드(*_nlp_joint.csv)가 아니라 TCP 등속으로 보정된 1 ms 명령(*_nlp_command.csv)을 잇는다.
  # 노드 CSV 를 재생기가 선형보간하면 row 안 TCP 속도가 -11..+17 % 흔들린다(2026-09-22 실측, 도장 품질 요구: row 안 등속).
  n=$(ls $OUT/$part/*_nlp_command.csv 2>/dev/null | wc -l)
  if [ "$n" -lt 2 ]; then echo "$part stitch skip (row $n개)"; return; fi
  rm -rf "${STITCHED:?}"/"${part:?}"
  ros2 run paint_viz stitch_transitions \
    --joint-csv-dir $OUT/$part --pattern "*_nlp_command.csv" --urdf $URDF --gantry-env $GANTRY_ENV --tcp-frame tcp \
    --scene "$(scene_file $part)" --output $STITCHED/$part "$@" > $LOG/${part}_stitch.log 2>&1
  echo "$part stitch exit=$? ($(grep -o '전이 총 시간 [0-9.]*s' $LOG/${part}_stitch.log), $(grep -o '최악 추종오차 [^,]*' $LOG/${part}_stitch.log))"
}

# --node-limit-scale: NLP 노드(4 cm 평균) 제약만 조여서, TCP 등속으로 만든 1 ms 명령의 순간 최대가
# 실제 한계(70 deg/s, 500 deg/s^2) 안에 들게 한다. 0.85,0.55 는 트렁크/눕힌펜더 가속도 순간 x1.02~1.03 초과 → 0.85,0.50.
# 범퍼는 0.42 m/s 에서 한계 자체가 빠듯해 마진을 주면 row 011 이 Infeasible → 1,1 로 두고 로그의 WARNING 으로 초과량을 본다.
# 부품 배치 xyz/rpy 는 legacy 월드 기준 값이며 place_pose 가 현재 환경의 오프셋(예: x7y4z3 z +0.781) 또는 파츠별 덮어쓰기(env yaml overrides)를 적용한다.
run() {
  part=$1; prepare_input "$part" || return 1                       # planner test/csv/<part> -> input/<part> (시간 프로파일)
  place_pose "$part" "$2" "$3" || return 1; xyz=$PART_XYZ; rpy=$PART_RPY; shift 3
  # 2026-10-20: 스케일은 env yaml nlp.node_limit_scale 이 기본 (legacy 0.85,0.50 / x7y4z3 0.85,0.25). 범퍼만 1,1 로 덮어쓴다.
  scale=""; case "$part" in ES300h_front_bumper) scale="--node-limit-scale 1,1" ;; esac
  echo "===== $part (part_xyz $xyz rpy $rpy) ====="
  python $SCRIPT $IN/$part/trajectory_*.csv \
    --urdf $URDF --gantry-env $GANTRY_ENV --output $OUT/$part --stride 1 --tcp-frame tcp --tool-clearance-weight 100 $scale \
    --initial-joint-deg "$INIT_JOINT" \
    --part-xyz="$xyz" --part-rpy="$rpy" \
    --ipopt-print-level 0 "$@" \
    > $LOG/$part.log 2>&1
  echo "$part exit=$?"
  extra=""   # 2026-10-06: 트렁크 --retract-max 0.4 예외 제거. planner 4단계 지그재그 방향 수정으로 move 10->11/12->13 이 1.5 m -> 0.13 m, 기본 리트랙트로 여유 126.5 mm
  stitch $part $extra
}
# PARTS 환경변수로 일부만 돌릴 수 있다: PARTS="ES300h_hood" ./run_all.sh
PARTS=${PARTS:-"ES300h_hood ES300h_front_bumper NX350h_right_fender ES300h_trunk"}
for part in $PARTS; do
  case $part in
    ES300h_hood)         run ES300h_hood          "-0.751,-1.316,1.879" "0,0,0" ;;
    ES300h_front_bumper) run ES300h_front_bumper  "-1.002,-1.040,4.872" "1.5707963,1.5707963,0" --ipopt-max-iter 4000 ;;
    NX350h_right_fender) run NX350h_right_fender  "-0.990,0.051,2.280"  "0,0,0" --ipopt-max-iter 4000 ;;  # z=2.080 이면 최하단 row Infeasible (bound 무관), scene yaml 과 동일하게 2.280
    ES300h_trunk)        run ES300h_trunk         "-0.806,-1.367,1.803" "0,0,0" ;;
    *) echo "모르는 파츠 $part" ;;
  esac
done
echo ALL_DONE
