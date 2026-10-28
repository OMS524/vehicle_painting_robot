#!/bin/bash
# 충돌 검사. 운동학이 계획 URDF 와 동일하면서 collision 형상이 있는 env 의 urdf.collision 을 쓴다
# (legacy: paint_cell_8axis_mesh.urdf. gantry_mesh_c 는 조인트 원점/부호가 달라 legacy 궤적과 맞지 않음 — 환경을 섞지 말 것).
# stitched 가 있는 파츠는 row+전이 전체, front_bumper 는 stitch 불가라 NLP row 만 검사한다.
# 갠트리 환경: GANTRY_ENV=x7y4z3 ./collision_all.sh (기본 x7y4z3, legacy 는 GANTRY_ENV=legacy).
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/../env.sh" || exit 1   # 환경 활성화 + 경로
mkdir -p "$COLLISION"
PARTS=${PARTS:-"ES300h_hood ES300h_trunk NX350h_right_fender NX350h_right_fender_flat NX350h_right_fender_flat_tool ES300h_front_bumper"}
for part in $PARTS; do
  echo "===== $part ====="
  if [ -d $STITCHED/$part ]; then dir=$STITCHED/$part; pat="*_joint.csv"; elif [ -d $OUT/$part ]; then dir=$OUT/$part; pat="*_nlp_joint.csv"; else echo "$part skip (출력 없음)"; continue; fi
  ros2 run paint_viz check_collisions \
    --joint-csv-dir $dir --pattern "$pat" --urdf $URDF_COLL --gantry-env $GANTRY_ENV \
    --scene "$(scene_file $part)" --stride 2 --part-stride 4 \
    > $COLLISION/$part.log 2>&1
  echo "$part exit=$? ($dir)"; tail -6 $COLLISION/$part.log
done
echo ALL_DONE
