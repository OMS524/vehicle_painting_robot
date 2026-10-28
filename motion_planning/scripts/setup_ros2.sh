#!/usr/bin/env bash
# 최초 1회: bash motion_planning/scripts/setup_ros2.sh
# 환경 생성은 miniforge/setup.sh control에서 수행한다. 이 파일은 ROS 패키지만 빌드한다.
set -euo pipefail
MP="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source "$MP/scripts/activate_ros2.sh"
set -u
WS="$PAINT_CELL_WS"
COLCON="$CONDA_PREFIX/bin/colcon"
if [[ ! -x "$COLCON" ]]; then
    echo "[setup] 제어 환경에 빌드 도구가 없습니다. miniforge/config/motion_planning.yaml을 확인하세요." >&2
    exit 1
fi
for pkg in paint_viz dsr_description2; do
    test -f "$WS/src/$pkg/package.xml" || { echo "[setup] 패키지 없음: $WS/src/$pkg" >&2; exit 1; }
done
(cd "$WS" && "$COLCON" build --packages-select dsr_description2 paint_viz)
set +u
source "$WS/install/setup.bash"
python -s -c "import pinocchio, casadi, yaml, rclpy; print('Python imports OK:', pinocchio.__version__, casadi.__version__)"
ros2 run paint_viz gantry_env --name x7y4z3 --get urdf.mesh
test -f "$(ros2 pkg prefix paint_viz)/share/paint_viz/urdf/tool_assy.stl"
echo "[setup] 완료. source $MP/scripts/activate_ros2.sh"
