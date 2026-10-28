#!/usr/bin/env bash
# source motion_planning/scripts/activate_ros2.sh
# Conda -> ROS Humble -> 이 워크스페이스 순서. 카메라/스캔 환경은 변경하지 않는다.
# 선택: CONDA_ROOT, PAINT_CELL_CONDA_ENV, ROS_SETUP, PAINT_CELL_WS
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo "source로 실행하세요: source motion_planning/scripts/activate_ros2.sh" >&2
    exit 1
fi
_mp_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export PAINT_CELL_REPO="$(cd "$_mp_dir/.." && pwd)"
export PAINT_CELL_WS="${PAINT_CELL_WS:-$_mp_dir/ros2_ws}"
if [[ "$(type -t conda 2>/dev/null)" != "function" ]]; then
    _conda_found=0
    for _c in "${CONDA_ROOT:-}" "$HOME/miniforge3" "$HOME/miniconda3" "$HOME/anaconda3" /opt/conda; do
        if [[ -n "$_c" && -f "$_c/etc/profile.d/conda.sh" ]]; then
            source "$_c/etc/profile.d/conda.sh"
            _conda_found=1
            break
        fi
    done
    if [[ "$_conda_found" != 1 ]]; then
        echo "[activate] Conda를 찾지 못했습니다. CONDA_ROOT를 설정하세요." >&2
        return 1
    fi
fi
conda activate "${PAINT_CELL_CONDA_ENV:-vehicle_painting_robot_control}" || return 1
source "${ROS_SETUP:-/opt/ros/humble/setup.bash}" || return 1
if [[ -f "$PAINT_CELL_WS/install/setup.bash" ]]; then
    source "$PAINT_CELL_WS/install/setup.bash" || return 1
fi
# source 코드를 우선하여 순수 알고리즘 수정에는 ROS 재빌드가 필요하지 않도록 한다.
export PYTHONPATH="$_mp_dir/src${PYTHONPATH:+:$PYTHONPATH}"
echo "[activate] $CONDA_PREFIX / ROS ${ROS_DISTRO:-?} / $PAINT_CELL_WS"
unset _mp_dir _conda_found _c
