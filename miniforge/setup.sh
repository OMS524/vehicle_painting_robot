#!/usr/bin/env bash
# 실행: bash /path/to/vehicle_painting_robot/miniforge/setup.sh [all|scan|planner|control] [--update]
# 기본: 없는 환경만 생성. --update: 명시한 기존 환경을 해당 YAML 기준으로 갱신.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CONFIG_DIR="$SCRIPT_DIR/config"
REPO_DIR="$(cd -- "$SCRIPT_DIR/.." && pwd)"
usage() {
    printf '%s\n' \
        '사용법: bash miniforge/setup.sh [all|scan|planner|control] [--update]' \
        '  인자 없음: 세 환경 중 없는 환경만 생성' \
        '  control: motion_planning / robot_controller용 환경 생성' \
        '  control --update: 기존 vehicle_painting_robot_motion_planning을 config/motion_planning.yaml 기준으로 갱신' \
        '  --update는 명시한 대상에만 적용하며, --prune/환경 삭제/ROS 빌드는 하지 않음'
}
TARGET=all
TARGET_SET=0
UPDATE=0
for arg in "$@"; do
    case "$arg" in
        all|scan|planner|control)
            if (( TARGET_SET )); then usage >&2; exit 2; fi
            TARGET="$arg"
            TARGET_SET=1
            ;;
        --update) UPDATE=1 ;;
        -h|--help) usage; exit 0 ;;
        *) usage >&2; exit 2 ;;
    esac
done
# 갱신 대상을 생략한 채 모든 환경을 바꾸지 않도록 한다.
if (( UPDATE && ! TARGET_SET )); then
    printf '%s\n' '--update에는 대상이 필요합니다. 예: control --update' >&2
    usage >&2
    exit 2
fi
case "$TARGET" in
    all) TARGETS=(scan planner control) ;;
    scan|planner|control) TARGETS=("$TARGET") ;;
esac

CONDA_BIN=${CONDA_EXE:-}
if [[ ! -x $CONDA_BIN ]]; then
    CONDA_BIN=$(type -P conda || true)
fi
if [[ ! -x $CONDA_BIN ]]; then
    for candidate in "$HOME/miniforge3/bin/conda" "$HOME/miniconda3/bin/conda" "$HOME/anaconda3/bin/conda"; do
        if [[ -x $candidate ]]; then CONDA_BIN=$candidate; break; fi
    done
fi
if [[ ! -x $CONDA_BIN ]]; then
    printf '%s\n' '먼저 Miniconda 또는 Miniforge를 설치하세요.' >&2
    exit 1
fi

# 기본 동작은 기존 환경 유지. 갱신은 --update를 명시한 경우에만 수행한다.
EXISTING_ENVS=$("$CONDA_BIN" env list)
for target in "${TARGETS[@]}"; do
    # 기존 CLI 대상(scan/planner/control)을 각 planning 환경에 연결한다.
    case "$target" in
        scan) env_name=vehicle_painting_robot_scan_planning; spec="$CONFIG_DIR/scan_planning.yaml" ;;
        planner) env_name=vehicle_painting_robot_path_planning; spec="$CONFIG_DIR/path_planning.yaml" ;;
        control) env_name=vehicle_painting_robot_motion_planning; spec="$CONFIG_DIR/motion_planning.yaml" ;;
    esac
    if [[ ! -f "$spec" ]]; then
        printf '[오류] 환경 정의 파일이 없습니다: %s\n' "$spec" >&2
        exit 1
    fi
    if awk -v name="$env_name" '$1 == name { found = 1 } END { exit !found }' <<< "$EXISTING_ENVS"; then
        if (( UPDATE )); then
            printf '\n[갱신] %s ← %s\n' "$env_name" "$spec"
            # --prune을 쓰지 않는다. YAML 밖의 사용자 설치 패키지를 일괄 제거하지 않는다.
            PYTHONNOUSERSITE=1 "$CONDA_BIN" env update --name "$env_name" --file "$spec"
            printf '[갱신 완료] conda activate %s\n' "$env_name"
        else
            printf '\n[건너뜀] %s: 이미 존재합니다. 패키지는 변경하지 않았습니다.\n' "$env_name"
            printf '갱신하려면: bash %q %s --update\n' "$SCRIPT_DIR/setup.sh" "$target"
        fi
    else
        # 설치 중에만 ~/.local 패키지를 제외한다. 영구 환경 변수/활성화 훅은 만들지 않는다.
        PYTHONNOUSERSITE=1 "$CONDA_BIN" env create --name "$env_name" --file "$spec"
        printf '\n[생성 완료] conda activate %s\n' "$env_name"
    fi
    if [[ "$target" == control ]]; then
        printf '\n%s\n' '[모션 플래닝] 아래 명령은 안내만 하며 자동 실행하지 않습니다.'
        printf '  파일 생성: conda activate %s\n' "$env_name"
        printf '  설정 확인: python -s %q --check\n' "$REPO_DIR/motion_planning/scripts/motion_planning.py"
        printf '  ROS 빌드(시스템 Humble 필요): bash %q\n' "$REPO_DIR/motion_planning/scripts/setup_ros2.sh"
        printf '  ROS 시각화 환경 활성화: source %q\n' "$REPO_DIR/motion_planning/scripts/activate_ros2.sh"
    fi
done
