#!/usr/bin/env bash
# 실행: bash /path/to/vehicle_painting_robot/conda/setup.sh [all|scan|planner|control]
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
usage() {
    printf '%s\n' '사용법: bash conda/setup.sh [all|scan|planner|control]'
}
if (( $# > 1 )); then
    usage >&2
    exit 2
fi
case "${1:-all}" in
    all) TARGETS=(scan planner control) ;;
    scan|planner|control) TARGETS=("$1") ;;
    -h|--help) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
esac

CONDA_BIN=${CONDA_EXE:-}
if [[ ! -x $CONDA_BIN ]]; then
    CONDA_BIN=$(type -P conda || true)
fi
if [[ ! -x $CONDA_BIN ]]; then
    for candidate in "$HOME/miniconda3/bin/conda" "$HOME/miniforge3/bin/conda" "$HOME/anaconda3/bin/conda"; do
        if [[ -x $candidate ]]; then CONDA_BIN=$candidate; break; fi
    done
fi
if [[ ! -x $CONDA_BIN ]]; then
    printf '%s\n' '먼저 Miniconda 또는 Miniforge를 설치하세요.' >&2
    exit 1
fi

# 기존 환경은 변경하지 않는다. 패키지 버전 일치 여부도 검사하지 않는다.
EXISTING_ENVS=$("$CONDA_BIN" env list)
for target in "${TARGETS[@]}"; do
    env_name="vehicle_painting_robot_${target}"
    if awk -v name="$env_name" '$1 == name { found = 1 } END { exit !found }' <<< "$EXISTING_ENVS"; then
        printf '\n[건너뜀] %s: 이미 존재합니다. 패키지는 변경하지 않았습니다.\n' "$env_name"
        continue
    fi
    # 설치 중에만 ~/.local 패키지를 제외해, 새 환경에 실제로 설치되게 한다.
    # 환경 활성화 설정이나 사용자 환경 변수는 변경하지 않는다.
    PYTHONNOUSERSITE=1 "$CONDA_BIN" env create --name "$env_name" --file "$SCRIPT_DIR/$target.yml"
    printf '\n[생성 완료] conda activate %s\n' "$env_name"
done
