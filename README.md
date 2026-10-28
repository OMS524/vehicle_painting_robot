# vehicle-painting-robot

## Installation

Ubuntu 22.04 / Linux x86_64 기준입니다. 스캔, 도장 경로 생성, 모션 플래닝은 서로 다른 Conda 환경을 사용합니다.
아래 명령은 별도 안내가 없으면 저장소 루트(`vehicle_painting_robot/`)에서 실행합니다.

### 1. 사전 준비 및 소스 다운로드

[Miniforge](https://github.com/conda-forge/miniforge#install) 또는 Miniconda를 먼저 설치하고,
터미널에서 `conda activate`를 사용할 수 있도록 설정합니다. 이미 설치되어 있다면 다시 설치할 필요가 없습니다.

```bash
sudo apt update
sudo apt install -y git build-essential cmake
```

처음 내려받는 경우:

```bash
git clone https://github.com/OMS524/vehicle_painting_robot.git vehicle_painting_robot
cd vehicle_painting_robot
```

이미 저장소가 있다면 해당 디렉토리로 이동한 후 두산 SDK 서브모듈을 준비합니다.

```bash
git submodule update --init --recursive -- robot_controller/doosan_a0912_controller/API-DRFL
```

SAM 3, EfficientSAM3, 커스텀 모델 등 테스트용 소스까지 필요하면
`git submodule update --init --recursive`로 전체 서브모듈을 내려받습니다.
모델 가중치와 각 모델의 실행 환경은 별도 준비 대상이며, 기본 DBSCAN 스캔 파이프라인에는 필요하지 않습니다.

### 2. Conda 환경 생성

```bash
bash miniforge/setup.sh
```

| 기능 | 생성되는 환경 | 패키지 설정 파일 |
| --- | --- | --- |
| 스캔·정합·클러스터링 | `vehicle_painting_robot_scan_planning` | [scan_planning.yaml](miniforge/config/scan_planning.yaml) |
| 도장 경로 생성 | `vehicle_painting_robot_path_planning` | [path_planning.yaml](miniforge/config/path_planning.yaml) |
| 모션 플래닝·로봇 제어 | `vehicle_painting_robot_motion_planning` | [motion_planning.yaml](miniforge/config/motion_planning.yaml) |

필요한 환경만 만들려면 다음 중 해당 명령만 실행합니다.

```bash
bash miniforge/setup.sh scan
bash miniforge/setup.sh planner
bash miniforge/setup.sh control
```

세 환경 모두 Python 3.10을 사용합니다. 주요 패키지 버전은 각 YAML에 정의되어 있습니다.
같은 이름의 환경이 이미 있으면 변경하지 않고 건너뜁니다. 기존 환경을 YAML 기준으로 갱신하려면
해당 환경의 프로그램을 종료한 후, 예를 들어 `bash miniforge/setup.sh control --update`를 실행합니다.

이 스크립트는 Conda 환경과 패키지만 설치합니다. ROS 설치·빌드, 제어 라이브러리 빌드,
카메라 권한 설정은 아래에서 별도로 진행합니다. SAM 3·EfficientSAM3·커스텀 모델 환경은 생성하지 않습니다.
자세한 내용은 [환경 설치 안내](miniforge/README.md)를 참고하세요.

### 3. 두산 제어 라이브러리 빌드

로봇을 이동시키는 스캔 또는 실제 로봇 제어에 필요합니다. 저장 데이터 후처리나 도장 경로 생성만 한다면 생략할 수 있습니다.

```bash
conda activate vehicle_painting_robot_motion_planning
cmake -S robot_controller/doosan_a0912_controller/test \
  -B robot_controller/doosan_a0912_controller/test/build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$CONDA_PREFIX" \
  -DDOOSAN_DRCF_VERSION=2
cmake --build robot_controller/doosan_a0912_controller/test/build -j2
```

결과는 `robot_controller/doosan_a0912_controller/test/build/libdoosan_controller_c_api.so`입니다.
위 명령은 빌드만 하며 로봇에 연결하거나 이동시키지 않습니다.
`DOOSAN_DRCF_VERSION=2`는 현재 프로젝트 설정이므로 실제 제어기의 DRCF 버전과 일치하는지 확인해야 합니다.
다른 위치에서 만든 `test/build`가 있다면 기존 빌드 폴더를 백업·이동한 뒤 다시 빌드합니다.

스캔은 별도 제어 프로세스를 실행하므로 [scan.yaml](scan_planning/config/scan.yaml)의
`control_python`을 **설치한 PC의 제어 환경 Python 절대 경로**로 수정해야 합니다.
현재 파일에는 개발 PC의 `/home/oms/miniforge3/...` 경로가 들어 있습니다.
제어 환경을 활성화한 상태에서 다음 명령으로 실제 경로를 확인할 수 있습니다.

```bash
python -c "import sys; print(sys.executable)"
```

`control_wrapper`, `control_library`는 같은 YAML에서 연결합니다.
상세 빌드 및 하드웨어 없는 라이브러리 검사는 [로봇 제어기 안내](robot_controller/doosan_a0912_controller/README.md)를 참고하세요.

### 4. Femto Bolt USB 접근 권한 설정

실제 카메라로 촬영할 PC에서 한 번 설정합니다. 카메라 Python SDK는 2단계 스캔 환경 설치에 포함됩니다.

```bash
conda activate vehicle_painting_robot_scan_planning
SDK_DIR="$(python -c 'import importlib.util, os; print(os.path.dirname(importlib.util.find_spec("pyorbbecsdk").origin))')"
python "$SDK_DIR/shared/setup_env.py"
```

SDK 설치 도구가 `sudo` 권한을 요청하고 Linux udev 규칙을 설치합니다.
설정 후에도 접근 권한이 적용되지 않으면 카메라 USB를 다시 연결합니다.
저장된 데이터만 처리하는 경우에는 이 단계를 생략할 수 있습니다.

### 5. ROS 2 시각화 패키지 빌드

모션 플래닝 결과를 RViz에서 확인할 때 필요합니다.
[ROS 2 Humble 공식 설치 안내](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)에 따라
시스템에 ROS 2 Humble Desktop을 먼저 설치합니다. 기본 설치 경로는 `/opt/ros/humble`입니다.

```bash
# 최초 빌드 또는 ROS 패키지 변경 후
bash motion_planning/scripts/setup_ros2.sh

# 이후 RViz를 사용할 터미널마다 실행
source motion_planning/scripts/activate_ros2.sh
```

첫 번째 명령은 `motion_planning/ros2_ws`의 `dsr_description2`, `paint_viz`를 빌드합니다.
두 번째 명령은 제어 Conda 환경 → 시스템 ROS → 해당 워크스페이스 순으로 활성화합니다.
`conda activate`만으로는 ROS 워크스페이스가 활성화되지 않습니다.
ROS 시각화를 사용하지 않는 스캔 후처리·도장 경로 생성·오프라인 모션 계산에는 이 단계가 필요하지 않습니다.

### 6. 실행 전 확인

다음은 설정 검사 또는 도움말 출력이며 로봇·카메라에 연결하지 않습니다.
전체 패키지 및 실장비 동작 검증을 대신하는 검사는 아닙니다.

```bash
conda activate vehicle_painting_robot_scan_planning
python -s scan_planning/scripts/scan_planning.py --check

conda activate vehicle_painting_robot_path_planning
python -s path_planning/scripts/path_planning.py --help

conda activate vehicle_painting_robot_motion_planning
python -s motion_planning/scripts/motion_planning.py --check
```

실제 촬영 전에는 로봇 IP, 목표 자세, 카메라 장착 TF/핸드아이 보정값과 이동 안전 조건을 별도로 확인해야 합니다.
저장소의 설정값은 개발 장비 기준이며 다른 장비에 그대로 적용하지 않습니다.

기능별 실행 방법은 [스캔](scan_planning/README.md), [도장 경로 생성](path_planning/README.md),
[모션 플래닝](motion_planning/README.md)을 참고하세요.
워크스테이션 앱의 개발·빌드는 [워크스테이션 안내](workstation/README.md)를 따릅니다.
앱 빌드는 위 설치 스크립트에 포함되지 않으며, 앱에서도 외부 planner 소스와 Python 환경이 필요합니다.

## Conda Activate
### Scan Planning
```bash
conda activate vehicle_painting_robot_scan_planning
```
```bash
conda activate vehicle_painting_robot_sam3
conda activate vehicle_painting_robot_efficientsam3
conda activate vehicle_painting_robot_custom_model
```
### Path Planning
```bash
conda activate vehicle_painting_robot_path_planning
```
### Motion Planning
```bash
conda activate vehicle_painting_robot_motion_planning
```
