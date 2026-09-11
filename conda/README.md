# 스캔 · 도장 경로 생성 · 로봇 제어 환경

세 기능을 별도 Conda 환경으로 설치한다. Miniconda 또는 Miniforge가 설치된
Ubuntu 22.04 / Linux x86_64 기준이며, sudo 없이 실행한다.

| 기능 | 환경 이름 | 패키지 정의 |
| --- | --- | --- |
| 스캔·3D 재구성·Xacro 뷰어 | `vehicle_painting_robot_scan` | [scan.yml](scan.yml) |
| 도장 경로 생성·CSV/HTML 디버깅 | `vehicle_painting_robot_planner` | [planner.yml](planner.yml) |
| 두산 로봇 제어·기구학·궤적 최적화 | `vehicle_painting_robot_control` | [control.yml](control.yml) |

## 설치

세 환경을 한 번에 생성한다.

```bash
bash ~/vehicle_painting_robot/conda/setup.sh
```

하나만 생성하려면 다음 중 하나를 실행한다.

```bash
bash ~/vehicle_painting_robot/conda/setup.sh scan
bash ~/vehicle_painting_robot/conda/setup.sh planner
bash ~/vehicle_painting_robot/conda/setup.sh control
```

같은 이름의 환경이 이미 있으면 해당 환경만 건너뛰고 나머지를 생성한다.
건너뛴 환경의 패키지를 갱신하거나, YAML과 버전이 같은지 검사하지는 않는다.
기존 통합 환경 `vehicle_painting_robot`도 변경하거나 삭제하지 않는다.
설치 명령에만 `PYTHONNOUSERSITE=1`을 적용해 `~/.local`의 패키지를 이미 설치된 것으로 오인하지 않게 한다.
이 설정은 Conda 환경에 저장하거나 활성화 훅으로 등록하지 않는다.

패키지 버전은 각 YAML에서 변경한다. Python은 세 환경 모두 3.10이며,
주요 패키지 버전을 고정하되 모든 전이 의존성까지 고정한 lock 파일은 아니다.
스캔·경로 생성은 pip wheel을 사용하고, 제어용 Pinocchio/CasADi/Eigen은 Conda-forge에서 설치한다.
스캔·경로 생성 환경에는 Pinocchio를 넣지 않고, 제어 환경에는 Orbbec SDK/Open3D를 넣지 않는다.

## 사용

실행할 기능에 맞는 환경 하나를 활성화한 후 일반 `python 파일.py`로 실행한다.

```bash
# 스캔 / Xacro 뷰어
conda activate vehicle_painting_robot_scan
python ~/vehicle_painting_robot/scan/doosan_a0912_scan_test/scripts/visualize_xacro.py

# 도장 경로 생성: 실행 전 test.py의 CASES와 파라미터를 확인한다.
conda activate vehicle_painting_robot_planner
python ~/vehicle_painting_robot/painting_trajectory_planner/path_planner/test/scripts/test.py

# 로봇 제어: 제어 .so 재빌드 및 장비 실행 조건을 확인한 다음 사용한다.
conda activate vehicle_painting_robot_control
```

`setup.sh`는 환경 생성과 패키지 설치만 한다.
SDK/Open3D 바이너리 수정, 시스템 라이브러리 교체, `LD_LIBRARY_PATH`/`LD_PRELOAD` 설정,
활성화 훅, 제어 라이브러리 자동 빌드·백업, 하드웨어 검사를 하지 않는다.
기존 SDK 바이너리 수정용 `configure_native_runtime.py`와 통합 `environment.yml`은 제거했다.

## 기존 환경을 다시 만들려면

해당 환경을 사용하는 프로그램을 종료한 다음, 재설치할 환경만 삭제한다.
예를 들어 기존 스캔 환경을 새 `scan.yml` 구성으로 바꾸려면:

```bash
conda deactivate
conda env remove --name vehicle_painting_robot_scan
bash ~/vehicle_painting_robot/conda/setup.sh scan
```

이전 통합 환경은 필요 없어졌을 때 사용자가 별도로 삭제한다.
단, 기존 제어 `.so`가 통합 환경의 라이브러리를 참조하므로 아래 재빌드 주의사항을 먼저 확인한다.

```bash
conda deactivate
conda env remove --name vehicle_painting_robot
```

환경 삭제는 프로젝트 소스나 스캔 데이터, 프로젝트의 제어 `.so` 파일을 삭제하지 않는다.
환경에 `conda env config vars set`으로 저장했던 설정은 환경 삭제와 함께 제거된다.
별도로 `.bashrc` 등에 추가했던 라이브러리 경로 설정은 자동으로 제거하지 않는다.
기존 ROS `PYTHONPATH`나 사용자 site-packages 설정도 바꾸지 않는다. 따라서 일반 실행에서는
외부에 설치된 같은 이름의 모듈이 먼저 선택될 수 있다. 현재 터미널에서는 Xacro가 ROS 경로,
Plotly가 `~/.local` 경로에서 로딩되는 경우를 확인했으며, 각 새 환경 내부에도 정의된 버전은 설치된다.

## 제어 라이브러리와 프로세스 분리 주의사항

- **제어 `.so` 재빌드가 별도로 필요하다.** 현재 빌드된
  `robot_control/doosan_a0912_controller/test/build/libdoosan_controller_c_api.so`는
  이전 `vehicle_painting_robot/lib`를 참조한다. 새 제어 환경을 활성화하고,
  이전 환경을 참조하는 CMake 캐시를 제거하거나 새 빌드 디렉토리를 사용해 다시 빌드해야 한다.
  기존 `CMakeLists.txt`는 활성화한 Conda 환경을 우선 탐색하고, 빌드에 사용한 Pinocchio 경로를 `.so`에 기록한다.
  새 빌드 디렉토리를 쓴다면 Python 래퍼의 `LIB_PATH`와 실제 `.so` 위치도 일치시켜야 한다.
- **가상환경 생성만으로 스캔·제어 통신이 구현되지는 않는다.** 스캔 Python에서 기존
  `DoosanRobot` 래퍼를 직접 사용하면 제어 `.so`도 스캔 프로세스에 로딩되어 분리 목적에 맞지 않는다.
  제어 환경의 Python으로 별도 제어 프로세스를 실행하고 이동 요청·완료·로봇 자세를 주고받는 연결 코드는 후속 작업이다.
- 이번 변경은 `conda/` 설치 구성만 대상으로 하며, 경로 생성 알고리즘·로봇 제어 로직·워크스테이션 실행 설정은 변경하지 않는다.

설치 방식 참고: [Conda 환경 관리](https://docs.conda.io/projects/conda/en/stable/user-guide/tasks/manage-environments.html#using-pip-in-an-environment),
[Orbbec Python SDK 의존성](https://github.com/orbbec/pyorbbecsdk/blob/v2-main/pyproject.toml).
