# Motion planning

`path_planning`에서 생성한 TCP 경로 CSV를 받아 관절 궤적과 행 사이 전이, 툴 교환 시퀀스를 생성한다.
계산은 Python으로 수행하고, ROS 2 Humble은 RViz 시각화에 사용한다.
실제 장비 제어 인터페이스는 `robot_controller`에 둔다.

## 디렉토리

```text
vehicle_painting_robot/
├── motion_planning/
│   ├── config/
│   │   ├── motion_planning.yaml       # 실행 대상·입출력·기존 실행 인자
│   │   ├── gantry/                    # legacy / spec_c / x7y4z3
│   │   ├── parts/                     # 부품 배치·점군·부품별 실행 인자
│   │   └── sequence/                  # 툴 교환·도장 순서
│   ├── scripts/
│   │   ├── motion_planning.py         # 상위 실행기, 기본은 검사만
│   │   ├── visualize_motion.py        # 저장된 결과 RViz 재생
│   │   ├── setup_ros2.sh              # ROS 패키지 빌드
│   │   └── activate_ros2.sh           # 제어 Conda + ROS 활성화
│   ├── src/motion_planner/
│   │   ├── 1_time_profile.py          # TCP 경로에 시간 프로파일 추가
│   │   ├── 2_optimization.py          # 기존 CasADi/Ipopt NLP
│   │   ├── 3_transition.py            # 도장 행 사이 전이 생성
│   │   ├── 4_sequence.py              # 홈·툴 장착·도장·반납 통합
│   │   ├── collision.py              # 샘플 기반 충돌 검사
│   │   ├── kinematics.py / dp_ik.py   # 공용 IK / 기존 DP IK
│   │   ├── configuration.py          # 설정과 파일 위치 해석
│   │   └── …                         # 도킹·데이터 IO·모델 생성 도구
│   ├── ros2_ws/src/
│   │   ├── paint_viz/                # ROS 노드·launch·RViz, 호환 진입점
│   │   └── dsr_description2/         # 로봇 메시 ROS 패키지
│   ├── test/
│   │   ├── input/                    # 전달받은 시간 프로파일 포함 입력
│   │   ├── legacy_nlp4cm/            # 기존 실험 스크립트·분석 자료 보존
│   │   └── check_restructure.py      # 백업 대비 알고리즘·데이터 보존 검사
│   ├── log/                         # 새 실행은 날짜_시간별 저장
│   │   └── imported_x7y4z3/          # 전달받은 기존 결과, 덮어쓰지 않음
│   └── backup/20261028_before_directory_restructure/
├── description/paint_cell/
│   ├── urdf/                        # 갠트리·툴 URDF 및 STL
│   └── a0912_meshes/                 # A0912 원본 메시
├── robot_controller/                   # 장비 SDK·실제 제어
└── miniforge/config/motion_planning.yaml  # vehicle_painting_robot_control 환경
```

`paint_viz`의 `config`, `urdf`, `motion_planner`와 `dsr_description2/meshes`는
실제 관리 위치를 가리키는 상대 심볼릭 링크다. 중복 소스를 따로 수정하지 않는다.
ROS 빌드 시 패키지에 코드와 자산을 설치하므로 기존 `package://paint_viz/...`,
`package://dsr_description2/...` 메시 URI를 유지한다.
숫자 파일은 `motion_planner.time_profile`, `.optimization`, `.transition`, `.sequence`라는 이름으로 import할 수 있다.

## 환경과 실행

아래 명령은 저장소 루트에서 실행한다. 환경 이름은 `vehicle_painting_robot_control`이다.

```bash
conda activate vehicle_painting_robot_control

# 입력·설정·실행 명령 확인: 계산/저장/로봇 연결을 하지 않음
python -s motion_planning/scripts/motion_planning.py --check

# 후드: 시간 프로파일 → NLP → 행 사이 전이
python -s motion_planning/scripts/motion_planning.py --run --part ES300h_hood

# config에 지정된 전체 부품에 위 3단계 실행
python -s motion_planning/scripts/motion_planning.py --run

# 충돌 검사·툴 교환 시퀀스까지 추가
python -s motion_planning/scripts/motion_planning.py --run --part ES300h_hood --stage all
```

`--part`는 여러 번 지정할 수 있다. 눕힌 펜더는 `--part NX350h_right_fender_flat`이다.
기본 입력은 `path_planning/test/csv/<part>/trajectory_NNN.csv`다.
해당 입력이 없으면 `test/input/<part>`의 기존 시간 프로파일 포함 CSV를 그대로 사용한다.
다른 데이터는 `--part <설정된 부품> --input-dir /path/to/row_csv`로 지정한다.
새 부품은 `config/parts/scene_<part>.yaml`에 점군 경로와 부품 배치를 먼저 정의한다.

이 실행기는 `--execute`를 허용하지 않는다. 모션 파일 생성과 RViz 재생은 실제 로봇을 움직이지 않는다.
기존 NLP의 `--execute` 호환 기능은 `robot_controller`의 기존 진입점에 남겼으며,
원래의 **6축 모델만 실행하는 제한**을 유지한다. 8축 갠트리 실기 제어를 새로 연결한 것은 아니다.

새 결과는 `log/YYYYMMDD_HHMMSS_microseconds/`에 저장된다.
`input`, `optimization`, `transition`, `sequence` 아래는 부품별 CSV,
`config`는 사용 설정 사본, `scenes`는 시각화용 배치, `logs`는 단계별 표준출력/오류,
`run.json`은 실행 명령과 종료 상태다. 충돌 검사 출력도 `logs/<part>_collision.log`에 저장된다.
수렴 실패 등 단계가 오류로 끝나면 해당 로그를 남기고 중단한다. 한 행뿐이면 전이 단계는 기존처럼 건너뛴다.

기존 결과를 입력으로 후속 단계만 실행할 수도 있다.

```bash
python -s motion_planning/scripts/motion_planning.py --run --part ES300h_hood \
  --stage transition --source-run motion_planning/log/imported_x7y4z3
```

`--stage`는 `time_profile`, `optimization`, `transition`, `collision`, `sequence`, `all`을 지원한다.
`--source-run`의 결과는 읽기만 하고, 새 결과는 별도의 날짜 디렉토리에 저장한다.

## ROS 빌드와 시각화

ROS 2 Humble이 `/opt/ros/humble`에 설치된 Ubuntu 환경 기준이다.
순수 파일 생성에는 ROS 빌드가 필요하지 않다.

```bash
# 최초 1회 또는 ROS 노드/launch 변경 시
bash motion_planning/scripts/setup_ros2.sh

# 시각화를 사용할 터미널에서
source motion_planning/scripts/activate_ros2.sh

# 전달받은 기존 결과 재생
python -s motion_planning/scripts/visualize_motion.py --part ES300h_hood

# 새 결과 재생: 실제 실행 로그 디렉토리로 변경
python -s motion_planning/scripts/visualize_motion.py \
  --run-dir motion_planning/log/<실행일시> --part ES300h_hood

# 툴 교환 포함 시퀀스
python -s motion_planning/scripts/visualize_motion.py --part ES300h_hood --stage sequence
```

`visualize_motion.py --check`는 GUI를 열지 않고 경로와 launch 명령만 확인한다.
시각화 실행기는 기존처럼 현재 Conda Python으로 ROS CLI도 실행한다.
일반 `ros2 launch`는 시스템 Python을 사용하지만, 설정을 읽을 때 최적화 모듈을 자동으로
불러오지 않도록 분리했으므로 CasADi/Pinocchio 없이도 시각화 launch 파일을 읽을 수 있다.
실제 최적화·IK·전이 계산은 해당 기능을 사용할 때 모듈을 로딩하며, 계속 제어 Conda 환경이 필요하다.
`--stage auto`는 기존 `play.sh`처럼 NLP보다 전이 결과가 최신이면 전이를 재생한다.
`setup_ros2.sh`는 환경을 생성/업그레이드하지 않으며 해당 워크스페이스의 두 ROS 패키지만 빌드한다.
환경 생성은 `bash miniforge/setup.sh control`이며, 이미 있는 환경은 이 명령으로 갱신되지 않는다.
설치에 사용하는 환경 정의는 `miniforge/config/motion_planning.yaml`이다.

## 이번 정리에서 유지한 것

- 시간 프로파일, NLP 목적함수/제약/가중치, IK, 전이, 도킹, 시퀀스 계산과 기본값을 유지했다.
- 기존 부품 배치, TCP 프레임, 갠트리 설정, URDF/메시 형상·TF를 유지했다.
- `paint_cell` 원본과 기존 스캔/경로 생성 알고리즘은 수정하지 않았다.
- 기존 ROS 명령(`stitch_transitions`, `build_sequence`, `check_collisions` 등)은 호환 진입점으로 유지했다.
- `robot_controller/.../test/scripts/trajectory_nlp.py`는 새 NLP 위치를 호출하는 호환 파일이다.
  기존 실제 장비 실행 함수는 같은 디렉토리의 `trajectory_execution.py`로 그대로 분리했다.

백업에는 정리 전 `motion_planning` 전체, 제어 scripts, 공통 description, miniforge가 들어 있다.
기존 결과는 `log/imported_x7y4z3`로 이동했으며 파일 내용과 과거 메타데이터의 경로 문자열도 보존했다.
과거 로그에 적힌 옛 경로는 현재 실행 위치를 뜻하지 않는다.
기존 설명과 알려진 수렴/가속도 한계는 [정리 전 README](backup/20261028_before_directory_restructure/motion_planning/README.md)에 남아 있다.
구조 정리가 기존 최적화의 실패 행이나 물리적 안전성을 해결했다는 의미는 아니다.

```bash
# 하드웨어 연결 없는 원본 보존 검사
python -B -s motion_planning/test/check_restructure.py

# 설정 로딩의 수치 라이브러리 분리 및 기존 단계 import 호환 검사
python -B -s motion_planning/test/test_lazy_imports.py
```

검사 범위: 계산 코드/기본값 AST 비교, 설정값 비교, 메시·입력·기존 결과 847개 파일 SHA-256 비교,
ROS 노드 로직과 경로 확인. 실제 계산 비교 로그는 `log/structure_verification`에 저장한다.
전체 부품 재계산이나 실기 동작 검증은 별도다.
