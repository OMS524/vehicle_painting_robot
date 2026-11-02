# Painting Process Workstation

차량 부품 스캔 데이터 불러오기, 도장 경로 생성, 최종 경로 코어 포인트 수정,
경로 재생성·저장과 모션 배치 미리보기를 하나의 4단계 공정으로 제공하는 Tauri
데스크톱 애플리케이션이다.

## 화면 구성

- 앱 실행 시 **홈 화면**에서 시작한다. `공정 시작`으로 기존 4단계 작업 화면을 연다. 홈에서는 URDF·3D 뷰를 미리 생성하거나 공정 계산/장비 제어를 실행하지 않고, 기본 파라미터와 실행 환경 정보만 읽는다.
- 상단 `홈` 버튼으로 돌아오면 현재 단계·데이터·설정·3D 시점을 유지하며 `작업 이어서`로 복귀한다. 홈의 현재 작업 카드는 이 실행 세션의 데이터만 보여주며, 앱 종료 후의 자동 복구나 작업 이력 기능은 아니다. 생성/재생성/저장/모션 계산 중에는 홈 이동을 잠근다.
- 계약사 전달용 단일 HTML은 홈을 거치지 않고 읽기 전용 경로 검토 화면을 바로 연다.
- 2번 디자인(왼쪽 워크벤치)을 적용했다. 왼쪽부터 **세로 공정 목록 → 현재 공정 설정 → 넓은 3D 화면** 순서이며, 별도 오른쪽 사이드바나 상단 가로 공정 메뉴는 없다.
- 공정 목록은 `src/components/ProcessNavigation.tsx`로 분리했다. 현재는 기존 4단계의 화면 전환이며, 반복 블록 추가·재배열·자동 실행 및 실제 도장 명령은 아직 구현하지 않았다.
- 표시 항목·조명·배경·개발 환경도 현재 공정의 왼쪽 패널 안에 모았다. 스캔 프리셋은 왼쪽/오른쪽을 한 쌍씩 표시하며 저장된 데이터 순서와 좌표는 변경하지 않는다.
- 본문·버튼은 13–14 px, 보조 설명은 12 px로 맞추고 같은 단계의 실행 버튼은 패널 하단에 모았다. 패널 내용은 별도로 스크롤된다.
- 스캔의 배치 설정과 파일 불러오기는 별도 탭이며, 경로 준비도 `새로 생성` / `CSV 불러오기` 탭으로 나눴다.
- 주요 경로 파라미터는 바로 표시하고 나머지는 `고급 파라미터`에 보관한다. 값이나 알고리즘은 변경하지 않는다.
- 좌표계 상세 설명·저장 위치·개발 환경·모션 실행 로그는 접어서 표시한다. 화면상의 한글 선택 항목은 기존 YAML 값(`axis`, `surface_adaptive` 등)을 그대로 사용한다.

## 공정 흐름

1. **스캔**
   - 모션 단계와 같은 갠트리·로봇 URDF를 `world_base` 좌표계로 표시한다. 실제 로봇 자세 조회나 스캔 명령은 실행하지 않는다.
   - 왼쪽(+Y) 3곳 / 오른쪽(−Y) 3곳의 **부품 배치 프리셋**을 제공한다. 초기 위치는 X = −2000/0/2000 mm, Y = ±1200 mm, Z = 1200 mm이며, 현장 측정값이 아닌 설치 전 예시다.
   - 프리셋 또는 화면의 사각형을 선택해 부품 중심 XYZ(mm), 부품 RPY(deg), 카메라→부품 스캔 방향(world 벡터)을 수정한다. 현재 배치를 복사해 사용자 프리셋을 추가하거나 삭제할 수 있다.
   - 프리셋 아래 `사용자 설정`은 별도의 직접 입력 배치를 선택한다. 최초 선택 시 현재 배치의 값을 복사하며, 이후 선택하면 이전 입력값을 유지한다. XYZ(mm)·RPY(deg)·스캔 방향을 수정해도 기존 6개 프리셋에는 영향을 주지 않는다. 자동 보관 및 `배치 설정 파일 저장`에 함께 포함된다.
   - 사각형은 1200 × 900 mm 예시 부품이며, 카메라 마커는 방향 설명용 800 mm 이격 표시다. 카메라 TCP/실제 촬영 자세, IK·충돌 검증을 의미하지 않는다.
   - 배치는 앱의 로컬 저장소에 자동 보관되며 `배치 설정 파일 저장`은 `scan_planning/log/workstation_layout_<ID>/scan_layout.json`에 m/rad 단위로 저장한다. 기존 스캔 설정/실행 코드는 변경하지 않는다.
   - 배치 설정은 불러온 스캔 점군이나 도장 경로 좌표를 자동 변환하지 않는다. 실제 갠트리 스캔 제어와 좌표계 연결은 별도 단계다.
   - 왼쪽 설정 패널의 `스캔 파일` 탭에서 PLY 또는 XYZ CSV를 선택한다.
   - 파일은 현재 작업용 임시 세션에 복사되고 Three.js 뷰어에 표시된다.
   - `부품 배치` / `스캔 파일` 탭으로 두 화면을 전환한다.
2. **도장 경로 생성**
   - `path_planning/config/painting_path.yaml`의
     기본값을 불러와 화면에서 수정한다.
   - 도장 방향과 스캔 방향을 입력하고 기존 Python 경로 생성 알고리즘을 실행한다.
   - `CSV 불러오기` 탭의 `저장된 도장 경로 불러오기 (CSV)`로 전체 경로 `painting_trajectory.csv` 또는 같은 형식의 CSV를 선택할 수 있다. 스캔 없이도 2단계로 바로 이동해 불러올 수 있다.
   - CSV의 필수 열·유한값·쿼터니언·인덱스 순서를 검증한다. 파일을 재샘플링/정렬/수정하지 않고 그대로 시각화·저장한다. 실패한 불러오기는 이전 작업을 대체하지 않는다.
   - **CSV 단독 불러오기는 검토 모드**다. 원본 슬라이싱 평면 및 확장부/코어 구분이 없어 포인트 편집·재생성은 비활성화한다. 임의로 평면이나 원본 파라미터를 추정하지 않는다. 3단계에서 그대로 저장한 뒤 4단계 모션 입력으로 쓸 수 있다.
   - 불러온 CSV에는 화면의 생성 파라미터를 적용하지 않는다. 해당 파라미터는 `도장 경로 생성`을 실행할 때만 사용한다.
3. **도장 경로 수정**
   - 2단계에서 생성된 최종 경로의 분홍색 코어 포인트를 선택해 드래그하거나 Inspector에서 위치·회전을 입력한다.
   - `선택 포인트 삭제`는 선택한 점 하나만 제거한다. 각 경로에는 최소 2개의 편집 포인트가 남아야 한다.
   - `선택 경로 삭제`는 해당 Row 전체를 제거한다. 삭제 후 `경로 재생성`을 눌러 저장에 반영한다.
   - 포인트 삭제는 빈 구간을 만드는 기능이 아니다. 남은 점으로 spline을 만들고 설정된 최종 간격(기본 40 mm)으로 다시 샘플링하므로 비슷한 위치에 새 샘플점이 생길 수 있다.
   - 위치 이동은 해당 row의 슬라이싱 평면 위로 강제 투영된다.
   - `경로 재생성`은 수정된 코어 포인트를 spline 입력 row로 변환해 기존 spline·structuring 단계에 전달한다.
   - `완료 및 저장`은 완료 데이터를 planner의 `log` 하위에 저장한다.
   - **수정하지 않았으면 재생성이 필요 없다.** 포인트 선택, 동일한 값의 입력/포커스 이동, 같은 회전의 q/−q 표현은 수정으로 처리하지 않는다. 실제 위치·회전 변경/삭제가 있을 때만 재생성이 필요하다.
4. **모션 플래닝**
   - 3단계의 `완료 및 저장` → `다음: 모션 플래닝`으로 진입한다.
   - 갠트리·A0912·툴의 URDF visual 모델과 확정된 경로/분사 방향을 함께 표시한다. 부품 점군은 표시하지 않는다.
   - 진입 시 모델만 읽으며 최적화, RViz, 로봇 제어는 시작하지 않는다.
   - 경로 전체의 X/Y/Z 이동(mm), Roll/Pitch/Yaw 회전(deg)을 입력한다. 원본 경로 CSV는 수정하지 않는다.
   - `모션 생성`: 기존 외부 모션 플래너의 시간 프로파일 → 최적화 → 경로 간 전이를 실행한다. 부품 점군 없는 전이는 기존 `3_transition.py`의 **고정 후퇴 거리 모드**(`--scene` 미지정, 기본 30 mm)를 호출한다. `계산 중단`으로 이 앱이 시작한 계산을 중단할 수 있다.
   - 계산 성공 후 `RViz 열기`를 눌러 **별도 RViz 창**에서 재생한다. 앱의 미리보기 로봇은 계속 초기 자세다.
   - 배치/속도/가속도를 바꾸면 기존 결과는 무효화되어 다시 계산해야 한다. 경로를 수정하면 3단계에서 다시 저장한다.
   - 실행 로그는 4단계 패널, 실패 로그는 팝업에 표시한다. 앱 종료 시 앱이 시작한 계산/RViz 프로세스도 종료한다.

### 4단계 좌표계와 한계

- 기준은 URDF 루트 `world_base`, Z가 위쪽이다. RViz에서는 `world`와 `world_base` 사이에 항등 변환을 사용한다.
- 배치는 `p_cell = Rz(yaw) Ry(pitch) Rx(roll) p_csv + translation`이다. 회전 중심은 CSV의 원점이며 부품 중심으로 자동 이동하지 않는다. 위치·회전이 모두 0이면 원본 좌표 그대로이다.
- 화면 입력은 mm/deg, 플래너 입력은 m/rad이다. CSV quaternion은 xyzw를 유지하고, 분사 방향은 로컬 +Z이다.
- 현재 기본 모델은 `motion_planning/config/motion_planning.yaml`의 `gantry_env`가 가리키는 `x7y4z3` 환경이다. URDF의 각 joint origin/axis와 `nlp.initial_joint`를 사용한다. 실제 로봇 위치를 읽지 않는다.
- 기존 환경의 `part_placement.xyz_offset`과 부품별 override는 **작업용 복사본에서만** 제거한다. 화면에서 지정한 배치에 기존 오프셋이 다시 더해지지 않는다. 원본 모션 YAML·URDF·알고리즘은 변경하지 않는다.
- 미리보기 이후 설정/URDF가 바뀌면 계산을 거절한다. 패널의 `원본 · 모델 · 실행 환경` → `모델 / 설정 다시 불러오기`를 사용한다.
- 미리보기는 도달 가능성/충돌 판정이 아니다. 기본 실행 단계에 collision/sequence는 포함하지 않으며 부품 점군도 전달하지 않는다. **모션 생성 성공은 실제 로봇 실행 안전성을 보증하지 않는다.** 하드웨어 실행 기능은 없다.
- 공유 실행기는 전이 단계에서 부품 점군이 필요한 `--scene`을 항상 전달한다. 따라서 어댑터는 공유 실행기로 시간 프로파일·최적화를 수행하고, 동일한 기존 전이 모듈을 `--scene` 없이 별도로 호출한다. 전이 결과와 종료 코드는 같은 `run.json`과 `logs/`에 기록한다. 적응형 표면 후퇴·부품 충돌 회피를 수행한 것으로 해석하면 안 된다.
- 기존 플래너의 숫자 직렬화 정밀도(위치 6 유효숫자, RPY 7 유효숫자)를 적용한다. RViz의 scene YAML 반올림에 의한 미세한 차이는 남을 수 있다.

### 모션 작업 저장

`motion_planning/log/workstation_<고유 ID>/`에 요청값, 원본 전체 CSV 복사본,
전체 CSV에서 순서대로 분리한 row CSV, 작업용 YAML과 로그를 저장한다.
기존 개별 row CSV에 오래된 파일이 남아 있어도 사용하지 않는다.
실제 모션 결과는 그 아래 `results/<일시>/`에 기존 플래너 형식으로 생성된다.
RViz는 이 작업의 결과 디렉토리를 명시하여 열고, 다른 테스트 결과로 대체하지 않는다.
화면의 로그/오류 팝업은 각 로그 파일 마지막 24 KiB를 표시한다. 전체 로그는 해당 파일에 남는다.

## 구조

```text
workstation/
├── src/                         # React + TypeScript + Three.js UI
├── src-tauri/                   # 파일 스테이징과 Python 프로세스 실행
├── backend/
│   ├── workstation_backend.py   # 기존 path planner를 호출하는 외부 어댑터
│   └── motion_backend.py        # URDF 미리보기 / 기존 motion planner 연결
└── standalone_html/             # 계약사 전달용 읽기 전용 단일 HTML

path_planning/
├── config/painting_path.yaml
├── scripts/path_planning.py       # 상위 제어기
├── src/painting_path_planner/     # 단계별 알고리즘
└── log/<YYYYMMDD_HHMMSS>/          # 완료 결과
```

경로 생성 알고리즘은 Tauri 실행 파일에 포함하지 않는다. 앱은 외부의
`workstation_backend.py`를 실행하고, 이 어댑터가 현재 개발 중인 planner
Python 모듈을 직접 import한다. 따라서 알고리즘 수정 후 앱을 다시 빌드할
필요가 없다.

## 완료 저장 데이터

`완료 및 저장`을 누르면 다음 형식으로 저장한다.

```text
path_planning/log/20260808_153012/
├── <scan-file>.ply 또는 .csv
├── painting_trajectory.csv
├── trajectory_000.csv
├── trajectory_001.csv
├── ...
├── painting_trajectory.yaml
├── edited_control_points.json
└── workstation_job.json
```

CSV 단독 불러오기는 원본 전체 CSV, `imported_trajectory.json`, 빈 편집점 목록,
`workstation_job.json`을 저장한다. 스캔 파일은 선택했을 때만 포함하고, 없는 원본
파라미터나 개별 row CSV는 만들지 않는다. 4단계는 전체 CSV에서 row를 분리하므로 사용 가능하다.

## 하드웨어 없는 회귀 검사

```bash
# 저장소 루트
conda run -n vehicle_painting_robot_path_planning python -B -s -m unittest discover -s workstation/backend -p 'test_point_deletion.py'
conda run -n vehicle_painting_robot_path_planning python -B -s -m unittest discover -s workstation/backend -p 'test_trajectory_import.py'
conda run -n vehicle_painting_robot_motion_planning python -B -s -m unittest discover -s workstation/backend -p 'test_motion_backend.py'
cd workstation
node backend/test_frontend.mjs
# 로컬 Chrome + 빌드된 dist 사용. 임시 프로필/로컬 서버, Tauri 명령은 mock.
npm run build
node backend/test_ui.mjs
```

## 개발 실행

Python 환경에는 현재 planner와 동일하게 `numpy`, `scipy`, `PyYAML`,
`open3d`가 설치되어 있어야 한다.

```bash
cd /home/oms/vehicle_painting_robot/workstation
export PATH="$HOME/.local/node-v22.17.0-linux-x64/bin:$HOME/.cargo/bin:$PATH"
npm run tauri dev
```

`npm run dev`는 UI 확인용 브라우저 미리보기만 제공한다. 브라우저는 로컬
Python 프로세스를 직접 실행할 수 없으므로 경로 생성·재생성·완료 저장은
Tauri 앱에서만 동작한다.

## 외부 경로 설정

기본 디렉터리 배치와 다를 경우 다음 환경변수로 연결 위치를 지정한다.

```bash
PAINTING_TRAJECTORY_PLANNER_ROOT=/absolute/path/to/path_planning
WORKSTATION_BACKEND_SCRIPT=/absolute/path/to/workstation_backend.py
PAINTING_WORKSTATION_PYTHON=/absolute/path/to/python3
PAINTING_MOTION_PLANNER_ROOT=/absolute/path/to/motion_planning
PAINTING_WORKSTATION_MOTION_PYTHON=/absolute/path/to/motion/python
```

일반 설치에서는 Miniforge/Miniconda/Anaconda 아래의
`vehicle_painting_robot_path_planning`, `vehicle_painting_robot_motion_planning`
Python을 각각 자동 탐색한다. 앱 더블클릭 시에도 두 환경을 구분해서 사용한다.
모션 미리보기에는 ROS 실행이 필요 없지만 모션 환경의 PyYAML과 외부 모델 파일은 필요하다.
`RViz 열기`는 현재 Linux + ROS 2 Humble용이며 `motion_planning/ros2_ws`를 먼저
빌드해야 한다. 앱이 `activate_ros2.sh`로 해당 모션 Conda/ROS 환경을 준비한다.

하드웨어 없는 어댑터 검증:

```bash
conda activate vehicle_painting_robot_motion_planning
python -s -m unittest discover -s workstation/backend -p 'test_motion*.py' -v
```

## 애플리케이션 빌드

```bash
npm run tauri build
```

빌드 결과에는 프론트엔드와 Tauri 명령만 들어간다. 실행할 컴퓨터에는 별도로
planner 소스, backend 어댑터와 Python 실행 환경이 있어야 한다.

2026-10-28 폴더 재구성 이후 빌드한 앱은 `path_planning`을 탐색한다.
이전 실행 파일에는 이전 폴더명이 들어 있으므로 갱신된 AppImage를 사용한다.
기존 작업 세션·계약사 전달 데이터의 `painting_trajectory.yaml` 파일 이름은
그대로 유지하며, 앱 기본 설정은 새 `config/painting_path.yaml`에서 읽는다.

## 읽기 전용 단일 HTML

기존 계약사 전달용 HTML은 계속 생성할 수 있다.

```bash
python3 standalone_html/build_standalone.py
```

각 HTML에는 PLY·전체 trajectory CSV·YAML과 프론트엔드가 내장되며 서버 없이
열 수 있다. 단일 HTML 모드는 시각화 전용이고 경로 생성·수정·저장은 지원하지
않는다.
