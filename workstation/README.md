# Painting Process Workstation

차량 부품 스캔 데이터 불러오기, 도장 경로 생성, 최종 경로 코어 포인트 수정,
경로 재생성과 완료 데이터 저장을 하나의 3단계 공정으로 제공하는 Tauri
데스크톱 애플리케이션이다.

## 공정 흐름

1. **스캔**
   - 오른쪽 공정 패널에서 PLY 또는 XYZ CSV를 선택한다.
   - 파일은 현재 작업용 임시 세션에 복사되고 Three.js 뷰어에 표시된다.
2. **도장 경로 생성**
   - `painting_trajectory_planner/path_planner/config/painting_trajectory.yaml`의
     기본값을 불러와 화면에서 수정한다.
   - 도장 방향과 스캔 방향을 입력하고 기존 Python 경로 생성 알고리즘을 실행한다.
3. **도장 경로 수정**
   - 2단계에서 생성된 최종 경로의 분홍색 코어 포인트를 선택해 드래그하거나 Inspector에서 위치·회전을 입력한다.
   - `선택 포인트 삭제`는 선택한 점 하나만 제거한다. 각 경로에는 최소 2개의 편집 포인트가 남아야 한다.
   - `선택 경로 삭제`는 해당 Row 전체를 제거한다. 삭제 후 `경로 재생성`을 눌러 저장에 반영한다.
   - 포인트 삭제는 빈 구간을 만드는 기능이 아니다. 남은 점으로 spline을 만들고 설정된 최종 간격(기본 40 mm)으로 다시 샘플링하므로 비슷한 위치에 새 샘플점이 생길 수 있다.
   - 위치 이동은 해당 row의 슬라이싱 평면 위로 강제 투영된다.
   - `경로 재생성`은 수정된 코어 포인트를 spline 입력 row로 변환해 기존 spline·structuring 단계에 전달한다.
   - `완료 및 저장`은 완료 데이터를 planner의 `log` 하위에 저장한다.

## 구조

```text
workstation/
├── src/                         # React + TypeScript + Three.js UI
├── src-tauri/                   # 파일 스테이징과 Python 프로세스 실행
├── backend/
│   └── workstation_backend.py   # 기존 planner를 호출하는 외부 어댑터
└── standalone_html/             # 계약사 전달용 읽기 전용 단일 HTML

painting_trajectory_planner/
└── path_planner/
    ├── config/painting_trajectory.yaml
    ├── scripts/                 # 수정하지 않고 호출하는 기존 알고리즘
    └── log/<YYYYMMDD_HHMMSS>/   # 완료 결과
```

경로 생성 알고리즘은 Tauri 실행 파일에 포함하지 않는다. 앱은 외부의
`workstation_backend.py`를 실행하고, 이 어댑터가 현재 개발 중인 planner
Python 모듈을 직접 import한다. 따라서 알고리즘 수정 후 앱을 다시 빌드할
필요가 없다.

## 완료 저장 데이터

`완료 및 저장`을 누르면 다음 형식으로 저장한다.

```text
painting_trajectory_planner/path_planner/log/20260808_153012/
├── <scan-file>.ply 또는 .csv
├── painting_trajectory.csv
├── trajectory_000.csv
├── trajectory_001.csv
├── ...
├── painting_trajectory.yaml
├── edited_control_points.json
└── workstation_job.json
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
PAINTING_TRAJECTORY_PLANNER_ROOT=/absolute/path/to/path_planner
WORKSTATION_BACKEND_SCRIPT=/absolute/path/to/workstation_backend.py
PAINTING_WORKSTATION_PYTHON=/absolute/path/to/python3
```

## 애플리케이션 빌드

```bash
npm run tauri build
```

빌드 결과에는 프론트엔드와 Tauri 명령만 들어간다. 실행할 컴퓨터에는 별도로
planner 소스, backend 어댑터와 Python 실행 환경이 있어야 한다.

## 읽기 전용 단일 HTML

기존 계약사 전달용 HTML은 계속 생성할 수 있다.

```bash
python3 standalone_html/build_standalone.py
```

각 HTML에는 PLY·전체 trajectory CSV·YAML과 프론트엔드가 내장되며 서버 없이
열 수 있다. 단일 HTML 모드는 시각화 전용이고 경로 생성·수정·저장은 지원하지
않는다.
