# A0912 + Femto Bolt 스캔 테스트

## 정면·왼쪽·오른쪽 촬영 및 통합

`three_view_scan.py`가 정면 → 왼쪽 → 오른쪽 순서로 이동하고, **목표 도달과
정지 유지가 확인된 뒤** 각 위치에서 Depth 1장을 저장한다. 마지막에는 실제 관절각과
Xacro TF로 세 점군을 `base_link` 좌표계에 통합한다. ROS는 사용하지 않는다.

- `three_view_scan.py`: scan 환경에서 카메라 취득, FK/TF 계산, 저장, 점군 통합.
- `three_view_scan.yaml`: 촬영 관절각, IP, 속도, 카메라 범위, 저장 위치 등의 설정.
- `robot_scan_worker.py`: control 환경의 별도 프로세스. 기존 Python 래퍼로 이동/실제 관절 상태 조회.
- `test_three_view_scan.py`: 합성 입력만 사용하는 오프라인 테스트.

### 1. 설정 입력

`three_view_scan.yaml`에서 아래 항목을 먼저 수정한다. 상대 파일 경로는 YAML 위치 기준이다.

1. `views`의 `front`, `left`, `right` 각각에 **펜던트로 안전을 확인한 J1~J6 관절각(deg)**을
   `joint_deg: [J1, J2, J3, J4, J5, J6]`로 입력한다. `null` 상태에서는 로봇에 연결하지 않는다.
   촬영 위치/시선 방향을 자동으로 계산하지 않는다.
2. `robot.ip`, 필요하면 포트, 이동 속도와 허용 이동량을 확인한다.
3. 실제 로봇/브라켓/카메라와 Xacro의 원점·축·장착 TF가 일치하는지 확인한 뒤에만
   `mounting_tf_confirmed: true`로 변경한다. 현재 Xacro에는 mesh 정렬 기반 장착값과
   카메라 내부 **nominal TF**가 들어 있으며 hand-eye 캘리브레이션 결과가 아니다.
4. `camera.min_depth_mm`, `max_depth_mm`를 대상 거리에 맞춘다. 배경/로봇 점이 포함되면
   `reconstruction.roi_min_mm`, `roi_max_mm`에 **base_link 기준** 관심 영역을 지정한다.
   여러 카메라가 연결되어 있으면 `serial_number`도 지정한다.

`max_move_delta_deg`는 현재 자세와 다음 목표 사이의 관절별 최대 변화 제한이다.
자동 충돌 회피/이동 경로 검증이 아니므로 **초기 위치 → 정면 → 왼쪽 → 오른쪽의 전체 이동 구간**,
툴 무게/무게중심 설정, 케이블, 작업자와 주변 장애물, 비상정지 접근성을 직접 확인해야 한다.
세 촬영에는 같은 부품의 겹치는 영역이 보이도록 하고, 촬영 중 대상과 로봇 베이스는 고정한다.

### 2. 실행

```bash
conda activate vehicle_painting_robot_scan
cd /home/oms/vehicle_painting_robot/scan/doosan_a0912_scan_test/scripts

# 하드웨어 연결 없이 파일/TF/환경/.so만 검사. 기본 실행도 검사만 한다.
python three_view_scan.py --check

# 실제 로봇 이동 + Depth 취득 + 통합/저장. 터미널에 SCAN 입력 시 시작한다.
python three_view_scan.py --execute
```

`--execute`는 로봇 제어권/서보를 활성화한다. 성공·실패·Ctrl+C 시 기존 제어기의
`shutdown()`(servo-off 및 연결 종료)을 호출한다. **네트워크/프로세스 장애 시 소프트웨어 종료가
물리적 정지를 보장하지는 않는다.** 오류가 나면 현장에서 정지 여부를 확인한다.
기본 흐름에는 별도의 복귀 이동을 넣지 않았으므로 정상 완료 시 오른쪽 촬영 위치에서 종료한다.

scan/control 환경은 **한 프로세스에서 섞지 않는다.** `control_python`에 지정한 Python을
자식 프로세스로 실행하고, JSON-lines로 요청/응답한다. 자식에게 상속되는 Python/네이티브
경로 오버라이드만 제거하며, Conda 전역 변수나 카메라 SDK 파일은 수정하지 않는다.

### 3. 저장 결과

기본 저장 위치는 `../log/YYYYMMDD_HHMMSS_ffffff/`다. 매번 새 디렉토리를 만들고
이전에 촬영한 결과는 덮어쓰지 않는다.

```text
20260911_153000_123456/
├── settings.json          # 실행 설정(해석된 절대 경로 포함)
├── model.urdf             # 실행 시 Xacro를 확장한 TF 모델 스냅샷
├── manifest.json          # 시점 순서/진행 상태/개수/환경 버전/오류
├── robot_worker.log       # Python 래퍼 및 C++ 제어 로그
├── front/                 # left/, right/도 동일
│   ├── depth_raw.npy      # 원본 uint16 Depth. mm = raw × camera.json의 scale
│   ├── camera.json        # 내참수/왜곡/해상도/단위/시리얼/타임스탬프
│   ├── pose.json          # 촬영 전후 실제 관절각·속도·TF, accepted 여부
│   ├── camera_raw.ply     # 원본 유효 점군: depth optical 좌표계, m
│   ├── camera_filtered.ply # 거리/ROI 범위 내 점군: depth optical 좌표계, m
│   └── base.ply           # 동일 점군: 로봇 base_link 좌표계, m
├── merged_raw.ply         # 세 base.ply를 그대로 합친 점군
└── merged.ply             # 통합 점군을 voxel_size_mm으로 다운샘플링
```

점군 색은 실제 RGB가 아니라 **front=빨강, left=초록, right=파랑** 구분색이다.
현재 테스트는 Depth 전용이며 RGB 취득, 삼각형 메쉬 생성, TSDF, ICP는 적용하지 않는다.
따라서 여기서 3D reconstruction 결과는 **TF 기반 통합 포인트클라우드**다.
TF/깊이 오차로 세 면이 겹치지 않으면 그대로 표시하므로 초기 장착 보정을 확인하기 좋다.

오류가 나면 다음 촬영 위치로 진행하지 않으며, 완료된 시점 파일과 `error.log`를 남긴다.
`manifest.json`의 `status: complete` 여부를 확인한다. 정지/자세 조회 검증을 통과하지 못한
Depth는 보존하되 `pose.json`의 `accepted: false`로 기록하고 통합에는 사용하지 않는다.

```bash
# 저장한 결과만 보기. 로봇/카메라 연결 없음.
python three_view_scan.py --view ../log/촬영디렉토리/merged.ply
```

### TF 및 촬영 시점 기준

```text
Depth 광학 좌표계 점(m)
  → T_flange_depth (Xacro 고정 TF)
  → T_base_flange (촬영 직후 실제 관절각으로 URDF FK)
  → base_link 좌표계 점(m)
```

SDK의 native Depth를 사용하므로 color 프레임으로 정렬하지 않는다. Depth 광학 좌표축은
X=오른쪽, Y=아래, Z=렌즈 전방이다. PointCloudFilter의 출력은 프레임에 저장된
`get_position_value_scale()`을 곱해 mm로 변환하고, 다시 1000으로 나눠 m로 저장한다.
Depth raw scale을 점군에 중복해서 적용하지 않는다.

카메라 스트림은 각 이동/정지 확인 후 새로 시작하고 `warmup_frames`개를 버린다.
촬영 전후 실제 관절각/속도를 확인하며, 허용 범위보다 움직였으면 그 촬영은 거부한다.
이는 **정지 촬영 방식**이고 카메라-로봇 하드웨어 동기화나 이동 중 스캔이 아니다.
`T_base_flange`는 목표 자세나 펜던트의 사용자 TCP 값이 아니라 **실제 관절각의 FK 계산값**이다.
관절 상태 두 조회와 카메라 타임스탬프는 별도 저장하며 원자적인 동시 측정으로 취급하지 않는다.

단위/광학 좌표계/카메라 보정 처리 참고:
[Orbbec PointCloudFilter 구현](https://github.com/orbbec/OrbbecSDK_v2/blob/main/src/filter/publicfilters/PointCloudProcess.cpp),
[PointsFrame 좌표 스케일 정의](https://github.com/orbbec/OrbbecSDK_v2/blob/main/include/libobsensor/hpp/Frame.hpp),
[Python SDK](https://github.com/orbbec/pyorbbecsdk/tree/v2-main).

### 제어 라이브러리와 오프라인 검사

실제 관절각/속도 조회를 위해 기존 제어 래퍼에 읽기 전용 API를 추가했다.
기존 이동 알고리즘은 변경하지 않았고, 기존 `build/`를 보존하며 별도 `build-scan/`에 빌드한다.
제어 환경을 다른 경로에 다시 설치하거나 제어 코드를 수정했을 때는 다음을 실행한다.

```bash
conda activate vehicle_painting_robot_control
cd /home/oms/vehicle_painting_robot/robot_control/doosan_a0912_controller/test
cmake -S . -B build-scan -DCMAKE_BUILD_TYPE=Release -DDOOSAN_DRCF_VERSION=2 -DCMAKE_PREFIX_PATH="$CONDA_PREFIX"
cmake --build build-scan --target doosan_controller_c_api -j2

conda activate vehicle_painting_robot_scan
cd /home/oms/vehicle_painting_robot/scan/doosan_a0912_scan_test/scripts
python -B -m unittest test_three_view_scan -v
```

테스트는 합성 로봇/Depth를 사용한다. SDK 테스트도 실제 장치 대신 메모리에 만든 Depth를
점군으로 변환하여 단위/축 방향을 확인하므로 로봇/카메라 연결이나 동작을 하지 않는다.

## A0912 + 스캔 툴 Xacro 시각화

```bash
conda activate vehicle_painting_robot_scan
python /home/oms/vehicle_painting_robot/scan/doosan_a0912_scan_test/scripts/visualize_xacro.py
```

기본 입력은 `../description/xacro/a0912_to_tool_scan.xacro`다. 로봇 베이스는
`world`에 고정되고, 회전 관절은 기본 0도로 표시한다. 브라켓은 파란색,
카메라는 원본 Xacro의 밝은 회색으로 표시한다. 마우스로 회전/확대하고,
오른쪽 목록에서 메쉬와 좌표계를 개별적으로 켜고 끌 수 있다.
좌표축 색상은 X=빨강, Y=초록, Z=파랑이다.

## 사용자 파라미터

`visualize_xacro.py` 상단에서 수정한다.

- `XACRO_PATH`: 표시할 Xacro/URDF 파일. 상대 경로는 `scripts` 기준.
- `JOINT_ANGLES_DEG`: 관절 이름별 회전 각도(deg). 미지정 관절은 0도.
- `FRAME_SIZE_MM`: 좌표축 표시 길이(mm).
- `FRAME_NAMES`: 표시할 링크 이름. `None`이면 전체 링크 좌표계 표시.
- `BACKGROUND_COLOR`, `WINDOW_WIDTH`, `WINDOW_HEIGHT`: 배경색과 창 크기.

명령줄에서 경로를 지정해도 된다.

```bash
python visualize_xacro.py --xacro ../description/xacro/tool_scan.xacro
python visualize_xacro.py --joint joint_2=-30 --joint joint_3=60
python visualize_xacro.py --frame-size-mm 35
python visualize_xacro.py --no-frames
python visualize_xacro.py --check
python visualize_xacro.py --check --joint joint_2=-30 --joint joint_3=60
```

`--check`는 창을 열지 않고 실제 메쉬와 링크 연결을 읽어 TF를 검증한다.
루트 좌표계에서 본 각 링크의 원점 위치는 터미널에 mm로 출력한다.
`FRAME_NAMES`에 없는 링크를 보려면 해당 이름을 추가하거나 `None`으로 변경한다.
`--joint`는 같은 관절의 `JOINT_ANGLES_DEG` 설정을 덮어쓴다. 관절 이름 오타,
유효하지 않은 각도, URDF의 회전 범위를 벗어난 입력은 오류로 알려준다.
각도는 정적인 시각화 자세만 결정하며 실제 로봇에는 명령을 보내지 않는다.

## 조립 좌표계와 단위

`a0912_to_tool_scan.xacro`는 `world -> base_link -> 로봇 관절 -> link_6 -> bracket_link`
순서로 연결된다. `world_to_base_link`의 origin이 로봇 설치 위치/방향이다.
스캔 툴만 불러오는 `tool_scan.xacro`의 루트는 **브라켓 단품 STL의 원점과 축**인 `bracket_link`다.
브라켓 STL에만 `scale="0.001 0.001 0.001"`을 적용하여 mm를 m로 변환한다.
공식 카메라 STL은 이미 m이므로 추가 배율을 적용하지 않는다.

브라켓 기준 `camera_base_link`의 위치는 `(0, 3, -51.26) mm`,
회전 RPY는 `(0, 0, 90) deg`다. 기존 조립 TF를 유지하며 실측 보정값은 아니다.
공식 `femto_bolt.urdf.xacro`를 include하여 카메라 내부 nominal TF도 연결한다.
원본 카메라 Xacro와 STL 파일들은 수정하지 않는다.

**브라켓 프레임을 실제 TCP와 동일하게 쓰려면 원점뿐 아니라 축 방향도 일치해야 한다.**
이 파일은 로봇이나 카메라를 제어하지 않으며, 동역학/충돌 안전 검증용 모델이 아니다.

## 실행 환경과 지원 범위

Xacro를 Python 라이브러리로 확장한 뒤 URDF의 joint origin, 관절 로컬 축 회전,
visual origin, mesh scale을 합성하여 Open3D로 표시한다.
별도 ROS 노드, RViz, 서버는 사용하지 않는다.
모듈이 없는 Python 환경이라면 해당 환경에 설치한다.

```bash
python -m pip install xacro numpy open3d trimesh pycollada
```

현재 뷰어는 **fixed/revolute/continuous 조인트 + 메쉬 visual**을 지원한다.
로봇의 DAE 메쉬는 `trimesh`/`pycollada`로 node 변환을 적용해 읽고 Open3D로 표시한다.
STL 등 기존 지원 형식은 Open3D로 읽으며 URDF의 메쉬 배율은 한 번만 적용한다.
GUI에서 실시간 관절 조작, prismatic/floating/planar/mimic 조인트,
primitive visual, `package://` 자동 탐색은 지원하지 않는다.
상대 메쉬 경로는 최상위 입력 Xacro의 디렉토리 기준이며, 다른 디렉토리를 include할 때는
Xacro에서 `xacro.abs_filename(...)`으로 절대 경로를 만들어 주는 것이 안전하다.
매크로 정의만 있는 `femto_bolt.urdf.xacro` 대신 이를 호출하는 조립 Xacro를 지정해야 한다.
`--check`에는 화면이 필요 없지만 실제 뷰어는 GUI/OpenGL이 가능한 데스크톱에서 실행한다.
외부 Xacro는 수식/파일 include를 처리하므로 신뢰할 수 있는 파일만 연다.

참고: [Xacro 공식 저장소](https://github.com/ros/xacro),
[Open3D 시각화 문서](https://www.open3d.org/docs/release/tutorial/visualization/visualization.html).
