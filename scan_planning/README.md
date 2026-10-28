# Scan Planning

상위 제어기는 `scripts/scan_planning.py`입니다. 실행 흐름은 다음과 같습니다.

1. **데이터 취득:** 로봇 이동/정지 확인 → RGB·Depth·SDK 점군·실측 자세 저장
2. **정합·보정:** 실측 TF로 base_link 초기 배치 → 모든 뷰 쌍 ICP → PGO → 병합
3. **클러스터링:** DBSCAN → 촬영 광축의 최소제곱 기준점에 가까운 유효 군집 선택

스캔 파이프라인은 `--execute`와 Enter 확인을 모두 거친 경우에만 로봇/카메라에 연결합니다.
기본 실행과 `--check`는 설정 검사이며 실장비 사용 가능성을 보증하지 않습니다.
저장 데이터를 처리할 때는 카메라 SDK·제어 라이브러리·ROS가 필요하지 않습니다.

## 디렉토리

- `src/scan_planner/`: 아래 세 단계의 실제 구현과 별도 제어 worker.
- `scripts/scan_planning.py`: 설정 로딩, 세 단계 호출 순서, 실패 처리, 로그 세션 관리.
- `scripts/run_scan.py`: 취득 단계만 실행.
- `scripts/run_registration.py`: 저장 취득 데이터 → TF 재구성 + 보정.
- `scripts/run_segmentation.py`: 저장 재구성/보정 데이터 → 부품 추출.
- `scripts/camera_diagnostic.py`: Femto Bolt RGB/Depth 수신·케이블 진단과 선택적 미리보기 (로봇 제어 없음).
- `scripts/run_hand_eye_calibration.py`: RGB/C2D 핸드아이 캘리브레이션 진입점.
- `test/`: 기존 실험 코드·데이터·모델을 보존한 영역. 새 코드에서 기존 테스트 모듈을 import하지 않습니다.
- `log/`: 새 코드의 모든 실행 결과.

### scan_planner 파일 구성

```text
src/scan_planner/
├── 1_acquisition.py    # 각 뷰 이동·카메라 취득·필터·TF·저장
├── 2_registration.py   # TF 초기 배치·병합, 4종 ICP, PGO
├── 3_clustering.py     # DBSCAN·시선 기준점·부품 선택
├── robot_worker.py     # 별도 제어 환경에서 SDK 래퍼 실행
└── __init__.py         # 숫자 파일명을 역할 이름으로 연결
```

`scripts/scan_planning.py`가 설정을 읽고 1 → 2 → 3 순서로 실행합니다.
각 파일 안에는 해당 단계의 보조 함수·클래스까지 함께 들어 있습니다.
1단계의 진입점은 `prepare_capture`/`acquire`, 2단계는 `reconstruct`/`run`,
3단계는 `run`입니다. `robot_worker.py`는 1단계의 연결부이지 별도의 공정이 아닙니다.

Python 코드에서는 `from scan_planner import acquisition, registration, clustering`로
불러옵니다. 장비·Open3D import는 필요한 실행 시점까지 지연하며, worker의 IK
계산용 import에는 카메라 SDK나 YAML 로더가 필요하지 않습니다.

기존 결과와의 호환을 위해 로그의 `01_capture`, `02_reconstruction`,
`03_registration`, `04_segmentation` 이름은 유지합니다. 가운데 두 폴더는
모두 2단계 정합·보정의 중간/최종 결과입니다. YAML 이름과 파라미터 값도 유지합니다.

2026-10-28 이름 변경 전 운영 코드·설정은 `backup/20261028_before_rename/`에
보존했습니다. 최상위 디렉토리 이름만 `scan_planning`으로 변경했으며
Conda 환경명 `vehicle_painting_robot_scan`, `config/scan.yaml`은 유지합니다.
상위 실행 파일은 `scripts/scan_planning.py`로 이름을 변경했습니다.
세 단계 통합 직전 소스·실행 코드·설정은 `backup/20261028_before_three_stages/`에
별도로 보존했습니다. 기존 세부 모듈은 운영 경로에서 제거했으며 테스트·모델·기존 로그는 변경하지 않았습니다.

SAM, EfficientSAM3, 커스텀 모델은 가져오지 않았습니다.
핸드아이 캘리브레이션 코드·촬영 설정 YAML·기존 RGB/C2D 보정값은
`src/hand_eye_calibration/`에 모았습니다. [보정 사용법](src/hand_eye_calibration/README.md)을 참고하세요.
`scripts/run_hand_eye_calibration.py`로 실행하며 새 촬영/재계산 결과는
`log/hand_eye_calibration/`에 저장합니다. 스캔 기본 보정값은
`src/hand_eye_calibration/results/c2d/calibration.json`을 읽으므로 테스트 폴더에 의존하지 않습니다.
중복을 피하려고 비어 있던 `config/hand_eye_calibration.yaml`은 백업 후 제거했습니다.
`scripts/camera_diagnostic.py`는 기존 케이블 진단 구현을 운영 경로로 복사한 독립 실행 도구입니다.
테스트 모듈을 import하지 않으며, 새 진단 결과는 `log/camera_diagnostic/`에 저장합니다.
기존 테스트의 파일/저장 경로는 수정하지 않았습니다. 테스트를 옮기면서 발생한 옛 상대·절대 경로 문제도 별도 수정 대상입니다.

## 설정

상대 경로는 실행한 터미널이 아니라 **각 YAML 파일 위치** 기준입니다.

| 파일 | 설정 |
|---|---|
| `config/scan.yaml` | 제어 연결부, 촬영 자세, TF, 카메라 프로파일, 거리/ROI, 재구성 voxel, 다른 YAML 경로 |
| `config/orbbec_sdk_filter.yaml` | SDK 필터 사용 여부·파라미터·이력 워밍업 |
| `config/registration.yaml` | 보정 켜기/끄기, ICP 방식·멀티스케일·PGO 기준 |
| `config/segmentation.yaml` | DBSCAN 거리/점 개수, 자동 기준점/수동 기준점, 군집 ID 선택 |

현재 기본 취득 모드는 기존 `multi_view_filter.py`의 `sdk_filters`입니다.
TemporalFilter를 사용하며, 이동 후 새 스트림과 초기 버림 프레임, 필터 이력 워밍업을 거쳐 마지막 출력 한 장을 저장합니다.
각 뷰마다 필터 이력을 초기화합니다.
`capture_mode: median`으로 바꾸면 기존 `multi_view_scan.py` 방식으로
0 제외 하위 중앙값과 최소 유효 관측 수를 적용합니다. 이 모드에서는 SDK 필터 설정을 적용하지 않습니다.
`camera.capture_frames/min_valid_frames`는 median 모드에서만 사용합니다.
두 방식을 자동 혼합하지 않습니다. 카메라 측 SDK 자체 처리는 별개입니다.

TF는 `pose_estimation.source: urdf / hand_eye`로 선택합니다.
비강체 행렬을 임의로 정규화하거나 무시하지 않습니다.
강체 TF 검사를 실패하면 원인을 해결한 뒤 실행해야 합니다.
ICP/PGO 결과는 이번 데이터의 정합 결과일 뿐이며 로봇 장착 TF/핸드아이 파일에 덮어쓰지 않습니다.

DBSCAN은 차량 부품의 의미를 인식하는 모델이 아닙니다.
거치대와 부품이 연결되어 있으면 함께 남을 수 있습니다.
광축이 평행하거나 시선 교차 오차가 크면 자동 선택하지 않으며,
`target_point_base_mm` 또는 `select_cluster_ids`를 지정하여 다시 실행할 수 있습니다.
이때 군집 시각화 결과를 먼저 확인하세요.

## 실행

프로젝트 루트에서:

```bash
conda activate vehicle_painting_robot_scan

# 설정만 검사 (장비 연결/로그 생성 없음)
python -s scan_planning/scripts/scan_planning.py --check

# 기존 5뷰 촬영 데이터로 전체 후처리 (새 촬영 없음)
python -s scan_planning/scripts/scan_planning.py \
  --capture-dir scan_planning/test/doosan_a0912_scan_test/log/20261026_235704_386974 \
  --visualize

# 취득 데이터 검사만
python -s scan_planning/scripts/scan_planning.py --capture-dir /촬영/폴더 --check

# 새 촬영부터 부품 추출까지: 제어 연결부와 안전 확인을 먼저 완료해야 함
python -s scan_planning/scripts/scan_planning.py --execute

# 단계별 실행
python -s scan_planning/scripts/run_scan.py --execute
python -s scan_planning/scripts/run_registration.py --capture-dir /촬영/폴더 --visualize
python -s scan_planning/scripts/run_segmentation.py --geometry-dir /재구성_또는_PGO/폴더 --visualize

```

`--capture-dir`에는 `manifest.json`, 각 뷰의 `camera_filtered.ply`, `pose.json`이 있는
촬영 폴더를 지정합니다. PLY 하나만으로는 여러 뷰의 보정/촬영 광축 복원이 불가능합니다.
입력에 저장된 카메라 점군과 실측 자세를 그대로 읽으며, 현재 scan.yaml의 촬영 자세/ROI로
이전 데이터를 다시 필터링하거나 TF를 덮어쓰지 않습니다.
`reconstruction.voxel_size_mm`, 보정·클러스터링 설정은 새 처리에 적용됩니다.

`--geometry-dir`에는 새 파이프라인의 `02_reconstruction` 또는 `03_registration` 폴더를 지정합니다.
후자는 **보정된 카메라 광축**으로 부품을 선택합니다.
단계 실패 시 이후 단계는 실행하지 않습니다. 자동 부품 선택 실패는
`needs_selection`과 종료 코드 2로 표시합니다. 선택 성공도 시각적 확인이 필요합니다.

### 카메라·케이블 진단

로봇 제어 없이 RGB/Depth 수신 FPS, 누락 의심·중복·타임아웃, SDK 경고와 USB 연결 정보를 기록합니다.
스캔 파이프라인과 별도 도구이므로 위 `--execute`/Enter 확인 규칙은 적용되지 않습니다.
`--help`는 장비 연결 없이 도움말만 표시하고, `--list-profiles` 및 측정 명령은 카메라에 연결합니다.

```bash
conda activate vehicle_painting_robot_scan

# 도움말만 확인 (장비 연결/로그 생성 없음)
python -s scan_planning/scripts/camera_diagnostic.py --help

# 연결한 카메라의 지원 프로파일 확인 (스트림 시작/로그 저장 없음)
python -s scan_planning/scripts/camera_diagnostic.py --list-profiles

# 직결 상태에서 600초 측정 + RGB/Depth 미리보기
python -s scan_planning/scripts/camera_diagnostic.py --label direct --seconds 600 --visualize
```

기존 진단 기본값(RGB 1920×1080 MJPG, Depth 640×576 Y16, 각 30 FPS)을 유지합니다.
이는 제조사 권장 기본값이라는 뜻이 아니며, 미지원 시 자동으로 낮추지 않고 오류를 표시합니다.
`--label`로 케이블을 구분하고 `--note`로 제품명·길이 등을 기록할 수 있습니다.
`--visualize`를 빼면 터미널 진단만 수행합니다. 미리보기 갱신률과 측정 수신 FPS는 다릅니다.
결과는 `log/camera_diagnostic/<일시>_<label>/`에 `summary.json`, `summary.txt`,
프레임·이벤트·구간별 로그와 SDK 로그로 저장합니다. `--output-root`로 저장 루트를 바꿀 수 있습니다.
수신 이상이 없다는 결과가 케이블의 실제 갠트리 운용 안정성을 보증하지는 않습니다.

## 새 제어 코드 연결

`config/scan.yaml`은 `paint_cell`에서 복사한 새
`robot_controller/doosan_a0912_controller` 구현에 연결되어 있습니다.
이전 테스트 제어 구현을 복원한 것이 아니라 복사본에 조회/IK API를 추가한 것입니다.
`vehicle_painting_robot_control` Python으로 worker를 실행하고,
`test/scripts/joint_target_test.py` 및 `test/build/libdoosan_controller_c_api.so`를 사용합니다.

현재 스캔용 프로세스 어댑터는 다음 Python 인터페이스를 요구합니다.

- `DoosanRobot(library_path=...)`
- `initialize(ip, port, mode, rt_ip, rt_port, ..., timeout)`
- `read_actual_joint_state()`: joint_deg, joint_velocity_deg_s, motion_running
- `read_actual_cartesian_state()`: flange_pose_mm_zyz_deg, tcp_pose_mm_zyz_deg
- `solve_closest_ik(pose_mm_zyz_deg, reference_joint_deg)`
- `velocity_control_to_position(...)`, `wait_until_motion_done(...)`, `shutdown()`
- IK 미해결 예외 `IKNoSolutionError`

C API 심볼/API 유무는 연결 전에 검사합니다. 추가 API는 실측 관절각·속도,
실측 플랜지·활성 TCP 자세, 이동 없는 SDK IK 계산을 제공합니다.
`paint_cell` 원본은 수정하지 않았습니다. 실측 조회 실패 시 목표 자세나 0 속도로 대신하지 않습니다.
빌드 및 장비 없이 하는 래퍼 검사는 `robot_controller/doosan_a0912_controller/README.md`를 참고하세요.

촬영 자세는 기존 설정의 다섯 관절 목표를 유지했습니다.
실제 장착/툴/초기 자세/전체 이동 경로/비상정지를 현장에서 확인한 뒤
`mounting_tf_confirmed: true`로 변경하세요. 이 코드는 이동 구간 충돌 검사를 하지 않습니다.

## 로그

```text
scan_planning/log/<실행시각>/
├── config/                  # 사용한 YAML 원본 스냅샷
├── resolved_scan_settings.json
├── resolved_settings.json    # 실제 적용한 모든 단계의 해석된 파라미터
├── manifest.json            # 단계 상태·입력·결과 경로·실행 시간
├── run.log
├── 01_capture/              # 새 촬영일 때만 생성
├── 02_reconstruction/       # TF 기반 각 뷰 base.ply, 병합본
├── 03_registration/         # 보정된 뷰/자세, pose graph, 요약, 병합본
├── 04_segmentation/         # 군집별 PLY, 라벨, 선택 미리보기, part.ply
├── part.ply                 # 선택 성공 시 최종 부품 점군
└── error.log                # 실패할 때
```

저장 데이터 재처리는 원본 취득 데이터를 복사하지 않고 manifest에 출처를 기록합니다.
따라서 재처리 원본도 계속 보관하세요. 실패한 단계의 중간 결과와 로그는 남깁니다.
점군/TF 위치 단위는 m, 설정 파일의 거리 파라미터는 이름에 명시한 mm입니다.
