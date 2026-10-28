# 핸드아이 캘리브레이션

로봇 끝에 카메라를 장착하고 체커보드를 외부에 고정하는 eye-in-hand 보정입니다.
기존 테스트 구현을 이곳으로 옮겼으며, 테스트 모듈을 import하지 않습니다.
카메라 변환·SDK IK·별도 제어 프로세스는 현재 `scan_planner/1_acquisition.py`를 재사용합니다.

## 파일 구성

```text
hand_eye_calibration/
├── hand_eye_calibration.py        # RGB/C2D 촬영, 체커보드 PnP, Hand-eye 계산·품질 검사
├── hand_eye_calibration.yaml      # RGB/C2D 공용 설정·기본 C2D·관절 자세 20개
└── results/
    ├── index.json                # 가져온 파일 출처·SHA-256·TF 검사 결과
    ├── rgb/
    │   ├── calibration.json      # RGB 직접 보정 결과: T_flange_color
    │   ├── camera.json           # 당시 내부 파라미터·SDK 외부 파라미터
    │   ├── source_*.json         # 당시 촬영 설정/로봇 설정/세션 기록 (실행 설정 아님)
    │   └── depth_tf_variants/    # raw / unit_axis 비교본 (운영 기본값 아님)
    └── c2d/
        ├── calibration.json      # C2D 직접 보정 결과: T_flange_depth
        ├── camera.json
        └── source_*.json
```

## 가져온 기존 보정값

| 구분 | 원본 촬영 세션 | 방법·표본 수 | 보정 결과 |
|---|---|---|---|
| RGB | `20261015_202951_112921` | TSAI · 28개 | `results/rgb/calibration.json` |
| C2D | `20261026_233453_229364` | TSAI · 20개 | `results/c2d/calibration.json` |

결과 JSON·카메라 파라미터·촬영 메타데이터를 **원본 그대로 복사**했습니다.
행렬을 재계산하거나 정규화하지 않았습니다. 원본 테스트 로그와 이미지는 삭제하지 않았습니다.
복사본의 과거 절대 경로는 출처 기록이며, 현재 촬영 설정으로 읽지 않습니다.
`results/`는 사용할 보정값과 근거 자료이며 원본 이미지가 포함된 재계산용 세션은 아닙니다.

현재 `../../config/scan.yaml`은 아래 파일을 읽습니다. 테스트 폴더가 없어도 이 보정값을 적용할 수 있습니다.

```yaml
pose_estimation:
  source: hand_eye
  hand_eye_calibration_file: ../src/hand_eye_calibration/results/c2d/calibration.json
```

RGB 결과에는 `T_flange_color`만 있습니다. 당시 SDK Depth→RGB 행렬 검증이 실패하여
유효한 `T_flange_depth`가 생성되지 않았기 때문에 RGB JSON을 Depth 스캔에 바로 지정하면 차단됩니다.
`rgb/depth_tf_variants/raw` 및 `unit_axis`도 비교용으로 보존했으며,
두 Depth 변환 모두 현재 스캔의 강체 변환 검사를 통과하지 않습니다. 자동으로 보정하거나 대체하지 않습니다.
C2D 결과는 Depth 광학 프레임 기준 직접 추정값이므로 SDK 외부 TF를 다시 곱하지 않습니다.
다만 C2D 이미지 생성 자체가 SDK 정렬·Depth 측정에 의존하며, 품질 검사 통과는 실물 절대 정확도 보증이 아닙니다.

## 새 캘리브레이션 설정

실제 수정할 설정은 **이 폴더의 `hand_eye_calibration.yaml` 하나**입니다. 비어 있던 `scan_planning/config/hand_eye_calibration.yaml`은
백업 후 제거하여 설정을 이중 관리하지 않습니다.

- `camera.image_source`: `rgb` / `c2d`. 기본은 `c2d`이며 원본 RGB로 보정하려면 같은 YAML에서 `rgb`로 변경합니다.
  모드만 바꾸어도 `poses`는 자동 변경되지 않으므로 해당 영상에서 보정판이 보이는지 다시 확인하세요.
- `board`: 내부 코너 13×8, 한 칸 40 mm. 실제 보정판과 일치하는지 확인하세요.
- `calibration.method`: 기본 TSAI. PARK / HORAUD / ANDREFF / DANIILIDIS도 기존처럼 지원합니다.
- `poses`: `joint_deg` 또는 `pose_mm_deg` 중 하나. **Cartesian은 base_link 기준 link_6(플랜지)**입니다.
  `[x, y, z, roll, pitch, yaw]`의 단위는 mm/deg이며, 고정축 `Rz(yaw) @ Ry(pitch) @ Rx(roll)`입니다.
  카메라 기준 좌표나 펜던트의 Euler ZYZ를 그대로 넣지 마세요.
- `pose_generation`: 기본 YAML의 주석 예시로 기준 자세에서 촬영 자세를 생성할 수도 있습니다.
  이때도 기준 자세는 플랜지이고, 명목 RGB 장착 TF와 대략적인 보드 높이는 촬영 계획에만 사용합니다.
- `scan_config`: `../../config/scan.yaml`. 현재 로봇 IP·제어 Python/래퍼/라이브러리·Xacro·정지 조건을 읽습니다.
  스캔용 `views`는 실행하지 않으며, 핸드아이의 `poses`와 이동 속도를 사용합니다.
- `output_dir`: `../../log/hand_eye_calibration`. 새 촬영 결과 저장 위치입니다.

현재 제어 연결부는 `scan.yaml`의 `control_python`, `control_wrapper`,
`control_library`에 지정되어 있습니다. 환경이나 빌드 위치를 바꿨다면 실제 경로와 일치하는지 확인하세요.
요구 인터페이스는 [상위 README](../../README.md)의 새 제어 코드 연결 항목을 참고하세요.
이전 테스트 제어 코드로 자동 연결하지 않고, 이번 작업에서 제어 구현도 바꾸지 않았습니다.

촬영 좌표/보정 파라미터는 보존했지만 과거 안전 승인을 승계하지 않도록 현재 YAML의
`board.fixed_board_confirmed`, `motion.plan_confirmed`는 `false`로 초기화했습니다.
실제 보드 고정·로봇 초기 자세·장착·툴·케이블·전체 이동 구간을 확인한 뒤 설정하세요.
IK 성공과 `--check`는 충돌 안전 확인이 아니며, 실제 이동은 SDK IK 후 관절 이동입니다.

## 실행

프로젝트 루트에서 실행합니다. 작업 디렉토리가 달라도 YAML 내부 경로는 해당 YAML 기준입니다.

```bash
conda activate vehicle_painting_robot_scan

# 현재 YAML 설정 검사: 로봇·카메라·제어 프로세스 실행 없음
# RGB로 보정하려면 먼저 hand_eye_calibration.yaml의 camera.image_source를 rgb로 변경
python -s scan_planning/scripts/run_hand_eye_calibration.py --check

# 카메라만 사용해 체커보드/코너 확인 (로봇 이동 없음)
python -s scan_planning/scripts/run_hand_eye_calibration.py --preview

# 설정·이동 경로·장착/보드 고정 확인 후 실제 로봇 이동 → 촬영 → 계산
python -s scan_planning/scripts/run_hand_eye_calibration.py --execute

# 저장된 촬영 세션 재계산: 장비 연결 없음
python -s scan_planning/scripts/run_hand_eye_calibration.py --solve /촬영/세션

```

`src/hand_eye_calibration/hand_eye_calibration.py`를 직접 실행해도 같은 기능입니다.
`--execute`는 시작/각 자세 이동 전 Enter 확인을 유지합니다. 다른 입력·오류·중단 시 다음 동작을 중단하고
연결된 제어 프로세스의 종료를 요청합니다. 소프트웨어 종료는 안전 인증된 비상정지를 대신하지 않습니다.

`--solve`는 입력 세션의 `settings.json`, `camera.json`, `robot_settings.json`, `manifest.json`,
각 자세의 `sample.json`과 RGB/C2D 이미지를 사용합니다. 현재 YAML의 이미지 모드/방법으로 바꾸지 않습니다.
`--exclude pose_03 pose_07`로 표본을 명시적으로 제외할 수 있습니다.
결과는 입력 세션을 변경하지 않고 기본 `scan_planning/log/hand_eye_calibration/calibration_<일시>/`에 저장합니다.
`--output-dir /저장/폴더`로 새 결과의 저장 위치를 지정할 수 있습니다.

기존 `--export-depth-variants`도 보존했습니다. 가져온 `results/rgb/calibration.json`을 지정하면
같은 폴더의 `camera.json`을 사용하여 비교본을 새 로그 폴더에 출력합니다. 테스트 폴더를 읽을 필요가 없습니다.
RGB 해/SDK 원본 행렬에 대한 비교 기능일 뿐, 강체 변환 불량을 해결한 것으로 취급하지 않습니다.

새 보정 결과는 현재 `results/`나 스캔 설정에 **자동으로 덮어쓰지 않습니다**.
품질 지표·코너 원점 일치·실제 정합 결과를 확인한 뒤 `scan.yaml`의 `hand_eye_calibration_file`로 선택하세요.
모든 `T_parent_child`는 child 좌표를 parent로 옮기는 4×4 행렬이며 위치 단위는 m입니다.

이번 이관 전 운영 설정과 빈 파일은 `scan_planning/backup/20261028_before_hand_eye_migration/`에 보존했습니다.
