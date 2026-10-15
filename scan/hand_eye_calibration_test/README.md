# A0912 / Femto Bolt 핸드아이 캘리브레이션 테스트

로봇 플랜지에 카메라를 고정하고 **체커보드는 로봇 밖에 고정**하는 eye-in-hand 방식이다.
카메라가 외부에 고정된 eye-to-hand 방식은 지원하지 않는다.
ROS 없이 기존 두산 SDK 제어 래퍼와 Orbbec SDK, OpenCV를 사용한다.

## 파일과 실행 환경

- `scripts/hand_eye_calibration.py`: 설정 검사, RGB 미리보기, SDK 로봇 이동/촬영, 보정 계산.
- `scripts/hand_eye_calibration.yaml`: 보정판, 기준 자세/촬영 계획, 저속 이동 및 품질 검사 설정.
- `log/일시/`: 실행하면서 생성되는 촬영/계산 결과.

메인 프로그램은 `vehicle_painting_robot_scan` 환경에서 실행한다.
로봇 제어는 기존 `robot_scan_worker.py`를 별도 **control 환경 Python 프로세스**로 실행한다.
두산 SDK나 제어 라이브러리를 다시 설치/수정하는 방식이 아니다.
기존 worker에 정지 상태의 **SDK 플랜지/TCP 실측 자세 조회 명령** `snapshot_cartesian`만 추가했다.
기존 스캔/이동 명령의 처리 방식은 그대로 유지한다.

로봇 IP, control Python, wrapper, `.so`, Xacro 및 정지 판정 설정은
YAML의 `scan_config`가 가리키는 기존 `multi_view_scan.yaml`에서 가져온다.
**기존 스캔의 views는 실행하지 않는다.** 촬영 자세는 이 YAML의 기준 자세로 생성하고,
속도/가속도/최대 관절 이동량은 이 YAML의 `motion`으로 덮어쓴다.

## 먼저 입력할 설정

1. `board.inner_corners`: 가로/세로 **내부 교차점** 수. 기본 9×6은 예시다.
2. `board.square_size_mm`: 실제 한 칸 크기(mm). 임의의 기본 치수는 넣지 않았다.
3. 보정판을 단단한 평면에 고정한 후 `board.fixed_board_confirmed: true`.
4. `pose_generation.reference_pose_mm_deg`: RGB 카메라가 보정판 중심을 거의 수직 아래로
   바라보는 **플랜지 기준 자세 한 개**. 기본 `null`이며 입력 전 실행이 차단된다.
5. 전체 이동 구간·툴 무게/무게중심·장착·케이블·주변 안전을 확인하고 `motion.plan_confirmed: true`.

Cartesian 입력은 **base_link 기준 플랜지(link_6)의 절대 위치·회전**이다.
`reference_pose_mm_deg: [x, y, z, roll, pitch, yaw]`, 위치 mm / 고정축 RPY deg이며
`R = Rz(yaw) @ Ry(pitch) @ Rx(roll)`이다. 펜던트의 Euler ZYZ 값을 그대로 넣으면 안 된다.
기준 자세를 잡을 때 `live_rgbd_cartesian.py`의 RGB 화면을 활용할 수 있다.
단, 그 도구의 `cartesian.target_frame`도 `link_6`여야 입력 좌표가 같은 의미다.

### 기준 자세로 촬영 위치 생성

```yaml
pose_generation:
  reference_pose_mm_deg: null  # 현장에서 확인한 [x,y,z,roll,pitch,yaw]로 교체
  board_height_mm: 0.0
  tilt_deg: 15.0
  min_camera_height_mm: 200.0
```

1. 기준 플랜지 자세와 Xacro의 **명목 flange→RGB optical TF**로 기준 카메라 자세를 구한다.
2. RGB의 +Z 시선과 `base_link` 기준 `Z=board_height_mm` 평면이 만나는 점을
   보정판의 **촬영용 추정 중심**으로 잡는다. 따라서 기준 RGB 화면 중앙에 판 중심을 맞춰야 한다.
3. 카메라와 추정 중심 사이 거리를 유지하며 중심을 계속 바라보도록 자세를 생성한다.
   기준 1개 → 작은 기울기(`tilt_deg/2`) 원주 8개 → 큰 기울기(`tilt_deg`) 원주 8개 순서다.
   기본값은 **7.5도/15도로 기울인 총 17개**이며 서로 다른 축 방향의 회전이 포함된다.
4. 명목 장착 TF를 역으로 적용해 각 카메라 자세를 플랜지 목표 XYZ/RPY로 변환한다.
   실제 이동은 기존 두산 SDK IK → 관절 이동이다. 원주를 따라 직선/원호 이동하는 명령이 아니다.

**`board_height_mm: 0`은 촬영 자세 생성용 근삿값이다.** 실제 판이 약 10 mm 아래에 있어도
그 오차를 보정 계산의 제약으로 넣지 않는다. 보정은 실제 RGB의 코너/PnP와 SDK 실측 플랜지 자세만
사용하며, 베이스 기준 판의 실제 위치는 미지수로 남긴다. 단, 높이/명목 TF가 틀리면 계획한
시야·간격은 달라질 수 있으므로 모든 시점에서 보정판 전체가 보이는지 확인해야 한다.
보정판 자체 좌표에서 코너의 Z=0인 것과 **로봇 베이스에서 판 높이를 Z=0으로 가정하는 것**은 다르다.

기준 시선은 수직 아래에서 20도 이내여야 하며, 생성 기울기는 최대 30도로 제한한다.
`min_camera_height_mm`는 추정 평면보다 카메라 원점이 얼마나 위에 있는지만 검사한다.
**툴 몸체/로봇 팔/케이블이나 관절 이동 구간의 충돌 검사가 아니다.**
`scan_config`의 `mounting_tf_confirmed`도 확인해야 한다. 촬영 계획용 TF는 근삿값으로 사용할 수 있지만
실제 시야와 이동 구간을 반드시 검토해야 한다.

`--check`는 추정 중심과 생성된 17개 좌표, 회전 다양성을 출력한다. 실제 IK 도달 가능성과
충돌 안전은 판정하지 않는다. `motion.plan_confirmed`를 켜기 전에 전체 구간을 검토하고,
기본 `confirm_each_pose: true`를 유지하여 각 이동 전에 확인한다.

기존 수동 `poses` 목록과 이전 저장 세션도 지원한다. 수동 목표를 쓰려면 `pose_generation`을
제거하고 `poses: [{name: pose_01, pose_mm_deg: [...]}, ...]`를 지정한다.
수동 목록에서는 `joint_deg`도 가능하다. `pose_generation`이 있으면 수동 `poses`는 실행하지 않는다.

보정판을 계속 바라보면서 서로 평행하지 않은 여러 축으로 카메라를 기울여야 한다.
위치만 바꾸거나 한 축으로만 회전하면 해를 구하기 어렵고 이 코드는 퇴화 입력을 거부한다.
자동 생성은 촬영 목표 계산이며 충돌 검사/경로 회피는 없다. IK가 풀려도 이동 구간이 안전하다는 뜻은 아니다.
기존 SDK IK → 관절 이동 방식을 사용하며 Cartesian 직선 이동(MoveL)이 아니다.

일반 체커보드는 코너에 고유 ID가 없어 180도 원점 뒤집힘이 생길 수 있다.
촬영 중 판을 움직이거나 뒤집지 말고, 이미지에서 같은 물리적 모서리가 계속 좌상단이 되도록 유지한다.
`corners.png`의 빨간 `O`가 모든 뷰에서 같은 모서리인지 반드시 확인한다.
코드는 검출 순서의 일반적인 180도 반전만 정규화하며 임의의 시점/큰 회전을 완전히 판별하지 않는다.
이 조건이 어려우면 고유 ID가 있는 ChArUco/AprilTag 타깃을 지원하도록 별도 확장해야 한다.

## 실행 명령어

```bash
conda activate vehicle_painting_robot_scan
cd /home/oms/vehicle_painting_robot/scan/hand_eye_calibration_test/scripts

# 기준 자세 입력 후 생성 좌표 출력 + 파일/환경 검사. 장치 연결·촬영·서보 활성화 없음.
python -s hand_eye_calibration.py --check

# 실제 보정판 치수를 입력한 뒤 카메라만 사용해 검출 상태 확인.
# 프레임 준비/검출 때문에 저속 갱신된다. Q/ESC/창 닫기로 종료.
python -s hand_eye_calibration.py --preview

# 필수 설정 완료 후 실제 로봇 이동 → 촬영 → 로봇 종료 → 계산/저장.
python -s hand_eye_calibration.py --execute
```

`--execute`에서는 시작 시 Enter로 동의하고, 기본적으로 각 자세로 이동하기 전에도 Enter로 확인한다.
다른 문자, EOF, Ctrl+C, SIGTERM 또는 오류 시 다음 동작을 중단하고 기존 제어기의 종료/servo-off를 요청한다.
소프트웨어 종료는 안전 인증된 비상정지가 아니다. 장치/통신 오류 시 현장의 실제 정지를 확인해야 한다.

저장된 세션만 다시 계산할 때는 로봇/카메라를 연결하지 않는다.

```bash
python -s hand_eye_calibration.py --solve ../log/촬영_디렉토리

# 원점 뒤집힘/불량 이미지 등을 확인한 뒤 명시적으로 제외 가능.
# 나머지 표본도 min_samples 및 회전 다양성 조건을 충족해야 한다.
python -s hand_eye_calibration.py --solve ../log/촬영_디렉토리 --exclude pose_03 pose_07

```

`--solve`는 **세션에 저장된** `settings.json`/`robot_settings.json`/`camera.json`을 사용하며,
원본 RGB에서 코너와 PnP를 다시 계산한다. 기존 표본/계산 결과를 덮어쓰지 않는다.
미완료 세션도 이미 accepted로 저장된 표본이 충분하다면 계산 가능하다.
검출 실패 시점의 원본은 보존하지만, 자동으로 다음 자세로 진행하지 않는다.

### 원본/축 길이 보정 Depth TF를 포함한 calibration.json 저장

이미 계산한 RGB 핸드아이 값(`T_flange_color`)에 SDK의 Depth→RGB 행렬을 곱해
플랜지 기준 Depth 좌표계(`T_flange_depth`)를 계산한다. 기존 보정 결과 전체를 유지하고
이 행렬을 추가한 `calibration.json` 두 개를 저장한다. 로봇·카메라 연결이나 RGB 코너
재검출 없이 저장된 JSON만 사용한다.

```bash
python -s hand_eye_calibration.py --export-depth-variants ../log/20261015_202951_112921

# 세션 내 특정 계산 결과를 지정할 수도 있다.
python -s hand_eye_calibration.py --export-depth-variants \
  ../log/20261015_202951_112921/calibration_20261015_203609_885625/calibration.json
```

세션 폴더를 지정하면 가장 최근에 완료된 `calibration_*/calibration.json`을 선택하고
선택한 경로를 출력한다. 계산 결과 폴더도 입력 가능하다. 원본 외부 파라미터는 같은 세션의
`camera.json`에서 `T_color_depth_sdk_raw`를 우선 사용하며, 없는 경우 `T_color_depth`를 사용한다.

결과는 세션의 새 `depth_tf_variants_일시/` 폴더에 저장한다.

- `raw/calibration.json`: 길이/직교성 검사에 실패해도 SDK 행렬을 그대로 곱한 결과.
- `unit_axis/calibration.json`: 기본 **X축(회전행렬 첫 번째 열)** 벡터를 자신의 길이로 나눠
  길이 1로 만든 뒤 곱한 결과. 다른 축을 비교하려면 `--normalize-axis y` 또는 `z`를 추가한다.

두 JSON 모두 기존 `T_flange_color`, 계산 방법, 촬영 표본, 사용/제외 목록, 오차·품질 결과,
`status`, `robot_pose_source`를 보존하고 `T_color_depth`, `T_flange_depth`를 추가한다.
`depth_tf_export`에는 원본 경로, 보정 방법, 축 길이, 축 사이 내적, 직교 오차,
determinant, SE(3) 허용오차 통과 여부를 기록한다. 위치 단위는 m다.
보정본은 선택한 벡터의 방향과 나머지 축·이동량을 유지한다. SVD 직교화나 특정 원소 교체는
적용하지 않으므로 축 간 직교 오차가 남을 수 있다.

이는 **Depth TF가 포함된 전체 핸드아이 보정 결과**이며 Depth 이미지나 점군을 생성하는 기능은 아니다.
기존 파일을 덮어쓰지 않으며 새 파일도 기존과 같은 `computed_requires_physical_validation` 상태를 유지한다.
`quality_checks_passed`는 원본 RGB 보정의 품질 결과를 그대로 유지하며, Depth TF의 진단값은 별도로 기록한다.
이미 출력한 Depth TF에 대한 과거 "미출력" 경고는 출처 정보에 보관하고 현재 상태에 맞는 경고를 기록한다.
스캔에서 비교하려면 기존 `pose_estimation.source: hand_eye`를 사용하고
`hand_eye_calibration_file`에 둘 중 하나를 지정한다. 원본/보정본은 파일 경로로 선택하며,
설정 예시는 [스캔 스크립트 README](../doosan_a0912_scan_test/scripts/README.md)의
"SDK 원본/축 길이 보정 TF로 정합 비교"를 참고한다.

## 계산과 좌표계

1. 생성한 각 플랜지 목표로 SDK IK/관절 이동 후 기존 정지 유지 조건을 확인한다.
2. 촬영 전·후 SDK에서 실제 플랜지 자세를 읽는다. **목표 자세나 URDF FK로 대신하지 않는다.**
3. 각 정지 자세에서 RGB 스트림을 새로 시작하고 준비 프레임을 버린 후 원본 RGB를 촬영한다.
4. SDK의 해당 RGB 프로파일 내부 파라미터/왜곡 계수와 체커보드 치수로 PnP를 계산한다.
5. 촬영 중 관절/플랜지 변화, 보정판 검출, 재투영 RMS를 검사한다.
6. `cv2.calibrateHandEye`로 `T_flange_color`를 구한다(기본 PARK, YAML에서 방식 선택).
7. 고정 보정판 위치·회전의 일관성 및 한 표본씩 제외했을 때 보정값 변화(leave-one-out)를 저장한다.

모든 행렬은 `T_parent_child`: **child 점을 parent 좌표로 변환**하는 4×4 행렬이며 위치는 m다.
RGB/Depth는 광학 프레임(x 오른쪽, y 아래, z 앞)이다.

```text
T_base_board = T_base_flange × T_flange_color × T_color_board
T_flange_depth = T_flange_color × T_color_depth
```

SDK 로봇 자세(mm/Euler ZYZ)는 m/회전행렬로 변환하고,
체커보드 좌표 역시 mm → m로 변환하여 단위를 맞춘다.
Depth->RGB TF는 SDK `depth_profile.get_extrinsic_to(color_profile)`에서 읽는다.
이 TF가 없거나 회전행렬 검증에 실패하면 **RGB 보정만 저장**하고 Depth TF 미출력 경고를 남긴다.
기본 촬영/`--solve`에서는 검증 실패한 회전행렬을 임의로 정규화하거나 transpose/부호 변경하지 않는다.
위의 `--export-depth-variants`를 명시한 경우에만 원본/축 길이 보정 비교 파일을 별도로 만든다.

**이 결과는 SDK 플랜지 자세 모델 기준**이다.
스캔의 `multi_view_scan.yaml`에서 `pose_estimation.source: hand_eye`와
`hand_eye_calibration_file: /결과/경로/calibration.json`을 지정하면,
`multi_view_scan.py`와 `multi_view_scan_single_frame.py`가 SDK 실측 플랜지 자세와
보정 `T_flange_depth`를 함께 사용한다. URDF FK에 보정값만 단순 대입하는 방식은 아니다.
`source: urdf`는 기존 실측 관절각 + URDF FK 방식이다.
보정 품질 검사 통과 여부와 별개로 독립 자세/정합 데이터로 검증해야 한다.
기존 Xacro, 로봇 TCP, 보정판 위치를 자동 수정하지 않는다.

## 저장 결과와 해석

- `settings.json`, `robot_settings.json`, `camera.json`: 실제 설정, 제어 연결 설정, 내부/외부 파라미터.
- `pose_plan.json`: 자동 생성에 사용한 명목 TF, 추정 중심, 카메라 높이, 생성 플랜지 목표 목록.
- `manifest.json`, `robot_worker.log`, 실패 시 `error.log`: 수집 상태/제어/오류 기록.
- 각 `pose_XX/color_rgb.png`: 원본 RGB.
- 각 `pose_XX/corners.png`: 코너·원점·보정판 좌표축 표시.
- 각 `pose_XX/sample.json`: 목표/실측 SDK 자세, 프레임 시각, PnP, 재투영 오차, 촬영 중 자세 변화.
- `calibration_일시/calibration.json`: 변환행렬, 사용/제외 표본, 회전 다양성, 품질 지표와 경고.

`quality_checks_passed`는 설정한 표본 품질/안정성 검사 결과이며 실제 절대 정확도 보장이 아니다.
성공 상태도 `computed_requires_physical_validation`로 표시한다.
보정판 평탄도/치수, RGB 내부 보정 정확도, 플랜지 측정 정확도, 장착 강성에 따른 오차는 별도 검증 대상이다.

참고: [OpenCV calibrateHandEye 공식 설명](https://docs.opencv.org/4.10.0/d9/d0c/group__calib3d.html#gaebfc1c9f7434196a374c382abf43439b),
[두산 posx의 Euler ZYZ](https://manual.doosanrobotics.com/en/programming-manual/3.4.0/publish/posx-x-0-y-0-z-0-a-0-b-0-c-0).
