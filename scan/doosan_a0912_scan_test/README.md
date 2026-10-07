# Doosan A0912 + ORBBEC Femto Bolt 스캔 테스트

## 실행 방법 (2026-10-07)

실행 파일은 `scripts/three_view_scan.py`, 설정 파일은
[`scripts/three_view_scan.yaml`](scripts/three_view_scan.yaml)이다.
**관절 입력과 Cartesian 입력 모두 실행 명령어는 동일하다.**
아래 1~8절은 2026-09-07의 제조사 CAD/TF 조사 기록이며, 현재 실행 방법은 이 절을 따른다.
촬영·저장 구조의 상세 설명은 [`scripts/README.md`](scripts/README.md)에 있다.

### 1) 환경 활성화와 작업 디렉토리 이동

```bash
conda activate vehicle_painting_robot_scan
cd /home/oms/vehicle_painting_robot/scan/doosan_a0912_scan_test/scripts
```

사용자가 실행하는 환경은 **scan 환경**이다. 로봇 제어 및 IK는 YAML의 `control_python`에
지정된 **control 환경 Python**을 자식 프로세스로 실행하므로 별도 터미널에서 제어 코드를
실행할 필요가 없다. 다른 PC/환경에서는 `control_python`, `control_wrapper`, `control_library`
경로를 확인한다. `robot_scan_worker.py`와 `cartesian_scan.py`를 직접 실행하지 않는다.

### 2) 목표 자세 설정

현재 YAML의 `views`에는 기존 `joint_deg` 값이 남아 있다. **XYZ/RPY로 이동하려면 각 시점의
`joint_deg`를 삭제하고 `pose_mm_deg`로 교체해야 한다.** 한 시점에 두 키를 동시에 쓰면 오류다.

```yaml
cartesian:
  target_frame: link_6
  position_tolerance_mm: 0.1
  orientation_tolerance_deg: 0.1
  max_iterations: 300

views:
  - name: view_01
    pose_mm_deg: null  # [x, y, z, roll, pitch, yaw] 실제 숫자 6개로 교체
  - name: view_02
    pose_mm_deg: null
  - name: view_03
    pose_mm_deg: null
```

- 위치는 **mm**, 회전은 **deg**, 기준 좌표계는 로봇 **`base_link`**다. 상대 이동량이 아닌 절대 자세다.
- RPY 규약은 `Rz(yaw) @ Ry(pitch) @ Rx(roll)`이다. 두산 기본 Euler ZYZ(A/B/C)를 그대로 넣지 않는다.
- `target_frame: link_6`는 **플랜지 원점·축**이다. 브라켓 기준이면 `bracket_link`,
  Depth 광학 원점 기준이면 `camera_depth_optical_frame`을 선택한다.
  플랜지와 브라켓은 원점이 같아도 축 방향이 다를 수 있다. 펜던트의 활성 TCP를 자동 적용하지 않는다.
- `null`이 남아 있으면 실제 실행은 차단된다. `views`는 원하는 시점 수만큼 지정하며 목록 순서대로 이동한다.
- 로봇 IP, 속도·이동량 제한, 장착 TF를 확인한다. `mounting_tf_confirmed: true`는 실물과 TF를 확인한 경우에만 사용한다.

### 3) 하드웨어 연결 없는 검사

```bash
python -s three_view_scan.py --check
```

파일·TF·라이브러리와, Cartesian 입력 시 제어 환경의 Pinocchio/IK 모델 로딩을 검사한다.
**로봇·카메라에 연결하거나 로봇을 움직이지 않는다.** 목표 미입력 여부도 출력한다.
현재 로봇 관절각은 읽지 않으므로 실제 시작 자세에서의 IK 성공이나 충돌 안전을 보장하는 검사는 아니다.
`--check`와 `--execute` 모두 생략한 기본 실행도 검사만 한다.

### 4) 실제 로봇 이동 + 스캔 + 결과 저장

```bash
python -s three_view_scan.py --execute
```

출력된 목표 자세와 현장 안전을 확인하고, 터미널에서 **`SCAN`**을 입력하면 시작한다.
전체 이동 구간의 장애물·케이블·작업자, 장착 및 툴 설정, 비상정지를 먼저 확인한다.
코드에 충돌 검사나 자동 회피는 없다.

Cartesian 입력 시 동작은 **현재 실측 관절각 조회 → IK로 목표 관절각 계산 → 기존 관절 이동
→ 정지 확인 → RGB/Depth 취득 → 다음 시점 → 통합·저장** 순서다.
Cartesian 직선 이동(MoveL)은 아니며, IK 실패 시 해당 이동을 수행하지 않고 중단한다.
정합은 목표값이 아니라 촬영 직후의 **실제 관절각과 Xacro TF**를 사용한다.

기본 저장 위치는 `log/YYYYMMDD_HHMMSS_ffffff/`이며 실행 시 전체 경로를 출력한다.
각 시점의 RGB·Depth·점군과 자세 로그, 통합본 `merged.ply`, `manifest.json` 등이 저장된다.
`manifest.json`의 `status: complete`가 정상 완료를 뜻한다.
기본 다중 프레임 모드는 시점마다 초기 30프레임을 버린 뒤 30개 Depth를 취득하고,
픽셀별 최소 3회 유효 측정된 값의 중앙값을 사용한다(현재 YAML 설정 기준).

### 5) 저장된 결과만 시각화

아래 `실행결과폴더명`을 실제 생성된 폴더 이름으로 바꾼다.
`--view`는 **다시 스캔하지 않으며 로봇·카메라에 연결하지 않는다.**

```bash
python -s three_view_scan.py --view ../log/실행결과폴더명/merged.ply

# 시점별 구분색 통합본 (RGB 촬영 시 생성)
python -s three_view_scan.py --view ../log/실행결과폴더명/merged_view_colors.ply

# 특정 시점의 점군: 로봇 베이스 좌표계
python -s three_view_scan.py --view ../log/실행결과폴더명/view_01/base.ply
```

### 선택 사항: 단일 프레임 촬영

여러 프레임을 통합하지 않고 워밍업 후 한 장만 취득하려면 다음 파일을 사용한다.
같은 YAML의 목표 자세·제어 설정을 사용하며 Cartesian 입력도 지원한다.
아래 두 명령은 위의 다중 프레임 실행에 추가로 실행하는 것이 아니라 **대체 실행 방법**이다.

```bash
# 검사만
python -s three_view_scan_single_frame.py --check

# 실제 이동·촬영 (현장 확인 후 SCAN 입력)
python -s three_view_scan_single_frame.py --execute
```

다른 YAML을 사용하려면 실행 명령에 `--config /절대경로/설정.yaml`을 추가한다.

## 1. 목적과 현재 결론

Doosan A0912에 브라켓으로 ORBBEC Femto Bolt를 장착하여 3D Reconstruction을 테스트한다.
브라켓의 로봇 부착면 중앙을 기준 원점으로 정하고, 로봇 TCP 좌표계와 연결하는 구성을 검토한다.

이 문서는 **제조사에서 제공하는 CAD와 TF 관련 자료를 조사한 기록**이다.
실제 브라켓의 CAD, 로봇 TCP 설정, 연결된 카메라의 보정값은 아직 확인하지 않았으며,
아래 수치는 이 장착 구성에 대해 실측·보정된 최종 TCP TF가 아니다.

- 자료 확인일: 2026-09-07
- ORBBEC는 Femto Bolt의 2D 도면, 3D STEP, 센서 좌표계 문서를 제공한다.
- 공식 ROS2 모델에는 하단 나사 장착부를 기준으로 깊이 광학 좌표계까지 연결하는 TF가 있다.
- 따라서 **브라켓 조립 CAD + 제조사의 nominal TF로 초기 TCP ↔ Depth TF를 계산하는 접근이 가능하다.**
- 단, 공식 모델은 해당 값을 **근사값**으로 설명한다. 조립 후 재구성 정밀도 검증이 필요하다.

근거: [공식 CAD 저장소][cad-repo], [공식 ROS2 모델][urdf], [공식 좌표계 문서][coordinates].

## 2. 주요 참고 사이트와 자료

| 자료 | 확인한 내용 | 이번 테스트에서의 용도 |
| --- | --- | --- |
| [Femto Bolt 제품 페이지][product] | 제품 개요와 장착 방식, CAD 제공 안내 | 제품 및 기구 자료의 출처 확인 |
| [Femto Bolt 공식 문서 모음][docs] | 좌표계, 하드웨어 사양, 보정 함수, CAD 다운로드 안내 | 관련 제조사 문서의 진입점 |
| [Hardware Specifications][hardware] | 제품 치수와 기구 자료 저장소 안내 | 브라켓 설계에 필요한 공식 자료 확인 |
| [OrbbecHardware / Femto Bolt][cad-repo] | 2D PDF, 3D STP, 스크린샷, 커넥터 사양 | 카메라 외형과 체결 위치 확인 |
| [공식 CAD 다운로드 ZIP][cad-zip] | STEP 및 PDF·DWG 도면 파일 | CAD 프로그램에서 조립 모델 구성 |
| [Coordinate Systems (iToF Camera)][coordinates] | 깊이 좌표계 원점, 축 방향, 내부 기울기 | 깊이 포인트를 해석할 좌표계 정의 |
| [femto_bolt.urdf.xacro][urdf] | 장착 원점에서 깊이 광학 좌표계까지의 nominal TF | 초기 기구·센서 변환 구성 |
| [Use Azure Kinect Calibration Functions][calibration] | 장치별 보정값 조회와 센서 간 좌표 변환 | 실제 카메라의 센서 간 extrinsics 확인 |

### CAD에서 확인한 파일

[GitHub 하드웨어 저장소][cad-repo]에는 다음 파일이 있다.

- [01_2D PDF/Femto Bolt.pdf][drawing]: 외형, 구멍 및 커넥터 위치를 확인한 2D 도면.
- [02_3D STP/Femto bolt.stp][step]: 브라켓과 조립할 때 사용할 수 있는 3D 모델.

[문서에서 연결한 CAD ZIP][cad-zip]에는 다음 파일이 포함되어 있었다.

```text
01_2D PDF/
  [2D][DWG]Femto Bolt V1.0_20240918.dwg
  [2D][PDF]Femto Bolt V1.0_20240918.pdf
02_3D STP/
  Femto bolt.stp
```

외형 CAD의 원점이나 렌즈 외형 중심이 깊이 데이터의 광학 원점과 같다고 가정하지 않는다.
이번 조사에서 장착부와 센서 좌표계 사이의 수치 연결 근거로 삼은 자료는 **공식 ROS2 모델**이다.

## 3. 깊이 카메라 좌표계

[공식 좌표계 문서][coordinates]에서 확인한 정의는 다음과 같다.

| 항목 | 정의 |
| --- | --- |
| 원점 | 문서에서 camera focal point로 표현한 광학 기준점 |
| +X | 오른쪽 |
| +Y | 아래쪽 |
| +Z | 카메라가 바라보는 전방 |
| 문서상 3D 좌표 단위 | mm |
| 내부 기울기 | 깊이 카메라가 컬러 카메라 쪽으로 아래로 6° 기울어짐 |

축 방향은 카메라 뒤에서 촬영 방향을 바라보는 관점으로 해석한다.
케이스 전면이나 임의의 CAD 원점을 그대로 깊이 원점으로 사용하지 않는다.

문서에는 WFOV 조명기가 깊이 카메라보다 추가로 1.3° 아래로 기울어진다는 설명도 있다.
이는 **조명기의 방향**에 관한 내용이며, 깊이 광학 좌표계에 1.3°를 추가 적용하라는 의미가 아니다.
좌표계 그림은 [제조사 원본 이미지][coordinate-image]에서도 확인할 수 있다.

## 4. 공식 ROS2 모델에서 확인한 TF

출처: [OrbbecSDK_ROS2의 femto_bolt.urdf.xacro][urdf].
아래 프레임은 실제 이름에서 설정별 접두사를 생략한 표기이다.
`base_link`는 **카메라의 하단 나사 장착부 기준**이며 로봇의 베이스가 아니다.

| 부모 → 자식 프레임 | 위치 XYZ [m, 코드 원값] | 위치 XYZ [mm] | 회전 RPY [rad, 코드 원값] |
| --- | --- | --- | --- |
| 카메라 `base_link` → `camera_link` | (0.03645, 0.00198, 0.021) | (36.45, 1.98, 21.00) | (0, 0, 0) |
| `camera_link` → `depth_frame` | (0, 0, 0) | (0, 0, 0) | (−0.0063718, 0.1061316, 0.000231) |
| `depth_frame` → `depth_optical_frame` | (0, 0, 0) | (0, 0, 0) | (−π/2, 0, −π/2) |

두 번째 회전은 약 (−0.365077°, +6.080893°, +0.013235°), 마지막은 (−90°, 0°, −90°)이다.
마지막 변환은 광학 축 규약으로 맞추는 회전이다.

코드는 이 수치가 근사값임을 명시한다. `use_nominal_extrinsics`는 실제 보정 TF를 발행하지
않는 경우 등에 모델의 센서 간 nominal TF를 사용할지 선택한다.
따라서 위 표를 실제 장치별로 보정된 장착 TF로 간주하지 않는다.

`main` 브랜치는 변경될 수 있다. 실제 구현 시 사용하는 파일의 커밋과 SDK 버전을 기록한다.

## 5. 브라켓 CAD와 연결하는 방법

아래 내용은 위 제조사 자료를 바탕으로 한 **이 프로젝트의 적용 방법**이다.

### 기준 좌표계

| 기호 | 의미 | 확보 방법 |
| --- | --- | --- |
| B | 로봇 베이스 좌표계 | 로봇 설정 |
| T | 로봇 TCP 좌표계 | 브라켓의 로봇 부착면 중앙과 일치하도록 설정하는 구상 |
| M | 카메라 하단 나사 장착 기준 좌표계 | 제조사 모델의 카메라 `base_link` |
| D | 깊이 광학 좌표계 | `depth_optical_frame` |

```text
로봇 베이스 B
  └─ 로봇 자세 → TCP T = 브라켓 기준 좌표계
       └─ 브라켓 조립 CAD → 카메라 장착 기준 M
            └─ 제조사 TF 체인 → 깊이 광학 좌표계 D
```

브라켓과 TCP는 원점만 맞출 것이 아니라 **X·Y·Z축 방향까지** 맞춰야 한다.
카메라를 측면 구멍으로 고정하더라도 CAD에서 M까지의 변환을 구해 같은 체인을 사용할 수 있다.
이때 카메라 모델의 장착 좌표계와 CAD에서 정의한 좌표계를 먼저 일치시킨다.

### 변환 합성 규칙

이 문서에서 `T_A_B`는 **B 좌표로 표현된 점을 A 좌표로 변환하는 4×4 행렬**이다.
점은 동차좌표 열벡터를 사용한다.

```text
p_A = T_A_B × p_B

T_T_D = T_T_M × T_M_D
T_B_D = T_B_T × T_T_D
p_B   = T_B_D × p_D
```

- `T_B_T`: 촬영 시점의 로봇 TCP 자세.
- `T_T_M`: 브라켓 조립 CAD에서 계산하는 고정 변환.
- `T_M_D`: 4절의 세 변환을 순서대로 합성한 초기 nominal 변환.

각 변환은 `T = [[R, t], [0, 1]]`로 구성한다. URDF RPY는
`R = Rz(yaw) × Ry(pitch) × Rx(roll)`로 구성하며, 서로 다른 프레임의 RPY를 단순히 더하지 않는다.
CAD, 로봇 자세, 포인트클라우드의 길이 단위는 계산 전에 하나로 통일한다.

로봇에서 읽은 자세가 플랜지 기준인지, 설정된 TCP 기준인지 확인한다.
이미 TCP 자세를 사용하는데 플랜지 → TCP 변환을 다시 적용하면 오프셋이 중복된다.
Doosan의 실제 TCP 등록값과 자세 표현 규약은 구현 단계에서 별도로 확인해야 한다.

## 6. 장치 보정값과 TCP TF의 차이

[제조사 보정 함수 문서][calibration]는 Orbbec SDK K4A Wrapper를 사용하는 경우 다음을 설명한다.

- `k4a_device_get_calibration()`으로 `k4a_calibration_t`를 조회한다.
- 보정 데이터는 장치와 동작 모드에 따라 달라지며, 조회 시 깊이 모드와 컬러 해상도를 지정한다.
- `k4a_calibration_3d_to_3d()`로 깊이·컬러·자이로·가속도계 좌표계 사이의 점을 변환할 수 있다.

이는 **카메라 내부 센서 간 보정 정보**다. 사용자 브라켓과 로봇 TCP 위치를 자동으로 알아내는
기능이 아니므로, 센서 extrinsics만 조회해서 TCP ↔ Depth TF가 완성되는 것은 아니다.
센서 간 변환에 실제 보정값을 사용하더라도 기구 장착 변환은 따로 확보해야 한다.

컬러 좌표계로 변환한 깊이 데이터는 목적 좌표계가 달라진다. 실제 구현에서는 출력
포인트클라우드의 기준 좌표계를 확인하고, D가 아닌 좌표계이면 그에 맞는 TF를 사용한다.
실제 TF 체인에 기울기가 이미 포함되어 있으면 별도로 6° 보정을 중복 적용하지 않는다.

## 7. 초기 테스트 적용 순서와 아직 확인할 사항

아래는 제조사가 정밀도를 보증한 절차가 아니라, 이번 구성에 대한 권장 검증 순서이다.

1. 제조사 STEP과 브라켓 CAD를 조립하고 TCP·장착 기준의 축을 정의한다.
2. `T_T_M`을 계산하고 제조사 nominal `T_M_D`와 합성한다.
3. 실제 사용할 SDK·드라이버의 출력 좌표계, 단위, TF 중복 여부를 확인한다.
4. 우선 로봇을 정지한 여러 자세에서 같은 표면을 촬영하여 베이스 좌표계로 변환한다.
5. 겹치는 표면의 위치·방향 오차를 확인한다. 필요한 정밀도에 못 미치면 핸드아이 캘리브레이션
   등으로 장착 변환을 보정하는 방안을 검토한다.

아직 확보되지 않은 정보:

- 브라켓의 실제 조립 치수와 카메라 체결 방향.
- 로봇에 등록할 TCP 위치·회전 및 로봇 자세 데이터의 규약.
- 실제 카메라의 센서 보정값, 출력 데이터의 좌표계·단위.
- 브라켓 제작·체결 오차와 최종 재구성 오차.

따라서 현재 결론은 **CAD 기반 초기 TF 구성이 가능하다**는 것이며,
**이 자료만으로 최종 스캔 정확도가 보장된다**는 의미는 아니다.

## 8. 조사 중 추가로 확인한 자료

- [ORBBEC 커뮤니티: Femto Bolt RGB Camera Focal Point][forum]: 전면에서 컬러 광학 원점까지의
  치수를 문의한 사용자 게시물이다. 확인 시 제조사의 치수 답변은 없었으며, 깊이 TF 수치의
  근거로 사용하지 않았다.
- [공식 ROS2 카메라 노드 소스][camera-node]: SDK·드라이버 구현을 추가 확인하기 위해 열람한
  자료다. 이 README의 TF 수치 근거는 4절의 URDF이며, 실행 중 발행되는 전체 TF 동작을
  해당 소스만으로 검증 완료했다는 의미는 아니다.

[product]: https://www.orbbec.com/products/tof-camera/femto-bolt/
[docs]: https://doc.orbbec.com/documentation/Orbbec%20Femto%20Bolt%20Documentation
[hardware]: https://doc.orbbec.com/documentation/Orbbec%20Femto%20Bolt%20Documentation/Femto%20Bolt%20Hardware%20Specifications
[cad-repo]: https://github.com/orbbec/OrbbecHardware/tree/main/Femto%20Bolt
[cad-zip]: https://orbbec-debian-repos-aws.s3.amazonaws.com/product/Femto_Bolt_CAD_files.zip
[drawing]: https://github.com/orbbec/OrbbecHardware/blob/main/Femto%20Bolt/01_2D%20PDF/Femto%20Bolt.pdf
[step]: https://github.com/orbbec/OrbbecHardware/blob/main/Femto%20Bolt/02_3D%20STP/Femto%20bolt.stp
[coordinates]: https://doc.orbbec.com/documentation/Orbbec%20Femto%20Bolt%20Documentation/Coordinate%20Systems%20%28iToF%20Camera%29
[coordinate-image]: https://new-orbbec3d-s3.s3.amazonaws.com/wp-content/uploads/2023/12/05121756/202312052.jpeg
[urdf]: https://github.com/orbbec/OrbbecSDK_ROS2/blob/main/orbbec_description/urdf/femto_bolt.urdf.xacro
[calibration]: https://doc.orbbec.com/documentation/Orbbec%20Femto%20Bolt%20Documentation/Use%20Azure%20Kinect%20Calibration%20Functions
[forum]: https://3dclub.orbbec3d.com/t/femto-bolt-rgb-camera-focal-point/4301
[camera-node]: https://github.com/orbbec/OrbbecSDK_ROS2/blob/main/orbbec_camera/src/ob_camera_node.cpp
