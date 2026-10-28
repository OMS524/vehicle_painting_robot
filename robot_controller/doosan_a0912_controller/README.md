








# Doosan A0912 제어기

`paint_cell`에서 복사한 제어기에 스캔용 실측 조회/IK API를 추가했습니다.
기존 CSV 궤적/속도 제어는 유지하며, `paint_cell` 원본과 제조사 SDK 소스는 변경하지 않습니다.

## 현재 빌드 방법

현재 `vehicle_painting_robot_control` 환경의 Eigen/Pinocchio를 사용합니다.
복사 전 빌드 캐시에 다른 절대 경로가 있다면 기존 `test/build`를 백업/이동한 후 빌드하세요.
SDK `API-DRFL`은 프로젝트에 고정된 Git 서브모듈입니다.

```bash
cd /home/oms/vehicle_painting_robot
git submodule update --init -- robot_controller/doosan_a0912_controller/API-DRFL
conda activate vehicle_painting_robot_control
cmake -S robot_controller/doosan_a0912_controller/test \
  -B robot_controller/doosan_a0912_controller/test/build \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$CONDA_PREFIX" \
  -DDOOSAN_DRCF_VERSION=2
cmake --build robot_controller/doosan_a0912_controller/test/build -j2
```

생성 파일: `test/build/libdoosan_controller_c_api.so`.
현재 빌드는 SDK와 제어 Conda 환경의 라이브러리 경로를 RUNPATH에 포함하므로
실행 전에 `LD_LIBRARY_PATH`를 수동 설정할 필요가 없습니다.
환경을 다른 위치로 옮기면 다시 빌드하세요. DRCF_VERSION=2는 기존 설정을 유지한 값입니다.

## 스캔용 Python API

`test/scripts/joint_target_test.py`를 import해서 `DoosanRobot(library_path=...)`로 사용합니다.
인자를 생략하면 이 제어기의 `test/build` 라이브러리를 사용합니다.
래퍼 파일을 직접 실행하면 기존 이동 테스트가 실행되므로, import와 구분하세요.

| 메서드 | 결과/단위 |
| --- | --- |
| `read_actual_joint_state()` | 실측 `joint_deg`, `joint_velocity_deg_s`, `motion_running` |
| `read_actual_cartesian_state()` | BASE 기준 실측 `flange_pose_mm_zyz_deg`, `tcp_pose_mm_zyz_deg` |
| `solve_closest_ik(pose_mm_zyz_deg, reference_joint_deg)` | `target_joint_deg`, `solution_space`, `sdk_status` |

자세는 XYZ **mm + Euler ZYZ deg**입니다. RPY를 그대로 전달하지 않습니다.
스캔 코드가 카메라 목표 RPY/장착 TF를 활성 TCP 기준 SDK 목표로 변환합니다.
Cartesian 조회 두 건은 순차 조회이므로 스캔 worker가 전후 관절 정지를 확인합니다.
조회가 실패하면 목표 자세나 0 속도로 대체하지 않고 오류를 발생시킵니다.

IK는 기존 C++의 0~7 solution-space 탐색을 사용하고, 주어진 기준 관절각과
L2 거리가 가장 가까운 정상 해를 반환합니다. SDK `ref_pos_opt=1`은 각 분기의
다회전 해를 실제 현재 관절각 기준으로 선택합니다. 따라서 스캔은 정지한 현재 각도를
reference로 사용합니다. 이 함수 자체는 이동하거나 활성 TCP를 변경하지 않습니다.
SDK status 1/2(작업영역 밖/손목 특이점)는 `IKNoSolutionError.sdk_status`로 전달합니다.
연결 실패·잘못된 입력·비정상 SDK 응답은 일반 오류이며 미해결 IK로 숨기지 않습니다.

함수 의미 참고: [두산 IK](https://manual.doosanrobotics.com/en/programming-manual/3.2.1/publish/ikin-pos-sol_space-ref-ref_pos_opt-iter_threshold),
[플랜지 자세](https://manual.doosanrobotics.com/en/programming-manual/3.2.1/publish/get_current_tool_flange_posx-ref),
[모션 상태](https://manual.doosanrobotics.com/en/programming-manual/3.4.0/publish/check_motion).
실제 C++ 호출 시그니처는 고정된 `API-DRFL/include/DRFLEx.h`를 따릅니다.

## 로봇 연결 없이 검사

```bash
cd /home/oms/vehicle_painting_robot
conda activate vehicle_painting_robot_control
python -B -s -m unittest discover -s robot_controller/doosan_a0912_controller/test/tests -v
python -B -s scan_planning/src/scan_planner/robot_worker.py \
  --wrapper robot_controller/doosan_a0912_controller/test/scripts/joint_target_test.py \
  --library robot_controller/doosan_a0912_controller/test/build/libdoosan_controller_c_api.so \
  --check </dev/null
```

검사는 로봇에 연결하거나 서보를 켜지 않습니다. 실제 장비의 통신·측정·IK 성공 및
충돌 안전 검증을 대신하지 않습니다. 스캔은 별도 스캔 환경에서 실행하고 제어 worker만
제어 환경을 사용합니다. 연결 경로는 `scan_planning/config/scan.yaml`에 지정되어 있습니다.

변경 전 복사본/기존 빌드 백업: `backup/20261028_before_scan_api/`.

## 이전 시스템 설치/개발 메모

아래는 복사본의 기존 메모입니다. 현재 Conda 빌드에는 위의 **현재 빌드 방법**을 사용하며,
아래 `/opt/openrobots` 설치·환경변수 설정을 추가로 적용하지 않습니다.

## Eigen
Eigen 설치
```bash
sudo apt update
sudo apt install libeigen3-dev
```

## Pinocchio
Pinocchio 설치
```bash
sudo apt install -y lsb-release curl
sudo mkdir -p /etc/apt/keyrings
curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | sudo tee /etc/apt/keyrings/robotpkg.asc
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | sudo tee /etc/apt/sources.list.d/robotpkg.list
sudo apt update
sudo apt install -y robotpkg-py3*-pinocchio
```

/opt/openrobots 환경변수 설정
```bash
export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
```

```bash
cd /home/oms/vehicle-painting-robot/robot_controller/doosan_a0912_controller/test
CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH cmake -S . -B build
cmake --build build -j4
```



지금 내가 계획하는게
현재 경로는 원하는 속도 x 100Hz로 나온 간격만큼 샘플링 했고 일단 선속도, 각속도, 사다리꼴 프로파일 등 구해서 트레젝토리를 만들긴했어
이걸 이용해서 NLP를 통해 최적값을 뽑고 TCP가 일정속도로 이동하게끔 제어하는건데 현재 로봇 제어는 리얼타임 주기가 1000Hz로 하고 있어
근데 NLP 결과가 포지션이여서 포지션 컨트롤 할려했는데 API 메뉴얼에서 포지션 컨트롤 API가 안정화가 안돼서 밸로시티 컨트롤 API 사용하라고 권장하더라고 그래서 밸로시티 컨트롤을 할 예정이야


[Offline]

1. TCP trajectory 준비
   - 기존 spline path X(s)
   - 기존 profile s(t), sdot(t), sddot(t)
   - CSV의 X_i 사용
   - 각 X_i에 대응되는 s_i도 확보

2. NLP 최적화
   Optimization variable:
   q_0, q_1, ..., q_N

   목적함수:
   1. Joint jump 최소화
      Σ ||q_i+1 - q_i||²

   2. Joint acceleration / smoothness 최소화
      Σ ||q_i+1 - 2q_i + q_i-1||²

   3. Manipulability 최대화
      -Σ w(q_i)

   constraint:
   1. Trajectory tracking
      FK(q_i) = X_i

   2. Joint position limit
      q_min ≤ q_i ≤ q_max

   3. Joint velocity limit
      |q_i+1 - q_i| / 0.01 ≤ qdot_max

   4. Joint acceleration limit
      |q_i+1 - 2q_i + q_i-1| / 0.01² ≤ qddot_max

   5. Singularity 회피
      w(q_i) ≥ w_min
      또는 σ_min(J(q_i)) ≥ ε

3. NLP 결과 저장
   (s_0, q_0)
   (s_1, q_1)
   ...
   (s_N, q_N)

4. q(s) 구성
   q = q(s)
   dq/ds 계산 가능하게 구성


[1000 Hz command 생성]

1. t_k = k * 0.001

2. 기존 profile에서 직접 계산
   s_k = s(t_k)
   sdot_k = sdot(t_k)

3. joint reference 계산
   q_ref(t_k) = q(s_k)

4. joint velocity reference 계산
   qdot_ref(t_k) = dq/ds(s_k) * sdot_k


[Online RT 1000 Hz]

1. q_actual 읽기

2. qdot_cmd 계산
   qdot_cmd = qdot_ref + Kp(q_ref - q_actual)

3. limit clamp

4. speedj_rt(qdot_cmd)






Constraint:
  position tolerance
  orientation angle tolerance
  joint position limit
  joint velocity limit
  joint acceleration limit

Objective:
  position error
  orientation angle error
  velocity smoothness
  acceleration smoothness
  joint center
  singularity avoidance
