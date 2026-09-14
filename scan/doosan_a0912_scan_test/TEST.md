# 1. 카메라 입력 테스트

```bash
conda activate vehicle_painting_robot_scan

SDK_DIR="$(python -s -c 'import importlib.util, os; print(os.path.dirname(importlib.util.find_spec("pyorbbecsdk").origin))')"

cd "$SDK_DIR"
```

Linux에서는 최초 한 번 Orbbec USB 접근 권한 규칙을 설치한다. 아래 명령은
SDK에 포함된 규칙을 `/etc/udev/rules.d/99-obsensor-libusb.rules`에 설치하고
udev 규칙을 다시 적용한다. `sudo` 비밀번호 입력이 필요하다.

```bash
sudo bash shared/install_udev_rules.sh
```

설치 후 **카메라 USB 케이블을 분리했다가 다시 연결**하고 실행한다.

```bash
python -s examples/quick_start.py
```

`Pipeline()`에서 `usbEnumerator openUsbDevice failed!`가 나오면
`$SDK_DIR/Log/OrbbecSDK.log.txt`를 확인한다. 카메라가 인식되면서
`Access denied (insufficient permissions)`가 기록된다면 USB 접근 권한 문제다.
위 규칙 설치와 USB 재연결 후 다시 확인한다.

여기서 `-s`는 `~/.local`의 패키지가 Conda 환경의 NumPy/Open3D보다 먼저
로딩되는 것을 막는다. USB 권한은 위 udev 규칙으로 별도로 설정해야 한다.

참고: [Orbbec 공식 Linux 환경 설정 안내](https://github.com/orbbec/pyorbbecsdk/tree/v2-main#getting-started).

# 2. xacro visualize

```bash
conda activate vehicle_painting_robot_scan

python -s /home/oms/vehicle_painting_robot/scan/doosan_a0912_scan_test/scripts/visualize_xacro.py
```

# 3. Scan

이동 명령 종료 후 실제 관절 속도와 자세가 안정되면 촬영한다. 목표각과의 차이는
진단용으로 저장하며, 정합 TF는 촬영 직후의 실제 관절각으로 계산한다.
`target_tolerance_deg`는 이동 프로파일 계산용이고 촬영 허용 조건에는 사용하지 않는다.
촬영은 YAML의 `views`에 적힌 순서로 진행한다. `name`과 `joint_deg` 항목을 추가하면
3개를 넘어 원하는 개수의 시점을 촬영한다. 이름은 중복 없이 지정하며 결과 폴더명이 된다.
추가 양식은 `three_view_scan.yaml`의 `views` 아래 주석 예시를 참고한다.
`camera.capture_color: true`이면 RGB도 촬영하여 `merged.ply`에 실제 색상을 저장한다.

```bash
conda activate vehicle_painting_robot_scan
cd /home/oms/vehicle_painting_robot/scan/doosan_a0912_scan_test/scripts

python -s three_view_scan.py --check
python -s three_view_scan.py --execute
```

```bash
python -s three_view_scan.py --view ../log/촬영디렉토리/merged.ply

# 시점별 구분색으로 정합 상태 비교 (컬러 촬영 시 함께 저장)
python -s three_view_scan.py --view ../log/촬영디렉토리/merged_view_colors.ply
```
