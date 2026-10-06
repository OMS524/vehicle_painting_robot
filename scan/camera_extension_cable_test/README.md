# Femto Bolt 연장선 진단

`scripts/camera_cable_diagnostic.py`는 RGB와 Depth를 동시에 켜고 **각 스트림의 SDK 수신 상태**를 기록한다.
로봇은 연결하거나 움직이지 않는다. 장치 펌웨어/SDK 파일/USB 시스템 설정도 변경하지 않는다.
프레임 번호 누락은 애플리케이션 수신 누락 의심값이지 USB 패킷 손실이나 케이블 불량 확정값이 아니다.

## 실행

OrbbecViewer, 기존 스캔 프로그램 등 카메라를 사용하는 다른 프로그램은 먼저 종료한다.

```bash
conda activate vehicle_painting_robot_scan
cd /home/oms/vehicle_painting_robot/scan/camera_extension_cable_test/scripts

# 기본 케이블 직결, 10분 측정. 준비 시간 5초는 통계에서 제외.
python camera_cable_diagnostic.py --label direct --seconds 600

# 케이블을 직접 교체하고 각각 별도 실행
python camera_cable_diagnostic.py --label cable_a --seconds 600
python camera_cable_diagnostic.py --label cable_b --seconds 600

# 영상도 확인: RGB / Depth를 나란히 표시. Q/ESC 또는 창 닫기로 중도 종료.
python camera_cable_diagnostic.py --label cable_a --seconds 600 --visualize

# 제품명, 총 길이, 별도 전원 여부 등을 결과에 기록
python camera_cable_diagnostic.py --label cable_a --note "제품명 / 총 길이 / 별도 전원 여부"
```

인자 없이 실행하면 파일 상단 기본 설정을 사용한다. `--help`로 모든 옵션을 확인할 수 있다.
터미널 `Ctrl+C`로 중단해도 가능한 범위에서 부분 결과를 저장한다. 중도 종료는 정상 시험 완료로 표시하지 않는다.

기본 프로파일은 기존 스캔 기록과 같다.

- RGB: **1920×1080 / MJPG / 30 FPS**
- Depth: **640×576 / Y16 / 30 FPS**
- 새 고유 프레임 무수신 타임아웃: **2초** (`--timeout`)
- 미리보기 갱신: **5 FPS** (`--preview-fps`). 수신 통계는 화면 갱신과 별개로 모든 콜백을 집계한다.
- 기본은 미리보기 없음. 정량 비교에서는 세 케이블 모두 같은 표시 설정/PC 부하를 사용한다.

지원 프로파일만 확인하려면:

```bash
python camera_cable_diagnostic.py --list-profiles
```

이 옵션은 장치를 열어 목록만 읽으며 스트리밍/결과 저장은 하지 않는다. 여러 카메라가 연결돼 있으면
`--serial <시리얼>`로 지정한다. 일반 진단 실행은 요청한 프로파일이 없으면 **실패로 기록**하며
저해상도/낮은 FPS로 자동 변경하지 않는다. 해상도·FPS를 바꿔 비교할 때는 세 조건을 모두 동일하게 바꾼다.

## 측정 방법

- SDK `Sensor.start(profile, callback)`으로 RGB와 Depth를 각각 수신한다.
  두 영상이 모두 모일 때만 결과를 주는 FrameSet 동기화는 사용하지 않는다. 한 스트림이 멈춰도
  다른 스트림을 계속 관측하기 위해서다. D2C/C2D, 3D 점군, 정합, 시간 평균 필터도 적용하지 않는다.
- 콜백에서는 프레임 번호, 장치 타임스탬프, 호스트 monotonic 시각, 해상도/포맷, 바이트 수만 읽어 큐에 넣는다.
  영상 디코딩/파일 저장은 콜백 밖에서 한다.
- 측정 시작은 두 스트림 시작 호출이 끝난 뒤 준비 시간이 지난 시점이다. 준비 프레임도 `frames.jsonl`에
  저장하지만 통계에는 포함하지 않는다. 최초 프레임 번호 이전의 누락 수는 알 수 없다.
- 동일한 직전 `(프레임 번호, 장치 시각)`이 반복되면 중복으로 분리한다. 고유 수신 FPS에서는 제외한다.
  번호 또는 장치 시각이 뒤로 가면 리셋/재정렬로 기록하고 거대한 누락 수를 만들지 않는다.
- 장치 프레임 간격이 정상 주기의 1.5배를 넘으면 `device_gap_events`로 기록한다.
- 호스트 도착 간격이 `max(100 ms, 정상 주기의 2.5배)`를 넘으면 `host_gap_events`로 기록한다.
- 무수신이 2초를 넘으면 `timeout_episodes`를 1회 증가시킨다. 멈춘 상태에서 계속 증가시키지 않으며,
  수신이 회복된 뒤 다시 멈추면 별도 사건이다. 처음부터 영상이 없거나 시험 끝까지 멈춘 구간도 포함한다.
- 장치 시간은 프레임 생성 측의 간격, 호스트 시간은 SDK 콜백 도착 간격이다.
  서로 다른 시계이므로 두 시각을 빼서 카메라→PC 절대 지연으로 해석하지 않는다.
- 로그 큐가 넘치면 계측 자체가 손실된 것이므로 `inconclusive`로 표시한다.
- 미리보기는 각 스트림 최신 프레임만 표시한다. 미리보기 자체가 건너뛴 프레임은 수신 누락으로 세지 않는다.
  마지막 영상이 남아 있더라도 `age`/`STALE / NO DATA`로 신선도를 표시한다.

RGB·Depth 간 시간 동기 정확도와 전체 영상 무결성을 인증하는 코드는 아니다.
`--visualize`를 켠 경우 일부 최신 RGB 프레임만 실제 디코딩하여 실패 횟수를 기록한다.
미리보기 없이 MJPG를 받을 때는 메타데이터/페이로드 존재를 검사하며 모든 JPEG를 디코딩하지 않는다.
Depth 0값은 대상의 반사/거리/가림으로도 생기므로 케이블 오류로 판정하지 않는다.

## 결과 파일

`scan/camera_extension_cable_test/log/YYYYMMDD_HHMMSS_ffffff_케이블이름/`

- `summary.txt`: 사람이 읽기 쉬운 최종 요약
- `summary.json`: 설정, 장치/펌웨어/SDK/Python, 실제 선택 프로파일, 스트림별 누적 통계, 판정 이유
- `frames.jsonl`: 프레임별 수신 메타데이터, 준비/측정 구분, 이상 표시. 영상 픽셀은 저장하지 않음
- `intervals.jsonl`: 약 1초 구간마다 실제 경과 시간과 스트림별 수신 FPS, 큐 크기
- `events.jsonl`: 무수신, 프레임 이상, 장치 연결/분리, SDK 경고·오류, 표시 오류
- `sdk_logs/`: SDK INFO 이상 원본 로그. 최초 SDK import 시 출력이나 네이티브 표준 오류가 모두 포함된다는 보장은 없음
- `usb_tree_before.txt`, `usb_tree_after.txt`: `lsusb -t` 출력. 없는 시스템은 조회 실패 사유만 기록
- `error.log`: 예외 발생 시 Python traceback

`summary.json`의 평균 수신 FPS는 **준비 시간을 제외한 전체 측정 시간**을 분모로 한다.
따라서 중간/마지막에 멈춘 시간도 평균에 반영된다. 화면 FPS나 수신이 잘 된 구간만의 평균이 아니다.
`max_no_unique_frame_ms`는 측정 시작부터 첫 프레임까지, 프레임 사이, 마지막 프레임부터 종료까지 모두 포함한다.
`max_arrival_gap_ms`는 실제 수신된 고유 프레임 사이만 대상으로 한다.

실행 상태와 관측 판정은 구분한다.

- `status=completed`: 지정된 시간 동안 계측 완료
- `status=interrupted`: 사용자 중단, 부분 결과
- `status=failed`: 장치/프로파일/SDK 등 오류
- `assessment=no_anomalies_observed`: 현재 검사 기준에서 관측된 이상 없음. 케이블 합격 인증이 아님
- `assessment=review_required`: FPS 95% 미만, 번호/시각/무수신 이상, SDK 경고 등 검토 대상 존재
- `assessment=inconclusive`: 중도 종료/실패/로그 큐 넘침으로 전체 시험 판정 불가

SDK 경고에는 케이블과 무관한 초기화/종료 경고도 있을 수 있으므로 `events.jsonl`, 원본 SDK 로그와
직결 기준 결과를 함께 본다. 종료 코드는 정상 관측 0, 실패/계측 불가 1, 검토 필요 2, 사용자 중단 130이다.

## 비교 시 유의사항

기본 케이블 직결 / 연장선 A / 연장선 B를 같은 USB 포트·전원·장면·프로파일·PC 부하에서 비교한다.
케이블별로 자동 프로파일을 선택하면 전송량이 달라져 비교가 부정확해질 수 있다.
처음에는 원본 영상 저장/포인트클라우드 생성 없이 수신만 비교하고, 이후 실제 스캔 처리와 겐트리 배선 조건에서 재검증한다.
CPU, SDK 버퍼, 운영체제 스케줄링도 누락/지연의 원인일 수 있다. 이 프로그램은 의심 원인을 자동 단정하지 않는다.

커널 USB reset/disconnect 오류까지 보려면 별도 터미널에서 사용자가 실행한다 (프로그램이 sudo를 실행하지 않음).

```bash
sudo dmesg -w
```

10분 시험에서 문제가 없었다고 장기 안정성/굴곡 수명/EMI 내성을 보장하는 것은 아니다.
실제 겐트리 동작 검증은 설비 안전 절차와 케이블 제조사 허용 굴곡 반경을 따른다.

## 하드웨어 없는 테스트

```bash
python -B -m unittest discover -s /home/oms/vehicle_painting_robot/scan/camera_extension_cable_test/scripts -p 'test_*.py' -v
```

합성 SDK 콜백으로 누락/중복/리셋/타임아웃/한쪽 스트림 정지/부분 시작 실패/중도 종료/로그 저장을 검증한다.
개발 검증 중 실제 카메라 연결·촬영 또는 GUI 표시는 수행하지 않았다. 구매한 연장선의 실측은 사용자가 실행해야 한다.

참고: [SDK Sensor API](https://orbbec.github.io/docs/OrbbecSDKv2/classob_1_1_sensor.html),
[Python SDK Frame API](https://orbbec.github.io/pyorbbecsdk/source/6_API_Reference/frames.html),
[SDK 콜백 처리 권고](https://orbbec.github.io/OrbbecSDK_v2/examples/1.stream.callback/).
