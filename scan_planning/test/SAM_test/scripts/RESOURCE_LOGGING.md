# SAM 3 자원 사용량 로그

`sam3_generate_masks.py`를 평소처럼 실행하면 마스크 실행 폴더에 함께 저장한다.
모델 API import와 모델 로딩 전부터 수집한다. `--check`는 모델을 실행하거나 로그를 만들지 않는다.
별도 모듈 `resource_usage.py`를 같은 디렉토리에 두어야 한다.

- `resource_usage.csv`: 시간별 CPU/RAM/GPU/VRAM 사용량.
- `performance_summary.json`: 수집한 값들의 평균·최대, 단계별 시간, PyTorch VRAM 최대값, 측정 경고.
- `manifest.json`: 위 로그 파일명 및 이미지별 `inference_elapsed_sec`.
- 기존 `mask.png`, `overlay.png`, `mask.json`, `candidates/` 출력은 유지된다.

## 설정과 의존성

스크립트 상단 `RESOURCE_SAMPLE_INTERVAL_SEC = 0.5`로 샘플링 주기를 바꾼다. 최소 0.1초다.
실제 수집 간격은 조회 지연에 따라 길어질 수 있으므로 CSV의 `elapsed_sec`를 기준으로 분석한다.

CPU/RAM에는 `psutil`, GPU 전체 및 프로세스 VRAM 조회에는 NVIDIA 드라이버의 `nvidia-smi`를 사용한다.
PyTorch 메모리 통계는 기존 `torch`를 사용하며, `nvidia-ml-py` 추가 설치는 필요 없다.
`psutil`이 없는 환경에서는 해당 환경에서 `python -m pip install psutil`을 실행한다.
CUDA 장치 UUID로 NVIDIA 조회 대상을 선택하므로 `CUDA_VISIBLE_DEVICES` 설정 시에도 물리 GPU 0으로 고정하지 않는다.

두 스크립트의 `CHECKPOINT_PATH`는 현재 PC에 다운로드된 로컬 가중치를 가리킨다.
이 설정에서는 Hugging Face 다운로드/버전 확인 요청 없이 로컬 파일을 불러온다.
캐시를 삭제하거나 다른 PC로 옮기면 해당 경로를 수정해야 한다. 파일이 없으면 자동 다운로드 대신 오류로 알린다.
`CHECKPOINT_PATH = None`으로 변경하는 경우에만 아래 자동 다운로드/캐시 확인 방식을 사용한다.

## 수치 해석

- `process_cpu_percent`: 이 Python 프로세스의 CPU 사용률. 논리 CPU 하나를 완전히 사용하면 100%,
  여러 코어를 사용하면 100%를 넘을 수 있다. 첫 샘플은 이전 측정이 없어 빈 값이다.
- `process_ram_mib`: 이 Python 프로세스의 상주 메모리(RSS).
- `gpu_utilization_percent_device`, `gpu_memory_used_mib_device`: 선택 GPU 전체의 사용률과 VRAM.
  다른 프로그램의 사용량도 포함되므로 SAM 3만의 사용률이라고 해석하지 않는다.
- `gpu_memory_used_mib_process`: NVIDIA가 보고한 현재 Python PID의 VRAM. CUDA 컨텍스트 등도 포함된다.
- `torch_allocated_mib`: PyTorch 텐서에 현재 할당된 VRAM.
- `torch_reserved_mib`: 캐싱 할당자가 확보한 VRAM. allocated와 더하지 않는다.
- `torch_peak_allocated_mib`, `torch_peak_reserved_mib`: 단계마다 카운터를 초기화하여 기록한 최대값.
  요약 최상위에는 각 단계 최대값 중 최대를 기록한다. PyTorch가 관리하지 않는 메모리는 포함되지 않는다.

메모리 단위는 MiB(1,048,576 바이트)다. CPU/RAM/GPU의 CSV 샘플 최대값은 순간 피크를 놓칠 수 있고,
PyTorch 단계별 피크는 샘플링 대신 할당자 최대값 카운터로 측정한다.
주기 수집 실패·지원되지 않는 지표는 CSV 빈 칸/JSON `null`로 남긴다. 오류는 요약 `warnings`에 기록한다.
각 샘플의 단계는 조회 시작 기준이므로, GPU 조회 중 다음 단계로 넘어갈 수 있다.

## 단계별 시간

`performance_summary.json`의 `phases`에 다음 단계와 이미지 경로가 기록된다.

1. `import_model_api`: PyTorch/SAM 3 API 로딩 및 CUDA 확인.
2. `model_load`: 모델과 프로세서 생성. 필요하면 가중치 다운로드 시간도 포함된다.
3. `image_load`: 각 RGB 이미지 읽기/변환.
4. `inference`: 이미지 인코딩과 텍스트 프롬프트 추론을 합친 시간.
5. `save_result`: 결과의 CPU 전송, 마스크·오버레이·메타데이터 저장.

CUDA 단계는 전후 동기화하여 GPU 작업 완료까지 측정한다. 첫 이미지 추론은 초기 준비 시간 때문에
후속 이미지보다 느릴 수 있다. 별도 워밍업 추론은 추가하지 않는다.
사용량 측정 스레드와 동기화의 오버헤드는 포함되므로 순수 모델 연산 벤치마크와는 다르다.

SAM 3 이미지 추론은 [공식 이미지 예제](https://github.com/facebookresearch/sam3/blob/main/examples/sam3_image_predictor_example.ipynb)처럼
`torch.autocast(..., dtype=torch.bfloat16)`로 이미지 인코딩과 텍스트 추론을 함께 실행한다.
내부 BF16 결과와 FP32 가중치의 자료형 불일치를 방지하며, 모델 가중치를 직접 변환하거나
SAM 3 원본을 수정하지 않는다. BF16 출력은 NumPy 저장 시에만 FP32로 변환한다.

정상 완료, Python 예외, Ctrl+C에는 수집된 CSV와 요약이 남고 `status`가 표시된다.
강제 종료(SIGKILL), 전원 차단, 디스크 기록 실패 시에는 요약 저장을 보장하지 못한다.
원본 스캔 데이터, 카메라 SDK, SAM 3 모델 코드는 변경하지 않는다.

## EfficientSAM3 실행

`efficientsam3_generate_masks.py`도 같은 `resource_usage.py`로 동일한 로그를 기록한다.
입력 목록·프롬프트·마스크 선택·오버레이 저장은 SAM 3 스크립트와 같은 구조이며,
출력만 `scan/sam_test/log/efficientsam3_masks/실행일시/`로 분리된다.

```bash
conda activate vehicle_painting_robot_efficientsam3
python /home/oms/vehicle_painting_robot/scan/sam_test/scripts/efficientsam3_generate_masks.py
```

기본 가중치는 [공식 EfficientSAM3 TV-M 전체 모델](https://huggingface.co/Simon7108528/EfficientSAM3/tree/main/efficientsam3_ft)의
`efficientsam3_tinyvit.pt`다. 현재는 `CHECKPOINT_PATH`에 지정한 로컬 가중치를 사용한다.
`CHECKPOINT_PATH = None`이면 Hugging Face 다운로드/캐시 확인을 사용하며 원본 `facebook/sam3` 모델은 요청하지 않는다.
`--check`는 입력/API/CUDA만 확인하며 가중치 다운로드나 추론은 하지 않는다.

마스크 메타데이터의 `schema: sam3_rgb_mask_v1`는 모델명이 아니라 기존 파일 규격명이다.
실제 모델명·구조는 `model`에 별도로 기록하고, 생성한 `mask.png` 및 후보 마스크는 기존
`extract_masked_depth.py`의 `MASK_PATHS`에 그대로 넣을 수 있다.
SAM 3와 성능을 비교할 때에는 모델 로딩/다운로드와 추론 시간을 구분하고,
가상환경별 PyTorch/CUDA 버전 및 첫 이미지 워밍업 차이를 함께 확인한다.

참고: [psutil](https://psutil.io/),
[NVIDIA SMI](https://docs.nvidia.com/deploy/nvidia-smi/index.html),
[PyTorch 메모리 피크](https://docs.pytorch.org/docs/2.10/generated/torch.cuda.memory.max_memory_allocated.html),
[CUDA 동기화](https://docs.pytorch.org/docs/2.10/generated/torch.cuda.synchronize.html).
