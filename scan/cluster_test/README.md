# DBSCAN 부품 점군 분리 테스트

SAM이나 학습 모델 없이 저장된 3D 정합 점군의 공간 연결성을 이용한다.
카메라/로봇을 연결하지 않으며 기존 스캔·경로 생성 알고리즘과 원본 파일은 수정하지 않는다.

## 실행

```bash
conda activate vehicle_painting_robot_scan
python /home/oms/vehicle_painting_robot/scan/cluster_test/scripts/extract_clustered_part.py --visualize
```

기본 입력은 `scan/doosan_a0912_scan_test/log/20260923_220542_796308`이다.
스크립트 상단 `CAPTURE_DIR` 또는 `--capture-dir`로 다른 촬영 결과를 지정한다.
필요한 파일은 `manifest.json`, `merged.ply`, 승인된 각 뷰의 `pose.json`이다.
Open3D와 NumPy를 사용하며 기존 스캔 환경에 설치되어 있다. SAM 환경이 아니다.

```bash
# 입력/자세/의존성 확인만 수행. 군집 계산/저장은 하지 않음.
python scan/cluster_test/scripts/extract_clustered_part.py --check

# 이미 저장된 결과만 표시. 다시 추출하거나 스캔하지 않음.
python scan/cluster_test/scripts/extract_clustered_part.py --view /결과/폴더
python scan/cluster_test/scripts/extract_clustered_part.py --view /결과/폴더 --layer clusters
python scan/cluster_test/scripts/extract_clustered_part.py --view /결과/폴더 --layer part
```

`--layer selection`은 선택 집합을 초록색, 나머지를 회색으로 표시한다.
`clusters`는 집합마다 다른 색, `part`는 선택된 입력 점의 원래 RGB를 표시한다.
빨간 구는 추정 기준점, 하늘색 선은 각 카메라의 실제 광축이다. 직선이 정확히 한 점에서
교차하지 않으므로 선과 구 사이에 오차가 있을 수 있다.

## 처리 방법

1. 동일 촬영의 `base_link` 좌표 점군(m)과 `T_base_depth`를 읽는다.
2. Depth optical frame의 **+Z축**을 시선으로 사용한다. 각 시선까지 수직거리 제곱합을
   최소로 만드는 위치를 선택 기준점으로 구한다. 로봇 flange 축을 사용하지 않는다.
3. 계산용 점군을 5 mm voxel로 다운샘플링하고 DBSCAN을 수행한다.
4. 최소 크기 이상인 집합 중 **voxel 점들의 평균 중심**이 기준점에 가장 가까운 집합을 선택한다.
   집합의 가장 가까운 한 점만 비교하면 큰 벽/바닥 집합이 유리해질 수 있어 중심을 비교한다.
5. 선택 라벨을 voxel의 정확한 소속 인덱스로 입력 점군에 되돌려 저장한다.
   원본에 존재하지 않는 점을 추가하거나 색/좌표를 보간하지 않는다.

시선이 거의 평행하거나 기준점이 카메라 뒤에 있거나 시선 오차가 너무 크면 자동 선택하지 않는다.
이 경우 군집 결과와 사유를 저장하고 `status: needs_selection`으로 표시한다. 부품 파일은 생성하지 않는다.
수동 기준점 또는 집합 번호를 지정해 재실행할 수 있다. 임의의 최대 집합으로 대체하지 않는다.

## 주요 설정

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| `VOXEL_SIZE_MM` | 5 | DBSCAN 계산용 voxel 크기 |
| `DBSCAN_EPS_MM` | 12 | 이웃점 연결 거리. 집합 전체 크기나 중심까지의 거리가 아님 |
| `DBSCAN_MIN_POINTS` | 8 | 핵심점 판정에 필요한 이웃 수. 자기 자신 포함 |
| `MIN_CLUSTER_VOXELS` | 300 | 자동 선택 후보의 최소 voxel 점 개수 |
| `TARGET_POINT_BASE_MM` | `None` | 자동 시선 추정. `[x, y, z]`를 넣으면 수동 기준점 |
| `SELECT_CLUSTER_IDS` | `None` | 자동 선택. `[0]`, `[0, 3]`이면 해당 집합 합집합을 선택 |
| `MAX_RAY_RMS_MM` | 100 | 시선 교차 오차의 허용 RMS |
| `MAX_TARGET_DISTANCE_MM` | 600 | 자동 선택할 집합 중심과 기준점의 최대 거리 |

`eps`가 너무 크면 부품과 거치대/배경이 합쳐지고, 너무 작으면 부품 자체가 조각난다.
`min_points`는 주변 밀도 조건이고 `MIN_CLUSTER_VOXELS`는 집합 전체 크기 조건이다.
입력/출력 PLY 좌표는 **m**, 상단 거리 파라미터와 보고서 중심 좌표는 **mm**다.

## 저장 결과

`scan/cluster_test/log/실행일시/`에 저장한다. 매 실행마다 새 폴더를 만든다.

- `part.ply`: 선택한 입력 점만 저장. 원래 XYZ/RGB 유지.
- `clusters_colored.ply`: 계산용 다운샘플 점군의 집합별 색상.
- `selection_preview.ply`: 선택 집합/배경 비교용 다운샘플 점군.
- `clusters/cluster_000.ply` 등: 자동 선택 가능한 크기의 각 집합. 수동 선택한 작은 집합도 저장.
- `cluster_labels.npy`: 입력 PLY 점 순서의 집합 번호. `-1`은 DBSCAN 노이즈, `-2`는 비유효 XYZ.
- `summary.json`: 원본 경로/해시, 설정, 시선 오차, 집합 크기/중심/색상, 선택 번호와 경고.

`POINT_CLOUD_FILENAME = "merged.ply"`이면 이미 스캔 단계에서 다운샘플된 점을 기준으로 보존한다.
원래 통합 점 전체를 기준으로 추출하려면 `"merged_raw.ply"`로 변경한다.
집합 번호는 입력/파라미터/Open3D 버전이 바뀌면 달라질 수 있으므로 이전 실행 번호를 그대로 신뢰하지 않는다.

## 한계와 확인

- **자동 선택은 부품 의미 인식이 아닌 기하학적 추정이다.** 중심이 가까운 거치대나 배경을 고를 수 있다.
- 부품과 거치대가 DBSCAN 연결 거리 안에서 이어지면 같은 집합이 되어 순수 DBSCAN으로 분리하기 어렵다.
- 부품의 창문/구멍은 채우지 않는다. 구멍 때문에 부품이 반드시 분리되지는 않지만, 완전히 끊긴 조각은
  다른 집합이 될 수 있다. 필요하면 여러 번호를 수동 선택한다.
- 큰 평면 제거, ROI 자르기, ICP, 공간 보간은 자동 적용하지 않는다. 평평한 문 부품을 잘못 삭제하지 않기 위함이다.
- 이 코드는 **부품 PLY 추출용**이다. 각 뷰의 마스크/Depth 파일을 새로 만들지는 않는다.
- 첫 번째와 두 번째 후보의 중심 거리 차이가 50 mm 미만이면 추가 확인 경고를 표시한다.
  `status: complete`는 저장 성공이지 선택 정확도의 보증이 아니다.

```bash
# 그림/summary.json 확인 후 같은 입력과 설정에서 여러 조각을 선택하는 예
python scan/cluster_test/scripts/extract_clustered_part.py --cluster-ids 0 3 --visualize

# 하드웨어 없는 회귀 테스트
python -m unittest discover -s scan/cluster_test/tests -v
```

참고: [Open3D DBSCAN 설명](https://www.open3d.org/docs/release/tutorial/geometry/pointcloud.html#DBSCAN-clustering),
[voxel 원본 인덱스 추적 API](https://www.open3d.org/docs/release/python_api/open3d.geometry.PointCloud.html#open3d.geometry.PointCloud.voxel_down_sample_and_trace).
