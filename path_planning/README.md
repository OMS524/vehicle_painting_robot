# Path Planning

표면 포인트 클라우드를 입력받아 차량 부품 표면을 따라가는 도장 trajectory를 생성하는 알고리즘이다.

현재 구현은 **axis**와 **surface_adaptive** 슬라이싱을 선택할 수 있다. 생성된 surface row를 보정한 뒤 TCP offset, B-Spline, 공간 간격 샘플링을 거쳐 최종 trajectory CSV를 생성한다. 시간·속도 프로파일은 생성하지 않는다.

## 디렉토리와 실행

```text
path_planning/
├── config/painting_path.yaml
├── scripts/
│   ├── path_planning.py             # 상위 제어기, 설정 로딩, CSV 저장, CLI
│   └── visualize_path.py            # 저장된 PLY/CSV/JSON 시각화
├── src/painting_path_planner/
│   ├── __init__.py                  # 숫자 접두어 모듈 로딩
│   ├── 1_slicing.py
│   ├── 2_correction.py
│   ├── 3_offset.py
│   ├── 4_spline.py
│   ├── structuring.py
│   └── visualization.py             # 공통 Plotly·단계별 디버그 시각화
├── test/                           # 기존 CASES, 입력 데이터, 결과, 검증 코드
├── log/                            # 실행 결과, legacy_csv/는 기존 csv/ 보존본
└── backup/                         # 과거 버전과 구조 변경 전 전체 백업
```

저장소 루트에서 실행한다. 입력 좌표의 길이 단위는 m이다.

```bash
conda activate vehicle_painting_robot_path_planning
cd /home/oms/vehicle_painting_robot

# 한 부품 경로 생성: log/<실행일시>/에 CSV, 사용 설정과 디버그 HTML 저장
python -s path_planning/scripts/path_planning.py \
  --input path_planning/test/data/ES300h_trunk.ply \
  --paint-dir 0 0 -1 --scan-dir 0 1 0 --visualize

# 기존 CASES 전체 테스트: 기존처럼 test/csv/<부품>/에 결과 저장
python -s path_planning/test/scripts/test.py

# 저장된 결과 시각화만 수행: 경로 재생성 없음
python -s path_planning/scripts/visualize_path.py \
  path_planning/test/data/ES300h_trunk.ply \
  path_planning/test/csv/ES300h_trunk/painting_trajectory.csv --open

# 하드웨어 없는 경로 순서 검사
python -s -B path_planning/test/scripts/test_row_order.py
```

`--config`로 다른 YAML, `--output-dir`로 비어 있는 출력 폴더를 지정할 수 있다.
`--open`은 디버그 HTML을 생성하고 브라우저에서 연다. 기본 생성은 브라우저를 열지 않는다.
YAML 파일 이름만 `painting_path.yaml`로 바꾸었으며, 기존 설정과의 호환을 위해
최상위 `painting_trajectory` 키와 파라미터 이름·값은 유지했다.

2026-10-28 구조 변경 전 원본 전체는
`backup/20261028_before_restructure/painting_trajectory_planner/`에 있다.
연결부 수정 전 파일과 기존 AppImage는 같은 백업의 `integration/`에 보존했다.
과거 날짜별 백업과 저장 결과 내부의 이전 경로 문자열은 당시 기록이므로 변경하지 않았다.

## 목표

- 입력 데이터: 차량 부품 표면 포인트 클라우드 (`csv`, `ply`)
- 출력 데이터: row별 도장 trajectory CSV
- 대상 부품: 차량 전체가 아니라 hood, door, bumper, trunk 같은 개별 부품
- 대상 표면: 엣지/얇은 측면보다 넓은 주 표면
- 기본 경로 패턴: raster / zigzag 기반 row trajectory

현재 알고리즘은 표면 도장 경로를 우선 목표로 한다. 엣지 전용 도장 경로는 별도 알고리즘 또는 후처리로 추가하는 것이 맞다.

## 전체 파이프라인

```text
surface point cloud
→ 1. surface spline extraction
→ 2. correction
→ 3. offset
→ 4. spline trajectory generation
→ 5. structuring / csv export
```

실행 순서는 `scripts/path_planning.py`에서 관리하며 알고리즘은 `src/painting_path_planner` 아래에 나뉘어 있다.

```text
1_slicing.py
2_correction.py
3_offset.py
4_spline.py
structuring.py
```

## 1. Surface Spline Extraction

입력 point cloud를 작업 좌표계로 변환한 뒤 일정 간격으로 slice를 생성한다.

작업 좌표계는 다음 방향으로 구성된다.

- `paint_dir`: 분사 방향, gun에서 부품을 향하는 방향
- `scan_dir`: row가 진행하는 기준 방향
- `surface_dir`: 표면에서 gun 쪽으로 향하는 방향

기본 slicing 방식은 축 기반이다.

1. point cloud 평균점을 기준으로 작업 좌표계 생성
2. `spline_start_side`에 따라 slice axis와 row axis 결정
3. `spline_spacing` 간격으로 slice 위치 생성
4. 각 slice 위치에서 `spline_half_width` 범위 안의 점군 추출
5. 추출한 점들의 slice axis 값을 slice 중심값으로 고정

출력은 `slice_profiles`이며 각 profile은 다음 데이터를 가진다.

```text
slice_position
slice_points_work
slice_points_world
```

`surface_extraction.slicing_method`에서 `axis` 또는 `surface_adaptive`를 선택한다.

1. point cloud를 작업 좌표계로 변환
2. PCA 기반 local surface normal 추정
3. point cloud를 `(row axis, slice axis)` occupancy cell로 나누고 `guide_extension_min_bin_points`개 이상인 cell만 유효 처리
4. 각 row strip의 cell이 양옆 strip 중 하나 이상에서도 반복되는지 검사해 고립 노이즈 제거
5. 중간의 빈 cell은 구멍으로 허용하고, 안정적으로 지지되는 최하단·최상단 범위가 가장 긴 row strip 선택
6. 선택한 strip과 양옆 strip의 모든 점에서 upper hull을 추출해 최종 guide 생성
7. 최종 guide point의 tangent를 법선으로 사용하는 local slicing plane 생성
8. 각 local slicing plane의 `± geodesic_band_half_width` slab 안에 있는 모든 표면 점 추출
9. 점이 `geodesic_min_band_points`보다 적을 때만 slab 폭을 단계적으로 확장
10. 추출한 전체 점군을 local plane 좌표계의 correction 단계로 전달

슬라이싱 평면 안의 점군이 중앙, 좌측, 우측처럼 여러 연결 성분으로 나뉘어 있어도 모두 `slice_points_work/world`에 보존한다. 가이드가 포함된 연결 성분만 선택하지 않는다.

`surface_adaptive` mode의 `slice_position`은 가이드라인 진행 순서를 나타내며 보정 슬라이스는 인접 원본 사이의 보간값을 갖는다. 실제 기본 row 간격은 guide의 아크길이 `spline_spacing`으로 결정하고, 첫 슬라이스는 `spline_start_offset`부터 시작한다. 가이드라인 자체는 자르지 않는다.

## 2. Correction

각 slice에서 추출한 점군을 하나의 표면 row로 정리하고 보정한다.

`axis` mode의 correction은 축 기반 slice를 전제로 한다. 각 slice point를 작업 좌표계의 2D 평면으로 투영한다.

```text
u = row_axis 좌표
w = z 좌표
points_uw = (u, w)
```

### 2.1 Endpoint / Component Path 추출

1. slice 점군을 `(u, w)` 평면에 투영
2. 가까운 중복점을 `quantize_step` 기준으로 정리
3. 각 점을 최근접 이웃 `k`개와 연결해 kNN graph 구성
4. graph connected component 분리
5. 각 component에서 Dijkstra 기반 longest path 추출
6. 너무 짧은 component는 노이즈로 제거
7. component를 left/top 기준으로 정렬
8. 정렬된 component를 순서대로 병합
9. 병합된 path의 시작점과 끝점을 전체 endpoint로 사용

현재 component 병합은 다음 backtrack 검사를 포함한다.

```text
다음 component 시작점이 현재 끝점보다 왼쪽으로
component_merge_backtrack_tolerance 이상 돌아가면 해당 component를 건너뜀
```

### 2.2 Convex Hull Chain 선택

1. slice 점군의 2D convex hull 계산
2. endpoint와 가장 가까운 hull point를 각각 찾음
3. 두 hull point 사이의 후보 chain 두 개 생성
4. endpoint chord 기준으로 더 위쪽에 놓이는 chain을 선택
5. 선택된 chain을 correction 기준 upper hull로 사용

### 2.3 Anchor / Free 구간 판단

선택된 hull chain을 일정 간격으로 샘플링하고, 각 샘플이 실제 표면 점군의 지지를 받는지 판단한다.

- anchor: 주변 표면 점과 충분히 가까운 샘플
- free: 주변 표면 점이 없어 부드럽게 연결해야 하는 샘플

### 2.4 Quadratic Optimization

upper hull 샘플 전체를 최적화 변수로 두고 보정 곡선을 계산한다.

목적함수는 다음 항으로 구성된다.

```text
endpoint 유지
anchor 유지
1차 차분 유지
2차 차분 최소화
```

최종 목적함수:

```text
minimize
  lambda_end     * endpoint_error
+ lambda_data    * anchor_error
+ lambda_stretch * first_difference_error
+ lambda_bend    * second_difference_error
```

출력은 다음 debug 데이터를 포함한다.

```text
corrected_rows
corrected_row_profiles
correction_observed_path_rows_world
correction_connected_component_rows_world
correction_convex_hull_rows_world
correction_selected_hull_rows_world
correction_supported_sample_rows_world
correction_endpoint_points_world
```

`surface_adaptive` mode는 각 local slicing plane 좌표계로 점을 변환한 뒤 동일한 upper-hull correction을 적용한다.

`surface_adaptive` correction 출력은 다음을 포함한다.

```text
corrected_rows
corrected_row_profiles
correction_observed_path_rows_world
correction_connected_component_rows_world
correction_endpoint_points_world
```

## 3. Offset

보정된 surface row를 TCP trajectory 기준 위치로 offset한다.

1. corrected row를 `offset_point_spacing` 간격으로 재샘플링
2. 각 row를 해당 local slicing plane의 `(row axis, other axis)` 좌표로 변환
3. local plane에서 row tangent를 계산하고 방향 일관성 정리
4. tangent를 90도 회전해 row 형상에 수직인 normal 계산
5. v1과 동일하게 row chord 진행 방향으로 normal 부호 결정
6. 계산된 normal 방향으로 `offset_distance`만큼 이동

출력은 다음과 같다.

```text
offset_rows
offset_points_world
offset_row_normals_world
```

`axis`와 `surface_adaptive` 모두 동일한 row 형상 기반 offset을 사용한다. `axis`에서는 local slicing plane이 작업 기준축과 일치하므로 v1 방식과 동일하고, `surface_adaptive`에서는 회전된 각 local slicing plane 안에서 같은 계산을 수행한다.

## 4. Spline Trajectory Generation

offset row를 입력받아 일정한 공간 간격의 trajectory를 생성한다.

각 row 처리 순서:

1. offset row anchor point 검증
2. offset normal의 반대 방향을 분사 방향으로 사용
3. anchor point 누적 거리 기준 interpolating B-Spline 생성
4. dense sampling으로 spline arc-length table 생성
5. 시작점/끝점 tangent 방향으로 extension 생성
6. extension 포함 전체 row의 arc-length table 생성
7. 전체 길이를 `final_point_spacing` 고정 아크길이 간격으로 샘플링하고 짧은 마지막 잔여 구간은 제외
8. 각 누적 거리 `s`에서 position 계산
9. 각 `s`에서 분사 방향 보간
10. 각 `s`에서 tangent 계산
11. tangent와 분사 방향으로 TCP orientation quaternion 계산
12. 표면 hit 검사
13. core 구간과 surface hit 결과를 합쳐 paint mask 생성
14. row별 trajectory 배열 저장

`raster_zigzag`가 켜져 있으면 row 진행 방향을 한 줄씩 번갈아 뒤집는다. 이때 position, tangent, orientation, paint mask도 함께 뒤집힌다.

## 5. Structuring / CSV Export

4단계에서 생성한 row별 trajectory 배열을 dataclass로 구조화하고 CSV로 저장한다.

슬라이싱에서 정한 진행 순서와 보정 슬라이스 삽입 순서, 4단계의 지그재그를 그대로 유지한다. 좌표 중앙값으로 다시 정렬하지 않는다.

```python
@dataclass(slots=True)
class TrajectoryPoint:
    s: float
    position: Vec3
    orientation: Quat
    paint: bool


@dataclass(slots=True)
class TrajectoryRow:
    row_index: int
    points: list[TrajectoryPoint] = field(default_factory=list)


@dataclass(slots=True)
class PaintTrajectory:
    frame_id: str
    rows: list[TrajectoryRow] = field(default_factory=list)
```

CSV에는 다음 정보가 저장된다.

```text
row_index
point_index
frame_id
s
position
orientation
paint
```

## 주요 파라미터

기본 파라미터는 `config/painting_path.yaml`에서 관리한다.

```text
surface_extraction.spline_spacing
surface_extraction.spline_half_width
surface_extraction.slicing_method
surface_extraction.geodesic_normal_neighbor_count
surface_extraction.geodesic_band_half_width
surface_extraction.guide_extension_min_bin_points
surface_extraction.geodesic_min_band_points
correction.endpoint.graph_neighbor_count
correction.endpoint.component_merge_backtrack_tolerance
correction.support.support_point_spacing
correction.optimization.lambda_*
offset.offset_point_spacing
offset.offset_distance
spline.spline_point_spacing
spline.final_point_spacing
spline.endpoint_extension_length
spline.raster_zigzag
```

## 현재 한계

`axis`는 door, hood, bumper처럼 기준축 slicing이 표면 흐름과 잘 맞는 부품에서 안정적이다. 하지만 곡률이 크고 표면 흐름이 축과 크게 어긋나는 부품에서는 row가 표면을 자연스럽게 따라가지 못할 수 있다.

`surface_adaptive`는 이 문제를 줄이기 위한 형상 대응 방식이며 다음 특성이 있다.

```text
인접 strip에서 반복 지지되는 occupancy cell로 고립 노이즈 제거
내부 빈 구간은 부품의 구멍으로 허용
위·아래 유효 범위가 가장 긴 strip 주변에서 upper hull guide 생성
guide tangent 기준 local slicing plane 생성
```

현재 선택식 구조는 다음과 같다.

```yaml
surface_extraction:
  slicing_method: axis   # axis | surface_adaptive
```

이렇게 하면 부품별로 고정 축 기반 slicing과 형상 대응 slicing을 선택할 수 있다.
