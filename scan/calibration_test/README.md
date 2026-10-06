# 오프라인 ICP / Pose Graph 테스트

저장된 각 뷰의 점군과 `pose.json`을 읽어 로봇 TF 초기값 이후의 보정만 비교한다.
카메라/로봇 연결, 재촬영, SDK 호출, 원본 파일 수정은 하지 않는다.
이름이 calibration_test이지만 **hand-eye/로봇 캘리브레이션 파라미터를 변경하는 코드가 아니다.**

## 실행 파일

| 방식 | 실행 파일 | 결과 폴더 (`log/` 아래) |
|---|---|---|
| Point-to-point ICP | `scripts/point_to_point_icp.py` | `point-to-point_ICP/실행시간/` |
| Point-to-plane ICP | `scripts/point_to_plane_icp.py` | `point-to-plane_ICP/실행시간/` |
| Generalized ICP | `scripts/generalized_icp.py` | `generalized_ICP/실행시간/` |
| Colored ICP | `scripts/colored_icp.py` | `colored_ICP/실행시간/` |
| ICP + Pose Graph | `scripts/icp_pose_graph.py` | `icp_pose_graph/ICP방식/실행시간/` |

`scripts/icp_common.py`는 입력/평가/저장을 공유하기 위한 공통부다. 네 파일은 각각 독립 실행한다.
기존 스캔 환경의 NumPy/Open3D를 사용한다. 별도의 모델 다운로드나 설치는 필요 없다.

```bash
conda activate vehicle_painting_robot_scan
cd /home/oms/vehicle_painting_robot/scan/calibration_test/scripts
python point_to_point_icp.py
python point_to_plane_icp.py
python generalized_icp.py
python colored_icp.py
```

각 파일 맨 위의 `CAPTURE_DIR`을 수정하거나 명령행으로 입력을 지정한다.

```bash
python point_to_plane_icp.py \
  --capture-dir /home/oms/vehicle_painting_robot/scan/doosan_a0912_scan_test/log/20260923_220542_796308 \
  --reference-view view_02
```

기본은 manifest의 첫 뷰를 고정한다. 첫 뷰가 항상 정면이라는 뜻은 아니다.
`REFERENCE_VIEW = "view_02"`처럼 원하는 기준 뷰 폴더 이름으로 변경할 수 있다.
비교할 때 네 스크립트의 입력, 기준 뷰, 공통 설정을 동일하게 사용한다.

## 지원 입력

**하나로 합쳐진 PLY가 아니라 뷰별 데이터가 남아 있는 결과 폴더가 필요하다.**

1. `three_view_scan.py`의 완료된 촬영 폴더
   - `manifest.json`
   - 각 승인된 뷰의 `camera_filtered.ply`, `pose.json`
   - `base_link`, m 단위, ICP 미적용 결과만 지원
2. SAM 마스크 Depth 추출 결과 폴더
   - `manifest.json`
   - 각 출력 뷰의 `part_camera.ply`, 복사된 `pose.json`
   - `method: saved_T_base_depth_no_ICP`
   - 폴더를 옮긴 경우에도 현재 결과 폴더 안의 파일을 읽음

Colored ICP에는 실제 RGB가 필요하다. 스캔의 `color_mode: view_colors`는 뷰별 임의 색상이므로
거부한다. RGB가 없는 PLY 역시 거부한다. 다른 세 알고리즘은 색상 없이도 실행할 수 있다.

## 기존 네 ICP 테스트의 보정 구조와 기본 설정

1. 카메라 좌표 PLY에 `T_base_depth`를 한 번 적용해 초기 base 점군 생성
2. 기준 뷰는 원래 로봇 base 자세에 고정
3. 다른 뷰 각각을 **같은 고정 기준 뷰**에 정합 (누적 점군에 순차 정합하지 않음)
4. 구한 추가 변환 `Delta`를 원래 해상도의 해당 뷰 전체에 적용
5. `T_base_depth_corrected = Delta @ T_base_depth_initial` 저장
6. 보정된 점군을 통합하고 별도로 출력 다운샘플링

기존 네 스크립트에는 Pose graph optimization, RANSAC 전역 정합, TSDF, 형상 변형을 적용하지 않는다.
네 방식 모두 동일한 coarse-to-fine 스케일을 사용한다.

| 계산용 voxel | 대응점 최대 거리 | 최대 반복 |
|---:|---:|---:|
| 10 mm | 30 mm | 60 |
| 5 mm | 15 mm | 40 |
| 2 mm | 6 mm | 30 |

이는 테스트 시작값이며 해당 부품의 최적값이나 정확도 보장이 아니다. 겹침/초기 오차/노이즈에 맞춰 조정한다.
Point-to-plane/GICP/Colored는 스케일별로 법선을 추정한다. 기본 반경은 voxel의 3배, 최대 이웃 30개다.
GICP의 epsilon은 0.001, Colored ICP의 형상 가중치는 0.968이다. 기본 손실을 사용하며
별도의 robust loss를 추가하지 않아 알고리즘 간 비교를 단순하게 유지한다.

- `evaluation_voxel_mm=5`, `evaluation_distance_mm=10`: 네 방식과 보정 전후에 고정된 **공통 평가 조건**
- `output_voxel_mm=2`: 최종 통합본 다운샘플링 크기 (정확도 2 mm라는 뜻이 아님)
- `roi_min_mm`, `roi_max_mm`: 초기 base 좌표의 정합/평가 영역. 기본 `None`으로 전체 영역 사용
  - ROI는 **정합/평가용 점만** 제한한다. 출력에는 ROI 밖 점도 같은 보정을 적용하여 유지한다.
  - 배경이 지배하는 경우 부품 주변 ROI 또는 SAM 추출 결과를 사용해 비교할 수 있다.
- OpenMP 스레드는 미지정 시 8개. 외부 `OMP_NUM_THREADS` 값이 있으면 유지한다.

## 저장 파일

- `summary.json`: 입력/해시/설정/기준 뷰/Open3D 버전/처리 시간, 각 뷰의 변환·평가·경고
- `run.log`: 진행 메시지와 Python 오류 출력 (일부 Open3D 네이티브 출력은 터미널에만 나올 수 있음)
- `error.log`: 처리 실패 시 traceback
- `각뷰/registered_base.ply`: 보정 후보 점군, 원래 입력 해상도/RGB 유지
- `각뷰/registration.json`: 초기 TF, 추가 변환, 최종 TF, 보정 전후 평가
- `merged_before.ply`: 기존 TF-only 통합본 (출력 voxel 적용)
- `merged_raw.ply`: ICP 보정 후 통합본 (출력 다운샘플링 전)
- `merged.ply`: ICP 보정 후 통합본 (출력 voxel 적용)
- `merged_before_view_colors.ply`, `merged_view_colors.ply`: 보정 전후 뷰 구분색 통합본

`before` / `after`에는 고정된 평가 점군·거리 기준으로 측정한 양방향 지표가 들어간다.

- `source_to_reference`: 해당 뷰에서 기준 뷰로 평가
- `reference_to_source`: 기준 뷰에서 해당 뷰로 평가
- `fitness`: 출발 점들 중 최대 대응 거리 이내의 대응점이 있는 비율
- `inlier_rmse_mm`: 그 대응점들의 거리 RMSE. 대응점이 없으면 **0이 아닌 null**
- `correspondences`: 대응점 개수
- `stages`: 각 스케일의 계산 시간·결과. 스케일마다 거리 기준이 달라 직접 비교에 주의
- `correction.centroid_shift_mm`: 보정 전후 점군 중심 이동량
- `correction.rotation_deg`: 추가 회전 크기
- `correction.translation_vector_base_mm`: Delta의 이동 벡터. base 원점 기준 회전과 결합된 값으로,
  회전이 있을 때 `translation_norm_mm`은 점군 중심의 실제 이동량과 다르다.

`complete`는 계산/저장 완료일 뿐 물리적 정확도나 수렴 보장이 아니다. 대응점이 없거나 계산이
실패하면 `failed`로 기록하고 최종 통합본을 만들지 않는다. 이미 완료한 개별 뷰는 남을 수 있다.
경고만 있는 경우 `complete_with_warnings`로 **후보 결과를 그대로 저장**한다. 자동 승인/원복은 하지 않는다.
기본 경고: 양방향 fitness < 0.15, 대응점 < 30, fitness 0.02 초과 감소, RMSE 0.1 mm 초과 증가,
중심 이동 > 50 mm, 회전 > 5°. 이것들은 시험용 검토 기준이지 로봇 안전 기준이 아니다.

평평한 문 표면의 옆 미끄러짐, 배경에 맞춘 오정합, 원래 TF의 절대 위치 오차를 별도로 확인해야 한다.
낮은 RMSE만으로 정확성을 확정하지 않는다. 원래 TF와 보정 TF를 모두 보관하며,
기존 클러스터 코드는 ICP 미적용 입력만 받으므로 이 결과를 그 입력에 바로 대체하지 않는다.

## 입력 확인 / 시각화

```bash
python colored_icp.py --check
python point_to_plane_icp.py --visualize
# 기존 결과만 보기. 재촬영/재정합하지 않음. 네 스크립트 모두 동일 옵션 지원.
python point_to_plane_icp.py --view /결과/실행시간폴더 --layer after-colors
python point_to_plane_icp.py --view /결과/실행시간폴더 --layer before-colors
python point_to_plane_icp.py --view /결과/실행시간폴더 --layer after
```

## 회귀 테스트

```bash
python -B -m unittest discover -s /home/oms/vehicle_painting_robot/scan/calibration_test/scripts -p 'test_*.py' -v
```

공식 API: [ICP](https://www.open3d.org/docs/0.18.0/tutorial/pipelines/icp_registration.html),
[GICP](https://www.open3d.org/docs/0.18.0/python_api/open3d.pipelines.registration.registration_generalized_icp.html),
[Colored ICP](https://www.open3d.org/docs/0.18.0/tutorial/pipelines/colored_pointcloud_registration.html).

## ICP + Pose Graph 별도 스크립트

`scripts/icp_pose_graph.py`는 **로봇 TF 초기 배치 → 뷰 쌍별 ICP → PGO → 병합/저장**을 수행한다.
기존 `icp_common.py`의 입력 검증, 네 방식의 다중 스케일 ICP, 평가를 재사용한다.
기존 네 스크립트의 동작과 설정은 바꾸지 않았다. 지원 입력 폴더도 위와 동일하다.

```bash
conda activate vehicle_painting_robot_scan
python /home/oms/vehicle_painting_robot/scan/calibration_test/scripts/icp_pose_graph.py

# ICP 방식 변경. 기본은 point_to_plane.
python /home/oms/vehicle_painting_robot/scan/calibration_test/scripts/icp_pose_graph.py --method generalized

# 계산 후 결과 시각화
python /home/oms/vehicle_painting_robot/scan/calibration_test/scripts/icp_pose_graph.py --visualize
```

파일 위의 `CAPTURE_DIR`, `REFERENCE_VIEW`, `ICP_METHOD`, `SETTINGS`, `GRAPH_SETTINGS`를 수정한다.
명령행 `--capture-dir`, `--reference-view`, `--method`, `--output-root`도 지원한다.
`--check`는 파일/단위/RGB/TF 검증만 하며 겹침 및 정합 가능 여부까지 확인하는 옵션은 아니다.

### 그래프 구성

1. 각 카메라 점군에 원래 `T_base_depth`를 한 번 적용한다. 아직 병합하지 않는다.
2. 모든 뷰 쌍을 한 번씩 검사한다. 세 뷰면 (0,1), (0,2), (1,2) 총 세 쌍이다.
   - 초기 10 mm voxel / 30 mm 대응 거리에서 **양방향** 겹침을 검사한다.
   - 통과한 쌍에 선택한 다중 스케일 ICP를 실행한다.
   - 고정된 5 mm voxel / 10 mm 평가 거리에서 양방향 대응점·fitness·RMSE를 검사한다.
   - 각 쌍의 보정 회전과 양방향 점군 중심 이동량을 검사한다.
3. 검사 통과 간선 중 `min(양방향 fitness) / max(양방향 RMSE_mm, 0.001)` 점수가
   높은 순서로 **모든 뷰를 연결하는 트리**를 구성한다. 촬영 순서에 인접하다고 자동 신뢰하지 않는다.
   - 트리 간선: `uncertain=False` (최적화의 연결 뼈대, 자동 제거하지 않음).
   - 추가 간선: `uncertain=True` (루프 제약, 최적화 중 불일치가 크면 제거될 수 있음).
   - 이 품질 점수/트리 선정은 휴리스틱이다. 부품에 맞은 정합인지 보장하지 않는다.
   - 모든 뷰가 연결되지 않으면 실패 처리하고 최종 통합본을 만들지 않는다.
4. Open3D `global_optimization` / Levenberg–Marquardt로 전체 노드를 함께 조정한다.
   각 간선의 정보 행렬은 최종 ICP 스케일 점군/대응 거리로 계산한다.
   모든 방식에 Open3D의 대응점 기반 정보 행렬을 쓰며, 각 ICP 목적함수의 정확한 Hessian이나
   보정된 센서 공분산을 쓰는 것은 아니다.
5. 기준 뷰의 원래 base 자세는 유지하고 최적화된 변환으로 전체 해상도 점군을 병합한다.
   `merged.ply`에만 최종 2 mm voxel을 적용한다. RGB는 원본 그대로 유지한다.

`GRAPH_SETTINGS` 기본 검사 기준:

- 초기/최종 양방향 `fitness >= 0.15`, 대응점 각각 30개 이상
- 최종 양방향 RMSE 각각 6 mm 이하
- 쌍별 보정의 점군 중심 이동 50 mm 이하, 회전 5° 이하
- Open3D `edge_prune_threshold=0.25`, `preference_loop_closure=1.0`

이는 시작용 실험 값이며 물리적 정확도/로봇 안전 기준이 아니다. 거부된 쌍도 이유와 함께 로그에 남긴다.
정보 행렬이 유효하지 않은 쌍도 제외한다. 기준을 완화하기 전에 원본 TF, 겹침, 배경/ROI를 확인한다.
두 뷰뿐이거나 검사/최적화 후 트리만 남으면 계산은 가능하지만 `complete_with_warnings`로
루프 부재를 알린다. 없는 겹침을 만들어 연결하지 않는다.

### 좌표계 / 결과 해석

- 입력 점군에는 원본 TF가 이미 적용되므로 그래프 노드 pose는 **base에서 추가로 적용하는 Delta**다.
  초기 노드가 모두 항등행렬인 것은 로봇 TF를 무시한 것이 아니라 이미 점군에 적용했기 때문이다.
- 간선 `delta_source_to_target_base`는 초기 base 점군 source를 target에 맞추는 변환이다.
  최적화한 노드 간 관계는 `inverse(Delta_target) @ Delta_source`다.
- 최종 카메라 TF는 `T_base_depth_corrected = Delta @ T_base_depth_initial`이다.
- 로봇 TF를 별도의 prior 제약으로 추가하지 않는다. 기준 뷰 하나를 고정해 좌표계 기준을 유지할 뿐,
  모든 뷰가 로봇 TF에서 일정 오차 이내에 머문다는 보장은 없다. 최종 노드 보정이 크면 경고한다.
- 루프가 있어도 PGO가 항상 각 쌍의 RMSE를 더 줄이는 것은 아니다. 서로 다른 관계를 함께 맞추는
  과정에서 일부 쌍의 잔차가 늘 수 있고, 물리적으로 잘못된 트리 간선을 자동으로 판별하지 못할 수 있다.

출력은 `log/icp_pose_graph/<ICP_METHOD>/<실행시간>/`에 저장한다.

- 기존과 동일한 `merged_before.ply`, `merged_raw.ply`, `merged.ply`, 뷰 구분색 PLY 두 개,
  `각뷰/registered_base.ply`, `각뷰/registration.json`, `run.log`, 실패 시 `error.log`
- `pose_graph_initial.json`: 원래 TF 배치의 노드와 검사 통과 ICP 간선
- `pose_graph_optimized.json`: 최적화된 노드와 살아남은 간선 (Open3D PoseGraph 형식)
- `summary.json`: 전체 설정/입력 해시/버전/기준 뷰/소요 시간/상태,
  모든 뷰 쌍의 승인·거부 이유, before/after_pairwise/after_pgo 평가,
  트리/루프 구분, 최종 간선 유지 여부/confidence, 각 뷰의 초기·최종 TF와 경고

쌍별 평가 키 `source_to_reference`에서 reference는 **그 쌍의 target**이며 항상 고정 기준 뷰는 아니다.
before/after_pairwise/after_pgo는 같은 평가 점군과 거리 기준을 사용한다.
독립적인 pairwise 결과들은 하나의 일관된 배치를 정의하지 않으므로 별도 `merged_icp.ply`는 만들지 않는다.
최적화 입력/출력 PoseGraph JSON과 쌍별 로그로 ICP와 PGO의 영향을 구분한다.

```bash
# 저장된 결과만 보기. 재촬영/재계산 없음.
python /home/oms/vehicle_painting_robot/scan/calibration_test/scripts/icp_pose_graph.py \
  --view /결과/실행시간폴더 --layer after-colors
```

Open3D 공식 [Multiway registration / Pose graph 예제](https://www.open3d.org/docs/release/tutorial/pipelines/multiway_registration.html#pose-graph)를
바탕으로 로봇 TF 초기 배치와 입력 데이터 검증을 추가했다. 예제와 달리 인접 뷰를 무조건 certain으로
취급하지 않고, 검사 통과 간선의 품질 기반 연결 트리를 사용한다.

## 구현 검증 기록 (2026-09-30)

- Open3D 0.18.0 / `vehicle_painting_robot_scan`에서 합성 데이터 회귀 테스트 10개 통과.
  네 알고리즘의 알려진 변환 복원, TF 곱셈 순서, 기준 뷰 고정, 원본/색상 보존,
  ROI 출력 보존, 무대응점 실패, RGB 검증, 이동된 SAM 결과 입력 등을 확인했다.
- `20260923_220542_796308` 실제 스캔을 기본 설정, `view_01` 고정으로 네 방식 모두 실행/저장했다.
- 원본 manifest, 카메라 점군, 자세 파일의 실행 전후 SHA-256이 일치했다.
- 결과 폴더는 각각 `point-to-point_ICP/20260930_205647_691525`,
  `point-to-plane_ICP/20260930_205652_024249`, `generalized_ICP/20260930_205656_620183`,
  `colored_ICP/20260930_205702_144999`이다.
- Colored ICP는 중심 이동이 약 212 mm / 529 mm이고 일부 fitness가 감소하여
  `complete_with_warnings`로 저장되었다. 이 후보를 좋은 정합으로 간주하면 안 된다.
  다른 세 방식의 `complete`도 물리적 정확도 보장은 아니다. GUI 육안 검증은 수행하지 않았다.
- 일부 실행 시간이 겹쳤으므로 이 기록의 처리 시간은 엄밀한 속도 벤치마크가 아니다.

## ICP + PGO 검증 기록 (2026-10-01)

- 기존 10개 + 새 PGO 8개, 총 18개 회귀 테스트 통과 (Open3D 0.18.0).
  새 테스트는 네 ICP 방식의 3뷰 정합, 서로 다른 카메라 좌표와 TF 곱셈 순서,
  첫 뷰가 아닌 기준 뷰 고정, 원본/색상/전체 해상도 보존, 미연결 그래프 거부,
  과도한 보정량 필터, 루프 부재 경고, 품질 기반 트리 선정, 실제 루프 최적화 영향,
  잘못된 루프 제거, 이동된 SAM 입력과 ROI 적용을 확인했다.
- 실제 `20260923_220542_796308` 촬영 결과에 기본 Point-to-plane + PGO 실행 완료.
  결과: `log/icp_pose_graph/point_to_plane/20261001_180506_442417/`.
  세 쌍 모두 검사 통과, 트리 간선 2개 + 루프 간선 1개가 최적화 후에도 유지되었다.
  `view_01` 고정, `view_02` 중심 이동 약 9.38 mm, `view_03` 약 20.17 mm.
  상태는 `complete`이며 원본 manifest/카메라 PLY/pose 7개 파일 해시가 실행 전후 동일했다.
- 약 20.5초는 해당 실행의 전체 처리 시간이며 정밀 성능 벤치마크가 아니다.
- 결과 GUI 육안 검증/실물 정밀도 검증은 하지 않았다. `complete`나 낮아진 RMSE를
  캘리브레이션 정확도 또는 로봇 운전 승인으로 해석하면 안 된다.
