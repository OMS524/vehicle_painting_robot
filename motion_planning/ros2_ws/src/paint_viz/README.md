# paint_viz

ROS 2 Humble 시각화 패키지다. 현재 위치는 `motion_planning/ros2_ws/src/paint_viz`다.
실행·환경·결과 경로는 [상위 README](../../../README.md)를 따른다.

## 역할

- `scene_publisher.py`: 부품 점군과 TCP 경로 마커 발행
- `trajectory_player.py`: 저장된 관절 CSV를 `/joint_states`로 재생
- `cell_pose_info.py`: 셀/프레임 정보 확인
- `launch/`, `rviz/`: 셀과 부품·경로를 RViz에 표시
- 나머지 Python 파일: 기존 ROS 명령을 유지하기 위한 `motion_planner` 호환 진입점

계산 원본은 `motion_planning/src/motion_planner`에서 관리한다.
`config`, `urdf`, `motion_planner`는 각각 공통 설정·모델·계산 코드를 가리키는 상대 링크다.
모델 자산 원본은 `description/paint_cell`에 있고 빌드 시 패키지 share에 설치된다.
따라서 URDF의 `package://paint_viz/urdf/...` URI는 그대로 유효하다.

## 실행

저장소 루트에서:

```bash
bash motion_planning/scripts/setup_ros2.sh
source motion_planning/scripts/activate_ros2.sh
python -s motion_planning/scripts/visualize_motion.py --part ES300h_hood

# 형상만 표시
ros2 launch paint_viz cell_view.launch.py gantry_env:=x7y4z3 gui:=true
```

빌드 도구는 `vehicle_painting_robot_motion_planning` 환경의 colcon을 사용한다.
ROS 노드/launch/설치 자산 변경은 재빌드한다. 활성화 스크립트는 순수 계산 소스를 우선 참조한다.

## 유지된 ROS 명령

| 명령 (`ros2 run paint_viz ...`) | 실제 계산 모듈 |
| --- | --- |
| `solve_viz_ik` | `motion_planner.kinematics` |
| `solve_dp_ik` | `motion_planner.dp_ik` |
| `stitch_transitions` | `motion_planner.transition` (`3_transition.py`) |
| `check_collisions` | `motion_planner.collision` |
| `build_sequence` | `motion_planner.sequence` (`4_sequence.py`) |
| `gantry_env` | `motion_planner.configuration` |
| `dock_path` | `motion_planner.dock_path` |
| `make_primitive_urdf`, `make_gantry_urdf`, `make_dock_urdfs`, `split_gantry_stl` | 동명의 `motion_planner` 모듈 |

각 명령의 기존 옵션은 `--help`로 확인할 수 있다.
기존 전이·IK·도킹 알고리즘 설명과 과거 실측 수치는
[정리 전 문서](../../../backup/20261028_before_directory_restructure/motion_planning/paint_viz/README.md)에 보존했다.
그 문서의 옛 파일 경로와 실행 명령은 현재 구조에 그대로 적용하지 않는다.
