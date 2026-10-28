"""DP-IK 결과 전용 RViz2 launch.

`paint_viz.launch.py` 와 구성은 같지만 **노드 이름과 기본 입력이 다르다**.
DP-IK 결과를 보는 동안 기존 NLP/viz_ik 파이프라인을 건드리지 않으려고 분리했다.

  static_transform_publisher  world -> part   (scene.yaml의 part_xyz/part_rpy)
  robot_state_publisher       URDF + /joint_states -> TF
  dp_scene_publisher          부품 점군, 도장 경로 마커 (part 프레임)
  dp_trajectory_player        solve_dp_ik 가 만든 joint CSV 재생 -> /joint_states
  rviz2                       dp_ik_viz.rviz

⚠️ 이 launch 와 paint_viz.launch.py 를 **같은 ROS 도메인에서 동시에 띄우면 안 된다.**
   노드 이름은 겹치지 않지만 /joint_states 와 /tf 는 공유라 두 로봇이 서로 덮어쓴다.
   NLP 결과와 나란히 비교하려면 한쪽을 다른 ROS_DOMAIN_ID 로 띄울 것.

       ROS_DOMAIN_ID=7 ros2 launch paint_viz dp_ik_viz.launch.py
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from motion_planner.scene_config import load_scene_config
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from motion_planner.configuration import load_gantry_env, repo_root


def _launch_setup(context, *args, **kwargs):
    share = get_package_share_directory("paint_viz")

    scene_file = LaunchConfiguration("scene").perform(context)
    gantry_env_name = LaunchConfiguration("gantry_env").perform(context)
    urdf_file = LaunchConfiguration("urdf").perform(context).strip()
    if not urdf_file:
        # urdf 를 명시하지 않으면 gantry_env 의 메시 URDF (RViz 용)
        env = load_gantry_env(gantry_env_name)
        urdf_file = str(env.urdf_path("mesh"))
        print(f"[launch] gantry_env {env.name}: URDF {urdf_file}")
    joint_csv_dir = os.path.expanduser(LaunchConfiguration("joint_csv_dir").perform(context))
    joint_csv_pattern = LaunchConfiguration("joint_csv_pattern").perform(context)
    rviz_config = LaunchConfiguration("rviz_config").perform(context)
    speed_scale = float(LaunchConfiguration("speed_scale").perform(context))
    use_rviz = LaunchConfiguration("rviz").perform(context).lower() in ("1", "true", "yes")

    scene = load_scene_config(scene_file)   # $REPO/$NLP4CM 치환 + 상대경로 해석 포함

    part_xyz = [float(value) for value in scene.get("part_xyz", [0.0, 0.0, 0.0])]
    part_rpy = [float(value) for value in scene.get("part_rpy", [0.0, 0.0, 0.0])]

    with open(urdf_file, "r", encoding="utf-8") as stream:
        robot_description = stream.read()

    actions = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="dp_world_to_part",
            arguments=[
                "--x", str(part_xyz[0]), "--y", str(part_xyz[1]), "--z", str(part_xyz[2]),
                "--roll", str(part_rpy[0]), "--pitch", str(part_rpy[1]), "--yaw", str(part_rpy[2]),
                "--frame-id", "world", "--child-frame-id", "part",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="dp_world_to_robot_root",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "world",
                "--child-frame-id", LaunchConfiguration("robot_root_frame").perform(context),
            ],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="dp_robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="paint_viz",
            executable="scene_publisher",
            name="dp_scene_publisher",
            parameters=[{
                "part_file": str(scene.get("part_file", "")),
                "trajectory_dir": str(scene.get("trajectory_dir", "")),
                "frame_id": "part",
                "max_part_points": int(scene.get("max_part_points", 120000)),
                "max_rows": int(scene.get("max_rows", 0)),
            }],
            output="screen",
        ),
        Node(
            package="paint_viz",
            executable="trajectory_player",
            name="dp_trajectory_player",
            parameters=[{
                "joint_csv_dir": joint_csv_dir,
                "joint_csv_pattern": joint_csv_pattern,
                "urdf": urdf_file,
                "tcp_frame": "auto",  # tcp(노즐 팁) > spray_gun > link_6
                "frame_id": "world",
                "speed_scale": speed_scale,
                "loop": True,
            }],
            output="screen",
        ),
    ]

    if use_rviz:
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="dp_rviz2",
                arguments=["-d", rviz_config or os.path.join(share, "rviz", "dp_ik_viz.rviz")],
            )
        )
    return actions


def generate_launch_description():
    share = get_package_share_directory("paint_viz")
    return LaunchDescription([
        DeclareLaunchArgument(
            "scene",
            default_value=os.path.join(share, "config", "scene.yaml"),
            description="부품 파일 / trajectory 디렉터리 / 부품 배치 설정 (solve_dp_ik 와 같은 파일)",
        ),
        DeclareLaunchArgument(
            "gantry_env", default_value="legacy",
            description="갠트리 환경 (config/gantry/<name>.yaml): legacy | spec_c | x7y4z3. urdf 를 비우면 여기서 URDF 를 고른다",
        ),
        DeclareLaunchArgument(
            "urdf",
            default_value="",
            description=(
                "RViz에 띄울 URDF. 비우면(기본) gantry_env 의 urdf.mesh. 명시하면 그대로 쓴다. "
                "RViz에 띄울 URDF. 기본은 갠트리 2축 + A0912 6축 = 8축 셀(원본 메시). "
                "solve_dp_ik 에 넘긴 것과 같은 URDF 여야 관절 이름이 맞는다"
            ),
        ),
        DeclareLaunchArgument(
            "robot_root_frame",
            default_value="world_base",
            description="URDF 루트 링크 이름 (8축 셀은 world_base)",
        ),
        DeclareLaunchArgument(
            "joint_csv_dir",
            default_value=os.path.join(os.environ.get("PAINT_CELL_WS", str(repo_root() / "motion_planning/ros2_ws")), "dp_ik"),
            description="solve_dp_ik 가 만든 joint CSV 디렉터리",
        ),
        DeclareLaunchArgument(
            "joint_csv_pattern",
            default_value="*_dp_joint.csv",
            description="재생할 파일 패턴. dp_ik_summary.csv 를 걸러내려고 _dp_joint 로 좁혀 둔다",
        ),
        DeclareLaunchArgument(
            "rviz_config", default_value=os.path.join(share, "rviz", "dp_ik_viz.rviz")
        ),
        DeclareLaunchArgument("speed_scale", default_value="1.0", description="재생 배속"),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=_launch_setup),
    ])
