"""부품 점군 + planner 경로 + 로봇 동작을 RViz2에 한 번에 띄우는 launch.

구성
  static_transform_publisher  world -> part   (scene.yaml의 part_xyz/part_rpy)
  robot_state_publisher       URDF + /joint_states -> TF
  scene_publisher             부품 점군, 도장 경로 마커 (part 프레임)
  trajectory_player           joint CSV 재생 -> /joint_states
  rviz2                       미리 만든 설정으로 실행

부품 좌표계를 TF로 분리했기 때문에, scene.yaml의 part_xyz/part_rpy만 바꾸면
표시와 IK가 같은 배치를 쓴다. IK 쪽도 같은 파일을 읽는다.
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
    joint_csv_dir = LaunchConfiguration("joint_csv_dir").perform(context)
    joint_csv_pattern = LaunchConfiguration("joint_csv_pattern").perform(context)
    rviz_config = LaunchConfiguration("rviz_config").perform(context)
    speed_scale = float(LaunchConfiguration("speed_scale").perform(context))
    tcp_frame = LaunchConfiguration("tcp_frame").perform(context)
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
            name="world_to_part",
            arguments=[
                "--x", str(part_xyz[0]), "--y", str(part_xyz[1]), "--z", str(part_xyz[2]),
                "--roll", str(part_rpy[0]), "--pitch", str(part_rpy[1]), "--yaw", str(part_rpy[2]),
                "--frame-id", "world", "--child-frame-id", "part",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_robot_root",
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
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="paint_viz",
            executable="scene_publisher",
            name="scene_publisher",
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
            name="trajectory_player",
            parameters=[{
                "joint_csv_dir": joint_csv_dir,
                "joint_csv_pattern": joint_csv_pattern,
                "urdf": urdf_file,
                "tcp_frame": tcp_frame,
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
                name="rviz2",
                arguments=["-d", rviz_config or os.path.join(share, "rviz", "paint_viz.rviz")],
            )
        )
    return actions


def generate_launch_description():
    share = get_package_share_directory("paint_viz")
    return LaunchDescription([
        DeclareLaunchArgument(
            "scene",
            default_value=os.path.join(share, "config", "scene.yaml"),
            description="부품 파일 / trajectory 디렉터리 / 부품 배치 설정",
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
                "dsr_description2 가 없는 환경이면 paint_cell_8axis.urdf(프리미티브)로 바꿀 것"
            ),
        ),
        DeclareLaunchArgument(
            "robot_root_frame",
            default_value="world_base",
            description="URDF 루트 링크 이름 (8축 셀은 world_base, 팔 단독은 base_link)",
        ),
        DeclareLaunchArgument(
            "joint_csv_dir",
            default_value=os.path.join(os.environ.get("PAINT_CELL_WS", str(repo_root() / "motion_planning/ros2_ws")), "viz_ik"),
            description="solve_viz_ik 가 만든 joint CSV 디렉터리",
        ),
        DeclareLaunchArgument(
            "joint_csv_pattern",
            default_value="*_joint.csv",
            description="재생할 joint CSV 글롭 (예: slot1_*.csv 로 도킹 경로 하나만)",
        ),
        DeclareLaunchArgument("rviz_config", default_value=os.path.join(share, "rviz", "paint_viz.rviz")),
        DeclareLaunchArgument("speed_scale", default_value="1.0", description="재생 배속"),
        DeclareLaunchArgument("tcp_frame", default_value="auto",
                              description="TCP 궤적(tcp_trail)을 그릴 프레임. auto = tcp(노즐 팁) > spray_gun > link_6 "
                                          "중 URDF 에 있는 것. 플랜지 궤적을 보려면 link_6"),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=_launch_setup),
    ])
