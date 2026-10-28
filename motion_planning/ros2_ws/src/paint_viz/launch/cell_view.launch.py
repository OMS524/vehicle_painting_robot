"""갠트리 + 로봇(+툴)만 RViz2 에 띄우는 launch. 부품 점군/경로/궤적 재생은 없다.

용도: TCP(스프레이건 노즐 팁 `tcp` 프레임)가 로봇 어디에 붙는지, 갠트리 중앙에서
팔이 어떻게 매달리는지 형상만 확인한다.

기본 자세 = trajectory_nlp.py 의 center_weight 항이 당기는 **관절범위 중앙값** q_mid
  q_mid = (lower + upper) / 2. 갠트리 모델에서는 NLP 가 joint_3 / joint_5 bound 를
  "joint_3=25:135,joint_5=25:135" 로 덮어쓰므로(ARM_JOINT_BOUNDS_DEG, auto) 그 둘의 중앙은 80 deg,
  나머지 팔 관절은 URDF 대칭 한계라 0 deg, gantry_x / gantry_y 는 limit 중간값이다.
  최종본 paint_cell_8axis_gantry_mesh_c.urdf 는 원점이 코너(툴체인저 X 끝단)라 중간값이
  각각 1.4132 / 1.1432 이고, 이때 브리지·캐리지가 월드 XY 원점(갠트리 중앙) 위에 온다.
  joint_bounds_deg 인자를 바꾸면 그 bound 의 중앙으로, arm_deg 를 주면 그 값으로 띄운다.

예
  ros2 launch paint_viz cell_view.launch.py                       # NLP 중앙값 자세
  ros2 launch paint_viz cell_view.launch.py gui:=true             # 슬라이더로 관절 조작 (초기값 = 중앙값)
  ros2 launch paint_viz cell_view.launch.py joint_bounds_deg:="joint_3=40:120,joint_5=25:135"
  ros2 launch paint_viz cell_view.launch.py arm_deg:="0,0,90,0,90,0"   # 명시 자세
  ros2 launch paint_viz cell_view.launch.py gantry_x:=0.5 gantry_y:=center
  ros2 launch paint_viz cell_view.launch.py gantry_env:=x7y4z3           # 신규 7x4x3 환경
  ros2 launch paint_viz cell_view.launch.py urdf:=/path/other.urdf

노드
  static_transform_publisher   world -> world_base (항등)
  robot_state_publisher        URDF -> TF
  joint_state_publisher(_gui)  zeros 파라미터로 위 자세를 /joint_states 로 계속 퍼블리시
  cell_pose_info               /joint_states 를 받아 tcp / link_6 월드 좌표를 로그 + RViz 텍스트 마커
  rviz2                        rviz/cell_view.rviz (RobotModel + TF: world/base_link/link_6/tcp)
"""

import math
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from motion_planner.configuration import load_gantry_env

ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
GANTRY_JOINTS = ["gantry_x", "gantry_y"]
# trajectory_nlp.py ARM_JOINT_BOUNDS_DEG(갠트리 모델 auto bound) 와 동일하게 유지할 것
NLP_ARM_JOINT_BOUNDS_DEG = "joint_3=25:135,joint_5=25:135"


def _joint_limits(urdf_text):
    """URDF 텍스트에서 {joint: (lower, upper)} (fixed 제외)."""
    limits = {}
    for joint in ET.fromstring(urdf_text).findall("joint"):
        if joint.attrib.get("type", "fixed") == "fixed":
            continue
        limit = joint.find("limit")
        if limit is None:
            continue
        limits[joint.attrib["name"]] = (
            float(limit.attrib.get("lower", "-inf")),
            float(limit.attrib.get("upper", "inf")),
        )
    return limits


def _apply_bound_overrides(limits, spec):
    """trajectory_nlp.apply_joint_bound_overrides 와 같은 규칙: "name=lo:hi,..." (deg), 좁히기만."""
    limits = dict(limits)
    for item in spec.split(","):
        item = item.strip()
        if not item:
            continue
        name, rng = item.split("=")
        lo_text, hi_text = rng.split(":")
        name = name.strip()
        if name not in limits:
            continue
        lo, hi = math.radians(float(lo_text)), math.radians(float(hi_text))
        if lo >= hi:
            raise ValueError(f"joint_bounds_deg: {name} lower {lo_text} >= upper {hi_text}")
        limits[name] = (max(lo, limits[name][0]), min(hi, limits[name][1]))
    return limits


def _gantry_value(text, name, limits):
    """'center' 면 limit 중간값, 아니면 숫자 [m] (joint 값 그대로)."""
    if text.strip().lower() == "center":
        if name not in limits:
            return 0.0
        lower, upper = limits[name]
        return 0.5 * (lower + upper)
    return float(text)


def _launch_setup(context, *args, **kwargs):
    share = get_package_share_directory("paint_viz")
    gantry_env_name = LaunchConfiguration("gantry_env").perform(context)
    urdf_file = LaunchConfiguration("urdf").perform(context).strip()
    env_info = ""
    if not urdf_file:
        env = load_gantry_env(gantry_env_name)
        urdf_file = str(env.urdf_path("mesh"))
        travel = env.world_travel_range()
        env_info = (f"gantry_env {env.name}: 레일 {env.rail['x_length']} x {env.rail['y_length']} m, "
                    f"브리지 {env.bridge_z:.4f} m, 장착면 {env.mount_z:.4f} m, "
                    f"캐리지 X {travel['x'][0]:+.4f}~{travel['x'][1]:+.4f} Y {travel['y'][0]:+.4f}~{travel['y'][1]:+.4f}")
    root_frame = LaunchConfiguration("robot_root_frame").perform(context)
    rviz_config = LaunchConfiguration("rviz_config").perform(context)
    use_gui = LaunchConfiguration("gui").perform(context).lower() in ("1", "true", "yes")
    use_rviz = LaunchConfiguration("rviz").perform(context).lower() in ("1", "true", "yes")
    tcp_frame = LaunchConfiguration("tcp_frame").perform(context)

    with open(urdf_file, "r", encoding="utf-8") as stream:
        robot_description = stream.read()
    limits = _joint_limits(robot_description)

    bounds_spec = LaunchConfiguration("joint_bounds_deg").perform(context)
    nlp_limits = _apply_bound_overrides(limits, bounds_spec)

    zeros = {}
    for name, key in zip(GANTRY_JOINTS, ("gantry_x", "gantry_y")):
        if name in limits:
            zeros[name] = _gantry_value(LaunchConfiguration(key).perform(context), name, nlp_limits)
    arm_text = LaunchConfiguration("arm_deg").perform(context).strip()
    if arm_text.lower() == "center":
        # NLP q_mid = (lower + upper) / 2, bound 덮어쓰기 적용 후
        for name in ARM_JOINTS:
            if name in nlp_limits:
                lower, upper = nlp_limits[name]
                zeros[name] = 0.5 * (lower + upper)
    else:
        arm_deg = [float(v) for v in arm_text.split(",")]
        if len(arm_deg) != len(ARM_JOINTS):
            raise ValueError(f"arm_deg 는 6개 값이어야 한다: {arm_deg}")
        for name, deg in zip(ARM_JOINTS, arm_deg):
            if name in limits:
                zeros[name] = math.radians(deg)

    pose_text = ", ".join(
        f"{k}={v:.4f}" + ("" if k.startswith("gantry") else f"({math.degrees(v):.1f}deg)")
        for k, v in zeros.items()
    )

    jsp_params = {"zeros": zeros, "rate": 20}
    actions = [
        LogInfo(msg=f"[cell_view] URDF {urdf_file}"),
        LogInfo(msg=f"[cell_view] {env_info or 'urdf 명시 (gantry_env 무시)'}"),
        LogInfo(msg=f"[cell_view] joint bound 덮어쓰기(NLP auto 와 동일): {bounds_spec!r}"),
        LogInfo(msg=f"[cell_view] 초기 자세(= NLP center_weight 가 당기는 q_mid): {pose_text}"),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_robot_root",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "world", "--child-frame-id", root_frame,
            ],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher_gui" if use_gui else "joint_state_publisher",
            executable="joint_state_publisher_gui" if use_gui else "joint_state_publisher",
            name="joint_state_publisher",
            parameters=[jsp_params],
            output="screen",
        ),
        Node(
            package="paint_viz",
            executable="cell_pose_info",
            name="cell_pose_info",
            parameters=[{"urdf": urdf_file, "tcp_frame": tcp_frame, "frame_id": "world"}],
            output="screen",
        ),
    ]
    if use_rviz:
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config or os.path.join(share, "rviz", "cell_view.rviz")],
            )
        )
    return actions


def generate_launch_description():
    share = get_package_share_directory("paint_viz")
    return LaunchDescription([
        DeclareLaunchArgument(
            "gantry_env", default_value="spec_c",
            description="갠트리 환경 (config/gantry/<name>.yaml): spec_c(기본, 확정 스펙 메시본) | legacy | x7y4z3. "
                        "urdf 를 비우면 여기서 URDF 를 고른다",
        ),
        DeclareLaunchArgument(
            "urdf",
            default_value="",
            description="띄울 URDF. 비우면(기본) gantry_env 의 urdf.mesh "
                        "(spec_c = gantry.stl 실측 메시 + 사용자 확정 스펙 최종본, 툴 장착, tcp 프레임 포함)",
        ),
        DeclareLaunchArgument("robot_root_frame", default_value="world_base",
                              description="URDF 루트 링크 (8축 셀 world_base, 팔 단독 base_link)"),
        DeclareLaunchArgument("gantry_x", default_value="center",
                              description="gantry_x 조인트 값 [m]. center = URDF limit 중간(갠트리 중앙)"),
        DeclareLaunchArgument("gantry_y", default_value="center",
                              description="gantry_y 조인트 값 [m]. center = URDF limit 중간(갠트리 중앙)"),
        DeclareLaunchArgument("arm_deg", default_value="center",
                              description="joint_1..6 [deg]. center = NLP 관절범위 중앙값 q_mid "
                                          "(joint_bounds_deg 적용 후 (lower+upper)/2)"),
        DeclareLaunchArgument("joint_bounds_deg", default_value=NLP_ARM_JOINT_BOUNDS_DEG,
                              description="trajectory_nlp --joint-bounds-deg 와 같은 형식. 중앙값 계산에 쓴다. "
                                          "빈 문자열이면 URDF 한계만"),
        DeclareLaunchArgument("tcp_frame", default_value="auto",
                              description="좌표를 찍을 TCP 프레임. auto = tcp > spray_gun > link_6"),
        DeclareLaunchArgument("gui", default_value="false",
                              description="true 면 joint_state_publisher_gui 슬라이더 (초기값은 위 자세)"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=os.path.join(share, "rviz", "cell_view.rviz")),
        OpaqueFunction(function=_launch_setup),
    ])
