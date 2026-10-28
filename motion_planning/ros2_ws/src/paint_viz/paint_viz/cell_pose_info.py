"""/joint_states 를 받아 TCP(노즐 팁)·플랜지(link_6)의 월드 좌표를 찍는 노드.

cell_view.launch.py 가 띄운다. 로그로 한 번(관절이 바뀌면 다시) 출력하고, RViz 용
텍스트 마커(~/pose_info)를 TCP 옆에 띄운다. pinocchio 가 없으면 조용히 종료한다.
"""

import math
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker

from motion_planner.tcp_frame import resolve_tcp_frame


def _rpy_deg(rotation):
    """회전행렬 -> roll/pitch/yaw [deg] (URDF rpy 규약)."""
    pitch = -math.asin(max(-1.0, min(1.0, rotation[2, 0])))
    roll = math.atan2(rotation[2, 1], rotation[2, 2])
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    return tuple(math.degrees(v) for v in (roll, pitch, yaw))


class CellPoseInfo(Node):
    def __init__(self):
        super().__init__("cell_pose_info")
        self.declare_parameter("urdf", "")
        self.declare_parameter("tcp_frame", "auto")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("text_height", 0.08)

        urdf = str(self.get_parameter("urdf").value)
        if not urdf or not os.path.isfile(urdf):
            raise FileNotFoundError(f"urdf 파라미터가 비었거나 파일이 없다: {urdf!r}")
        import pinocchio as pin  # 지연 import: 없으면 main 에서 잡는다

        self._pin = pin
        self.model = pin.buildModelFromUrdf(urdf)
        self.data = self.model.createData()
        self.tcp_name, self.tcp_id = resolve_tcp_frame(self.model, str(self.get_parameter("tcp_frame").value))
        self.flange_id = self.model.getFrameId("link_6") if self.model.existFrame("link_6") else None
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.text_height = float(self.get_parameter("text_height").value)
        # joint_name -> q index (1-DoF 조인트만)
        self.q_index = {}
        for joint_id in range(1, self.model.njoints):
            joint = self.model.joints[joint_id]
            if joint.nq == 1:
                self.q_index[self.model.names[joint_id]] = joint.idx_q
        self.q = self._neutral()
        self._last_key = None

        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.marker_pub = self.create_publisher(Marker, "~/pose_info", qos)
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self.get_logger().info(f"TCP 프레임: {self.tcp_name} (URDF {os.path.basename(urdf)})")

    def _neutral(self):
        q = self._pin.neutral(self.model)
        return np.asarray(q, dtype=float).copy()

    def _on_joint_state(self, msg):
        for name, position in zip(msg.name, msg.position):
            index = self.q_index.get(name)
            if index is not None:
                self.q[index] = float(position)
        key = tuple(np.round(self.q, 5))
        if key == self._last_key:
            return
        self._last_key = key
        self._report(msg.header.stamp)

    def _report(self, stamp):
        pin = self._pin
        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)
        tcp = self.data.oMf[self.tcp_id]
        lines = [f"{self.tcp_name}: xyz=({tcp.translation[0]:+.4f}, {tcp.translation[1]:+.4f}, {tcp.translation[2]:+.4f})"
                 f" rpy=({', '.join(f'{v:+.1f}' for v in _rpy_deg(tcp.rotation))}) deg  spray(+z)=("
                 f"{', '.join(f'{v:+.3f}' for v in tcp.rotation[:, 2])})"]
        if self.flange_id is not None and self.flange_id != self.tcp_id:
            flange = self.data.oMf[self.flange_id]
            offset = tcp.translation - flange.translation
            lines.append(f"link_6: xyz=({flange.translation[0]:+.4f}, {flange.translation[1]:+.4f}, "
                         f"{flange.translation[2]:+.4f})  flange->tcp {np.linalg.norm(offset):.4f} m")
        joints = ", ".join(
            f"{name}={self.q[i]:.4f}" if name.startswith("gantry") else f"{name}={math.degrees(self.q[i]):.1f}"
            for name, i in sorted(self.q_index.items(), key=lambda kv: kv[1]))
        lines.append(f"q: {joints}")
        for line in lines:
            self.get_logger().info(line)

        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = "pose_info"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = float(tcp.translation[0])
        marker.pose.position.y = float(tcp.translation[1])
        marker.pose.position.z = float(tcp.translation[2]) - 0.15
        marker.pose.orientation.w = 1.0
        marker.scale.z = self.text_height
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.9, 0.2, 1.0
        marker.text = "\n".join(lines[:2])
        self.marker_pub.publish(marker)

        sphere = Marker()
        sphere.header = marker.header
        sphere.ns = "tcp_point"
        sphere.id = 1
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position.x = float(tcp.translation[0])
        sphere.pose.position.y = float(tcp.translation[1])
        sphere.pose.position.z = float(tcp.translation[2])
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.03
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 1.0, 0.2, 0.2, 0.9
        self.marker_pub.publish(sphere)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CellPoseInfo()
    except ImportError:
        print("[cell_pose_info] pinocchio 가 없어 TCP 좌표 출력을 끈다")
        rclpy.shutdown()
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
