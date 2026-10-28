"""IK joint CSV를 재생해 RViz에서 로봇이 도장 경로를 따라 움직이게 하는 노드.

퍼블리시 토픽
  /joint_states            sensor_msgs/JointState     robot_state_publisher가 TF로 바꾼다
  ~/tcp_trail              visualization_msgs/Marker  실제로 지나간 TCP 궤적
  ~/status                 visualization_msgs/Marker  현재 row / 진행률 텍스트

joint CSV의 t 컬럼을 실시간 기준으로 재생하고, row가 끝나면 다음 row로 넘어간다.
마지막 row가 끝나면 처음부터 반복한다(loop 파라미터).
"""

import glob
import os

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Vector3
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

from motion_planner.data_io import read_joint_csv


class TrajectoryPlayer(Node):
    def __init__(self):
        super().__init__("trajectory_player")

        self.declare_parameter("joint_csv_dir", "")
        self.declare_parameter("joint_csv_pattern", "*_joint.csv")
        self.declare_parameter("urdf", "")
        self.declare_parameter("tcp_frame", "auto")  # auto = tcp > spray_gun > link_6 중 URDF 에 있는 것
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("rate_hz", 50.0)
        self.declare_parameter("speed_scale", 1.0)
        self.declare_parameter("loop", True)
        self.declare_parameter("trail_max_points", 20000)
        self.declare_parameter("row_gap_sec", 0.3)

        self.frame_id = self.get_parameter("frame_id").value
        self.speed_scale = max(float(self.get_parameter("speed_scale").value), 1.0e-3)
        self.loop = bool(self.get_parameter("loop").value)
        self.row_gap_sec = float(self.get_parameter("row_gap_sec").value)
        self.trail_max_points = int(self.get_parameter("trail_max_points").value)

        self.segments = self._load_segments()
        self.joint_names = self._resolve_joint_names()
        self._model, self._data, self._tcp_frame_id = self._load_kinematics()

        self.joint_publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.trail_publisher = self.create_publisher(Marker, "~/tcp_trail", 1)
        self.status_publisher = self.create_publisher(Marker, "~/status", 1)

        self.segment_index = 0
        self.elapsed = 0.0
        self.finished = False
        self.trail_points = []

        rate = max(float(self.get_parameter("rate_hz").value), 1.0)
        self.dt = 1.0 / rate
        self.create_timer(self.dt, self._tick)

        if self.segments:
            total = sum(segment[0][-1] for segment in self.segments)
            self.get_logger().info(
                f"joint 궤적 {len(self.segments)}개 재생 시작 (총 {total:.1f}s, "
                f"speed_scale={self.speed_scale})"
            )

    def _load_segments(self):
        directory = str(self.get_parameter("joint_csv_dir").value)
        pattern = str(self.get_parameter("joint_csv_pattern").value)
        if not directory or not os.path.isdir(directory):
            self.get_logger().error(
                f"joint_csv_dir 이 없다: {directory or '(비어 있음)'} — "
                "먼저 solve_viz_ik 로 joint CSV를 만들어야 로봇이 움직인다"
            )
            return []

        files = sorted(glob.glob(os.path.join(directory, pattern)))
        if not files:
            self.get_logger().error(f"{directory} 에 {pattern} 파일이 없다")
            return []

        segments = []
        for path in files:
            try:
                times, joint_names, positions = read_joint_csv(path)
            except (ValueError, OSError) as error:
                self.get_logger().warn(f"{os.path.basename(path)} 건너뜀: {error}")
                continue
            times = times - times[0]
            segments.append((times, positions, os.path.basename(path), joint_names))
        return segments

    def _resolve_joint_names(self):
        """관절 이름은 joint CSV 헤더에서 온다(갠트리 직동축 포함)."""
        if not self.segments:
            return []
        return list(self.segments[0][3])

    def _load_kinematics(self):
        """TCP 궤적 표시는 선택 기능이라 pinocchio/URDF가 없으면 조용히 끈다."""
        urdf = str(self.get_parameter("urdf").value)
        if not urdf or not os.path.isfile(urdf):
            return None, None, None
        try:
            import pinocchio as pin
        except ImportError:
            self.get_logger().warn("pinocchio가 없어 TCP 궤적 표시를 끈다")
            return None, None, None

        from motion_planner.tcp_frame import resolve_tcp_frame

        model = pin.buildModelFromUrdf(urdf)
        requested = str(self.get_parameter("tcp_frame").value)
        try:
            frame_name, frame_id = resolve_tcp_frame(model, requested)
        except ValueError as error:
            self.get_logger().warn(f"{error} — TCP 궤적 표시를 끈다")
            return None, None, None
        self.get_logger().info(f"TCP 궤적 프레임: {frame_name} (요청 {requested})")
        return model, model.createData(), frame_id

    def _sample(self, times, q_rad, elapsed):
        """구간 안 시각 elapsed에서 관절값을 선형보간한다."""
        if elapsed <= times[0]:
            return q_rad[0]
        if elapsed >= times[-1]:
            return q_rad[-1]
        index = int(np.searchsorted(times, elapsed))
        t0, t1 = times[index - 1], times[index]
        ratio = 0.0 if t1 <= t0 else (elapsed - t0) / (t1 - t0)
        return q_rad[index - 1] + ratio * (q_rad[index] - q_rad[index - 1])

    def _tick(self):
        if not self.segments or self.finished:
            return

        times, positions, name, _ = self.segments[self.segment_index]
        q = self._sample(times, positions, self.elapsed)

        stamp = self.get_clock().now().to_msg()
        message = JointState()
        message.header.stamp = stamp
        message.name = self.joint_names
        message.position = [float(value) for value in q]
        self.joint_publisher.publish(message)

        self._publish_tcp_trail(q, stamp)
        self._publish_status(stamp, name, times[-1])

        self.elapsed += self.dt * self.speed_scale
        if self.elapsed > times[-1] + self.row_gap_sec:
            self.elapsed = 0.0
            self.segment_index += 1
            if self.segment_index >= len(self.segments):
                if self.loop:
                    self.segment_index = 0
                    self.trail_points = []
                    self.get_logger().info("전체 경로 1회 완료 — 처음부터 반복")
                else:
                    self.segment_index = len(self.segments) - 1
                    self.finished = True
                    self.get_logger().info("전체 경로 재생 완료")

    def _publish_tcp_trail(self, q, stamp):
        if self._model is None:
            return
        import pinocchio as pin

        pin.forwardKinematics(self._model, self._data, np.asarray(q, dtype=float))
        pin.updateFramePlacements(self._model, self._data)
        position = self._data.oMf[self._tcp_frame_id].translation

        self.trail_points.append(Point(x=float(position[0]), y=float(position[1]), z=float(position[2])))
        if len(self.trail_points) > self.trail_max_points:
            self.trail_points = self.trail_points[-self.trail_max_points :]

        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = "tcp_trail"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale = Vector3(x=0.006, y=0.0, z=0.0)
        marker.color = ColorRGBA(r=1.0, g=0.25, b=0.0, a=1.0)
        marker.points = list(self.trail_points)
        self.trail_publisher.publish(marker)

    def _publish_status(self, stamp, name, duration):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = stamp
        marker.ns = "status"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position = Point(x=0.0, y=0.0, z=1.6)
        marker.pose.orientation.w = 1.0
        marker.scale = Vector3(x=0.0, y=0.0, z=0.08)
        marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
        marker.text = (
            f"{self.segment_index + 1}/{len(self.segments)}  {name}  "
            f"{min(self.elapsed, duration):.1f}/{duration:.1f}s"
        )
        self.status_publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlayer()
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
