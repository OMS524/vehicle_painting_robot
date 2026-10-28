"""도장 부품 점군과 planner가 만든 TCP 경로를 RViz2로 퍼블리시하는 노드.

퍼블리시 토픽
  ~/part_cloud   sensor_msgs/PointCloud2   부품 표면 점군 (PLY 또는 CSV)
  ~/paint_paths  visualization_msgs/MarkerArray
                 - row별 경로 선 (분사 ON 구간은 색, OFF 구간은 회색)
                 - row 시작점 라벨
정적 데이터라 TRANSIENT_LOCAL QoS로 한 번만 보내고 유지한다.
"""

import colorsys
import os

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray

from motion_planner.ply_io import read_surface_points
from motion_planner.data_io import read_trajectory_dir

PAINT_OFF_COLOR = ColorRGBA(r=0.55, g=0.55, b=0.55, a=0.35)


def latched_qos(depth=1):
    return QoSProfile(
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def row_color(row_index, row_count):
    """row마다 구분되는 색을 HSV 원을 돌며 만든다."""
    hue = (row_index / max(row_count, 1)) % 1.0
    r, g, b = colorsys.hsv_to_rgb(hue, 0.85, 0.95)
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=1.0)


def make_point_cloud(frame_id, stamp, points, colors=None):
    """Nx3 점군(+ 선택적 RGB)을 PointCloud2로 만든다."""
    points = np.asarray(points, dtype=np.float32)
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    if colors is None:
        data = points
        point_step = 12
    else:
        # rgb를 float32 한 칸에 packed uint32로 넣는 RViz 관례를 따른다.
        colors = np.asarray(colors, dtype=np.uint32)
        packed = (colors[:, 0] << 16) | (colors[:, 1] << 8) | colors[:, 2]
        data = np.column_stack([points, packed.astype(np.uint32).view(np.float32)])
        fields.append(
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1)
        )
        point_step = 16

    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = 1
    msg.width = len(points)
    msg.fields = fields
    msg.is_bigendian = False
    msg.point_step = point_step
    msg.row_step = point_step * len(points)
    msg.is_dense = True
    msg.data = np.ascontiguousarray(data, dtype=np.float32).tobytes()
    return msg


class ScenePublisher(Node):
    def __init__(self):
        super().__init__("scene_publisher")

        self.declare_parameter("part_file", "")
        self.declare_parameter("trajectory_dir", "")
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("max_part_points", 120000)
        self.declare_parameter("max_rows", 0)
        self.declare_parameter("path_line_width", 0.004)
        self.declare_parameter("publish_period_sec", 2.0)

        self.frame_id = self.get_parameter("frame_id").value
        self.part_file = str(self.get_parameter("part_file").value)
        self.trajectory_dir = str(self.get_parameter("trajectory_dir").value)

        self.cloud_publisher = self.create_publisher(PointCloud2, "~/part_cloud", latched_qos())
        self.path_publisher = self.create_publisher(MarkerArray, "~/paint_paths", latched_qos())

        self.cloud_msg = self._build_part_cloud()
        self.path_msg = self._build_path_markers()

        self._publish_once()
        # RViz를 나중에 켜도 보이도록 TRANSIENT_LOCAL에 더해 주기적으로도 보낸다.
        period = float(self.get_parameter("publish_period_sec").value)
        if period > 0.0:
            self.create_timer(period, self._publish_once)

    def _build_part_cloud(self):
        if not self.part_file:
            self.get_logger().warn("part_file 파라미터가 비어 있어 부품 점군을 건너뛴다")
            return None
        if not os.path.isfile(self.part_file):
            self.get_logger().error(f"부품 파일을 찾을 수 없다: {self.part_file}")
            return None

        max_points = int(self.get_parameter("max_part_points").value)
        points, colors = read_surface_points(self.part_file, max_points=max_points)
        self.get_logger().info(
            f"부품 점군 {len(points)}점 로드: {os.path.basename(self.part_file)}"
        )
        return make_point_cloud(self.frame_id, self.get_clock().now().to_msg(), points, colors)

    def _build_path_markers(self):
        if not self.trajectory_dir:
            self.get_logger().warn("trajectory_dir 파라미터가 비어 있어 경로 표시를 건너뛴다")
            return None
        if not os.path.isdir(self.trajectory_dir):
            self.get_logger().error(f"trajectory 디렉터리를 찾을 수 없다: {self.trajectory_dir}")
            return None

        rows = read_trajectory_dir(self.trajectory_dir, max_rows=int(self.get_parameter("max_rows").value))
        if not rows:
            self.get_logger().error(f"trajectory CSV가 없다: {self.trajectory_dir}")
            return None

        stamp = self.get_clock().now().to_msg()
        width = float(self.get_parameter("path_line_width").value)
        markers = MarkerArray()

        for order, row in enumerate(rows):
            color = row_color(order, len(rows))
            line = Marker()
            line.header.frame_id = self.frame_id
            line.header.stamp = stamp
            line.ns = "paint_path"
            line.id = row.row_index
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale = Vector3(x=width, y=0.0, z=0.0)
            for position, painting in zip(row.positions, row.paint):
                line.points.append(
                    Point(x=float(position[0]), y=float(position[1]), z=float(position[2]))
                )
                line.colors.append(color if painting else PAINT_OFF_COLOR)
            markers.markers.append(line)

            label = Marker()
            label.header.frame_id = self.frame_id
            label.header.stamp = stamp
            label.ns = "paint_path_label"
            label.id = row.row_index
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position = Point(
                x=float(row.positions[0][0]),
                y=float(row.positions[0][1]),
                z=float(row.positions[0][2]),
            )
            label.pose.orientation.w = 1.0
            label.scale = Vector3(x=0.0, y=0.0, z=0.02)
            label.color = color
            label.text = f"row {row.row_index}"
            markers.markers.append(label)

        painted = sum(int(row.paint.sum()) for row in rows)
        total = sum(len(row) for row in rows)
        self.get_logger().info(
            f"경로 {len(rows)} row / {total} point 로드 (분사 ON {painted}점)"
        )
        return markers

    def _publish_once(self):
        if self.cloud_msg is not None:
            self.cloud_publisher.publish(self.cloud_msg)
        if self.path_msg is not None:
            self.path_publisher.publish(self.path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScenePublisher()
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
