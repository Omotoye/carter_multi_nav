import math

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class PlanningMapClearer(Node):
    def __init__(self):
        super().__init__("planning_map_clearer")

        self.declare_parameter("input_topic", "map")
        self.declare_parameter("output_topic", "planning_map")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("clear_radius", 0.60)

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self._map_frame = self.get_parameter("map_frame").get_parameter_value().string_value
        self._base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self._clear_radius = (
            self.get_parameter("clear_radius").get_parameter_value().double_value
        )

        transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        volatile_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._publisher = self.create_publisher(
            OccupancyGrid, output_topic, transient_qos
        )
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self._last_tf_warn_ns = 0

        # The live SLAM map durability differs between runs, so subscribe both
        # ways and normalize it into a single transient-local planning map.
        self.create_subscription(
            OccupancyGrid, input_topic, self._handle_map, transient_qos
        )
        self.create_subscription(
            OccupancyGrid, input_topic, self._handle_map, volatile_qos
        )

        self.get_logger().info(
            "Clearing a %.2fm radius robot footprint from '%s' into '%s'"
            % (self._clear_radius, input_topic, output_topic)
        )

    def _handle_map(self, msg: OccupancyGrid):
        cleaned = OccupancyGrid()
        cleaned.header = msg.header
        cleaned.info = msg.info
        cleaned.data = list(msg.data)

        transform = self._lookup_robot_pose()
        if transform is None:
            self._publisher.publish(cleaned)
            return

        resolution = float(cleaned.info.resolution)
        if resolution <= 0.0 or cleaned.info.width == 0 or cleaned.info.height == 0:
            self._publisher.publish(cleaned)
            return

        origin_x = float(cleaned.info.origin.position.x)
        origin_y = float(cleaned.info.origin.position.y)
        robot_x = float(transform.transform.translation.x)
        robot_y = float(transform.transform.translation.y)

        center_x = int(round((robot_x - origin_x) / resolution))
        center_y = int(round((robot_y - origin_y) / resolution))
        radius_cells = max(1, int(math.ceil(self._clear_radius / resolution)))
        width = int(cleaned.info.width)
        height = int(cleaned.info.height)

        for dy in range(-radius_cells, radius_cells + 1):
            cell_y = center_y + dy
            if cell_y < 0 or cell_y >= height:
                continue
            for dx in range(-radius_cells, radius_cells + 1):
                if (dx * dx) + (dy * dy) > radius_cells * radius_cells:
                    continue
                cell_x = center_x + dx
                if cell_x < 0 or cell_x >= width:
                    continue
                cleaned.data[(cell_y * width) + cell_x] = 0

        self._publisher.publish(cleaned)

    def _lookup_robot_pose(self):
        try:
            return self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                Time(),
                timeout=Duration(seconds=0.0),
            )
        except TransformException as exc:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_tf_warn_ns >= 5_000_000_000:
                self.get_logger().warn(
                    "Publishing raw map because %s -> %s is not ready yet: %s"
                    % (self._map_frame, self._base_frame, exc)
                )
                self._last_tf_warn_ns = now_ns
            return None


def main(args=None):
    rclpy.init(args=args)
    node = PlanningMapClearer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
