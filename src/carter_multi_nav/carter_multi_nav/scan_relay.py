from functools import partial

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

from carter_multi_nav.common import DEFAULT_ROBOTS, LIDAR_FRAME, prefixed_frame


class ScanRelay(Node):
    def __init__(self):
        super().__init__("scan_relay")

        self.declare_parameter("robot_names", list(DEFAULT_ROBOTS))
        robot_names = list(
            self.get_parameter("robot_names").get_parameter_value().string_array_value
        )

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._relay_publishers = {}
        self._relay_subscriptions = []
        for robot_name in robot_names:
            raw_out = f"/rviz/{robot_name}/front_2d_lidar/scan"
            filtered_out = f"/rviz/{robot_name}/scan_filtered"
            self._relay_publishers[(robot_name, "raw")] = self.create_publisher(
                LaserScan, raw_out, qos
            )
            self._relay_publishers[(robot_name, "filtered")] = self.create_publisher(
                LaserScan, filtered_out, qos
            )
            self._relay_subscriptions.append(
                self.create_subscription(
                    LaserScan,
                    f"/{robot_name}/front_2d_lidar/scan",
                    partial(self._handle_scan, robot_name, "raw"),
                    qos,
                )
            )
            self._relay_subscriptions.append(
                self.create_subscription(
                    LaserScan,
                    f"/{robot_name}/scan_filtered",
                    partial(self._handle_scan, robot_name, "filtered"),
                    qos,
                )
            )

        self.get_logger().info(
            "Relaying scan topics for robots: %s" % ", ".join(robot_names)
        )

    def _handle_scan(self, robot_name: str, key: str, msg: LaserScan):
        relayed = LaserScan()
        relayed.header = msg.header
        relayed.header.frame_id = prefixed_frame(
            robot_name, msg.header.frame_id or LIDAR_FRAME
        )
        relayed.angle_min = msg.angle_min
        relayed.angle_max = msg.angle_max
        relayed.angle_increment = msg.angle_increment
        relayed.time_increment = msg.time_increment
        relayed.scan_time = msg.scan_time
        relayed.range_min = msg.range_min
        relayed.range_max = msg.range_max
        relayed.ranges = list(msg.ranges)
        relayed.intensities = list(msg.intensities)
        self._relay_publishers[(robot_name, key)].publish(relayed)


def main(args=None):
    rclpy.init(args=args)
    node = ScanRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
