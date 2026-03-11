import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan


class ScanMotionGate(Node):
    def __init__(self):
        super().__init__("scan_motion_gate")

        self.declare_parameter("scan_in", "scan_filtered")
        self.declare_parameter("scan_out", "scan_motion_safe")
        self.declare_parameter("odom_topic", "chassis/odom")
        self.declare_parameter("max_rotation_per_scan_deg", 2.0)
        self.declare_parameter("max_angular_velocity", 0.35)
        self.declare_parameter("holdoff_after_rotation", 0.40)
        self.declare_parameter("stale_odom_timeout", 0.5)
        self.declare_parameter("status_interval", 5.0)

        scan_in = self.get_parameter("scan_in").get_parameter_value().string_value
        scan_out = self.get_parameter("scan_out").get_parameter_value().string_value
        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value

        self._max_rotation_per_scan_deg = (
            self.get_parameter("max_rotation_per_scan_deg")
            .get_parameter_value()
            .double_value
        )
        self._max_angular_velocity = (
            self.get_parameter("max_angular_velocity").get_parameter_value().double_value
        )
        self._holdoff_after_rotation = (
            self.get_parameter("holdoff_after_rotation")
            .get_parameter_value()
            .double_value
        )
        self._stale_odom_timeout = (
            self.get_parameter("stale_odom_timeout").get_parameter_value().double_value
        )
        status_interval = (
            self.get_parameter("status_interval").get_parameter_value().double_value
        )

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._latest_omega_z = None
        self._latest_odom_time = None
        self._last_scan_stamp = None
        self._blocked_until = None

        self._received = 0
        self._published = 0
        self._dropped_high_rotation = 0
        self._dropped_holdoff = 0
        self._dropped_stale_odom = 0

        self._publisher = self.create_publisher(LaserScan, scan_out, qos)
        self.create_subscription(Odometry, odom_topic, self._handle_odom, qos)
        self.create_subscription(LaserScan, scan_in, self._handle_scan, qos)
        self.create_timer(status_interval, self._report)

        self.get_logger().info(
            "Gating scans on '%s' -> '%s' using odom '%s', max_rotation_per_scan_deg=%.2f, max_angular_velocity=%.3f rad/s"
            % (
                scan_in,
                scan_out,
                odom_topic,
                self._max_rotation_per_scan_deg,
                self._max_angular_velocity,
            )
        )

    def _handle_odom(self, msg: Odometry):
        self._latest_omega_z = float(msg.twist.twist.angular.z)
        self._latest_odom_time = Time.from_msg(msg.header.stamp)

    def _effective_duration(self, msg: LaserScan, stamp: Time):
        if msg.scan_time > 0.0:
            return float(msg.scan_time)

        if msg.time_increment > 0.0 and len(msg.ranges) > 1:
            return float(msg.time_increment) * float(len(msg.ranges) - 1)

        if self._last_scan_stamp is not None:
            delta = (stamp - self._last_scan_stamp).nanoseconds / 1e9
            if delta > 0.0:
                return delta

        return None

    def _handle_scan(self, msg: LaserScan):
        self._received += 1
        stamp = Time.from_msg(msg.header.stamp)
        duration = self._effective_duration(msg, stamp)
        self._last_scan_stamp = stamp

        if self._blocked_until is not None and stamp < self._blocked_until:
            self._dropped_holdoff += 1
            return

        if (
            self._latest_omega_z is None
            or self._latest_odom_time is None
            or abs((stamp - self._latest_odom_time).nanoseconds) / 1e9
            > self._stale_odom_timeout
        ):
            self._dropped_stale_odom += 1
            return

        omega = abs(self._latest_omega_z)
        rotation_deg = None
        if duration is not None:
            rotation_deg = math.degrees(omega * duration)

        if omega > self._max_angular_velocity:
            self._dropped_high_rotation += 1
            self._blocked_until = stamp + Duration(seconds=self._holdoff_after_rotation)
            return

        if rotation_deg is not None and rotation_deg > self._max_rotation_per_scan_deg:
            self._dropped_high_rotation += 1
            self._blocked_until = stamp + Duration(seconds=self._holdoff_after_rotation)
            return

        self._publisher.publish(msg)
        self._published += 1

    def _report(self):
        if self._received == 0:
            self.get_logger().warn("No scans received yet.")
            return

        drop_ratio = 100.0 * (
            (self._received - self._published) / float(self._received)
        )
        self.get_logger().info(
            "Scan gate stats: received=%d published=%d dropped_high_rotation=%d dropped_holdoff=%d dropped_stale_odom=%d drop_ratio=%.1f%%"
            % (
                self._received,
                self._published,
                self._dropped_high_rotation,
                self._dropped_holdoff,
                self._dropped_stale_odom,
                drop_ratio,
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = ScanMotionGate()
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
