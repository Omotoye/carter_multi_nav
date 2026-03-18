import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


class NavReadyGate(Node):
    def __init__(self):
        super().__init__("nav_ready_gate")

        self.declare_parameter("map_topic", "map")
        self.declare_parameter("scan_topic", "scan_filtered")
        self.declare_parameter("odom_topic", "chassis/odom")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("consecutive_tf_successes", 5)
        self.declare_parameter("check_rate_hz", 5.0)
        self.declare_parameter("timeout", 30.0)

        map_topic = self.get_parameter("map_topic").get_parameter_value().string_value
        scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self._scan_topic = scan_topic
        self._odom_topic = odom_topic
        self._map_frame = (
            self.get_parameter("map_frame").get_parameter_value().string_value
        )
        self._base_frame = (
            self.get_parameter("base_frame").get_parameter_value().string_value
        )
        self._required_tf_successes = (
            self.get_parameter("consecutive_tf_successes")
            .get_parameter_value()
            .integer_value
        )
        check_rate_hz = (
            self.get_parameter("check_rate_hz").get_parameter_value().double_value
        )
        self._timeout = self.get_parameter("timeout").get_parameter_value().double_value

        map_qos_transient = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        map_qos_volatile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._have_map = False
        self._have_scan = False
        self._have_odom = False
        self._tf_successes = 0
        self.exit_code = 0

        self._start_time = time.monotonic()
        self._last_wait_log = None

        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)

        # The live /<robot>/map source is not stable across runs: sometimes it
        # is transient-local and sometimes it is volatile. Subscribe both ways
        # so either the latched map or live map updates can satisfy readiness.
        self.create_subscription(
            OccupancyGrid, map_topic, self._handle_map, map_qos_transient
        )
        self.create_subscription(
            OccupancyGrid, map_topic, self._handle_map, map_qos_volatile
        )
        self.create_subscription(LaserScan, scan_topic, self._handle_scan, sensor_qos)
        self.create_subscription(Odometry, odom_topic, self._handle_odom, sensor_qos)
        self.create_timer(1.0 / max(check_rate_hz, 1.0), self._check_readiness)

        self.get_logger().info(
            "Waiting for Nav readiness: map='%s', scan='%s', odom='%s', tf=%s->%s, timeout=%.1fs"
            % (
                map_topic,
                scan_topic,
                odom_topic,
                self._map_frame,
                self._base_frame,
                self._timeout,
            )
        )

    def _handle_map(self, _msg: OccupancyGrid):
        self._have_map = True

    def _handle_scan(self, _msg: LaserScan):
        self._have_scan = True

    def _handle_odom(self, _msg: Odometry):
        self._have_odom = True

    def _ready_to_check_tf(self):
        return self._have_map and self._have_scan and self._have_odom

    def _check_readiness(self):
        now = time.monotonic()
        elapsed = now - self._start_time

        missing = []
        if not self._have_map:
            missing.append("map")
        if not self._have_scan:
            missing.append(self._scan_topic)
        if not self._have_odom:
            missing.append(self._odom_topic)

        if self._ready_to_check_tf():
            if self._tf_buffer.can_transform(
                self._map_frame,
                self._base_frame,
                Time(),
                timeout=Duration(seconds=0.0),
            ):
                self._tf_successes += 1
            else:
                self._tf_successes = 0
                missing.append(f"tf:{self._map_frame}->{self._base_frame}")
        else:
            self._tf_successes = 0

        if self._tf_successes >= self._required_tf_successes:
            self.get_logger().info(
                "Nav readiness satisfied after %.1fs; starting Nav2." % elapsed
            )
            self.exit_code = 0
            rclpy.shutdown()
            return

        if elapsed >= self._timeout:
            self.get_logger().error(
                "Nav readiness timed out after %.1fs; missing=%s tf_successes=%d/%d"
                % (
                    elapsed,
                    ", ".join(missing) if missing else "none",
                    self._tf_successes,
                    self._required_tf_successes,
                )
            )
            self.exit_code = 1
            rclpy.shutdown()
            return

        if (
            self._last_wait_log is None
            or now - self._last_wait_log >= 2.0
        ):
            wait_for = ", ".join(missing) if missing else "stable tf"
            self.get_logger().info(
                "Nav readiness pending after %.1fs; waiting for %s (%d/%d stable tf checks)"
                % (
                    elapsed,
                    wait_for,
                    self._tf_successes,
                    self._required_tf_successes,
                )
            )
            self._last_wait_log = now


def main(args=None):
    rclpy.init(args=args)
    node = NavReadyGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        exit_code = getattr(node, "exit_code", 0)
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)
