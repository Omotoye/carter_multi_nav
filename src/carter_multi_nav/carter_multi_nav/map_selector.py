import time
from functools import partial

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from carter_multi_nav.common import DEFAULT_ROBOTS


class MapSelector(Node):
    """Republishes one robot's collaborative SLAM map as the shared map.

    Each decentralized `slam_toolbox` instance is expected to already publish
    the collaborative map produced from peer `LocalizedLaserScan` exchange.
    This node only chooses which robot's map to expose on `/shared_map`; it
    does not merge, rasterize, or clear any occupancy data itself.
    """

    def __init__(self):
        super().__init__("map_selector")

        self.declare_parameter("robot_names", list(DEFAULT_ROBOTS))
        self.declare_parameter("preferred_source", "")
        self.declare_parameter("output_topic", "/shared_map")
        self.declare_parameter("source_name_topic", "/shared_map_source")
        self.declare_parameter("source_stale_timeout", 5.0)
        self.declare_parameter("merge_publish_frequency", 1.0)

        self._robot_names = [
            name.strip()
            for name in list(
                self.get_parameter("robot_names")
                .get_parameter_value()
                .string_array_value
            )
            if name.strip()
        ]
        if not self._robot_names:
            self._robot_names = list(DEFAULT_ROBOTS)

        preferred_source = (
            self.get_parameter("preferred_source")
            .get_parameter_value()
            .string_value.strip()
        )
        self._preferred_source = preferred_source
        self._active_source = preferred_source if preferred_source else ""
        self._source_stale_timeout = (
            self.get_parameter("source_stale_timeout")
            .get_parameter_value()
            .double_value
        )
        publish_frequency = (
            self.get_parameter("merge_publish_frequency")
            .get_parameter_value()
            .double_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        source_name_topic = (
            self.get_parameter("source_name_topic")
            .get_parameter_value()
            .string_value
        )

        # Publishers
        pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(OccupancyGrid, output_topic, pub_qos)
        self._source_publisher = self.create_publisher(
            String, source_name_topic, pub_qos
        )

        # Per-robot state
        self._latest_maps = {}
        self._last_update_wall = {}  # wall-clock monotonic timestamps
        self._subscriptions = []

        # QoS profiles
        map_transient_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        map_volatile_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        for robot_name in self._robot_names:
            for qos in (map_transient_qos, map_volatile_qos):
                self._subscriptions.append(
                    self.create_subscription(
                        OccupancyGrid,
                        f"/{robot_name}/map",
                        partial(self._handle_map, robot_name),
                        qos,
                    )
                )

        # Rate-limit shared-map republishes using wall-clock time.
        # We avoid create_timer() because it uses sim time, which may not
        # advance if /clock messages haven't been received yet.
        self._publish_interval = (
            1.0 / publish_frequency if publish_frequency > 0.0 else 1.0
        )
        self._last_publish_wall_time = 0.0

        self.get_logger().info(
            "Selecting a shared SLAM map from robots [%s] onto '%s' (preferred source: '%s')"
            % (
                ", ".join(self._robot_names),
                output_topic,
                preferred_source or self._robot_names[0],
            )
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _handle_map(self, robot_name: str, msg: OccupancyGrid):
        self._latest_maps[robot_name] = msg
        self._last_update_wall[robot_name] = time.monotonic()
        self._maybe_publish()

    def _maybe_publish(self):
        """Publish the selected shared map if enough wall-clock time has elapsed."""
        now = time.monotonic()
        if now - self._last_publish_wall_time < self._publish_interval:
            return
        self._last_publish_wall_time = now
        self._publish_selected_map()

    # ------------------------------------------------------------------
    # Source selection (kept for TF relay compatibility)
    # ------------------------------------------------------------------

    def _has_known_cells(self, msg: OccupancyGrid) -> bool:
        return any(cell >= 0 for cell in msg.data)

    def _source_is_fresh(self, robot_name: str) -> bool:
        last_wall = self._last_update_wall.get(robot_name)
        if last_wall is None:
            return False
        age = time.monotonic() - last_wall
        return age <= self._source_stale_timeout

    def _choose_source(self) -> str:
        if not self._robot_names:
            return ""
        # Prefer current active source if still fresh
        if (
            self._active_source
            and self._active_source in self._latest_maps
            and self._source_is_fresh(self._active_source)
            and self._has_known_cells(self._latest_maps[self._active_source])
        ):
            return self._active_source
        # Then preferred source
        if (
            self._preferred_source
            and self._preferred_source in self._latest_maps
            and self._source_is_fresh(self._preferred_source)
            and self._has_known_cells(self._latest_maps[self._preferred_source])
        ):
            return self._preferred_source
        # Then any fresh source
        for robot_name in self._robot_names:
            if (
                robot_name in self._latest_maps
                and self._source_is_fresh(robot_name)
                and self._has_known_cells(self._latest_maps[robot_name])
            ):
                return robot_name
        return self._active_source or (
            self._robot_names[0] if self._robot_names else ""
        )

    def _publish_selected_map(self):
        source = self._choose_source()
        if not source:
            self.get_logger().warn(
                "No shared-map source chosen. maps=%s fresh=%s"
                % (
                    list(self._latest_maps.keys()),
                    [r for r in self._robot_names if self._source_is_fresh(r)],
                ),
                throttle_duration_sec=5.0,
            )
            return
        if source != self._active_source:
            self._active_source = source
            self.get_logger().info("Shared map source: '%s'" % source)
            self._source_publisher.publish(String(data=source))

        source_map = self._latest_maps.get(source)
        if source_map is None:
            return

        self._publisher.publish(source_map)
        self._source_publisher.publish(String(data=source))


def main(args=None):
    rclpy.init(args=args)
    node = MapSelector()
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
