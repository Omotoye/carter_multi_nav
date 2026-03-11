import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class MapSelector(Node):
    def __init__(self):
        super().__init__("map_selector")

        self.declare_parameter("robot_names", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("preferred_source", "")
        self.declare_parameter("source_topic", "/carter1/map")
        self.declare_parameter("output_topic", "/shared_map")
        self.declare_parameter("source_name_topic", "/shared_map_source")
        self.declare_parameter("source_stale_timeout", 5.0)

        robot_names = list(
            self.get_parameter("robot_names").get_parameter_value().string_array_value
        )
        preferred_source = (
            self.get_parameter("preferred_source").get_parameter_value().string_value
        ).strip()
        source_topic = (
            self.get_parameter("source_topic").get_parameter_value().string_value
        ).strip()
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        source_name_topic = (
            self.get_parameter("source_name_topic").get_parameter_value().string_value
        )
        self._source_stale_timeout = (
            self.get_parameter("source_stale_timeout").get_parameter_value().double_value
        )

        publisher_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(
            OccupancyGrid, output_topic, publisher_qos
        )
        self._source_publisher = self.create_publisher(
            String, source_name_topic, publisher_qos
        )
        # The source map may be latched or live depending on the active SLAM
        # publisher, so subscribe both ways and normalize it to /shared_map.
        subscription_qos_volatile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        subscription_qos_transient = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._robot_names = robot_names
        self._preferred_source = preferred_source
        self._active_source = preferred_source if preferred_source else ""
        self._latest_maps = {}
        self._last_update_ns = {}
        self._subscriptions = []

        if robot_names:
            for robot_name in robot_names:
                topic = f"/{robot_name}/map"
                self._subscriptions.append(
                    self.create_subscription(
                        OccupancyGrid,
                        topic,
                        lambda msg, robot=robot_name: self._handle_map(robot, msg),
                        subscription_qos_transient,
                    )
                )
                self._subscriptions.append(
                    self.create_subscription(
                        OccupancyGrid,
                        topic,
                        lambda msg, robot=robot_name: self._handle_map(robot, msg),
                        subscription_qos_volatile,
                    )
                )
            self.create_timer(1.0, self._refresh_source)
            self.get_logger().info(
                "Republishing shared map for robots [%s] to '%s' (preferred source: '%s')"
                % (
                    ", ".join(robot_names),
                    output_topic,
                    preferred_source or robot_names[0],
                )
            )
        else:
            self._subscriptions.append(
                self.create_subscription(
                    OccupancyGrid,
                    source_topic,
                    lambda msg: self._publish_selected_map("", msg),
                    subscription_qos_transient,
                )
            )
            self._subscriptions.append(
                self.create_subscription(
                    OccupancyGrid,
                    source_topic,
                    lambda msg: self._publish_selected_map("", msg),
                    subscription_qos_volatile,
                )
            )
            self.get_logger().info(
                "Republishing map from '%s' to '%s'" % (source_topic, output_topic)
            )

    def _has_known_cells(self, msg: OccupancyGrid) -> bool:
        return any(cell >= 0 for cell in msg.data)

    def _source_is_fresh(self, robot_name: str) -> bool:
        last_update_ns = self._last_update_ns.get(robot_name)
        if last_update_ns is None:
            return False
        age_ns = self.get_clock().now().nanoseconds - last_update_ns
        return age_ns <= int(self._source_stale_timeout * 1e9)

    def _choose_source(self) -> str:
        if not self._robot_names:
            return ""

        if (
            self._active_source
            and self._source_is_fresh(self._active_source)
            and self._has_known_cells(self._latest_maps[self._active_source])
        ):
            return self._active_source

        if (
            self._preferred_source
            and self._preferred_source in self._latest_maps
            and self._source_is_fresh(self._preferred_source)
            and self._has_known_cells(self._latest_maps[self._preferred_source])
        ):
            return self._preferred_source

        for robot_name in self._robot_names:
            if (
                robot_name in self._latest_maps
                and self._source_is_fresh(robot_name)
                and self._has_known_cells(self._latest_maps[robot_name])
            ):
                return robot_name

        if self._active_source and self._source_is_fresh(self._active_source):
            return self._active_source

        if (
            self._preferred_source
            and self._preferred_source in self._latest_maps
            and self._source_is_fresh(self._preferred_source)
        ):
            return self._preferred_source

        for robot_name in self._robot_names:
            if robot_name in self._latest_maps and self._source_is_fresh(robot_name):
                return robot_name

        return self._active_source or (self._robot_names[0] if self._robot_names else "")

    def _publish_source_name(self, source_name: str):
        if not source_name:
            return
        self._source_publisher.publish(String(data=source_name))

    def _publish_selected_map(self, source_name: str, msg: OccupancyGrid):
        self._publisher.publish(msg)
        self._publish_source_name(source_name)

    def _switch_source_if_needed(self, new_source: str):
        if not new_source or new_source == self._active_source:
            return
        self._active_source = new_source
        self.get_logger().info("Switching shared map source to '%s'" % new_source)
        self._publish_source_name(new_source)

    def _refresh_source(self):
        if not self._robot_names:
            return

        chosen_source = self._choose_source()
        if not chosen_source:
            return

        self._switch_source_if_needed(chosen_source)
        selected_map = self._latest_maps.get(chosen_source)
        if selected_map is not None:
            self._publisher.publish(selected_map)

    def _handle_map(self, robot_name: str, msg: OccupancyGrid):
        self._latest_maps[robot_name] = msg
        self._last_update_ns[robot_name] = self.get_clock().now().nanoseconds

        chosen_source = self._choose_source()
        if not chosen_source:
            chosen_source = robot_name
        self._switch_source_if_needed(chosen_source)

        if robot_name == self._active_source:
            self._publish_selected_map(robot_name, msg)


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
