import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class MapSelector(Node):
    def __init__(self):
        super().__init__("map_selector")

        self.declare_parameter("source_topic", "/carter1/map")
        self.declare_parameter("output_topic", "/shared_map")

        source_topic = self.get_parameter("source_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value

        publisher_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        subscription_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._publisher = self.create_publisher(
            OccupancyGrid, output_topic, publisher_qos
        )
        self._subscription = self.create_subscription(
            OccupancyGrid,
            source_topic,
            self._handle_map,
            subscription_qos,
        )

        self.get_logger().info(
            "Republishing map from '%s' to '%s'" % (source_topic, output_topic)
        )

    def _handle_map(self, msg: OccupancyGrid):
        self._publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapSelector()
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
