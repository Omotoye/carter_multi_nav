from functools import partial

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from carter_multi_nav.common import DEFAULT_ROBOTS, MAP_FRAME, ROOT_FRAME, prefixed_frame


class TfAggregateRelay(Node):
    def __init__(self):
        super().__init__("tf_aggregate_relay")

        self.declare_parameter("robot_names", list(DEFAULT_ROBOTS))
        self.declare_parameter("dynamic_suffix", "tf")
        self.declare_parameter("static_suffix", "tf_static")
        self.declare_parameter("shared_map_source", "")
        self.declare_parameter("shared_map_source_topic", "/shared_map_source")

        robot_names = list(
            self.get_parameter("robot_names").get_parameter_value().string_array_value
        )
        dynamic_suffix = (
            self.get_parameter("dynamic_suffix").get_parameter_value().string_value
        )
        static_suffix = (
            self.get_parameter("static_suffix").get_parameter_value().string_value
        )
        self._shared_map_source = (
            self.get_parameter("shared_map_source").get_parameter_value().string_value
        ).strip()
        shared_map_source_topic = (
            self.get_parameter("shared_map_source_topic")
            .get_parameter_value()
            .string_value
        )

        dynamic_sub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        dynamic_pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        static_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._dynamic_pub = self.create_publisher(TFMessage, "/tf", dynamic_pub_qos)
        self._static_pub = self.create_publisher(TFMessage, "/tf_static", static_qos)
        self._relay_subscriptions = []
        self._source_subscription = self.create_subscription(
            String,
            shared_map_source_topic,
            self._handle_source_update,
            static_qos,
        )

        for robot_name in robot_names:
            self._relay_subscriptions.append(
                self.create_subscription(
                    TFMessage,
                    f"/{robot_name}/{dynamic_suffix}",
                    partial(self._handle_dynamic, robot_name),
                    dynamic_sub_qos,
                )
            )
            self._relay_subscriptions.append(
                self.create_subscription(
                    TFMessage,
                    f"/{robot_name}/{static_suffix}",
                    partial(self._handle_static, robot_name),
                    static_qos,
                )
            )

        self.get_logger().info(
            "Aggregating TF for robots: %s" % ", ".join(robot_names)
        )

    def _handle_source_update(self, msg: String):
        source_name = str(msg.data or "").strip()
        if not source_name or source_name == self._shared_map_source:
            return
        self._shared_map_source = source_name
        self.get_logger().info(
            "Using '%s' as the shared map TF source" % self._shared_map_source
        )

    def _should_skip(self, robot_name: str, transform: TransformStamped) -> bool:
        parent = str(transform.header.frame_id or "").lstrip("/")
        child = str(transform.child_frame_id or "").lstrip("/")
        if parent != MAP_FRAME and child != MAP_FRAME:
            return False
        return robot_name != self._shared_map_source

    def _rewrite_frame(self, robot_name: str, frame_id: str) -> str:
        frame = str(frame_id or "").lstrip("/")
        if frame in (ROOT_FRAME, MAP_FRAME):
            return frame
        return prefixed_frame(robot_name, frame)

    def _rewrite_transform(self, robot_name: str, transform: TransformStamped):
        if self._should_skip(robot_name, transform):
            return None

        parent = str(transform.header.frame_id or "").lstrip("/")
        child = str(transform.child_frame_id or "").lstrip("/")
        if not parent or not child:
            return None

        rewritten = TransformStamped()
        rewritten.header.stamp = transform.header.stamp
        rewritten.header.frame_id = self._rewrite_frame(robot_name, parent)
        rewritten.child_frame_id = self._rewrite_frame(robot_name, child)
        rewritten.transform.translation = transform.transform.translation
        rewritten.transform.rotation = transform.transform.rotation
        return rewritten

    def _forward(self, robot_name: str, msg: TFMessage, publisher):
        transforms = []
        for transform in msg.transforms:
            rewritten = self._rewrite_transform(robot_name, transform)
            if rewritten is not None:
                transforms.append(rewritten)

        if transforms:
            publisher.publish(TFMessage(transforms=transforms))

    def _handle_dynamic(self, robot_name: str, msg: TFMessage):
        self._forward(robot_name, msg, self._dynamic_pub)

    def _handle_static(self, robot_name: str, msg: TFMessage):
        self._forward(robot_name, msg, self._static_pub)


def main(args=None):
    rclpy.init(args=args)
    node = TfAggregateRelay()
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
