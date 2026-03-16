import math
from functools import partial

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from carter_multi_nav.common import (
    DEFAULT_ROBOTS,
    MAP_FRAME,
    ROOT_FRAME,
    prefixed_frame,
)


def _normalize_frame(frame_id: str) -> str:
    return str(frame_id or "").lstrip("/")


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def _normalize_quaternion(x: float, y: float, z: float, w: float):
    norm = math.sqrt((x * x) + (y * y) + (z * z) + (w * w))
    if norm <= 1e-12:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / norm, y / norm, z / norm, w / norm)


def _quaternion_dot(lhs, rhs) -> float:
    return (
        (lhs[0] * rhs[0])
        + (lhs[1] * rhs[1])
        + (lhs[2] * rhs[2])
        + (lhs[3] * rhs[3])
    )


def _quaternion_angle(lhs, rhs) -> float:
    dot = abs(_clamp(_quaternion_dot(lhs, rhs), -1.0, 1.0))
    return 2.0 * math.acos(dot)


def _slerp_quaternion(lhs, rhs, alpha: float):
    lhs = _normalize_quaternion(*lhs)
    rhs = _normalize_quaternion(*rhs)
    alpha = _clamp(alpha, 0.0, 1.0)
    dot = _quaternion_dot(lhs, rhs)

    if dot < 0.0:
        rhs = (-rhs[0], -rhs[1], -rhs[2], -rhs[3])
        dot = -dot

    if dot > 0.9995:
        blended = tuple(
            lhs[index] + (alpha * (rhs[index] - lhs[index])) for index in range(4)
        )
        return _normalize_quaternion(*blended)

    theta_0 = math.acos(_clamp(dot, -1.0, 1.0))
    sin_theta_0 = math.sin(theta_0)
    if abs(sin_theta_0) <= 1e-12:
        return lhs

    theta = theta_0 * alpha
    sin_theta = math.sin(theta)
    scale_lhs = math.sin(theta_0 - theta) / sin_theta_0
    scale_rhs = sin_theta / sin_theta_0
    return tuple(
        (scale_lhs * lhs[index]) + (scale_rhs * rhs[index]) for index in range(4)
    )


class MapTransformSmoother:
    def __init__(
        self,
        *,
        enabled: bool,
        alpha: float,
        max_translation_jump: float,
        max_rotation_jump: float,
    ):
        self._enabled = enabled
        self._alpha = _clamp(alpha, 0.0, 1.0)
        self._max_translation_jump = max_translation_jump
        self._max_rotation_jump = max_rotation_jump
        self._state = None

    def reset(self):
        self._state = None

    def filter(self, transform: TransformStamped):
        translation = (
            float(transform.transform.translation.x),
            float(transform.transform.translation.y),
            float(transform.transform.translation.z),
        )
        rotation = _normalize_quaternion(
            float(transform.transform.rotation.x),
            float(transform.transform.rotation.y),
            float(transform.transform.rotation.z),
            float(transform.transform.rotation.w),
        )

        if not self._enabled:
            self._state = (translation, rotation)
            return transform, False

        if self._state is None:
            self._state = (translation, rotation)
            filtered = TransformStamped()
            filtered.header = transform.header
            filtered.child_frame_id = transform.child_frame_id
            filtered.transform.translation.x = translation[0]
            filtered.transform.translation.y = translation[1]
            filtered.transform.translation.z = translation[2]
            filtered.transform.rotation.x = rotation[0]
            filtered.transform.rotation.y = rotation[1]
            filtered.transform.rotation.z = rotation[2]
            filtered.transform.rotation.w = rotation[3]
            return filtered, False

        previous_translation, previous_rotation = self._state
        translation_jump = math.dist(translation, previous_translation)
        rotation_jump = _quaternion_angle(rotation, previous_rotation)
        rejected = False

        if self._max_translation_jump > 0.0 and translation_jump > self._max_translation_jump:
            rejected = True
        if self._max_rotation_jump > 0.0 and rotation_jump > self._max_rotation_jump:
            rejected = True

        if rejected:
            translation = previous_translation
            rotation = previous_rotation
        else:
            translation = tuple(
                previous_translation[index]
                + (self._alpha * (translation[index] - previous_translation[index]))
                for index in range(3)
            )
            rotation = _slerp_quaternion(previous_rotation, rotation, self._alpha)
            self._state = (translation, rotation)

        filtered = TransformStamped()
        filtered.header = transform.header
        filtered.child_frame_id = transform.child_frame_id
        filtered.transform.translation.x = translation[0]
        filtered.transform.translation.y = translation[1]
        filtered.transform.translation.z = translation[2]
        filtered.transform.rotation.x = rotation[0]
        filtered.transform.rotation.y = rotation[1]
        filtered.transform.rotation.z = rotation[2]
        filtered.transform.rotation.w = rotation[3]
        return filtered, rejected


class TfAggregateRelay(Node):
    def __init__(self):
        super().__init__("tf_aggregate_relay")

        self.declare_parameter("robot_names", list(DEFAULT_ROBOTS))
        self.declare_parameter("dynamic_suffix", "tf")
        self.declare_parameter("static_suffix", "tf_static")
        self.declare_parameter("shared_map_source", "")
        self.declare_parameter("shared_map_source_topic", "/shared_map_source")
        self.declare_parameter("map_tf_smoothing_enabled", True)
        self.declare_parameter("map_tf_smoothing_alpha", 0.3)
        self.declare_parameter("map_tf_max_translation_jump", 0.05)
        self.declare_parameter("map_tf_max_rotation_jump", 0.02)

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
        self._map_tf_smoothing_enabled = (
            self.get_parameter("map_tf_smoothing_enabled")
            .get_parameter_value()
            .bool_value
        )
        self._map_tf_smoothing_alpha = (
            self.get_parameter("map_tf_smoothing_alpha")
            .get_parameter_value()
            .double_value
        )
        self._map_tf_max_translation_jump = (
            self.get_parameter("map_tf_max_translation_jump")
            .get_parameter_value()
            .double_value
        )
        self._map_tf_max_rotation_jump = (
            self.get_parameter("map_tf_max_rotation_jump")
            .get_parameter_value()
            .double_value
        )
        shared_map_source_topic = (
            self.get_parameter("shared_map_source_topic")
            .get_parameter_value()
            .string_value
        )
        self._map_transform_smoother = MapTransformSmoother(
            enabled=self._map_tf_smoothing_enabled,
            alpha=self._map_tf_smoothing_alpha,
            max_translation_jump=self._map_tf_max_translation_jump,
            max_rotation_jump=self._map_tf_max_rotation_jump,
        )
        self._last_jump_warn_ns = 0

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
        self._map_transform_smoother.reset()
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

        parent = _normalize_frame(transform.header.frame_id)
        child = _normalize_frame(transform.child_frame_id)
        if not parent or not child:
            return None

        rewritten = TransformStamped()
        rewritten.header.stamp = transform.header.stamp
        rewritten.header.frame_id = self._rewrite_frame(robot_name, parent)
        rewritten.child_frame_id = self._rewrite_frame(robot_name, child)
        rewritten.transform.translation = transform.transform.translation
        rewritten.transform.rotation = transform.transform.rotation

        if self._should_smooth_map_transform(robot_name, parent, child):
            rewritten, rejected = self._map_transform_smoother.filter(rewritten)
            if rejected:
                now_ns = self.get_clock().now().nanoseconds
                if now_ns - self._last_jump_warn_ns >= 5_000_000_000:
                    self.get_logger().warn(
                        "Rejected shared map TF jump from '%s' on map->%s; keeping the previous filtered transform"
                        % (robot_name, ROOT_FRAME)
                    )
                    self._last_jump_warn_ns = now_ns
        return rewritten

    def _should_smooth_map_transform(
        self, robot_name: str, parent: str, child: str
    ) -> bool:
        return (
            robot_name == self._shared_map_source
            and parent == MAP_FRAME
            and child == ROOT_FRAME
        )

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
