import math

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from carter_multi_nav.common import (
    DEFAULT_ROOT_POSES,
    DEFAULT_ROBOTS,
    FOOTPRINT_POLYGON,
    prefixed_frame,
)


def _yaw_from_quaternion(quaternion) -> float:
    siny_cosp = 2.0 * (
        (quaternion.w * quaternion.z) + (quaternion.x * quaternion.y)
    )
    cosy_cosp = 1.0 - 2.0 * (
        (quaternion.y * quaternion.y) + (quaternion.z * quaternion.z)
    )
    return math.atan2(siny_cosp, cosy_cosp)


def _parse_root_poses(raw_value: str):
    root_poses = dict(DEFAULT_ROOT_POSES)
    text = str(raw_value or "").strip()
    if not text:
        return root_poses

    for entry in text.split(";"):
        chunk = entry.strip()
        if not chunk or "=" not in chunk:
            continue
        robot_name, pose_csv = chunk.split("=", 1)
        robot_name = robot_name.strip()
        parts = [part.strip() for part in pose_csv.split(",") if part.strip()]
        if robot_name and len(parts) == 4:
            root_poses[robot_name] = tuple(float(part) for part in parts)

    return root_poses


def _inflated_footprint_bounds(padding: float):
    xs = [point[0] for point in FOOTPRINT_POLYGON]
    ys = [point[1] for point in FOOTPRINT_POLYGON]
    return (
        min(xs) - padding,
        max(xs) + padding,
        min(ys) - padding,
        max(ys) + padding,
    )


class PlanningMapClearer(Node):
    def __init__(self):
        super().__init__("planning_map_clearer")

        self.declare_parameter("robot_name", "")
        self.declare_parameter("robot_names", list(DEFAULT_ROBOTS))
        self.declare_parameter("input_topic", "map")
        self.declare_parameter("output_topic", "planning_map")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("root_poses", "")
        self.declare_parameter("clear_radius", 0.60)
        self.declare_parameter("footprint_padding", 0.10)
        self.declare_parameter("publish_frequency", 5.0)

        self._robot_name = (
            self.get_parameter("robot_name").get_parameter_value().string_value.strip()
        )
        if not self._robot_name:
            self._robot_name = self.get_namespace().strip("/")
        configured_robot_names = list(
            self.get_parameter("robot_names").get_parameter_value().string_array_value
        )
        self._robot_names = [
            robot_name.strip()
            for robot_name in configured_robot_names
            if robot_name.strip()
        ]
        if not self._robot_names:
            self._robot_names = list(DEFAULT_ROBOTS)
        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self._map_frame = self.get_parameter("map_frame").get_parameter_value().string_value
        self._base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self._root_poses = _parse_root_poses(
            self.get_parameter("root_poses").get_parameter_value().string_value
        )
        footprint_padding = (
            self.get_parameter("footprint_padding").get_parameter_value().double_value
        )
        self._clear_radius = (
            self.get_parameter("clear_radius").get_parameter_value().double_value
        )
        self._footprint_bounds = _inflated_footprint_bounds(footprint_padding)
        self._lookup_radius = max(
            self._clear_radius,
            max(
                math.hypot(bound_x, bound_y)
                for bound_x in self._footprint_bounds[:2]
                for bound_y in self._footprint_bounds[2:]
            ),
        )
        publish_frequency = (
            self.get_parameter("publish_frequency").get_parameter_value().double_value
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
        self._shutting_down = False
        self._latest_map = None

        # The live SLAM map durability differs between runs, so subscribe both
        # ways and normalize it into a single transient-local planning map.
        self._subscriptions = [
            self.create_subscription(
                OccupancyGrid, input_topic, self._handle_map, transient_qos
            ),
            self.create_subscription(
                OccupancyGrid, input_topic, self._handle_map, volatile_qos
            ),
        ]
        if publish_frequency > 0.0:
            self._publish_timer = self.create_timer(
                1.0 / publish_frequency, self._publish_latest_map
            )
        else:
            self._publish_timer = None

        self.get_logger().info(
            "Clearing robot footprints from '%s' into '%s' using aggregated shared TF"
            % (input_topic, output_topic)
        )

    def _handle_map(self, msg: OccupancyGrid):
        if self._shutting_down:
            return
        self._latest_map = msg
        self._publish_cleaned_map(msg)

    def _publish_latest_map(self):
        if self._shutting_down or self._latest_map is None:
            return
        self._publish_cleaned_map(self._latest_map)

    def _publish_cleaned_map(self, msg: OccupancyGrid):
        cleaned = OccupancyGrid()
        cleaned.header = msg.header
        cleaned.info = msg.info
        cleaned.data = list(msg.data)

        resolution = float(cleaned.info.resolution)
        if resolution <= 0.0 or cleaned.info.width == 0 or cleaned.info.height == 0:
            self._publisher.publish(cleaned)
            return

        origin_x = float(cleaned.info.origin.position.x)
        origin_y = float(cleaned.info.origin.position.y)
        width = int(cleaned.info.width)
        height = int(cleaned.info.height)
        cleared_any = False
        for robot_name in self._robot_names:
            transform = self._lookup_robot_pose(robot_name)
            if transform is None:
                continue
            cleared_any = True
            self._clear_robot_footprint(
                cleaned,
                robot_x=float(transform.transform.translation.x),
                robot_y=float(transform.transform.translation.y),
                robot_yaw=_yaw_from_quaternion(transform.transform.rotation),
                origin_x=origin_x,
                origin_y=origin_y,
                resolution=resolution,
                width=width,
                height=height,
            )

        if not cleared_any:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_tf_warn_ns >= 5_000_000_000:
                self.get_logger().warn(
                    "Publishing raw planning map because no robot poses are available in the shared map frame yet."
                )
                self._last_tf_warn_ns = now_ns

        self._publisher.publish(cleaned)

    def _clear_robot_footprint(
        self,
        cleaned: OccupancyGrid,
        *,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        origin_x: float,
        origin_y: float,
        resolution: float,
        width: int,
        height: int,
    ):
        min_local_x, max_local_x, min_local_y, max_local_y = self._footprint_bounds
        radius_cells = max(1, int(math.ceil(self._lookup_radius / resolution)))
        center_x = int(round((robot_x - origin_x) / resolution))
        center_y = int(round((robot_y - origin_y) / resolution))
        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)

        for dy in range(-radius_cells, radius_cells + 1):
            cell_y = center_y + dy
            if cell_y < 0 or cell_y >= height:
                continue
            world_y = origin_y + ((cell_y + 0.5) * resolution)
            for dx in range(-radius_cells, radius_cells + 1):
                cell_x = center_x + dx
                if cell_x < 0 or cell_x >= width:
                    continue

                world_x = origin_x + ((cell_x + 0.5) * resolution)
                rel_x = world_x - robot_x
                rel_y = world_y - robot_y
                local_x = (cos_yaw * rel_x) + (sin_yaw * rel_y)
                local_y = (-sin_yaw * rel_x) + (cos_yaw * rel_y)
                if (
                    local_x < min_local_x
                    or local_x > max_local_x
                    or local_y < min_local_y
                    or local_y > max_local_y
                ):
                    continue

                cleaned.data[(cell_y * width) + cell_x] = 0

    def _lookup_robot_pose(self, robot_name: str):
        try:
            return self._tf_buffer.lookup_transform(
                self._map_frame,
                prefixed_frame(robot_name, self._base_frame),
                Time(),
                timeout=Duration(seconds=0.0),
            )
        except TransformException as exc:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_tf_warn_ns >= 5_000_000_000:
                self.get_logger().warn(
                    "Planning map clearing is waiting for shared TF %s -> %s: %s"
                    % (
                        self._map_frame,
                        prefixed_frame(robot_name, self._base_frame),
                        exc,
                    )
                )
                self._last_tf_warn_ns = now_ns
            return None

    def shutdown(self):
        if self._shutting_down:
            return

        self._shutting_down = True
        if self._publish_timer is not None:
            try:
                self.destroy_timer(self._publish_timer)
            except Exception:
                pass
            self._publish_timer = None
        for subscription in self._subscriptions:
            try:
                self.destroy_subscription(subscription)
            except Exception:
                pass
        self._subscriptions.clear()

    def should_ignore_runtime_error(self, exc: RuntimeError) -> bool:
        if self._shutting_down or not rclpy.ok():
            return True
        return "Unable to convert call argument" in str(exc)


def main(args=None):
    rclpy.init(args=args)
    node = PlanningMapClearer()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as exc:
        if not node.should_ignore_runtime_error(exc):
            raise
    finally:
        node.shutdown()
        try:
            executor.remove_node(node)
        except Exception:
            pass
        try:
            executor.shutdown()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
