import math

import rclpy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from carter_multi_nav.common import DEFAULT_ROOT_POSES, DEFAULT_ROBOTS


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
        self._clear_radius = (
            self.get_parameter("clear_radius").get_parameter_value().double_value
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
        self._global_robot_poses = {}

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
        odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        for robot_name in self._robot_names:
            self._subscriptions.append(
                self.create_subscription(
                    Odometry,
                    f"/{robot_name}/chassis/odom",
                    lambda msg, robot=robot_name: self._handle_odom(robot, msg),
                    odom_qos,
                )
            )
        if publish_frequency > 0.0:
            self._publish_timer = self.create_timer(
                1.0 / publish_frequency, self._publish_latest_map
            )
        else:
            self._publish_timer = None

        self.get_logger().info(
            "Clearing a %.2fm radius robot footprint from '%s' into '%s'"
            % (self._clear_radius, input_topic, output_topic)
        )

    def _handle_map(self, msg: OccupancyGrid):
        if self._shutting_down:
            return
        self._latest_map = msg
        self._publish_cleaned_map(msg)

    def _handle_odom(self, robot_name: str, msg: Odometry):
        root_x, root_y, _root_z, root_yaw = self._root_poses.get(
            robot_name, self._root_poses.get(self._robot_name, (0.0, 0.0, 0.0, 0.0))
        )
        pose = msg.pose.pose
        local_x = float(pose.position.x)
        local_y = float(pose.position.y)
        local_yaw = _yaw_from_quaternion(pose.orientation)
        self._global_robot_poses[robot_name] = {
            "x": root_x + (math.cos(root_yaw) * local_x) - (math.sin(root_yaw) * local_y),
            "y": root_y + (math.sin(root_yaw) * local_x) + (math.cos(root_yaw) * local_y),
            "yaw": root_yaw + local_yaw,
        }

    def _publish_latest_map(self):
        if self._shutting_down or self._latest_map is None:
            return
        self._publish_cleaned_map(self._latest_map)

    def _publish_cleaned_map(self, msg: OccupancyGrid):
        cleaned = OccupancyGrid()
        cleaned.header = msg.header
        cleaned.info = msg.info
        cleaned.data = list(msg.data)

        self_transform = self._lookup_robot_pose()
        if self_transform is None:
            self._publisher.publish(cleaned)
            return

        resolution = float(cleaned.info.resolution)
        if resolution <= 0.0 or cleaned.info.width == 0 or cleaned.info.height == 0:
            self._publisher.publish(cleaned)
            return

        origin_x = float(cleaned.info.origin.position.x)
        origin_y = float(cleaned.info.origin.position.y)
        radius_cells = max(1, int(math.ceil(self._clear_radius / resolution)))
        width = int(cleaned.info.width)
        height = int(cleaned.info.height)

        robot_positions = [
            (
                float(self_transform.transform.translation.x),
                float(self_transform.transform.translation.y),
            )
        ]
        map_to_global = self._lookup_map_to_global()
        if map_to_global is not None:
            yaw = _yaw_from_quaternion(map_to_global.transform.rotation)
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            translation_x = float(map_to_global.transform.translation.x)
            translation_y = float(map_to_global.transform.translation.y)
            for robot_name, pose in self._global_robot_poses.items():
                if robot_name == self._robot_name:
                    continue
                map_x = translation_x + (cos_yaw * float(pose["x"])) - (
                    sin_yaw * float(pose["y"])
                )
                map_y = translation_y + (sin_yaw * float(pose["x"])) + (
                    cos_yaw * float(pose["y"])
                )
                robot_positions.append((map_x, map_y))

        for robot_x, robot_y in robot_positions:
            center_x = int(round((robot_x - origin_x) / resolution))
            center_y = int(round((robot_y - origin_y) / resolution))
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

    def _lookup_map_to_global(self):
        try:
            return self._tf_buffer.lookup_transform(
                self._map_frame,
                "global_odom",
                Time(),
                timeout=Duration(seconds=0.0),
            )
        except TransformException:
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
