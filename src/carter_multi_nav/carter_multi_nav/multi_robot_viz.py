import math
from dataclasses import dataclass
from functools import partial

import rclpy
from geometry_msgs.msg import Point32, PolygonStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

from carter_multi_nav.common import DEFAULT_ROBOTS, MAP_FRAME, ODOM_FRAME, ROOT_FRAME


def _yaw_from_quaternion(quaternion) -> float:
    siny_cosp = 2.0 * (
        (quaternion.w * quaternion.z) + (quaternion.x * quaternion.y)
    )
    cosy_cosp = 1.0 - 2.0 * (
        (quaternion.y * quaternion.y) + (quaternion.z * quaternion.z)
    )
    return math.atan2(siny_cosp, cosy_cosp)


def _write_quaternion(target, yaw: float):
    half_yaw = 0.5 * yaw
    target.x = 0.0
    target.y = 0.0
    target.z = math.sin(half_yaw)
    target.w = math.cos(half_yaw)


@dataclass
class Transform2D:
    x: float
    y: float
    yaw: float

    def apply(self, point_x: float, point_y: float):
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        return (
            self.x + (cos_yaw * point_x) - (sin_yaw * point_y),
            self.y + (sin_yaw * point_x) + (cos_yaw * point_y),
        )

    def compose(self, other: "Transform2D") -> "Transform2D":
        point_x, point_y = self.apply(other.x, other.y)
        return Transform2D(point_x, point_y, self.yaw + other.yaw)

    def inverse(self) -> "Transform2D":
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        return Transform2D(
            x=-(cos_yaw * self.x) - (sin_yaw * self.y),
            y=(sin_yaw * self.x) - (cos_yaw * self.y),
            yaw=-self.yaw,
        )


def _transform_from_msg(transform) -> Transform2D:
    return Transform2D(
        x=float(transform.translation.x),
        y=float(transform.translation.y),
        yaw=_yaw_from_quaternion(transform.rotation),
    )


def _pose_transform_from_msg(pose) -> Transform2D:
    return Transform2D(
        x=float(pose.position.x),
        y=float(pose.position.y),
        yaw=_yaw_from_quaternion(pose.orientation),
    )


class MultiRobotViz(Node):
    def __init__(self):
        super().__init__("multi_robot_viz")

        self.declare_parameter("robot_names", list(DEFAULT_ROBOTS))
        self.declare_parameter("enable_shared_map_merge", False)
        self.declare_parameter("shared_map_topic", "/shared_map")
        self.declare_parameter("shared_map_resolution", 0.05)
        self.declare_parameter("shared_map_publish_frequency", 1.0)
        self.declare_parameter("occupied_threshold", 50)

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

        self._shared_map_resolution = (
            self.get_parameter("shared_map_resolution").get_parameter_value().double_value
        )
        self._occupied_threshold = (
            self.get_parameter("occupied_threshold").get_parameter_value().integer_value
        )
        self._enable_shared_map_merge = (
            self.get_parameter("enable_shared_map_merge")
            .get_parameter_value()
            .bool_value
        )

        map_pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        path_pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        best_effort_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        tf_static_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._shared_map_publisher = None
        if self._enable_shared_map_merge:
            self._shared_map_publisher = self.create_publisher(
                OccupancyGrid,
                self.get_parameter("shared_map_topic").get_parameter_value().string_value,
                map_pub_qos,
            )
        self._plan_publishers = {}
        self._local_plan_publishers = {}
        self._costmap_publishers = {}
        self._footprint_publishers = {}
        self._latest_maps = {}
        self._odom_to_root = {}
        self._root_to_map = {}
        self._subscriptions = []

        for robot_name in self._robot_names:
            self._plan_publishers[robot_name] = self.create_publisher(
                Path, f"/rviz/{robot_name}/plan", path_pub_qos
            )
            self._local_plan_publishers[robot_name] = self.create_publisher(
                Path, f"/rviz/{robot_name}/local_plan", path_pub_qos
            )
            self._costmap_publishers[robot_name] = self.create_publisher(
                OccupancyGrid,
                f"/rviz/{robot_name}/local_costmap/costmap",
                map_pub_qos,
            )
            self._footprint_publishers[robot_name] = self.create_publisher(
                PolygonStamped,
                f"/rviz/{robot_name}/local_costmap/published_footprint",
                reliable_qos,
            )

            if self._enable_shared_map_merge:
                for durability_qos in (map_pub_qos, reliable_qos):
                    self._subscriptions.append(
                        self.create_subscription(
                            OccupancyGrid,
                            f"/{robot_name}/map",
                            partial(self._handle_map, robot_name),
                            durability_qos,
                        )
                    )
            for durability_qos in (map_pub_qos, reliable_qos):
                self._subscriptions.append(
                    self.create_subscription(
                        OccupancyGrid,
                        f"/{robot_name}/local_costmap/costmap",
                        partial(self._handle_local_costmap, robot_name),
                        durability_qos,
                    )
                )

            self._subscriptions.append(
                self.create_subscription(
                    Path,
                    f"/{robot_name}/plan",
                    partial(self._handle_path, robot_name, True),
                    reliable_qos,
                )
            )
            self._subscriptions.append(
                self.create_subscription(
                    Path,
                    f"/{robot_name}/local_plan",
                    partial(self._handle_path, robot_name, False),
                    reliable_qos,
                )
            )
            # Regulated Pure Pursuit exposes the controller-tracked path on
            # `received_global_plan` rather than publishing `local_plan`.
            self._subscriptions.append(
                self.create_subscription(
                    Path,
                    f"/{robot_name}/received_global_plan",
                    partial(self._handle_path, robot_name, False),
                    reliable_qos,
                )
            )
            self._subscriptions.append(
                self.create_subscription(
                    PolygonStamped,
                    f"/{robot_name}/local_costmap/published_footprint",
                    partial(self._handle_footprint, robot_name),
                    reliable_qos,
                )
            )
            self._subscriptions.append(
                self.create_subscription(
                    TFMessage,
                    f"/{robot_name}/tf",
                    partial(self._handle_tf, robot_name),
                    best_effort_qos,
                )
            )
            self._subscriptions.append(
                self.create_subscription(
                    TFMessage,
                    f"/{robot_name}/tf_static",
                    partial(self._handle_tf, robot_name),
                    tf_static_qos,
                )
            )

        self._publish_timer = None
        publish_frequency = (
            self.get_parameter("shared_map_publish_frequency")
            .get_parameter_value()
            .double_value
        )
        if self._enable_shared_map_merge and publish_frequency > 0.0:
            self._publish_timer = self.create_timer(
                1.0 / publish_frequency, self._publish_shared_map
            )

        self.get_logger().info(
            "Publishing RViz nav relays for robots [%s]%s"
            % (
                ", ".join(self._robot_names),
                " with shared map merge enabled" if self._enable_shared_map_merge else "",
            )
        )

    def _handle_tf(self, robot_name: str, msg: TFMessage):
        for transform in msg.transforms:
            parent = str(transform.header.frame_id or "").lstrip("/")
            child = str(transform.child_frame_id or "").lstrip("/")
            transform_2d = _transform_from_msg(transform.transform)
            if parent == ROOT_FRAME and child == ODOM_FRAME:
                self._odom_to_root[robot_name] = transform_2d
            elif parent == MAP_FRAME and child == ROOT_FRAME:
                self._root_to_map[robot_name] = transform_2d

    def _root_from_frame(self, robot_name: str, frame_id: str):
        frame = str(frame_id or "").lstrip("/")
        if frame == ROOT_FRAME:
            return Transform2D(0.0, 0.0, 0.0)
        if frame == ODOM_FRAME:
            return self._odom_to_root.get(robot_name)
        if frame == MAP_FRAME:
            root_to_map = self._root_to_map.get(robot_name)
            if root_to_map is None:
                return None
            return root_to_map.inverse()
        return None

    def _transform_pose(self, transform: Transform2D, pose):
        source_pose = _pose_transform_from_msg(pose)
        transformed = transform.compose(source_pose)
        pose.position.x = transformed.x
        pose.position.y = transformed.y
        pose.position.z = 0.0
        _write_quaternion(pose.orientation, transformed.yaw)

    def _handle_map(self, robot_name: str, msg: OccupancyGrid):
        self._latest_maps[robot_name] = msg

    def _handle_local_costmap(self, robot_name: str, msg: OccupancyGrid):
        transform = self._root_from_frame(robot_name, msg.header.frame_id)
        if transform is None:
            return

        relayed = OccupancyGrid()
        relayed.header = msg.header
        relayed.header.frame_id = ROOT_FRAME
        relayed.info = msg.info
        relayed.data = list(msg.data)

        origin = _pose_transform_from_msg(msg.info.origin)
        transformed_origin = transform.compose(origin)
        relayed.info.origin.position.x = transformed_origin.x
        relayed.info.origin.position.y = transformed_origin.y
        relayed.info.origin.position.z = 0.0
        _write_quaternion(relayed.info.origin.orientation, transformed_origin.yaw)

        self._costmap_publishers[robot_name].publish(relayed)

    def _handle_path(self, robot_name: str, is_global_plan: bool, msg: Path):
        transform = self._root_from_frame(robot_name, msg.header.frame_id)
        if transform is None:
            return

        relayed = Path()
        relayed.header = msg.header
        relayed.header.frame_id = ROOT_FRAME
        for pose_stamped in msg.poses:
            pose_copy = PoseStamped()
            pose_copy.header = pose_stamped.header
            pose_copy.header.frame_id = ROOT_FRAME
            pose_copy.pose = pose_stamped.pose
            self._transform_pose(transform, pose_copy.pose)
            relayed.poses.append(pose_copy)

        if is_global_plan:
            self._plan_publishers[robot_name].publish(relayed)
        else:
            self._local_plan_publishers[robot_name].publish(relayed)

    def _handle_footprint(self, robot_name: str, msg: PolygonStamped):
        transform = self._root_from_frame(robot_name, msg.header.frame_id)
        if transform is None:
            return

        relayed = PolygonStamped()
        relayed.header = msg.header
        relayed.header.frame_id = ROOT_FRAME
        for point in msg.polygon.points:
            transformed_x, transformed_y = transform.apply(
                float(point.x), float(point.y)
            )
            point_copy = Point32()
            point_copy.x = transformed_x
            point_copy.y = transformed_y
            point_copy.z = 0.0
            relayed.polygon.points.append(point_copy)
        self._footprint_publishers[robot_name].publish(relayed)

    def _map_bounds(self, robot_name: str, msg: OccupancyGrid):
        transform = self._root_from_frame(robot_name, msg.header.frame_id)
        if transform is None:
            return None

        origin = transform.compose(_pose_transform_from_msg(msg.info.origin))
        width_m = float(msg.info.width) * float(msg.info.resolution)
        height_m = float(msg.info.height) * float(msg.info.resolution)
        corners = [
            origin.apply(0.0, 0.0),
            origin.apply(width_m, 0.0),
            origin.apply(0.0, height_m),
            origin.apply(width_m, height_m),
        ]
        xs = [point[0] for point in corners]
        ys = [point[1] for point in corners]
        return {
            "transform": origin,
            "min_x": min(xs),
            "max_x": max(xs),
            "min_y": min(ys),
            "max_y": max(ys),
        }

    def _publish_shared_map(self):
        if not self._enable_shared_map_merge or self._shared_map_publisher is None:
            return
        if not self._latest_maps:
            return

        valid_maps = []
        for robot_name, msg in self._latest_maps.items():
            bounds = self._map_bounds(robot_name, msg)
            if bounds is not None:
                valid_maps.append((robot_name, msg, bounds))
        if not valid_maps:
            return

        resolution = max(self._shared_map_resolution, 0.01)
        min_x = min(item[2]["min_x"] for item in valid_maps)
        max_x = max(item[2]["max_x"] for item in valid_maps)
        min_y = min(item[2]["min_y"] for item in valid_maps)
        max_y = max(item[2]["max_y"] for item in valid_maps)

        width = max(1, int(math.ceil((max_x - min_x) / resolution)))
        height = max(1, int(math.ceil((max_y - min_y) / resolution)))
        merged = [-1] * (width * height)

        for _robot_name, msg, bounds in valid_maps:
            origin = bounds["transform"]
            source_resolution = float(msg.info.resolution)
            source_width = int(msg.info.width)
            for index, cell in enumerate(msg.data):
                if cell < 0:
                    continue
                cell_x = index % source_width
                cell_y = index // source_width
                world_x, world_y = origin.apply(
                    (cell_x + 0.5) * source_resolution,
                    (cell_y + 0.5) * source_resolution,
                )
                merged_x = int(math.floor((world_x - min_x) / resolution))
                merged_y = int(math.floor((world_y - min_y) / resolution))
                if merged_x < 0 or merged_y < 0 or merged_x >= width or merged_y >= height:
                    continue
                merged_index = (merged_y * width) + merged_x
                if cell >= self._occupied_threshold:
                    merged[merged_index] = 100
                elif merged[merged_index] != 100:
                    merged[merged_index] = 0

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ROOT_FRAME
        msg.info.map_load_time = msg.header.stamp
        msg.info.resolution = resolution
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = min_x
        msg.info.origin.position.y = min_y
        msg.info.origin.position.z = 0.0
        _write_quaternion(msg.info.origin.orientation, 0.0)
        msg.data = merged
        self._shared_map_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotViz()
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
