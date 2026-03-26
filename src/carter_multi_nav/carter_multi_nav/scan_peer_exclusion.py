import math

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from carter_multi_nav.common import (
    BASE_FOOTPRINT_FRAME,
    DEFAULT_ROBOTS,
    FOOTPRINT_POLYGON,
    LIDAR_FRAME,
    prefixed_frame,
)


def _is_finite_range(value: float, minimum: float, maximum: float) -> bool:
    return math.isfinite(value) and minimum <= value <= maximum


def _yaw_from_quaternion(quaternion) -> float:
    siny_cosp = 2.0 * (
        (quaternion.w * quaternion.z) + (quaternion.x * quaternion.y)
    )
    cosy_cosp = 1.0 - 2.0 * (
        (quaternion.y * quaternion.y) + (quaternion.z * quaternion.z)
    )
    return math.atan2(siny_cosp, cosy_cosp)


def _inflated_rectangle_bounds(margin: float):
    xs = [point[0] for point in FOOTPRINT_POLYGON]
    ys = [point[1] for point in FOOTPRINT_POLYGON]
    return (
        min(xs) - margin,
        max(xs) + margin,
        min(ys) - margin,
        max(ys) + margin,
    )


def _ray_rectangle_entry_distance(origin_x, origin_y, dir_x, dir_y, bounds):
    min_x, max_x, min_y, max_y = bounds
    epsilon = 1e-9
    t_min = float("-inf")
    t_max = float("inf")

    for origin, direction, lower, upper in (
        (origin_x, dir_x, min_x, max_x),
        (origin_y, dir_y, min_y, max_y),
    ):
        if abs(direction) < epsilon:
            if origin < lower or origin > upper:
                return None
            continue

        first = (lower - origin) / direction
        second = (upper - origin) / direction
        if first > second:
            first, second = second, first
        t_min = max(t_min, first)
        t_max = min(t_max, second)
        if t_min > t_max:
            return None

    if t_max < max(t_min, 0.0):
        return None
    return max(t_min, 0.0)


class ScanPeerExclusion(Node):
    def __init__(self):
        super().__init__("scan_peer_exclusion")

        self.declare_parameter("robot_name", "")
        self.declare_parameter("robot_names", list(DEFAULT_ROBOTS))
        self.declare_parameter("scan_in", "scan_filtered")
        self.declare_parameter("scan_out", "scan_peer_filtered")
        self.declare_parameter("peer_base_frame", BASE_FOOTPRINT_FRAME)
        self.declare_parameter("peer_exclusion_margin", 0.40)
        self.declare_parameter("range_tolerance", 0.20)
        self.declare_parameter("status_interval", 5.0)
        self.declare_parameter("cached_pose_max_age", 2.0)
        # Masked beams are set to this range instead of inf so the costmap
        # raytrace clears stale obstacle cells where the peer was.  Must be
        # between obstacle_max_range (6.0) and raytrace_max_range (8.0) so
        # the beam clears but does not mark a new obstacle.
        self.declare_parameter("clearing_range", 7.0)

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

        self._peer_names = [
            robot_name
            for robot_name in self._robot_names
            if robot_name != self._robot_name
        ]
        self._peer_base_frame = (
            self.get_parameter("peer_base_frame").get_parameter_value().string_value
        ).strip()
        scan_out = (
            self.get_parameter("scan_out").get_parameter_value().string_value.strip()
            or "scan_peer_filtered"
        )
        self._range_tolerance = (
            self.get_parameter("range_tolerance").get_parameter_value().double_value
        )
        self._clearing_range = (
            self.get_parameter("clearing_range").get_parameter_value().double_value
        )
        self._bounds = _inflated_rectangle_bounds(
            self.get_parameter("peer_exclusion_margin")
            .get_parameter_value()
            .double_value
        )
        status_interval = (
            self.get_parameter("status_interval").get_parameter_value().double_value
        )

        scan_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._publisher = self.create_publisher(LaserScan, scan_out, scan_qos)
        self._subscription = self.create_subscription(
            LaserScan,
            self.get_parameter("scan_in").get_parameter_value().string_value,
            self._handle_scan,
            scan_qos,
        )
        self._tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self._last_tf_warn_ns = 0
        self._cached_pose_max_age = (
            self.get_parameter("cached_pose_max_age").get_parameter_value().double_value
        )

        # Cache of last-known peer poses so TF dropouts don't cause
        # unfiltered scans to leak through.  Keys are peer names.
        self._cached_peer_poses = {}  # {peer_name: {"x", "y", "yaw", "stamp_ns"}}

        self._received_scans = 0
        self._published_scans = 0
        self._masked_scans = 0
        self._masked_beams = 0
        self._peers_available = 0
        self._cache_hits = 0

        self.create_timer(max(status_interval, 0.5), self._report)

        self.get_logger().info(
            "Masking peer robots from SLAM scans for '%s' using peers [%s]"
            % (self._robot_name, ", ".join(self._peer_names) or "<none>")
        )

    def _lookup_peer_poses(self, scan_frame: str):
        peer_poses = []
        now_ns = self.get_clock().now().nanoseconds
        max_age_ns = int(self._cached_pose_max_age * 1e9)

        for peer_name in self._peer_names:
            try:
                transform = self._tf_buffer.lookup_transform(
                    scan_frame,
                    prefixed_frame(peer_name, self._peer_base_frame),
                    Time(),
                    timeout=Duration(seconds=0.0),
                )
            except TransformException as exc:
                # TF failed — fall back to cached pose if recent enough.
                cached = self._cached_peer_poses.get(peer_name)
                if cached is not None and (now_ns - cached["stamp_ns"]) < max_age_ns:
                    peer_poses.append(
                        {
                            "name": cached["name"],
                            "x": cached["x"],
                            "y": cached["y"],
                            "yaw": cached["yaw"],
                        }
                    )
                    self._cache_hits += 1
                else:
                    if now_ns - self._last_tf_warn_ns >= 5_000_000_000:
                        self.get_logger().warn(
                            "Peer exclusion has no TF and no recent cache for '%s': %s"
                            % (peer_name, exc)
                        )
                        self._last_tf_warn_ns = now_ns
                continue

            translation = transform.transform.translation
            pose = {
                "name": peer_name,
                "x": float(translation.x),
                "y": float(translation.y),
                "yaw": _yaw_from_quaternion(transform.transform.rotation),
            }
            # Update cache with fresh TF data.
            self._cached_peer_poses[peer_name] = {
                **pose,
                "stamp_ns": now_ns,
            }
            peer_poses.append(pose)
        return peer_poses

    def _mask_scan(self, msg: LaserScan, peer_poses):
        masked_ranges = list(msg.ranges)
        masked_intensities = list(msg.intensities)
        masked_beam_count = 0

        for index, scan_range in enumerate(msg.ranges):
            if not _is_finite_range(scan_range, msg.range_min, msg.range_max):
                continue

            beam_angle = msg.angle_min + (index * msg.angle_increment)
            for peer_pose in peer_poses:
                peer_distance = math.hypot(peer_pose["x"], peer_pose["y"])
                if peer_distance > msg.range_max + 1.5:
                    continue

                local_origin_x = -(
                    math.cos(peer_pose["yaw"]) * peer_pose["x"]
                    + math.sin(peer_pose["yaw"]) * peer_pose["y"]
                )
                local_origin_y = (
                    math.sin(peer_pose["yaw"]) * peer_pose["x"]
                    - math.cos(peer_pose["yaw"]) * peer_pose["y"]
                )
                local_angle = beam_angle - peer_pose["yaw"]
                local_dir_x = math.cos(local_angle)
                local_dir_y = math.sin(local_angle)
                entry_distance = _ray_rectangle_entry_distance(
                    local_origin_x,
                    local_origin_y,
                    local_dir_x,
                    local_dir_y,
                    self._bounds,
                )
                if entry_distance is None:
                    continue
                if scan_range + self._range_tolerance < entry_distance:
                    continue

                masked_ranges[index] = self._clearing_range
                if index < len(masked_intensities):
                    masked_intensities[index] = 0.0
                masked_beam_count += 1
                break

        if masked_beam_count == 0:
            return msg, 0

        masked = LaserScan()
        masked.header = msg.header
        masked.angle_min = msg.angle_min
        masked.angle_max = msg.angle_max
        masked.angle_increment = msg.angle_increment
        masked.time_increment = msg.time_increment
        masked.scan_time = msg.scan_time
        masked.range_min = msg.range_min
        masked.range_max = msg.range_max
        masked.ranges = masked_ranges
        masked.intensities = masked_intensities
        return masked, masked_beam_count

    def _handle_scan(self, msg: LaserScan):
        self._received_scans += 1
        scan_frame = prefixed_frame(
            self._robot_name, msg.header.frame_id or LIDAR_FRAME
        )
        peer_poses = self._lookup_peer_poses(scan_frame)
        self._peers_available = max(self._peers_available, len(peer_poses))
        if not peer_poses:
            self._publisher.publish(msg)
            self._published_scans += 1
            return

        filtered_scan, masked_beam_count = self._mask_scan(msg, peer_poses)
        if masked_beam_count > 0:
            self._masked_scans += 1
            self._masked_beams += masked_beam_count
        self._publisher.publish(filtered_scan)
        self._published_scans += 1

    def _report(self):
        if self._received_scans == 0:
            self.get_logger().warn("No scans received yet.")
            return

        self.get_logger().info(
            "Peer exclusion stats: scans=%d published=%d masked_scans=%d masked_beams=%d peers_seen=%d cache_hits=%d"
            % (
                self._received_scans,
                self._published_scans,
                self._masked_scans,
                self._masked_beams,
                self._peers_available,
                self._cache_hits,
            )
        )


def main(args=None):
    rclpy.init(args=args)
    node = ScanPeerExclusion()
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
