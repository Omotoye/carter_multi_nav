import math
from functools import partial

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from carter_multi_nav.common import DEFAULT_ROOT_POSES, DEFAULT_ROBOTS, FOOTPRINT_POLYGON
from carter_multi_nav.report_utils import append_csv, ensure_output_dir, write_json


def _yaw_from_quaternion(quaternion) -> float:
    siny_cosp = 2.0 * (
        (quaternion.w * quaternion.z) + (quaternion.x * quaternion.y)
    )
    cosy_cosp = 1.0 - 2.0 * (
        (quaternion.y * quaternion.y) + (quaternion.z * quaternion.z)
    )
    return math.atan2(siny_cosp, cosy_cosp)


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _parse_root_pose_overrides(entries):
    poses = dict(DEFAULT_ROOT_POSES)
    for entry in entries:
        text = str(entry or "").strip()
        if not text or "=" not in text:
            continue
        robot_name, values = text.split("=", 1)
        parts = [part.strip() for part in values.split(",") if part.strip()]
        if len(parts) != 4:
            continue
        poses[robot_name.strip()] = tuple(float(part) for part in parts)
    return poses


def _transform_polygon(pose_x, pose_y, pose_yaw):
    cos_yaw = math.cos(pose_yaw)
    sin_yaw = math.sin(pose_yaw)
    transformed = []
    for point_x, point_y in FOOTPRINT_POLYGON:
        transformed.append(
            (
                pose_x + (cos_yaw * point_x) - (sin_yaw * point_y),
                pose_y + (sin_yaw * point_x) + (cos_yaw * point_y),
            )
        )
    return transformed


def _point_in_polygon(point_x, point_y, polygon):
    inside = False
    previous_x, previous_y = polygon[-1]
    for current_x, current_y in polygon:
        intersects = ((current_y > point_y) != (previous_y > point_y)) and (
            point_x
            < (previous_x - current_x) * (point_y - current_y) / (previous_y - current_y + 1e-12)
            + current_x
        )
        if intersects:
            inside = not inside
        previous_x, previous_y = current_x, current_y
    return inside


def _cell_state(value: int, occupied_threshold: int) -> str:
    if value < 0:
        return "unknown"
    if value >= occupied_threshold:
        return "occupied"
    return "free"


class MultiRobotMapMetrics(Node):
    def __init__(self):
        super().__init__("multi_robot_map_metrics")

        self.declare_parameter("robot_names", list(DEFAULT_ROBOTS))
        self.declare_parameter("robot_root_poses", [""])
        self.declare_parameter("report_interval", 5.0)
        self.declare_parameter("output_dir", "")
        self.declare_parameter("occupied_threshold", 50)
        self.declare_parameter("comparison_resolution", 0.20)

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

        self._root_poses = _parse_root_pose_overrides(
            self.get_parameter("robot_root_poses").get_parameter_value().string_array_value
        )
        self._report_interval = (
            self.get_parameter("report_interval").get_parameter_value().double_value
        )
        self._output_dir = ensure_output_dir(
            self.get_parameter("output_dir").get_parameter_value().string_value
        )
        self._occupied_threshold = (
            self.get_parameter("occupied_threshold").get_parameter_value().integer_value
        )
        self._comparison_resolution = (
            self.get_parameter("comparison_resolution").get_parameter_value().double_value
        )

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._maps = {}
        self._global_poses = {}
        self._subscriptions = []

        for robot_name in self._robot_names:
            for map_kind, topic_suffix in (("map", "map"), ("planning_map", "planning_map")):
                self._subscriptions.append(
                    self.create_subscription(
                        OccupancyGrid,
                        f"/{robot_name}/{topic_suffix}",
                        partial(self._handle_map, robot_name, map_kind),
                        map_qos,
                    )
                )
            self._subscriptions.append(
                self.create_subscription(
                    Odometry,
                    f"/{robot_name}/chassis/odom",
                    partial(self._handle_odom, robot_name),
                    odom_qos,
                )
            )

        self._json_path = ""
        self._robot_csv_path = ""
        self._pair_csv_path = ""
        if self._output_dir:
            self._json_path = f"{self._output_dir}/multi_robot_map_metrics.json"
            self._robot_csv_path = f"{self._output_dir}/multi_robot_map_metrics_robot.csv"
            self._pair_csv_path = f"{self._output_dir}/multi_robot_map_metrics_pair.csv"

        self.create_timer(max(self._report_interval, 0.5), self._report)

        self.get_logger().info(
            "Computing map metrics for robots [%s]" % ", ".join(self._robot_names)
        )

    def _handle_map(self, robot_name: str, map_kind: str, msg: OccupancyGrid):
        self._maps[(robot_name, map_kind)] = msg

    def _handle_odom(self, robot_name: str, msg: Odometry):
        root_x, root_y, _root_z, root_yaw = self._root_poses.get(
            robot_name, DEFAULT_ROOT_POSES[robot_name]
        )
        pose = msg.pose.pose
        local_x = float(pose.position.x)
        local_y = float(pose.position.y)
        local_yaw = _yaw_from_quaternion(pose.orientation)
        global_x = root_x + (math.cos(root_yaw) * local_x) - (math.sin(root_yaw) * local_y)
        global_y = root_y + (math.sin(root_yaw) * local_x) + (math.cos(root_yaw) * local_y)
        self._global_poses[robot_name] = {
            "x": global_x,
            "y": global_y,
            "yaw": _normalize_angle(root_yaw + local_yaw),
        }

    def _grid_bounds(self, msg: OccupancyGrid):
        origin = msg.info.origin.position
        width = float(msg.info.width) * float(msg.info.resolution)
        height = float(msg.info.height) * float(msg.info.resolution)
        return (
            float(origin.x),
            float(origin.y),
            float(origin.x) + width,
            float(origin.y) + height,
        )

    def _world_to_grid(self, msg: OccupancyGrid, world_x: float, world_y: float):
        resolution = float(msg.info.resolution)
        if resolution <= 0.0:
            return None
        origin = msg.info.origin
        origin_yaw = _yaw_from_quaternion(origin.orientation)
        dx = world_x - float(origin.position.x)
        dy = world_y - float(origin.position.y)
        local_x = (math.cos(origin_yaw) * dx) + (math.sin(origin_yaw) * dy)
        local_y = (-math.sin(origin_yaw) * dx) + (math.cos(origin_yaw) * dy)
        cell_x = int(math.floor(local_x / resolution))
        cell_y = int(math.floor(local_y / resolution))
        if cell_x < 0 or cell_y < 0:
            return None
        if cell_x >= int(msg.info.width) or cell_y >= int(msg.info.height):
            return None
        return cell_x, cell_y

    def _cell_center_world(self, msg: OccupancyGrid, cell_x: int, cell_y: int):
        resolution = float(msg.info.resolution)
        origin = msg.info.origin
        origin_yaw = _yaw_from_quaternion(origin.orientation)
        local_x = (cell_x + 0.5) * resolution
        local_y = (cell_y + 0.5) * resolution
        return (
            float(origin.position.x)
            + (math.cos(origin_yaw) * local_x)
            - (math.sin(origin_yaw) * local_y),
            float(origin.position.y)
            + (math.sin(origin_yaw) * local_x)
            + (math.cos(origin_yaw) * local_y),
        )

    def _sample_polygon(self, msg: OccupancyGrid, polygon):
        min_x = min(point[0] for point in polygon)
        max_x = max(point[0] for point in polygon)
        min_y = min(point[1] for point in polygon)
        max_y = max(point[1] for point in polygon)

        min_cell = self._world_to_grid(msg, min_x, min_y)
        max_cell = self._world_to_grid(msg, max_x, max_y)
        if min_cell is None or max_cell is None:
            bbox = self._grid_bounds(msg)
            if max_x < bbox[0] or min_x > bbox[2] or max_y < bbox[1] or min_y > bbox[3]:
                return {
                    "occupied_ratio": None,
                    "free_ratio": None,
                    "unknown_ratio": None,
                    "occupied_cells": 0,
                    "free_cells": 0,
                    "unknown_cells": 0,
                    "sampled_cells": 0,
                }
            cell_x_values = [
                value
                for value in (
                    self._world_to_grid(msg, min_x, min_y),
                    self._world_to_grid(msg, min_x, max_y),
                    self._world_to_grid(msg, max_x, min_y),
                    self._world_to_grid(msg, max_x, max_y),
                )
                if value is not None
            ]
            if not cell_x_values:
                return {
                    "occupied_ratio": None,
                    "free_ratio": None,
                    "unknown_ratio": None,
                    "occupied_cells": 0,
                    "free_cells": 0,
                    "unknown_cells": 0,
                    "sampled_cells": 0,
                }
            min_cell_x = max(min(value[0] for value in cell_x_values) - 1, 0)
            max_cell_x = min(max(value[0] for value in cell_x_values) + 1, int(msg.info.width) - 1)
            min_cell_y = max(min(value[1] for value in cell_x_values) - 1, 0)
            max_cell_y = min(max(value[1] for value in cell_x_values) + 1, int(msg.info.height) - 1)
        else:
            min_cell_x = max(min(min_cell[0], max_cell[0]) - 1, 0)
            max_cell_x = min(max(min_cell[0], max_cell[0]) + 1, int(msg.info.width) - 1)
            min_cell_y = max(min(min_cell[1], max_cell[1]) - 1, 0)
            max_cell_y = min(max(min_cell[1], max_cell[1]) + 1, int(msg.info.height) - 1)

        occupied = 0
        free = 0
        unknown = 0
        sampled = 0
        width = int(msg.info.width)
        for cell_y in range(min_cell_y, max_cell_y + 1):
            for cell_x in range(min_cell_x, max_cell_x + 1):
                world_x, world_y = self._cell_center_world(msg, cell_x, cell_y)
                if not _point_in_polygon(world_x, world_y, polygon):
                    continue
                sampled += 1
                value = int(msg.data[(cell_y * width) + cell_x])
                state = _cell_state(value, self._occupied_threshold)
                if state == "occupied":
                    occupied += 1
                elif state == "free":
                    free += 1
                else:
                    unknown += 1

        if sampled == 0:
            return {
                "occupied_ratio": None,
                "free_ratio": None,
                "unknown_ratio": None,
                "occupied_cells": 0,
                "free_cells": 0,
                "unknown_cells": 0,
                "sampled_cells": 0,
            }

        return {
            "occupied_ratio": occupied / float(sampled),
            "free_ratio": free / float(sampled),
            "unknown_ratio": unknown / float(sampled),
            "occupied_cells": occupied,
            "free_cells": free,
            "unknown_cells": unknown,
            "sampled_cells": sampled,
        }

    def _pairwise_comparison(self, left_msg: OccupancyGrid, right_msg: OccupancyGrid):
        left_bounds = self._grid_bounds(left_msg)
        right_bounds = self._grid_bounds(right_msg)
        min_x = max(left_bounds[0], right_bounds[0])
        min_y = max(left_bounds[1], right_bounds[1])
        max_x = min(left_bounds[2], right_bounds[2])
        max_y = min(left_bounds[3], right_bounds[3])
        if min_x >= max_x or min_y >= max_y:
            return {
                "compared_cells": 0,
                "agreement_ratio": None,
                "exclusive_occupied_ratio": None,
                "both_occupied_ratio": None,
            }

        compared = 0
        agreements = 0
        exclusive_occupied = 0
        both_occupied = 0
        step = max(self._comparison_resolution, left_msg.info.resolution, right_msg.info.resolution)
        y = min_y + (0.5 * step)
        while y < max_y:
            x = min_x + (0.5 * step)
            while x < max_x:
                left_cell = self._world_to_grid(left_msg, x, y)
                right_cell = self._world_to_grid(right_msg, x, y)
                if left_cell is None or right_cell is None:
                    x += step
                    continue
                left_index = (left_cell[1] * int(left_msg.info.width)) + left_cell[0]
                right_index = (right_cell[1] * int(right_msg.info.width)) + right_cell[0]
                left_state = _cell_state(int(left_msg.data[left_index]), self._occupied_threshold)
                right_state = _cell_state(int(right_msg.data[right_index]), self._occupied_threshold)
                if "unknown" in (left_state, right_state):
                    x += step
                    continue
                compared += 1
                if left_state == right_state:
                    agreements += 1
                if left_state == "occupied" and right_state == "occupied":
                    both_occupied += 1
                elif left_state != right_state and "occupied" in (left_state, right_state):
                    exclusive_occupied += 1
                x += step
            y += step

        if compared == 0:
            return {
                "compared_cells": 0,
                "agreement_ratio": None,
                "exclusive_occupied_ratio": None,
                "both_occupied_ratio": None,
            }

        return {
            "compared_cells": compared,
            "agreement_ratio": agreements / float(compared),
            "exclusive_occupied_ratio": exclusive_occupied / float(compared),
            "both_occupied_ratio": both_occupied / float(compared),
        }

    def _build_summary(self):
        robots = {}
        pairs = {}

        for robot_name in self._robot_names:
            pose = self._global_poses.get(robot_name)
            robot_metrics = {
                "global_pose": pose,
                "map": {},
                "planning_map": {},
            }
            if pose is None:
                robots[robot_name] = robot_metrics
                continue

            self_polygon = _transform_polygon(pose["x"], pose["y"], pose["yaw"])
            for map_kind in ("map", "planning_map"):
                map_msg = self._maps.get((robot_name, map_kind))
                if map_msg is None:
                    continue
                map_metrics = {
                    "start_occupied_ratio": None,
                    "start_unknown_ratio": None,
                    "peer_occupancy_ratio": None,
                    "peer_metrics": {},
                }
                self_sample = self._sample_polygon(map_msg, self_polygon)
                map_metrics["start_occupied_ratio"] = self_sample["occupied_ratio"]
                map_metrics["start_unknown_ratio"] = self_sample["unknown_ratio"]

                peer_ratios = []
                for peer_name in self._robot_names:
                    if peer_name == robot_name:
                        continue
                    peer_pose = self._global_poses.get(peer_name)
                    if peer_pose is None:
                        continue
                    peer_polygon = _transform_polygon(
                        peer_pose["x"], peer_pose["y"], peer_pose["yaw"]
                    )
                    peer_sample = self._sample_polygon(map_msg, peer_polygon)
                    map_metrics["peer_metrics"][peer_name] = peer_sample
                    if peer_sample["occupied_ratio"] is not None:
                        peer_ratios.append(peer_sample["occupied_ratio"])
                if peer_ratios:
                    map_metrics["peer_occupancy_ratio"] = sum(peer_ratios) / float(len(peer_ratios))
                robot_metrics[map_kind] = map_metrics

            robots[robot_name] = robot_metrics

        for map_kind in ("map", "planning_map"):
            for index, left_name in enumerate(self._robot_names):
                left_msg = self._maps.get((left_name, map_kind))
                if left_msg is None:
                    continue
                for right_name in self._robot_names[index + 1 :]:
                    right_msg = self._maps.get((right_name, map_kind))
                    if right_msg is None:
                        continue
                    pair_key = f"{left_name}__{right_name}__{map_kind}"
                    pairs[pair_key] = {
                        "left_robot": left_name,
                        "right_robot": right_name,
                        "map_kind": map_kind,
                        **self._pairwise_comparison(left_msg, right_msg),
                    }

        return {
            "robot_names": self._robot_names,
            "robots": robots,
            "pairs": pairs,
            "occupied_threshold": self._occupied_threshold,
            "comparison_resolution": self._comparison_resolution,
        }

    def _write_summary(self, summary):
        if not self._output_dir:
            return

        write_json(self._json_path, summary)

        for robot_name, robot_summary in summary["robots"].items():
            for map_kind in ("map", "planning_map"):
                metrics = robot_summary.get(map_kind) or {}
                append_csv(
                    self._robot_csv_path,
                    [
                        "robot_name",
                        "map_kind",
                        "start_occupied_ratio",
                        "start_unknown_ratio",
                        "peer_occupancy_ratio",
                    ],
                    {
                        "robot_name": robot_name,
                        "map_kind": map_kind,
                        "start_occupied_ratio": metrics.get("start_occupied_ratio"),
                        "start_unknown_ratio": metrics.get("start_unknown_ratio"),
                        "peer_occupancy_ratio": metrics.get("peer_occupancy_ratio"),
                    },
                )

        for pair_key, pair_summary in summary["pairs"].items():
            append_csv(
                self._pair_csv_path,
                [
                    "pair_key",
                    "left_robot",
                    "right_robot",
                    "map_kind",
                    "compared_cells",
                    "agreement_ratio",
                    "exclusive_occupied_ratio",
                    "both_occupied_ratio",
                ],
                {"pair_key": pair_key, **pair_summary},
            )

    def _report(self):
        if not self._maps:
            self.get_logger().warn("No maps received yet.")
            return

        summary = self._build_summary()
        self._write_summary(summary)

        robot_lines = []
        for robot_name in self._robot_names:
            planning = summary["robots"].get(robot_name, {}).get("planning_map", {})
            robot_lines.append(
                "%s(start_occ=%s peer_occ=%s)"
                % (
                    robot_name,
                    planning.get("start_occupied_ratio"),
                    planning.get("peer_occupancy_ratio"),
                )
            )
        self.get_logger().info(
            "Map metrics: %s" % ", ".join(robot_lines)
        )


def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotMapMetrics()
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
