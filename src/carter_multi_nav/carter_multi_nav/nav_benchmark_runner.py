import math
import time

import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from carter_multi_nav.common import DEFAULT_ROBOTS
from carter_multi_nav.report_utils import append_csv, ensure_output_dir, write_json


_STATUS_NAMES = {
    0: "UNKNOWN",
    1: "ACCEPTED",
    2: "EXECUTING",
    3: "CANCELING",
    4: "SUCCEEDED",
    5: "CANCELED",
    6: "ABORTED",
}


def _yaw_to_quaternion(yaw: float):
    half_yaw = 0.5 * yaw
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(half_yaw),
        "w": math.cos(half_yaw),
    }


def _safe_average(values):
    if not values:
        return None
    return sum(values) / float(len(values))


class NavBenchmarkRunner(Node):
    def __init__(self):
        super().__init__("nav_benchmark_runner")

        self.declare_parameter("goal_file", "")
        self.declare_parameter("scenario_names_csv", "")
        self.declare_parameter("output_dir", "")
        self.declare_parameter("robot_names", list(DEFAULT_ROBOTS))
        self.declare_parameter("action_name", "navigate_to_pose")
        self.declare_parameter("inter_goal_delay", 1.0)
        self.declare_parameter("server_timeout", 15.0)
        self.declare_parameter("cmd_motion_threshold", 0.02)
        self.declare_parameter("odom_motion_threshold", 0.02)

        self._goal_file = (
            self.get_parameter("goal_file").get_parameter_value().string_value.strip()
        )
        if not self._goal_file:
            raise RuntimeError("nav_benchmark_runner requires goal_file")

        self._selected_scenarios = [
            item.strip()
            for item in self.get_parameter("scenario_names_csv")
            .get_parameter_value()
            .string_value.split(",")
            if item.strip()
        ]
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

        self._action_name = (
            self.get_parameter("action_name").get_parameter_value().string_value
        ).strip()
        self._inter_goal_delay = (
            self.get_parameter("inter_goal_delay").get_parameter_value().double_value
        )
        self._server_timeout = (
            self.get_parameter("server_timeout").get_parameter_value().double_value
        )
        self._cmd_motion_threshold = (
            self.get_parameter("cmd_motion_threshold").get_parameter_value().double_value
        )
        self._odom_motion_threshold = (
            self.get_parameter("odom_motion_threshold").get_parameter_value().double_value
        )
        self._output_dir = ensure_output_dir(
            self.get_parameter("output_dir").get_parameter_value().string_value
        )

        self._scenarios = self._load_goals(self._goal_file)
        self._action_clients = {
            robot_name: ActionClient(
                self, NavigateToPose, f"/{robot_name}/{self._action_name}"
            )
            for robot_name in self._robot_names
        }

        best_effort_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._subscriptions = []
        for robot_name in self._robot_names:
            self._subscriptions.append(
                self.create_subscription(
                    Twist,
                    f"/{robot_name}/cmd_vel",
                    lambda msg, robot=robot_name: self._handle_cmd_vel(robot, msg),
                    best_effort_qos,
                )
            )
            self._subscriptions.append(
                self.create_subscription(
                    Odometry,
                    f"/{robot_name}/chassis/odom",
                    lambda msg, robot=robot_name: self._handle_odom(robot, msg),
                    best_effort_qos,
                )
            )
            self._subscriptions.append(
                self.create_subscription(
                    Path,
                    f"/{robot_name}/plan",
                    lambda msg, robot=robot_name: self._handle_plan(robot, msg),
                    reliable_qos,
                )
            )

        self._records = []
        self._current_goal = None

        self.get_logger().info(
            "Running navigation benchmarks from '%s' for scenarios [%s]"
            % (self._goal_file, ", ".join(self._selected_scenarios) or "all")
        )

    def _load_goals(self, goal_file: str):
        with open(goal_file, "r", encoding="utf-8") as handle:
            payload = yaml.safe_load(handle) or {}

        scenarios = {}
        if isinstance(payload.get("scenarios"), dict):
            source = payload["scenarios"]
        elif isinstance(payload.get("goals"), list):
            source = {"default": {"goals": payload["goals"]}}
        else:
            raise RuntimeError("goal_file must contain 'scenarios' or 'goals'")

        for scenario_name, scenario_payload in source.items():
            if isinstance(scenario_payload, dict):
                goals = list(scenario_payload.get("goals") or [])
            elif isinstance(scenario_payload, list):
                goals = list(scenario_payload)
            else:
                goals = []
            scenarios[scenario_name] = goals

        if self._selected_scenarios:
            missing = [
                name for name in self._selected_scenarios if name not in scenarios
            ]
            if missing:
                raise RuntimeError(
                    "Unknown benchmark scenarios: %s" % ", ".join(missing)
                )
        return scenarios

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _active_record(self, robot_name: str):
        current = self._current_goal
        if current is None or current["robot_name"] != robot_name:
            return None
        return current

    def _handle_cmd_vel(self, robot_name: str, msg: Twist):
        record = self._active_record(robot_name)
        if record is None:
            return

        linear_speed = abs(float(msg.linear.x))
        angular_speed = abs(float(msg.angular.z))
        record["cmd_linear_samples"].append(linear_speed)
        record["cmd_angular_samples"].append(angular_speed)
        if (
            record["first_nonzero_cmd_time_sec"] is None
            and linear_speed >= self._cmd_motion_threshold
        ):
            record["first_nonzero_cmd_time_sec"] = self._now_sec()

    def _handle_odom(self, robot_name: str, msg: Odometry):
        record = self._active_record(robot_name)
        if record is None:
            return

        twist = msg.twist.twist
        linear_speed = math.hypot(float(twist.linear.x), float(twist.linear.y))
        angular_speed = abs(float(twist.angular.z))
        record["odom_linear_samples"].append(linear_speed)
        record["odom_angular_samples"].append(angular_speed)
        if (
            record["first_nonzero_odom_time_sec"] is None
            and linear_speed >= self._odom_motion_threshold
        ):
            record["first_nonzero_odom_time_sec"] = self._now_sec()

    def _handle_plan(self, robot_name: str, _msg: Path):
        record = self._active_record(robot_name)
        if record is None or record["first_plan_time_sec"] is not None:
            return
        record["first_plan_time_sec"] = self._now_sec()

    def _feedback_callback(self, robot_name: str, feedback_msg):
        record = self._active_record(robot_name)
        if record is None:
            return
        if record["first_feedback_time_sec"] is None:
            record["first_feedback_time_sec"] = self._now_sec()
        current_pose = feedback_msg.feedback.current_pose.pose.position
        record["last_feedback_pose"] = {
            "x": float(current_pose.x),
            "y": float(current_pose.y),
            "z": float(current_pose.z),
        }

    def _spin_until(self, predicate, timeout_sec: float = None):
        start = time.monotonic()
        while not predicate():
            rclpy.spin_once(self, timeout_sec=0.1)
            if timeout_sec is not None and (time.monotonic() - start) >= timeout_sec:
                return False
        return True

    def _goal_message(self, goal_spec: dict):
        yaw = float(goal_spec.get("yaw", 0.0))
        quaternion = _yaw_to_quaternion(yaw)
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = str(goal_spec.get("frame_id", "map"))
        goal.pose.pose.position.x = float(goal_spec["x"])
        goal.pose.pose.position.y = float(goal_spec["y"])
        goal.pose.pose.position.z = float(goal_spec.get("z", 0.0))
        goal.pose.pose.orientation.x = quaternion["x"]
        goal.pose.pose.orientation.y = quaternion["y"]
        goal.pose.pose.orientation.z = quaternion["z"]
        goal.pose.pose.orientation.w = quaternion["w"]
        return goal

    def _finalize_record(self, record, result=None, error_message=""):
        record["result_time_sec"] = self._now_sec()
        if result is None:
            record["status"] = "ERROR"
            record["error_code"] = None
            record["error_msg"] = error_message
        else:
            status_value = int(result.status)
            record["status"] = _STATUS_NAMES.get(status_value, str(status_value))
            record["error_code"] = int(result.result.error_code)
            record["error_msg"] = result.result.error_msg.strip()

        for field in (
            "accepted_time_sec",
            "first_plan_time_sec",
            "first_feedback_time_sec",
            "first_nonzero_cmd_time_sec",
            "first_nonzero_odom_time_sec",
        ):
            if record[field] is not None:
                record[field.replace("_time_sec", "_latency_sec")] = (
                    record[field] - record["sent_time_sec"]
                )
            else:
                record[field.replace("_time_sec", "_latency_sec")] = None

        if record["accepted_time_sec"] is not None:
            record["travel_time_sec"] = (
                record["result_time_sec"] - record["accepted_time_sec"]
            )
        else:
            record["travel_time_sec"] = None

        record["cmd_linear_mean"] = _safe_average(record["cmd_linear_samples"])
        record["cmd_linear_max"] = (
            max(record["cmd_linear_samples"]) if record["cmd_linear_samples"] else None
        )
        record["cmd_angular_mean"] = _safe_average(record["cmd_angular_samples"])
        record["odom_linear_mean"] = _safe_average(record["odom_linear_samples"])
        record["odom_linear_max"] = (
            max(record["odom_linear_samples"]) if record["odom_linear_samples"] else None
        )
        record["odom_angular_mean"] = _safe_average(record["odom_angular_samples"])

        self._records.append(record)
        self._current_goal = None

    def _run_goal(self, scenario_name: str, goal_index: int, goal_spec: dict):
        robot_name = str(goal_spec.get("robot", "")).strip()
        record = {
            "scenario": scenario_name,
            "goal_index": goal_index,
            "label": str(goal_spec.get("label", f"{scenario_name}_{goal_index}")),
            "robot_name": robot_name,
            "goal_x": float(goal_spec.get("x", 0.0)),
            "goal_y": float(goal_spec.get("y", 0.0)),
            "goal_z": float(goal_spec.get("z", 0.0)),
            "goal_yaw": float(goal_spec.get("yaw", 0.0)),
            "frame_id": str(goal_spec.get("frame_id", "map")),
            "sent_time_sec": self._now_sec(),
            "accepted_time_sec": None,
            "first_plan_time_sec": None,
            "first_feedback_time_sec": None,
            "first_nonzero_cmd_time_sec": None,
            "first_nonzero_odom_time_sec": None,
            "result_time_sec": None,
            "travel_time_sec": None,
            "cmd_linear_samples": [],
            "cmd_angular_samples": [],
            "odom_linear_samples": [],
            "odom_angular_samples": [],
            "last_feedback_pose": None,
        }
        if robot_name not in self._action_clients:
            self._finalize_record(record, result=None, error_message="unknown_robot")
            self.get_logger().error(
                "Skipping benchmark goal for unknown robot '%s'" % robot_name
            )
            return

        client = self._action_clients[robot_name]
        if not client.wait_for_server(timeout_sec=self._server_timeout):
            self._finalize_record(
                record, result=None, error_message="action_server_not_ready"
            )
            self.get_logger().error(
                "Action server '/%s/%s' did not become ready"
                % (robot_name, self._action_name)
            )
            return

        self._current_goal = record

        goal_msg = self._goal_message(goal_spec)
        self.get_logger().info(
            "Benchmark goal %s -> %s (%.2f, %.2f)"
            % (record["label"], robot_name, record["goal_x"], record["goal_y"])
        )

        send_future = client.send_goal_async(
            goal_msg,
            feedback_callback=lambda feedback, robot=robot_name: self._feedback_callback(
                robot, feedback
            ),
        )
        if not self._spin_until(lambda: send_future.done(), timeout_sec=self._server_timeout):
            self._finalize_record(record, result=None, error_message="send_goal_timeout")
            return

        try:
            goal_handle = send_future.result()
        except Exception as exc:
            self._finalize_record(record, result=None, error_message=str(exc))
            return

        if not goal_handle.accepted:
            self._finalize_record(record, result=None, error_message="goal_rejected")
            return

        record["accepted_time_sec"] = self._now_sec()
        result_future = goal_handle.get_result_async()
        self._spin_until(lambda: result_future.done())

        try:
            result = result_future.result()
        except Exception as exc:
            self._finalize_record(record, result=None, error_message=str(exc))
            return

        self._finalize_record(record, result=result)

    def _write_outputs(self):
        summary = {
            "goal_file": self._goal_file,
            "records": self._records,
            "success_count": sum(1 for record in self._records if record["status"] == "SUCCEEDED"),
            "failure_count": sum(1 for record in self._records if record["status"] != "SUCCEEDED"),
        }
        if self._output_dir:
            write_json(f"{self._output_dir}/nav_benchmark_results.json", summary)
            for record in self._records:
                append_csv(
                    f"{self._output_dir}/nav_benchmark_results.csv",
                    [
                        "scenario",
                        "goal_index",
                        "label",
                        "robot_name",
                        "goal_x",
                        "goal_y",
                        "goal_yaw",
                        "status",
                        "error_code",
                        "error_msg",
                        "first_plan_latency_sec",
                        "first_feedback_latency_sec",
                        "first_nonzero_cmd_latency_sec",
                        "first_nonzero_odom_latency_sec",
                        "travel_time_sec",
                        "cmd_linear_mean",
                        "cmd_linear_max",
                        "odom_linear_mean",
                        "odom_linear_max",
                        "odom_angular_mean",
                    ],
                    record,
                )

    def run(self):
        scenario_names = self._selected_scenarios or list(self._scenarios.keys())
        for scenario_name in scenario_names:
            goals = self._scenarios.get(scenario_name, [])
            for goal_index, goal_spec in enumerate(goals, start=1):
                self._run_goal(scenario_name, goal_index, goal_spec)
                if self._inter_goal_delay > 0.0:
                    deadline = time.monotonic() + self._inter_goal_delay
                    while time.monotonic() < deadline:
                        rclpy.spin_once(self, timeout_sec=0.1)

        self._write_outputs()
        self.get_logger().info(
            "Benchmark finished: %d records" % len(self._records)
        )


def main(args=None):
    rclpy.init(args=args)
    node = NavBenchmarkRunner()
    try:
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
