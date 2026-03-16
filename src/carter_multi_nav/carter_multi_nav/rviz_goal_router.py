from copy import deepcopy

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from carter_multi_nav.common import DEFAULT_ROBOTS


_STATUS_NAMES = {
    0: "UNKNOWN",
    1: "ACCEPTED",
    2: "EXECUTING",
    3: "CANCELING",
    4: "SUCCEEDED",
    5: "CANCELED",
    6: "ABORTED",
}


class RvizGoalRouter(Node):
    def __init__(self):
        super().__init__("rviz_goal_router")

        self.declare_parameter("robot_names", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("input_topic", "/goal_pose")
        self.declare_parameter("action_name", "navigate_to_pose")
        self.declare_parameter("last_robot_topic", "/rviz_goal_router/last_robot")
        self.declare_parameter("next_robot_topic", "/rviz_goal_router/next_robot")
        self.declare_parameter(
            "forwarded_goal_topic", "/rviz_goal_router/forwarded_goal"
        )
        self.declare_parameter("start_index", 0)
        self.declare_parameter("dispatch_retry_period", 0.25)

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

        self._input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        ).strip()
        self._action_name = (
            self.get_parameter("action_name").get_parameter_value().string_value
        ).strip()
        last_robot_topic = (
            self.get_parameter("last_robot_topic").get_parameter_value().string_value
        ).strip()
        next_robot_topic = (
            self.get_parameter("next_robot_topic").get_parameter_value().string_value
        ).strip()
        forwarded_goal_topic = (
            self.get_parameter("forwarded_goal_topic").get_parameter_value().string_value
        ).strip()
        dispatch_retry_period = (
            self.get_parameter("dispatch_retry_period")
            .get_parameter_value()
            .double_value
        )
        self._next_index = (
            self.get_parameter("start_index").get_parameter_value().integer_value
            % len(self._robot_names)
        )
        self._goal_sequence = 0
        self._pending_futures = {}
        self._pending_dispatches = []

        debug_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._last_robot_publisher = self.create_publisher(
            String, last_robot_topic, debug_qos
        )
        self._next_robot_publisher = self.create_publisher(
            String, next_robot_topic, debug_qos
        )
        self._forwarded_goal_publisher = self.create_publisher(
            PoseStamped, forwarded_goal_topic, debug_qos
        )
        self._subscription = self.create_subscription(
            PoseStamped,
            self._input_topic,
            self._handle_goal,
            input_qos,
        )
        self._dispatch_timer = self.create_timer(
            max(dispatch_retry_period, 0.05), self._dispatch_pending_goals
        )

        self._action_clients = {
            robot_name: ActionClient(
                self, NavigateToPose, f"/{robot_name}/{self._action_name}"
            )
            for robot_name in self._robot_names
        }

        self._publish_next_robot()
        self.get_logger().info(
            "Routing RViz goals from '%s' across robots [%s] using action '%s'"
            % (
                self._input_topic,
                ", ".join(self._robot_names),
                self._action_name,
            )
        )

    def _publish_next_robot(self):
        next_robot = self._robot_names[self._next_index]
        self._next_robot_publisher.publish(String(data=next_robot))

    def _format_pose(self, msg: PoseStamped) -> str:
        return "frame=%s x=%.2f y=%.2f z=%.2f" % (
            msg.header.frame_id or "<empty>",
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        )

    def _handle_goal(self, msg: PoseStamped):
        robot_name = self._robot_names[self._next_index]
        action_path = f"/{robot_name}/{self._action_name}"

        goal = NavigateToPose.Goal()
        goal.pose = deepcopy(msg)

        self._goal_sequence += 1
        goal_id = self._goal_sequence
        self._last_robot_publisher.publish(String(data=robot_name))
        self._forwarded_goal_publisher.publish(goal.pose)
        self.get_logger().info(
            "Queued RViz goal #%d for '%s' (%s)"
            % (goal_id, action_path, self._format_pose(goal.pose))
        )

        self._next_index = (self._next_index + 1) % len(self._robot_names)
        self._publish_next_robot()
        self._pending_dispatches.append(
            {
                "goal": goal,
                "goal_id": goal_id,
                "robot_name": robot_name,
                "last_wait_log_ns": 0,
            }
        )
        self._dispatch_pending_goals()

    def _dispatch_pending_goals(self):
        if not self._pending_dispatches:
            return

        still_pending = []
        now_ns = self.get_clock().now().nanoseconds
        for pending in self._pending_dispatches:
            robot_name = pending["robot_name"]
            client = self._action_clients[robot_name]
            if not client.server_is_ready():
                if now_ns - pending["last_wait_log_ns"] >= 5_000_000_000:
                    self.get_logger().warn(
                        "Waiting for action server '/%s/%s' before dispatching RViz goal #%d."
                        % (robot_name, self._action_name, pending["goal_id"])
                    )
                    pending["last_wait_log_ns"] = now_ns
                still_pending.append(pending)
                continue

            future = client.send_goal_async(pending["goal"])
            self._pending_futures[future] = {
                "goal_id": pending["goal_id"],
                "robot_name": robot_name,
            }
            future.add_done_callback(self._handle_goal_response)
            self.get_logger().info(
                "Dispatching queued RViz goal #%d to '/%s/%s'."
                % (pending["goal_id"], robot_name, self._action_name)
            )

        self._pending_dispatches = still_pending

    def _handle_goal_response(self, future):
        context = self._pending_futures.pop(future, None)
        if context is None:
            return

        goal_id = context["goal_id"]
        robot_name = context["robot_name"]
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(
                "RViz goal #%d failed to send to '%s': %s"
                % (goal_id, robot_name, exc)
            )
            return

        if not goal_handle.accepted:
            self.get_logger().warn(
                "RViz goal #%d was rejected by '%s'." % (goal_id, robot_name)
            )
            return

        self.get_logger().info(
            "RViz goal #%d accepted by '%s'." % (goal_id, robot_name)
        )
        result_future = goal_handle.get_result_async()
        self._pending_futures[result_future] = {
            "goal_id": goal_id,
            "robot_name": robot_name,
        }
        result_future.add_done_callback(self._handle_result)

    def _handle_result(self, future):
        context = self._pending_futures.pop(future, None)
        if context is None:
            return

        goal_id = context["goal_id"]
        robot_name = context["robot_name"]
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(
                "RViz goal #%d for '%s' failed while waiting for the result: %s"
                % (goal_id, robot_name, exc)
            )
            return

        status = int(result.status)
        status_name = _STATUS_NAMES.get(status, str(status))
        error_code = int(result.result.error_code)
        error_msg = result.result.error_msg.strip()
        if error_msg:
            self.get_logger().info(
                "RViz goal #%d finished for '%s': status=%s error_code=%d error_msg='%s'"
                % (goal_id, robot_name, status_name, error_code, error_msg)
            )
        else:
            self.get_logger().info(
                "RViz goal #%d finished for '%s': status=%s error_code=%d"
                % (goal_id, robot_name, status_name, error_code)
            )


def main(args=None):
    rclpy.init(args=args)
    node = RvizGoalRouter()
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
