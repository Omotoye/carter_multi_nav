import sys
import time

import rclpy
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class Nav2ServiceGate(Node):
    def __init__(self):
        super().__init__("nav2_service_gate")

        self.declare_parameter(
            "managed_nodes",
            [
                "controller_server",
                "smoother_server",
                "planner_server",
                "behavior_server",
                "bt_navigator",
                "waypoint_follower",
                "velocity_smoother",
            ],
        )
        self.declare_parameter("consecutive_service_successes", 3)
        self.declare_parameter("check_rate_hz", 2.0)
        self.declare_parameter("timeout", 45.0)

        self._managed_nodes = list(
            self.get_parameter("managed_nodes").get_parameter_value().string_array_value
        )
        self._required_successes = (
            self.get_parameter("consecutive_service_successes")
            .get_parameter_value()
            .integer_value
        )
        check_rate_hz = (
            self.get_parameter("check_rate_hz").get_parameter_value().double_value
        )
        self._timeout = self.get_parameter("timeout").get_parameter_value().double_value

        self._start_time = time.monotonic()
        self._last_wait_log = None
        self._service_successes = 0
        self.exit_code = 0

        self._service_clients = {}
        for node_name in self._managed_nodes:
            self._service_clients[node_name] = {
                "get_state": self.create_client(GetState, f"{node_name}/get_state"),
                "change_state": self.create_client(
                    ChangeState, f"{node_name}/change_state"
                ),
            }

        self.create_timer(1.0 / max(check_rate_hz, 1.0), self._check_services)

        self.get_logger().info(
            "Waiting for Nav2 lifecycle services for nodes: %s (timeout=%.1fs)"
            % (", ".join(self._managed_nodes), self._timeout)
        )

    def _check_services(self):
        now = time.monotonic()
        elapsed = now - self._start_time

        missing = []
        for node_name, clients in self._service_clients.items():
            if not clients["get_state"].service_is_ready():
                missing.append(f"{node_name}/get_state")
            if not clients["change_state"].service_is_ready():
                missing.append(f"{node_name}/change_state")

        if not missing:
            self._service_successes += 1
        else:
            self._service_successes = 0

        if self._service_successes >= self._required_successes:
            self.get_logger().info(
                "Nav2 lifecycle services are ready after %.1fs; starting lifecycle manager."
                % elapsed
            )
            self.exit_code = 0
            rclpy.shutdown()
            return

        if elapsed >= self._timeout:
            self.get_logger().error(
                "Timed out waiting for Nav2 lifecycle services after %.1fs; missing=%s"
                % (elapsed, ", ".join(missing) if missing else "none")
            )
            self.exit_code = 1
            rclpy.shutdown()
            return

        if self._last_wait_log is None or now - self._last_wait_log >= 2.0:
            wait_for = ", ".join(missing) if missing else "stable service checks"
            self.get_logger().info(
                "Nav2 lifecycle services pending after %.1fs; waiting for %s (%d/%d stable checks)"
                % (
                    elapsed,
                    wait_for,
                    self._service_successes,
                    self._required_successes,
                )
            )
            self._last_wait_log = now


def main(args=None):
    rclpy.init(args=args)
    node = Nav2ServiceGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        exit_code = getattr(node, "exit_code", 0)
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)
