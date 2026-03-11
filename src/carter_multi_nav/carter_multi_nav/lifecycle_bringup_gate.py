import sys
import time

import rclpy
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class LifecycleBringupGate(Node):
    def __init__(self):
        super().__init__("lifecycle_bringup_gate")

        self.declare_parameter("managed_nodes", [""])
        self.declare_parameter("timeout", 30.0)

        self._managed_nodes = [
            node_name
            for node_name in self.get_parameter("managed_nodes")
            .get_parameter_value()
            .string_array_value
            if node_name
        ]
        self._timeout = self.get_parameter("timeout").get_parameter_value().double_value
        self._start_time = time.monotonic()
        self._last_wait_log = None
        self.exit_code = 0

        self._service_clients = {}
        for node_name in self._managed_nodes:
            self._service_clients[node_name] = {
                "get_state": self.create_client(GetState, f"{node_name}/get_state"),
                "change_state": self.create_client(
                    ChangeState, f"{node_name}/change_state"
                ),
            }

        self.get_logger().info(
            "Waiting to bring up lifecycle nodes: %s (timeout=%.1fs)"
            % (", ".join(self._managed_nodes), self._timeout)
        )

    def _spin_until(self, predicate):
        while rclpy.ok() and not predicate():
            if time.monotonic() - self._start_time >= self._timeout:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return predicate()

    def _wait_for_services(self):
        def services_ready():
            return all(
                clients["get_state"].service_is_ready()
                and clients["change_state"].service_is_ready()
                for clients in self._service_clients.values()
            )

        while rclpy.ok():
            if services_ready():
                return True

            elapsed = time.monotonic() - self._start_time
            if elapsed >= self._timeout:
                return False

            if self._last_wait_log is None or time.monotonic() - self._last_wait_log >= 2.0:
                missing = []
                for node_name, clients in self._service_clients.items():
                    if not clients["get_state"].service_is_ready():
                        missing.append(f"{node_name}/get_state")
                    if not clients["change_state"].service_is_ready():
                        missing.append(f"{node_name}/change_state")
                self.get_logger().info(
                    "Lifecycle bringup pending after %.1fs; waiting for %s"
                    % (elapsed, ", ".join(missing) if missing else "services")
                )
                self._last_wait_log = time.monotonic()

            rclpy.spin_once(self, timeout_sec=0.1)

        return False

    def _get_state(self, node_name):
        future = self._service_clients[node_name]["get_state"].call_async(
            GetState.Request()
        )
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None:
            raise RuntimeError(f"Failed to get lifecycle state for {node_name}")
        return future.result().current_state.label

    def _change_state(self, node_name, transition_id, label):
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = self._service_clients[node_name]["change_state"].call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done() or future.result() is None:
            raise RuntimeError(f"Timed out during {label} for {node_name}")
        if not future.result().success:
            raise RuntimeError(f"Lifecycle transition {label} failed for {node_name}")

    def bringup(self):
        if not self._managed_nodes:
            self.get_logger().error("No managed lifecycle nodes were provided.")
            return 1

        if not self._wait_for_services():
            self.get_logger().error(
                "Timed out waiting for lifecycle services for: %s"
                % ", ".join(self._managed_nodes)
            )
            return 1

        for node_name in self._managed_nodes:
            state = self._get_state(node_name)
            self.get_logger().info(f"{node_name} initial state: {state}")

            if state == "active":
                continue
            if state == "unconfigured":
                self._change_state(
                    node_name, Transition.TRANSITION_CONFIGURE, "configure"
                )
                state = self._get_state(node_name)
                self.get_logger().info(f"{node_name} state after configure: {state}")

            if state == "inactive":
                self._change_state(
                    node_name, Transition.TRANSITION_ACTIVATE, "activate"
                )
                state = self._get_state(node_name)
                self.get_logger().info(f"{node_name} state after activate: {state}")

            if state != "active":
                self.get_logger().error(
                    f"{node_name} did not reach active state; current state: {state}"
                )
                return 1

        self.get_logger().info(
            "Lifecycle bringup succeeded for: %s" % ", ".join(self._managed_nodes)
        )
        return 0


def main(args=None):
    rclpy.init(args=args)
    node = LifecycleBringupGate()
    try:
        node.exit_code = node.bringup()
    except (KeyboardInterrupt, ExternalShutdownException):
        node.exit_code = 1
    except Exception as exc:
        node.get_logger().error(str(exc))
        node.exit_code = 1
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(node.exit_code)
