"""Shared MAVROS plumbing for the mission scripts."""

from __future__ import annotations

import rclpy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

# mavros publishes state BEST_EFFORT; a RELIABLE subscription never matches it.
STATE_QOS = QoSProfile(
    depth=10,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)

ARM_TIMEOUT = 120.0


class Copter(Node):
    """A node that can talk the arm/mode/takeoff sequence to ArduPilot."""

    def __init__(self, name: str) -> None:
        super().__init__(name)
        self.state = State()
        self.create_subscription(State, "/mavros/state", self._on_state, STATE_QOS)
        self.set_mode = self.create_client(SetMode, "/mavros/set_mode")
        self.arming = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.takeoff = self.create_client(CommandTOL, "/mavros/cmd/takeoff")

    def _on_state(self, msg: State) -> None:
        self.state = msg

    def call(self, client, request):
        while not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(f"waiting for {client.srv_name}")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        return future.result()

    def wait(self, predicate, timeout: float) -> bool:
        deadline = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        while rclpy.ok() and not predicate():
            if self.get_clock().now().nanoseconds > deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.2)
        return predicate()

    def require(self, predicate, what: str, timeout: float = 60.0) -> None:
        if not self.wait(predicate, timeout):
            raise TimeoutError(f"timed out waiting for {what}")

    def guided(self) -> None:
        self.require(lambda: self.state.connected, "MAVROS to connect to the FCU")
        self.get_logger().info("switching to GUIDED")
        self.call(self.set_mode, SetMode.Request(custom_mode="GUIDED"))
        self.require(lambda: self.state.mode == "GUIDED", "GUIDED mode")

    def arm_and_takeoff(self, altitude: float) -> None:
        self.guided()

        # Arming is refused until the EKF settles, so keep asking.
        self.get_logger().info("arming")
        deadline = self.get_clock().now().nanoseconds + int(ARM_TIMEOUT * 1e9)
        while not self.state.armed:
            if self.get_clock().now().nanoseconds > deadline:
                raise TimeoutError("timed out waiting for arming")
            self.call(self.arming, CommandBool.Request(value=True))
            self.wait(lambda: self.state.armed, timeout=2.0)

        self.get_logger().info(f"taking off to {altitude} m")
        result = self.call(self.takeoff, CommandTOL.Request(altitude=altitude))
        if not result.success:
            raise RuntimeError(f"takeoff rejected, MAV_RESULT={result.result}")


def run(node: Copter, spin_after: bool = True) -> None:
    try:
        node.run()
        if spin_after:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
