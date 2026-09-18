"""What an exploration strategy can ask of the aircraft, over MAVROS in GUIDED.

Waypoints are in the map frame, but MAVROS setpoints are in the EKF local frame
whose origin is wherever the vehicle armed. The offset between the two is
measured once after takeoff, by comparing the map -> base_link transform with
the local position. In the sim that transform is ground truth, on the aircraft
it comes from SLAM, so nothing here depends on which.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass

import rclpy
from copter import Copter
from geometry import yaw
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from occupancy import Grid
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener
from std_msgs.msg import String

SETPOINT_RATE = 10.0
ARRIVE_RADIUS = 0.6
LEG_TIMEOUT = 60.0
YAW_TOLERANCE = 0.15
SETTLE = 1.0  # lets the map catch up with a turn
AIRBORNE = 0.5  # m, above this the drone is flying rather than sitting on its skids


@dataclass
class Person:
    id: int
    status: str        # candidate, confirmed or lost, see tracker.py
    x: float
    y: float
    confidence: float
    age: float         # s since last seen


class Flight(Copter):
    def __init__(self, altitude: float, map_frame: str = "map",
                 body_frame: str = "base_link") -> None:
        super().__init__("explore")
        self.altitude = altitude
        self.map_frame, self.body_frame = map_frame, body_frame
        self.local: PoseStamped | None = None
        self.target: PoseStamped | None = None
        self.map: OccupancyGrid | None = None
        self._people: list[Person] = []
        self.offset = (0.0, 0.0)

        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)
        self.setpoint = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 lambda msg: setattr(self, "local", msg), qos_profile_sensor_data)
        self.create_subscription(
            OccupancyGrid, "/map", lambda msg: setattr(self, "map", msg),
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, "/people/tracks", self._on_tracks, 1)
        self.create_timer(1.0 / SETPOINT_RATE, self._resend)

    def _pose(self):
        return self.tf_buffer.lookup_transform(
            self.map_frame, self.body_frame, rclpy.time.Time()).transform

    def _has_pose(self) -> bool:
        return self.tf_buffer.can_transform(self.map_frame, self.body_frame, rclpy.time.Time())

    def here(self) -> tuple[float, float]:
        t = self._pose().translation
        return t.x, t.y

    def heading(self) -> float:
        return yaw(self._pose().rotation)

    def grid(self) -> Grid | None:
        if self.map is None:
            return None
        return Grid.from_msg(self.map)

    def _on_tracks(self, msg: String) -> None:
        self._people = [Person(t["id"], t["status"], t["x"], t["y"], t["confidence"], t["age"])
                        for t in json.loads(msg.data) if t["label"] == "person"]

    def people(self) -> list[Person]:
        """Every track from the tracker's latest report, refreshed once a second."""
        return self._people

    def _resend(self) -> None:
        """GUIDED drops a setpoint after ~3 s of silence, so keep resending it."""
        if self.target is not None:
            self.target.header.stamp = self.get_clock().now().to_msg()
            self.setpoint.publish(self.target)

    def take_off(self) -> None:
        self.require(lambda: self.local is not None and self._has_pose(),
                     f"local position and {self.map_frame} -> {self.body_frame}")
        # A run stopped in mid-air leaves the drone armed and hovering, and ArduPilot
        # refuses to take off from a height it is already at. Resuming instead of
        # taking off again lets the first leg of the mission fly it back to altitude.
        if self.state.armed and self.local.pose.position.z > AIRBORNE:
            self.get_logger().info(
                f"already flying at {self.local.pose.position.z:.1f} m, resuming")
            self.guided()
        else:
            self.arm_and_takeoff(self.altitude)
            self.require(lambda: self.local.pose.position.z > self.altitude - 0.3,
                         f"climb to {self.altitude} m", timeout=60.0)
        x, y = self.here()
        local = self.local.pose.position
        self.offset = local.x - x, local.y - y
        self.get_logger().info(f"map -> local offset {self.offset[0]:.2f} {self.offset[1]:.2f}")

    def fly_to(self, x: float, y: float, heading: float) -> bool:
        self.target = PoseStamped()
        self.target.header.frame_id = "map"
        self.target.pose.position.x = x + self.offset[0]
        self.target.pose.position.y = y + self.offset[1]
        self.target.pose.position.z = self.altitude
        self.target.pose.orientation.z = math.sin(heading / 2)
        self.target.pose.orientation.w = math.cos(heading / 2)
        goal = self.target.pose.position

        # 3D, so a drone lying on the floor under the waypoint has not arrived.
        return self.wait(
            lambda: math.dist(_xyz(self.local.pose.position), _xyz(goal)) < ARRIVE_RADIUS,
            LEG_TIMEOUT)

    def turn(self, heading: float) -> bool:
        """Turn in place. MAVROS's local orientation is empty here, so the map pose is checked."""
        self.fly_to(*self.here(), heading)
        turned = self.wait(lambda: abs(math.remainder(self.heading() - heading, math.tau))
                           < YAW_TOLERANCE, LEG_TIMEOUT)
        self.wait(lambda: False, SETTLE)
        return turned


def _xyz(p) -> tuple[float, float, float]:
    return p.x, p.y, p.z
