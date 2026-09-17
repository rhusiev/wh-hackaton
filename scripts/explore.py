#!/usr/bin/env python3
"""Fly a lawnmower sweep of the warehouse aisles in GUIDED, over MAVROS.

    ./scripts/explore.py --altitude 2.5

Waypoints are given in the Gazebo map frame, but MAVROS setpoints are in the
EKF local frame whose origin is wherever the vehicle armed. The offset between
the two is measured once at the start by comparing the ground-truth odometry
with the local position, so the pattern below does not depend on the spawn.
"""

from __future__ import annotations

import argparse
import math

import rclpy
from copter import Copter, run
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

# Rack rows sit at y = +/-7.5 and +/-2.5 and are 1.1 m deep, so these are the
# five clear lanes. x stops short of the 32 m hall's end walls.
LANES = (-9.3, -5.0, 0.0, 5.0, 9.3)
X_SPAN = (-13.0, 13.0)

SETPOINT_RATE = 10.0
ARRIVE_RADIUS = 0.6
LEG_TIMEOUT = 60.0


def lawnmower() -> list[tuple[float, float]]:
    points = []
    for i, y in enumerate(LANES):
        x0, x1 = X_SPAN if i % 2 == 0 else X_SPAN[::-1]
        points += [(x0, y), (x1, y)]
    return points



def _xyz(p) -> tuple[float, float, float]:
    return p.x, p.y, p.z

class Explore(Copter):
    def __init__(self, altitude: float, lanes: int) -> None:
        super().__init__("explore")
        self.altitude = altitude
        self.waypoints = lawnmower()[: 2 * lanes]
        self.local: PoseStamped | None = None
        self.truth: Odometry | None = None
        self.target: PoseStamped | None = None

        self.setpoint = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._on_local, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/ground_truth/odom",
                                 self._on_truth, qos_profile_sensor_data)
        self.create_timer(1.0 / SETPOINT_RATE, self._resend)

    def _on_local(self, msg: PoseStamped) -> None:
        self.local = msg

    def _on_truth(self, msg: Odometry) -> None:
        self.truth = msg

    def map_to_local(self) -> tuple[float, float]:
        truth = self.truth.pose.pose.position
        local = self.local.pose.position
        return local.x - truth.x, local.y - truth.y

    def _resend(self) -> None:
        """GUIDED drops a setpoint after ~3 s of silence, so keep resending it."""
        if self.target is not None:
            self.target.header.stamp = self.get_clock().now().to_msg()
            self.setpoint.publish(self.target)

    def fly_to(self, x: float, y: float, yaw: float, offset) -> bool:
        self.target = PoseStamped()
        self.target.header.frame_id = "map"
        self.target.pose.position.x = x + offset[0]
        self.target.pose.position.y = y + offset[1]
        self.target.pose.position.z = self.altitude
        self.target.pose.orientation.z = math.sin(yaw / 2)
        self.target.pose.orientation.w = math.cos(yaw / 2)
        goal = self.target.pose.position

        # 3D, so a drone lying on the floor under the waypoint has not arrived.
        return self.wait(
            lambda: math.dist(_xyz(self.local.pose.position), _xyz(goal)) < ARRIVE_RADIUS,
            LEG_TIMEOUT)

    def run(self) -> None:
        self.require(lambda: self.local is not None and self.truth is not None,
                     "local position and ground truth")
        self.arm_and_takeoff(self.altitude)
        self.require(lambda: self.local.pose.position.z > self.altitude - 0.3,
                     f"climb to {self.altitude} m", timeout=60.0)

        offset = self.map_to_local()
        self.get_logger().info(f"map -> local offset {offset[0]:.2f} {offset[1]:.2f}")

        previous = None
        for x, y in self.waypoints:
            yaw = 0.0 if previous is None else math.atan2(y - previous[1], x - previous[0])
            self.get_logger().info(f"leg to map ({x:.1f}, {y:.1f})")
            if not self.fly_to(x, y, yaw, offset):
                self.get_logger().error("leg timed out, stopping the sweep")
                return
            previous = (x, y)
        self.get_logger().info("sweep complete, holding at the last waypoint")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--altitude", type=float, default=2.5)
    parser.add_argument("--lanes", type=int, default=len(LANES),
                        help="how many of the five lanes to sweep")
    parsed, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    run(Explore(parsed.altitude, parsed.lanes))


if __name__ == "__main__":
    main()
