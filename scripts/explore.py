#!/usr/bin/env python3
"""Explore the warehouse in GUIDED, over MAVROS.

    ./scripts/explore.py --altitude 2.5   # lawnmower sweep of the known aisles
    ./scripts/explore.py --frontier       # no prior layout, fly to the unknown in /map

--frontier needs /map, from the grid mapper or RTAB-Map. It looks around once,
then repeatedly flies a short leg toward the nearest reachable frontier and
turns to look into it, until no reachable frontier is left.

Waypoints are given in the Gazebo map frame, but MAVROS setpoints are in the
EKF local frame whose origin is wherever the vehicle armed. The offset between
the two is measured once at the start by comparing the ground-truth odometry
with the local position, so the pattern below does not depend on the spawn.
"""

from __future__ import annotations

import argparse
import math

import numpy as np
import rclpy
from copter import Copter, run
from frontier import Grid, plan
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data

# Rack rows sit at y = +/-7.5 and +/-2.5 and are 1.1 m deep, so these are the
# five clear lanes. x stops short of the 32 m hall's end walls.
LANES = (-9.3, -5.0, 0.0, 5.0, 9.3)
X_SPAN = (-13.0, 13.0)

SETPOINT_RATE = 10.0
ARRIVE_RADIUS = 0.6
LEG_TIMEOUT = 60.0
YAW_TOLERANCE = 0.15

# Frontier mode. Legs are short so each plan uses the map the last leg revealed.
FRONTIER_LEG = 3.0
CLEARANCE = 0.8          # arm tip is 0.4 m from the centre, the rest is position error
MIN_FRONTIER_CELLS = 4
VISITED_RADIUS = 1.5     # a frontier still unknown after looking at it is given up
SETTLE = 1.0             # lets the map catch up with a turn


def lawnmower() -> list[tuple[float, float]]:
    points = []
    for i, y in enumerate(LANES):
        x0, x1 = X_SPAN if i % 2 == 0 else X_SPAN[::-1]
        points += [(x0, y), (x1, y)]
    return points



def _xyz(p) -> tuple[float, float, float]:
    return p.x, p.y, p.z


def _yaw(q) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Explore(Copter):
    def __init__(self, altitude: float, lanes: int, frontier: bool, max_time: float) -> None:
        super().__init__("explore")
        self.altitude = altitude
        self.waypoints = lawnmower()[: 2 * lanes]
        self.frontier, self.max_time = frontier, max_time
        self.local: PoseStamped | None = None
        self.truth: Odometry | None = None
        self.target: PoseStamped | None = None
        self.grid: OccupancyGrid | None = None

        self.setpoint = self.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10)
        self.create_subscription(PoseStamped, "/mavros/local_position/pose",
                                 self._on_local, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/ground_truth/odom",
                                 self._on_truth, qos_profile_sensor_data)
        self.create_timer(1.0 / SETPOINT_RATE, self._resend)
        if frontier:
            self.create_subscription(
                OccupancyGrid, "/map", lambda msg: setattr(self, "grid", msg),
                QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))

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

    def here(self) -> tuple[float, float]:
        p = self.truth.pose.pose.position
        return p.x, p.y

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

    def turn(self, yaw: float, offset) -> bool:
        x, y = self.here()
        self.fly_to(x, y, yaw, offset)
        turned = self.wait(lambda: abs(math.remainder(
            _yaw(self.truth.pose.pose.orientation) - yaw, math.tau)) < YAW_TOLERANCE, LEG_TIMEOUT)
        self.wait(lambda: False, SETTLE)
        return turned

    def sweep(self, offset) -> None:
        previous = None
        for x, y in self.waypoints:
            yaw = 0.0 if previous is None else math.atan2(y - previous[1], x - previous[0])
            self.get_logger().info(f"leg to map ({x:.1f}, {y:.1f})")
            if not self.fly_to(x, y, yaw, offset):
                self.get_logger().error("leg timed out, stopping the sweep")
                return
            previous = (x, y)
        self.get_logger().info("sweep complete, holding at the last waypoint")

    def explore_frontiers(self, offset) -> None:
        self.require(lambda: self.grid is not None, "/map")
        for _ in range(4):
            self.turn(math.remainder(_yaw(self.truth.pose.pose.orientation) + math.pi / 2, math.tau),
                      offset)

        deadline = self.get_clock().now().nanoseconds + int(self.max_time * 1e9)
        visited: list[tuple[float, float]] = []
        while self.get_clock().now().nanoseconds < deadline:
            info = self.grid.info
            grid = Grid(np.asarray(self.grid.data, dtype=np.int8).reshape(info.height, info.width),
                        info.resolution, (info.origin.position.x, info.origin.position.y))
            goal = plan(grid, self.here(), CLEARANCE, MIN_FRONTIER_CELLS, visited, VISITED_RADIUS)
            if goal is None:
                self.get_logger().info("no reachable frontier left, holding")
                return

            here = self.here()
            if goal.path:
                x, y = goal.path[0]
                distance = math.dist(here, (x, y))
                if distance > FRONTIER_LEG:
                    x, y = (here[0] + (x - here[0]) * FRONTIER_LEG / distance,
                            here[1] + (y - here[1]) * FRONTIER_LEG / distance)
                final = len(goal.path) == 1 and distance <= FRONTIER_LEG
                self.get_logger().info(f"leg to map ({x:.1f}, {y:.1f}), frontier at "
                                       f"({goal.path[-1][0]:.1f}, {goal.path[-1][1]:.1f})")
                # Face the leg first: the obstacle scan only covers what the camera sees.
                yaw = math.atan2(y - here[1], x - here[0])
                self.turn(yaw, offset)
                if not self.fly_to(x, y, yaw, offset):
                    self.get_logger().warn("leg timed out, giving up on that frontier")
                    visited.append(goal.path[-1])
                    continue
                if not final:
                    continue
            self.turn(goal.look_yaw, offset)
            visited.append(self.here())
        self.get_logger().info(f"stopped after {self.max_time:.0f} s, holding")

    def run(self) -> None:
        self.require(lambda: self.local is not None and self.truth is not None,
                     "local position and ground truth")
        self.arm_and_takeoff(self.altitude)
        self.require(lambda: self.local.pose.position.z > self.altitude - 0.3,
                     f"climb to {self.altitude} m", timeout=60.0)

        offset = self.map_to_local()
        self.get_logger().info(f"map -> local offset {offset[0]:.2f} {offset[1]:.2f}")

        if self.frontier:
            self.explore_frontiers(offset)
        else:
            self.sweep(offset)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--altitude", type=float, default=2.5)
    parser.add_argument("--lanes", type=int, default=len(LANES),
                        help="how many of the five lanes to sweep")
    parser.add_argument("--frontier", action="store_true",
                        help="explore the unknown in /map instead of sweeping the aisles")
    parser.add_argument("--max-time", type=float, default=600.0,
                        help="frontier mode gives up after this many seconds")
    parsed, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    run(Explore(parsed.altitude, parsed.lanes, parsed.frontier, parsed.max_time))


if __name__ == "__main__":
    main()
