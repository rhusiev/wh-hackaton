#!/usr/bin/env python3
"""Arm in GUIDED, climb to a target altitude and hold, over MAVROS.

    ./scripts/takeoff_hover.py --altitude 2.0
"""

from __future__ import annotations

import argparse

import rclpy
from copter import Copter, run


class TakeoffHover(Copter):
    def __init__(self, altitude: float) -> None:
        super().__init__("takeoff_hover")
        self.altitude = altitude

    def run(self) -> None:
        self.arm_and_takeoff(self.altitude)
        self.get_logger().info("takeoff accepted, holding")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--altitude", type=float, default=2.0)
    parsed, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    run(TakeoffHover(parsed.altitude))


if __name__ == "__main__":
    main()
