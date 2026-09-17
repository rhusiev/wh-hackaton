#!/usr/bin/env python3
"""Watch how close the drone gets to anything in the world, from ground truth.

    ./run.sh clearance            # during a flight, Ctrl-C prints the summary

Obstacles are the box collisions of the static models in worlds/warehouse.sdf,
plus a box around each person where the world puts them. The drone is a
cylinder around its props. The gap is measured between the two, so below 0 is
contact. Nothing here feeds back into the flight; it is a separate process
reading the 50 Hz ground-truth odometry, about 0.2 ms per message.
"""

from __future__ import annotations

import argparse
import json
import math
import signal
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

WORLDS = Path(__file__).resolve().parent.parent / "worlds"
DRONE_RADIUS = 0.40        # 0.27 m arm plus a 0.127 m prop
DRONE_HALF_HEIGHT = 0.15   # landing legs to prop tips
NEAR = 0.3                 # a pass closer than this is logged, once, at its closest


@dataclass
class Pass:
    gap: float
    name: str
    at: tuple[float, float, float]


class Boxes:
    """Oriented (yaw only) boxes as arrays, so one pose is checked against all at once."""

    def __init__(self, entries: list[tuple[str, tuple[float, ...], tuple[float, ...]]]) -> None:
        self.names = [name for name, _, _ in entries]
        pose = np.array([pose for _, pose, _ in entries])
        self.centre, self.yaw = pose[:, :3], pose[:, 3]
        self.half = np.array([size for _, _, size in entries]) / 2

    def gaps(self, x: float, y: float, z: float) -> np.ndarray:
        dx, dy = x - self.centre[:, 0], y - self.centre[:, 1]
        c, s = np.cos(self.yaw), np.sin(self.yaw)
        local = np.abs(np.stack([c * dx + s * dy, -s * dx + c * dy], axis=1))
        horizontal = np.linalg.norm(np.maximum(local - self.half[:, :2], 0), axis=1) - DRONE_RADIUS
        vertical = np.abs(z - self.centre[:, 2]) - self.half[:, 2] - DRONE_HALF_HEIGHT
        # Overlapping in one axis, the gap is the other; overlapping in neither, the corner.
        return np.where(vertical <= 0, horizontal,
                        np.where(horizontal <= 0, vertical, np.hypot(horizontal, vertical)))


def _pose(element: ET.Element | None) -> tuple[float, ...]:
    values = [float(v) for v in element.text.split()] if element is not None else [0.0] * 6
    return values[0], values[1], values[2], values[5]


def load_world(world: str) -> Boxes:
    root = ET.parse(WORLDS / f"{world}.sdf").getroot()
    entries = []
    for model in root.iter("model"):
        mx, my, mz, myaw = _pose(model.find("pose"))
        for collision in model.iter("collision"):
            box = collision.find("geometry/box/size")
            if box is None:
                continue
            x, y, z, yaw = _pose(collision.find("pose"))
            c, s = math.cos(myaw), math.sin(myaw)
            entries.append((f"{model.get('name')}/{collision.get('name')}",
                            (mx + c * x - s * y, my + s * x + c * y, mz + z, myaw + yaw),
                            tuple(float(v) for v in box.text.split())))
    yaws = {include.findtext("name"): _pose(include.find("pose"))[3]
            for include in root.iter("include")}
    for person in json.loads((WORLDS / f"{world}_targets.json").read_text())["targets"]:
        entries.append((person["name"], (*person["xyz"], yaws.get(person["name"], 0.0)),
                        tuple(person["size"])))
    return Boxes(entries)


class Clearance(Node):
    def __init__(self, boxes: Boxes) -> None:
        super().__init__("clearance")
        self.boxes = boxes
        self.closest = Pass(math.inf, "", (0.0, 0.0, 0.0))
        self.passing: Pass | None = None  # the closest point of the pass under way
        self.contacts = 0
        self.create_subscription(Odometry, "/ground_truth/odom", self.on_odom,
                                 qos_profile_sensor_data)

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        if p.z < 0.5:  # on the ground, before takeoff or after landing
            return
        gaps = self.boxes.gaps(p.x, p.y, p.z)
        i = int(gaps.argmin())
        sample = Pass(float(gaps[i]), self.boxes.names[i], (p.x, p.y, p.z))
        if sample.gap < self.closest.gap:
            self.closest = sample
        if sample.gap >= NEAR:
            if self.passing is not None:
                self.report(self.passing)
                self.passing = None
            return
        if self.passing is None or sample.gap < self.passing.gap:
            self.passing = sample

    def report(self, closest: Pass) -> None:
        x, y, z = closest.at
        text = f"{closest.gap:.2f} m from {closest.name} at ({x:.1f}, {y:.1f}, {z:.1f})"
        if closest.gap < 0:
            self.contacts += 1
            self.get_logger().error(f"contact: {text}")
        else:
            self.get_logger().warn(f"close: {text}")

    def summary(self) -> str:
        if self.passing is not None:
            self.report(self.passing)
        if math.isinf(self.closest.gap):
            return "never airborne"
        x, y, z = self.closest.at
        return (f"closest {self.closest.gap:.2f} m from {self.closest.name} at "
                f"({x:.1f}, {y:.1f}, {z:.1f}), {self.contacts} contact(s)")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--world", default="warehouse")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = Clearance(load_world(args.world))
    node.get_logger().info(f"{len(node.boxes.names)} obstacles")
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    print(node.summary(), flush=True)


if __name__ == "__main__":
    main()
