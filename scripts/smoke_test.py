#!/usr/bin/env python3
"""Check that a running simulation actually produces what the AR stack needs.

Start the sim first, then run this against it:

    ./run.sh sim gui:=false sitl:=true
    ./run.sh ./scripts/smoke_test.py

Every check is a fact about the contract the rest of the code depends on, so a
failure here is a real failure, not a flaky timing test. Exit status is 0 only
if everything passed.

Rates are measured against **sim time**, not the wall clock. Gazebo runs the
tricopter in lock-step with SITL at a 2 ms physics step, so on a busy machine the
whole sim drops below real time and every wall-clock rate drops with it. That is
a slow machine, not a broken topic. The real-time factor is reported on its own
line instead.

The detector publishes an empty array every cycle, so its rate check works on
the ground; the per-detection check only has something to look at once the drone
is high enough to see over a person's head, which is why it is skipped and not
failed when nothing is in view.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, LaserScan, PointCloud2
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection3DArray

# Must match models/tricopter/model.sdf.
EXPECTED_FX = 466.1
DEPTH_NEAR, DEPTH_FAR = 0.7, 12.0

GREEN, RED, DIM, RESET = "\033[32m", "\033[31m", "\033[2m", "\033[0m"


class Smoke(Node):
    def __init__(self) -> None:
        super().__init__("smoke_test")
        self.counts: dict[str, int] = {}
        self.last: dict[str, object] = {}
        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)
        self.sim_start: float | None = None
        self.sim_now = 0.0
        self.wall = 0.0
        self.create_subscription(Clock, "/clock", self._on_clock, qos_profile_sensor_data)

        for topic, msg_type in (
            ("/camera/color/image_raw", Image),
            ("/camera/depth/image_raw", Image),
            ("/camera/color/camera_info", CameraInfo),
            ("/camera/depth/points", PointCloud2),
            ("/ground_truth/odom", Odometry),
            ("/oak/spatial_detections", Detection3DArray),
            ("/scan", LaserScan),
            ("/mavros/obstacle/send", LaserScan),
            ("/mavros/state", State),
            ("/mavros/local_position/pose", PoseStamped),
        ):
            self.create_subscription(
                msg_type, topic, self._record(topic), qos_profile_sensor_data)

    def _on_clock(self, msg: Clock) -> None:
        self.sim_now = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.sim_start is None:
            self.sim_start = self.sim_now

    def _record(self, topic: str):
        def callback(msg) -> None:
            self.counts[topic] = self.counts.get(topic, 0) + 1
            self.last[topic] = msg
        return callback

    def collect(self, seconds: float) -> None:
        start = time.monotonic()
        while rclpy.ok() and time.monotonic() - start < seconds:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.wall = time.monotonic() - start

    @property
    def sim_window(self) -> float:
        return 0.0 if self.sim_start is None else self.sim_now - self.sim_start

    def rate(self, topic: str) -> float:
        return self.counts.get(topic, 0) / self.sim_window


class Report:
    def __init__(self) -> None:
        self.failures = 0

    def check(self, name: str, ok: bool, detail: str = "") -> bool:
        mark = f"{GREEN}PASS{RESET}" if ok else f"{RED}FAIL{RESET}"
        print(f"  [{mark}] {name}{DIM + '  ' + detail + RESET if detail else ''}")
        self.failures += not ok
        return ok


def check_websocket(port: int, report: Report) -> None:
    async def fetch() -> str:
        import websockets
        async with websockets.connect(f"ws://127.0.0.1:{port}", open_timeout=5) as ws:
            return await asyncio.wait_for(ws.recv(), timeout=5)

    try:
        payload = json.loads(asyncio.run(fetch()))
    except Exception as exc:
        report.check(f"AR websocket on {port}", False, str(exc))
        return
    report.check("AR websocket serves JSON", "t" in payload and "targets" in payload,
                 f"{len(payload.get('targets', []))} tracks")
    drone = payload.get("drone")
    report.check("AR payload carries the drone pose", isinstance(drone, dict),
                 f"x={drone['x']} y={drone['y']}" if drone else "missing")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--window", type=float, default=12.0)
    parser.add_argument("--ar-port", type=int, default=8790)
    parsed, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = Smoke()
    print(f"collecting for {parsed.window:.0f} s ...\n")
    node.collect(parsed.window)
    report = Report()
    if not report.check("sim clock is advancing", node.sim_window > 0.5,
                        f"{node.sim_window:.1f} s of sim time"):
        node.destroy_node()
        rclpy.shutdown()
        return 1
    rtf = node.sim_window / node.wall
    print(f"{DIM}  real-time factor {rtf:.2f}"
          f"{'' if rtf > 0.8 else ' - slow, but rates below are in sim time'}{RESET}\n")

    print("topics")
    for topic, low, high in (
        ("/camera/color/image_raw", 8.0, 20.0),
        ("/camera/depth/image_raw", 8.0, 20.0),
        ("/camera/color/camera_info", 8.0, 20.0),
        ("/camera/depth/points", 5.0, 20.0),
        ("/ground_truth/odom", 25.0, 60.0),
        ("/scan", 5.0, 20.0),
        ("/mavros/obstacle/send", 5.0, 20.0),
        ("/mavros/state", 0.5, 10.0),
    ):
        rate = node.rate(topic)
        report.check(topic, low <= rate <= high, f"{rate:.1f} Hz, want {low}-{high}")

    print("\ncamera")
    info = node.last.get("/camera/color/camera_info")
    if report.check("camera_info received", info is not None):
        fx = info.k[0]
        report.check("focal length matches the OAK-D FOV", abs(fx - EXPECTED_FX) < 5.0,
                     f"fx={fx:.1f}, want {EXPECTED_FX}")
        report.check("resolution is 640x400", (info.width, info.height) == (640, 400),
                     f"{info.width}x{info.height}")

    depth = node.last.get("/camera/depth/image_raw")
    if report.check("depth image received", depth is not None):
        values = np.frombuffer(depth.data, dtype=np.float32)
        finite = values[np.isfinite(values) & (values > 0)]
        report.check("depth encoding is 32FC1", depth.encoding == "32FC1", depth.encoding)
        report.check("depth respects the 0.7-12 m stereo clip",
                     finite.size > 0 and finite.min() >= DEPTH_NEAR - 0.01
                     and finite.max() <= DEPTH_FAR + 0.01,
                     f"{finite.min():.2f}-{finite.max():.2f} m" if finite.size else "empty")

    print("\nframes")
    for parent, child in (("map", "base_link"), ("base_link", "camera_link"),
                          ("camera_link", "camera_optical_frame")):
        try:
            node.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
            report.check(f"{parent} -> {child}", True)
        except Exception as exc:
            report.check(f"{parent} -> {child}", False, str(exc)[:60])

    if depth is not None:
        report.check("depth image is tagged with the optical frame",
                     depth.header.frame_id == "camera_optical_frame",
                     depth.header.frame_id)

    print("\nflight")
    state = node.last.get("/mavros/state")
    if report.check("mavros heard from the FCU", state is not None):
        report.check("FCU link is up", state.connected, f"mode={state.mode}")

    odom = node.last.get("/ground_truth/odom")
    if odom is not None:
        p = odom.pose.pose.position
        report.check("drone is inside the warehouse",
                     abs(p.x) < 16 and abs(p.y) < 10 and -1 < p.z < 8,
                     f"({p.x:.1f}, {p.y:.1f}, {p.z:.1f})")

    print("\nperception")
    detections = node.counts.get("/oak/spatial_detections", 0)
    report.check("detector is publishing", detections > 0,
                 f"{detections} messages in {node.sim_window:.0f} s of sim time")
    seen = node.last.get("/oak/spatial_detections")
    if seen is not None and seen.detections:
        first = seen.detections[0]
        c = first.bbox.center.position
        report.check("detection carries a label and a 3D position",
                     bool(first.results) and math.isfinite(c.z),
                     f"{first.id} at z={c.z:.1f} m")
    check_websocket(parsed.ar_port, report)

    node.destroy_node()
    rclpy.shutdown()
    print(f"\n{report.failures} failure(s)")
    return 1 if report.failures else 0


if __name__ == "__main__":
    sys.exit(main())
