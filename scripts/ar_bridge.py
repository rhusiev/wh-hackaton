#!/usr/bin/env python3
"""Serve the AR minimap to Spectacles over a WebSocket.

Detections arrive per frame and in the camera frame, which is the wrong shape
for a heads-up display: the operator needs a person to stay on the minimap after
the drone has flown past the aisle. So this node fuses detections into world
frame tracks that persist, and pushes a small JSON snapshot at a fixed rate.

Payload, all lengths in metres in the map frame:

    {"t": 1789594325.2,
     "drone": {"x": -13.4, "y": 0.1, "z": 2.0, "yaw": 0.02},
     "targets": [{"id": 0, "label": "person", "x": -9.0, "y": 0.0, "z": 0.81, "h": 1.62,
                  "head": {"x": -9.0, "y": 0.02, "z": 1.5, "size": 0.22},
                  "score": 0.78, "age": 1.3, "hits": 12}],
     "map": {"res": 0.2, "w": 200, "h": 200, "x0": -20.0, "y0": -20.0,
             "cells": "<base64 of w*h bytes, 0 free / 1 occupied / 2 unknown>"}}

A target is a box standing on the floor: centre x, y, z and height h. "head" is
present once the detector has seen a head for it. "map" is present only while
something publishes /map.

The fusing is tracker.py's NearestTracker; the tracker parameter ("module:Class" or
"path/to/file.py:Class") swaps it for anything with the same update and targets.
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import json
import threading

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection3DArray

from geometry import quat_to_matrix, yaw
from plugin import load
from tracker import Sighting, Tracker

MAP_PERIOD = 1.0


class ArBridge(Node):
    def __init__(self) -> None:
        super().__init__("ar_bridge")
        self.declare_parameters("", [
            ("map_frame", "map"),
            ("body_frame", "base_link"),
            ("tracker", "tracker:NearestTracker"),
            ("merge_radius", 1.0),
            ("track_timeout", 0.0),
            # About 1.5 s in view at 4 Hz; stray depth splits of a real person rarely reach it.
            ("min_hits", 6),
            # Nobody's head is higher than this above the floor.
            ("max_top", 2.3),
        ])
        self.map_frame, self.body_frame = self._p("map_frame"), self._p("body_frame")
        self.tracker: Tracker = load(self._p("tracker"))(
            merge_radius=self._p("merge_radius"), min_hits=self._p("min_hits"),
            max_top=self._p("max_top"), timeout=self._p("track_timeout"))

        self.lock = threading.Lock()
        self.grid: dict | None = None
        self.last_grid = 0.0

        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)
        self.create_subscription(
            Detection3DArray, "/oak/spatial_detections", self.on_detections, 10)
        # Volatile on purpose: it matches a latched publisher too, the reverse
        # does not, and rtabmap's durability differs between releases.
        self.create_subscription(OccupancyGrid, "/map", self.on_map, 1)

    def _p(self, name: str):
        return self.get_parameter(name).value

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_detections(self, msg: Detection3DArray) -> None:
        if not msg.detections:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, msg.header.frame_id, rclpy.time.Time())
        except Exception as exc:
            self.get_logger().warn(f"no transform to {self.map_frame}: {exc}",
                                   throttle_duration_sec=5.0)
            return

        rot = quat_to_matrix(tf.transform.rotation)
        t = tf.transform.translation
        trans = np.array([t.x, t.y, t.z])
        # A head shares its detection id with the person it belongs to, within a frame.
        sightings: dict[str, Sighting] = {}
        heads: dict[str, np.ndarray] = {}
        for det in msg.detections:
            if not det.results:
                continue
            c = det.bbox.center.position
            xyz = rot @ np.array([c.x, c.y, c.z]) + trans
            hypothesis = det.results[0].hypothesis
            # Sizes are in the camera's optical axes, where y points down the image.
            if hypothesis.class_id == "head":
                heads[det.id] = np.append(xyz, det.bbox.size.y)
            else:
                sightings[det.id] = Sighting(hypothesis.class_id, xyz, det.bbox.size.y,
                                             hypothesis.score, None)
        for det_id, sighting in sightings.items():
            sighting.head = heads.get(det_id)
        with self.lock:
            self.tracker.update(list(sightings.values()), self.now())

    def on_map(self, msg: OccupancyGrid) -> None:
        now = self.now()
        if now - self.last_grid < MAP_PERIOD:
            return
        self.last_grid = now
        cells = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        packed = np.where(cells < 0, 2, np.where(cells >= 50, 1, 0)).astype(np.uint8)
        with self.lock:
            self.grid = {
                "res": round(msg.info.resolution, 4),
                "w": msg.info.width,
                "h": msg.info.height,
                "x0": round(msg.info.origin.position.x, 3),
                "y0": round(msg.info.origin.position.y, 3),
                "cells": base64.b64encode(packed.tobytes()).decode(),
            }

    def snapshot(self) -> str:
        now = self.now()
        payload = {"t": round(now, 2), "drone": None, "targets": []}
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.body_frame, rclpy.time.Time())
            t, q = tf.transform.translation, tf.transform.rotation
            payload["drone"] = {
                "x": round(t.x, 3), "y": round(t.y, 3), "z": round(t.z, 3),
                "yaw": round(yaw(q), 4),
            }
        except Exception:
            pass

        with self.lock:
            payload["targets"] = self.tracker.targets(now)
            if self.grid is not None:
                payload["map"] = self.grid
        return json.dumps(payload, separators=(",", ":"))


async def serve(node: ArBridge, host: str, port: int, rate: float) -> None:
    import websockets

    async def handler(socket):
        node.get_logger().info(f"AR client connected: {socket.remote_address}")
        try:
            while True:
                await socket.send(node.snapshot())
                await asyncio.sleep(1.0 / rate)
        except Exception:
            node.get_logger().info("AR client disconnected")

    async with websockets.serve(handler, host, port):
        node.get_logger().info(f"AR websocket on ws://{host}:{port}")
        await asyncio.Future()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8790)
    parser.add_argument("--rate", type=float, default=10.0)
    known, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = ArBridge()
    thread = threading.Thread(
        target=lambda: asyncio.run(serve(node, known.host, known.port, known.rate)),
        daemon=True)
    thread.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
