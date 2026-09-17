#!/usr/bin/env python3
"""Serve the AR minimap to Spectacles over a WebSocket.

Detections arrive per frame and in the camera frame, which is the wrong shape
for a heads-up display: the operator needs a person to stay on the minimap after
the drone has flown past the aisle. So this node fuses detections into world
frame tracks that persist, and pushes a small JSON snapshot at a fixed rate.

Payload, all lengths in metres in the map frame:

    {"t": 1789594325.2,
     "drone": {"x": -13.4, "y": 0.1, "z": 2.0, "yaw": 0.02},
     "targets": [{"id": 0, "label": "person", "x": -9.0, "y": 0.0,
                  "score": 0.78, "age": 1.3, "hits": 12}],
     "map": {"res": 0.4, "w": 80, "h": 50, "x0": -16.0, "y0": -10.0,
             "cells": "<base64 of w*h bytes, 0 free / 1 occupied / 2 unknown>"}}

"map" is present only while something publishes /map.
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import json
import math
import threading

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection3DArray

MAP_PERIOD = 1.0


class Track:
    __slots__ = ("id", "label", "xyz", "score", "hits", "last_seen")

    def __init__(self, track_id: int, label: str, xyz: np.ndarray, score: float, now: float):
        self.id, self.label = track_id, label
        self.xyz, self.score, self.hits, self.last_seen = xyz, score, 1, now

    def update(self, xyz: np.ndarray, score: float, now: float) -> None:
        # Running mean: repeated looks at the same person cancel the stereo noise.
        self.hits += 1
        self.xyz += (xyz - self.xyz) / min(self.hits, 20)
        self.score = max(self.score, score)
        self.last_seen = now


class ArBridge(Node):
    def __init__(self) -> None:
        super().__init__("ar_bridge")
        self.declare_parameters("", [
            ("map_frame", "map"),
            ("body_frame", "base_link"),
            ("merge_radius", 1.5),
            ("track_timeout", 0.0),
        ])
        self.map_frame, self.body_frame = self._p("map_frame"), self._p("body_frame")
        self.merge_radius = self._p("merge_radius")
        self.track_timeout = self._p("track_timeout")

        self.lock = threading.Lock()
        self.tracks: list[Track] = []
        self.next_id = 0
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

        q = tf.transform.rotation
        rot = _quat_to_matrix(q)
        trans = np.array([tf.transform.translation.x,
                          tf.transform.translation.y,
                          tf.transform.translation.z])
        now = self.now()
        with self.lock:
            for det in msg.detections:
                if not det.results:
                    continue
                c = det.bbox.center.position
                xyz = rot @ np.array([c.x, c.y, c.z]) + trans
                self.absorb(xyz, det.results[0].hypothesis, now)

    def absorb(self, xyz: np.ndarray, hypothesis, now: float) -> None:
        for track in self.tracks:
            if (track.label == hypothesis.class_id
                    and np.linalg.norm(track.xyz[:2] - xyz[:2]) < self.merge_radius):
                track.update(xyz, hypothesis.score, now)
                return
        self.tracks.append(Track(self.next_id, hypothesis.class_id, xyz,
                                 hypothesis.score, now))
        self.next_id += 1

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
                "yaw": round(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                        1 - 2 * (q.y * q.y + q.z * q.z)), 4),
            }
        except Exception:
            pass

        with self.lock:
            if self.track_timeout > 0:
                self.tracks = [t for t in self.tracks
                               if now - t.last_seen < self.track_timeout]
            payload["targets"] = [
                {"id": t.id, "label": t.label,
                 "x": round(float(t.xyz[0]), 3), "y": round(float(t.xyz[1]), 3),
                 "score": round(t.score, 3), "age": round(now - t.last_seen, 2),
                 "hits": t.hits}
                for t in self.tracks
            ]
            if self.grid is not None:
                payload["map"] = self.grid
        return json.dumps(payload, separators=(",", ":"))


def _quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


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
