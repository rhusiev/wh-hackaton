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
    __slots__ = ("id", "label", "xyz", "height", "head", "score", "hits", "last_seen")

    def __init__(self, track_id: int, label: str, xyz: np.ndarray, height: float, score: float,
                 now: float):
        self.id, self.label = track_id, label
        self.xyz, self.height, self.head = xyz, height, None
        self.score, self.hits, self.last_seen = score, 1, now

    def update(self, xyz: np.ndarray, height: float, score: float, now: float) -> None:
        # Running mean: repeated looks at the same person cancel the stereo noise.
        self.hits += 1
        self.xyz += (xyz - self.xyz) / self.weight()
        self.height += (height - self.height) / self.weight()
        self.score = max(self.score, score)
        self.last_seen = now

    def update_head(self, head: np.ndarray) -> None:
        """head is x, y, z and size, from the same frame as the latest update."""
        self.head = head if self.head is None else self.head + (head - self.head) / self.weight()

    def weight(self) -> int:
        return min(self.hits, 20)


class ArBridge(Node):
    def __init__(self) -> None:
        super().__init__("ar_bridge")
        self.declare_parameters("", [
            ("map_frame", "map"),
            ("body_frame", "base_link"),
            ("merge_radius", 1.0),
            ("track_timeout", 0.0),
            # About 1.5 s in view at 4 Hz; stray depth splits of a real person rarely reach it.
            ("min_hits", 6),
            # Nobody's head is higher than this above the floor.
            ("max_top", 2.3),
        ])
        self.map_frame, self.body_frame = self._p("map_frame"), self._p("body_frame")
        self.merge_radius = self._p("merge_radius")
        self.track_timeout = self._p("track_timeout")
        self.min_hits, self.max_top = self._p("min_hits"), self._p("max_top")

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
            # A head shares its detection id with the person it belongs to, within a frame.
            people, heads = {}, []
            for det in msg.detections:
                if not det.results:
                    continue
                c = det.bbox.center.position
                xyz = rot @ np.array([c.x, c.y, c.z]) + trans
                hypothesis = det.results[0].hypothesis
                # Sizes are in the camera's optical axes, where y points down the image.
                if hypothesis.class_id == "head":
                    heads.append((det.id, np.append(xyz, det.bbox.size.y)))
                elif xyz[2] + det.bbox.size.y / 2 <= self.max_top:
                    people[det.id] = self.absorb(xyz, det.bbox.size.y, hypothesis, now)
            for det_id, head in heads:
                if det_id in people:
                    people[det_id].update_head(head)

    def absorb(self, xyz: np.ndarray, height: float, hypothesis, now: float) -> Track:
        # The nearest track, not the first in range: two people 1.5 m apart must not share one.
        near = [(np.linalg.norm(t.xyz[:2] - xyz[:2]), t) for t in self.tracks
                if t.label == hypothesis.class_id]
        distance, track = min(near, key=lambda pair: pair[0], default=(math.inf, None))
        if distance < self.merge_radius:
            track.update(xyz, height, hypothesis.score, now)
            return track
        track = Track(self.next_id, hypothesis.class_id, xyz, height, hypothesis.score, now)
        self.tracks.append(track)
        self.next_id += 1
        return track

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
            payload["targets"] = [_target(t, now) for t in self.tracks
                                  if t.hits >= self.min_hits]
            if self.grid is not None:
                payload["map"] = self.grid
        return json.dumps(payload, separators=(",", ":"))


def _target(track: Track, now: float) -> dict:
    x, y, z = (round(float(c), 3) for c in track.xyz)
    target = {"id": track.id, "label": track.label, "x": x, "y": y, "z": z,
              "h": round(track.height, 3), "score": round(track.score, 3),
              "age": round(now - track.last_seen, 2), "hits": track.hits}
    if track.head is not None:
        target["head"] = dict(zip(("x", "y", "z", "size"),
                                  (round(float(c), 3) for c in track.head)))
    return target


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
