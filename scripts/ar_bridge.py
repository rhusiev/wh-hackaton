#!/usr/bin/env python3
"""Serve the AR minimap to Spectacles over a WebSocket.

Detections arrive per frame and in the camera frame, which is the wrong shape
for a heads-up display: the operator needs a person to stay on the minimap after
the drone has flown past the aisle. So this node fuses detections into world
frame tracks that persist, and pushes a small JSON snapshot at a fixed rate.

Payload, all lengths in metres in the map frame:

    {"t": 1789594325.2,
     "drone": {"x": -13.4, "y": 0.1, "z": 2.0, "yaw": 0.02},
     "targets": [{"id": 0, "label": "person", "status": "confirmed", "x": -9.0, "y": 0.0, "z": 0.81, "h": 1.62,
                  "head": {"x": -9.0, "y": 0.02, "z": 1.5, "size": 0.22},
                  "score": 0.78, "confidence": 0.95, "age": 1.3, "hits": 12}],
     "map": {"res": 0.2, "w": 200, "h": 200, "x0": -20.0, "y0": -20.0,
             "cells": "<base64 of w*h bytes, 0 free / 1 occupied / 2 unknown>"},
     "view": "<base64 JPEG of the drone's colour image, only with --video>"}

A target is a box standing on the floor: centre x, y, z and height h. "head" is
present once the detector has seen a head for it. "status" is "confirmed", or
"lost" for a person no longer where they were last seen, "age" seconds ago. "map" is present only while
something publishes /map.

The fusing is tracker.py's NearestTracker; the tracker parameter ("module:Class" or
"path/to/file.py:Class") swaps it for anything with the same update and tracks.
Every track, candidates included, also goes to /people/tracks as the same JSON
list, for an explorer that wants a closer look.
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import json
import threading

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray

from geometry import quat_to_matrix, yaw
from occupancy import OCCUPIED, UNKNOWN, Grid
from plugin import load
from tracker import Sighting, Tracker
from view import CameraView

MAP_PERIOD = 1.0
# What the drone is looking at, for a preview that has no camera of its own. JPEG
# because the payload is JSON and a raw 640x400 frame is 750 KB of base64 per send.
VIEW_QUALITY = 60


class ArBridge(Node):
    def __init__(self, video: bool = False) -> None:
        super().__init__("ar_bridge")
        self.declare_parameters("", [
            ("map_frame", "map"),
            ("body_frame", "base_link"),
            ("tracker", "tracker:NearestTracker"),
            ("merge_radius", 1.0),
            # A lost person is forgotten after this long unseen.
            ("lost_timeout", 120.0),
            # About 1.5 s in view at 4 Hz; stray depth splits of a real person rarely reach it.
            ("min_hits", 6),
            # Nobody's head is higher than this above the floor.
            ("max_top", 2.3),
            # Closer than this, in frame and with nothing measured in front, it should be detected.
            ("miss_range", 8.0),
        ])
        self.map_frame, self.body_frame = self._p("map_frame"), self._p("body_frame")
        self.tracker: Tracker = load(self._p("tracker"))(
            merge_radius=self._p("merge_radius"), min_hits=self._p("min_hits"),
            max_top=self._p("max_top"), miss_range=self._p("miss_range"),
            lost_timeout=self._p("lost_timeout"))

        self.lock = threading.Lock()
        self.grid: dict | None = None
        self.occupancy: Grid | None = None
        self.info: CameraInfo | None = None
        self.view_jpeg: str | None = None
        self.depth: np.ndarray | None = None
        self.last_grid = 0.0
        self.last_tracks = 0.0

        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)
        self.create_subscription(
            Detection3DArray, "/oak/spatial_detections", self.on_detections, 10)
        # Volatile on purpose: it matches a latched publisher too, the reverse
        # does not, and rtabmap's durability differs between releases.
        self.create_subscription(OccupancyGrid, "/map", self.on_map, 1)
        self.create_subscription(CameraInfo, "/camera/color/camera_info",
                                 lambda msg: setattr(self, "info", msg), qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/depth/image_raw", self.on_depth,
                                 qos_profile_sensor_data)
        if video:
            self.create_subscription(Image, "/camera/color/image_raw", self.on_colour,
                                     qos_profile_sensor_data)
        self.tracks_out = self.create_publisher(String, "/people/tracks", 1)

    def _p(self, name: str):
        return self.get_parameter(name).value

    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_detections(self, msg: Detection3DArray) -> None:
        # Empty frames count too: they are what lowers the confidence of a false person.
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
        view = self.view(rot, trans)
        now = self.now()
        with self.lock:
            self.tracker.update(list(sightings.values()), now, view.visible if view else _never)
            tracks = self.tracker.tracks(now)
        if now - self.last_tracks >= MAP_PERIOD:
            self.last_tracks = now
            self.tracks_out.publish(String(data=json.dumps(tracks, separators=(",", ":"))))

    def view(self, rot: np.ndarray, trans: np.ndarray) -> CameraView | None:
        if self.info is None:
            return None
        k = self.info.k
        return CameraView(rot, trans, (k[0], k[4], k[2], k[5]), (self.info.width, self.info.height),
                          self.occupancy, self.depth)

    def on_colour(self, msg: Image) -> None:
        """The drone's own view, encoded once here rather than per connected client."""
        frame = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
        if msg.encoding == "rgb8":
            frame = frame[:, :, ::-1]
        ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, VIEW_QUALITY])
        if ok:
            self.view_jpeg = base64.b64encode(jpeg.tobytes()).decode()

    def on_depth(self, msg: Image) -> None:
        """The last depth frame, which says what is really in front of a track."""
        if msg.encoding == "32FC1":
            self.depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)

    def on_map(self, msg: OccupancyGrid) -> None:
        now = self.now()
        if now - self.last_grid < MAP_PERIOD:
            return
        self.last_grid = now
        self.occupancy = Grid.from_msg(msg)
        cells = self.occupancy.cells
        packed = np.where(cells == UNKNOWN, 2, np.where(cells >= OCCUPIED, 1, 0)).astype(np.uint8)
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
            payload["targets"] = [t for t in self.tracker.tracks(now) if t["status"] != "candidate"]
            if self.grid is not None:
                payload["map"] = self.grid
        if self.view_jpeg is not None:
            payload["view"] = self.view_jpeg
        return json.dumps(payload, separators=(",", ":"))


def _never(_xyz: np.ndarray, _max_range: float) -> bool:
    return False


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
    parser.add_argument("--video", action="store_true",
                        help="also send the drone's colour image, for ./run.sh preview")
    known, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = ArBridge(known.video)
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
