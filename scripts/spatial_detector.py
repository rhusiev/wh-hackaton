#!/usr/bin/env python3
"""Stand-in for the OAK-D's on-device spatial detection network.

The real camera runs the detector on its own Myriad X and reports each hit as a
label plus an XYZ in the camera frame. Here the target positions are already
known from the world generator, so the only question left is what the camera can
actually see: inside the frustum, inside the stereo range, big enough in pixels
to fire a detector, and not hidden behind a rack. Occlusion is tested against
the measured depth image, so it fails in the same places the real camera does.

Replace this node with depthai_ros_driver on the real aircraft and the topic
contract below does not change.
"""

from __future__ import annotations

import argparse
import json
import random
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Vector3
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import (
    BoundingBox3D,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)
from visualization_msgs.msg import Marker, MarkerArray

from stereo import DEPTH_MAX, DEPTH_MIN, DEPTH_RELIABLE, sigma


def quat_to_matrix(q) -> np.ndarray:
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class SpatialDetector(Node):
    def __init__(self, targets: list[dict]) -> None:
        super().__init__("spatial_detector")
        self.declare_parameters("", [
            ("map_frame", "map"),
            ("camera_frame", "camera_optical_frame"),
            ("min_range", DEPTH_MIN),
            ("max_range", DEPTH_MAX),
            ("min_pixel_width", 24.0),
            ("occlusion_tolerance", 0.6),
            ("rate", 10.0),
            ("seed", 1),
        ])
        self.map_frame = self._p("map_frame")
        self.camera_frame = self._p("camera_frame")
        self.min_range, self.max_range = self._p("min_range"), self._p("max_range")
        self.min_pixel_width = self._p("min_pixel_width")
        self.occlusion_tolerance = self._p("occlusion_tolerance")
        self.period = 1.0 / self._p("rate")

        self.targets = targets
        self.rng = random.Random(self._p("seed"))
        self.info: CameraInfo | None = None
        self.last_stamp = 0.0

        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)

        self.detections = self.create_publisher(
            Detection3DArray, "/oak/spatial_detections", 10)
        self.markers = self.create_publisher(MarkerArray, "/oak/detection_markers", 10)
        self.create_subscription(
            CameraInfo, "/camera/depth/camera_info", self.on_info, qos_profile_sensor_data)
        self.create_subscription(
            Image, "/camera/depth/image_raw", self.on_depth, qos_profile_sensor_data)

        self.get_logger().info(f"tracking {len(targets)} targets")

    def _p(self, name: str):
        return self.get_parameter(name).value

    def on_info(self, msg: CameraInfo) -> None:
        self.info = msg

    def on_depth(self, msg: Image) -> None:
        if self.info is None or msg.encoding != "32FC1":
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp - self.last_stamp < self.period:
            return
        self.last_stamp = stamp

        try:
            tf = self.tf_buffer.lookup_transform(
                self.camera_frame, self.map_frame, rclpy.time.Time())
        except Exception as exc:
            self.get_logger().warn(f"no {self.map_frame} -> {self.camera_frame}: {exc}",
                                   throttle_duration_sec=5.0)
            return

        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        rot = quat_to_matrix(tf.transform.rotation)
        trans = np.array([tf.transform.translation.x,
                          tf.transform.translation.y,
                          tf.transform.translation.z])

        out = Detection3DArray()
        out.header = msg.header
        visible = []
        for target in self.targets:
            hit = self.evaluate(target, rot, trans, depth, msg)
            if hit is not None:
                out.detections.append(hit[0])
                visible.append((target, hit[1]))
        self.detections.publish(out)
        self.publish_markers(msg, visible)

    def evaluate(self, target, rot, trans, depth, msg):
        p = rot @ np.array(target["xyz"]) + trans
        z = float(p[2])
        if not self.min_range <= z <= self.max_range:
            return None

        fx, fy = self.info.k[0], self.info.k[4]
        cx, cy = self.info.k[2], self.info.k[5]
        u, v = fx * p[0] / z + cx, fy * p[1] / z + cy
        if not (0 <= u < msg.width and 0 <= v < msg.height):
            return None
        if fx * target["size"][0] / z < self.min_pixel_width:
            return None

        # Anything measurably nearer than the target along the same ray is in
        # front of it. The median rejects speckle, the 3σ margin the far-range error.
        ui, vi = int(u), int(v)
        patch = depth[max(vi - 2, 0):vi + 3, max(ui - 2, 0):ui + 3]
        finite = patch[np.isfinite(patch) & (patch > 0)]
        if finite.size and float(np.median(finite)) < z - max(self.occlusion_tolerance, 3 * sigma(z)):
            return None

        # Misses and position error grow with range, and past the reliable
        # range both get markedly worse.
        if self.rng.random() > self.confidence(z):
            return None
        error = float(sigma(z))
        noisy = [float(c + self.rng.gauss(0.0, error)) for c in p]

        det = Detection3D()
        det.header = msg.header
        det.id = target["name"]
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = target["label"]
        hypothesis.hypothesis.score = round(self.confidence(z), 3)
        hypothesis.pose.pose.position = Point(x=noisy[0], y=noisy[1], z=noisy[2])
        det.results.append(hypothesis)
        det.bbox = BoundingBox3D()
        det.bbox.center.position = Point(x=noisy[0], y=noisy[1], z=noisy[2])
        det.bbox.center.orientation.w = 1.0
        sx, sy, sz = target["size"]
        det.bbox.size = Vector3(x=sx, y=sy, z=sz)
        return det, z

    @staticmethod
    def confidence(z: float) -> float:
        if z <= DEPTH_RELIABLE:
            return max(0.4, 1.0 - (z - 4.0) / 12.0)
        return 0.4 - 0.3 * (z - DEPTH_RELIABLE) / (DEPTH_MAX - DEPTH_RELIABLE)

    def publish_markers(self, msg, visible) -> None:
        array = MarkerArray()
        for i, (target, z) in enumerate(visible):
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.header.stamp = msg.header.stamp
            marker.ns, marker.id = "targets", i
            marker.type, marker.action = Marker.CYLINDER, Marker.ADD
            marker.pose.position = Point(x=target["xyz"][0], y=target["xyz"][1], z=0.9)
            marker.pose.orientation.w = 1.0
            marker.scale = Vector3(x=0.5, y=0.5, z=1.8)
            fade = max(0.25, 1.0 - z / self.max_range)
            marker.color = ColorRGBA(r=1.0, g=0.35, b=0.0, a=float(fade))
            marker.lifetime.sec = 2
            array.markers.append(marker)
        self.markers.publish(array)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--targets", type=Path, default=Path(__file__).resolve()
                        .parent.parent / "worlds" / "warehouse_targets.json")
    known, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = SpatialDetector(json.loads(known.targets.read_text())["targets"])
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
