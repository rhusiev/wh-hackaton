#!/usr/bin/env python3
"""Find people and their heads in the OAK-D colour image and place them with the depth.

A YOLO11n-pose network gives a box and 17 body keypoints per person. The head
box is built from the face and ear keypoints, and the distance is the median
stereo depth over the person's torso. On the aircraft the same network runs on
the OAK-D itself, and the topics below stay the same.

    /oak/detections_2d       Detection2DArray, colour pixels, a "person" and a "head" per id
    /oak/spatial_detections  Detection3DArray, camera_optical_frame, the same pairs in metres
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort
import rclpy
from geometry_msgs.msg import Point, Vector3
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import ColorRGBA, Header
from vision_msgs.msg import (
    BoundingBox2D,
    BoundingBox3D,
    Detection2D,
    Detection2DArray,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)
from visualization_msgs.msg import Marker, MarkerArray

from stereo import DEPTH_MAX, DEPTH_MIN

MODEL = Path(__file__).resolve().parent.parent / "models" / "detector" / "yolo11n-pose.onnx"

FACE = slice(0, 5)  # nose, eyes, ears
SHOULDERS = [5, 6]
# The depth hits the front of the body, the track should sit in its middle.
BODY_HALF_DEPTH = 0.15
PAD = 114 / 255


@dataclass
class Person:
    box: np.ndarray  # x0, y0, x1, y1 in image pixels
    score: float
    head: np.ndarray | None


def head_box(keypoints: np.ndarray, box: np.ndarray, min_conf: float) -> np.ndarray | None:
    face = keypoints[FACE]
    face = face[face[:, 2] > min_conf, :2]
    shoulders = keypoints[SHOULDERS]
    has_shoulders = bool((shoulders[:, 2] > min_conf).all())

    # A head is about 1/8 of a standing body and half the shoulder width. Face
    # keypoints are the better anchor, but from behind only the shoulders remain.
    size = (box[3] - box[1]) / 8
    if has_shoulders:
        size = max(size, 0.55 * abs(shoulders[0, 0] - shoulders[1, 0]))
    if len(face) >= 2:
        size = max(size, 1.6 * np.ptp(face, axis=0).max())
        cx, cy = face.mean(axis=0)
        cy -= 0.15 * size  # eyes and ears sit below the middle of the head
    elif has_shoulders:
        cx, cy = shoulders[:, :2].mean(axis=0)
        cy -= size
    else:
        return None
    return np.array([cx - size / 2, cy - size / 2, cx + size / 2, cy + size / 2])


class PoseNetwork:
    def __init__(self, path: Path, threads: int) -> None:
        options = ort.SessionOptions()
        options.intra_op_num_threads = threads
        self.session = ort.InferenceSession(str(path), options, providers=["CPUExecutionProvider"])
        self.input = self.session.get_inputs()[0]
        self.height, self.width = self.input.shape[2:]

    def __call__(self, rgb: np.ndarray, min_score: float, min_keypoint: float,
                 iou: float = 0.5) -> list[Person]:
        scale = min(self.width / rgb.shape[1], self.height / rgb.shape[0])
        resized = cv2.resize(rgb, (round(rgb.shape[1] * scale), round(rgb.shape[0] * scale)))
        batch = np.full((1, 3, self.height, self.width), PAD, dtype=np.float32)
        batch[0, :, :resized.shape[0], :resized.shape[1]] = resized.transpose(2, 0, 1) / 255

        # One row per anchor: centre x, y, w, h, score, then x, y, confidence per keypoint.
        rows = self.session.run(None, {self.input.name: batch})[0][0].T
        rows = rows[rows[:, 4] > min_score]
        xywh = np.column_stack([rows[:, :2] - rows[:, 2:4] / 2, rows[:, 2:4]])
        keep = cv2.dnn.NMSBoxes(xywh.tolist(), rows[:, 4].tolist(), min_score, iou)

        people = []
        for i in np.ravel(keep):
            box = np.concatenate([xywh[i, :2], xywh[i, :2] + xywh[i, 2:]]) / scale
            keypoints = rows[i, 5:].reshape(17, 3).copy()
            keypoints[:, :2] /= scale
            people.append(Person(box, float(rows[i, 4]), head_box(keypoints, box, min_keypoint)))
        return people


def torso_depth(depth: np.ndarray, box: np.ndarray) -> float | None:
    x0, y0, x1, y1 = box
    w, h = x1 - x0, y1 - y0
    patch = depth[int(y0 + 0.15 * h):int(y0 + 0.6 * h) + 1, int(x0 + 0.3 * w):int(x1 - 0.3 * w) + 1]
    valid = patch[np.isfinite(patch) & (patch >= DEPTH_MIN) & (patch <= DEPTH_MAX)]
    # A mostly empty patch is a person seen past a gap or through stereo dropout.
    if valid.size < max(4, 0.3 * patch.size):
        return None
    return float(np.median(valid))


class PersonDetector(Node):
    def __init__(self) -> None:
        super().__init__("person_detector")
        self.declare_parameters("", [
            ("rate", 5.0),
            ("threads", 2),
            ("min_score", 0.4),
            ("min_keypoint", 0.5),
        ])
        self.period = 1.0 / self._p("rate")
        self.min_score, self.min_keypoint = self._p("min_score"), self._p("min_keypoint")
        self.network = PoseNetwork(MODEL, self._p("threads"))
        self.info: CameraInfo | None = None
        self.last_stamp = 0.0

        self.detections_2d = self.create_publisher(Detection2DArray, "/oak/detections_2d", 10)
        self.detections_3d = self.create_publisher(
            Detection3DArray, "/oak/spatial_detections", 10)
        self.markers = self.create_publisher(MarkerArray, "/oak/detection_markers", 10)
        self.create_subscription(
            CameraInfo, "/camera/color/camera_info", self.on_info, qos_profile_sensor_data)
        subs = [Subscriber(self, Image, topic, qos_profile=qos_profile_sensor_data)
                for topic in ("/camera/color/image_raw", "/camera/depth/image_raw")]
        self.sync = ApproximateTimeSynchronizer(subs, queue_size=5, slop=0.03)
        self.sync.registerCallback(self.on_frames)
        self.get_logger().info(f"{MODEL.name} on {self.network.width}x{self.network.height}")

    def _p(self, name: str):
        return self.get_parameter(name).value

    def on_info(self, msg: CameraInfo) -> None:
        self.info = msg

    def on_frames(self, colour: Image, depth: Image) -> None:
        stamp = colour.header.stamp.sec + colour.header.stamp.nanosec * 1e-9
        if self.info is None or stamp - self.last_stamp < self.period:
            return
        self.last_stamp = stamp

        rgb = np.frombuffer(colour.data, np.uint8).reshape(colour.height, colour.width, 3)
        metres = np.frombuffer(depth.data, np.float32).reshape(depth.height, depth.width)
        # Depth is aligned to the colour camera but may be sent at a lower resolution.
        to_depth = depth.width / colour.width

        people = self.network(rgb, self.min_score, self.min_keypoint)
        out_2d = Detection2DArray(header=colour.header)
        out_3d = Detection3DArray(header=colour.header)
        for i, person in enumerate(people):
            parts = [("person", person.box)]
            if person.head is not None:
                parts.append(("head", person.head))
            out_2d.detections += [_detection_2d(colour.header, str(i), label, box, person.score)
                                  for label, box in parts]
            z = torso_depth(metres, person.box * to_depth)
            if z is None:
                continue
            z += BODY_HALF_DEPTH
            out_3d.detections += [self._detection_3d(colour.header, str(i), label, box, z,
                                                     person.score)
                                  for label, box in parts]
        self.detections_2d.publish(out_2d)
        self.detections_3d.publish(out_3d)
        self.publish_markers(out_3d)

    def _detection_3d(self, header: Header, det_id: str, label: str, box: np.ndarray, z: float,
                      score: float) -> Detection3D:
        fx, fy, cx, cy = self.info.k[0], self.info.k[4], self.info.k[2], self.info.k[5]
        u, v = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
        centre = Point(x=float((u - cx) * z / fx), y=float((v - cy) * z / fy), z=z)
        width, height = float((box[2] - box[0]) * z / fx), float((box[3] - box[1]) * z / fy)
        det = Detection3D(header=header, id=det_id)
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id, hypothesis.hypothesis.score = label, score
        hypothesis.pose.pose.position = centre
        det.results.append(hypothesis)
        det.bbox = BoundingBox3D(size=Vector3(x=width, y=height, z=min(width, 2 * BODY_HALF_DEPTH)))
        det.bbox.center.position = centre
        det.bbox.center.orientation.w = 1.0
        return det

    def publish_markers(self, detections: Detection3DArray) -> None:
        array = MarkerArray(markers=[Marker(header=detections.header, action=Marker.DELETEALL)])
        for i, det in enumerate(detections.detections):
            head = det.results[0].hypothesis.class_id == "head"
            marker = Marker(header=detections.header, ns="people", id=i, action=Marker.ADD,
                            type=Marker.SPHERE if head else Marker.CUBE)
            marker.pose = det.bbox.center
            marker.scale = det.bbox.size
            marker.color = (ColorRGBA(r=1.0, g=0.9, b=0.1, a=0.8) if head
                            else ColorRGBA(r=1.0, g=0.35, b=0.0, a=0.5))
            array.markers.append(marker)
        self.markers.publish(array)


def _detection_2d(header: Header, det_id: str, label: str, box: np.ndarray,
                  score: float) -> Detection2D:
    det = Detection2D(header=header, id=det_id)
    hypothesis = ObjectHypothesisWithPose()
    hypothesis.hypothesis.class_id, hypothesis.hypothesis.score = label, score
    det.results.append(hypothesis)
    det.bbox = BoundingBox2D(size_x=float(box[2] - box[0]), size_y=float(box[3] - box[1]))
    det.bbox.center.position.x = float(box[0] + box[2]) / 2
    det.bbox.center.position.y = float(box[1] + box[3]) / 2
    return det


def main() -> None:
    rclpy.init()
    node = PersonDetector()
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
