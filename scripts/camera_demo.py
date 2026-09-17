#!/usr/bin/env python3
"""Show the OAK-D colour image next to its depth, false-coloured over the 0.7-30 m range.

People and heads from /oak/detections_2d are drawn on the colour image.

    ./run.sh demo                  # live window
    ./run.sh demo --save demo.png  # write one frame and exit, for headless hosts
"""

import argparse

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

from stereo import DEPTH_MAX, DEPTH_MIN

# The detector runs slower than the camera, so its boxes are drawn on later frames too.
DETECTION_MAX_AGE = 0.5
BOX_COLOURS = {"person": (0, 0, 255), "head": (0, 255, 255)}


def colourize(depth: np.ndarray) -> np.ndarray:
    # Square root, so the near metres that matter for flying keep most of the colours.
    valid = np.isfinite(depth)
    scaled = np.sqrt((np.clip(np.where(valid, depth, DEPTH_MAX), DEPTH_MIN, DEPTH_MAX) - DEPTH_MIN)
                     / (DEPTH_MAX - DEPTH_MIN))
    image = cv2.applyColorMap((255 * (1 - scaled)).astype(np.uint8), cv2.COLORMAP_TURBO)
    image[~valid] = 0  # no stereo match or outside the range
    return image


class CameraDemo(Node):
    def __init__(self, save: str | None) -> None:
        super().__init__("camera_demo")
        self.save = save
        self.bridge = CvBridge()
        subs = [Subscriber(self, Image, topic, qos_profile=qos_profile_sensor_data)
                for topic in ("/camera/color/image_raw", "/camera/depth/image_raw")]
        self.sync = ApproximateTimeSynchronizer(subs, queue_size=5, slop=0.05)
        self.sync.registerCallback(self.on_frames)
        self.detections = Detection2DArray()
        self.create_subscription(Detection2DArray, "/oak/detections_2d",
                                 lambda msg: setattr(self, "detections", msg), 10)

    def on_frames(self, colour: Image, depth: Image) -> None:
        left = self.bridge.imgmsg_to_cv2(colour, "bgr8")
        self.draw_detections(left, _seconds(colour) - _seconds(self.detections))
        metres = self.bridge.imgmsg_to_cv2(depth, "32FC1")
        frame = np.hstack([left, colourize(metres)])
        distance = metres[depth.height // 2, depth.width // 2]
        label = f"centre {distance:.2f} m" if np.isfinite(distance) else "centre out of range"
        cv2.putText(frame, label, (left.shape[1] + 10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        if self.save:
            cv2.imwrite(self.save, frame)
            self.get_logger().info(f"wrote {self.save}")
            raise SystemExit
        cv2.imshow("OAK-D colour | depth", frame)
        if cv2.waitKey(1) in (ord("q"), 27):
            raise SystemExit


    def draw_detections(self, image: np.ndarray, age: float) -> None:
        if abs(age) > DETECTION_MAX_AGE:
            return
        for det in self.detections.detections:
            label = det.results[0].hypothesis.class_id
            c, half = det.bbox.center.position, (det.bbox.size_x / 2, det.bbox.size_y / 2)
            corners = [(int(c.x - sign * half[0]), int(c.y - sign * half[1])) for sign in (1, -1)]
            cv2.rectangle(image, *corners, BOX_COLOURS.get(label, (255, 255, 255)), 1)
            if label == "person":
                score = det.results[0].hypothesis.score
                cv2.putText(image, f"{score:.2f}", (corners[0][0], corners[0][1] - 3),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, BOX_COLOURS[label], 1)


def _seconds(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--save", help="write one side-by-side frame to this path and exit")
    args = parser.parse_args()
    rclpy.init()
    try:
        rclpy.spin(CameraDemo(args.save))
    except (SystemExit, KeyboardInterrupt):
        pass


if __name__ == "__main__":
    main()
