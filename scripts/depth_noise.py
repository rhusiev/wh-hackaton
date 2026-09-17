#!/usr/bin/env python3
"""Turn Gazebo's exact depth into OAK-D-like stereo depth.

Error grows with the square of range and is correlated over 8x8 px patches,
because stereo matches windows rather than pixels. Past 12 m whole patches drop
out, and anything outside 0.7-30 m after the noise is invalid (NaN).
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from stereo import DEPTH_MAX, DEPTH_MIN, dropout, sigma

PATCH = 8


class DepthNoise(Node):
    def __init__(self) -> None:
        super().__init__("depth_noise")
        self.rng = np.random.default_rng(1)
        self.out = self.create_publisher(Image, "/camera/depth/image_raw", qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/depth/ideal/image_raw", self.on_depth,
                                 qos_profile_sensor_data)

    def patches(self, draw, height: int, width: int) -> np.ndarray:
        coarse = draw((-(-height // PATCH), -(-width // PATCH)), dtype=np.float32)
        return coarse.repeat(PATCH, 0).repeat(PATCH, 1)[:height, :width]

    def on_depth(self, msg: Image) -> None:
        h, w = msg.height, msg.width
        z = np.frombuffer(msg.data, dtype=np.float32).reshape(h, w)
        # Unit variance overall: mostly patch-correlated, with a little per-pixel speckle.
        unit = (0.95 * self.patches(self.rng.standard_normal, h, w)
                + 0.3 * self.rng.standard_normal((h, w), dtype=np.float32))
        noisy = z + sigma(z) * unit
        noisy[(noisy < DEPTH_MIN) | (noisy > DEPTH_MAX) | ~np.isfinite(z)] = np.nan
        noisy[self.patches(self.rng.random, h, w) < dropout(z)] = np.nan
        msg.data = noisy.tobytes()
        self.out.publish(msg)

def main() -> None:
    rclpy.init()
    try:
        rclpy.spin(DepthNoise())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
