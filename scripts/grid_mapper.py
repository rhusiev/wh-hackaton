#!/usr/bin/env python3
"""Build the 2D occupancy grid /map from the depth scan and the drone's pose.

The stand-in for RTAB-Map when slam:=false: the pose is taken as known, so the
grid is only as good as the scan. Each beam clears the cells it passes and marks
the cell it ends in; a beam with no return clears up to the scan's range. A cell
hit often enough clears ten times slower: a rack upright is thinner than a cell,
and beams passing beside it through the open shelves would otherwise clear it
again. Anything that does leave, like a person seen in the slice, still clears.
"""

from __future__ import annotations

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from geometry import yaw

HIT, MISS = 0.85, -0.4
LOG_ODDS_MIN, LOG_ODDS_MAX = -2.0, 3.5
SOLID = 2.5              # log-odds from which misses count a tenth


class GridMapper(Node):
    def __init__(self) -> None:
        super().__init__("grid_mapper")
        self.declare_parameters("", [
            ("map_frame", "map"),
            ("resolution", 0.2),
            ("origin", [-20.0, -20.0]),
            ("size", [40.0, 40.0]),
            ("publish_period", 1.0),
        ])
        self.map_frame = self._p("map_frame")
        self.res = self._p("resolution")
        self.origin = np.array(self._p("origin"))
        self.shape = tuple(int(round(s / self.res)) for s in reversed(self._p("size")))
        self.log_odds = np.zeros(self.shape, dtype=np.float32)
        self.seen = np.zeros(self.shape, dtype=bool)

        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer, self)
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.grid = self.create_publisher(OccupancyGrid, "/map", latched)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.create_timer(self._p("publish_period"), self.publish)

    def _p(self, name: str):
        return self.get_parameter(name).value

    def on_scan(self, scan: LaserScan) -> None:
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, scan.header.frame_id,
                                                 rclpy.time.Time())
        except Exception as exc:
            self.get_logger().warn(f"no {self.map_frame} -> {scan.header.frame_id}: {exc}",
                                   throttle_duration_sec=5.0)
            return
        t, q = tf.transform.translation, tf.transform.rotation
        heading = yaw(q)

        ranges = np.asarray(scan.ranges, dtype=np.float32)
        angles = heading + scan.angle_min + scan.angle_increment * np.arange(len(ranges))
        hit = np.isfinite(ranges)
        # inf is "nothing inside the range", NaN or too close is no answer at all.
        usable = hit | (np.isinf(ranges) & (ranges > 0))
        length = np.where(hit, ranges, scan.range_max)[usable]
        angles = angles[usable]

        # Samples every half cell along each beam, stopping one cell short of the end.
        steps = np.arange(0.0, scan.range_max, self.res / 2)
        along = steps[None, :] < (length[:, None] - self.res)
        xs = t.x + steps[None, :] * np.cos(angles)[:, None]
        ys = t.y + steps[None, :] * np.sin(angles)[:, None]
        self.add(xs[along], ys[along], MISS)
        ends = hit[usable]
        self.add(t.x + length[ends] * np.cos(angles[ends]),
                 t.y + length[ends] * np.sin(angles[ends]), HIT)

    def add(self, xs: np.ndarray, ys: np.ndarray, delta: float) -> None:
        cols = ((xs - self.origin[0]) / self.res).astype(int)
        rows = ((ys - self.origin[1]) / self.res).astype(int)
        inside = (rows >= 0) & (rows < self.shape[0]) & (cols >= 0) & (cols < self.shape[1])
        cells = np.unique(rows[inside] * self.shape[1] + cols[inside])
        flat = self.log_odds.reshape(-1)
        step = np.where(flat[cells] >= SOLID, delta / 10, delta) if delta < 0 else delta
        flat[cells] = np.clip(flat[cells] + step, LOG_ODDS_MIN, LOG_ODDS_MAX)
        self.seen.reshape(-1)[cells] = True

    def publish(self) -> None:
        msg = OccupancyGrid()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.res
        msg.info.height, msg.info.width = self.shape
        msg.info.origin.position.x, msg.info.origin.position.y = map(float, self.origin)
        msg.info.origin.orientation.w = 1.0
        probability = 100 / (1 + np.exp(-self.log_odds))
        msg.data = np.where(self.seen, probability, -1).astype(np.int8).ravel().tolist()
        self.grid.publish(msg)


def main() -> None:
    rclpy.init()
    node = GridMapper()
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
