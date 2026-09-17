#!/usr/bin/env python3
"""Republish /scan RELIABLE for mavros's obstacle_distance plugin.

pointcloud_to_laserscan publishes BEST_EFFORT and the plugin subscribes RELIABLE,
so they never match, and neither exposes a QoS override.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def main() -> None:
    rclpy.init()
    node = Node("scan_relay")
    out = node.create_publisher(LaserScan, "/mavros/obstacle/send", 5)
    node.create_subscription(LaserScan, "/scan", out.publish, qos_profile_sensor_data)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
