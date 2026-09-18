#!/usr/bin/env python3
"""Feed RTAB-Map's visual odometry to the EKF as VISION_POSITION_ESTIMATE.

With config/gps_denied.parm the flight controller takes its position from
ExternalNav, which mavros sends from /mavros/vision_pose/pose. rgbd_odometry
publishes an Odometry on /odom instead, and nothing else converts the two.

A lost frame comes through as a pose full of NaNs, which the EKF would take as a
real measurement, so those frames are dropped and the EKF coasts on the IMU.

Without GNSS nothing ever sets the EKF origin or home, and ArduPilot will not arm
until both exist, so this node sends them once the odometry is running. The
latitude and longitude are arbitrary indoors - everything downstream works in the
local frame - but the EKF needs some earth reference to hang its origin on.
"""

import math

import rclpy
from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import HomePosition
from mavros_msgs.srv import CommandHome
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data

ORIGIN = (-35.363262, 149.165237, 584.0)


def main() -> None:
    rclpy.init()
    node = Node("vision_relay")
    out = node.create_publisher(PoseStamped, "/mavros/vision_pose/pose", 5)
    origin = node.create_publisher(
        GeoPointStamped, "/mavros/global_position/set_gp_origin",
        QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
    home = node.create_client(CommandHome, "/mavros/cmd/set_home")
    sent = False

    # Home is only accepted once the EKF has an origin and a position it trusts,
    # which is a moment after the origin lands, so it is asked for until it takes.
    def set_home() -> None:
        if home.service_is_ready():
            home.call_async(CommandHome.Request(
                current_gps=False, latitude=ORIGIN[0], longitude=ORIGIN[1], altitude=ORIGIN[2]))

    asking = node.create_timer(2.0, set_home)
    node.create_subscription(
        HomePosition, "/mavros/home_position/home",
        lambda _: (asking.cancel(), node.get_logger().info("home set")), 1)

    def relay(msg: Odometry) -> None:
        nonlocal sent
        pose = msg.pose.pose
        if any(math.isnan(v) for v in (pose.position.x, pose.position.y, pose.position.z)):
            return
        out.publish(PoseStamped(header=msg.header, pose=pose))
        if not sent:
            point = GeoPointStamped(header=msg.header)
            point.position.latitude, point.position.longitude, point.position.altitude = ORIGIN
            origin.publish(point)
            node.get_logger().info("origin sent, vision pose flowing")
            sent = True

    node.create_subscription(Odometry, "/odom", relay, qos_profile_sensor_data)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
