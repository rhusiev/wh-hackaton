"""Perception and AR stack: depth -> obstacles, targets, minimap.

Rates and resolutions here are sized for the Raspberry Pi 5 that carries this on
the real aircraft, not for the desktop running the simulator. Raising them will
look better in RViz and will not run on the drone.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = ROOT / "scripts"

# Must match the rgbd sensor in models/tricopter/model.sdf.
HFOV_HALF = 0.6004
DEPTH_MIN = 0.7
DEPTH_MAX = 12.0


# No use_sim_time: Gazebo publishes /clock every physics step, and in rclpy that
# alone costs half a core per node. These nodes only use message stamps.
def _script(name: str, *args) -> list:
    return ["python3", str(SCRIPTS / name), *args]


def generate_launch_description() -> LaunchDescription:
    slam = LaunchConfiguration("slam")

    args = [
        DeclareLaunchArgument("slam", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("ar_port", default_value="8790"),
        DeclareLaunchArgument("detector", default_value="true", choices=["true", "false"]),
    ]

    # ArduPilot's proximity/avoidance wants a flat scan. The slice is taken in
    # base_link so it stays level while the drone pitches into a move.
    to_scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="depth_to_scan",
        output="screen",
        remappings=[("cloud_in", "/camera/depth/points"), ("scan", "/scan")],
        parameters=[{
            "use_sim_time": True,
            "target_frame": "base_link",
            "transform_tolerance": 0.05,
            "min_height": -0.35,
            "max_height": 0.45,
            "angle_min": -HFOV_HALF,
            "angle_max": HFOV_HALF,
            "angle_increment": 0.0087,
            "scan_time": 1.0 / 15.0,
            "range_min": DEPTH_MIN,
            "range_max": DEPTH_MAX,
            "use_inf": True,
        }],
    )

    detector = ExecuteProcess(
        cmd=_script("spatial_detector.py"),
        output="screen",
        condition=IfCondition(LaunchConfiguration("detector")),
    )

    ar_bridge = ExecuteProcess(
        cmd=_script("ar_bridge.py", "--port", LaunchConfiguration("ar_port")),
        output="screen",
    )

    scan_relay = ExecuteProcess(cmd=_script("scan_relay.py"), output="screen")

    # Visual odometry and the 2D grid the AR minimap is drawn from. Decimation
    # and the feature cap are the Pi 5 budget, not a quality choice.
    common = {
        "use_sim_time": True,
        "frame_id": "base_link",
        "odom_frame_id": "vodom",
        "approx_sync": True,
        "subscribe_depth": True,
        "wait_imu_to_init": False,
    }
    remaps = [
        ("rgb/image", "/camera/color/image_raw"),
        ("rgb/camera_info", "/camera/color/camera_info"),
        ("depth/image", "/camera/depth/image_raw"),
    ]

    visual_odometry = Node(
        package="rtabmap_odom",
        executable="rgbd_odometry",
        name="rgbd_odometry",
        output="screen",
        condition=IfCondition(slam),
        remappings=remaps,
        parameters=[common | {
            "Odom/Strategy": "0",
            "Vis/MaxFeatures": "400",
            "Vis/MinInliers": "15",
            "Mem/ImagePreDecimation": "2",
        }],
    )

    rtabmap = Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        condition=IfCondition(slam),
        arguments=["--delete_db_on_start"],
        remappings=remaps + [("scan", "/scan")],
        parameters=[common | {
            "subscribe_scan": True,
            "Rtabmap/DetectionRate": "1.0",
            "Grid/CellSize": "0.1",
            "Grid/RangeMax": "8.0",
            "Grid/FromDepth": "false",
            "Grid/MaxObstacleHeight": "2.5",
            "RGBD/NeighborLinkRefining": "true",
            "Mem/ImagePreDecimation": "2",
        }],
    )

    return LaunchDescription(
        args + [to_scan, scan_relay, detector, ar_bridge, visual_odometry, rtabmap])
