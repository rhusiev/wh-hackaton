"""Perception and AR stack: depth -> obstacles, targets, minimap.

Every module talks only over the topics in docs/DESIGN.md, so each can be switched
off here and replaced by your own node: detector:=none, mapper:=false, ar:=false.

Rates and resolutions here are sized for the Raspberry Pi 5 that carries this on
the real aircraft, not for the desktop running the simulator. Raising them will
look better in RViz and will not run on the drone.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.descriptions import ParameterValue
from launch_ros.actions import Node

ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = ROOT / "scripts"

# Must match the rgbd sensor in models/tricopter/model.sdf.
HFOV_HALF = 0.6004
DEPTH_MIN = 0.7
# Depth goes to 30 m, but avoidance only trusts the reliable 12 m.
SCAN_RANGE_MAX = 12.0


# No use_sim_time: Gazebo publishes /clock every physics step, and in rclpy that
# alone costs half a core per node. These nodes only use message stamps.
def _script(name: str, *args) -> list:
    return ["python3", str(SCRIPTS / name), *args]


def generate_launch_description() -> LaunchDescription:
    slam = LaunchConfiguration("slam")
    sim_time = ParameterValue(LaunchConfiguration("sim_time"), value_type=bool)

    args = [
        DeclareLaunchArgument("slam", default_value="false", choices=["true", "false"]),
        # False on the aircraft, where the clock is the machine's own. Only the nodes
        # that look up a transform need it; the rest read message stamps.
        DeclareLaunchArgument("sim_time", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("ar_port", default_value="8790"),
        # yolo runs the real network on the colour image, truth reads the true
        # positions of the people and is nearly free.
        DeclareLaunchArgument("detector", default_value="yolo", choices=["yolo", "truth", "none"]),
        # The known-pose grid mapper; with slam:=true RTAB-Map publishes /map instead.
        DeclareLaunchArgument("mapper", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("ar", default_value="true", choices=["true", "false"]),
        # The drone's colour image in the AR payload, for ./run.sh preview --camera
        # drone. Real glasses do not want it: the wearer is not looking at a screen.
        DeclareLaunchArgument("ar_video", default_value="true", choices=["true", "false"]),
        # Only the truth detector needs it: it reads that world's people.
        DeclareLaunchArgument("world", default_value="warehouse"),
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
            "use_sim_time": sim_time,
            "target_frame": "base_link",
            "transform_tolerance": 0.05,
            # From 2.8 m this still takes in the top of the 2.45 m shelf deck.
            "min_height": -0.5,
            "max_height": 0.45,
            "angle_min": -HFOV_HALF,
            "angle_max": HFOV_HALF,
            "angle_increment": 0.0087,
            "scan_time": 1.0 / 15.0,
            "range_min": DEPTH_MIN,
            "range_max": SCAN_RANGE_MAX,
            "use_inf": True,
        }],
    )

    detector = LaunchConfiguration("detector")
    detectors = [
        ExecuteProcess(cmd=_script(script, *args), output="screen",
                       condition=IfCondition(PythonExpression(["'", detector, f"' == '{name}'"])))
        for name, script, args in (("yolo", "person_detector.py", ()),
                                   ("truth", "spatial_detector.py",
                                    ("--world", LaunchConfiguration("world"))))
    ]

    # Two processes, one condition each: a flag cannot be a substitution, because an
    # empty argument is still an argument and argparse would reject it.
    ar_bridge = [
        ExecuteProcess(
            cmd=_script("ar_bridge.py", "--port", LaunchConfiguration("ar_port"),
                        "--world", LaunchConfiguration("world"), *extra),
            output="screen",
            condition=IfCondition(PythonExpression(
                ["'", LaunchConfiguration("ar"), "' == 'true' and '",
                 LaunchConfiguration("ar_video"), f"' == '{video}'"])),
        )
        for video, extra in (("true", ("--video",)), ("false", ()))
    ]

    scan_relay = ExecuteProcess(cmd=_script("scan_relay.py"), output="screen")
    vision_relay = ExecuteProcess(cmd=_script("vision_relay.py"), output="screen",
                                  condition=IfCondition(slam))

    grid_mapper = ExecuteProcess(
        cmd=_script("grid_mapper.py"), output="screen",
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration("mapper"), "' == 'true' and '", slam, "' != 'true'"])))

    # Visual odometry and the 2D grid the AR minimap is drawn from. Decimation
    # and the feature cap are the Pi 5 budget, not a quality choice.
    common = {
        "use_sim_time": sim_time,
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
            "Grid/MaxObstacleHeight": "3.5",
            "RGBD/NeighborLinkRefining": "true",
            "Mem/ImagePreDecimation": "2",
        }],
    )

    return LaunchDescription(
        args + [to_scan, scan_relay, grid_mapper, *detectors, *ar_bridge, visual_odometry,
                rtabmap, vision_relay])
