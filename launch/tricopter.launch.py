"""Bringup for the real aircraft: camera, flight controller, perception, AR.

This is the same stack tricopter_sim.launch.py runs, with the simulator taken
out. What is left is the three things that differ on hardware:

    camera:=oak    start the depthai driver, once depthai-ros is installed
    fcu_url        a serial link to the flight controller, not a UDP port
    sim_time       false, because there is no /clock to follow

Nothing downstream changes, because the sim publishes the same topics the OAK-D
does. It has not been flown yet - see docs/DESIGN.md for what is still assumed.
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare

ROOT = Path(__file__).resolve().parent.parent


def generate_launch_description() -> LaunchDescription:
    args = [
        # none by default because depthai_ros_driver is not in this image: it is
        # installed on the Pi, and the sim never needs it.
        DeclareLaunchArgument("camera", default_value="none", choices=["oak", "none"]),
        # The Pi's link to the flight controller. Override for a different wiring.
        DeclareLaunchArgument("fcu_url", default_value="/dev/ttyAMA0:921600"),
        DeclareLaunchArgument("depth_decimation", default_value="4"),
        DeclareLaunchArgument("slam", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("mapper", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("ar", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("ar_port", default_value="8790"),
    ]

    oak = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("depthai_ros_driver"), "launch",
                                  "camera.launch.py"])),
        launch_arguments=[("params_file", str(ROOT / "config" / "oak.yaml"))],
        condition=IfCondition(PythonExpression(
            ["'", LaunchConfiguration("camera"), "' == 'oak'"])),
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ROOT / "launch" / "camera.launch.py")),
        launch_arguments=[("depth_decimation", LaunchConfiguration("depth_decimation")),
                          ("sim_time", "false")],
    )

    mavros_launch = PathJoinSubstitution([FindPackageShare("mavros"), "launch"])
    mavros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(PathJoinSubstitution([mavros_launch, "node.launch"])),
        launch_arguments=[
            ("pluginlists_yaml", str(ROOT / "config" / "mavros_plugins.yaml")),
            ("config_yaml", PathJoinSubstitution([mavros_launch, "apm_config.yaml"])),
            ("fcu_url", LaunchConfiguration("fcu_url")),
            ("gcs_url", ""),
            ("tgt_system", "1"),
            ("tgt_component", "1"),
        ],
    )

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ROOT / "launch" / "perception.launch.py")),
        launch_arguments=[
            ("slam", LaunchConfiguration("slam")),
            # Not "truth": that detector reads a world file, and there is no world.
            ("detector", "yolo"),
            ("mapper", LaunchConfiguration("mapper")),
            ("ar", LaunchConfiguration("ar")),
            ("ar_port", LaunchConfiguration("ar_port")),
            ("sim_time", "false"),
        ],
    )

    return LaunchDescription(args + [oak, camera, mavros, perception])
