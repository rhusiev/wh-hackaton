"""Gazebo + ArduPilot SITL bringup for the tricopter, in world:=warehouse or world:=garden.

Everything here is simulation. The camera geometry and the perception stack it
starts are shared with the aircraft, in camera.launch.py and perception.launch.py,
which tricopter.launch.py includes instead.

ArduPilot SITL itself is normally started separately with scripts/run_sitl.sh so
that the MAVProxy console keeps a usable stdin; pass sitl:=true for a headless
one-command bringup.
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

ROOT = Path(__file__).resolve().parent.parent


def _prepend(var: str, value: str) -> SetEnvironmentVariable:
    existing = os.environ.get(var, "")
    return SetEnvironmentVariable(var, f"{value}:{existing}" if existing else value)


def generate_launch_description() -> LaunchDescription:
    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    verbose = LaunchConfiguration("verbose")

    args = [
        DeclareLaunchArgument("world", default_value="warehouse"),
        DeclareLaunchArgument("gui", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("verbose", default_value="1"),
        # 1 = full 640x400 into the cloud and scan, 2 = 320x200, 4 = 160x100.
        DeclareLaunchArgument("depth_decimation", default_value="4"),
        DeclareLaunchArgument("sitl", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("gps_denied", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("mavros", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("fcu_url", default_value="udp://:14551@"),
        DeclareLaunchArgument("perception", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("slam", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("detector", default_value="yolo", choices=["yolo", "truth", "none"]),
        DeclareLaunchArgument("mapper", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("ar", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("ar_port", default_value="8790"),
        DeclareLaunchArgument("rviz", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("foxglove", default_value="false", choices=["true", "false"]),
    ]

    models = ":".join(str(p) for p in (ROOT / "models", ROOT / "models" / "people", ROOT / "worlds"))
    env = [_prepend("GZ_SIM_RESOURCE_PATH", models)]

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments=[
            (
                "gz_args",
                [
                    world, ".sdf -r -v ", verbose,
                    PythonExpression(["'' if '", gui, "' == 'true' else ' -s'"]),
                ],
            ),
            ("on_exit_shutdown", "true"),
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        output="screen",
        parameters=[{"config_file": str(ROOT / "config" / "gz_bridge.yaml")}],
    )

    image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="image_bridge",
        output="screen",
        arguments=["/rgbd/image", "/rgbd/depth_image"],
        remappings=[
            ("/rgbd/image", "/camera/color/image_raw"),
            ("/rgbd/depth_image", "/camera/depth/ideal/image_raw"),
        ],
    )

    depth_noise = ExecuteProcess(
        cmd=["python3", str(ROOT / "scripts" / "depth_noise.py")], output="screen")

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ROOT / "launch" / "camera.launch.py")),
        launch_arguments=[("depth_decimation", LaunchConfiguration("depth_decimation")),
                          ("sim_time", "true")],
    )

    sitl = ExecuteProcess(
        cmd=[str(ROOT / "scripts" / "run_sitl.sh")],
        additional_env={"MAP": "0", "CONSOLE": "0",
                        "GPS_DENIED": LaunchConfiguration("gps_denied")},
        output="screen",
        condition=IfCondition(LaunchConfiguration("sitl")),
    )

    # mavros's node.launch, as its apm.launch uses it; hand-rolling the node means
    # re-deriving how its plugin sub-nodes get their parameters.
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
        condition=IfCondition(LaunchConfiguration("mavros")),
    )

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ROOT / "launch" / "perception.launch.py")),
        launch_arguments=[
            ("slam", LaunchConfiguration("slam")),
            ("detector", LaunchConfiguration("detector")),
            ("mapper", LaunchConfiguration("mapper")),
            ("ar", LaunchConfiguration("ar")),
            ("ar_port", LaunchConfiguration("ar_port")),
            ("world", LaunchConfiguration("world")),
            ("sim_time", "true"),
        ],
        condition=IfCondition(LaunchConfiguration("perception")),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", str(ROOT / "config" / "sim.rviz")],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    foxglove = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("foxglove_bridge"), "launch", "foxglove_bridge_launch.xml"]
            )
        ),
        launch_arguments=[("port", "8888")],
        condition=IfCondition(LaunchConfiguration("foxglove")),
    )

    ld = LaunchDescription(args + env)
    ld.add_action(gz_sim)
    ld.add_action(bridge)
    ld.add_action(image_bridge)
    ld.add_action(depth_noise)
    ld.add_action(camera)
    ld.add_action(TimerAction(period=9.0, actions=[sitl]))
    ld.add_action(TimerAction(period=11.0, actions=[mavros]))
    ld.add_action(TimerAction(period=12.0, actions=[perception]))
    ld.add_action(rviz)
    ld.add_action(foxglove)
    return ld
