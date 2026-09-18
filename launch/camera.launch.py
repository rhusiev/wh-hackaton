"""Where the camera is on the airframe, and the cloud its depth image becomes.

None of this is simulation: the mounting angle is a fact about the drone, and a
real OAK-D publishes the same /camera/depth/image_raw the bridge does, so both
the sim and the aircraft include this file unchanged.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode, ParameterValue

# Must match camera_link in models/tricopter/model.sdf.
CAMERA_XYZ = ("0.135", "0", "-0.01")
CAMERA_PITCH = "0.2618"


def generate_launch_description() -> LaunchDescription:
    args = [
        # 1 = full 640x400 into the cloud and scan, 2 = 320x200, 4 = 160x100.
        DeclareLaunchArgument("depth_decimation", default_value="4"),
        DeclareLaunchArgument("sim_time", default_value="true", choices=["true", "false"]),
    ]
    sim_time = ParameterValue(LaunchConfiguration("sim_time"), value_type=bool)

    static_tf = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf_base_to_camera",
            arguments=[
                "--x", CAMERA_XYZ[0], "--y", CAMERA_XYZ[1], "--z", CAMERA_XYZ[2],
                "--roll", "0", "--pitch", CAMERA_PITCH, "--yaw", "0",
                "--frame-id", "base_link", "--child-frame-id", "camera_link",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf_camera_to_optical",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--roll", "-1.5708", "--pitch", "0", "--yaw", "-1.5708",
                "--frame-id", "camera_link", "--child-frame-id", "camera_optical_frame",
            ],
        ),
    ]

    # Gazebo's own /rgbd/points is X-forward while being tagged with the optical
    # frame, so the usable cloud is rebuilt here from the depth image instead.
    # The default 4x decimation keeps 160 columns, still finer than the scan's 139
    # bins, and cuts the cloud and the scan slicer's work 16x.
    decimation = ParameterValue(LaunchConfiguration("depth_decimation"), value_type=int)
    depth_to_cloud = ComposableNodeContainer(
        name="depth_proc",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="image_proc",
                plugin="image_proc::CropDecimateNode",
                name="depth_decimate",
                parameters=[{
                    "use_sim_time": sim_time,
                    "decimation_x": decimation,
                    "decimation_y": decimation,
                    "interpolation": 0,  # nearest, depth must not be blended
                }],
                remappings=[
                    ("in/image_raw", "/camera/depth/image_raw"),
                    ("in/camera_info", "/camera/depth/camera_info"),
                    ("out/image_raw", "/camera/depth/decimated/image_raw"),
                    ("out/camera_info", "/camera/depth/decimated/camera_info"),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="depth_image_proc",
                plugin="depth_image_proc::PointCloudXyzNode",
                name="point_cloud_xyz",
                parameters=[{"use_sim_time": sim_time}],
                remappings=[
                    ("image_rect", "/camera/depth/decimated/image_raw"),
                    ("camera_info", "/camera/depth/decimated/camera_info"),
                    ("points", "/camera/depth/points"),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
    )

    return LaunchDescription(args + static_tf + [depth_to_cloud])
