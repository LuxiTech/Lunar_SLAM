"""Start the Hik stereo/H30 pipeline and adapt it to the project sensor API."""

import os
from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, NotSubstitution


def _static_transforms(context):
    config_path = LaunchConfiguration("config").perform(context)
    with Path(config_path).open("r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file) or {}
    settings = config.get("luxi_adapter", {}).get("ros__parameters", {})
    if not isinstance(settings, dict):
        raise RuntimeError("luxi_adapter.ros__parameters must be a mapping.")

    def value(name, default):
        return str(settings.get(name, default))

    return [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="hik_base_to_left_camera_tf",
            arguments=[
                "--x", value("camera_x", 0.0), "--y", value("camera_y", 0.0),
                "--z", value("camera_z", 0.0), "--qx", value("camera_qx", -0.5),
                "--qy", value("camera_qy", 0.5), "--qz", value("camera_qz", -0.5),
                "--qw", value("camera_qw", 0.5),
                "--frame-id", value("base_frame", "base_link"),
                "--child-frame-id", value("camera_frame", "left_camera_optical_frame"),
            ],
            output="screen"),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="hik_left_camera_to_imu_tf",
            arguments=[
                "--x", value("imu_x", -0.06080516708588524),
                "--y", value("imu_y", -0.017371973406354727),
                "--z", value("imu_z", -0.12304891470347533),
                "--qx", value("imu_qx", 0.6868459901794202),
                "--qy", value("imu_qy", 0.024915427523349555),
                "--qz", value("imu_qz", -0.0010234338137173815),
                "--qw", value("imu_qw", 0.7263750820540378),
                "--frame-id", value("camera_frame", "left_camera_optical_frame"),
                "--child-frame-id", value("imu_frame", "imu_link"),
            ],
            output="screen"),
    ]


def generate_launch_description():
    hik_bringup_share = get_package_share_directory("hik_bringup")
    camera_params = f"{hik_bringup_share}/config/camera_params.yaml"
    mvs_root = os.path.abspath(os.environ.get("MVS_ROOT", "/opt/MVS"))
    mvs_library_dirs = {
        os.path.join(mvs_root, "lib", "aarch64"),
        os.path.join(mvs_root, "lib", "64"),
    }
    algorithm_environment = {
        "LD_LIBRARY_PATH": os.pathsep.join(
            entry
            for entry in os.environ.get("LD_LIBRARY_PATH", "").split(os.pathsep)
            if entry and os.path.abspath(entry) not in mvs_library_dirs
        )
    }
    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            description="Hik adapter parameter YAML selected by sensor_bringup.launch.py."),
        DeclareLaunchArgument("external_trigger", default_value="true"),
        DeclareLaunchArgument(
            "stereo_proc_params",
            default_value=f"{hik_bringup_share}/config/stereo_proc.yaml"),
        DeclareLaunchArgument("enable_adapter", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f"{hik_bringup_share}/launch/stereo_camera_bringup.launch.py"),
            launch_arguments={
                "use_rviz": "false",
                "use_imu": "true",
                "external_trigger": LaunchConfiguration("external_trigger"),
                "stereo_proc_params": LaunchConfiguration("stereo_proc_params"),
                "start_stereo_camera": NotSubstitution(
                    LaunchConfiguration("enable_adapter")
                ),
                "start_stereo_depth": "false",
            }.items(),
        ),
        # Keep both raw stereo images and the multi-megabyte RGBDImage inside
        # one process. The MVS camera initializes before depth is loaded.
        ComposableNodeContainer(
            package="rclcpp_components",
            executable="component_container_mt",
            name="luxi_sensor_container",
            namespace="",
            composable_node_descriptions=[
                ComposableNode(
                    package="hikrobot_camera_driver",
                    plugin="hikrobot_camera_driver::StereoCameraNode",
                    name="stereo_node",
                    parameters=[
                        camera_params,
                        {
                            "external_trigger": ParameterValue(
                                LaunchConfiguration("external_trigger"), value_type=bool
                            )
                        },
                    ],
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
            ],
            condition=IfCondition(LaunchConfiguration("enable_adapter")),
            additional_env=algorithm_environment,
            output="screen",
        ),
        TimerAction(period=5.0, actions=[
            LoadComposableNodes(
                target_container="/luxi_sensor_container",
                composable_node_descriptions=[
                    ComposableNode(
                        package="stereo_depth",
                        plugin="StereoDepthNode",
                        name="stereo_depth_node",
                        parameters=[LaunchConfiguration("stereo_proc_params")],
                        # Preserve lazy JPEG generation: the native publisher is
                        # exposed directly on the canonical preview topic, so it
                        # encodes only while a web/remote subscriber is present.
                        remappings=[
                            (
                                "/stereo/preview/left_rectified_color/compressed",
                                "/sensors/rgbd/color/image_raw/compressed",
                            )
                        ],
                        extra_arguments=[{"use_intra_process_comms": True}],
                    ),
                    ComposableNode(
                        package="luxi_adapter",
                        plugin="luxi_adapter::SensorAdapter",
                        name="luxi_adapter",
                        parameters=[LaunchConfiguration("config")],
                        extra_arguments=[{"use_intra_process_comms": True}],
                    ),
                ],
                condition=IfCondition(LaunchConfiguration("enable_adapter")),
            ),
            Node(
                package="stereo_depth",
                executable="stereo_depth_node",
                name="stereo_depth_node",
                parameters=[LaunchConfiguration("stereo_proc_params")],
                remappings=[
                    (
                        "/stereo/preview/left_rectified_color/compressed",
                        "/sensors/rgbd/color/image_raw/compressed",
                    )
                ],
                condition=UnlessCondition(LaunchConfiguration("enable_adapter")),
                additional_env=algorithm_environment,
                output="screen",
            ),
        ]),
        OpaqueFunction(function=_static_transforms),
    ])
