"""Opt-in 10 Hz / 960x540 CREStereo mapping profile for Jetson Orin NX."""

import importlib.util
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


_BASE_SPEC = importlib.util.spec_from_file_location(
    "lunar_usb_crestereo_launch", Path(__file__).with_name("usb_crestereo_rtabmap.launch.py")
)
_BASE = importlib.util.module_from_spec(_BASE_SPEC)
_BASE_SPEC.loader.exec_module(_BASE)


def generate_launch_description():
    common_names = [action.name for action in _BASE._BASE._common_launch_arguments()]
    forwarded = {name: LaunchConfiguration(name) for name in common_names}
    forwarded.update({
        "crestereo_params": LaunchConfiguration("crestereo_params"),
        "crestereo_model_path": LaunchConfiguration("crestereo_model_path"),
        "execution_provider": LaunchConfiguration("execution_provider"),
        "crestereo_frontend_target_rate": "10.0",
        "crestereo_frontend_poll_multiplier": "10.0",
        "crestereo_frontend_max_keypoints": "640",
        "crestereo_lightglue_cuda_graph_layers": "9",
        "crestereo_lightglue_cuda_graph_keypoints": "640",
        "crestereo_mapping_rate": "2.0",
        # CREStereo already publishes one normalized, atomic RGB-D packet.
        # Direct consumption removes one full 960x540 DDS relay; the adapter
        # remains active for H30 topic normalization and optional web outputs.
        "crestereo_sensor_rgbd_topic": "/usb_stereo/rgbd_image",
    })
    return LaunchDescription([
        *_BASE._BASE._common_launch_arguments(),
        DeclareLaunchArgument(
            "crestereo_params",
            default_value=PathJoinSubstitution([
                FindPackageShare("usb_camera_driver"),
                "config",
                "crestereo_depth_max_performance.yaml",
            ]),
        ),
        DeclareLaunchArgument(
            "crestereo_model_path",
            default_value=PathJoinSubstitution([
                FindPackageShare("usb_camera_driver"),
                "models",
                "crestereo_init_iter2_180x320_fp16.onnx",
            ]),
        ),
        DeclareLaunchArgument("execution_provider", default_value="cuda"),
        DeclareLaunchArgument("thermal_warning_temp_c", default_value="85.0"),
        DeclareLaunchArgument("thermal_stop_temp_c", default_value="92.0"),
        Node(
            package="lunar_usb_rtabmap_bringup",
            executable="jetson_health_guard",
            name="jetson_health_guard",
            parameters=[{
                "warning_temp_c": ParameterValue(
                    LaunchConfiguration("thermal_warning_temp_c"), value_type=float
                ),
                "stop_temp_c": ParameterValue(
                    LaunchConfiguration("thermal_stop_temp_c"), value_type=float
                ),
                "fail_on_throttle": True,
            }],
            output="screen",
            on_exit=Shutdown(reason="Jetson health guard exited"),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("lunar_usb_rtabmap_bringup"),
                    "launch",
                    "usb_crestereo_rtabmap.launch.py",
                ])
            ),
            launch_arguments=forwarded.items(),
        ),
    ])
