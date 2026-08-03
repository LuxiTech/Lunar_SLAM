"""Start the calibrated Hik stereo/H30 sensor pipeline without a SLAM backend."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("hik_bringup")
    depth_params = LaunchConfiguration("depth_params")
    return LaunchDescription([
        DeclareLaunchArgument(
            "depth_params",
            default_value=os.path.join(share, "config", "rtabmap_stereo_imu.yaml"),
            description="Hik depth parameters; the default is the reviewed 512x375 CUDA-SGM profile."),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, "launch", "stereo_imu_calibrated.launch.py"))),
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="base_to_left_camera_optical_static_tf", output="screen",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--qx", "-0.5", "--qy", "0.5", "--qz", "-0.5", "--qw", "0.5",
                "--frame-id", "base_link", "--child-frame-id", "left_camera_optical_frame"]),
        TimerAction(
            period=5.0,
            actions=[Node(
                package="stereo_depth", executable="stereo_depth_node",
                name="stereo_depth_node", output="screen", parameters=[depth_params])]),
    ])
