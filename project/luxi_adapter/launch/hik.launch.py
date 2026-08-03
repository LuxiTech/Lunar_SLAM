"""Hik profile: relay an already-running Hik stereo/H30 pipeline to Lunar SLAM."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            description="Hik adapter parameter YAML selected by sensor_bringup.launch.py."),
        Node(
            package="luxi_adapter",
            executable="sensor_adapter_node",
            name="luxi_adapter",
            parameters=[LaunchConfiguration("config")],
            output="screen"),
    ])
