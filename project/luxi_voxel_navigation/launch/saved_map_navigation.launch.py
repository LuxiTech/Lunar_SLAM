"""Run HLoc/ICP localization and plan on a selected saved OctoMap."""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    cloud_path = LaunchConfiguration("cloud_path")
    octomap_library_path = (
        "/home/lunar/project/lunar_slam/3parts/octomap/install/lib"
    )
    library_path = os.pathsep.join(
        part
        for part in (octomap_library_path, os.environ.get("LD_LIBRARY_PATH", ""))
        if part
    )
    localization_launch = PythonLaunchDescriptionSource([
        FindPackageShare("luxi_hloc"), "/launch/hloc_icp_localization.launch.py"
    ])
    return LaunchDescription([
        DeclareLaunchArgument("database_path", default_value=""),
        DeclareLaunchArgument("octomap_path", default_value=""),
        DeclareLaunchArgument("cloud_path", default_value=""),
        DeclareLaunchArgument("hloc_map_directory", default_value=""),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/navigation/cmd_vel"),
        SetEnvironmentVariable("LD_LIBRARY_PATH", library_path),
        IncludeLaunchDescription(
            localization_launch,
            launch_arguments={
                "map_directory": LaunchConfiguration("hloc_map_directory"),
                "cloud_path": cloud_path,
                "device": "cuda",
                "icp_publish_tf": "true",
            }.items()),
        Node(
            package="luxi_voxel_navigation",
            executable="octomap_file_loader_node",
            name="octomap_file_loader",
            parameters=[{"initial_octomap_path": LaunchConfiguration("octomap_path")}],
            output="screen"),
        Node(
            package="luxi_voxel_navigation",
            executable="octomap_astar_planner_node",
            name="octomap_astar_planner",
            parameters=[{"octomap_topic": "/navigation/octomap"}],
            output="screen"),
        Node(
            package="luxi_voxel_navigation",
            executable="path_follower_node",
            name="path_follower",
            parameters=[{"cmd_vel_topic": LaunchConfiguration("cmd_vel_topic")}],
            output="screen"),
    ])
