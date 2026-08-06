"""Run HLoc/ICP localization and plan on a selected saved OctoMap."""

import os
from pathlib import Path

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


def _workspace_root() -> Path:
    configured = os.environ.get("LUXI_WORKSPACE_ROOT", "").strip()
    if configured:
        return Path(configured).expanduser().resolve()
    for start in (Path(__file__).resolve(), Path.cwd().resolve()):
        for candidate in (start, *start.parents):
            if (candidate / "project").is_dir() and (candidate / "maps").is_dir():
                return candidate
    raise RuntimeError("Cannot locate lunar_slam; set LUXI_WORKSPACE_ROOT")


def generate_launch_description() -> LaunchDescription:
    cloud_path = LaunchConfiguration("cloud_path")
    octomap_library_path = str(_workspace_root() / "3parts" / "octomap" / "install" / "lib")
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
