"""Localize in a selected RTAB-Map database and plan on its saved OctoMap."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    database_path = LaunchConfiguration("database_path")
    localization_launch = PythonLaunchDescriptionSource([
        FindPackageShare("luxi_rtab_map"), "/launch/rgbd_localization.launch.py"
    ])
    return LaunchDescription([
        DeclareLaunchArgument("database_path", default_value=""),
        DeclareLaunchArgument("octomap_path", default_value=""),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/navigation/cmd_vel"),
        IncludeLaunchDescription(
            localization_launch,
            launch_arguments={"database_path": database_path, "rviz": "false"}.items()),
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
