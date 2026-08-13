"""Run the existing HLoc/ICP localization chain with ground-supported 3D A*."""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
    octomap_library_path = str(_workspace_root() / "3parts" / "octomap" / "install" / "lib")
    system_library_path = "/lib/aarch64-linux-gnu"
    library_path = os.pathsep.join(
        part for part in (
            system_library_path,
            octomap_library_path,
            os.environ.get("LD_LIBRARY_PATH", ""),
        ) if part
    )
    localization_launch = PythonLaunchDescriptionSource([
        FindPackageShare("luxi_hloc"), "/launch/hloc_icp_localization.launch.py"
    ])
    config_path = PathJoinSubstitution([
        FindPackageShare("luxi_3d_navigation"), "config", "navigation.yaml"
    ])

    return LaunchDescription([
        DeclareLaunchArgument("database_path", default_value=""),
        DeclareLaunchArgument("octomap_path", default_value=""),
        DeclareLaunchArgument("cloud_path", default_value=""),
        DeclareLaunchArgument("hloc_map_directory", default_value=""),
        DeclareLaunchArgument("semantic_path", default_value=""),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/navigation/cmd_vel"),
        SetEnvironmentVariable("LD_LIBRARY_PATH", library_path),
        Node(
            package="rtabmap_odom",
            executable="rgbd_odometry",
            name="navigation_rgbd_odometry",
            parameters=[{
                "frame_id": "base_link",
                "odom_frame_id": "odom",
                "publish_tf": True,
                "subscribe_rgbd": True,
                "wait_imu_to_init": True,
                "always_check_imu_tf": False,
                "always_process_most_recent_frame": True,
                "qos": 2,
                "qos_imu": 2,
            }],
            remappings=[
                ("rgbd_image", "/sensors/rgbd/rgbd_image"),
                ("odom", "/navigation/odom"),
                ("imu", "/sensors/imu/data"),
            ],
            arguments=["--ros-args", "--log-level", "warn"],
            output="screen",
        ),
        IncludeLaunchDescription(
            localization_launch,
            launch_arguments={
                "map_directory": LaunchConfiguration("hloc_map_directory"),
                "cloud_path": LaunchConfiguration("cloud_path"),
                "device": "cuda",
                "icp_publish_tf": "true",
            }.items(),
        ),
        Node(
            package="luxi_voxel_navigation",
            executable="octomap_file_loader_node",
            name="octomap_file_loader",
            parameters=[{"initial_octomap_path": LaunchConfiguration("octomap_path")}],
            output="screen",
        ),
        Node(
            package="luxi_3d_navigation",
            executable="octomap_3d_astar_planner_node",
            name="octomap_3d_astar_planner",
            parameters=[config_path, {
                "octomap_topic": "/navigation/octomap",
                "cloud_path": LaunchConfiguration("cloud_path"),
                "semantic_path": LaunchConfiguration("semantic_path"),
            }],
            output="screen",
        ),
        Node(
            package="luxi_3d_navigation",
            executable="terrain_path_follower_node",
            name="terrain_path_follower",
            parameters=[config_path, {"cmd_vel_topic": LaunchConfiguration("cmd_vel_topic")}],
            output="screen",
        ),
    ])
