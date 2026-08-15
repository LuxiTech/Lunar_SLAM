"""Start HLoc coarse localization and feed accepted poses to Open3D ICP."""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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


WORKSPACE = _workspace_root()


def _latest_cloud() -> Path:
    candidates = []
    for directory in (WORKSPACE / "maps" / "octo_maps").glob("map*_octomap"):
        try:
            map_number = int(directory.name.removeprefix("map").removesuffix("_octomap"))
        except ValueError:
            continue
        for cloud in directory.glob("*_cloud.ply"):
            candidates.append((map_number, cloud.stat().st_mtime, cloud))
    if not candidates:
        return WORKSPACE / "maps" / "octo_maps" / "map001_octomap" / "map001_cloud.ply"
    return max(candidates, key=lambda item: (item[0], item[1]))[2]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_directory",
                default_value=str(WORKSPACE / "maps/hloc_maps/map012"),
            ),
            DeclareLaunchArgument("cloud_path", default_value=str(_latest_cloud())),
            DeclareLaunchArgument("device", default_value="cuda"),
            DeclareLaunchArgument("icp_publish_tf", default_value="false"),
            DeclareLaunchArgument(
                "hloc_config_file",
                default_value=PathJoinSubstitution([
                    FindPackageShare("luxi_hloc"), "config", "hloc_localization.yaml"
                ]),
            ),
            DeclareLaunchArgument(
                "icp_config_file",
                default_value=PathJoinSubstitution([
                    FindPackageShare("luxi_location"), "config", "icp_localization.yaml"
                ]),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("luxi_hloc"), "/launch/hloc_localization.launch.py"]
                ),
                launch_arguments={
                    "map_directory": LaunchConfiguration("map_directory"),
                    "device": LaunchConfiguration("device"),
                    "config_file": LaunchConfiguration("hloc_config_file"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("luxi_location"), "/launch/icp_localization.launch.py"]
                ),
                launch_arguments={
                    "map_path": LaunchConfiguration("cloud_path"),
                    "config_file": LaunchConfiguration("icp_config_file"),
                    "initial_pose_topic": "/luxi_hloc/coarse_pose",
                    "initial_pose_max_variance": "0.5",
                    "publish_tf": LaunchConfiguration("icp_publish_tf"),
                }.items(),
            ),
        ]
    )
