"""Launch RTAB-Map RGB-D localization against the latest or selected database."""

import os
import re

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


MAPS_DIRECTORY = "/home/lunar/project/lunar_slam/maps/rtab_maps"


def _resolve_database_path(context: object) -> list[LogInfo]:
    """Require a real saved database, defaulting to the latest mapNNN.db."""
    requested = LaunchConfiguration("database_path").perform(context).strip()
    if requested:
        database_path = os.path.abspath(os.path.expanduser(requested))
    else:
        candidates = []
        if os.path.isdir(MAPS_DIRECTORY):
            for filename in os.listdir(MAPS_DIRECTORY):
                match = re.fullmatch(r"map(\d+)\.db", filename)
                if match:
                    candidates.append((int(match.group(1)), filename))
        if not candidates:
            raise RuntimeError("No saved rtab_maps/mapNNN.db exists for localization.")
        database_path = os.path.join(MAPS_DIRECTORY, max(candidates)[1])
    if not os.path.isfile(database_path):
        raise RuntimeError(f"Localization database does not exist: {database_path}")
    context.launch_configurations["database_path"] = database_path
    return [LogInfo(msg=f"RTAB-Map localization database: {database_path}")]


def generate_launch_description() -> LaunchDescription:
    mapping_launch = PythonLaunchDescriptionSource([
        FindPackageShare("luxi_rtab_map"), "/launch/rgbd_mapping.launch.py"
    ])
    return LaunchDescription([
        DeclareLaunchArgument(
            "database_path", default_value="",
            description="Saved mapNNN.db; empty selects the latest saved map."),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rtabmap_viz", default_value="false"),
        OpaqueFunction(function=_resolve_database_path),
        IncludeLaunchDescription(
            mapping_launch,
            launch_arguments={
                "database_path": LaunchConfiguration("database_path"),
                "localization": "true",
                "new_map": "false",
                "load_saved_map": "true",
                "rviz": LaunchConfiguration("rviz"),
                "rtabmap_viz": LaunchConfiguration("rtabmap_viz"),
            }.items()),
    ])
