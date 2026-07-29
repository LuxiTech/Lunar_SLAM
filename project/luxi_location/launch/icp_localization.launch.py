"""Start planar Open3D ICP localization on the latest exported RTAB-Map cloud."""

from pathlib import Path
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


WORKSPACE = Path("/home/lunar/project/lunar_slam")
MAP_ID = re.compile(r"^map(\d+)_octomap$")


def _latest_cloud() -> Path:
    candidates = []
    for directory in (WORKSPACE / "maps" / "octo_maps").glob("map*_octomap"):
        matched = MAP_ID.fullmatch(directory.name)
        if not matched:
            continue
        clouds = list(directory.glob("*_cloud.ply"))
        if clouds:
            candidates.append((int(matched.group(1)), max(clouds, key=lambda path: path.stat().st_mtime)))
    if not candidates:
        return WORKSPACE / "maps" / "octo_maps" / "map001_octomap" / "map001_cloud.ply"
    return max(candidates, key=lambda item: item[0])[1]


def _localization_node(context):
    map_path = Path(LaunchConfiguration("map_path").perform(context)).expanduser().resolve()
    if not map_path.is_file():
        raise RuntimeError(f"ICP point cloud map does not exist: {map_path}")
    return [
        Node(
            package="luxi_location",
            executable="icp_localization_node",
            name="luxi_icp_localization",
            output="screen",
            parameters=[
                LaunchConfiguration("config_file"),
                {
                    "map_path": str(map_path),
                    "initial_pose_topic": LaunchConfiguration("initial_pose_topic"),
                    "initial_pose_max_variance": ParameterValue(
                        LaunchConfiguration("initial_pose_max_variance"),
                        value_type=float),
                    "publish_tf": ParameterValue(
                        LaunchConfiguration("publish_tf"), value_type=bool),
                },
            ],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("luxi_location"))
    return LaunchDescription([
        DeclareLaunchArgument(
            "map_path",
            default_value=str(_latest_cloud()),
            description="Saved RTAB-Map colored PLY; default selects the highest mapNNN export."),
        DeclareLaunchArgument(
            "config_file",
            default_value=str(package_share / "config" / "icp_localization.yaml")),
        DeclareLaunchArgument("initial_pose_topic", default_value="/initialpose"),
        DeclareLaunchArgument("initial_pose_max_variance", default_value="0.0"),
        DeclareLaunchArgument("publish_tf", default_value="true"),
        OpaqueFunction(function=_localization_node),
    ])
