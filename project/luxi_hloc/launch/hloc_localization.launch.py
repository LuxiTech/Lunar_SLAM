"""Start HLoc coarse localization using a selected or latest HLoc map."""

from pathlib import Path
import re

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


WORKSPACE = Path("/home/lunar/project/lunar_slam")
MAP_PATTERN = re.compile(r"^map(\d+)$")


def _latest_map() -> Path:
    candidates = []
    root = WORKSPACE / "maps" / "hloc_maps"
    if root.is_dir():
        for directory in root.iterdir():
            match = MAP_PATTERN.fullmatch(directory.name)
            if match and (directory / "metadata.yaml").is_file():
                candidates.append((int(match.group(1)), directory))
    return max(candidates)[1] if candidates else root / "map001"


def _start_node(context):
    map_directory = Path(
        LaunchConfiguration("map_directory").perform(context)
    ).expanduser().resolve()
    if not (map_directory / "metadata.yaml").is_file():
        raise RuntimeError(f"HLoc map is incomplete: {map_directory}")
    return [
        LogInfo(msg=f"HLoc map: {map_directory}"),
        Node(
            package="luxi_hloc",
            executable="hloc_localizer_node",
            name="luxi_hloc_localizer",
            output="screen",
            parameters=[
                LaunchConfiguration("config_file"),
                {
                    "map_directory": str(map_directory),
                    "device": LaunchConfiguration("device"),
                    "query_period": ParameterValue(
                        LaunchConfiguration("query_period"),
                        value_type=float,
                    ),
                },
            ],
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("map_directory", default_value=str(_latest_map())),
            DeclareLaunchArgument(
                "config_file",
                default_value=str(
                    WORKSPACE
                    / "install/luxi_hloc/share/luxi_hloc/config/hloc_localization.yaml"
                ),
            ),
            DeclareLaunchArgument("device", default_value="cuda"),
            DeclareLaunchArgument("query_period", default_value="1.0"),
            OpaqueFunction(function=_start_node),
        ]
    )
