"""Start HLoc coarse localization and feed accepted poses to Open3D ICP."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


WORKSPACE = Path("/home/lunar/project/lunar_slam")


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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        WORKSPACE
                        / "install/luxi_hloc/share/luxi_hloc/launch/hloc_localization.launch.py"
                    )
                ),
                launch_arguments={
                    "map_directory": LaunchConfiguration("map_directory"),
                    "device": LaunchConfiguration("device"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(
                        WORKSPACE
                        / "install/luxi_location/share/luxi_location/launch/"
                        "icp_localization.launch.py"
                    )
                ),
                launch_arguments={
                    "map_path": LaunchConfiguration("cloud_path"),
                    "initial_pose_topic": "/luxi_hloc/coarse_pose",
                    "initial_pose_max_variance": "0.5",
                    "publish_tf": LaunchConfiguration("icp_publish_tf"),
                }.items(),
            ),
        ]
    )
