"""Start exactly one configured hardware profile and its project adapter."""

from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _profile_from_config(config_path: str) -> str:
    with Path(config_path).open("r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file) or {}
    parameters = config.get("luxi_adapter", {}).get("ros__parameters", {})
    profile = parameters.get("hardware_profile")
    if not isinstance(profile, str) or not profile:
        raise RuntimeError(
            "The luxi_adapter.ros__parameters.hardware_profile setting is required."
        )
    return profile


def _start_profile(context):
    config_path = LaunchConfiguration("config").perform(context)
    profile = _profile_from_config(config_path)
    package_share = Path(get_package_share_directory("luxi_adapter"))
    profile_launch = package_share / "launch" / f"{profile}.launch.py"
    if not profile_launch.is_file():
        raise RuntimeError(
            f"Unsupported hardware profile '{profile}'. Expected a launch file at {profile_launch}."
        )
    return [
        LogInfo(msg=f"Starting luxi_adapter hardware profile: {profile}"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(profile_launch)),
            launch_arguments={"config": config_path}.items(),
        ),
    ]


def generate_launch_description():
    package_share = get_package_share_directory("luxi_adapter")
    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            default_value=str(Path(package_share) / "config" / "sensor_bringup.yaml"),
            description="Adapter configuration; its hardware_profile selects one profile.",
        ),
        OpaqueFunction(function=_start_profile),
    ])
