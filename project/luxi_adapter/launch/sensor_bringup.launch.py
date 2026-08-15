"""Start exactly one configured hardware profile and its project adapter."""

import os
from pathlib import Path
from typing import NamedTuple

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


class HardwareSelection(NamedTuple):
    """Resolved public hardware profile, launch file and parameter file."""

    profile: str
    launch_path: Path
    config_path: Path


SENSOR_PROCESS_SIGNATURES = (
    "luxi_adapter/lib/luxi_adapter/sensor_adapter_node",
    "imu_filter_madgwick_node",
    "luxi_adapter/lib/luxi_adapter/imu_level_calibrator_node",
)


def find_residual_sensor_processes(proc_root: Path = Path("/proc")) -> list[tuple[int, str]]:
    """Find live project sensor components before launching a second pipeline."""
    residual = []
    current_pid = os.getpid()
    for entry in proc_root.iterdir():
        if not entry.name.isdigit() or int(entry.name) == current_pid:
            continue
        try:
            command = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(
                "utf-8", errors="replace"
            ).strip()
        except OSError:
            continue
        if any(signature in command for signature in SENSOR_PROCESS_SIGNATURES):
            residual.append((int(entry.name), command))
    return sorted(residual)


def _profile_from_config(config_path: str) -> str:
    with Path(config_path).open("r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file) or {}
    parameters = config.get("luxi_adapter", {}).get("ros__parameters", {})
    profile = parameters.get("hardware_profile")
    if not isinstance(profile, str) or not profile:
        raise RuntimeError(
            "The luxi_adapter.ros__parameters.hardware_profile setting is required."
        )
    return profile.strip().lower()


def resolve_hardware_selection(
    hardware: str, config_path: str, package_share: Path
) -> HardwareSelection:
    """Resolve ``hardware:=...`` to one driver launch and one complete config."""
    profile = hardware.strip().lower()
    resolved_config = Path(config_path).expanduser().resolve() if config_path else None
    if not profile or profile == "auto":
        if resolved_config is None:
            resolved_config = package_share / "config" / "sensor_bringup.yaml"
        profile = _profile_from_config(str(resolved_config))
    elif resolved_config is None:
        config_name = (
            "sensor_bringup.yaml"
            if profile == "d435i"
            else f"{profile}_sensor_bringup.yaml"
        )
        resolved_config = package_share / "config" / config_name

    if resolved_config.is_file():
        configured_profile = _profile_from_config(str(resolved_config))
        if configured_profile != profile:
            raise RuntimeError(
                f"Hardware config {resolved_config} selects '{configured_profile}', "
                f"but launch requested '{profile}'."
            )

    launch_path = package_share / "launch" / f"{profile}.launch.py"
    return HardwareSelection(profile, launch_path, resolved_config)


def resolve_hik_stereo_proc_params(requested_path: str, hik_bringup_share: Path) -> Path:
    """Return an explicit file path so an outer empty launch value cannot leak inward."""
    if requested_path.strip():
        return Path(requested_path).expanduser().resolve()
    return hik_bringup_share / "config" / "stereo_proc.yaml"


def _start_profile(context):
    residual = find_residual_sensor_processes()
    if residual:
        details = ", ".join(f"PID {pid}" for pid, _command in residual)
        raise RuntimeError(
            "A sensor pipeline is already running or left residual components: "
            f"{details}. Stop the old pipeline before starting another one."
        )
    package_share = Path(get_package_share_directory("luxi_adapter"))
    selection = resolve_hardware_selection(
        LaunchConfiguration("hardware").perform(context),
        LaunchConfiguration("config").perform(context),
        package_share,
    )
    if not selection.config_path.is_file():
        raise RuntimeError(
            f"Hardware profile config does not exist: {selection.config_path}"
        )
    if not selection.launch_path.is_file():
        raise RuntimeError(
            f"Unsupported hardware profile '{selection.profile}'. "
            f"Expected a launch file at {selection.launch_path}."
        )
    profile_arguments = {"config": str(selection.config_path)}
    if selection.profile == "hik":
        profile_arguments["external_trigger"] = LaunchConfiguration("external_trigger")
        profile_arguments["enable_adapter"] = LaunchConfiguration("enable_adapter")
        profile_arguments["stereo_proc_params"] = str(
            resolve_hik_stereo_proc_params(
                LaunchConfiguration("stereo_proc_params").perform(context),
                Path(get_package_share_directory("hik_bringup")),
            )
        )
    return [
        LogInfo(
            msg=(
                f"Starting luxi_adapter hardware profile: {selection.profile} "
                f"with {selection.config_path}"
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(selection.launch_path)),
            launch_arguments=profile_arguments.items(),
        ),
    ]


def generate_launch_description():
    package_share = get_package_share_directory("luxi_adapter")
    return LaunchDescription([
        DeclareLaunchArgument(
            "hardware",
            default_value="auto",
            description=(
                "Hardware profile to start (for example hik or d435i). "
                "auto preserves config-driven selection."
            ),
        ),
        DeclareLaunchArgument(
            "config",
            default_value="",
            description=(
                "Optional complete profile config override. The selected hardware's "
                "standard config is used when empty."
            ),
        ),
        DeclareLaunchArgument(
            "external_trigger",
            default_value="true",
            description="Hik only: require the rig's external hardware trigger.",
        ),
        DeclareLaunchArgument(
            "stereo_proc_params",
            default_value="",
            description="Hik only: stereo depth parameter file.",
        ),
        DeclareLaunchArgument(
            "enable_adapter",
            default_value="true",
            description="Hik only: republish native images through the generic sensor API.",
        ),
        OpaqueFunction(function=_start_profile),
    ])
