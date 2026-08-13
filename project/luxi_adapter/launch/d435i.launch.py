"""D435i hardware profile: vendor driver, canonical relay, IMU filter and static TF."""

import os
import shlex
from pathlib import Path

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def _settings(config_path: str) -> dict:
    with Path(config_path).open("r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file) or {}
    settings = config.get("luxi_adapter", {}).get("ros__parameters", {})
    if not isinstance(settings, dict):
        raise RuntimeError("luxi_adapter.ros__parameters must be a mapping.")
    return settings


def _required(settings: dict, name: str):
    value = settings.get(name)
    if value is None or value == "":
        raise RuntimeError(f"The D435i profile requires config setting '{name}'.")
    return value


def _as_bool(value, name: str) -> bool:
    if isinstance(value, bool):
        return value
    raise RuntimeError(f"The D435i config setting '{name}' must be true or false.")


def _workspace_root() -> Path:
    configured = os.environ.get("LUXI_WORKSPACE_ROOT", "").strip()
    if configured:
        return Path(configured).expanduser().resolve()
    for start in (Path(__file__).resolve(), Path.cwd().resolve()):
        for candidate in (start, *start.parents):
            if (candidate / "project").is_dir() and (candidate / "device").is_dir():
                return candidate
    raise RuntimeError("Cannot locate lunar_slam; set LUXI_WORKSPACE_ROOT")


def _launch_d435i(context):
    config_path = LaunchConfiguration("config").perform(context)
    settings = _settings(config_path)
    configured_setup = str(settings.get("d435i_setup", "")).strip()
    setup_path = Path(
        configured_setup
        or _workspace_root() / "device" / "D435i" / "ros2_ws" / "install" / "setup.bash"
    ).expanduser()
    if not setup_path.is_file():
        raise RuntimeError(f"D435i setup script does not exist: {setup_path}")

    enable_imu = _as_bool(settings.get("enable_imu", True), "enable_imu")
    rmw_implementation = str(settings.get("rmw_implementation", "rmw_cyclonedds_cpp"))
    ros_domain_id = str(settings.get("ros_domain_id", 42))
    driver_command = shlex.join([
        "ros2", "launch", "lunar_realsense_bringup", "d435i.launch.py",
        "rviz:=false",
        f"enable_imu:={'true' if enable_imu else 'false'}",
        "enable_pointcloud:=false",
        f"unite_imu_method:={settings.get('d435i_unite_imu_method', 2)}",
        f"color_profile:={settings.get('d435i_color_profile', '640,480,30')}",
        f"depth_profile:={settings.get('d435i_depth_profile', '640,480,30')}",
        "initial_reset:=" + (
            "true" if _as_bool(settings.get("d435i_initial_reset", False), "d435i_initial_reset")
            else "false"
        ),
    ])
    process_script = "; ".join([
        "set -e",
        f"source {shlex.quote(str(setup_path))}",
        "export ROS_LOCALHOST_ONLY=0",
        f"export ROS_DOMAIN_ID={shlex.quote(ros_domain_id)}",
        f"export RMW_IMPLEMENTATION={shlex.quote(rmw_implementation)}",
        f"exec {driver_command}",
    ])
    environment = {
        "ROS_LOCALHOST_ONLY": "0",
        "ROS_DOMAIN_ID": ros_domain_id,
        "RMW_IMPLEMENTATION": rmw_implementation,
    }
    actions = [
        LogInfo(msg=f"Launching D435i driver using {setup_path}"),
        ExecuteProcess(
            cmd=["/bin/bash", "-c", process_script],
            additional_env=environment,
            output="screen",
        ),
        Node(
            package="luxi_adapter",
            executable="sensor_adapter_node",
            name="luxi_adapter",
            parameters=[config_path],
            additional_env=environment,
            output="screen",
        ),
    ]
    if enable_imu:
        filter_command = shlex.join([
            "ros2", "run", "imu_filter_madgwick", "imu_filter_madgwick_node",
            "--ros-args", "-r", "__node:=sensor_imu_filter",
            "--params-file", config_path,
            "-r", "imu/data_raw:="
            + str(settings.get("imu_output_topic", "/sensors/imu/data_raw")),
            "-r", "imu/data:=/sensors/imu/data",
        ])
        filter_script = "; ".join([
            "set -e",
            f"source {shlex.quote(str(setup_path))}",
            "export ROS_LOCALHOST_ONLY=0",
            f"export ROS_DOMAIN_ID={shlex.quote(ros_domain_id)}",
            f"export RMW_IMPLEMENTATION={shlex.quote(rmw_implementation)}",
            f"exec {filter_command}",
        ])
        actions.append(ExecuteProcess(
            cmd=["/bin/bash", "-c", filter_script],
            additional_env=environment,
            output="screen",
        ))
    auto_level = _as_bool(settings.get("auto_level_imu", True), "auto_level_imu")
    if auto_level and enable_imu:
        actions.append(Node(
            package="luxi_adapter",
            executable="imu_level_calibrator_node",
            name="imu_level_calibrator",
            parameters=[{
                "imu_topic": str(settings.get("imu_output_topic", "/sensors/imu/data_raw")),
                "base_frame": str(settings.get("base_frame", "base_link")),
                "camera_frame": str(settings.get("camera_frame", "camera_link")),
                "camera_x": float(settings.get("camera_x", 0.0)),
                "camera_y": float(settings.get("camera_y", 0.0)),
                "camera_z": float(settings.get("camera_z", 0.0)),
                "camera_yaw": float(settings.get("camera_yaw", 0.0)),
                "calibration_samples": int(settings.get("auto_level_samples", 200)),
                "gravity_tolerance": float(settings.get("auto_level_gravity_tolerance", 0.8)),
                "maximum_angular_speed": float(
                    settings.get("auto_level_maximum_angular_speed", 0.05)
                ),
                "maximum_tilt_degrees": float(
                    settings.get("auto_level_maximum_tilt_degrees", 40.0)
                ),
                "auto_start": bool(settings.get("auto_level_start_automatically", False)),
                "calibration_service": str(settings.get(
                    "imu_level_calibration_service", "/sensors/imu/calibrate_level"
                )),
                "status_topic": str(settings.get(
                    "imu_level_status_topic", "/sensors/imu/level_calibration_status"
                )),
            }],
            additional_env=environment,
            output="screen",
        ))
    else:
        actions.append(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_sensor_tf",
            arguments=[
                "--x", str(settings.get("camera_x", 0.0)),
                "--y", str(settings.get("camera_y", 0.0)),
                "--z", str(settings.get("camera_z", 0.0)),
                "--roll", str(settings.get("camera_roll", 0.0)),
                "--pitch", str(settings.get("camera_pitch", 0.0)),
                "--yaw", str(settings.get("camera_yaw", 0.0)),
                "--frame-id", str(settings.get("base_frame", "base_link")),
                "--child-frame-id", str(settings.get("camera_frame", "camera_link")),
            ],
            additional_env=environment,
            output="screen",
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            description="Adapter configuration selected by sensor_bringup.launch.py.",
        ),
        OpaqueFunction(function=_launch_d435i),
    ])
