"""ZED X hardware profile: GMSL2 driver, canonical RGB-D/IMU and mounting TF."""

import os
import pwd
import shlex
from pathlib import Path

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _settings(config_path: str) -> dict:
    with Path(config_path).open("r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file) or {}
    settings = config.get("luxi_adapter", {}).get("ros__parameters", {})
    if not isinstance(settings, dict):
        raise RuntimeError("luxi_adapter.ros__parameters must be a mapping.")
    return settings


def _as_bool(value, name: str) -> bool:
    if isinstance(value, bool):
        return value
    raise RuntimeError(f"The ZED X config setting '{name}' must be true or false.")


def _workspace_root() -> Path:
    configured = os.environ.get("LUXI_WORKSPACE_ROOT", "").strip()
    if configured:
        return Path(configured).expanduser().resolve()
    for start in (Path(__file__).resolve(), Path.cwd().resolve()):
        for candidate in (start, *start.parents):
            if (candidate / "project").is_dir() and (candidate / "device").is_dir():
                return candidate
    raise RuntimeError("Cannot locate lunar_slam; set LUXI_WORKSPACE_ROOT")


def _launch_zedx(context):
    config_path = LaunchConfiguration("config").perform(context)
    settings = _settings(config_path)
    workspace_root = _workspace_root()
    device_root = workspace_root / "device" / "sterellab_ZEDX"
    setup_path = Path(
        str(settings.get("zedx_setup", "")).strip()
        or device_root / "ros2_ws" / "install" / "setup.bash"
    ).expanduser()
    runtime_prepare = Path(
        str(settings.get("zedx_runtime_prepare", "")).strip()
        or device_root / "scripts" / "prepare_container_runtime.sh"
    ).expanduser()
    if not setup_path.is_file():
        raise RuntimeError(f"ZED X setup script does not exist: {setup_path}")
    if not runtime_prepare.is_file():
        raise RuntimeError(f"ZED X runtime preparation script does not exist: {runtime_prepare}")

    enable_imu = _as_bool(settings.get("enable_imu", True), "enable_imu")
    publish_point_cloud = _as_bool(
        settings.get("zedx_publish_point_cloud", False), "zedx_publish_point_cloud"
    )
    enable_positional_tracking = _as_bool(
        settings.get("zedx_enable_positional_tracking", False),
        "zedx_enable_positional_tracking",
    )
    rmw_implementation = str(settings.get("rmw_implementation", "rmw_fastrtps_cpp"))
    ros_domain_id = str(settings.get("ros_domain_id", 42))
    camera_name = str(settings.get("zedx_camera_name", "zed"))
    runtime_user = str(settings.get("zedx_runtime_user", "lunar"))
    runtime_uid = str(pwd.getpwnam(runtime_user).pw_uid)
    display = os.environ.get("DISPLAY", ":0")
    if not display or not Path(f"/tmp/.X11-unix/X{display.lstrip(':').split('.')[0]}").exists():
        display = ":0"
    xauthority = os.environ.get("XAUTHORITY", "/tmp/.docker.xauth")
    xdg_runtime_dir = f"/tmp/zedx-runtime-{runtime_uid}"

    def bool_text(value):
        return "true" if value else "false"

    parameter_overrides = ";".join([
        f"general.grab_resolution:={settings.get('zedx_grab_resolution', 'HD1200')}",
        f"general.grab_frame_rate:={settings.get('zedx_grab_frame_rate', 30)}",
        f"general.pub_resolution:={settings.get('zedx_publish_resolution', 'CUSTOM')}",
        "general.pub_downscale_factor:="
        + str(settings.get("zedx_publish_downscale_factor", 2.0)),
        f"general.pub_frame_rate:={settings.get('zedx_publish_frame_rate', 10.0)}",
        f"depth.depth_mode:={settings.get('zedx_depth_mode', 'NEURAL_LIGHT')}",
        "depth.depth_stabilization:="
        + str(settings.get("zedx_depth_stabilization", 0)),
        "depth.depth_confidence:="
        + str(settings.get("zedx_depth_confidence", 80)),
        "depth.depth_texture_conf:="
        + str(settings.get("zedx_depth_texture_confidence", 100)),
        f"depth.publish_point_cloud:={bool_text(publish_point_cloud)}",
        f"pos_tracking.pos_tracking_enabled:={bool_text(enable_positional_tracking)}",
        "pos_tracking.pos_tracking_mode:="
        + str(settings.get("zedx_positional_tracking_mode", "AUTO")),
        "pos_tracking.imu_fusion:="
        + bool_text(_as_bool(
            settings.get("zedx_positional_tracking_imu_fusion", True),
            "zedx_positional_tracking_imu_fusion",
        )),
        "pos_tracking.area_memory:="
        + bool_text(_as_bool(
            settings.get("zedx_positional_tracking_area_memory", False),
            "zedx_positional_tracking_area_memory",
        )),
        "pos_tracking.reset_odom_with_loop_closure:="
        + bool_text(_as_bool(
            settings.get("zedx_reset_odom_with_loop_closure", False),
            "zedx_reset_odom_with_loop_closure",
        )),
        "pos_tracking.two_d_mode:="
        + bool_text(_as_bool(
            settings.get("zedx_positional_tracking_two_d_mode", True),
            "zedx_positional_tracking_two_d_mode",
        )),
        "pos_tracking.set_gravity_as_origin:="
        + bool_text(_as_bool(
            settings.get("zedx_set_gravity_as_origin", True),
            "zedx_set_gravity_as_origin",
        )),
        f"pos_tracking.publish_odom_pose:={bool_text(enable_positional_tracking)}",
        "sensors.publish_imu:=false",
        f"sensors.publish_imu_raw:={bool_text(enable_imu)}",
    ])
    driver_command = shlex.join([
        "ros2", "launch", "zed_wrapper", "zed_camera.launch.py",
        "camera_model:=zedx",
        f"camera_name:={camera_name}",
        f"serial_number:={settings.get('zedx_serial_number', 45570700)}",
        "enable_ipc:=false",
        "publish_urdf:=true",
        f"publish_tf:={bool_text(enable_positional_tracking)}",
        "publish_map_tf:=false",
        f"publish_imu_tf:={bool_text(enable_imu)}",
        f"param_overrides:={parameter_overrides}",
        "node_log_type:=screen",
    ])
    process_script = "; ".join([
        "set -e",
        f"source {shlex.quote(str(setup_path))}",
        f"{shlex.quote(str(runtime_prepare))}",
        f"mkdir -p {shlex.quote(xdg_runtime_dir)}",
        f"chmod 700 {shlex.quote(xdg_runtime_dir)}",
        f"exec {driver_command}",
    ])
    driver_environment = {
        "DISPLAY": display,
        "XAUTHORITY": xauthority,
        "XDG_RUNTIME_DIR": xdg_runtime_dir,
        "ROS_LOCALHOST_ONLY": "0",
        "ROS_DOMAIN_ID": ros_domain_id,
        "RMW_IMPLEMENTATION": rmw_implementation,
    }
    sudo_environment = [f"{key}={value}" for key, value in driver_environment.items()]
    actions = [
        LogInfo(
            msg=(
                f"Launching ZED X S/N {settings.get('zedx_serial_number', 45570700)} "
                f"using {setup_path}"
            )
        ),
        ExecuteProcess(
            cmd=[
                "sudo", "-n", "-u", runtime_user, "-H", "env", *sudo_environment,
                "/bin/bash", "-c", process_script,
            ],
            output="screen",
        ),
        Node(
            package="luxi_adapter",
            executable="sensor_adapter_node",
            name="luxi_adapter",
            parameters=[config_path],
            additional_env=driver_environment,
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="zedx_base_to_camera_tf",
            arguments=[
                "--x", str(settings.get("camera_x", 0.0)),
                "--y", str(settings.get("camera_y", 0.0)),
                "--z", str(settings.get("camera_z", 0.0)),
                "--roll", str(settings.get("camera_roll", 0.0)),
                "--pitch", str(settings.get("camera_pitch", 0.0)),
                "--yaw", str(settings.get("camera_yaw", 0.0)),
                "--frame-id", str(settings.get("base_frame", "base_link")),
                "--child-frame-id", str(settings.get("camera_frame", "zed_camera_link")),
            ],
            additional_env=driver_environment,
            output="screen",
        ),
    ]
    if enable_imu:
        actions.append(Node(
            package="imu_filter_madgwick",
            executable="imu_filter_madgwick_node",
            name="sensor_imu_filter",
            parameters=[config_path],
            remappings=[
                ("imu/data_raw", str(settings.get("imu_output_topic", "/sensors/imu/data_raw"))),
                ("imu/data", "/sensors/imu/data"),
            ],
            additional_env=driver_environment,
            output="screen",
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "config",
            description="ZED X adapter configuration selected by sensor_bringup.launch.py.",
        ),
        OpaqueFunction(function=_launch_zedx),
    ])
