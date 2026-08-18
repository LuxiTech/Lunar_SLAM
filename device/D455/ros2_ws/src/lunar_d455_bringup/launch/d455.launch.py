"""Launch a D455 with wide aligned RGB-D, TF and optional combined IMU."""

import os
import platform
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _workspace_root() -> Path:
    configured = os.environ.get("LUXI_WORKSPACE_ROOT", "").strip()
    if configured:
        return Path(configured).expanduser().resolve()
    for start in (Path(__file__).resolve(), Path.cwd().resolve()):
        for candidate in (start, *start.parents):
            if (candidate / "project").is_dir() and (candidate / "device").is_dir():
                return candidate
    return Path("/home/lunar/project/lunar_slam")


def generate_launch_description() -> LaunchDescription:
    package_share = Path(get_package_share_directory("lunar_d455_bringup"))
    default_config = package_share / "config" / "d455.yaml"
    rsusb_lib_dir = (
        _workspace_root()
        / "device"
        / "D455"
        / "ros2_ws"
        / "3parts"
        / "librealsense"
        / "install-rsusb"
        / "lib"
    )
    library_path = os.pathsep.join(
        str(path)
        for path in (rsusb_lib_dir, os.environ.get("LD_LIBRARY_PATH", ""))
        if str(path)
    )
    pointcloud_filter = (
        "pointcloud__neon_"
        if platform.machine() in {"aarch64", "arm64"}
        else "pointcloud"
    )

    camera = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="camera",
        name="camera",
        parameters=[
            LaunchConfiguration("config"),
            {
                "enable_gyro": ParameterValue(
                    LaunchConfiguration("enable_imu"), value_type=bool
                ),
                "enable_accel": ParameterValue(
                    LaunchConfiguration("enable_imu"), value_type=bool
                ),
                "unite_imu_method": ParameterValue(
                    LaunchConfiguration("unite_imu_method"), value_type=int
                ),
                "rgb_camera.color_profile": LaunchConfiguration("color_profile"),
                "depth_module.depth_profile": LaunchConfiguration("depth_profile"),
                "initial_reset": ParameterValue(
                    LaunchConfiguration("initial_reset"), value_type=bool
                ),
                f"{pointcloud_filter}.enable": ParameterValue(
                    LaunchConfiguration("enable_pointcloud"), value_type=bool
                ),
            },
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        additional_env={"LD_LIBRARY_PATH": library_path},
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=str(default_config)),
        DeclareLaunchArgument("color_profile", default_value="848,480,15"),
        DeclareLaunchArgument("depth_profile", default_value="848,480,15"),
        DeclareLaunchArgument("initial_reset", default_value="false"),
        DeclareLaunchArgument("enable_imu", default_value="true"),
        DeclareLaunchArgument("enable_pointcloud", default_value="false"),
        DeclareLaunchArgument("unite_imu_method", default_value="2"),
        DeclareLaunchArgument("log_level", default_value="info"),
        camera,
    ])
