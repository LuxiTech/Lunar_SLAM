"""Launch Intel RealSense D435i with RGB-D, point cloud, TF, optional IMU, and RViz."""

import os
import platform

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    profile_default = "640,480,30"
    rsusb_lib_dir = (
        "/home/lunar/project/lunar_slam/device/D435i/ros2_ws/3parts/"
        "librealsense/install-rsusb/lib"
    )
    library_path = os.pathsep.join(
        path for path in (rsusb_lib_dir, os.environ.get("LD_LIBRARY_PATH", "")) if path
    )
    pointcloud_filter = "pointcloud__neon_" if platform.machine() in {"aarch64", "arm64"} else "pointcloud"
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("lunar_realsense_bringup"), "rviz", "d435i.rviz"]
    )

    realsense = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="camera",
        name="camera",
        parameters=[
            {
            "camera_namespace": "camera",
            "camera_name": "camera",
            "device_type": "d435i",
            "enable_color": True,
            "enable_depth": True,
            # RGB-D SLAM does not consume raw infrared images. Disable both
            # streams to reduce USB bandwidth and avoid unnecessary sensor load.
            "enable_infra1": False,
            "enable_infra2": False,
            # D435i mapping uses the motion streams to keep orientation stable
            # during camera rotation. The launch argument can still disable
            # IMU explicitly when a container has no HID device access.
            "enable_gyro": ParameterValue(
                LaunchConfiguration("enable_imu"), value_type=bool
            ),
            "enable_accel": ParameterValue(
                LaunchConfiguration("enable_imu"), value_type=bool
            ),
            "unite_imu_method": ParameterValue(
                LaunchConfiguration("unite_imu_method"), value_type=int
            ),
            "enable_sync": True,
            "align_depth.enable": True,
            # Smooth depth noise before alignment feeds RGB-D odometry. Keep
            # hole filling disabled so moving the camera cannot reuse stale
            # depth values from previous frames.
            "spatial_filter.enable": True,
            "temporal_filter.enable": True,
            # D435i accepts 0 (disabled), 1 (50 Hz), or 2 (60 Hz). The
            # driver default of 3 is rejected by this firmware.
            "rgb_camera.power_line_frequency": 2,
            f"{pointcloud_filter}.enable": ParameterValue(
                LaunchConfiguration("enable_pointcloud"), value_type=bool
            ),
            f"{pointcloud_filter}.stream_filter": 2,
            f"{pointcloud_filter}.stream_index_filter": 0,
            f"{pointcloud_filter}.ordered_pc": False,
            f"{pointcloud_filter}.allow_no_texture_points": False,
            "publish_tf": True,
            # Camera extrinsics are constant. Publish them on /tf_static so
            # RGB-D odometry cannot lose frames when a dynamic TF publisher
            # briefly falls behind under localization load.
            "tf_publish_rate": 0.0,
            "rgb_camera.color_profile": LaunchConfiguration("color_profile"),
            "depth_module.depth_profile": LaunchConfiguration("depth_profile"),
            "initial_reset": ParameterValue(LaunchConfiguration("initial_reset"), value_type=bool),
            }
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
        additional_env={"LD_LIBRARY_PATH": library_path},
        output="screen",
        emulate_tty=True,
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("color_profile", default_value=profile_default),
            DeclareLaunchArgument("depth_profile", default_value=profile_default),
            DeclareLaunchArgument("initial_reset", default_value="false"),
            DeclareLaunchArgument("enable_imu", default_value="true"),
            DeclareLaunchArgument("enable_pointcloud", default_value="true"),
            DeclareLaunchArgument("unite_imu_method", default_value="2"),
            DeclareLaunchArgument("log_level", default_value="info"),
            realsense,
            rviz,
        ]
    )
