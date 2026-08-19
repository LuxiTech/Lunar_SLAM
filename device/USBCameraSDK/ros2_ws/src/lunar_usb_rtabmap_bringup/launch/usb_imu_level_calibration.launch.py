"""Temporarily start H30 and calibrate the USB optical-frame mount angle."""

import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument(
            "imu_params",
            default_value=PathJoinSubstitution([
                FindPackageShare("usb_camera_bringup"),
                "config",
                "h30_imu.yaml",
            ]),
        ),
        DeclareLaunchArgument("imu_topic", default_value="/imu/data"),
        DeclareLaunchArgument("calibration_service", default_value="/sensors/imu/calibrate_level"),
        DeclareLaunchArgument(
            "status_topic", default_value="/sensors/imu/level_calibration_status"
        ),
        DeclareLaunchArgument("calibration_samples", default_value="200"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("camera_frame", default_value="left_camera_optical_frame"),
        DeclareLaunchArgument("camera_x", default_value="0.20"),
        DeclareLaunchArgument("camera_y", default_value="0.044982"),
        DeclareLaunchArgument("camera_z", default_value="0.20"),
        DeclareLaunchArgument("camera_yaw", default_value=str(-math.pi / 2.0)),
        Node(
            package="yesense_std_ros2",
            executable="yesense_node_publisher",
            # Keep the YAML node key so h30_imu.yaml is actually applied.
            name="yesense_pub",
            parameters=[LaunchConfiguration("imu_params")],
            output="screen",
        ),
        # Kalibr run04 T_cam0_imu: left optical frame -> H30 frame.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="imu_level_camera_imu_static_tf",
            arguments=[
                "--x", "0.04391590957628977",
                "--y", "0.0007111012450178807",
                "--z", "-0.03359544596572737",
                "--qx", "0.7081849349309625",
                "--qy", "0.0017267657694375585",
                "--qz", "-0.007348742978277029",
                "--qw", "0.7059866232397363",
                "--frame-id", LaunchConfiguration("camera_frame"),
                "--child-frame-id", "imu_link",
            ],
            output="screen",
        ),
        Node(
            package="luxi_adapter",
            executable="imu_level_calibrator_node",
            name="usb_imu_level_calibrator",
            parameters=[{
                "imu_topic": LaunchConfiguration("imu_topic"),
                "calibration_service": LaunchConfiguration("calibration_service"),
                "status_topic": LaunchConfiguration("status_topic"),
                "calibration_samples": LaunchConfiguration("calibration_samples"),
                "base_frame": LaunchConfiguration("base_frame"),
                "camera_frame": LaunchConfiguration("camera_frame"),
                "camera_x": LaunchConfiguration("camera_x"),
                "camera_y": LaunchConfiguration("camera_y"),
                "camera_z": LaunchConfiguration("camera_z"),
                "camera_yaw": LaunchConfiguration("camera_yaw"),
                # A forward optical frame is nominally RPY=(-90, 0, -90) deg.
                # Tilt limits must be measured from that orientation, not zero.
                "nominal_roll": -math.pi / 2.0,
                "nominal_pitch": 0.0,
                "gravity_tolerance": 1.2,
                "maximum_angular_speed": 0.08,
                "maximum_tilt_degrees": 40.0,
                "auto_start": False,
            }],
            output="screen",
        ),
    ])
