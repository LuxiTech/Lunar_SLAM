"""Start calibrated USB stereo, H30 IMU and the fixed VPI RGB-D frontend."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _reject_removed_arguments(context):
    removed = [
        name for name in (
            "depth_backend", "use_cuda_sgm", "vpi_fallback_to_sgbm"
        )
        if name in context.launch_configurations
    ]
    if removed:
        raise RuntimeError(
            "USB RGB-D is VPI-only; removed launch arguments: "
            + ", ".join(removed)
        )
    return []


def generate_launch_description():
    driver_share = Path(get_package_share_directory("usb_camera_driver"))
    bringup_share = Path(get_package_share_directory("usb_camera_bringup"))
    return LaunchDescription([
        OpaqueFunction(function=_reject_removed_arguments),
        DeclareLaunchArgument(
            "camera_params",
            default_value=str(driver_share / "config" / "stereo_camera.yaml"),
        ),
        DeclareLaunchArgument(
            "depth_params",
            default_value=str(driver_share / "config" / "stereo_depth.yaml"),
        ),
        DeclareLaunchArgument(
            "calibration_file",
            default_value=str(driver_share / "config" / "stereo_opencv.yaml"),
        ),
        DeclareLaunchArgument(
            "imu_params",
            default_value=str(bringup_share / "config" / "h30_imu.yaml"),
        ),
        DeclareLaunchArgument(
            "start_imu",
            default_value="true",
            description="Start the H30 serial driver and its camera-to-IMU transform.",
        ),
        DeclareLaunchArgument(
            "point_cloud_max_depth_m",
            default_value="3.0",
            description="Maximum depth included in the live USB point cloud.",
        ),
        Node(
            package="usb_camera_driver",
            executable="stereo_node",
            name="stereo_node",
            parameters=[
                LaunchConfiguration("camera_params"),
                {"output_encoding": "jpeg"},
            ],
            output="screen",
            on_exit=[
                EmitEvent(
                    event=Shutdown(reason="USB stereo capture exited")
                ),
            ],
        ),
        Node(
            package="usb_camera_driver",
            executable="stereo_depth_node",
            name="usb_stereo_depth_node",
            parameters=[
                LaunchConfiguration("depth_params"),
                {
                    "calibration_file": LaunchConfiguration("calibration_file"),
                    "point_cloud_max_depth_m": LaunchConfiguration(
                        "point_cloud_max_depth_m"
                    ),
                },
            ],
            output="screen",
        ),
        Node(
            package="yesense_std_ros2",
            executable="yesense_node_publisher",
            name="yesense_pub",
            parameters=[LaunchConfiguration("imu_params")],
            condition=IfCondition(LaunchConfiguration("start_imu")),
            output="screen",
        ),
        # Kalibr run04 T_cam0_imu: left optical frame -> IMU frame.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="usb_camera_imu_static_tf",
            condition=IfCondition(LaunchConfiguration("start_imu")),
            arguments=[
                "--x", "0.04391590957628977",
                "--y", "0.0007111012450178807",
                "--z", "-0.03359544596572737",
                "--qx", "0.7081849349309625",
                "--qy", "0.0017267657694375585",
                "--qz", "-0.007348742978277029",
                "--qw", "0.7059866232397363",
                "--frame-id", "left_camera_optical_frame",
                "--child-frame-id", "imu_link",
            ],
            output="screen",
        ),
    ])
