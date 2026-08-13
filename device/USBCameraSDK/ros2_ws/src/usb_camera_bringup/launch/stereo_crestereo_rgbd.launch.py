"""Start calibrated USB stereo, CREStereo RGB-D and H30 IMU."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    driver_share = Path(get_package_share_directory("usb_camera_driver"))
    bringup_share = Path(get_package_share_directory("usb_camera_bringup"))
    return LaunchDescription([
        DeclareLaunchArgument(
            "camera_params",
            default_value=str(driver_share / "config" / "stereo_camera.yaml"),
        ),
        DeclareLaunchArgument(
            "crestereo_params",
            default_value=str(driver_share / "config" / "crestereo_depth.yaml"),
        ),
        DeclareLaunchArgument(
            "calibration_file",
            default_value=str(driver_share / "config" / "stereo_opencv.yaml"),
        ),
        DeclareLaunchArgument(
            "model_path",
            default_value=str(
                driver_share / "models" / "crestereo_init_iter2_180x320_fp16.onnx"
            ),
        ),
        DeclareLaunchArgument("execution_provider", default_value="cuda"),
        DeclareLaunchArgument(
            "imu_params", default_value=str(bringup_share / "config" / "h30_imu.yaml")
        ),
        DeclareLaunchArgument("start_imu", default_value="true"),
        DeclareLaunchArgument("point_cloud_max_depth_m", default_value="3.0"),
        DeclareLaunchArgument("point_cloud_far_sparse_start_m", default_value="0.0"),
        DeclareLaunchArgument("point_cloud_medium_max_depth_m", default_value="0.0"),
        DeclareLaunchArgument("point_cloud_medium_sparse_pixel_step", default_value="1"),
        DeclareLaunchArgument("point_cloud_far_sparse_pixel_step", default_value="1"),
        Node(
            package="usb_camera_driver",
            executable="stereo_node",
            name="stereo_node",
            parameters=[LaunchConfiguration("camera_params"), {"output_encoding": "jpeg"}],
            output="screen",
            on_exit=[EmitEvent(event=Shutdown(reason="USB stereo capture exited"))],
        ),
        Node(
            package="usb_camera_driver",
            executable="crestereo_depth_node",
            name="usb_crestereo_depth_node",
            parameters=[
                LaunchConfiguration("crestereo_params"),
                {
                    "calibration_file": LaunchConfiguration("calibration_file"),
                    "model_path": LaunchConfiguration("model_path"),
                    "execution_provider": LaunchConfiguration("execution_provider"),
                    "point_cloud_max_depth_m": LaunchConfiguration(
                        "point_cloud_max_depth_m"
                    ),
                    "point_cloud_far_sparse_start_m": LaunchConfiguration(
                        "point_cloud_far_sparse_start_m"
                    ),
                    "point_cloud_medium_max_depth_m": LaunchConfiguration(
                        "point_cloud_medium_max_depth_m"
                    ),
                    "point_cloud_medium_sparse_pixel_step": LaunchConfiguration(
                        "point_cloud_medium_sparse_pixel_step"
                    ),
                    "point_cloud_far_sparse_pixel_step": LaunchConfiguration(
                        "point_cloud_far_sparse_pixel_step"
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
