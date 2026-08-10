"""Start calibrated USB stereo, H30 IMU and the CPU RGB-D frontend."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
            "depth_backend",
            default_value="opencv_cuda_sgm",
            description=(
                "Stereo backend: opencv_cuda_sgm, vpi_ofa_pva_vic, "
                "vpi_cuda or cpu_sgbm."
            ),
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
        ),
        Node(
            package="usb_camera_driver",
            executable="stereo_depth_node",
            name="usb_stereo_depth_node",
            parameters=[
                LaunchConfiguration("depth_params"),
                {
                    "calibration_file": LaunchConfiguration("calibration_file"),
                    "depth_backend": LaunchConfiguration("depth_backend"),
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
        # Kalibr run03_left T_cam0_imu: left optical frame -> IMU frame.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="usb_camera_imu_static_tf",
            condition=IfCondition(LaunchConfiguration("start_imu")),
            arguments=[
                "--x", "0.045539743687302736",
                "--y", "0.003107102982720423",
                "--z", "-0.034259962926458117",
                "--qx", "0.7158095538699986",
                "--qy", "-0.002576409241311148",
                "--qz", "0.001520959914006512",
                "--qw", "0.6982891459737829",
                "--frame-id", "left_camera_optical_frame",
                "--child-frame-id", "imu_link",
            ],
            output="screen",
        ),
    ])
