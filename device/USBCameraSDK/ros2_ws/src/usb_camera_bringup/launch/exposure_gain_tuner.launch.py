"""Start the USB stereo camera and an interactive exposure/gain window."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    driver_share = Path(get_package_share_directory("usb_camera_driver"))
    return LaunchDescription([
        DeclareLaunchArgument(
            "display",
            default_value=":0",
            description="X11 display used by the OpenCV GTK tuning window.",
        ),
        SetEnvironmentVariable(
            "DISPLAY", LaunchConfiguration("display")
        ),
        DeclareLaunchArgument(
            "start_camera",
            default_value="true",
            description=(
                "Start stereo_node. Set false when RTAB-Map or another launch "
                "already owns the cameras."
            ),
        ),
        DeclareLaunchArgument(
            "camera_params",
            default_value=str(driver_share / "config" / "stereo_camera.yaml"),
        ),
        DeclareLaunchArgument(
            "tuner_params",
            default_value=str(
                driver_share / "config" / "opencv_exposure_gain_tuner.yaml"
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
            condition=IfCondition(LaunchConfiguration("start_camera")),
            output="screen",
        ),
        TimerAction(
            period=1.0,
            actions=[Node(
                package="usb_camera_driver",
                executable="opencv_exposure_gain_tuner",
                name="opencv_exposure_gain_tuner",
                parameters=[LaunchConfiguration("tuner_params")],
                output="screen",
            )],
        ),
    ])
