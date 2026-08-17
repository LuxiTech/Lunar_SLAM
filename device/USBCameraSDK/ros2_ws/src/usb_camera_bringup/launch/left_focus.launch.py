import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    driver_share = get_package_share_directory('usb_camera_driver')
    default_camera_params = os.path.join(
        driver_share, 'config', 'stereo_calibration.yaml')
    default_viewer_params = os.path.join(
        driver_share, 'config', 'opencv_focus_viewer.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_params_file', default_value=default_camera_params),
        DeclareLaunchArgument(
            'viewer_params_file', default_value=default_viewer_params),
        Node(
            package='usb_camera_driver',
            executable='stereo_node',
            name='stereo_node',
            output='screen',
            parameters=[LaunchConfiguration('camera_params_file')],
        ),
        TimerAction(
            period=1.0,
            actions=[Node(
                package='usb_camera_driver',
                executable='opencv_focus_viewer',
                name='opencv_focus_viewer',
                output='screen',
                parameters=[LaunchConfiguration('viewer_params_file')],
            )],
        ),
    ])
