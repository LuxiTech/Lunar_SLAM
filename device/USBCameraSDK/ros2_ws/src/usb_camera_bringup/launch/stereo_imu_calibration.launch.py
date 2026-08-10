import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_params = os.path.join(
        get_package_share_directory('usb_camera_driver'),
        'config',
        'stereo_imu_calibration.yaml',
    )
    imu_params = os.path.join(
        get_package_share_directory('usb_camera_bringup'),
        'config',
        'h30_imu.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument('camera_params', default_value=camera_params),
        DeclareLaunchArgument('imu_params', default_value=imu_params),
        Node(
            package='usb_camera_driver',
            executable='stereo_node',
            name='stereo_node',
            output='screen',
            parameters=[LaunchConfiguration('camera_params')],
        ),
        Node(
            package='yesense_std_ros2',
            executable='yesense_node_publisher',
            name='yesense_pub',
            output='screen',
            parameters=[LaunchConfiguration('imu_params')],
        ),
    ])
