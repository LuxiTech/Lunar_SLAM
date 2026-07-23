import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('hik_bringup')
    default_params = os.path.join(bringup_share, 'config', 'h30_imu.yaml')
    imu_params = LaunchConfiguration('imu_params')

    return LaunchDescription([
        DeclareLaunchArgument(
            'imu_params',
            default_value=default_params,
            description='Path to the WheelTec H30 IMU parameter YAML file',
        ),
        Node(
            package='yesense_std_ros2',
            executable='yesense_node_publisher',
            name='yesense_pub',
            output='screen',
            parameters=[imu_params],
        ),
    ])
