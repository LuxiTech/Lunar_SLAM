from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    bringup_share = get_package_share_directory('usb_camera_bringup')
    default_params = os.path.join(
        get_package_share_directory('usb_camera_driver'),
        'config',
        'stereo_camera.yaml',
    )
    default_rviz = os.path.join(bringup_share, 'rviz', 'stereo_view.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        Node(
            package='usb_camera_driver',
            executable='stereo_node',
            name='stereo_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', default_rviz],
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
