import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('hik_bringup'), 'rviz', 'camera_view.rviz')
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='stereo_camera_view',
            output='screen',
            arguments=['-d', rviz_config],
        ),
    ])
