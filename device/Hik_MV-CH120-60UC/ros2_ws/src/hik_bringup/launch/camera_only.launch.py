from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hikrobot_camera_driver',
            executable='stereo_node',
            name='stereo_node',
            output='screen',
        ),
    ])
