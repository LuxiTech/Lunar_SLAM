import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('hik_bringup')
    default_camera_params = os.path.join(bringup_share, 'config', 'camera_params.yaml')
    default_stereo_proc_params = os.path.join(bringup_share, 'config', 'stereo_proc.yaml')

    camera_params = LaunchConfiguration('camera_params')
    stereo_proc_params = LaunchConfiguration('stereo_proc_params')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_params',
            default_value=default_camera_params,
            description='Path to the camera node parameter YAML file',
        ),
        DeclareLaunchArgument(
            'stereo_proc_params',
            default_value=default_stereo_proc_params,
            description='Path to the stereo depth parameter YAML file',
        ),
        Node(
            package='hikrobot_camera_driver',
            executable='stereo_node',
            name='stereo_node',
            output='screen',
            parameters=[camera_params],
        ),
        Node(
            package='stereo_depth',
            executable='stereo_depth_node',
            name='stereo_depth_node',
            output='screen',
            parameters=[stereo_proc_params],
        ),
    ])
