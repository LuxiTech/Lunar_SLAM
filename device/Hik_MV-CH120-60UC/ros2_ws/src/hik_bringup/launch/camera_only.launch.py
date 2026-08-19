import os
import platform

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory('hik_bringup')
    camera_params = LaunchConfiguration('camera_params')
    external_trigger = LaunchConfiguration('external_trigger')
    mvs_root = os.environ.get('MVS_ROOT', '/opt/MVS')
    mvs_arch = 'aarch64' if platform.machine() in ('aarch64', 'arm64') else '64'
    mvs_library_path = os.path.join(mvs_root, 'lib', mvs_arch)
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_params',
            default_value=os.path.join(bringup_share, 'config', 'camera_params.yaml'),
            description='Path to the camera node parameter YAML file'),
        DeclareLaunchArgument(
            'external_trigger',
            default_value='true',
            description='Use the camera external trigger; set false for local free-run preview'),
        Node(
            package='hikrobot_camera_driver',
            executable='stereo_node',
            name='stereo_node',
            output='screen',
            parameters=[
                camera_params,
                {'external_trigger': ParameterValue(external_trigger, value_type=bool)},
            ],
            additional_env={
                'LD_LIBRARY_PATH': mvs_library_path + os.pathsep + os.environ.get('LD_LIBRARY_PATH', ''),
            },
        ),
    ])
