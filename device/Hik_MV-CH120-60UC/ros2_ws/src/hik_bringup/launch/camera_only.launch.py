import os
import platform

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    mvs_root = os.environ.get('MVS_ROOT', '/opt/MVS')
    mvs_arch = 'aarch64' if platform.machine() in ('aarch64', 'arm64') else '64'
    mvs_library_path = os.path.join(mvs_root, 'lib', mvs_arch)
    return LaunchDescription([
        Node(
            package='hikrobot_camera_driver',
            executable='stereo_node',
            name='stereo_node',
            output='screen',
            additional_env={
                'LD_LIBRARY_PATH': mvs_library_path + os.pathsep + os.environ.get('LD_LIBRARY_PATH', ''),
            },
        ),
    ])
