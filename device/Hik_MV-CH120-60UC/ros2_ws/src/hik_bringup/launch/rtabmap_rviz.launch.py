"""Open the RViz layout used to inspect the LuXi RTAB-Map session."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, UnsetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('hik_bringup')
    return LaunchDescription([
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
        SetEnvironmentVariable('QT_X11_NO_MITSHM', '1'),
        SetEnvironmentVariable('DISPLAY', os.environ.get('DISPLAY', ':0')),
        UnsetEnvironmentVariable('WAYLAND_DISPLAY'),
        Node(
            package='rviz2', executable='rviz2', name='rtabmap_rviz',
            output='screen',
            arguments=['-d', os.path.join(share, 'rviz', 'rtabmap.rviz')]),
    ])
