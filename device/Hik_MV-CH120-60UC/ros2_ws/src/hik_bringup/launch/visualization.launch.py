import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, UnsetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('hik_bringup'),
        'rviz',
        'stereo_view.rviz',
    )
    return LaunchDescription([
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
        SetEnvironmentVariable('QT_X11_NO_MITSHM', '1'),
        SetEnvironmentVariable('DISPLAY', os.environ.get('DISPLAY', ':0')),
        UnsetEnvironmentVariable('WAYLAND_DISPLAY'),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
