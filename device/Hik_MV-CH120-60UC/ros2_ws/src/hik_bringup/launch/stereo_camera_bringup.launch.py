import os
import platform

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
    UnsetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory('hik_bringup')
    default_camera_params = os.path.join(bringup_share, 'config', 'camera_params.yaml')
    default_stereo_proc_params = os.path.join(bringup_share, 'config', 'stereo_proc.yaml')
    default_rviz_config = os.path.join(bringup_share, 'rviz', 'stereo_view.rviz')
    default_imu_params = os.path.join(bringup_share, 'config', 'h30_imu.yaml')

    camera_params = LaunchConfiguration('camera_params')
    external_trigger = LaunchConfiguration('external_trigger')
    stereo_proc_params = LaunchConfiguration('stereo_proc_params')
    start_stereo_depth = LaunchConfiguration('start_stereo_depth')
    use_rviz = LaunchConfiguration('use_rviz')
    use_imu = LaunchConfiguration('use_imu')
    imu_params = LaunchConfiguration('imu_params')
    mvs_root = os.environ.get('MVS_ROOT', '/opt/MVS')
    mvs_arch = 'aarch64' if platform.machine() in ('aarch64', 'arm64') else '64'
    mvs_library_path = os.path.join(mvs_root, 'lib', mvs_arch)
    mvs_environment = {
        'LD_LIBRARY_PATH': mvs_library_path + os.pathsep + os.environ.get('LD_LIBRARY_PATH', ''),
    }

    return LaunchDescription([
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
        SetEnvironmentVariable('QT_X11_NO_MITSHM', '1'),
        SetEnvironmentVariable('DISPLAY', os.environ.get('DISPLAY', ':0')),
        UnsetEnvironmentVariable('WAYLAND_DISPLAY'),
        DeclareLaunchArgument(
            'camera_params',
            default_value=default_camera_params,
            description='Path to the camera node parameter YAML file',
        ),
        DeclareLaunchArgument(
            'external_trigger',
            default_value='true',
            description='Use Line0 hardware synchronization for the stereo cameras',
        ),
        DeclareLaunchArgument(
            'stereo_proc_params',
            default_value=default_stereo_proc_params,
            description='Path to the stereo depth parameter YAML file',
        ),
        DeclareLaunchArgument(
            'start_stereo_depth',
            default_value='true',
            description='Start the standalone stereo depth node',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz with stereo images and point cloud',
        ),
        DeclareLaunchArgument(
            'use_imu',
            default_value='false',
            description='Start the WheelTec H30 IMU driver',
        ),
        DeclareLaunchArgument(
            'imu_params',
            default_value=default_imu_params,
            description='Path to the WheelTec H30 IMU parameter YAML file',
        ),
        Node(
            package='hikrobot_camera_driver',
            executable='stereo_node',
            name='stereo_node',
            output='screen',
            parameters=[
                camera_params,
                {'external_trigger': ParameterValue(external_trigger, value_type=bool)},
            ],
            additional_env=mvs_environment,
        ),
        # The MVS SDK needs a few seconds to open and synchronize both U3V
        # cameras.  Starting the OpenCV depth process concurrently can abort
        # the camera process on this ARM64 MVS runtime, so let the camera own
        # the initialization window first.
        TimerAction(
            period=5.0,
            actions=[Node(
                package='stereo_depth',
                executable='stereo_depth_node',
                name='stereo_depth_node',
                output='screen',
                parameters=[stereo_proc_params],
                condition=IfCondition(start_stereo_depth),
            )],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', default_rviz_config],
            output='screen',
            condition=IfCondition(use_rviz),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'h30_imu.launch.py'),
            ),
            launch_arguments={'imu_params': imu_params}.items(),
            condition=IfCondition(use_imu),
        ),
    ])
