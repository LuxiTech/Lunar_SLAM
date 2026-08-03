"""Start the stereo cameras, H30 IMU and the reviewed Kalibr TF.

This launch deliberately does not alter camera or IMU timestamps.  The
camera-to-IMU time offset is documented in config/kalibr_cam_imu.yaml and
must be supplied to the later VIO/fusion package.
"""

import os
import platform

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('hik_bringup')
    default_camera_params = os.path.join(bringup_share, 'config', 'camera_params.yaml')
    default_imu_params = os.path.join(bringup_share, 'config', 'h30_imu.yaml')
    camera_params = LaunchConfiguration('camera_params')
    imu_params = LaunchConfiguration('imu_params')
    use_imu = LaunchConfiguration('use_imu')
    mvs_root = os.environ.get('MVS_ROOT', '/opt/MVS')
    mvs_arch = 'aarch64' if platform.machine() in ('aarch64', 'arm64') else '64'
    mvs_environment = {
        'LD_LIBRARY_PATH': os.path.join(mvs_root, 'lib', mvs_arch) + os.pathsep + os.environ.get('LD_LIBRARY_PATH', ''),
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_params', default_value=default_camera_params,
            description='Path to stereo camera ROS parameter YAML file'),
        DeclareLaunchArgument(
            'imu_params', default_value=default_imu_params,
            description='Path to WheelTec H30 IMU parameter YAML file'),
        DeclareLaunchArgument(
            'use_imu', default_value='true',
            description='Start the H30 IMU driver when the /dev/H30-imu device is available'),
        Node(
            package='hikrobot_camera_driver', executable='stereo_node',
            name='stereo_node', output='screen', parameters=[camera_params],
            additional_env=mvs_environment),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'h30_imu.launch.py')),
            launch_arguments={'imu_params': imu_params}.items(),
            condition=IfCondition(use_imu)),
        # T_left_camera_optical_frame_imu_link from Kalibr dynamic_05.
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='kalibr_camera_imu_static_tf', output='screen',
            arguments=[
                '--x', '-0.06080516708588524',
                '--y', '-0.017371973406354727',
                '--z', '-0.12304891470347533',
                '--qx', '0.6868459901794202',
                '--qy', '0.024915427523349555',
                '--qz', '-0.0010234338137173815',
                '--qw', '0.7263750820540378',
                '--frame-id', 'left_camera_optical_frame',
                '--child-frame-id', 'imu_link',
            ]),
    ])
