"""Start the stereo cameras, H30 IMU and imported experimental Kalibr TF.

This launch deliberately does not alter camera or IMU timestamps.  The
camera-to-IMU time offset is documented in config/kalibr_cam_imu.yaml and
must be supplied to the later VIO/fusion package.
"""

import os
import platform

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('hik_bringup')
    default_camera_params = os.path.join(bringup_share, 'config', 'camera_params.yaml')
    default_imu_params = os.path.join(bringup_share, 'config', 'h30_imu.yaml')
    camera_params = LaunchConfiguration('camera_params')
    imu_params = LaunchConfiguration('imu_params')
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
        Node(
            package='hikrobot_camera_driver', executable='stereo_node',
            name='stereo_node', output='screen', parameters=[camera_params],
            additional_env=mvs_environment),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'h30_imu.launch.py')),
            launch_arguments={'imu_params': imu_params}.items()),
        # T_left_camera_optical_frame_imu_link from Kalibr dynamic_04.
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='kalibr_camera_imu_static_tf', output='screen',
            arguments=[
                '--x', '0.05740985968686727',
                '--y', '0.02595491340073357',
                '--z', '-0.057732071298254306',
                '--qx', '0.508445592440157',
                '--qy', '-0.485493288077626',
                '--qz', '0.494171813224474',
                '--qw', '0.511442631948285',
                '--frame-id', 'left_camera_optical_frame',
                '--child-frame-id', 'imu_link',
            ]),
    ])
