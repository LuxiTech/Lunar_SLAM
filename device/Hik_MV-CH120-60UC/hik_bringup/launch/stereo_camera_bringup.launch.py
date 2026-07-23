import os

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
    default_stereo_proc_params = os.path.join(bringup_share, 'config', 'stereo_proc.yaml')
    default_rviz_config = os.path.join(bringup_share, 'rviz', 'stereo_view.rviz')
    default_imu_params = os.path.join(bringup_share, 'config', 'h30_imu.yaml')

    camera_params = LaunchConfiguration('camera_params')
    stereo_proc_params = LaunchConfiguration('stereo_proc_params')
    use_rviz = LaunchConfiguration('use_rviz')
    use_imu = LaunchConfiguration('use_imu')
    imu_params = LaunchConfiguration('imu_params')

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
            parameters=[camera_params],
        ),
        Node(
            package='stereo_depth',
            executable='stereo_depth_node',
            name='stereo_depth_node',
            output='screen',
            parameters=[stereo_proc_params],
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
