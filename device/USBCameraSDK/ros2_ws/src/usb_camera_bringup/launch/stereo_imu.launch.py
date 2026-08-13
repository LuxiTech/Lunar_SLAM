"""Start the calibrated USB stereo cameras and H30 IMU."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera_params = os.path.join(
        get_package_share_directory('usb_camera_driver'),
        'config',
        'stereo_camera.yaml',
    )
    imu_params = os.path.join(
        get_package_share_directory('usb_camera_bringup'),
        'config',
        'h30_imu.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument('camera_params', default_value=camera_params),
        DeclareLaunchArgument('imu_params', default_value=imu_params),
        Node(
            package='usb_camera_driver',
            executable='stereo_node',
            name='stereo_node',
            output='screen',
            parameters=[LaunchConfiguration('camera_params')],
        ),
        Node(
            package='yesense_std_ros2',
            executable='yesense_node_publisher',
            name='yesense_pub',
            output='screen',
            parameters=[LaunchConfiguration('imu_params')],
        ),
        # Kalibr run04 T_cam0_imu: parent camera optical frame, child IMU frame.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_imu_static_tf',
            output='screen',
            arguments=[
                '--x', '0.04391590957628977',
                '--y', '0.0007111012450178807',
                '--z', '-0.03359544596572737',
                '--qx', '0.7081849349309625',
                '--qy', '0.0017267657694375585',
                '--qz', '-0.007348742978277029',
                '--qw', '0.7059866232397363',
                '--frame-id', 'left_camera_optical_frame',
                '--child-frame-id', 'imu_link',
            ],
        ),
    ])
