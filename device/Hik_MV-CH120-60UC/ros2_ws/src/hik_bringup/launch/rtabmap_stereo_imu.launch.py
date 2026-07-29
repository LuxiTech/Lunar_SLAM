"""Complete stereo-inertial RTAB-Map bringup for the LuXi camera rig."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, UnsetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('hik_bringup')
    camera_params = LaunchConfiguration('camera_params')
    imu_params = LaunchConfiguration('imu_params')
    slam_params = LaunchConfiguration('slam_params')
    use_rtabmap_viz = LaunchConfiguration('use_rtabmap_viz')
    use_status_monitor = LaunchConfiguration('use_status_monitor')
    database_path = LaunchConfiguration('database_path')
    delete_db_on_start = LaunchConfiguration('delete_db_on_start')
    # MVS exports an older libusb in LD_LIBRARY_PATH. System PCL (used by
    # RTAB-Map) requires libusb_set_option, which only exists in the system
    # libusb. Apply this only to RTAB-Map processes: the Hikrobot SDK keeps
    # using its own runtime libraries.
    rtabmap_environment = {
        'LD_PRELOAD': '/usr/lib/x86_64-linux-gnu/libusb-1.0.so.0',
    }

    rgbd_remappings = [
        ('rgbd_image', '/stereo/rgbd_image'),
    ]
    rtabmap_remappings = rgbd_remappings + [
        ('odom', '/odom'),
    ]

    return LaunchDescription([
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
        SetEnvironmentVariable('QT_X11_NO_MITSHM', '1'),
        SetEnvironmentVariable('DISPLAY', os.environ.get('DISPLAY', ':0')),
        UnsetEnvironmentVariable('WAYLAND_DISPLAY'),
        DeclareLaunchArgument(
            'camera_params',
            default_value=os.path.join(bringup_share, 'config', 'camera_params.yaml'),
            description='Stereo camera ROS parameters'),
        DeclareLaunchArgument(
            'imu_params',
            default_value=os.path.join(bringup_share, 'config', 'h30_imu.yaml'),
            description='H30 IMU ROS parameters'),
        DeclareLaunchArgument(
            'slam_params',
            default_value=os.path.join(bringup_share, 'config', 'rtabmap_stereo_imu.yaml'),
            description='RTAB-Map and rectification parameters'),
        DeclareLaunchArgument(
            'use_rtabmap_viz', default_value='false',
            description='Start the Qt RTAB-Map GUI (uses additional GPU/RAM)'),
        DeclareLaunchArgument(
            'use_status_monitor', default_value='true',
            description='Print RTAB-Map node/odom/mapData status while mapping'),
        DeclareLaunchArgument(
            'database_path', default_value=os.path.expanduser('~/.ros/luxi_stereo_rtabmap.db'),
            description='RTAB-Map database path for the mapping session'),
        DeclareLaunchArgument(
            'delete_db_on_start', default_value='true',
            description='Start a new map by deleting the selected database'),
        # Camera, H30 driver and left_camera_optical_frame -> imu_link TF.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'stereo_imu_calibrated.launch.py')),
            launch_arguments={
                'camera_params': camera_params,
                'imu_params': imu_params,
            }.items()),
        # RTAB-Map expects a robot/body frame whose X axis points forward and Z
        # points up. The Hikrobot images are in ROS optical frame coordinates
        # (Z forward, X right, Y down), so attach the optical camera frame below
        # base_link before visual odometry starts.
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='base_to_left_camera_optical_static_tf', output='screen',
            arguments=[
                '--x', '0.0',
                '--y', '0.0',
                '--z', '0.0',
                '--qx', '-0.5',
                '--qy', '0.5',
                '--qz', '-0.5',
                '--qw', '0.5',
                '--frame-id', 'base_link',
                '--child-frame-id', 'left_camera_optical_frame',
            ]),
        # Standard rectified stereo topics and matching CameraInfo.
        Node(
            package='stereo_depth', executable='stereo_depth_node',
            name='stereo_depth_node', output='screen', parameters=[slam_params]),
        Node(
            package='rtabmap_odom', executable='rgbd_odometry',
            name='rgbd_odometry', output='screen', parameters=[slam_params],
            remappings=rgbd_remappings, additional_env=rtabmap_environment),
        Node(
            package='hik_bringup', executable='odom_tf_republisher.py',
            name='odom_tf_republisher', output='screen'),
        Node(
            package='stereo_depth', executable='rgb_cloud_accumulator_node',
            name='rgb_cloud_accumulator_node', output='screen',
            parameters=[slam_params]),
        Node(
            package='rtabmap_slam', executable='rtabmap', name='rtabmap',
            output='screen', parameters=[slam_params, {
                'database_path': database_path,
                'delete_db_on_start': delete_db_on_start,
            }],
            remappings=rtabmap_remappings, additional_env=rtabmap_environment),
        Node(
            package='hik_bringup', executable='rtabmap_status.py',
            name='rtabmap_status', output='screen',
            condition=IfCondition(use_status_monitor)),
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', name='rtabmap_viz',
            output='screen', parameters=[slam_params], remappings=rgbd_remappings,
            condition=IfCondition(use_rtabmap_viz), additional_env=rtabmap_environment),
    ])
