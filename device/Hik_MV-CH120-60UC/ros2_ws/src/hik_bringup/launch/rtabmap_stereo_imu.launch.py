"""Complete stereo-inertial RTAB-Map bringup for the LuXi camera rig."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
    UnsetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _cleanup_database_after_rtabmap(context):
    """Remove only an opted-in database after rtabmap has closed it cleanly."""
    clear_on_exit = LaunchConfiguration('clear_db_on_exit').perform(context)
    if clear_on_exit.lower() not in ('1', 'true', 'yes', 'on'):
        return []

    database = os.path.abspath(os.path.expanduser(
        LaunchConfiguration('database_path').perform(context)))
    home = os.path.abspath(os.path.expanduser('~'))
    # A launch argument must never turn this convenience cleanup into a broad
    # delete. Restrict it to an explicit SQLite database in this user's home.
    if not database.startswith(home + os.sep) or not database.endswith('.db'):
        return [LogInfo(msg=(
            'Not removing database outside the user home or without a .db suffix: '
            + database))]

    removed = []
    for candidate in (database, database + '-wal', database + '-shm'):
        if os.path.isfile(candidate):
            os.remove(candidate)
            removed.append(candidate)
    if removed:
        return [LogInfo(msg='Removed temporary RTAB-Map database: ' + ', '.join(removed))]
    return [LogInfo(msg='Temporary RTAB-Map database was already absent: ' + database)]


def generate_launch_description():
    bringup_share = get_package_share_directory('hik_bringup')
    # The MVS SDK ships its own libusb.  It is older than the system version
    # required by PCL on Ubuntu 22.04, so a globally exported MVS library path
    # makes rgbd_odometry and rtabmap fail at startup.  Keep the inherited ROS
    # and workspace paths, but remove only the MVS SDK entries here.  The
    # included camera launch adds MVS back exclusively for stereo_node.
    mvs_root = os.path.abspath(os.environ.get('MVS_ROOT', '/opt/MVS'))
    mvs_library_dirs = {
        os.path.join(mvs_root, 'lib', 'aarch64'),
        os.path.join(mvs_root, 'lib', '64'),
    }
    sanitized_ld_library_path = os.pathsep.join(
        item for item in os.environ.get('LD_LIBRARY_PATH', '').split(os.pathsep)
        if item and os.path.abspath(item) not in mvs_library_dirs)
    camera_params = LaunchConfiguration('camera_params')
    imu_params = LaunchConfiguration('imu_params')
    use_imu = LaunchConfiguration('use_imu')
    slam_params = LaunchConfiguration('slam_params')
    use_rtabmap_viz = LaunchConfiguration('use_rtabmap_viz')
    use_debug_cloud = LaunchConfiguration('use_debug_cloud')
    use_status_monitor = LaunchConfiguration('use_status_monitor')
    database_path = LaunchConfiguration('database_path')
    delete_db_on_start = LaunchConfiguration('delete_db_on_start')
    clear_db_on_exit = LaunchConfiguration('clear_db_on_exit')
    rgbd_remappings = [
        ('rgbd_image', '/stereo/rgbd_image'),
        # rtabmap_odom and rtabmap subscribe to the relative topic "imu".
        # The H30 driver intentionally publishes the standard global topic.
        ('imu', '/imu/data'),
    ]
    rtabmap_remappings = rgbd_remappings + [
        ('odom', '/odom'),
    ]

    return LaunchDescription([
        SetEnvironmentVariable('LD_LIBRARY_PATH', sanitized_ld_library_path),
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
            'use_imu', default_value='false',
            description='Start H30 driver (disabled for visual-only mapping when /dev/H30-imu is absent)'),
        DeclareLaunchArgument(
            'slam_params',
            default_value=os.path.join(bringup_share, 'config', 'rtabmap_stereo_imu.yaml'),
            description='RTAB-Map and rectification parameters'),
        DeclareLaunchArgument(
            'use_rtabmap_viz', default_value='false',
            description='Start the Qt RTAB-Map GUI in lightweight map/graph mode'),
        DeclareLaunchArgument(
            'use_debug_cloud', default_value='true',
            description='Publish the low-latency /luxi/cloud_map_accumulated RViz cloud alongside RTAB-Map'),
        DeclareLaunchArgument(
            'use_status_monitor', default_value='true',
            description='Print RTAB-Map node/odom/mapData status while mapping'),
        DeclareLaunchArgument(
            'database_path', default_value=os.path.expanduser('~/.ros/luxi_stereo_rtabmap.db'),
            description='RTAB-Map database path for the mapping session'),
        DeclareLaunchArgument(
            'delete_db_on_start', default_value='true',
            description='Start a new map by deleting the selected database'),
        DeclareLaunchArgument(
            'clear_db_on_exit', default_value='true',
            description='Delete the temporary RTAB-Map database after Ctrl+C; set false to save it'),
        # Camera, H30 driver and left_camera_optical_frame -> imu_link TF.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'stereo_imu_calibrated.launch.py')),
            launch_arguments={
                'camera_params': camera_params,
                'imu_params': imu_params,
                'use_imu': use_imu,
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
        # The ARM64 MVS runtime must finish opening both U3V cameras before
        # OpenCV/RTAB-Map subscribers start.  Without this window, concurrent
        # startup can abort stereo_node before it publishes its first frame.
        TimerAction(
            # MVS needs about five seconds to open both triggered USB cameras
            # on this NX. Leave a small margin so RTAB-Map never receives the
            # driver's initial pre-TF frame.
            period=6.0,
            actions=[
                # Standard rectified stereo topics and matching CameraInfo.
                Node(
                    package='stereo_depth', executable='stereo_depth_node',
                    name='stereo_depth_node', output='screen', parameters=[slam_params]),
                Node(
                    package='rtabmap_odom', executable='rgbd_odometry',
                    name='rgbd_odometry', output='screen', parameters=[slam_params],
                    remappings=rgbd_remappings),
                Node(
                    package='hik_bringup', executable='odom_tf_republisher.py',
                    name='odom_tf_republisher', output='screen', parameters=[{
                        # Smooth only the RViz/TF representation. RTAB-Map
                        # continues to consume the original /odom samples.
                        'position_smoothing_alpha': 0.35,
                        'orientation_smoothing_alpha': 0.35,
                    }]),
                Node(
                    package='stereo_depth', executable='rgb_cloud_accumulator_node',
                    name='rgb_cloud_accumulator_node', output='screen',
                    parameters=[slam_params],
                    condition=IfCondition(use_debug_cloud)),
                rtabmap_node := Node(
                    package='rtabmap_slam', executable='rtabmap', name='rtabmap',
                    output='screen', parameters=[slam_params, {
                        'database_path': database_path,
                        'delete_db_on_start': delete_db_on_start,
                    }],
                    remappings=rtabmap_remappings),
                Node(
                    package='hik_bringup', executable='rtabmap_status.py',
                    name='rtabmap_status', output='screen',
                    condition=IfCondition(use_status_monitor)),
                Node(
                    package='rtabmap_viz', executable='rtabmap_viz', name='rtabmap_viz',
                    output='screen', parameters=[slam_params],
                    arguments=['-d', os.path.join(bringup_share, 'config', 'rtabmap_gui.ini')],
                    remappings=rgbd_remappings,
                    condition=IfCondition(use_rtabmap_viz)),
            ],
        ),
        # This is deliberately tied to rtabmap's process exit, rather than
        # launch shutdown, so SQLite has finished flushing before cleanup.
        RegisterEventHandler(
            OnProcessExit(
                target_action=rtabmap_node,
                on_exit=[OpaqueFunction(function=_cleanup_database_after_rtabmap)])),
    ])
