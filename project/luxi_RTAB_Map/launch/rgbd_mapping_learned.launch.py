"""Run level-A odometry and level-B SuperPoint features with RTAB-Map."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    base_launch = PathJoinSubstitution(
        [FindPackageShare("luxi_rtab_map"), "launch", "rgbd_mapping.launch.py"]
    )
    learned_rtabmap_args = PythonExpression(
        [
            "'--Rtabmap/DetectionRate 1 "
            "--Rtabmap/LoopThr 0.2 "
            "--RGBD/LinearUpdate 0.05 "
            "--RGBD/AngularUpdate 0.03 "
            "--Kp/MinDepth ",
            LaunchConfiguration("mapping_depth_min"),
            " --Kp/MaxDepth ",
            LaunchConfiguration("mapping_depth_max"),
            " "
            # SIFT selects RTAB's float/L2 registration path only. With
            # Mem/UseOdomFeatures=true, the actual 256-D descriptors come from the
            # external SuperPoint RGBDImage and no SIFT extraction is performed.
            "--Kp/DetectorStrategy 1 "
            "--Kp/NNStrategy 1 "
            "--Kp/MaxFeatures 2048 "
            "--Vis/FeatureType 1 "
            "--Vis/MaxFeatures 2048 "
            # Reject the weak 21/36 loop closure pattern observed in map033.
            "--Vis/MinInliers 30 "
            "--Optimizer/Robust true "
            "--RGBD/OptimizeMaxError 1.0 "
            "--Mem/UseOdomFeatures true "
            "--Grid/DepthDecimation ",
            LaunchConfiguration("grid_depth_decimation"),
            " --Grid/RangeMin ",
            LaunchConfiguration("mapping_depth_min"),
            " --Grid/RangeMax ",
            LaunchConfiguration("mapping_depth_max"),
            " --Grid/NoiseFilteringRadius ",
            LaunchConfiguration("grid_noise_filtering_radius"),
            " --Grid/NoiseFilteringMinNeighbors ",
            LaunchConfiguration("grid_noise_filtering_min_neighbors"),
            "'",
        ]
    )
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch),
        launch_arguments={
            "rviz": LaunchConfiguration("rviz"),
            "rtabmap_viz": LaunchConfiguration("rtabmap_viz"),
            "database_path": LaunchConfiguration("database_path"),
            "new_map": LaunchConfiguration("new_map"),
            "overwrite_existing_database": LaunchConfiguration("overwrite_existing_database"),
            "load_saved_map": LaunchConfiguration("load_saved_map"),
            "wait_for_camera": LaunchConfiguration("wait_for_camera"),
            "camera_wait_timeout": LaunchConfiguration("camera_wait_timeout"),
            "learned_frontend": "true",
            "visual_odometry": "false",
            "icp_odometry": "false",
            "odom_topic": "/luxi_visual_frontend/odom",
            "subscribe_odom_info": "false",
            "wait_for_transform": "1.0",
            # Start RTAB before the learned frontend's first accepted frame so
            # its volatile /tf subscription cannot miss the initial odom TF.
            # Model loading takes about five seconds on the NX, leaving a short
            # subscriber-ready window without triggering an input timeout.
            "rtabmap_start_delay": "3.0",
            # Both messages are emitted by one accepted frontend frame and
            # carry the exact same sensor stamp. Exact sync prevents RTAB-Map
            # from pairing odometry with the adjacent RGBD feature frame.
            "approx_sync": "false",
            "rgbd_sync": "false",
            "subscribe_rgbd": "true",
            "rgbd_topic": "/luxi_visual_frontend/rgbd_image",
            "visual_frontend_minimum_depth": LaunchConfiguration(
                "mapping_depth_min"),
            "visual_frontend_maximum_depth": LaunchConfiguration(
                "mapping_depth_max"),
            "visual_frontend_camera_to_imu_time_offset": LaunchConfiguration(
                "camera_to_imu_time_offset"),
            "use_imu": LaunchConfiguration("use_imu"),
            "imu_topic": LaunchConfiguration("imu_topic"),
            "rtabmap_imu_topic": PythonExpression(
                [
                    "'",
                    LaunchConfiguration("imu_topic"),
                    "' if '",
                    LaunchConfiguration("use_rtabmap_imu"),
                    "'.lower() in ('true', '1', 'yes', 'on') else "
                    "'/luxi_visual_frontend/rtabmap_imu_disabled'",
                ]
            ),
            "rtabmap_args": learned_rtabmap_args,
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("rtabmap_viz", default_value="false"),
            DeclareLaunchArgument("database_path", default_value=""),
            DeclareLaunchArgument("new_map", default_value="false"),
            DeclareLaunchArgument("overwrite_existing_database", default_value="false"),
            DeclareLaunchArgument("load_saved_map", default_value="true"),
            DeclareLaunchArgument("wait_for_camera", default_value="true"),
            DeclareLaunchArgument("camera_wait_timeout", default_value="60.0"),
            DeclareLaunchArgument(
                "grid_depth_decimation",
                default_value="2",
                description=(
                    "Hardware-independent depth decimation compatible with both "
                    "640x480 and 1024x750 sensor profiles."
                ),
            ),
            DeclareLaunchArgument("mapping_depth_min", default_value="0.2"),
            DeclareLaunchArgument("mapping_depth_max", default_value="6.0"),
            DeclareLaunchArgument("camera_to_imu_time_offset", default_value="0.0"),
            DeclareLaunchArgument("use_imu", default_value="true"),
            DeclareLaunchArgument("imu_topic", default_value="/sensors/imu/data"),
            DeclareLaunchArgument(
                "use_rtabmap_imu",
                default_value="false",
                description=(
                    "Forward IMU orientation to RTAB in addition to the learned "
                    "frontend. Disabled by default to prevent duplicate yaw fusion."
                ),
            ),
            DeclareLaunchArgument("grid_noise_filtering_radius", default_value="0.0"),
            DeclareLaunchArgument(
                "grid_noise_filtering_min_neighbors", default_value="5"),
            mapping,
        ]
    )
