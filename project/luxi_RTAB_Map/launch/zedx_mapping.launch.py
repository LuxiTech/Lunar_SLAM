"""ZED X-only RTAB-Map pipeline using SDK stereo-inertial odometry."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    """Keep ZED local odometry separate from RTAB-Map global loop closure."""
    base_launch = PathJoinSubstitution(
        [FindPackageShare("luxi_rtab_map"), "launch", "rgbd_mapping.launch.py"]
    )
    rtabmap_args = PythonExpression(
        [
            "'--Rtabmap/DetectionRate 1 "
            "--Rtabmap/LoopThr 0.2 "
            "--RGBD/LinearUpdate 0.05 "
            "--RGBD/AngularUpdate 0.03 "
            "--Reg/Force3DoF ",
            LaunchConfiguration("planar_motion"),
            " --RGBD/ForceOdom3DoF ",
            LaunchConfiguration("planar_motion"),
            " --Grid/3D true "
            "--Kp/MinDepth 0.2 "
            "--Kp/MaxDepth 6.0 "
            "--Kp/DetectorStrategy 1 "
            "--Kp/NNStrategy 1 "
            "--Kp/MaxFeatures 2048 "
            "--Vis/FeatureType 1 "
            "--Vis/MaxFeatures 2048 "
            "--Vis/MinInliers 30 "
            "--Optimizer/Robust true "
            "--RGBD/OptimizeMaxError 1.0 "
            # Native odometry does not attach external feature descriptors.
            # RTAB extracts SIFT at 1 Hz for its independent loop closures.
            "--Mem/UseOdomFeatures false "
            "--Mem/NotLinkedNodesKept false "
            "--Mem/IntermediateNodeDataKept false "
            "--Mem/DepthCompressionFormat .png "
            "--Grid/DepthDecimation 2 "
            "--Grid/RangeMin 0.2 "
            "--Grid/RangeMax 6.0 "
            "--Grid/NoiseFilteringRadius 0.0 "
            "--Grid/NoiseFilteringMinNeighbors 5'",
        ]
    )
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch),
        launch_arguments={
            "rviz": LaunchConfiguration("rviz"),
            "rtabmap_viz": LaunchConfiguration("rtabmap_viz"),
            "database_path": LaunchConfiguration("database_path"),
            "new_map": LaunchConfiguration("new_map"),
            "overwrite_existing_database": LaunchConfiguration(
                "overwrite_existing_database"
            ),
            "load_saved_map": LaunchConfiguration("load_saved_map"),
            "wait_for_camera": LaunchConfiguration("wait_for_camera"),
            "camera_wait_timeout": LaunchConfiguration("camera_wait_timeout"),
            "base_frame": "zed_camera_link",
            "learned_frontend": "false",
            "visual_odometry": "false",
            "icp_odometry": "false",
            "rgbd_sync": "false",
            "subscribe_rgbd": "true",
            "rgbd_topic": "/sensors/rgbd/rgbd_image",
            "odom_topic": "/zed/zed_node/odom",
            "subscribe_odom_info": "false",
            "publish_odom_tf": "false",
            "use_imu": "false",
            "rtabmap_imu_topic": "/zedx/rtabmap_imu_disabled",
            "approx_sync": "true",
            "approx_sync_max_interval": "0.05",
            "topic_queue_size": "30",
            "sync_queue_size": "20",
            "rtabmap_start_delay": "0.5",
            "rtabmap_args": rtabmap_args,
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
            # ZED X camera-body mapping must preserve Z/roll/pitch. The web
            # profile also passes false explicitly, independent of other cameras.
            DeclareLaunchArgument("planar_motion", default_value="false"),
            mapping,
        ]
    )
