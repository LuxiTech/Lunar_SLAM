"""Run level-A odometry and level-B SuperPoint features with RTAB-Map."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    base_launch = PathJoinSubstitution(
        [FindPackageShare("luxi_rtab_map"), "launch", "rgbd_mapping.launch.py"]
    )
    learned_rtabmap_args = (
        "--Rtabmap/DetectionRate 1 "
        "--Rtabmap/LoopThr 0.2 "
        "--RGBD/LinearUpdate 0.1 "
        "--RGBD/AngularUpdate 0.1 "
        "--Kp/MinDepth 0.2 "
        "--Kp/MaxDepth 6.0 "
        # SIFT selects RTAB's float/L2 registration path only. With
        # Mem/UseOdomFeatures=true, the actual 256-D descriptors come from the
        # external SuperPoint RGBDImage and no SIFT extraction is performed.
        "--Kp/DetectorStrategy 1 "
        "--Kp/NNStrategy 1 "
        "--Kp/MaxFeatures 2048 "
        "--Vis/FeatureType 1 "
        "--Vis/MaxFeatures 2048 "
        "--Vis/MinInliers 20 "
        "--Mem/UseOdomFeatures true"
    )
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch),
        launch_arguments={
            "rviz": LaunchConfiguration("rviz"),
            "rtabmap_viz": LaunchConfiguration("rtabmap_viz"),
            "database_path": LaunchConfiguration("database_path"),
            "new_map": LaunchConfiguration("new_map"),
            "load_saved_map": LaunchConfiguration("load_saved_map"),
            "wait_for_camera": LaunchConfiguration("wait_for_camera"),
            "camera_wait_timeout": LaunchConfiguration("camera_wait_timeout"),
            "learned_frontend": "true",
            "visual_odometry": "false",
            "icp_odometry": "false",
            "odom_topic": "/luxi_visual_frontend/odom",
            "rgbd_sync": "false",
            "subscribe_rgbd": "true",
            "rgbd_topic": "/luxi_visual_frontend/rgbd_image",
            "rtabmap_args": learned_rtabmap_args,
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("rtabmap_viz", default_value="false"),
            DeclareLaunchArgument("database_path", default_value=""),
            DeclareLaunchArgument("new_map", default_value="false"),
            DeclareLaunchArgument("load_saved_map", default_value="true"),
            DeclareLaunchArgument("wait_for_camera", default_value="true"),
            DeclareLaunchArgument("camera_wait_timeout", default_value="60.0"),
            mapping,
        ]
    )
