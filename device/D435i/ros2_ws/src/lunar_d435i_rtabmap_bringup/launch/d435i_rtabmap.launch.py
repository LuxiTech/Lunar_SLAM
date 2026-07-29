"""Launch D435i RGB-D SLAM with RTAB-Map.

This launch file is the adaptation layer for the current camera-only setup:

- lunar_realsense_bringup publishes D435i RGB, aligned depth, IMU, point cloud
  and camera TF.
- A zero static transform base_link -> camera_link is published by default so
  RTAB-Map and Nav2 can run without a physical robot base model.
- rtabmap_launch consumes the aligned RGB-D stream and publishes map/odom TF,
  occupancy grid and RTAB-Map database.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    realsense_launch = PathJoinSubstitution(
        [FindPackageShare("lunar_realsense_bringup"), "launch", "d435i.launch.py"]
    )
    rtabmap_launch = PathJoinSubstitution(
        [FindPackageShare("rtabmap_launch"), "launch", "rtabmap.launch.py"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("lunar_d435i_rtabmap_bringup"), "rviz", "d435i_rtabmap.rviz"]
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch),
        condition=IfCondition(LaunchConfiguration("launch_camera")),
        launch_arguments={
            "rviz": "false",
            "color_profile": LaunchConfiguration("color_profile"),
            "depth_profile": LaunchConfiguration("depth_profile"),
            "initial_reset": LaunchConfiguration("initial_reset"),
            "log_level": LaunchConfiguration("camera_log_level"),
        }.items(),
    )

    base_to_camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_d435i_tf",
        arguments=[
            "--x",
            LaunchConfiguration("camera_x"),
            "--y",
            LaunchConfiguration("camera_y"),
            "--z",
            LaunchConfiguration("camera_z"),
            "--roll",
            LaunchConfiguration("camera_roll"),
            "--pitch",
            LaunchConfiguration("camera_pitch"),
            "--yaw",
            LaunchConfiguration("camera_yaw"),
            "--frame-id",
            LaunchConfiguration("base_frame"),
            "--child-frame-id",
            LaunchConfiguration("camera_frame"),
        ],
        condition=IfCondition(LaunchConfiguration("publish_base_to_camera_tf")),
        output="screen",
    )

    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap_launch),
        launch_arguments={
            "stereo": "false",
            "localization": LaunchConfiguration("localization"),
            "rtabmap_viz": LaunchConfiguration("rtabmap_viz"),
            "rviz": "false",
            "rviz_cfg": rviz_config,
            "frame_id": LaunchConfiguration("base_frame"),
            "map_frame_id": LaunchConfiguration("map_frame"),
            "odom_frame_id": "",
            "database_path": LaunchConfiguration("database_path"),
            "rgb_topic": LaunchConfiguration("rgb_topic"),
            "depth_topic": LaunchConfiguration("depth_topic"),
            "camera_info_topic": LaunchConfiguration("camera_info_topic"),
            "rgbd_sync": "true",
            "approx_rgbd_sync": "true",
            "approx_sync": "true",
            "approx_sync_max_interval": LaunchConfiguration("approx_sync_max_interval"),
            "visual_odometry": "true",
            "icp_odometry": "false",
            "qos": LaunchConfiguration("qos"),
            "qos_image": LaunchConfiguration("qos"),
            "qos_camera_info": LaunchConfiguration("qos"),
            "wait_for_transform": LaunchConfiguration("wait_for_transform"),
            "rtabmap_args": LaunchConfiguration("rtabmap_args"),
            "odom_args": LaunchConfiguration("odom_args"),
            "log_level": LaunchConfiguration("rtabmap_log_level"),
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("rviz")),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("launch_camera", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("rtabmap_viz", default_value="false"),
            DeclareLaunchArgument("localization", default_value="false"),
            DeclareLaunchArgument("database_path", default_value="~/.ros/lunar_d435i_rtabmap.db"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("camera_frame", default_value="camera_link"),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("publish_base_to_camera_tf", default_value="true"),
            DeclareLaunchArgument("camera_x", default_value="0.0"),
            DeclareLaunchArgument("camera_y", default_value="0.0"),
            DeclareLaunchArgument("camera_z", default_value="0.0"),
            DeclareLaunchArgument("camera_roll", default_value="0.0"),
            DeclareLaunchArgument("camera_pitch", default_value="0.0"),
            DeclareLaunchArgument("camera_yaw", default_value="0.0"),
            DeclareLaunchArgument("rgb_topic", default_value="/camera/camera/color/image_raw"),
            DeclareLaunchArgument(
                "depth_topic",
                default_value="/camera/camera/aligned_depth_to_color/image_raw",
            ),
            DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera/color/camera_info"),
            DeclareLaunchArgument("color_profile", default_value="640,480,15"),
            DeclareLaunchArgument("depth_profile", default_value="640,480,15"),
            DeclareLaunchArgument("initial_reset", default_value="false"),
            DeclareLaunchArgument("qos", default_value="2"),
            DeclareLaunchArgument("approx_sync_max_interval", default_value="0.05"),
            DeclareLaunchArgument("wait_for_transform", default_value="0.5"),
            DeclareLaunchArgument("rtabmap_args", default_value=""),
            DeclareLaunchArgument("odom_args", default_value=""),
            DeclareLaunchArgument("camera_log_level", default_value="info"),
            DeclareLaunchArgument("rtabmap_log_level", default_value="info"),
            camera,
            base_to_camera_tf,
            rtabmap,
            rviz,
        ]
    )
