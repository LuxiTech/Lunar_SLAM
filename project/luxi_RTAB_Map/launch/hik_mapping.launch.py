"""Feed Hik RGB-D/H30 data into the project learned RTAB-Map pipeline."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    adapter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("luxi_adapter"), "launch", "sensor_bringup.launch.py"]
            )
        ),
        launch_arguments={
            "hardware": "hik",
            "external_trigger": LaunchConfiguration("external_trigger"),
            "stereo_proc_params": LaunchConfiguration("stereo_proc_params"),
            "enable_adapter": "true",
        }.items(),
    )
    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("luxi_rtab_map"),
                    "launch",
                    "rgbd_mapping_learned.launch.py",
                ]
            )
        ),
        launch_arguments={
            "rviz": LaunchConfiguration("rviz"),
            "database_path": LaunchConfiguration("database_path"),
            "new_map": LaunchConfiguration("new_map"),
            "overwrite_existing_database": LaunchConfiguration("overwrite_existing_database"),
            "load_saved_map": LaunchConfiguration("load_saved_map"),
            "camera_wait_timeout": LaunchConfiguration("camera_wait_timeout"),
            "wait_for_camera": "true",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "external_trigger",
            default_value="true",
            description="Use the rig's hardware trigger; false is a diagnostic free-run fallback.",
        ),
        DeclareLaunchArgument(
            "stereo_proc_params",
            default_value=PathJoinSubstitution(
                [FindPackageShare("hik_bringup"), "config", "stereo_proc.yaml"]
            ),
            description="Hik stereo-depth parameters only; RTAB-Map parameters stay project-owned.",
        ),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("database_path", default_value=""),
        DeclareLaunchArgument("new_map", default_value="true"),
        DeclareLaunchArgument("overwrite_existing_database", default_value="false"),
        DeclareLaunchArgument("load_saved_map", default_value="false"),
        DeclareLaunchArgument("camera_wait_timeout", default_value="60.0"),
        adapter_launch,
        # hik_bringup starts stereo_depth after its camera-open window. Enter
        # the canonical RGB-D/IMU synchronization gate after that timer fires.
        TimerAction(period=8.0, actions=[mapping_launch]),
    ])
