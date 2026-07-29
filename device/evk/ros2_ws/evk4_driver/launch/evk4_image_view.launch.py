from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_name = LaunchConfiguration("camera_name")
    serial = LaunchConfiguration("serial")
    fps = LaunchConfiguration("fps")
    display_type = LaunchConfiguration("type")
    erc_rate = LaunchConfiguration("erc_rate")

    pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("evk4_driver"),
                        "launch",
                        "evk4_view.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "camera_name": camera_name,
            "serial": serial,
            "trigger_in_mode": "disabled",
            "erc_mode": "enabled",
            "erc_rate": erc_rate,
            "fps": fps,
            "type": display_type,
            "show_window": "true",
            "preview_fps": fps,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_name",
                default_value="event_camera",
                description="相机和图像话题命名空间。",
            ),
            DeclareLaunchArgument(
                "serial",
                default_value="",
                description="EVK4 序列号。",
            ),
            DeclareLaunchArgument(
                "fps",
                default_value="60.0",
                description="进程内低延迟渲染和预览帧率。",
            ),
            DeclareLaunchArgument(
                "type",
                default_value="time_slice",
                description="渲染模式：time_slice 或 sharp。",
            ),
            DeclareLaunchArgument(
                "erc_rate",
                default_value="1000000",
                description="预览事件率上限，单位 events/s。",
            ),
            SetEnvironmentVariable("QT_QPA_PLATFORM", "xcb"),
            pipeline,
        ]
    )
