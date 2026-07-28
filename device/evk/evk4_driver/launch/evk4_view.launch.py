from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_name = LaunchConfiguration("camera_name")
    serial = LaunchConfiguration("serial")
    trigger_in_mode = LaunchConfiguration("trigger_in_mode")
    erc_mode = LaunchConfiguration("erc_mode")
    erc_rate = LaunchConfiguration("erc_rate")
    anti_flicker = LaunchConfiguration("anti_flicker")
    anti_flicker_low_frequency = LaunchConfiguration(
        "anti_flicker_low_frequency"
    )
    anti_flicker_high_frequency = LaunchConfiguration(
        "anti_flicker_high_frequency"
    )
    trail_filter = LaunchConfiguration("trail_filter")
    trail_filter_type = LaunchConfiguration("trail_filter_type")
    trail_filter_threshold = LaunchConfiguration("trail_filter_threshold")
    fps = LaunchConfiguration("fps")
    display_type = LaunchConfiguration("type")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_name",
                default_value="event_camera",
                description="相机节点名，同时决定事件和图像话题命名空间。",
            ),
            DeclareLaunchArgument(
                "serial",
                default_value="",
                description="EVK4 序列号。留空时打开第一台可用相机。",
            ),
            DeclareLaunchArgument(
                "trigger_in_mode",
                default_value="external",
                description="触发输入模式：external、loopback 或 disabled。",
            ),
            DeclareLaunchArgument(
                "erc_mode",
                default_value="enabled",
                description="可视化事件率控制：enabled 或 disabled。",
            ),
            DeclareLaunchArgument(
                "erc_rate",
                default_value="1000000",
                description="可视化事件率上限，单位 events/s。",
            ),
            DeclareLaunchArgument(
                "anti_flicker",
                default_value="false",
                description="启用 EVK4 硬件防闪烁带阻滤波。",
            ),
            DeclareLaunchArgument(
                "anti_flicker_low_frequency",
                default_value="90",
                description="防闪烁带阻下限，单位 Hz。",
            ),
            DeclareLaunchArgument(
                "anti_flicker_high_frequency",
                default_value="110",
                description="防闪烁带阻上限，单位 Hz。",
            ),
            DeclareLaunchArgument(
                "trail_filter",
                default_value="false",
                description="启动时启用硬件事件轨迹滤波。",
            ),
            DeclareLaunchArgument(
                "trail_filter_type",
                default_value="stc_cut_trail",
                description="轨迹滤波模式。",
            ),
            DeclareLaunchArgument(
                "trail_filter_threshold",
                default_value="5000",
                description="轨迹滤波阈值，单位微秒。",
            ),
            DeclareLaunchArgument(
                "fps",
                default_value="25.0",
                description="事件渲染图像帧率。",
            ),
            DeclareLaunchArgument(
                "type",
                default_value="time_slice",
                description="事件渲染类型：time_slice 或 sharp。",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("evk4_driver"),
                                "launch",
                                "demo",
                                "view.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "camera_name": camera_name,
                    "serial": serial,
                    "trigger_in_mode": trigger_in_mode,
                    "erc_mode": erc_mode,
                    "erc_rate": erc_rate,
                    "anti_flicker": anti_flicker,
                    "anti_flicker_low_frequency": anti_flicker_low_frequency,
                    "anti_flicker_high_frequency": anti_flicker_high_frequency,
                    "trail_filter": trail_filter,
                    "trail_filter_type": trail_filter_type,
                    "trail_filter_threshold": trail_filter_threshold,
                    "fps": fps,
                    "type": display_type,
                }.items(),
            ),
        ]
    )
