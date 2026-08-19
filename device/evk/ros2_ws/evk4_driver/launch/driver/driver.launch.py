from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params_file = PathJoinSubstitution(
        [
            FindPackageShare("evk4_driver"),
            "config",
            "driver",
            "evk4_hd.params.yaml",
        ]
    )
    params_file = LaunchConfiguration("params_file", default=default_params_file)
    camera_name = LaunchConfiguration("camera_name")
    serial = LaunchConfiguration("serial")
    frame_id = LaunchConfiguration("frame_id")
    trigger_in_mode = LaunchConfiguration("trigger_in_mode")
    use_multithreading = LaunchConfiguration("use_multithreading")
    statistics_print_interval = LaunchConfiguration("statistics_print_interval")
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

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="EVK4 HD 参数文件。",
            ),
            DeclareLaunchArgument(
                "camera_name",
                default_value="event_camera",
                description="ROS 节点名，同时作为事件话题命名空间。",
            ),
            DeclareLaunchArgument(
                "serial",
                default_value="",
                description="EVK4 序列号。留空时打开第一台可用相机。",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="event_camera",
                description="事件消息 header 中使用的 frame_id。",
            ),
            DeclareLaunchArgument(
                "trigger_in_mode",
                default_value="external",
                description="触发输入模式：external、loopback 或 disabled。",
            ),
            DeclareLaunchArgument(
                "use_multithreading",
                default_value="false",
                description="是否分离 SDK 回调线程与 ROS 发布线程。",
            ),
            DeclareLaunchArgument(
                "statistics_print_interval",
                default_value="2.0",
                description="驱动统计信息打印周期，单位秒。",
            ),
            DeclareLaunchArgument(
                "erc_mode",
                default_value="disabled",
                description="硬件事件率控制：enabled 或 disabled。",
            ),
            DeclareLaunchArgument(
                "erc_rate",
                default_value="100000000",
                description="硬件事件率上限，单位 events/s。",
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
                description="轨迹滤波模式：trail、stc_cut_trail 或 stc_keep_trail。",
            ),
            DeclareLaunchArgument(
                "trail_filter_threshold",
                default_value="5000",
                description="轨迹滤波阈值，单位微秒。",
            ),
            Node(
                package="metavision_driver",
                executable="driver_node",
                name=camera_name,
                output="screen",
                parameters=[
                    params_file,
                    {
                        "serial": ParameterValue(serial, value_type=str),
                        "frame_id": ParameterValue(frame_id, value_type=str),
                        "trigger_in_mode": ParameterValue(
                            trigger_in_mode, value_type=str
                        ),
                        "use_multithreading": ParameterValue(
                            use_multithreading, value_type=bool
                        ),
                        "statistics_print_interval": ParameterValue(
                            statistics_print_interval, value_type=float
                        ),
                        "erc_mode": ParameterValue(erc_mode, value_type=str),
                        "erc_rate": ParameterValue(erc_rate, value_type=int),
                        "anti_flicker": ParameterValue(
                            anti_flicker, value_type=bool
                        ),
                        "anti_flicker_low_frequency": ParameterValue(
                            anti_flicker_low_frequency, value_type=int
                        ),
                        "anti_flicker_high_frequency": ParameterValue(
                            anti_flicker_high_frequency, value_type=int
                        ),
                        "trail_filter": ParameterValue(trail_filter, value_type=bool),
                        "trail_filter_type": ParameterValue(
                            trail_filter_type, value_type=str
                        ),
                        "trail_filter_threshold": ParameterValue(
                            trail_filter_threshold, value_type=int
                        ),
                    },
                ],
            ),
        ]
    )
