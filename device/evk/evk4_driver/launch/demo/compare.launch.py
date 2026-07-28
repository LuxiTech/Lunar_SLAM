from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_name = LaunchConfiguration("camera_name")
    serial = LaunchConfiguration("serial")
    trigger_in_mode = LaunchConfiguration("trigger_in_mode")
    fps = LaunchConfiguration("fps")

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("evk4_driver"),
                        "launch",
                        "driver",
                        "driver.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "camera_name": camera_name,
            "serial": serial,
            "trigger_in_mode": trigger_in_mode,
            "erc_mode": "enabled",
            "erc_rate": "1000000",
            "trail_filter": "false",
        }.items(),
    )

    renderers = [
        Node(
            package="event_camera_renderer",
            executable="renderer_node",
            namespace=camera_name,
            name="renderer_time_slice",
            output="screen",
            parameters=[
                {
                    "fps": ParameterValue(fps, value_type=float),
                    "display_type": "time_slice",
                }
            ],
            remappings=[
                ("~/events", "events"),
                ("~/image_raw", "time_slice/image_raw"),
            ],
        ),
        Node(
            package="event_camera_renderer",
            executable="renderer_node",
            namespace=camera_name,
            name="renderer_sharp",
            output="screen",
            parameters=[
                {
                    "fps": ParameterValue(fps, value_type=float),
                    "display_type": "sharp",
                }
            ],
            remappings=[
                ("~/events", "events"),
                ("~/image_raw", "sharp/image_raw"),
            ],
        ),
    ]

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
                "trigger_in_mode",
                default_value="disabled",
                description="触发输入模式。",
            ),
            DeclareLaunchArgument(
                "fps",
                default_value="25.0",
                description="两个渲染器的输出帧率。",
            ),
            driver,
            *renderers,
        ]
    )
