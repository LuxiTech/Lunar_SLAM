from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_name = LaunchConfiguration("camera_name")
    serial = LaunchConfiguration("serial")
    output = LaunchConfiguration("output")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_name",
                default_value="event_camera",
                description="相机节点名，同时决定事件话题路径。",
            ),
            DeclareLaunchArgument(
                "serial",
                default_value="",
                description="EVK4 序列号。留空时打开第一台可用相机。",
            ),
            DeclareLaunchArgument(
                "output",
                default_value="evk4_events",
                description="rosbag 输出目录。",
            ),
            IncludeLaunchDescription(
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
                }.items(),
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "bag",
                    "record",
                    "-o",
                    output,
                    "--topics",
                    ["/", camera_name, "/events"],
                ],
                output="screen",
            ),
        ]
    )
