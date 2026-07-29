from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial = LaunchConfiguration("serial")
    fps = LaunchConfiguration("fps")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial",
                default_value="",
                description="EVK4 序列号。",
            ),
            DeclareLaunchArgument(
                "fps",
                default_value="25.0",
                description="渲染图像帧率。",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("evk4_driver"),
                                "launch",
                                "demo",
                                "compare.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "serial": serial,
                    "fps": fps,
                }.items(),
            ),
        ]
    )
