from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="rgbd"),
            DeclareLaunchArgument("database_path", default_value="/tmp/luxi_rtab_map_test.db"),
            DeclareLaunchArgument("delete_db_on_start", default_value="true"),
            DeclareLaunchArgument("max_frames", default_value="0"),
            DeclareLaunchArgument("rgb_topic", default_value="/sensors/rgbd/color/image_raw"),
            DeclareLaunchArgument(
                "depth_topic",
                default_value="/sensors/rgbd/depth/image_raw",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/sensors/rgbd/color/camera_info",
            ),
            Node(
                package="luxi_rtab_map",
                executable="luxi_rtab_map_node",
                name="luxi_rtab_map_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "mode": LaunchConfiguration("mode"),
                        "database_path": LaunchConfiguration("database_path"),
                        "delete_db_on_start": LaunchConfiguration("delete_db_on_start"),
                        "max_frames": LaunchConfiguration("max_frames"),
                        "rgb_topic": LaunchConfiguration("rgb_topic"),
                        "depth_topic": LaunchConfiguration("depth_topic"),
                        "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                    }
                ],
            ),
        ]
    )
