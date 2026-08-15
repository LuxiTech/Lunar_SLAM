"""Start the learned RGB-D visual frontend."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    """Declare frontend launch arguments and start its ROS node."""
    default_config = PathJoinSubstitution(
        [FindPackageShare("luxi_visual_frontend"), "config", "visual_odometry.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("config_file", default_value=default_config),
            DeclareLaunchArgument("device", default_value="cuda"),
            DeclareLaunchArgument("publish_tf", default_value="true"),
            DeclareLaunchArgument(
                "odom_topic", default_value="/luxi_visual_frontend/odom"
            ),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument(
                "status_topic", default_value="/luxi_visual_frontend/status"
            ),
            DeclareLaunchArgument(
                "diagnostics_topic",
                default_value="/luxi_visual_frontend/diagnostics",
            ),
            Node(
                package="luxi_visual_frontend",
                executable="visual_odometry_node",
                name="luxi_visual_frontend",
                output="screen",
                parameters=[
                    LaunchConfiguration("config_file"),
                    {
                        "device": LaunchConfiguration("device"),
                        "publish_tf": LaunchConfiguration("publish_tf"),
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "odom_frame": LaunchConfiguration("odom_frame"),
                        "status_topic": LaunchConfiguration("status_topic"),
                        "diagnostics_topic": LaunchConfiguration("diagnostics_topic"),
                    },
                ],
            ),
        ]
    )
