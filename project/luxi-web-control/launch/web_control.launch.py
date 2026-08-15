# Copyright 2026 lunar
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch the generic browser-to-Twist control service."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    Shutdown,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    """Build the web control launch description."""
    default_config = os.path.join(
        get_package_share_directory("luxi_web_control"),
        "config",
        "web_control.yaml",
    )

    def web_control_node(condition=None):
        return Node(
            package="luxi_web_control",
            executable="web_control_node",
            name="web_control",
            output="screen",
            condition=condition,
            on_exit=Shutdown(reason="web control node exited"),
            parameters=[
                LaunchConfiguration("config"),
                {
                    "cmd_vel_topic": ParameterValue(
                        LaunchConfiguration("web_cmd_vel_topic"),
                        value_type=str,
                    ),
                    "robot_namespace": ParameterValue(
                        LaunchConfiguration("robot_namespace"),
                        value_type=str,
                    ),
                    "bind_address": ParameterValue(
                        LaunchConfiguration("bind_address"),
                        value_type=str,
                    ),
                    "http_port": ParameterValue(
                        LaunchConfiguration("http_port"),
                        value_type=int,
                    ),
                },
            ],
        )

    # A long-lived ros2cli daemon can retain stale Fast DDS graph/SHM state
    # after camera and RTAB-Map processes are killed.  The symptom is an
    # endless ParticipantEntitiesInfo Fast CDR / Bad alloc loop in every new
    # rclpy node.  The daemon is not needed by running ROS nodes, so stopping
    # it before this appliance-style service starts is safe; ros2cli starts a
    # fresh daemon automatically on the next graph command.
    reset_daemon = ExecuteProcess(
        cmd=["ros2", "daemon", "stop"],
        output="screen",
        condition=IfCondition(LaunchConfiguration("reset_ros_daemon")),
    )

    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        # Keep a distinct name for the deferred OnProcessExit action. When
        # this launch is included from the D1 launch, a same-named outer
        # cmd_vel_topic would otherwise replace the scoped manual input after
        # the ROS daemon reset completes.
        DeclareLaunchArgument(
            "web_cmd_vel_topic",
            default_value=LaunchConfiguration("cmd_vel_topic"),
        ),
        DeclareLaunchArgument("robot_namespace", default_value="d15041873"),
        DeclareLaunchArgument("bind_address", default_value="0.0.0.0"),
        DeclareLaunchArgument("http_port", default_value="8080"),
        DeclareLaunchArgument(
            "reset_ros_daemon",
            default_value="true",
            description=(
                "Stop a stale ros2cli daemon before starting the web node; "
                "running ROS nodes are not affected"
            ),
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=reset_daemon,
                on_exit=[web_control_node()],
            ),
            condition=IfCondition(LaunchConfiguration("reset_ros_daemon")),
        ),
        reset_daemon,
        web_control_node(
            condition=UnlessCondition(LaunchConfiguration("reset_ros_daemon"))
        ),
    ])
