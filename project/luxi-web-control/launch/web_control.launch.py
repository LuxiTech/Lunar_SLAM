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
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    """Build the web control launch description."""
    default_config = os.path.join(
        get_package_share_directory("luxi_web_control"),
        "config",
        "web_control.yaml",
    )

    def launch_web_control(context):
        # Resolve every launch configuration while the included launch scope is
        # still active.  The daemon-stop process exits after that scope has
        # returned; keeping LaunchConfiguration substitutions in its deferred
        # OnProcessExit action would then fail with "configuration does not
        # exist" when this file is included by lekiwi_web_control.launch.py.
        config = LaunchConfiguration("config").perform(context)
        web_cmd_vel_topic = LaunchConfiguration("web_cmd_vel_topic").perform(
            context
        )
        robot_namespace = LaunchConfiguration("robot_namespace").perform(
            context
        )
        bind_address = LaunchConfiguration("bind_address").perform(context)
        http_port = int(LaunchConfiguration("http_port").perform(context))
        restrict_http = IfCondition(
            LaunchConfiguration("restrict_http_to_d1_lan")
        ).evaluate(context)

        web_node = Node(
            package="luxi_web_control",
            executable="web_control_node",
            name="web_control",
            output="screen",
            on_exit=Shutdown(reason="web control node exited"),
            parameters=[
                config,
                {
                    "cmd_vel_topic": web_cmd_vel_topic,
                    "robot_namespace": robot_namespace,
                    "bind_address": bind_address,
                    "http_port": http_port,
                    "restrict_http_to_d1_lan": restrict_http,
                },
            ],
        )

        if not IfCondition(
            LaunchConfiguration("reset_ros_daemon")
        ).evaluate(context):
            return [web_node]

        # A long-lived ros2cli daemon can retain stale Fast DDS graph/SHM state
        # after camera and RTAB-Map processes are killed.  Stop it before the
        # appliance-style web service starts; the next ros2cli command starts a
        # fresh daemon automatically.
        reset_daemon = ExecuteProcess(
            cmd=["ros2", "daemon", "stop"],
            output="screen",
        )
        return [
            RegisterEventHandler(
                OnProcessExit(
                    target_action=reset_daemon,
                    on_exit=[web_node],
                )
            ),
            reset_daemon,
        ]

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
        DeclareLaunchArgument(
            "restrict_http_to_d1_lan", default_value="false"
        ),
        DeclareLaunchArgument("http_port", default_value="8080"),
        DeclareLaunchArgument(
            "reset_ros_daemon",
            default_value="true",
            description=(
                "Stop a stale ros2cli daemon before starting the web node; "
                "running ROS nodes are not affected"
            ),
        ),
        OpaqueFunction(function=launch_web_control),
    ])
