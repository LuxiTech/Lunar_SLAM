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
from launch.actions import DeclareLaunchArgument, Shutdown
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
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("bind_address", default_value="0.0.0.0"),
        DeclareLaunchArgument("web_ui_mode", default_value="map_portal"),
        DeclareLaunchArgument("restrict_http_to_d1_lan", default_value="false"),
        DeclareLaunchArgument("http_port", default_value="8080"),
        Node(
            package="luxi_web_control",
            executable="web_control_node",
            name="web_control",
            output="screen",
            on_exit=Shutdown(reason="web control node exited"),
            parameters=[
                LaunchConfiguration("config"),
                {
                    "cmd_vel_topic": ParameterValue(
                        LaunchConfiguration("cmd_vel_topic"),
                        value_type=str,
                    ),
                    "bind_address": ParameterValue(
                        LaunchConfiguration("bind_address"),
                        value_type=str,
                    ),
                    "web_ui_mode": ParameterValue(
                        LaunchConfiguration("web_ui_mode"),
                        value_type=str,
                    ),
                    "http_port": ParameterValue(
                        LaunchConfiguration("http_port"),
                        value_type=int,
                    ),
                    "restrict_http_to_d1_lan": ParameterValue(
                        LaunchConfiguration("restrict_http_to_d1_lan"),
                        value_type=bool,
                    ),
                },
            ],
        ),
    ])
