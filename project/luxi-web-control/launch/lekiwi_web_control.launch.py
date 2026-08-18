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

"""Launch web control with the DDS settings used by the connected D1 robot."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    """Configure Fast DDS discovery and include the generic web controller."""
    package_share = get_package_share_directory("luxi_web_control")
    generic_launch = os.path.join(
        package_share,
        "launch",
        "web_control.launch.py",
    )
    return LaunchDescription([
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument(
            "standard_cmd_vel_topic",
            default_value="/d1/cmd_vel_standard",
        ),
        DeclareLaunchArgument("bind_address", default_value="127.0.0.1"),
        DeclareLaunchArgument("web_ui_mode", default_value="map_portal"),
        DeclareLaunchArgument("http_port", default_value="8080"),
        DeclareLaunchArgument(
            "config",
            default_value=os.path.join(
                package_share,
                "config",
                "web_control.yaml",
            ),
        ),
        SetEnvironmentVariable("ROS_DOMAIN_ID", "42"),
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
        SetEnvironmentVariable(
            "ROS_AUTOMATIC_DISCOVERY_RANGE",
            "SUBNET",
        ),
        SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "0"),
        GroupAction(
            scoped=True,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(generic_launch),
                launch_arguments={
                    "cmd_vel_topic": LaunchConfiguration(
                        "standard_cmd_vel_topic"
                    ),
                    "bind_address": LaunchConfiguration("bind_address"),
                    "web_ui_mode": LaunchConfiguration("web_ui_mode"),
                    "http_port": LaunchConfiguration("http_port"),
                    # DDS still uses the D1 domain/network settings below, but
                    # the HTTP UI must remain reachable through every address
                    # selected by bind_address (including 0.0.0.0).
                    "restrict_http_to_d1_lan": "false",
                    "config": LaunchConfiguration("config"),
                }.items(),
            )],
        ),
        Node(
            package="luxi_3d_navigation",
            executable="velocity_command_mux_node",
            name="d1_velocity_command_mux",
            output="screen",
            parameters=[{
                "manual_input_topic": LaunchConfiguration(
                    "standard_cmd_vel_topic"
                ),
                "navigation_input_topic": "/navigation/cmd_vel",
                "output_topic": LaunchConfiguration("cmd_vel_topic"),
                "navigation_active_topic": "/navigation/active",
                "navigation_stop_topic": "/navigation/stop",
                "emergency_stop_topic": "/navigation/emergency_stop",
                "invert_angular_z": False,
            }],
        ),
    ])
