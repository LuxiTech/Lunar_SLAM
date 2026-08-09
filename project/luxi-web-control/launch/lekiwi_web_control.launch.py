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

"""Launch web control with the DDS settings used by the LeKiwi base."""

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
            default_value="/lekiwi/cmd_vel_standard",
        ),
        DeclareLaunchArgument("bind_address", default_value="0.0.0.0"),
        DeclareLaunchArgument("http_port", default_value="8080"),
        DeclareLaunchArgument(
            "config",
            default_value=os.path.join(
                package_share,
                "config",
                "web_control.yaml",
            ),
        ),
        SetEnvironmentVariable("ROS_DOMAIN_ID", "0"),
        SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
        SetEnvironmentVariable(
            "ROS_AUTOMATIC_DISCOVERY_RANGE",
            "SUBNET",
        ),
        SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "0"),
        SetEnvironmentVariable("FASTDDS_BUILTIN_TRANSPORTS", "UDPv4"),
        GroupAction(
            scoped=True,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(generic_launch),
                launch_arguments={
                    "cmd_vel_topic": LaunchConfiguration(
                        "standard_cmd_vel_topic"
                    ),
                    "bind_address": LaunchConfiguration("bind_address"),
                    "http_port": LaunchConfiguration("http_port"),
                    "config": LaunchConfiguration("config"),
                }.items(),
            )],
        ),
        Node(
            package="luxi_3d_navigation",
            executable="twist_direction_adapter_node",
            name="lekiwi_twist_direction_adapter",
            output="screen",
            parameters=[{
                "input_topic": LaunchConfiguration(
                    "standard_cmd_vel_topic"
                ),
                "output_topic": LaunchConfiguration("cmd_vel_topic"),
                "invert_angular_z": True,
            }],
        ),
    ])
