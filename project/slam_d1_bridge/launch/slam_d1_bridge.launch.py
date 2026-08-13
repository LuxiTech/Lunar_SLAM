# Copyright 2026 ROS 2 Developer
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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="d15041873"),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [
                        get_package_share_directory("slam_d1_bridge"),
                        "config",
                        "slam_d1_bridge.yaml",
                    ]
                ),
            ),
            Node(
                package="slam_d1_bridge",
                executable="slam_d1_bridge",
                name="slam_d1_bridge",
                namespace=namespace,
                parameters=[params_file],
                remappings=[
                    ("cmd_vel", "/cmd_vel"),
                    ("slam/pose", "/slam/pose"),
                ],
                output="screen",
            ),
        ]
    )
