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

"""Launch the RViz clicked-point obstacle annotation node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Create the manual map annotation launch description."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("frame_id", default_value="map"),
            DeclareLaunchArgument("map_id", default_value=""),
            DeclareLaunchArgument("map_topic", default_value="/rtabmap/map"),
            DeclareLaunchArgument(
                "output_path",
                default_value=(
                    "/home/lunar/project/lunar_slam/maps/occupancy_maps/semantic_obstacles.yaml"
                ),
            ),
            DeclareLaunchArgument("map_transient_local", default_value="false"),
            Node(
                package="luxi_navigation",
                executable="manual_map_annotator",
                name="manual_map_annotator",
                output="screen",
                parameters=[
                    {
                        "frame_id": LaunchConfiguration("frame_id"),
                        "map_id": LaunchConfiguration("map_id"),
                        "map_topic": LaunchConfiguration("map_topic"),
                        "output_path": LaunchConfiguration("output_path"),
                        "map_transient_local": LaunchConfiguration(
                            "map_transient_local"
                        ),
                    }
                ],
            ),
        ]
    )
