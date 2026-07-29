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

"""End-to-end ROS test for clicked-point polygon annotation."""

import time

from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from visualization_msgs.msg import MarkerArray

from luxi_navigation.manual_map_annotator import ManualMapAnnotator


def spin_until(executor, condition, timeout=1.5):
    deadline = time.monotonic() + timeout
    while not condition() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
    assert condition()


def test_clicked_polygon_publishes_yaml_markers_and_mask(tmp_path):
    rclpy.init()
    output_path = tmp_path / "obstacles.yaml"
    annotator = ManualMapAnnotator(
        parameter_overrides=[
            Parameter("output_path", value=str(output_path)),
            Parameter("map_id", value="test_map"),
            Parameter("map_topic", value="/annotation_test/map"),
            Parameter("clicked_point_topic", value="/annotation_test/clicked"),
            Parameter("markers_topic", value="/annotation_test/markers"),
            Parameter("obstacle_mask_topic", value="/annotation_test/mask"),
        ]
    )
    client_node = Node("manual_annotation_test_client")
    clicked_publisher = client_node.create_publisher(
        PointStamped, "/annotation_test/clicked", 10
    )
    map_publisher = client_node.create_publisher(
        OccupancyGrid, "/annotation_test/map", 10
    )
    received_markers = []
    received_masks = []
    client_node.create_subscription(
        MarkerArray, "/annotation_test/markers", received_markers.append, 10
    )
    client_node.create_subscription(
        OccupancyGrid, "/annotation_test/mask", received_masks.append, 10
    )
    start_client = client_node.create_client(
        Trigger, "/manual_map_annotator/start_polygon"
    )
    finish_client = client_node.create_client(
        Trigger, "/manual_map_annotator/finish_polygon"
    )
    executor = SingleThreadedExecutor()
    executor.add_node(annotator)
    executor.add_node(client_node)

    try:
        spin_until(executor, start_client.service_is_ready)
        spin_until(executor, finish_client.service_is_ready)
        spin_until(
            executor, lambda: clicked_publisher.get_subscription_count() > 0
        )
        spin_until(executor, lambda: map_publisher.get_subscription_count() > 0)

        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.info.width = 5
        grid.info.height = 5
        grid.info.resolution = 1.0
        grid.info.origin.orientation.w = 1.0
        grid.data = [0] * 25
        map_publisher.publish(grid)

        start_future = start_client.call_async(Trigger.Request())
        spin_until(executor, start_future.done)
        assert start_future.result().success

        for vertex_count, (x_value, y_value) in enumerate(
            [(1.0, 1.0), (3.0, 1.0), (3.0, 3.0)], start=1
        ):
            point = PointStamped()
            point.header.frame_id = "map"
            point.point.x = x_value
            point.point.y = y_value
            clicked_publisher.publish(point)
            spin_until(
                executor,
                lambda: annotator.draft is not None
                and len(annotator.draft.polygon) == vertex_count,
            )

        finish_future = finish_client.call_async(Trigger.Request())
        spin_until(executor, finish_future.done)
        assert finish_future.result().success
        spin_until(executor, lambda: bool(received_markers))
        spin_until(executor, lambda: bool(received_masks))

        assert output_path.exists()
        assert "obstacle_001" in output_path.read_text(encoding="utf-8")
        assert any(value == 100 for value in received_masks[-1].data)
        assert any(
            marker.text == "obstacle_001"
            for marker in received_markers[-1].markers
        )
    finally:
        executor.remove_node(client_node)
        executor.remove_node(annotator)
        client_node.destroy_node()
        annotator.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
