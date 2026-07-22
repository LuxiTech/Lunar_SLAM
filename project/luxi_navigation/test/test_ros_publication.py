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

"""ROS transport tests for published velocity messages."""

import time

import pytest
import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from luxi_navigation.keyboard_teleop import KeyboardTeleop


@pytest.fixture
def ros_nodes():
    rclpy.init()
    teleop = KeyboardTeleop(
        parameter_overrides=[
            Parameter("keyboard_enabled", value=False),
            Parameter("cmd_vel_topic", value="/luxi_navigation/test_cmd_vel"),
            Parameter("linear_speed", value=0.2),
            Parameter("angular_speed", value=0.6),
            Parameter("command_timeout", value=0.1),
        ]
    )
    observer = Node("luxi_navigation_test_observer")
    received = []
    observer.create_subscription(
        Twist, "/luxi_navigation/test_cmd_vel", received.append, 10
    )
    executor = SingleThreadedExecutor()
    executor.add_node(teleop)
    executor.add_node(observer)

    # Allow DDS endpoint discovery before the first assertion.
    deadline = time.monotonic() + 0.5
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)

    yield teleop, executor, received

    executor.remove_node(observer)
    executor.remove_node(teleop)
    observer.destroy_node()
    teleop.destroy_node()
    executor.shutdown()
    if rclpy.ok():
        rclpy.shutdown()


def wait_for_message(executor, received, previous_count):
    deadline = time.monotonic() + 1.0
    while len(received) <= previous_count and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
    assert len(received) > previous_count
    return received[-1]


@pytest.mark.parametrize(
    "key, expected_linear, expected_angular",
    [
        ("w", 0.2, 0.0),
        ("s", -0.2, 0.0),
        ("a", 0.0, 0.6),
        ("d", 0.0, -0.6),
        (" ", 0.0, 0.0),
    ],
)
def test_published_twist_for_each_key(
    ros_nodes, key, expected_linear, expected_angular
):
    teleop, executor, received = ros_nodes
    previous_count = len(received)
    teleop.apply_key(key)
    teleop.publish_once()
    message = wait_for_message(executor, received, previous_count)
    assert message.linear.x == pytest.approx(expected_linear)
    assert message.linear.y == 0.0
    assert message.angular.z == pytest.approx(expected_angular)


def test_timeout_publishes_stop(ros_nodes):
    teleop, executor, received = ros_nodes
    teleop.apply_key("w", timestamp=10.0)
    assert teleop.enforce_timeout(timestamp=10.2)
    previous_count = len(received)
    teleop.publish_once()
    message = wait_for_message(executor, received, previous_count)
    assert message.linear.x == 0.0
    assert message.angular.z == 0.0
