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

"""End-to-end test from HTTP request to observed ROS Twist messages."""

import json
from pathlib import Path
import threading
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter

from luxi_web_control.web_control_node import WebControlNode


def _post(base_url, path, body):
    request = Request(
        base_url + path,
        data=json.dumps(body).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urlopen(request, timeout=2.0) as response:
        return response.status, json.load(response)


def _wait_for(predicate, timeout=2.5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return False


def test_http_command_watchdog_and_estop_reach_ros(tmp_path):
    source_web = Path(__file__).parents[1] / "web"
    static_cloud = tmp_path / "map011_cloud.ply"
    static_cloud.write_text(
        "ply\nformat ascii 1.0\nelement vertex 2\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property uchar red\nproperty uchar green\nproperty uchar blue\n"
        "end_header\n0 1 2 3 4 5\n6 7 8 9 10 11\n",
        encoding="ascii",
    )
    rclpy.init()
    node = WebControlNode(parameter_overrides=[
        Parameter("http_port", value=0),
        Parameter("bind_address", value="127.0.0.1"),
        Parameter("cmd_vel_topic", value="/web_control_test/cmd_vel"),
        Parameter("command_timeout", value=0.25),
        Parameter("max_linear_x", value=0.2),
        Parameter("max_angular_z", value=0.5),
        Parameter("web_root", value=str(source_web)),
        Parameter("enable_mapping_control", value=False),
        Parameter("enable_preview", value=False),
    ])
    observer = rclpy.create_node("web_control_test_observer")
    messages = []
    observer.create_subscription(
        Twist,
        "/web_control_test/cmd_vel",
        lambda message: messages.append(message),
        10,
    )
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.add_node(observer)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    base_url = f"http://127.0.0.1:{node.http_port}"

    try:
        assert node.web_root == source_web.resolve()
        with urlopen(base_url + "/", timeout=2.0) as response:
            assert response.status == 200
            assert b"Luxi" in response.read()

        with urlopen(base_url + "/api/preview/cloud", timeout=2.0) as response:
            assert response.status == 200
            preview = json.load(response)["cloud"]
            assert preview["point_count"] == 0
            assert preview["points"] == []

        assert node._load_navigation_cloud("map011", str(static_cloud)) == ""
        with urlopen(base_url + "/api/navigation/cloud", timeout=2.0) as response:
            assert response.status == 200
            saved_cloud = json.load(response)["cloud"]
            assert saved_cloud == {
                "map_id": "map011",
                "point_count": 2,
                "error": None,
                "points": [[0.0, 1.0, 2.0, 3, 4, 5], [6.0, 7.0, 8.0, 9, 10, 11]],
            }
        try:
            urlopen(base_url + "/api/preview/rgb", timeout=2.0)
            assert False, "an RGB endpoint without camera input must return 404"
        except HTTPError as error:
            assert error.code == 404

        try:
            _post(base_url, "/api/mapping/start", {})
            assert False, (
                "disabled mapping control should reject a start request"
            )
        except HTTPError as error:
            assert error.code == 409

        status, result = _post(
            base_url,
            "/api/cmd_vel",
            {"linear_x": 1.0, "angular_z": -2.0},
        )
        assert status == 200
        assert result["command"] == {
            "linear_x": 0.2,
            "linear_y": 0.0,
            "angular_z": -0.5,
        }
        assert _wait_for(
            lambda: any(
                abs(message.linear.x - 0.2) < 1.0e-6
                and abs(message.angular.z + 0.5) < 1.0e-6
                for message in messages
            )
        )

        assert _wait_for(
            lambda: node.status()["timed_out"]
            and messages
            and messages[-1].linear.x == 0.0,
            timeout=2.0,
        )

        status, result = _post(base_url, "/api/estop", {"active": True})
        assert status == 200
        assert result["estop_active"] is True
        try:
            _post(base_url, "/api/cmd_vel", {"linear_x": 0.1})
            assert False, (
                "moving command should be rejected while estop is active"
            )
        except HTTPError as error:
            assert error.code == 423

        _post(base_url, "/api/estop", {"active": False})
        _post(base_url, "/api/cmd_vel", {"linear_x": 0.1})
        assert _wait_for(
            lambda: any(message.linear.x == 0.1 for message in messages)
        )
        _post(base_url, "/api/stop", {})
        assert _wait_for(lambda: messages and messages[-1].linear.x == 0.0)
    finally:
        executor.shutdown(timeout_sec=1.0)
        spin_thread.join(timeout=1.0)
        node.close()
        executor.remove_node(observer)
        executor.remove_node(node)
        observer.destroy_node()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
