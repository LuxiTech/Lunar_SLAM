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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float64

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
    maps_root = tmp_path / "maps"
    static_cloud = maps_root / "octo_maps/map011_octomap/map011_cloud.ply"
    static_cloud.parent.mkdir(parents=True)
    database = maps_root / "rtab_maps/map011.db"
    database.parent.mkdir(parents=True)
    database.write_bytes(b"0123456789abcdef")
    (static_cloud.parent / "map011.bt").write_bytes(b"octomap")
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
        Parameter("maps_root", value=str(maps_root)),
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

        with urlopen(base_url + "/map_projection.js", timeout=2.0) as response:
            assert response.status == 200
            assert b"unprojectGround" in response.read()

        with urlopen(base_url + "/map_portal.js", timeout=2.0) as response:
            assert response.status == 200
            assert b"/api/maps" in response.read()

        with urlopen(base_url + "/api/maps", timeout=2.0) as response:
            catalog = json.load(response)
            assert catalog["maps"][0]["id"] == "map011"
            assert "database_path" not in catalog["maps"][0]
            assert any(
                item["layer"] == "database"
                for item in catalog["maps"][0]["files"]
            )

        with urlopen(base_url + "/api/capabilities", timeout=2.0) as response:
            capabilities = json.load(response)
            assert any(
                operation["path"] == "/api/maps/{map_id}/preview/cloud"
                for operation in capabilities["operations"]
            )

        range_request = Request(
            base_url + "/api/maps/map011/download/database",
            headers={"Range": "bytes=4-9"},
        )
        with urlopen(range_request, timeout=2.0) as response:
            assert response.status == 206
            assert response.headers["Content-Range"] == "bytes 4-9/16"
            assert response.headers["Accept-Ranges"] == "bytes"
            assert response.read() == b"456789"

        with urlopen(base_url + "/", timeout=2.0) as response:
            page = response.read()
            assert b"semanticSaveButton" in page
            assert b"semanticGroundZ" in page
            assert b"navigationShowSemantics" in page
            assert b"navigationShowTraversable" in page
            assert b"navigationShowCostmap" in page
            assert b"navigationFilterButton" in page
            assert b"navigationStartButton" in page
            assert b"navigationHaltButton" in page
            assert b"robotControlToggle" in page
            assert b"robotBatteryState" in page
            assert b"bodyHeight" in page
            assert b"imuCalibrationButton" in page
            assert b"cameraProfileSelect" in page
            assert b"cameraStartButton" in page
            assert b"cameraStopButton" in page

        with urlopen(base_url + "/app.js", timeout=2.0) as response:
            assert response.headers["Cache-Control"] == "no-store"
            app = response.read()
            assert b"/api/semantic/save" in app
            assert b"applySemanticBrush" in app
            assert b"semanticAnnotation && navigationShowSemantics.checked" in app
            assert b"/api/navigation/terrain" in app
            assert b"[...maps].reverse().find" in app
            assert b"navigationDrag.yaw +" in app
            assert b'filtered: navigationUseFiltered' in app
            assert b'api("/api/navigation/start")' in app
            assert b'api("/api/navigation/halt")' in app
            assert b"path.stale" in app
            assert "规划失败".encode("utf-8") in app
            assert b"nearestTraversableGoal(event)" in app
            assert b'api("/api/navigation/goal", goal)' in app
            assert b"selectedGoal.x, selectedGoal.y, selectedGoal.z" in app
            assert "地表z=".encode("utf-8") in app
            assert b'api("/api/robot/control", {active: requested})' in app
            assert b'api("/api/robot/height", {height: Number(bodyHeightInput.value)})' in app
            assert b'api("/api/imu/calibrate")' in app
            assert b'"/api/camera/start"' in app
            assert b'api("/api/camera/stop"' in app
            assert b"controlClientId" in app
            assert b"MATCHES_LOW" in app

        imu_calibration = node.status()["imu_calibration"]
        assert imu_calibration["service_available"] is False
        try:
            _post(base_url, "/api/imu/calibrate", {})
            assert False, "unavailable IMU calibration must reject requests"
        except HTTPError as error:
            assert error.code == 409

        camera = node.status()["camera"]
        assert [profile["id"] for profile in camera["profiles"]] == [
            "d455", "d435i", "hik",
        ]
        try:
            _post(base_url, "/api/camera/start", {"profile": "unknown"})
            assert False, "an unknown camera profile must be rejected"
        except HTTPError as error:
            assert error.code == 409

        robot_control = node.status()["robot_control"]
        assert robot_control["state"] == "disabled"
        assert robot_control["posture"] == "offline"
        assert robot_control["control_ready"] is False
        try:
            _post(base_url, "/api/robot/control", {"active": True})
            assert False, "disabled D1 control must reject enable requests"
        except HTTPError as error:
            assert error.code == 409

        try:
            _post(base_url, "/api/robot/height", {"height": 0.0})
            assert False, "disabled D1 control must reject height requests"
        except HTTPError as error:
            assert error.code == 409
        try:
            _post(base_url, "/api/robot/height", {"height": 10.0})
            assert False, "out-of-range body height must be rejected"
        except HTTPError as error:
            assert error.code == 400

        try:
            urlopen(base_url + "/api/preview/cloud", timeout=2.0)
            assert False, "live mapping cloud must be disabled by default"
        except HTTPError as error:
            assert error.code == 404

        assert node._load_navigation_cloud("map011", str(static_cloud)) == ""
        with urlopen(
            base_url + "/api/maps/map011/preview/cloud", timeout=2.0
        ) as response:
            assert response.status == 200
            assert json.load(response)["cloud"]["map_id"] == "map011"
        with urlopen(base_url + "/api/navigation/cloud", timeout=2.0) as response:
            assert response.status == 200
            saved_cloud = json.load(response)["cloud"]
            assert saved_cloud == {
                "map_id": "map011",
                "variant": "original",
                "point_count": 2,
                "error": None,
                "points": [[0.0, 1.0, 2.0, 3, 4, 5], [6.0, 7.0, 8.0, 9, 10, 11]],
            }
        with urlopen(base_url + "/api/navigation/terrain", timeout=2.0) as response:
            assert response.status == 200
            terrain = json.load(response)["terrain"]
            assert terrain["traversable_points"] == []
            assert terrain["obstacle_points"] == []
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

        try:
            _post(
                base_url,
                "/api/navigation/load_map",
                {"map_id": "map011", "filtered": "true"},
            )
            assert False, "a non-boolean filtered selector must be rejected"
        except HTTPError as error:
            assert error.code == 400

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
        owner = "browser-owner-0001"
        stale = "browser-stale-0002"
        _post(
            base_url,
            "/api/cmd_vel",
            {"linear_x": 0.1, "client_id": owner},
        )
        assert _wait_for(
            lambda: any(message.linear.x == 0.1 for message in messages)
        )
        try:
            urlopen(base_url + "/api/navigation/cloud", timeout=2.0)
            assert False, "large previews must pause during manual control"
        except HTTPError as error:
            assert error.code == 409
        _, ignored = _post(
            base_url,
            "/api/stop",
            {"client_id": stale},
        )
        assert ignored["ignored"] is True
        assert node.status()["state"] == "moving"
        _, legacy_stop = _post(base_url, "/api/stop", {})
        assert legacy_stop["ignored"] is True
        try:
            _post(base_url, "/api/cmd_vel", {"linear_x": 0.1})
            assert False, "an old browser must not overwrite a modern lease"
        except HTTPError as error:
            assert error.code == 423
        try:
            _post(
                base_url,
                "/api/cmd_vel",
                {"linear_x": 0.1, "client_id": stale},
            )
            assert False, "a second browser must not steal a live lease"
        except HTTPError as error:
            assert error.code == 423
        _, stopped = _post(
            base_url,
            "/api/stop",
            {"client_id": owner},
        )
        assert stopped["stopped"] is True
        assert _wait_for(lambda: messages and messages[-1].linear.x == 0.0)
        with urlopen(base_url + "/api/navigation/cloud", timeout=2.0) as response:
            assert response.status == 200
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


def test_d1_battery_status_and_height_request_reach_ros():
    source_web = Path(__file__).parents[1] / "web"
    rclpy.init()
    node = WebControlNode(parameter_overrides=[
        Parameter("http_port", value=0),
        Parameter("bind_address", value="127.0.0.1"),
        Parameter("cmd_vel_topic", value="/web_d1_test/cmd_vel"),
        Parameter("web_root", value=str(source_web)),
        Parameter("enable_d1_control", value=True),
        Parameter("d1_battery1_topic", value="/web_d1_test/status/battery1"),
        Parameter("d1_battery2_topic", value="/web_d1_test/status/battery2"),
        Parameter(
            "d1_body_height_command_topic",
            value="/web_d1_test/command/body_height",
        ),
        Parameter(
            "d1_body_height_status_topic",
            value="/web_d1_test/status/body_height",
        ),
        Parameter("d1_fsm_topic", value="/web_d1_test/fsm"),
        Parameter("enable_mapping_control", value=False),
        Parameter("enable_navigation_control", value=False),
        Parameter("enable_preview", value=False),
    ])
    observer = rclpy.create_node("web_d1_test_observer")
    battery_qos = QoSProfile(
        depth=5,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    battery_publisher = observer.create_publisher(
        BatteryState, "/web_d1_test/status/battery1", battery_qos
    )
    height_messages = []
    observer.create_subscription(
        Float64,
        "/web_d1_test/command/body_height",
        height_messages.append,
        10,
    )
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor.add_node(observer)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    base_url = f"http://127.0.0.1:{node.http_port}"

    try:
        assert _wait_for(lambda: battery_publisher.get_subscription_count() > 0)
        battery = BatteryState()
        battery.percentage = 0.76
        battery.voltage = 48.2
        for _ in range(3):
            battery_publisher.publish(battery)
            time.sleep(0.03)
        assert _wait_for(
            lambda: node.d1_control_status()["battery"]["online"]
        )
        status = node.d1_control_status()["battery"]
        assert status["percentage"] == 76.0
        assert status["packs"][0]["voltage"] == 48.2

        # Feature availability must never make the browser switch look active.
        # The switch represents feedback-verified SDK control, not merely a
        # configured manager or a latched physical posture.
        now = time.monotonic()
        node.count_publishers = lambda _topic: 1
        node.d1_control.status = lambda: {
            "enabled": True,
            "state": "inactive",
            "active": False,
            "transitioning": False,
            "last_error": "",
            "log_path": "",
        }
        with node._d1_status_lock:
            node._d1_fsm_state = "idle"
            node._d1_fsm_received_at = now
            node._d1_controller_mode = "biped"
            node._d1_controller_received_at = now
            node._d1_sdk_active = False
            node._d1_sdk_received_at = now
        prone = node.d1_control_status()
        assert prone["state"] == "inactive"
        assert prone["posture"] == "prone"
        assert prone["active"] is False
        assert prone["control_ready"] is False

        with node._d1_status_lock:
            node._d1_fsm_state = "loco"
            node._d1_fsm_received_at = time.monotonic()
        standing = node.d1_control_status()
        assert standing["state"] == "standing_uncontrolled"
        assert standing["active"] is False

        node.d1_control_status = lambda: {
            "enabled": True,
            "control_ready": True,
            "controller_mode": "quadruped",
        }
        try:
            _post(base_url, "/api/robot/height", {"height": 4.5})
            raise AssertionError("non-biped height request should be rejected")
        except HTTPError as error:
            assert error.code == 409
            result = json.load(error)
        assert "双足 LQR 模式" in result["error"]
        assert not height_messages

        node.d1_control_status = lambda: {
            "enabled": True,
            "control_ready": True,
            "controller_mode": "biped",
        }
        response_status, result = _post(
            base_url, "/api/robot/height", {"height": 4.5}
        )
        assert response_status == 200
        assert result["ok"] is True
        assert _wait_for(
            lambda: height_messages
            and abs(height_messages[-1].data - 4.5) < 1.0e-9
        )
    finally:
        node.close()
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        observer.destroy_node()
        node.destroy_node()
        rclpy.shutdown()
