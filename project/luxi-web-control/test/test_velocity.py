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

"""Unit tests for web velocity validation."""

import math
import os
import struct
import subprocess
import threading
from types import SimpleNamespace

import pytest
import yaml

from pathlib import Path

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path as NavigationPath
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, Float32, String

from luxi_web_control.web_control_node import discover_navigation_maps
from luxi_web_control.web_control_node import D1ControlManager
from luxi_web_control.web_control_node import classify_d1_posture
from luxi_web_control.web_control_node import CameraController
from luxi_web_control.web_control_node import extract_colored_ply_points
from luxi_web_control.web_control_node import extract_sparse_cloud
from luxi_web_control.web_control_node import HlocIndexBuilder
from luxi_web_control.web_control_node import MappingController
from luxi_web_control.web_control_node import is_managed_web_control_command
from luxi_web_control.web_control_node import localization_covariance_ready
from luxi_web_control.web_control_node import localization_pose_summary
from luxi_web_control.web_control_node import make_access_urls
from luxi_web_control.web_control_node import mapping_graph_conflicts
from luxi_web_control.web_control_node import NavigationController
from luxi_web_control.web_control_node import normalize_battery_percentage
from luxi_web_control.web_control_node import newest_changed_map_database
from luxi_web_control.web_control_node import parse_body_height
from luxi_web_control.web_control_node import parse_control_client_id
from luxi_web_control.web_control_node import parse_octomap_point_output
from luxi_web_control.web_control_node import parse_terrain_point_output
from luxi_web_control.web_control_node import parse_navigation_goal
from luxi_web_control.web_control_node import parse_http_byte_range
from luxi_web_control.web_control_node import parse_velocity, VelocityCommand
from luxi_web_control.web_control_node import public_navigation_maps
from luxi_web_control.web_control_node import resolve_d1_http_bind_address
from luxi_web_control.web_control_node import resolve_d1_http_bind_addresses
from luxi_web_control.web_control_node import rtab_map_database_snapshot
from luxi_web_control.web_control_node import validate_d1_http_bind_address
from luxi_web_control.web_control_node import web_api_capabilities
from luxi_web_control.web_control_node import WebControlNode


LIMITS = VelocityCommand(0.25, 0.1, 0.8)
WORKSPACE_ROOT = Path(__file__).resolve().parents[3]


def test_saved_map_browser_preview_is_bounded_by_default():
    config = yaml.safe_load(
        (WORKSPACE_ROOT / "project/luxi-web-control/config/web_control.yaml").read_text(
            encoding="utf-8"
        )
    )
    limit = config["web_control"]["ros__parameters"]["max_saved_cloud_points"]
    assert 10_000 <= limit <= 50_000


def test_live_mapping_cloud_is_disabled_and_bounded_if_reenabled():
    config = yaml.safe_load(
        (WORKSPACE_ROOT / "project/luxi-web-control/config/web_control.yaml").read_text(
            encoding="utf-8"
        )
    )
    parameters = config["web_control"]["ros__parameters"]
    assert parameters["enable_cloud_preview"] is False
    assert 100 <= parameters["max_cloud_points"] <= 5000
    assert parameters["command_timeout"] == 0.8


def test_navigation_preview_uses_measured_twenty_five_centimeter_robot_radius():
    config = yaml.safe_load(
        (WORKSPACE_ROOT / "project/luxi-web-control/config/web_control.yaml").read_text(
            encoding="utf-8"
        )
    )
    radius = config["web_control"]["ros__parameters"][
        "navigation_robot_radius"
    ]
    assert radius == 0.25


def test_navigation_costmap_outer_edge_is_sixty_centimeters():
    config = yaml.safe_load(
        (WORKSPACE_ROOT / "project/luxi-web-control/config/web_control.yaml").read_text(
            encoding="utf-8"
        )
    )
    parameters = config["web_control"]["ros__parameters"]
    assert parameters["navigation_robot_radius"] == 0.25
    assert parameters["navigation_costmap_margin"] == 0.35
    assert parameters["navigation_robot_radius"] + parameters["navigation_costmap_margin"] == 0.60


def test_mapping_auto_filter_is_enabled_and_visible_in_both_pages():
    web_root = WORKSPACE_ROOT / "project/luxi-web-control/web"
    config = yaml.safe_load(
        (WORKSPACE_ROOT / "project/luxi-web-control/config/web_control.yaml").read_text(
            encoding="utf-8"
        )
    )
    assert config["web_control"]["ros__parameters"]["auto_filter_after_mapping"] is True
    app = (web_root / "app.js").read_text(encoding="utf-8")
    portal = (web_root / "map_portal.js").read_text(encoding="utf-8")
    assert "showAutomaticallyFilteredMap" in app
    assert "showAutomaticallyFilteredMap" in portal


def test_newest_changed_map_database_tracks_mapping_output(tmp_path):
    maps_root = tmp_path / "maps"
    rtab_maps = maps_root / "rtab_maps"
    rtab_maps.mkdir(parents=True)
    existing = rtab_maps / "map040.db"
    existing.write_bytes(b"old")
    before = rtab_map_database_snapshot(maps_root)

    created = rtab_maps / "map041.db"
    created.write_bytes(b"new map")
    after = rtab_map_database_snapshot(maps_root)

    assert newest_changed_map_database(before, after) == created.resolve()


def test_mapping_postprocess_generates_filtered_variant():
    node = WebControlNode.__new__(WebControlNode)
    node._navigation_map_operation_lock = threading.Lock()
    node._mapping_postprocess_lock = threading.Lock()
    node._mapping_postprocess = {
        "enabled": True,
        "state": "waiting",
        "map_id": "map041",
        "message": "",
    }
    node.navigation_maps = lambda: [{"id": "map041"}]
    calls = []

    def convert(record, filtered):
        calls.append((record["id"], filtered))
        return True, "filtered map ready"

    node._convert_navigation_map = convert
    node._run_mapping_postprocess("map041")

    assert calls == [("map041", True)]
    assert node._mapping_postprocess["state"] == "completed"
    assert node._mapping_postprocess["message"] == "filtered map ready"


def test_mapping_keeps_3d_cloud_with_planar_test_trajectory():
    launch_source = (
        WORKSPACE_ROOT
        / "project/luxi_RTAB_Map/launch/rgbd_mapping_learned.launch.py"
    ).read_text(encoding="utf-8")

    assert 'LaunchConfiguration("planar_motion")' in launch_source
    assert '" --Grid/3D true "' in launch_source
    assert '"--Reg/Force3DoF "' in launch_source
    assert '" --RGBD/ForceOdom3DoF "' in launch_source


def test_goal_can_be_selected_before_icp_recovers_without_being_sent():
    app = (WORKSPACE_ROOT / "project/luxi-web-control/web/app.js").read_text(
        encoding="utf-8"
    )

    assert 'navigation.state !== "running" || !hasTraversableTerrain' in app
    assert "if (!navigationStatus.planning_localization_ready)" in app
    assert "selectedGoalPending = true" in app
    assert 'await api("/api/navigation/goal", goal)' in app


def test_drive_page_prioritizes_control_over_large_previews():
    web_root = WORKSPACE_ROOT / "project/luxi-web-control/web"
    page = (web_root / "index.html").read_text(encoding="utf-8")
    app = (web_root / "app.js").read_text(encoding="utf-8")

    assert page.index('id="rgbPreview"') < page.index('id="joystickPad"')
    assert 'id="cloudPreview"' not in page
    assert "refreshCloudPreview" not in app
    assert "cancelHeavyPreviewRequests" in app
    assert "{timeoutMs: 500}" in app
    assert "let robotControlReady = false" in app
    assert "consecutiveCommandTimeouts >= 3" in app
    assert "const RGB_PREVIEW_INTERVAL_MS = 100" in app
    assert "setInterval(refreshRgbPreview, RGB_PREVIEW_INTERVAL_MS)" in app
    assert 'window.addEventListener("blur"' not in app


def test_realsense_web_previews_have_headroom_for_ten_hz_delivery():
    for config_name in ("sensor_bringup.yaml", "d455_sensor_bringup.yaml"):
        config = yaml.safe_load(
            (
                WORKSPACE_ROOT
                / "project/luxi_adapter/config"
                / config_name
            ).read_text(encoding="utf-8")
        )
        parameters = config["luxi_adapter"]["ros__parameters"]
        assert parameters["compressed_color_rate"] == 12.0

    source = (
        WORKSPACE_ROOT / "project/luxi_adapter/src/sensor_adapter.cpp"
    ).read_text(encoding="utf-8")
    assert '"compressed_color_rate", 12.0' in source


def test_lekiwi_launch_uses_vehicle_domain_42():
    launch_source = (
        WORKSPACE_ROOT
        / "project/luxi-web-control/launch/lekiwi_web_control.launch.py"
    ).read_text(encoding="utf-8")

    assert 'SetEnvironmentVariable("ROS_DOMAIN_ID", "42")' in launch_source


def test_d1_control_keeps_standard_yaw_direction():
    launch_source = (
        WORKSPACE_ROOT
        / "project/luxi-web-control/launch/lekiwi_web_control.launch.py"
    ).read_text(encoding="utf-8")

    assert '"invert_angular_z": False' in launch_source
    assert 'default_value="/d1/cmd_vel_standard"' in launch_source


def test_d1_control_scripts_do_not_depend_on_ros_daemon_discovery():
    scripts = WORKSPACE_ROOT / "project/slam_d1_bridge/scripts"
    for script_name in (
        "start_slam_d1_bridge.sh",
        "stop_slam_d1_bridge.sh",
    ):
        source = (scripts / script_name).read_text(encoding="utf-8")
        assert 'D1_DISCOVERY_SPIN_TIME="${D1_DISCOVERY_SPIN_TIME:-30.0}"' in source
        assert 'topic info --no-daemon --spin-time "${D1_DISCOVERY_SPIN_TIME}"' in source
        assert 'D1_SERVICE_TIMEOUT="${D1_SERVICE_TIMEOUT:-30s}"' in source
        assert (
            'timeout --signal=INT --kill-after=2s "${D1_SERVICE_TIMEOUT}"'
            in source
        )
        assert "rcl_interfaces/srv/SetParameters" in source
        assert "successful=True" in source


def test_d1_control_scripts_resolve_the_current_workspace():
    bridge_scripts = WORKSPACE_ROOT / "project/slam_d1_bridge/scripts"
    for script_name in (
        "start_slam_d1_bridge.sh",
        "stop_slam_d1_bridge.sh",
    ):
        source = (bridge_scripts / script_name).read_text(encoding="utf-8")
        assert 'readlink -f -- "${BASH_SOURCE[0]}"' in source
        assert "/home/nvidia/Desktop/lunar_slam" not in source

    web_script = (WORKSPACE_ROOT / "scripts/start_d1_web_control.sh").read_text(
        encoding="utf-8"
    )
    assert 'readlink -f -- "${BASH_SOURCE[0]}"' in web_script
    assert 'D1_DISCOVERY_SPIN_TIME="${D1_DISCOVERY_SPIN_TIME:-30.0}"' in web_script
    assert 'topic info --no-daemon --spin-time "${D1_DISCOVERY_SPIN_TIME}"' in web_script
    assert 'source "${D1_LAN_DDS_SETUP}"' in web_script
    assert 'readonly WEB_URL="http://${D1_LAN_ADDRESS}:8080"' in web_script
    assert '"bind_address:=${D1_LAN_ADDRESS}"' in web_script
    assert "/home/nvidia/Desktop/lunar_slam" not in web_script


def test_d1_dds_is_restricted_without_restricting_the_http_server():
    bridge = WORKSPACE_ROOT / "project/slam_d1_bridge"
    setup_source = (bridge / "scripts/setup_d1_lan_dds.sh").read_text(
        encoding="utf-8"
    )
    profile = (bridge / "config/fastdds_lan_only.xml.in").read_text(
        encoding="utf-8"
    )
    launch_source = (
        WORKSPACE_ROOT
        / "project/luxi-web-control/launch/lekiwi_web_control.launch.py"
    ).read_text(encoding="utf-8")

    assert "^192\\.168\\.123\\.[0-9]+$" in setup_source
    assert 'export FASTRTPS_DEFAULT_PROFILES_FILE="${d1_profile_path}"' in setup_source
    assert "<address>127.0.0.1</address>" in profile
    assert "<address>@D1_LAN_ADDRESS@</address>" in profile
    assert "<useBuiltinTransports>false</useBuiltinTransports>" in profile
    assert "FASTDDS_BUILTIN_TRANSPORTS" not in launch_source
    assert 'DeclareLaunchArgument("bind_address", default_value="127.0.0.1")' in launch_source
    assert '"restrict_http_to_d1_lan": "false"' in launch_source


def test_d1_web_control_is_enabled_and_exposes_switch():
    config = yaml.safe_load(
        (WORKSPACE_ROOT / "project/luxi-web-control/config/web_control.yaml")
        .read_text(encoding="utf-8")
    )
    assert config["web_control"]["ros__parameters"]["enable_d1_control"] is True

    page = (WORKSPACE_ROOT / "project/luxi-web-control/web/index.html").read_text(
        encoding="utf-8"
    )
    app = (WORKSPACE_ROOT / "project/luxi-web-control/web/app.js").read_text(
        encoding="utf-8"
    )
    assert 'id="robotControlToggle"' in page
    assert 'api("/api/robot/control", {active: requested})' in app


def test_d1_web_exposes_bridge_battery_and_body_height_interfaces():
    config = yaml.safe_load(
        (WORKSPACE_ROOT / "project/luxi-web-control/config/web_control.yaml")
        .read_text(encoding="utf-8")
    )["web_control"]["ros__parameters"]
    bridge = yaml.safe_load(
        (WORKSPACE_ROOT / "project/slam_d1_bridge/config/slam_d1_bridge.yaml")
        .read_text(encoding="utf-8")
    )["/**"]["ros__parameters"]
    page = (WORKSPACE_ROOT / "project/luxi-web-control/web/index.html").read_text(
        encoding="utf-8"
    )
    app = (WORKSPACE_ROOT / "project/luxi-web-control/web/app.js").read_text(
        encoding="utf-8"
    )

    assert config["d1_battery1_topic"] == "/d15041873/status/battery1"
    assert config["d1_battery2_topic"] == "/d15041873/status/battery2"
    assert bridge["battery1_status_topic"] == "status/battery1"
    assert bridge["battery2_status_topic"] == "status/battery2"
    assert config["d1_body_height_command_topic"] == (
        "/d15041873/command/body_height"
    )
    assert bridge["body_height_command_topic"] == "command/body_height"
    assert config["d1_body_height_minimum"] == bridge["body_height_minimum"]
    assert config["d1_body_height_maximum"] == bridge["body_height_maximum"]
    assert 'id="robotBatteryState"' in page
    assert 'id="bodyHeight"' in page
    assert '单体双足模式·腿部高度' in page
    assert 'max="9" step="0.1"' in page
    assert 'api("/api/robot/height", {height: Number(bodyHeightInput.value)})' in app
    assert "height.supported === true" in app
    assert "heightPercentage.toFixed(0)" in app
    assert bridge["fsm_mode"] == "loco"
    assert bridge["height_fsm_mode"] == "loco"
    assert bridge["body_height_maximum_rate"] == 1.0
    assert bridge["body_height_linear_z_scale"] == 0.03


def test_body_height_validation_and_battery_percentage_normalization():
    assert parse_body_height({"height": 0.0}, 0.0, 9.0) == 0.0
    assert parse_body_height({"height": 9.0}, 0.0, 9.0) == 9.0
    with pytest.raises(ValueError, match="between"):
        parse_body_height({"height": 10.0}, 0.0, 9.0)
    with pytest.raises(ValueError, match="number"):
        parse_body_height({"height": True}, 0.0, 9.0)
    with pytest.raises(ValueError, match="finite"):
        parse_body_height({"height": math.nan}, 0.0, 9.0)

    assert normalize_battery_percentage(0.91) == 91.0
    assert normalize_battery_percentage(91.0) == 91.0
    assert normalize_battery_percentage(110.0) == 100.0
    assert normalize_battery_percentage(math.nan) is None


def test_web_imu_calibration_button_and_mapping_gate_are_present():
    page = (WORKSPACE_ROOT / "project/luxi-web-control/web/index.html").read_text(
        encoding="utf-8"
    )
    app = (WORKSPACE_ROOT / "project/luxi-web-control/web/app.js").read_text(
        encoding="utf-8"
    )
    backend = (
        WORKSPACE_ROOT
        / "project/luxi-web-control/luxi_web_control/web_control_node.py"
    ).read_text(encoding="utf-8")

    assert 'id="imuCalibrationButton"' in page
    assert 'api("/api/imu/calibrate")' in app
    assert 'currentImuCalibration.state !== "calibrated"' in app
    assert 'calibration.get("state") != "calibrated"' in backend
    assert "请先在水平面完成 IMU 一键校准" in backend


def test_d1_control_manager_runs_enable_and_disable_scripts(tmp_path):
    pid_file = tmp_path / "bridge.pid"
    start_script = tmp_path / "start.sh"
    stop_script = tmp_path / "stop.sh"
    start_script.write_text(
        f"#!/bin/sh\necho {os.getpid()} > {pid_file}\n",
        encoding="utf-8",
    )
    stop_script.write_text(
        f"#!/bin/sh\nrm -f {pid_file}\n",
        encoding="utf-8",
    )
    start_script.chmod(0o755)
    stop_script.chmod(0o755)
    manager = D1ControlManager(
        enabled=True,
        start_script=start_script,
        stop_script=stop_script,
        bridge_pid_file=pid_file,
        log_path=tmp_path / "d1.log",
    )
    assert manager.transition_timeout == 90.0

    assert manager.set_active(True)[0]
    for _ in range(100):
        if not manager.status()["transitioning"]:
            break
        threading.Event().wait(0.01)
    assert manager.status()["state"] == "active"

    assert manager.set_active(False)[0]
    for _ in range(100):
        if not manager.status()["transitioning"]:
            break
        threading.Event().wait(0.01)
    assert manager.status()["state"] == "inactive"


@pytest.mark.parametrize(
    ("fsm_state", "posture"),
    [
        ("loco", "standing"),
        ("car", "standing"),
        ("rl_3", "standing"),
        ("transform_up", "standing_up"),
        ("transform_down", "prone"),
        ("idle", "prone"),
        ("", "unknown"),
        ("unexpected", "unknown"),
    ],
)
def test_d1_posture_is_derived_from_actual_fsm(fsm_state, posture):
    assert classify_d1_posture(fsm_state) == posture


def test_map_export_filters_isolated_depth_outliers():
    script = (
        WORKSPACE_ROOT / "project/luxi_RTAB_Map/scripts/export_3d_map.sh"
    ).read_text(encoding="utf-8")
    assert "--opt 0" in script
    assert "--min_range 0.35" in script
    assert "--edge_bleeding_error 0.10" in script
    assert "--noise_radius 0.08" in script
    assert "--noise_k 8" in script


def test_map_export_uses_five_centimeter_octomap_resolution():
    script = (
        WORKSPACE_ROOT / "tools/export_rtabmap_octomap.sh"
    ).read_text(encoding="utf-8")
    assert '"${octomap_path}" 0.05' in script


def test_velocity_is_clamped_to_server_limits():
    command = parse_velocity(
        {"linear_x": 10.0, "linear_y": -3.0, "angular_z": 2.0},
        LIMITS,
    )
    assert command == VelocityCommand(0.25, -0.1, 0.8)


def test_missing_velocity_axes_default_to_zero():
    command = parse_velocity({"linear_x": 0.12}, LIMITS)
    assert command == VelocityCommand(0.12, 0.0, 0.0)


def test_control_client_id_is_optional_but_bounded():
    assert parse_control_client_id({}) == ""
    assert parse_control_client_id({"client_id": "browser-12345678"}) == (
        "browser-12345678"
    )
    with pytest.raises(ValueError):
        parse_control_client_id({"client_id": "short"})
    with pytest.raises(ValueError):
        parse_control_client_id({"client_id": "bad value with spaces"})


def test_d1_http_bind_accepts_only_concrete_loopback_or_wired_addresses():
    validate_d1_http_bind_address("127.0.0.1")
    validate_d1_http_bind_address("192.168.123.66")
    validate_d1_http_bind_address("192.168.137.132")
    with pytest.raises(ValueError):
        validate_d1_http_bind_address("0.0.0.0")
    with pytest.raises(ValueError):
        validate_d1_http_bind_address("198.18.0.1")


def test_d1_wildcard_bind_resolves_to_both_allowed_control_lans():
    addresses = ["192.168.137.132", "192.168.123.66", "198.18.0.1"]
    assert resolve_d1_http_bind_addresses(
        "0.0.0.0",
        addresses,
    ) == ["192.168.123.66", "192.168.137.132"]
    assert resolve_d1_http_bind_address("0.0.0.0", addresses) == (
        "192.168.123.66"
    )
    assert resolve_d1_http_bind_address(
        "192.168.137.132",
        ["192.168.123.66"],
    ) == "192.168.137.132"
    with pytest.raises(ValueError, match="no 192.168.123.x or 192.168.137.x"):
        resolve_d1_http_bind_addresses(
            "0.0.0.0",
            ["10.0.0.2", "198.18.0.1"],
        )


def test_hloc_diagnostics_are_bounded_and_json_safe():
    node = SimpleNamespace(
        _navigation_lock=threading.Lock(),
        _hloc_diagnostics=None,
        _hloc_diagnostics_received_at=None,
    )
    message = String()
    message.data = (
        '{"reason":"MATCHES_LOW","matches":3,'
        '"median_depth_residual":Infinity,"unexpected":"ignored"}'
    )
    WebControlNode._on_navigation_hloc_diagnostics(node, message)
    assert node._hloc_diagnostics == {
        "reason": "MATCHES_LOW",
        "matches": 3,
        "median_depth_residual": None,
    }
    assert node._hloc_diagnostics_received_at is not None


def test_wildcard_bind_address_expands_to_lan_urls():
    urls = make_access_urls(
        "0.0.0.0",
        8080,
        ["10.0.0.2", "192.168.123.66"],
    )
    assert urls == [
        "http://10.0.0.2:8080",
        "http://192.168.123.66:8080",
    ]

    source = (
        WORKSPACE_ROOT
        / "project/luxi-web-control/luxi_web_control/web_control_node.py"
    ).read_text(encoding="utf-8")
    assert "self.access_urls = make_access_urls(" in source


def test_specific_bind_address_is_printed_directly():
    assert make_access_urls("127.0.0.1", 9000) == [
        "http://127.0.0.1:9000"
    ]


def test_only_our_own_web_control_command_can_be_auto_stopped():
    assert is_managed_web_control_command(
        "/workspace/install/luxi_web_control/lib/luxi_web_control/"
        "web_control_node"
    )
    assert not is_managed_web_control_command("python3 -m http.server 8080")


def test_web_launch_shuts_down_when_the_web_node_exits():
    launch_source = (
        WORKSPACE_ROOT / "project/luxi-web-control/launch/web_control.launch.py"
    ).read_text(encoding="utf-8")
    assert 'on_exit=Shutdown(reason="web control node exited")' in launch_source


def test_web_main_restores_sigint_for_automatic_port_takeover():
    source = (
        WORKSPACE_ROOT
        / "project/luxi-web-control/luxi_web_control/web_control_node.py"
    ).read_text(encoding="utf-8")
    assert "signal.signal(signal.SIGINT, signal.default_int_handler)" in source


def test_documented_cleanup_uses_the_bounded_workspace_script():
    readme = (
        WORKSPACE_ROOT / "project/luxi-web-control/README.md"
    ).read_text(encoding="utf-8")
    cleanup = WORKSPACE_ROOT / "scripts/stop_luxi_system.sh"
    source = cleanup.read_text(encoding="utf-8")
    assert "bash scripts/stop_luxi_system.sh" in readme
    assert "post_if_available /api/stop" in source
    assert "post_if_available /api/mapping/stop" in source
    assert "post_if_available /api/navigation/stop" in source
    assert "post_if_available /api/camera/stop" in source
    assert "signal_matches INT" in source
    assert "signal_matches TERM" in source
    assert "luxi_[^/[:space:]]+" in source
    assert "slam_d1_bridge" in source
    assert "rtabmap_(slam|odom|sync|viz)" in source
    assert "lunar_d455_bringup" in source
    assert "[d]455\\.launch\\.py" in source
    assert "aved_map_navigation" in source
    assert "request_robot_shutdown" in source
    assert "/api/robot/control" in source
    assert 'WEB_URLS=("http://127.0.0.1:8080")' in source
    assert "ip -4 -o addr show scope global" in source
    assert '"${candidate_url}/api/status"' in source
    assert "--max-time 1" in source
    assert "remove_stale_pid_files" in source
    assert "sport = :8080" in source


def test_system_restart_script_stops_everything_before_relaunching_web():
    restart = WORKSPACE_ROOT / "scripts/restart_luxi_system.sh"
    source = restart.read_text(encoding="utf-8")

    assert os.access(restart, os.X_OK)
    stop_position = source.index('bash "${WORKSPACE}/scripts/stop_luxi_system.sh"')
    launch_position = source.index(
        "setsid ros2 launch luxi_web_control lekiwi_web_control.launch.py"
    )
    assert stop_position < launch_position
    assert '"bind_address:=${BIND_ADDRESS}"' in source
    assert '"http_port:=${HTTP_PORT}"' in source
    assert '"web_ui_mode:=${WEB_UI_MODE}"' in source
    assert 'source "${WORKSPACE}/install/setup.bash"' in source
    assert 'echo "${launch_pid}" >"${PID_FILE}"' in source


def test_system_restart_worker_is_detached_and_preserves_web_arguments(
    tmp_path, monkeypatch
):
    restart = tmp_path / "scripts/restart_luxi_system.sh"
    restart.parent.mkdir()
    restart.write_text("#!/usr/bin/env bash\n", encoding="utf-8")
    restart.chmod(0o755)
    node = WebControlNode.__new__(WebControlNode)
    node.workspace_root = tmp_path
    node.bind_address = "0.0.0.0"
    node.http_port = 8080
    node.web_ui_mode = "map_portal"
    node._restart_lock = threading.Lock()
    node._restart_requested = False
    stops = []
    node.stop_motion = lambda force: stops.append(force)
    calls = []
    monkeypatch.setattr(
        subprocess,
        "Popen",
        lambda command, **options: calls.append((command, options)),
    )

    accepted, message = node.request_system_restart()

    assert accepted is True
    assert "安全重启" in message
    assert stops == [True]
    command, options = calls[0]
    assert command[:3] == ["/bin/bash", "-c", command[2]]
    assert str(restart) in command
    assert "--bind-address" in command and "0.0.0.0" in command
    assert "--http-port" in command and "8080" in command
    assert "--web-ui-mode" in command and "map_portal" in command
    assert options["start_new_session"] is True
    assert options["close_fds"] is True


def test_map_export_sanitizes_camera_sdk_libraries():
    script = WORKSPACE_ROOT / "tools/export_rtabmap_octomap.sh"
    environment = os.environ.copy()
    environment["LD_LIBRARY_PATH"] = (
        "/opt/MVS/lib/aarch64:/usr/local/cuda/lib64:/opt/MVS/lib/64"
    )
    result = subprocess.run(
        [
            "/bin/bash",
            "-c",
            f"source {script}; sanitize_mvs_library_path; "
            'printf %s "$LD_LIBRARY_PATH"',
        ],
        check=True,
        capture_output=True,
        text=True,
        env=environment,
    )
    assert result.stdout == "/usr/local/cuda/lib64"


def test_mapping_start_only_requires_workspace_setup(tmp_path):
    workspace_setup = Path(tmp_path / "workspace_setup.bash")
    workspace_setup.touch()
    controller = MappingController(
        enabled=True,
        package="luxi_rtab_map",
        launch_file="rgbd_mapping.launch.py",
        rmw_implementation="rmw_cyclonedds_cpp",
        sensor_setup=None,
        workspace_setup=workspace_setup,
        log_path=Path(tmp_path / "mapping.log"),
    )
    command = controller._command()[-1]
    assert f"source {workspace_setup}" in command
    assert "device/D435i" not in command
    assert "new_map:=true" in command
    assert "planar_motion:=true" in command


def test_camera_controller_uses_allow_list_and_project_sensor_bringup(tmp_path):
    workspace_setup = Path(tmp_path / "setup.bash")
    workspace_setup.touch()
    controller = CameraController(
        enabled=True,
        profiles=("d455", "d435i", "hik"),
        default_profile="d455",
        package="luxi_adapter",
        launch_file="sensor_bringup.launch.py",
        workspace_setup=workspace_setup,
        log_path=Path(tmp_path / "camera.log"),
        rmw_implementation="rmw_fastrtps_cpp",
    )

    command = controller._command("d455")[-1]
    assert f"source {workspace_setup}" in command
    assert "ros2 launch luxi_adapter sensor_bringup.launch.py hardware:=d455" in command
    assert "export ROS_DOMAIN_ID=42" in command
    assert "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" in command
    started, message = controller.switch("unknown")
    assert started is False
    assert "unsupported camera profile" in message
    status = controller.status()
    assert [profile["id"] for profile in status["profiles"]] == [
        "d455", "d435i", "hik",
    ]


def test_camera_controls_are_in_the_first_web_column():
    page = (
        WORKSPACE_ROOT / "project/luxi-web-control/web/index.html"
    ).read_text(encoding="utf-8")

    camera = page.index('class="mapping-panel camera-control-panel"')
    preview = page.index('class="drive-rgb-preview"')
    settings = page.index('class="panel settings-panel"')
    assert camera < preview < settings


def test_mapping_graph_conflicts_detects_external_slam_nodes():
    assert mapping_graph_conflicts([
        ("web_control", "/"),
        ("luxi_visual_frontend", "/"),
        ("rtabmap", "/rtabmap"),
    ]) == ["/luxi_visual_frontend", "/rtabmap/rtabmap"]
    assert mapping_graph_conflicts([
        ("web_control", "/"),
        ("stereo_depth_node", "/"),
    ]) == []


def test_mapping_status_extracts_error_from_launch_log(tmp_path):
    log_path = Path(tmp_path / "mapping.log")
    log_path.write_text(
        "[INFO] mapping started\n[ERROR] camera input is unavailable\n",
        encoding="utf-8",
    )
    controller = MappingController(
        enabled=True,
        package="luxi_rtab_map",
        launch_file="rgbd_mapping.launch.py",
        rmw_implementation="rmw_cyclonedds_cpp",
        sensor_setup=None,
        workspace_setup=Path(tmp_path / "workspace_setup.bash"),
        log_path=log_path,
    )
    assert controller._latest_log_error() == (
        "[ERROR] camera input is unavailable"
    )


def test_mapping_status_reports_child_failure_when_launch_exits_zero(tmp_path):
    class ExitedLaunch:
        def poll(self):
            return 0

    log_path = Path(tmp_path / "mapping.log")
    log_path.write_text(
        "[ERROR] [rtabmap]: process has died\n",
        encoding="utf-8",
    )
    controller = MappingController(
        enabled=True,
        package="luxi_rtab_map",
        launch_file="rgbd_mapping.launch.py",
        rmw_implementation="rmw_cyclonedds_cpp",
        sensor_setup=None,
        workspace_setup=Path(tmp_path / "workspace_setup.bash"),
        log_path=log_path,
    )
    controller._process = ExitedLaunch()

    status = controller.status()

    assert status["state"] == "failed"
    assert status["last_exit_code"] == 0
    assert status["last_error"] == "[ERROR] [rtabmap]: process has died"


def test_navigation_requires_exported_cloud_and_passes_it_to_launch(tmp_path):
    setup = Path(tmp_path / "setup.bash")
    database = Path(tmp_path / "map.db")
    octomap = Path(tmp_path / "map.bt")
    cloud = Path(tmp_path / "map_cloud.ply")
    semantic = Path(tmp_path / "annotations.json")
    hloc_map = Path(tmp_path / "hloc_map")
    hloc_map.mkdir()
    (hloc_map / "metadata.yaml").touch()
    for path in (setup, database, octomap):
        path.touch()
    controller = NavigationController(
        enabled=True,
        package="luxi_3d_navigation",
        launch_file="saved_map_navigation.launch.py",
        rmw_implementation="rmw_cyclonedds_cpp",
        sensor_setup=None,
        workspace_setup=setup,
        octomap_library_path=Path(tmp_path),
        log_path=Path(tmp_path / "navigation.log"),
    )

    started, message = controller.start(
        "map012", database, octomap, cloud, hloc_map, semantic
    )
    assert not started
    assert str(cloud) in message

    cloud.touch()
    command = controller._command(database, octomap, cloud, hloc_map, semantic)
    assert f"cloud_path:={cloud}" in command[-1]
    assert f"hloc_map_directory:={hloc_map}" in command[-1]
    assert f"semantic_path:={semantic}" in command[-1]


def test_sparse_cloud_extracts_finite_xyzrgb_points():
    cloud = PointCloud2()
    cloud.width = 3
    cloud.height = 1
    cloud.is_bigendian = False
    cloud.point_step = 20
    cloud.row_step = cloud.width * cloud.point_step
    cloud.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=16, datatype=PointField.FLOAT32, count=1),
    ]
    rgb = struct.unpack("<f", struct.pack("<I", 0x00112233))[0]
    invalid = struct.unpack("<f", struct.pack("<I", 0x00abcdef))[0]
    cloud.data = b"".join([
        struct.pack("<fff4xf", 1.0, 2.0, 3.0, rgb),
        struct.pack("<fff4xf", math.nan, 0.0, 1.0, invalid),
        struct.pack("<fff4xf", -1.0, 0.5, 0.25, rgb),
    ])

    expected = [
        (1.0, 2.0, 3.0, 17, 34, 51),
        (-1.0, 0.5, 0.25, 17, 34, 51),
    ]
    assert extract_sparse_cloud(cloud, 10) == expected
    assert extract_sparse_cloud(cloud, 0) == expected


def test_hloc_index_builder_runs_export_and_cuda_model_build(
    tmp_path,
    monkeypatch,
):
    monkeypatch.setenv(
        "LD_LIBRARY_PATH",
        "/opt/MVS/lib/aarch64:/usr/local/cuda/lib64:/opt/MVS/lib/64",
    )
    exporter = tmp_path / "rtab_hloc_exporter"
    model_builder = tmp_path / "build_reference_model.py"
    database = tmp_path / "map021.db"
    output = tmp_path / "hloc_maps" / "map021"
    for path in (exporter, model_builder, database):
        path.touch()
    calls = []

    def fake_run(command, **kwargs):
        calls.append((command, kwargs))
        if command[0] == str(model_builder):
            output.mkdir(parents=True, exist_ok=True)
            (output / "metadata.yaml").write_text(
                "schema_version: 1\n",
                encoding="utf-8",
            )
        return type("Result", (), {"stdout": "ok"})()

    monkeypatch.setattr(
        "luxi_web_control.web_control_node.subprocess.run",
        fake_run,
    )
    builder = HlocIndexBuilder(True, exporter, model_builder, 30.0)

    built, message = builder.build("map021", database, output)

    assert built
    assert "CUDA" in message
    assert [call[0][0] for call in calls] == [
        str(exporter),
        str(model_builder),
    ]
    assert calls[1][0][-1] == "--overwrite"
    assert calls[1][1]["env"]["LUXI_HLOC_RUNTIME"] == "gpu"
    assert all(
        call[1]["env"]["LD_LIBRARY_PATH"] == "/usr/local/cuda/lib64"
        for call in calls
    )


def test_navigation_maps_require_database_and_octomap_pair(tmp_path):
    (tmp_path / "rtab_maps").mkdir()
    (tmp_path / "octo_maps" / "map011_octomap").mkdir(parents=True)
    (tmp_path / "octo_maps" / "map011_filtered_octomap").mkdir(parents=True)
    (tmp_path / "rtab_maps" / "map011.db").write_bytes(b"database")
    (tmp_path / "rtab_maps" / "map012.db").write_bytes(b"database")
    (tmp_path / "hloc_maps" / "map011").mkdir(parents=True)
    (tmp_path / "hloc_maps" / "map011" / "metadata.yaml").write_text(
        "schema_version: 1\n",
        encoding="utf-8",
    )
    (tmp_path / "octo_maps" / "map011_octomap" / "map011.bt").write_bytes(b"octomap")
    cloud_path = tmp_path / "octo_maps" / "map011_octomap" / "map011_cloud.ply"
    cloud_path.write_text(
        "ply\nformat ascii 1.0\nelement vertex 2\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property uchar red\nproperty uchar green\nproperty uchar blue\n"
        "end_header\n0 0 0 255 0 1\n1.25 -2.5 3 4 5 6\n",
        encoding="ascii",
    )
    filtered_octomap = (
        tmp_path / "octo_maps" / "map011_filtered_octomap" / "map011.bt"
    )
    filtered_octomap.write_bytes(b"filtered octomap")
    filtered_cloud = (
        tmp_path / "octo_maps" / "map011_filtered_octomap"
        / "map011_filtered_cloud.ply"
    )
    filtered_cloud.write_text(cloud_path.read_text(encoding="ascii"), encoding="ascii")

    assert discover_navigation_maps(tmp_path) == [
        {
            "id": "map011",
            "database_path": str((tmp_path / "rtab_maps" / "map011.db").resolve()),
            "octomap_path": str(
                (tmp_path / "octo_maps" / "map011_octomap" / "map011.bt").resolve()
            ),
            "cloud_path": str(cloud_path.resolve()),
            "filtered_octomap_path": str(filtered_octomap.resolve()),
            "filtered_cloud_path": str(filtered_cloud.resolve()),
            "hloc_map_directory": str(
                (tmp_path / "hloc_maps" / "map011").resolve()
            ),
            "convertible": True,
            "loadable": True,
            "localizable": True,
            "filtered_loadable": True,
            "filtered_localizable": True,
        },
        {
            "id": "map012",
            "database_path": str((tmp_path / "rtab_maps" / "map012.db").resolve()),
            "octomap_path": None,
            "cloud_path": None,
            "filtered_octomap_path": None,
            "filtered_cloud_path": None,
            "hloc_map_directory": None,
            "convertible": True,
            "loadable": False,
            "localizable": False,
            "filtered_loadable": False,
            "filtered_localizable": False,
        },
    ]


def test_public_map_catalog_hides_paths_and_lists_downloads(tmp_path):
    database = tmp_path / "rtab_maps" / "map007.db"
    octomap = tmp_path / "octo_maps" / "map007_octomap" / "map007.bt"
    cloud = tmp_path / "octo_maps" / "map007_octomap" / "map007_cloud.ply"
    metadata = tmp_path / "hloc_maps" / "map007" / "metadata.yaml"
    annotations = tmp_path / "semantic_maps" / "map007" / "annotations.json"
    for path, content in (
        (database, b"database"),
        (octomap, b"octomap"),
        (cloud, b"ply"),
        (metadata, b"schema_version: 1\n"),
        (annotations, b"{}"),
    ):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(content)

    catalog = public_navigation_maps(tmp_path)

    assert len(catalog) == 1
    assert catalog[0]["id"] == "map007"
    assert catalog[0]["default_variant"] == "original"
    assert "database_path" not in catalog[0]
    assert "cloud_path" not in catalog[0]
    assert {file["layer"] for file in catalog[0]["files"]} == {
        "database", "cloud", "octomap", "annotations", "hloc_metadata",
    }
    assert all(
        file["download_url"].startswith("/api/maps/map007/download/")
        for file in catalog[0]["files"]
    )


@pytest.mark.parametrize(
    ("value", "size", "expected"),
    [
        ("", 100, None),
        ("bytes=0-9", 100, (0, 9)),
        ("bytes=90-", 100, (90, 99)),
        ("bytes=-10", 100, (90, 99)),
        ("bytes=0-999", 100, (0, 99)),
    ],
)
def test_http_byte_ranges_support_resumable_map_downloads(value, size, expected):
    assert parse_http_byte_range(value, size) == expected


@pytest.mark.parametrize("value", ["items=0-1", "bytes=", "bytes=101-102"])
def test_invalid_http_byte_ranges_are_rejected(value):
    with pytest.raises(ValueError):
        parse_http_byte_range(value, 100)


def test_map_portal_is_the_configured_public_ui():
    package = WORKSPACE_ROOT / "project/luxi-web-control"
    config = yaml.safe_load(
        (package / "config/web_control.yaml").read_text(encoding="utf-8")
    )["web_control"]["ros__parameters"]
    launch_source = (package / "launch/web_control.launch.py").read_text(
        encoding="utf-8"
    )
    page = (package / "web/map_portal.html").read_text(encoding="utf-8")
    app = (package / "web/map_portal.js").read_text(encoding="utf-8")

    assert config["web_ui_mode"] == "map_portal"
    assert 'DeclareLaunchArgument("web_ui_mode", default_value="map_portal")' in launch_source
    assert 'id="mapCanvas"' in page
    assert 'id="mapLoading"' in page
    assert 'id="mapLoadingText"' in page
    assert 'id="mapSelect" class="map-select" size="10"' in page
    assert 'id="mapFiles"' in page
    assert '<script src="/map_projection.js"></script>' in page
    assert 'id="filteredToggle" type="checkbox" checked' in page
    assert 'id="showCostmap" type="checkbox" checked' in page
    assert 'id="cameraProfileSelect"' in page
    assert 'id="rgbPreview"' in page
    assert 'id="robotControlToggle"' in page
    assert 'id="robotBatteryState"' in page
    assert 'id="imuCalibrationState"' in page
    assert 'id="imuCalibrationButton"' in page
    assert 'id="joystickPad"' in page
    assert 'id="manualControlState"' in page
    assert 'id="restartSystemButton"' in page
    assert 'id="restartOverlay"' in page
    assert 'href="/developer"' in page
    assert "开发者模式" in page
    assert "选择并发送目标点" in page
    assert "开始运动前：请开启相机、开启机器人控制，并完成 IMU 校准" in page
    assert "机器人请确保已开启并保持完全静止" in page
    assert page.index('class="map-panel"') < page.index('class="panel navigation-panel"')
    navigation_panel = page.index('class="panel navigation-panel"')
    manual_panel = page.index('class="panel manual-control-panel"')
    rgb_panel = page.index('class="panel rgb-panel"')
    assert navigation_panel < manual_panel < rgb_panel
    assert page.index('class="panel rgb-panel"') < page.index('class="panel camera-panel"')
    assert page.index('class="panel camera-panel"') < page.index('class="panel robot-panel"')
    assert page.index('class="panel robot-panel"') < page.index('class="panel status-panel"')
    assert page.index('class="panel status-panel"') < page.index('class="panel imu-panel"')
    assert 'post("/api/navigation/goal", target)' in app
    assert 'post("/api/navigation/home/set", home)' in app
    assert 'post("/api/navigation/home/return")' in app
    assert 'id="setHomeButton"' in page
    assert 'id="returnHomeButton"' in page
    assert "updateGoalDirection(event)" in app
    assert "finishGoalGesture(event, true)" in app
    assert "mapProjection.unprojectGround" in app
    assert "按住并拖动箭头选择方向" in page
    assert 'post("/api/navigation/start")' in app
    assert 'post("/api/navigation/halt")' in app
    assert 'post("/api/cmd_vel", {...command, client_id: controlClientId})' in app
    assert 'fetch("/api/stop"' in app
    assert 'setInterval(sendManualCommand, 100)' in app
    assert 'manualKeyActions' in app
    assert 'elements.joystickPad.addEventListener("pointerdown"' in app
    assert 'elements.sendGoal.addEventListener("click", chooseGoalOnMap)' in app
    assert 'sendNavigationGoal(state.goal)' in app
    assert 'latestFiltered?.id' in app
    assert 'setMapLoading(true, `正在加载 ${record.id}' in app
    assert 'setMapLoading(true, "地图数据已准备，正在加载三维图层…")' in app
    assert 'setMapLoading(false)' in app
    assert 'loadInitialFilteredPreview' not in app
    assert 'post("/api/system/restart", {confirm: "restart_all_services"})' in app
    assert 'elements.restartSystem.addEventListener("click", restartSystem)' in app
    assert 'refreshMaps();' in app
    assert "file.download_url" in app
    assert "record.default_variant" in app
    assert "elements.mapSelect.addEventListener(\"change\"" in app
    assert "elements.mapFiles.append(...record.files.map(createDownloadLink))" in app
    assert "elements.canvas.addEventListener(\"pointermove\"" in app
    assert "state.view.yaw = state.pointer.yaw" in app
    assert "state.view.pitch = Math.max(" in app
    assert "mapProjection.projectMapPoint(" in app
    assert 'post("/api/camera/start", {profile})' in app
    assert 'post("/api/camera/stop")' in app
    assert 'post("/api/robot/control", {active: requested})' in app
    assert 'post("/api/imu/calibrate")' in app
    assert 'elements.imuButton.addEventListener("click", startImuCalibration)' in app
    assert "state.robotControl.active" in app
    assert 'fetch(`/api/preview/rgb?t=${Date.now()}`' in app
    assert "elements.showCostmap.checked" in app
    assert "`/api/maps/${state.loadedMapId}/preview`" in app


def test_both_pages_share_navigation_recovery_rules_and_controls():
    web = WORKSPACE_ROOT / "project/luxi-web-control/web"
    developer_page = (web / "index.html").read_text(encoding="utf-8")
    user_page = (web / "map_portal.html").read_text(encoding="utf-8")
    developer_app = (web / "app.js").read_text(encoding="utf-8")
    user_app = (web / "map_portal.js").read_text(encoding="utf-8")
    shared = (web / "map_projection.js").read_text(encoding="utf-8")

    assert '<script src="/map_projection.js" defer></script>' in developer_page
    assert '<script src="/map_projection.js"></script>' in user_page
    assert "window.LuxiNavigationUi" in developer_app
    assert "window.LuxiNavigationUi" in user_app
    assert "forward_clear" not in developer_app
    assert "forward_clear" not in user_app
    assert "丢失前最后一次前方净空" in shared
    assert "旋转没有时间上限" in shared


def test_web_api_capabilities_cover_all_map_and_control_actions():
    capabilities = web_api_capabilities()
    operations = {
        (item["method"], item["path"])
        for item in capabilities["operations"]
    }
    assert ("GET", "/api/maps") in operations
    assert ("GET", "/api/preview/rgb") in operations
    assert ("POST", "/api/maps/{map_id}/preview") in operations
    assert ("GET", "/api/maps/{map_id}/preview/costmap") in operations
    assert ("GET", "/api/maps/{map_id}/preview/obstacles") in operations
    assert ("GET", "/api/maps/{map_id}/download/{layer}") in operations
    assert ("POST", "/api/navigation/goal") in operations
    assert ("POST", "/api/navigation/home/set") in operations
    assert ("POST", "/api/navigation/home/return") in operations
    assert ("POST", "/api/navigation/start") in operations
    assert ("POST", "/api/navigation/halt") in operations
    assert ("POST", "/api/cmd_vel") in operations
    assert ("POST", "/api/stop") in operations
    assert ("POST", "/api/system/restart") in operations
    assert ("POST", "/api/mapping/start") in operations
    assert ("POST", "/api/camera/start") in operations
    assert ("POST", "/api/imu/calibrate") in operations


def test_filtered_map_is_the_default_preview_variant():
    node = WebControlNode.__new__(WebControlNode)
    node.public_navigation_map = lambda _map_id: {"filtered_loadable": True}
    assert node.default_map_preview_filtered("map037")
    node.public_navigation_map = lambda _map_id: {"filtered_loadable": False}
    assert not node.default_map_preview_filtered("map037")


def test_colored_ply_points_extracts_xyzrgb_from_ascii_export(tmp_path):
    cloud_path = tmp_path / "map011_cloud.ply"
    cloud_path.write_text(
        "ply\nformat ascii 1.0\nelement vertex 3\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property uchar red\nproperty uchar green\nproperty uchar blue\n"
        "end_header\n0 1 2 1 2 3\n4.5678 5 6 254 253 252\n7 8 9 0 0 0\n",
        encoding="ascii",
    )

    assert extract_colored_ply_points(cloud_path, 2) == [
        (0.0, 1.0, 2.0, 1, 2, 3),
        (7.0, 8.0, 9.0, 0, 0, 0),
    ]


def test_colored_ply_points_reads_all_vertices_when_limit_is_zero(tmp_path):
    cloud_path = tmp_path / "complete_cloud.ply"
    cloud_path.write_text(
        "ply\nformat ascii 1.0\nelement vertex 3\n"
        "property float x\nproperty float y\nproperty float z\n"
        "property uchar red\nproperty uchar green\nproperty uchar blue\n"
        "end_header\n0 1 2 1 2 3\n4 5 6 254 253 252\n7 8 9 0 0 0\n",
        encoding="ascii",
    )

    assert extract_colored_ply_points(cloud_path, 0) == [
        (0.0, 1.0, 2.0, 1, 2, 3),
        (4.0, 5.0, 6.0, 254, 253, 252),
        (7.0, 8.0, 9.0, 0, 0, 0),
    ]


def test_octomap_converter_output_keeps_occupied_voxel_size():
    assert parse_octomap_point_output(
        "resolution 0.1\n1.2345 -2 3 0.1\n0 0 0 0.2\n"
    ) == (
        0.1,
        [(1.234, -2.0, 3.0, 0.1), (0.0, 0.0, 0.0, 0.2)],
    )


def test_terrain_converter_output_separates_obstacles_and_costs():
    assert parse_terrain_point_output(
        "resolution 0.1\n"
        "traversable 1.2345 -2 0.1 0.75\n"
        "obstacle 0 0 0.2 1\n"
    ) == (
        0.1,
        [(1.234, -2.0, 0.1, 0.75)],
        [(0.0, 0.0, 0.2)],
    )


def test_terrain_loader_fits_the_matching_point_cloud(monkeypatch, tmp_path):
    node = WebControlNode.__new__(WebControlNode)
    node.terrain_points_executable = Path("/test/terrain_map_to_points")
    node.max_terrain_points = 12000
    node.navigation_robot_radius = 0.10
    node.navigation_costmap_margin = 0.60
    node.navigation_ground_normal_radius = 0.30
    node.navigation_ground_max_slope_degrees = 35.0
    node.navigation_obstacle_min_height = 0.15
    node.navigation_terrain_load_timeout = 90.0
    node.navigation = SimpleNamespace(octomap_library_path=tmp_path)
    node._navigation_lock = threading.Lock()

    def fake_run(command, **kwargs):
        assert command == [
            "/test/terrain_map_to_points", "/maps/map042.bt", "12000",
            "0.1", "0.6", "/maps/map042_cloud.ply", "0.3", "35.0",
            "0.15",
        ]
        assert kwargs["timeout"] == 90.0
        return subprocess.CompletedProcess(
            command, 0,
            stdout=(
                "resolution 0.1\n"
                "traversable 1 2 0.1 0.25\n"
                "obstacle 3 4 0.2 1\n"
            ),
            stderr="",
        )

    monkeypatch.setattr(subprocess, "run", fake_run)
    assert node._load_navigation_terrain(
        "map042", "/maps/map042.bt", "/maps/map042_cloud.ply"
    ) == ""
    assert node._terrain_traversable_points == [(1.0, 2.0, 0.1, 0.25)]
    assert node._terrain_obstacle_points == [(3.0, 4.0, 0.2)]


def test_localization_requires_confident_xyz_yaw_covariance():
    covariance = [0.0] * 36
    covariance[0] = 0.5
    covariance[7] = 1.0
    covariance[35] = 2.0
    assert localization_covariance_ready(covariance, 10.0)

    covariance[35] = 9999.0
    assert not localization_covariance_ready(covariance, 10.0)


def test_localization_pose_summary_returns_planar_heading():
    message = PoseWithCovarianceStamped()
    message.pose.pose.position.x = 1.25
    message.pose.pose.position.y = -0.5
    message.pose.pose.orientation.z = math.sin(math.pi / 4.0)
    message.pose.pose.orientation.w = math.cos(math.pi / 4.0)

    assert localization_pose_summary(message) == {
        "x": 1.25,
        "y": -0.5,
        "z": 0.0,
        "yaw": round(math.pi / 2.0, 5),
        "yaw_degrees": 90.0,
    }


def test_localization_pose_summary_rejects_invalid_quaternion():
    message = PoseWithCovarianceStamped()
    message.pose.pose.orientation.w = 0.0
    assert localization_pose_summary(message) is None


def test_terrain_pose_callback_keeps_the_ground_constrained_height():
    node = WebControlNode.__new__(WebControlNode)
    node._navigation_lock = threading.Lock()
    node._terrain_pose = None
    node._terrain_pose_received_at = None
    message = PoseStamped()
    message.pose.position.x = -6.825
    message.pose.position.y = 2.375
    message.pose.position.z = -2.575
    message.pose.orientation.w = 1.0

    node._on_navigation_terrain_pose(message)

    assert node._terrain_pose == {
        "x": -6.825,
        "y": 2.375,
        "z": -2.575,
        "yaw": 0.0,
        "yaw_degrees": 0.0,
    }
    assert node._terrain_pose_received_at is not None


@pytest.mark.parametrize("payload", [
    {"x": 0.1, "y": -0.2},
    {"x": 0, "y": 0, "z": 0.3},
])
def test_navigation_goal_accepts_finite_coordinates(payload):
    assert parse_navigation_goal(payload) == (
        float(payload["x"]), float(payload["y"]),
        float(payload.get("z", 0.0)), float(payload.get("yaw", 0.0)))


def test_navigation_goal_normalizes_final_heading():
    parsed = parse_navigation_goal({"x": 0, "y": 0, "yaw": 3 * math.pi})
    assert parsed[:3] == (0.0, 0.0, 0.0)
    assert math.isclose(abs(parsed[3]), math.pi)


@pytest.mark.parametrize("payload", [
    {"x": True, "y": 0}, {"x": math.nan, "y": 0}, {"x": 0, "y": "bad"},
    {"x": 0, "y": 0, "yaw": math.inf},
])
def test_navigation_goal_rejects_invalid_coordinates(payload):
    with pytest.raises(ValueError):
        parse_navigation_goal(payload)


def test_navigation_motion_requires_localization_and_path():
    node = WebControlNode.__new__(WebControlNode)
    node._lock = threading.Lock()
    node._navigation_lock = threading.Lock()
    node._estop_active = False
    node._navigation_follower_state = "plan_ready"
    published = []
    node.navigation_start_publisher = SimpleNamespace(
        publish=published.append
    )
    node.navigation_status = lambda: {
        "state": "running",
        "localization_ready": True,
        "planning_localization_ready": True,
        "path_ready": True,
        "active": False,
    }

    started, message = node.start_navigation_motion()

    assert started
    assert message == "navigation start command sent"
    assert len(published) == 1
    assert published[0].data is True
    assert node._navigation_follower_state == "starting"

    node.navigation_status = lambda: {
        "state": "running",
        "localization_ready": True,
        "planning_localization_ready": True,
        "path_ready": False,
        "active": False,
    }
    started, message = node.start_navigation_motion()
    assert not started
    assert "valid path" in message
    assert len(published) == 1


def test_navigation_halt_clears_paths_but_retains_localization():
    node = WebControlNode.__new__(WebControlNode)
    node._navigation_lock = threading.Lock()
    node._navigation_active = True
    node._navigation_follower_state = "active"
    node._planned_path_points = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]
    node._last_valid_path_points = list(node._planned_path_points)
    node._path_received_at = 1.0
    node._last_valid_path_received_at = 1.0
    node._planner_map_ready = True
    node._planning_state = "ready"
    node._planning_error = "old error"
    node._refined_localization_pose = {"x": 2.0, "y": 3.0}
    published = []
    node.navigation_stop_publisher = SimpleNamespace(publish=published.append)

    node.halt_navigation_motion(clear_path=True)

    assert len(published) == 1
    assert published[0].data is True
    assert node._navigation_active is False
    assert node._navigation_follower_state == "stopped"
    assert node._planned_path_points == []
    assert node._last_valid_path_points == []
    assert node._path_received_at is None
    assert node._last_valid_path_received_at is None
    assert node._planning_state == "map_ready"
    assert node._planning_error == ""
    assert node._refined_localization_pose == {"x": 2.0, "y": 3.0}


def test_navigation_active_transition_clears_finished_task_path():
    node = WebControlNode.__new__(WebControlNode)
    node._navigation_lock = threading.Lock()
    node._navigation_active = True
    node._planned_path_points = [(0.0, 0.0, 0.0)]
    node._last_valid_path_points = list(node._planned_path_points)
    node._path_received_at = 1.0
    node._last_valid_path_received_at = 1.0
    node._return_home_pending = True

    message = Bool()
    message.data = False
    node._on_navigation_active(message)

    assert node._planned_path_points == []
    assert node._last_valid_path_points == []
    # The independent task manager, not this low-level active edge, owns the
    # return-home state machine.
    assert node._return_home_pending is True

    task_status = String()
    task_status.data = "cancelled"
    node._on_navigation_task_status(task_status)
    assert node._return_home_pending is False


def test_navigation_home_and_return_are_forwarded_to_task_manager():
    node = WebControlNode.__new__(WebControlNode)
    node._navigation_lock = threading.Lock()
    node._navigation_cloud_map_id = "map039"
    node._terrain_map_id = "map039"
    node._voxel_frame_id = "map"
    node._navigation_home = None
    node._navigation_task_state = "idle"
    node._return_home_pending = False
    node.navigation_status = lambda: {
        "state": "running", "home": node._navigation_home,
    }
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(
            to_msg=lambda: PoseStamped().header.stamp
        )
    )
    homes = []
    returns = []
    node.navigation_home_publisher = SimpleNamespace(publish=homes.append)
    node.navigation_return_home_publisher = SimpleNamespace(
        publish=returns.append
    )
    accepted, _message = node.set_navigation_home(1.0, -0.5, 0.1, 0.4)
    assert accepted
    assert node._navigation_home["x"] == 1.0
    assert len(homes) == 1
    assert homes[0].header.frame_id == "map"
    assert homes[0].pose.position.x == 1.0

    calls = []
    node.stop_motion = lambda force=True: calls.append(("stop", force))
    accepted, _message = node.return_navigation_home()

    assert accepted
    assert calls == [("stop", True)]
    assert len(returns) == 1
    assert returns[0].data is True
    assert node._navigation_task_state == "return_planning"


def test_return_home_path_does_not_auto_start_in_web_process():
    node = WebControlNode.__new__(WebControlNode)
    node._navigation_lock = threading.Lock()
    node._planned_path_points = []
    node._last_valid_path_points = []
    node._path_frame_id = "map"
    node._last_valid_path_frame_id = "map"
    node._path_received_at = None
    node._last_valid_path_received_at = None
    node._planning_state = "pending"
    node._planning_error = ""
    node._navigation_follower_state = "plan_ready"
    node._return_home_pending = True
    starts = []
    node.start_navigation_motion = lambda: (
        starts.append(True) or (True, "started")
    )
    path = NavigationPath()
    path.poses.append(PoseStamped())

    node._on_navigation_path(path)

    assert starts == []
    assert node._return_home_pending is True


def test_relocalization_replans_at_slower_rotation_speed():
    config = yaml.safe_load(
        (WORKSPACE_ROOT / "project/luxi_3d_navigation/config/navigation.yaml")
        .read_text(encoding="utf-8")
    )
    follower = config["terrain_path_follower"]["ros__parameters"]
    safety = config["navigation_safety_gate"]["ros__parameters"]
    assert follower["localization_recovery_angular_speed"] == 0.10
    assert follower["localization_recovery_timeout"] == 0.0
    assert follower["replan_request_topic"] == "/navigation/replan_request"
    assert safety["dead_reckoning_duration"] == 0.80
    assert safety["maximum_dead_reckoning_linear_speed"] == 0.10
    source = (
        WORKSPACE_ROOT / "project/luxi_3d_navigation/src/"
        "terrain_path_follower_node.cpp"
    ).read_text(encoding="utf-8")
    assert "requestReplanAfterRelocalization" in source
    assert "replanning_after_relocalization" in source


@pytest.mark.parametrize(
    ("navigation_active", "path_ready"),
    [(True, False), (False, True)],
)
def test_manual_velocity_is_rejected_until_navigation_is_halted(
    navigation_active, path_ready
):
    node = WebControlNode.__new__(WebControlNode)
    node.d1_control_status = lambda: {"enabled": False, "control_ready": False}
    node.navigation_status = lambda: {
        "active": navigation_active, "path_ready": path_ready,
    }

    accepted, message = node.accept_command(
        VelocityCommand(linear_x=0.1), "browser-12345678"
    )

    assert accepted is False
    assert "halt it first" in message


def test_failed_replan_removes_old_preview():
    node = WebControlNode.__new__(WebControlNode)
    node._navigation_lock = threading.Lock()
    node._planned_path_points = []
    node._last_valid_path_points = []
    node._path_frame_id = "map"
    node._last_valid_path_frame_id = "map"
    node._path_received_at = None
    node._last_valid_path_received_at = None
    node._planning_state = "pending"
    node._planning_error = ""
    node._navigation_follower_state = "plan_ready"

    valid_path = NavigationPath()
    valid_path.header.frame_id = "map"
    valid_path.poses.append(PoseStamped())
    valid_path.poses[0].pose.position.x = 1.0
    node._on_navigation_path(valid_path)
    assert node.path_preview()["valid"] is True

    node._planning_state = "pending"
    node._on_navigation_path(NavigationPath())
    failure = String()
    failure.data = "failed_start_or_goal_unsupported"
    node._on_navigation_planning_status(failure)

    preview = node.path_preview()
    assert preview["valid"] is False
    assert preview["stale"] is False
    assert preview["active_point_count"] == 0
    assert preview["points"] == []
    assert preview["planning_state"] == "failed"
    assert "可通行地形" in preview["error"]


def test_goal_reached_clears_global_path_and_suppresses_late_preview():
    node = WebControlNode.__new__(WebControlNode)
    node._navigation_lock = threading.Lock()
    node._navigation_follower_state = "active"
    node._planned_path_points = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]
    node._last_valid_path_points = list(node._planned_path_points)
    node._path_frame_id = "map"
    node._last_valid_path_frame_id = "map"
    node._path_received_at = 1.0
    node._last_valid_path_received_at = node._path_received_at
    node._planning_state = "ready"
    node._planning_error = ""

    reached = String()
    reached.data = "goal_reached"
    node._on_navigation_follower_state(reached)

    assert node._planned_path_points == []
    assert node._last_valid_path_points == []
    assert node._planning_state == "goal_reached"
    assert node.path_preview()["points"] == []
    assert node.path_preview()["active_point_count"] == 0

    # A delayed/transient ROS path must remain hidden while the follower still
    # reports completion. It becomes available normally for the next plan.
    delayed = NavigationPath()
    delayed.poses.append(PoseStamped())
    delayed.poses[0].pose.position.x = 2.0
    node._on_navigation_path(delayed)
    assert node.path_preview()["points"] == []

    ready = String()
    ready.data = "plan_ready"
    node._on_navigation_follower_state(ready)
    assert node.path_preview()["points"] == [(2.0, 0.0, 0.0)]


def test_icp_fitness_refreshes_map_verification():
    node = WebControlNode.__new__(WebControlNode)
    node._navigation_lock = threading.Lock()
    node._refined_localization_status = ""
    node._refined_localization_verified_at = None
    node._refined_localization_fitness = None
    node.navigation_icp_minimum_fitness = 0.25
    node._coarse_gate_ready = True
    node._refined_localization_pose = {"x": 0.0}
    node._refined_localization_received_at = 1.0

    good_fitness = Float32()
    good_fitness.data = 0.9
    node._on_navigation_refined_fitness(good_fitness)
    verified_at = node._refined_localization_verified_at
    assert verified_at is not None

    bad_fitness = Float32()
    bad_fitness.data = 0.0
    node._on_navigation_refined_fitness(bad_fitness)
    assert node._refined_localization_verified_at == verified_at


def test_dead_reckoning_health_is_exposed_during_bounded_recovery():
    node = WebControlNode.__new__(WebControlNode)
    node._navigation_lock = threading.Lock()
    node._localization_health = "tracking"
    node._localization_health_received_at = None

    message = String()
    message.data = "dead_reckoning"
    node._on_navigation_localization_health(message)

    assert node._localization_health == "dead_reckoning"
    assert node._localization_health_received_at is not None


def test_navigation_goal_is_blocked_without_recent_icp_verification():
    node = WebControlNode.__new__(WebControlNode)
    published = []
    node.navigation_goal_publisher = SimpleNamespace(publish=published.append)
    node.navigation_status = lambda: {
        "state": "running",
        "localization_ready": True,
        "planning_localization_ready": False,
    }

    accepted, message = node.set_navigation_goal(0.5, 0.0, 0.0)

    assert accepted is False
    assert "recent accepted ICP" in message
    assert published == []


def test_navigation_goal_is_blocked_until_terrain_map_is_ready():
    node = WebControlNode.__new__(WebControlNode)
    published = []
    node.navigation_goal_publisher = SimpleNamespace(publish=published.append)
    node.navigation_status = lambda: {
        "state": "running",
        "planning_localization_ready": True,
        "planner_map_ready": False,
    }

    accepted, message = node.set_navigation_goal(1.5, 0.0, 0.0)

    assert accepted is False
    assert "terrain map" in message
    assert published == []


def test_planner_map_status_distinguishes_loading_from_ready():
    node = WebControlNode.__new__(WebControlNode)
    node._navigation_lock = threading.Lock()
    node._planned_path_points = []
    node._planning_state = "idle"
    node._planning_error = ""
    node._planner_map_ready = False

    loading = String()
    loading.data = "loading_map"
    node._on_navigation_planning_status(loading)
    assert node._planning_state == "loading_map"
    assert node._planner_map_ready is False

    ready = String()
    ready.data = "map_ready"
    node._on_navigation_planning_status(ready)
    assert node._planning_state == "map_ready"
    assert node._planner_map_ready is True


@pytest.mark.parametrize("value", ["fast", True, None, math.inf, math.nan])
def test_invalid_velocity_is_rejected(value):
    with pytest.raises(ValueError):
        parse_velocity({"linear_x": value}, LIMITS)
