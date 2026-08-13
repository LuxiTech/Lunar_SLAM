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
import signal
import sqlite3
import struct
import subprocess
import threading
from types import SimpleNamespace

import pytest
import yaml

from pathlib import Path

from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud2, PointField

from luxi_web_control.web_control_node import discover_navigation_maps
from luxi_web_control.web_control_node import D1ControlManager
from luxi_web_control.web_control_node import classify_d1_posture
from luxi_web_control.web_control_node import extract_colored_ply_points
from luxi_web_control.web_control_node import extract_sparse_cloud
from luxi_web_control.web_control_node import HlocIndexBuilder
from luxi_web_control.web_control_node import MappingController
from luxi_web_control.web_control_node import is_managed_web_control_command
from luxi_web_control.web_control_node import localization_covariance_ready
from luxi_web_control.web_control_node import localization_pose_summary
from luxi_web_control.web_control_node import make_access_urls
from luxi_web_control.web_control_node import mapping_graph_conflicts
from luxi_web_control.web_control_node import rtabmap_database_conversion_error
from luxi_web_control.web_control_node import NavigationController
from luxi_web_control.web_control_node import parse_octomap_point_output
from luxi_web_control.web_control_node import parse_terrain_point_output
from luxi_web_control.web_control_node import parse_navigation_goal
from luxi_web_control.web_control_node import parse_velocity, VelocityCommand
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


def test_navigation_preview_uses_ten_centimeter_robot_radius():
    config = yaml.safe_load(
        (WORKSPACE_ROOT / "project/luxi-web-control/config/web_control.yaml").read_text(
            encoding="utf-8"
        )
    )
    radius = config["web_control"]["ros__parameters"][
        "navigation_robot_radius"
    ]
    assert radius == 0.10


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
        assert "topic info --no-daemon --spin-time 5.0" in source
        assert "rcl_interfaces/srv/SetParameters" in source
        assert "successful=True" in source


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
    assert "--decimation 2" in script
    assert "--max_range 10.0" in script
    assert "--noise_radius 0.35" in script
    assert "--noise_k 3" in script
    assert "--min_range 0.35" in script
    assert "--edge_bleeding_error 0.10" in script


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
    assert "signal_matches INT" in source
    assert "signal_matches TERM" in source
    assert "velocity_command_mux_node" in source
    assert "sport = :8080" in source


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


def test_usb_primary_mapping_command_uses_imu_by_default(tmp_path):
    workspace_setup = Path(tmp_path / "workspace_setup.bash")
    workspace_setup.touch()
    controller = MappingController(
        enabled=True,
        package="lunar_usb_rtabmap_bringup",
        launch_file="usb_rtabmap.launch.py",
        rmw_implementation="rmw_fastrtps_cpp",
        sensor_setup=None,
        workspace_setup=workspace_setup,
        log_path=Path(tmp_path / "mapping.log"),
        launch_arguments=(
            "new_map:=true",
            "rviz:=false",
            "rtabmap_viz:=false",
            "use_imu:=true",
            "planar_mode:=false",
        ),
    )

    command = controller._command()[-1]

    assert "usb_crestereo_rtabmap.launch.py" in command
    assert " mode:=" not in command
    assert "use_imu:=true" in command
    assert "planar_mode:=false" in command
    assert "RMW_IMPLEMENTATION=rmw_fastrtps_cpp" in command


def test_device_workspace_overlays_algorithm_workspace(tmp_path):
    workspace_setup = Path(tmp_path / "workspace_setup.bash")
    sensor_setup = Path(tmp_path / "usb_setup.bash")
    workspace_setup.touch()
    sensor_setup.touch()
    controller = MappingController(
        enabled=True,
        package="lunar_usb_rtabmap_bringup",
        launch_file="usb_rtabmap.launch.py",
        rmw_implementation="rmw_fastrtps_cpp",
        sensor_setup=sensor_setup,
        workspace_setup=workspace_setup,
        log_path=Path(tmp_path / "mapping.log"),
    )

    command = controller._command()[-1]

    assert command.index(f"source {workspace_setup}") < command.index(
        f"source {sensor_setup}"
    )


def test_web_mapping_controller_defaults_to_crestereo_primary(tmp_path):
    workspace_setup = Path(tmp_path / "workspace_setup.bash")
    workspace_setup.touch()
    controller = MappingController(
        enabled=True,
        package="lunar_usb_rtabmap_bringup",
        launch_file="usb_rtabmap.launch.py",
        rmw_implementation="rmw_fastrtps_cpp",
        sensor_setup=None,
        workspace_setup=workspace_setup,
        log_path=Path(tmp_path / "mapping.log"),
        launch_arguments=("new_map:=true", "use_imu:=true", "rviz:=false"),
    )

    command = controller._command()[-1]
    status = controller.status()

    assert "usb_crestereo_rtabmap.launch.py" in command
    assert " mode:=" not in command
    assert "use_imu:=true" in command
    assert "new_map:=true" in command
    assert status["frontend"] == "crestereo_cuda_graph+luxi_direct_odom"
    assert status["mode"] == "crestereo"
    assert status["modes"] == ["crestereo", "vpi"]
    assert status["default_mode"] == "crestereo"


def test_web_mapping_controller_selects_crestereo_launch(tmp_path):
    workspace_setup = Path(tmp_path / "workspace_setup.bash")
    workspace_setup.touch()
    controller = MappingController(
        enabled=True,
        package="lunar_usb_rtabmap_bringup",
        launch_file="usb_rtabmap.launch.py",
        rmw_implementation="rmw_fastrtps_cpp",
        sensor_setup=None,
        workspace_setup=workspace_setup,
        log_path=Path(tmp_path / "mapping.log"),
        launch_arguments=("new_map:=true", "use_imu:=true", "rviz:=false"),
        crestereo_use_imu=True,
    )

    command = controller._command("crestereo")[-1]
    controller._mode = "crestereo"
    status = controller.status()

    assert "usb_crestereo_rtabmap.launch.py" in command
    assert "usb_rtabmap.launch.py" not in command
    assert "new_map:=true" in command
    assert "use_imu:=true" in command
    assert "use_imu:=false" not in command
    assert status["frontend"] == "crestereo_cuda_graph+luxi_direct_odom"


def test_web_mapping_controller_keeps_vpi_imu_setting(tmp_path):
    workspace_setup = Path(tmp_path / "workspace_setup.bash")
    workspace_setup.touch()
    controller = MappingController(
        enabled=True,
        package="lunar_usb_rtabmap_bringup",
        launch_file="usb_rtabmap.launch.py",
        rmw_implementation="rmw_fastrtps_cpp",
        sensor_setup=None,
        workspace_setup=workspace_setup,
        log_path=Path(tmp_path / "mapping.log"),
        launch_arguments=("new_map:=true", "use_imu:=true", "rviz:=false"),
        crestereo_use_imu=False,
    )

    command = controller._command("vpi")[-1]

    assert "usb_rtabmap.launch.py" in command
    assert "use_imu:=true" in command
    assert "use_imu:=false" not in command


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


def test_mapping_status_prioritizes_h30_root_cause(tmp_path):
    log_path = Path(tmp_path / "mapping.log")
    log_path.write_text(
        "[ERROR] IMU NO DATA: received zero packets for 8.0 s\n"
        "[ERROR] Caught exception: sensor synchronization failed\n",
        encoding="utf-8",
    )
    controller = MappingController(
        enabled=True,
        package="lunar_usb_rtabmap_bringup",
        launch_file="usb_crestereo_rtabmap.launch.py",
        rmw_implementation="rmw_fastrtps_cpp",
        sensor_setup=None,
        workspace_setup=Path(tmp_path / "workspace_setup.bash"),
        log_path=log_path,
    )

    assert "IMU NO DATA" in controller._latest_log_error()


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


def test_mapping_stop_signals_only_top_level_launch(tmp_path):
    class GracefulLaunch:
        pid = 4242

        def __init__(self):
            self.running = True
            self.signals = []

        def poll(self):
            return None if self.running else 0

        def send_signal(self, sent_signal):
            self.signals.append(sent_signal)
            self.running = False

        def wait(self, timeout):
            del timeout
            return 0

    process = GracefulLaunch()
    controller = MappingController(
        enabled=True,
        package="lunar_usb_rtabmap_bringup",
        launch_file="usb_rtabmap.launch.py",
        rmw_implementation="rmw_fastrtps_cpp",
        sensor_setup=None,
        workspace_setup=Path(tmp_path / "workspace_setup.bash"),
        log_path=Path(tmp_path / "mapping.log"),
    )
    controller._process = process

    stopped, _ = controller.stop()

    assert stopped
    assert process.signals == [signal.SIGINT]
    assert controller.status()["last_exit_code"] == 0


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


def test_rtabmap_database_rejects_capture_without_odometry_links(tmp_path):
    database = tmp_path / "map066.db"
    connection = sqlite3.connect(database)
    connection.executescript(
        "CREATE TABLE Node(id INTEGER PRIMARY KEY);"
        "CREATE TABLE Link(from_id INTEGER, to_id INTEGER, type INTEGER);"
        "INSERT INTO Node VALUES(1);"
        "INSERT INTO Node VALUES(2);"
        "INSERT INTO Link VALUES(1, 1, 9);"
        "INSERT INTO Link VALUES(2, 2, 9);"
    )
    connection.close()

    error = rtabmap_database_conversion_error(database)

    assert "2 nodes but no odometry links" in error
    assert "must be recorded again" in error


def test_rtabmap_database_accepts_neighbor_odometry_link(tmp_path):
    database = tmp_path / "map067.db"
    connection = sqlite3.connect(database)
    connection.executescript(
        "CREATE TABLE Node(id INTEGER PRIMARY KEY);"
        "CREATE TABLE Link(from_id INTEGER, to_id INTEGER, type INTEGER);"
        "INSERT INTO Node VALUES(1);"
        "INSERT INTO Node VALUES(2);"
        "INSERT INTO Link VALUES(1, 2, 0);"
    )
    connection.close()

    assert rtabmap_database_conversion_error(database) == ""


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
    node.navigation = SimpleNamespace(octomap_library_path=tmp_path)
    node._navigation_lock = threading.Lock()

    def fake_run(command, **kwargs):
        assert command == [
            "/test/terrain_map_to_points", "/maps/map042.bt", "12000",
            "0.1", "0.6", "/maps/map042_cloud.ply", "0.3", "35.0",
            "0.15",
        ]
        assert kwargs["timeout"] == 60.0
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


@pytest.mark.parametrize("payload", [
    {"x": 0.1, "y": -0.2},
    {"x": 0, "y": 0, "z": 0.3},
])
def test_navigation_goal_accepts_finite_coordinates(payload):
    assert parse_navigation_goal(payload) == (
        float(payload["x"]), float(payload["y"]), float(payload.get("z", 0.0)))


@pytest.mark.parametrize("payload", [
    {"x": True, "y": 0}, {"x": math.nan, "y": 0}, {"x": 0, "y": "bad"},
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
        "path_ready": False,
        "active": False,
    }
    started, message = node.start_navigation_motion()
    assert not started
    assert "valid path" in message
    assert len(published) == 1


@pytest.mark.parametrize("value", ["fast", True, None, math.inf, math.nan])
def test_invalid_velocity_is_rejected(value):
    with pytest.raises(ValueError):
        parse_velocity({"linear_x": value}, LIMITS)
