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
import struct

import pytest

from pathlib import Path

from sensor_msgs.msg import PointCloud2, PointField

from luxi_web_control.web_control_node import extract_sparse_cloud
from luxi_web_control.web_control_node import MappingController
from luxi_web_control.web_control_node import is_managed_web_control_command
from luxi_web_control.web_control_node import make_access_urls
from luxi_web_control.web_control_node import parse_velocity, VelocityCommand


LIMITS = VelocityCommand(0.25, 0.1, 0.8)


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


def test_mapping_start_requires_existing_setup_files(tmp_path):
    controller = MappingController(
        enabled=True,
        package="luxi_rtab_map",
        launch_file="rgbd_mapping.launch.py",
        rmw_implementation="rmw_cyclonedds_cpp",
        d435_setup=Path(tmp_path / "missing_d435_setup.bash"),
        workspace_setup=Path(tmp_path / "missing_workspace_setup.bash"),
        log_path=Path(tmp_path / "mapping.log"),
    )
    started, message = controller.start()
    assert not started
    assert "mapping setup file is missing" in message


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
        d435_setup=Path(tmp_path / "d435_setup.bash"),
        workspace_setup=Path(tmp_path / "workspace_setup.bash"),
        log_path=log_path,
    )
    assert controller._latest_log_error() == (
        "[ERROR] camera input is unavailable"
    )


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

    assert extract_sparse_cloud(cloud, 10) == [
        (1.0, 2.0, 3.0, 17, 34, 51),
        (-1.0, 0.5, 0.25, 17, 34, 51),
    ]


@pytest.mark.parametrize("value", ["fast", True, None, math.inf, math.nan])
def test_invalid_velocity_is_rejected(value):
    with pytest.raises(ValueError):
        parse_velocity({"linear_x": value}, LIMITS)
