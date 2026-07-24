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

"""Serve a browser controller and publish safe, standard Twist commands."""

from dataclasses import dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import ipaddress
import json
import math
import os
from pathlib import Path
import re
import shlex
import signal
import socket
import struct
import subprocess
import threading
import time
from typing import Any, Dict, Optional, Tuple
from urllib.parse import urlsplit

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path as NavigationPath
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CompressedImage, PointCloud2, PointField
from visualization_msgs.msg import Marker


MAX_REQUEST_BYTES = 16 * 1024
SIOCGIFADDR = 0x8915
MAP_IDENTIFIER = re.compile(r"^map\d+$")
PLY_SCALAR_FORMATS = {
    "char": "b", "int8": "b", "uchar": "B", "uint8": "B",
    "short": "h", "int16": "h", "ushort": "H", "uint16": "H",
    "int": "i", "int32": "i", "uint": "I", "uint32": "I",
    "float": "f", "float32": "f", "double": "d", "float64": "d",
}


def _map_id_from_export_path(candidate: Path, octo_directory: Path) -> str:
    """Infer a mapNNN identifier from a file or one of its export folders."""
    map_id = candidate.stem if MAP_IDENTIFIER.fullmatch(candidate.stem) else ""
    if map_id:
        return map_id
    for parent in candidate.parents:
        if parent == octo_directory.parent:
            break
        if MAP_IDENTIFIER.fullmatch(parent.name):
            return parent.name
        matched = re.match(r"^(map\d+)_octomap$", parent.name)
        if matched:
            return matched.group(1)
    return ""


def discover_navigation_maps(maps_root: Path) -> list:
    """Return saved RTAB-Map/OctoMap pairs grouped by their mapNNN identifier."""
    rtab_directory = maps_root / "rtab_maps"
    octo_directory = maps_root / "octo_maps"
    databases = {
        candidate.stem: candidate.resolve()
        for candidate in rtab_directory.glob("map*.db")
        if MAP_IDENTIFIER.fullmatch(candidate.stem)
    }
    octomaps: Dict[str, Path] = {}
    for candidate in octo_directory.rglob("*.bt"):
        map_id = _map_id_from_export_path(candidate, octo_directory)
        if map_id:
            previous = octomaps.get(map_id)
            if previous is None or candidate.stat().st_mtime > previous.stat().st_mtime:
                octomaps[map_id] = candidate.resolve()
    clouds: Dict[str, Path] = {}
    for candidate in octo_directory.rglob("*_cloud.ply"):
        map_id = _map_id_from_export_path(candidate, octo_directory)
        if map_id:
            previous = clouds.get(map_id)
            if previous is None or candidate.stat().st_mtime > previous.stat().st_mtime:
                clouds[map_id] = candidate.resolve()
    maps = []
    for map_id in sorted(set(databases) | set(octomaps), key=lambda value: int(value[3:])):
        database = databases.get(map_id)
        octomap = octomaps.get(map_id)
        cloud = clouds.get(map_id)
        maps.append({
            "id": map_id,
            "database_path": str(database) if database else None,
            "octomap_path": str(octomap) if octomap else None,
            "cloud_path": str(cloud) if cloud else None,
            "loadable": database is not None and octomap is not None,
        })
    return maps


def extract_colored_ply_points(path: Path, max_points: int) -> list:
    """Read a bounded XYZRGB sample from a standard RTAB-Map/PCL PLY export."""
    with path.open("rb") as stream:
        if stream.readline().strip() != b"ply":
            raise ValueError("not a PLY file")
        file_format = None
        vertex_count = None
        properties = []
        in_vertex_element = False
        while True:
            raw_line = stream.readline()
            if not raw_line:
                raise ValueError("incomplete PLY header")
            words = raw_line.decode("ascii").strip().split()
            if words == ["end_header"]:
                break
            if words[:1] == ["format"] and len(words) >= 2:
                file_format = words[1]
            elif words[:2] == ["element", "vertex"] and len(words) == 3:
                vertex_count = int(words[2])
                in_vertex_element = True
            elif words[:1] == ["element"]:
                in_vertex_element = False
            elif in_vertex_element and words[:1] == ["property"]:
                if len(words) != 3 or words[1] == "list" or words[1] not in PLY_SCALAR_FORMATS:
                    raise ValueError("unsupported PLY vertex property")
                properties.append((words[2], words[1]))
        if file_format not in {"ascii", "binary_little_endian"}:
            raise ValueError("unsupported PLY format")
        if vertex_count is None:
            raise ValueError("PLY has no vertex element")
        names = [name for name, _ in properties]
        if not {"x", "y", "z"}.issubset(names):
            raise ValueError("PLY vertices need x, y and z")
        stride = max(1, (vertex_count + max_points - 1) // max_points)
        unpack = None
        record_size = 0
        if file_format == "binary_little_endian":
            format_string = "<" + "".join(PLY_SCALAR_FORMATS[kind] for _, kind in properties)
            unpack = struct.Struct(format_string).unpack
            record_size = struct.calcsize(format_string)
        points = []
        for index in range(vertex_count):
            if file_format == "ascii":
                values = stream.readline().decode("ascii").split()
                vertex = dict(zip(names, values))
            else:
                data = stream.read(record_size)
                if len(data) != record_size:
                    raise ValueError("truncated PLY vertex data")
                vertex = dict(zip(names, unpack(data)))
            if index % stride:
                continue
            try:
                x, y, z = (float(vertex[axis]) for axis in ("x", "y", "z"))
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError("invalid PLY vertex") from exc
            if not all(math.isfinite(value) for value in (x, y, z)):
                continue
            colors = [max(0, min(255, round(float(vertex.get(channel, 255)))))
                      for channel in ("red", "green", "blue")]
            points.append((round(x, 3), round(y, 3), round(z, 3), *colors))
    return points


def parse_octomap_point_output(output: str) -> Tuple[float, list]:
    """Parse bounded occupied voxel rows emitted by octomap_to_points."""
    lines = output.splitlines()
    if not lines:
        raise ValueError("OctoMap converter returned no data")
    header = lines[0].split()
    if len(header) != 2 or header[0] != "resolution":
        raise ValueError("invalid OctoMap converter header")
    try:
        resolution = float(header[1])
    except ValueError as exc:
        raise ValueError("invalid OctoMap resolution") from exc
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError("invalid OctoMap resolution")
    points = []
    for line in lines[1:]:
        values = line.split()
        if len(values) != 4:
            raise ValueError("invalid OctoMap voxel row")
        try:
            x, y, z, size = (float(value) for value in values)
        except ValueError as exc:
            raise ValueError("invalid OctoMap voxel row") from exc
        if not all(math.isfinite(value) for value in (x, y, z)):
            continue
        if not math.isfinite(size) or size <= 0.0:
            continue
        points.append((round(x, 3), round(y, 3), round(z, 3), round(size, 3)))
    return round(resolution, 4), points


def localization_covariance_ready(covariance: list, maximum: float) -> bool:
    """Return whether RTAB-Map has supplied a finite, confident map pose."""
    if len(covariance) < 36 or not math.isfinite(maximum) or maximum <= 0.0:
        return False
    values = (covariance[0], covariance[7], covariance[35])
    return all(math.isfinite(value) and 0.0 <= value <= maximum
               for value in values)


def parse_navigation_goal(payload: Dict[str, Any]) -> Tuple[float, float, float]:
    """Validate a map-frame browser goal without accepting NaN or booleans."""
    values = []
    for name in ("x", "y", "z"):
        raw_value = payload.get(name, 0.0)
        if isinstance(raw_value, bool) or not isinstance(raw_value, (int, float)):
            raise ValueError(f"{name} must be a number")
        value = float(raw_value)
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")
        values.append(value)
    return values[0], values[1], values[2]


def discover_lan_ipv4_addresses() -> list:
    """Return non-loopback IPv4 addresses assigned to local interfaces."""
    addresses = set()
    physical_addresses = set()
    try:
        import fcntl

        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            for _, interface_name in socket.if_nameindex():
                try:
                    request = struct.pack(
                        "256s",
                        interface_name[:15].encode("utf-8"),
                    )
                    response = fcntl.ioctl(sock.fileno(), SIOCGIFADDR, request)
                    address = socket.inet_ntoa(response[20:24])
                    parsed = ipaddress.ip_address(address)
                    if not parsed.is_loopback and not parsed.is_link_local:
                        addresses.add(address)
                        device_path = Path(
                            "/sys/class/net",
                            interface_name,
                            "device",
                        )
                        if device_path.exists():
                            physical_addresses.add(address)
                except (OSError, UnicodeError, ValueError):
                    continue
    except (ImportError, OSError):
        pass
    preferred_addresses = physical_addresses or addresses
    return sorted(
        preferred_addresses,
        key=lambda value: ipaddress.ip_address(value).packed,
    )


def make_access_urls(
    bind_address: str,
    port: int,
    lan_addresses: Optional[list] = None,
) -> list:
    """Build browser URLs appropriate for the configured listen address."""
    if bind_address == "0.0.0.0":
        addresses = (
            discover_lan_ipv4_addresses()
            if lan_addresses is None
            else lan_addresses
        )
        if not addresses:
            addresses = ["127.0.0.1"]
    else:
        addresses = [bind_address]
    return [f"http://{address}:{port}" for address in addresses]


def is_managed_web_control_command(command: str) -> bool:
    """Return whether a process command belongs to this web control package."""
    return "luxi_web_control" in command and "web_control_node" in command


def _listening_process_ids(port: int) -> list:
    """Return process IDs listening on a TCP port using Linux ss output."""
    try:
        result = subprocess.run(
            ["ss", "-H", "-ltnp", f"sport = :{port}"],
            capture_output=True,
            check=False,
            text=True,
            timeout=2.0,
        )
    except (OSError, subprocess.TimeoutExpired):
        return []
    matches = re.findall(r"pid=(\d+)", result.stdout)
    return sorted({int(match) for match in matches})


def _process_command(pid: int) -> str:
    """Return a process command line, or an empty string if it disappeared."""
    try:
        return Path(f"/proc/{pid}/cmdline").read_bytes().replace(
            b"\x00",
            b" ",
        ).decode("utf-8", errors="replace")
    except OSError:
        return ""


def stop_existing_web_control(
    port: int,
    timeout: float = 5.0,
) -> Tuple[bool, str]:
    """Stop only same-user luxi_web_control listeners on a requested port."""
    pids = _listening_process_ids(port)
    if not pids:
        return True, ""

    own_pids = []
    foreign_pids = []
    for pid in pids:
        try:
            same_user = os.stat(f"/proc/{pid}").st_uid == os.getuid()
        except OSError:
            continue
        if same_user and is_managed_web_control_command(_process_command(pid)):
            own_pids.append(pid)
        else:
            foreign_pids.append(pid)
    if foreign_pids:
        return False, (
            f"port {port} is held by a non-web-control process: {foreign_pids}"
        )
    if not own_pids:
        return False, (
            f"port {port} is occupied but its owner cannot be identified"
        )

    for pid in own_pids:
        try:
            os.kill(pid, signal.SIGINT)
        except ProcessLookupError:
            continue
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        remaining = _listening_process_ids(port)
        if not remaining:
            return True, (
                f"stopped previous web controller process(es): {own_pids}"
            )
        time.sleep(0.05)
    return False, f"previous web controller did not release port {port}"


@dataclass(frozen=True)
class VelocityCommand:
    """Planar velocity values used to construct a Twist message."""

    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0

    @property
    def moving(self) -> bool:
        """Return whether any commanded axis is non-zero."""
        return any(abs(value) > 1.0e-9 for value in self.as_tuple())

    def as_tuple(self) -> Tuple[float, float, float]:
        """Return values in x, y, yaw order."""
        return self.linear_x, self.linear_y, self.angular_z

    def as_dict(self) -> Dict[str, float]:
        """Return a JSON-friendly representation."""
        return {
            "linear_x": self.linear_x,
            "linear_y": self.linear_y,
            "angular_z": self.angular_z,
        }


def clamp(value: float, limit: float) -> float:
    """Clamp a signed value to a symmetric non-negative limit."""
    return max(-limit, min(limit, value))


def parse_velocity(
    payload: Dict[str, Any],
    limits: VelocityCommand,
) -> VelocityCommand:
    """Validate and clamp an API velocity payload."""
    values = []
    for name, limit in zip(
        ("linear_x", "linear_y", "angular_z"),
        limits.as_tuple(),
    ):
        raw_value = payload.get(name, 0.0)
        if (
            isinstance(raw_value, bool)
            or not isinstance(raw_value, (int, float))
        ):
            raise ValueError(f"{name} must be a number")
        value = float(raw_value)
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")
        values.append(clamp(value, limit))
    return VelocityCommand(*values)


def extract_sparse_cloud(
    message: PointCloud2,
    max_points: int,
) -> list:
    """Extract finite XYZRGB samples from a PointCloud2 message."""
    field_by_name = {field.name: field for field in message.fields}
    required_fields = ("x", "y", "z")
    if any(name not in field_by_name for name in required_fields):
        return []
    if message.point_step <= 0 or message.width <= 0 or message.height <= 0:
        return []

    total_points = message.width * message.height
    stride = max(1, (total_points + max_points - 1) // max_points)
    endian = ">" if message.is_bigendian else "<"
    x_field = field_by_name["x"]
    y_field = field_by_name["y"]
    z_field = field_by_name["z"]
    rgb_field = field_by_name.get("rgb") or field_by_name.get("rgba")
    data = message.data
    points = []

    for point_index in range(0, total_points, stride):
        row_index, column_index = divmod(point_index, message.width)
        offset = row_index * message.row_step + column_index * message.point_step
        try:
            x = struct.unpack_from(endian + "f", data, offset + x_field.offset)[0]
            y = struct.unpack_from(endian + "f", data, offset + y_field.offset)[0]
            z = struct.unpack_from(endian + "f", data, offset + z_field.offset)[0]
        except struct.error:
            continue
        if not all(math.isfinite(value) for value in (x, y, z)):
            continue

        red, green, blue = 210, 225, 235
        if rgb_field is not None:
            try:
                if rgb_field.datatype == PointField.FLOAT32:
                    rgb_float = struct.unpack_from(
                        endian + "f",
                        data,
                        offset + rgb_field.offset,
                    )[0]
                    rgb = struct.unpack(
                        endian + "I",
                        struct.pack(endian + "f", rgb_float),
                    )[0]
                elif rgb_field.datatype == PointField.UINT32:
                    rgb = struct.unpack_from(
                        endian + "I",
                        data,
                        offset + rgb_field.offset,
                    )[0]
                else:
                    rgb = 0
                red = (rgb >> 16) & 0xff
                green = (rgb >> 8) & 0xff
                blue = rgb & 0xff
            except struct.error:
                pass
        points.append(
            (round(x, 3), round(y, 3), round(z, 3), red, green, blue)
        )
    return points


class MappingController:
    """Own the RTAB-Map launch process started from the web interface."""

    def __init__(
        self,
        enabled: bool,
        package: str,
        launch_file: str,
        rmw_implementation: str,
        d435_setup: Path,
        workspace_setup: Path,
        log_path: Path,
    ) -> None:
        self.enabled = enabled
        self.package = package
        self.launch_file = launch_file
        self.rmw_implementation = rmw_implementation
        self.d435_setup = d435_setup
        self.workspace_setup = workspace_setup
        self.log_path = log_path
        self._lock = threading.Lock()
        self._process: Optional[subprocess.Popen] = None
        self._started_at: Optional[float] = None
        self._last_exit_code: Optional[int] = None
        self._last_error = ""
        self._stop_requested = False

    def _command(self) -> list:
        """Build a shell-free, source-aware RTAB-Map launch command."""
        source_commands = [
            f"source {shlex.quote(str(self.d435_setup))}",
            f"source {shlex.quote(str(self.workspace_setup))}",
            "export ROS_LOCALHOST_ONLY=0",
            "export RMW_IMPLEMENTATION="
            + shlex.quote(self.rmw_implementation),
        ]
        launch_command = shlex.join([
            "ros2",
            "launch",
            self.package,
            self.launch_file,
            "rviz:=false",
            "rtabmap_viz:=false",
        ])
        script = "set -e; " + "; ".join(source_commands)
        script += f"; exec {launch_command}"
        return ["/bin/bash", "-c", script]

    def start(self) -> Tuple[bool, str]:
        """Start a fresh managed RTAB-Map process when prerequisites exist."""
        with self._lock:
            if not self.enabled:
                return False, "mapping control is disabled"
            if self._process is not None and self._process.poll() is None:
                return False, "RTAB-Map mapping is already running"
            missing = [
                path
                for path in (self.d435_setup, self.workspace_setup)
                if not path.is_file()
            ]
            if missing:
                return False, (
                    "mapping setup file is missing: "
                    + ", ".join(str(path) for path in missing)
                )
            try:
                self.log_path.parent.mkdir(parents=True, exist_ok=True)
                with self.log_path.open("a", encoding="utf-8") as log_file:
                    log_file.write(
                        "\n===== RTAB-Map started by luxi_web_control =====\n"
                    )
                    self._process = subprocess.Popen(
                        self._command(),
                        stdout=log_file,
                        stderr=subprocess.STDOUT,
                        start_new_session=True,
                        env=os.environ.copy(),
                    )
            except OSError as exc:
                self._process = None
                self._last_error = str(exc)
                return False, f"unable to start RTAB-Map: {exc}"
            self._started_at = time.monotonic()
            self._last_exit_code = None
            self._last_error = ""
            self._stop_requested = False
            return True, "RTAB-Map launch process started"

    def stop(self) -> Tuple[bool, str]:
        """Gracefully stop only this controller's RTAB-Map process."""
        with self._lock:
            process = self._process
            if process is None or process.poll() is not None:
                self._update_exit_state_locked()
                return True, "RTAB-Map mapping is already stopped"
            self._stop_requested = True
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                self._update_exit_state_locked()
                return True, "RTAB-Map mapping is already stopped"

        try:
            process.wait(timeout=8.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=2.0)

        with self._lock:
            self._update_exit_state_locked()
        return True, "RTAB-Map mapping stopped"

    def _update_exit_state_locked(self) -> None:
        """Record a child exit code while holding the manager lock."""
        if self._process is None:
            return
        exit_code = self._process.poll()
        if exit_code is None:
            return
        self._last_exit_code = exit_code
        if exit_code != 0 and not self._stop_requested:
            self._last_error = self._latest_log_error()
        self._stop_requested = False
        self._process = None

    def _latest_log_error(self) -> str:
        """Extract the last useful error line from the managed launch log."""
        try:
            with self.log_path.open("rb") as log_file:
                log_file.seek(0, os.SEEK_END)
                size = log_file.tell()
                log_file.seek(max(0, size - 8192))
                log_text = log_file.read().decode("utf-8", errors="replace")
                lines = log_text.splitlines()
        except OSError:
            return "RTAB-Map exited; mapping log is unavailable"
        for line in reversed(lines):
            clean_line = line.strip()
            if "[ERROR]" in clean_line or "Caught exception" in clean_line:
                return clean_line
        return "RTAB-Map exited; inspect the mapping log"

    def status(self) -> Dict[str, Any]:
        """Return a JSON-friendly snapshot of the managed mapping process."""
        with self._lock:
            self._update_exit_state_locked()
            running = self._process is not None
            pid = self._process.pid if self._process is not None else None
            started_at = self._started_at
            exit_code = self._last_exit_code
            error = self._last_error
        if not self.enabled:
            state = "disabled"
        elif running:
            state = "running"
        elif exit_code not in (None, 0) and error:
            state = "failed"
        else:
            state = "stopped"
        return {
            "enabled": self.enabled,
            "state": state,
            "pid": pid,
            "uptime_seconds": (
                None if started_at is None or not running
                else round(time.monotonic() - started_at, 1)
            ),
            "last_exit_code": exit_code,
            "last_error": error,
            "log_path": str(self.log_path),
        }


class NavigationController:
    """Own selected-map localization and static OctoMap planning processes."""

    def __init__(
        self,
        enabled: bool,
        package: str,
        launch_file: str,
        rmw_implementation: str,
        d435_setup: Path,
        workspace_setup: Path,
        octomap_library_path: Path,
        log_path: Path,
    ) -> None:
        self.enabled = enabled
        self.package = package
        self.launch_file = launch_file
        self.rmw_implementation = rmw_implementation
        self.d435_setup = d435_setup
        self.workspace_setup = workspace_setup
        self.octomap_library_path = octomap_library_path
        self.log_path = log_path
        self._lock = threading.Lock()
        self._process: Optional[subprocess.Popen] = None
        self._started_at: Optional[float] = None
        self._last_exit_code: Optional[int] = None
        self._last_error = ""
        self._stop_requested = False
        self._map_id = ""

    def _command(self, database_path: Path, octomap_path: Path) -> list:
        source_commands = [
            f"source {shlex.quote(str(self.d435_setup))}",
            f"source {shlex.quote(str(self.workspace_setup))}",
            "export ROS_LOCALHOST_ONLY=0",
            "export RMW_IMPLEMENTATION=" + shlex.quote(self.rmw_implementation),
            "export LD_LIBRARY_PATH=" + shlex.quote(str(self.octomap_library_path))
            + ':${LD_LIBRARY_PATH:-}',
        ]
        launch_command = shlex.join([
            "ros2", "launch", self.package, self.launch_file,
            f"database_path:={database_path}",
            f"octomap_path:={octomap_path}",
            "cmd_vel_topic:=/navigation/cmd_vel",
        ])
        script = "set -e; " + "; ".join(source_commands)
        return ["/bin/bash", "-c", script + f"; exec {launch_command}"]

    def start(
        self,
        map_id: str,
        database_path: Path,
        octomap_path: Path,
    ) -> Tuple[bool, str]:
        with self._lock:
            if not self.enabled:
                return False, "navigation control is disabled"
            if self._process is not None and self._process.poll() is None:
                return False, "selected-map navigation is already running"
            missing = [
                path for path in (
                    self.d435_setup,
                    self.workspace_setup,
                    database_path,
                    octomap_path,
                )
                if not path.is_file()
            ]
            if missing:
                return False, (
                    "navigation prerequisite is missing: "
                    + ", ".join(str(path) for path in missing)
                )
            try:
                self.log_path.parent.mkdir(parents=True, exist_ok=True)
                with self.log_path.open("a", encoding="utf-8") as log_file:
                    log_file.write(
                        f"\n===== Navigation {map_id} started by "
                        "luxi_web_control =====\n"
                    )
                    self._process = subprocess.Popen(
                        self._command(database_path, octomap_path),
                        stdout=log_file, stderr=subprocess.STDOUT,
                        start_new_session=True, env=os.environ.copy())
            except OSError as exc:
                self._process = None
                self._last_error = str(exc)
                return False, f"unable to start navigation: {exc}"
            self._started_at = time.monotonic()
            self._last_exit_code = None
            self._last_error = ""
            self._stop_requested = False
            self._map_id = map_id
            return True, f"navigation map {map_id} is starting"

    def stop(self) -> Tuple[bool, str]:
        with self._lock:
            process = self._process
            if process is None or process.poll() is not None:
                self._update_exit_state_locked()
                return True, "selected-map navigation is already stopped"
            self._stop_requested = True
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                self._update_exit_state_locked()
                return True, "selected-map navigation is already stopped"
        try:
            process.wait(timeout=8.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=2.0)
        with self._lock:
            self._update_exit_state_locked()
        return True, "selected-map navigation stopped"

    def _update_exit_state_locked(self) -> None:
        if self._process is None:
            return
        exit_code = self._process.poll()
        if exit_code is None:
            return
        self._last_exit_code = exit_code
        if exit_code != 0 and not self._stop_requested:
            self._last_error = self._latest_log_error()
        self._stop_requested = False
        self._process = None

    def _latest_log_error(self) -> str:
        try:
            with self.log_path.open("rb") as log_file:
                log_file.seek(0, os.SEEK_END)
                log_file.seek(max(0, log_file.tell() - 8192))
                lines = log_file.read().decode("utf-8", errors="replace").splitlines()
        except OSError:
            return "navigation exited; navigation log is unavailable"
        for line in reversed(lines):
            if "[ERROR]" in line or "Caught exception" in line:
                return line.strip()
        return "navigation exited; inspect the navigation log"

    def status(self) -> Dict[str, Any]:
        with self._lock:
            self._update_exit_state_locked()
            running = self._process is not None
            snapshot = {
                "pid": self._process.pid if self._process else None,
                "uptime_seconds": (
                    None if not running or self._started_at is None
                    else round(time.monotonic() - self._started_at, 1)
                ),
                "last_exit_code": self._last_exit_code,
                "last_error": self._last_error,
                "map_id": self._map_id or None,
            }
        if not self.enabled:
            state = "disabled"
        elif running:
            state = "running"
        elif snapshot["last_exit_code"] not in (None, 0) and snapshot["last_error"]:
            state = "failed"
        else:
            state = "stopped"
        return {
            "enabled": self.enabled,
            "state": state,
            "log_path": str(self.log_path),
            **snapshot,
        }


class ControlHTTPServer(ThreadingHTTPServer):
    """Threaded HTTP server carrying a reference to its ROS node."""

    daemon_threads = True
    allow_reuse_address = True

    def __init__(
        self,
        address: Tuple[str, int],
        control_node: "WebControlNode",
    ):
        self.control_node = control_node
        super().__init__(address, ControlRequestHandler)


class ControlRequestHandler(BaseHTTPRequestHandler):
    """Small same-origin REST API and static file handler."""

    server: ControlHTTPServer

    def log_message(self, _format: str, *args: Any) -> None:
        """Keep normal requests out of stdout."""

    def _send_headers(
        self,
        status: int,
        content_type: str,
        content_length: int,
        cache_control: str = "no-store",
    ) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(content_length))
        self.send_header("Cache-Control", cache_control)
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.end_headers()

    def _send_json(self, status: int, body: Dict[str, Any]) -> None:
        data = json.dumps(body, ensure_ascii=False).encode("utf-8")
        self._send_headers(
            status,
            "application/json; charset=utf-8",
            len(data),
        )
        self.wfile.write(data)

    def _send_error_json(self, status: int, message: str) -> None:
        self._send_json(status, {"ok": False, "error": message})

    def _read_json(self) -> Dict[str, Any]:
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError as exc:
            raise ValueError("invalid Content-Length") from exc
        if length <= 0:
            return {}
        if length > MAX_REQUEST_BYTES:
            raise OverflowError("request body is too large")
        try:
            payload = json.loads(self.rfile.read(length).decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ValueError("request body must be valid JSON") from exc
        if not isinstance(payload, dict):
            raise ValueError("JSON body must be an object")
        return payload

    def do_OPTIONS(self) -> None:  # noqa: N802
        """Answer browser CORS preflight requests."""
        self._send_headers(HTTPStatus.NO_CONTENT, "text/plain", 0)

    def do_GET(self) -> None:  # noqa: N802
        """Serve current state or one of the bundled web assets."""
        path = urlsplit(self.path).path
        if path == "/api/status":
            self._send_json(HTTPStatus.OK, self.server.control_node.status())
            return
        if path == "/api/preview/rgb":
            image, content_type = self.server.control_node.rgb_preview()
            if image is None:
                self._send_error_json(HTTPStatus.NOT_FOUND, "RGB preview unavailable")
                return
            self._send_headers(HTTPStatus.OK, content_type, len(image))
            self.wfile.write(image)
            return
        if path == "/api/preview/cloud":
            self._send_json(
                HTTPStatus.OK,
                {"ok": True, "cloud": self.server.control_node.cloud_preview()},
            )
            return
        if path == "/api/navigation/maps":
            self._send_json(
                HTTPStatus.OK,
                {"ok": True, "maps": self.server.control_node.navigation_maps(),
                 "navigation": self.server.control_node.navigation_status()},
            )
            return
        if path == "/api/navigation/voxels":
            self._send_json(
                HTTPStatus.OK,
                {"ok": True, "voxels": self.server.control_node.voxel_preview()},
            )
            return
        if path == "/api/navigation/cloud":
            self._send_json(
                HTTPStatus.OK,
                {"ok": True, "cloud": self.server.control_node.navigation_cloud_preview()},
            )
            return
        if path == "/api/navigation/path":
            self._send_json(
                HTTPStatus.OK,
                {"ok": True, "path": self.server.control_node.path_preview()},
            )
            return

        assets = {
            "/": ("index.html", "text/html; charset=utf-8"),
            "/index.html": ("index.html", "text/html; charset=utf-8"),
            "/app.js": ("app.js", "text/javascript; charset=utf-8"),
            "/styles.css": ("styles.css", "text/css; charset=utf-8"),
        }
        asset = assets.get(path)
        if asset is None:
            self._send_error_json(HTTPStatus.NOT_FOUND, "not found")
            return
        filename, content_type = asset
        try:
            data = (self.server.control_node.web_root / filename).read_bytes()
        except OSError:
            self._send_error_json(
                HTTPStatus.INTERNAL_SERVER_ERROR,
                "web asset unavailable",
            )
            return
        self._send_headers(
            HTTPStatus.OK,
            content_type,
            len(data),
            cache_control="public, max-age=60",
        )
        self.wfile.write(data)

    def do_POST(self) -> None:  # noqa: N802
        """Accept motion, emergency-stop, and RTAB-Map lifecycle requests."""
        path = urlsplit(self.path).path
        try:
            payload = self._read_json()
        except OverflowError as exc:
            self._send_error_json(
                HTTPStatus.REQUEST_ENTITY_TOO_LARGE,
                str(exc),
            )
            return
        except ValueError as exc:
            self._send_error_json(HTTPStatus.BAD_REQUEST, str(exc))
            return

        node = self.server.control_node
        if path == "/api/cmd_vel":
            try:
                command = parse_velocity(payload, node.limits)
                accepted, reason = node.accept_command(command)
            except ValueError as exc:
                self._send_error_json(HTTPStatus.BAD_REQUEST, str(exc))
                return
            if not accepted:
                self._send_error_json(HTTPStatus.LOCKED, reason)
                return
            self._send_json(
                HTTPStatus.OK,
                {"ok": True, "command": command.as_dict()},
            )
            return
        if path == "/api/stop":
            node.stop_motion()
            self._send_json(HTTPStatus.OK, {"ok": True})
            return
        if path == "/api/estop":
            active = payload.get("active", True)
            if not isinstance(active, bool):
                self._send_error_json(
                    HTTPStatus.BAD_REQUEST,
                    "active must be boolean",
                )
                return
            node.set_estop(active)
            self._send_json(
                HTTPStatus.OK,
                {"ok": True, "estop_active": active},
            )
            return
        if path == "/api/mapping/start":
            started, message = node.start_mapping()
            if not started:
                self._send_error_json(HTTPStatus.CONFLICT, message)
                return
            self._send_json(
                HTTPStatus.ACCEPTED,
                {
                    "ok": True,
                    "message": message,
                    "mapping": node.mapping_status(),
                },
            )
            return
        if path == "/api/mapping/stop":
            stopped, message = node.stop_mapping()
            if not stopped:
                self._send_error_json(
                    HTTPStatus.INTERNAL_SERVER_ERROR,
                    message,
                )
                return
            self._send_json(
                HTTPStatus.OK,
                {
                    "ok": True,
                    "message": message,
                    "mapping": node.mapping_status(),
                },
            )
            return
        if path == "/api/navigation/load_map":
            map_id = payload.get("map_id")
            if not isinstance(map_id, str):
                self._send_error_json(HTTPStatus.BAD_REQUEST, "map_id must be a string")
                return
            loaded, message = node.load_navigation_map(map_id)
            if not loaded:
                self._send_error_json(HTTPStatus.CONFLICT, message)
                return
            self._send_json(HTTPStatus.ACCEPTED, {
                "ok": True, "message": message,
                "navigation": node.navigation_status(),
            })
            return
        if path == "/api/navigation/stop":
            stopped, message = node.stop_navigation()
            if not stopped:
                self._send_error_json(HTTPStatus.INTERNAL_SERVER_ERROR, message)
                return
            self._send_json(HTTPStatus.OK, {
                "ok": True, "message": message,
                "navigation": node.navigation_status(),
            })
            return
        if path == "/api/navigation/goal":
            try:
                x, y, z = parse_navigation_goal(payload)
                accepted, message = node.set_navigation_goal(x, y, z)
            except ValueError as exc:
                self._send_error_json(HTTPStatus.BAD_REQUEST, str(exc))
                return
            if not accepted:
                self._send_error_json(HTTPStatus.CONFLICT, message)
                return
            self._send_json(HTTPStatus.ACCEPTED, {
                "ok": True, "message": message, "goal": {"x": x, "y": y, "z": z},
            })
            return
        self._send_error_json(HTTPStatus.NOT_FOUND, "not found")


class WebControlNode(Node):
    """Publish browser velocity requests as standard geometry_msgs/Twist."""

    def __init__(self, parameter_overrides: Optional[list] = None) -> None:
        super().__init__(
            "web_control",
            parameter_overrides=parameter_overrides,
        )
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("bind_address", "0.0.0.0")
        self.declare_parameter("http_port", 8080)
        self.declare_parameter("auto_stop_existing_web_control", True)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("command_timeout", 0.6)
        self.declare_parameter("max_linear_x", 0.25)
        self.declare_parameter("max_linear_y", 0.0)
        self.declare_parameter("max_angular_z", 0.8)
        self.declare_parameter("enable_output", True)
        self.declare_parameter("web_root", "")
        self.declare_parameter("enable_mapping_control", True)
        self.declare_parameter("mapping_launch_package", "luxi_rtab_map")
        self.declare_parameter("mapping_launch_file", "rgbd_mapping.launch.py")
        self.declare_parameter(
            "mapping_rmw_implementation",
            "rmw_cyclonedds_cpp",
        )
        self.declare_parameter("mapping_d435_setup", "")
        self.declare_parameter("mapping_workspace_setup", "")
        self.declare_parameter("mapping_log_path", "")
        self.declare_parameter("enable_navigation_control", True)
        self.declare_parameter("navigation_launch_package", "luxi_voxel_navigation")
        self.declare_parameter("navigation_launch_file", "saved_map_navigation.launch.py")
        self.declare_parameter("navigation_rmw_implementation", "rmw_cyclonedds_cpp")
        self.declare_parameter("navigation_d435_setup", "")
        self.declare_parameter("navigation_workspace_setup", "")
        self.declare_parameter("navigation_log_path", "")
        self.declare_parameter("maps_root", "")
        self.declare_parameter("navigation_goal_topic", "/navigation/goal_pose")
        self.declare_parameter("navigation_marker_topic", "/navigation/occupied_voxels")
        self.declare_parameter("navigation_path_topic", "/navigation/planned_path")
        self.declare_parameter(
            "navigation_localization_pose_topic", "/rtabmap/localization_pose"
        )
        self.declare_parameter("navigation_localization_max_variance", 100.0)
        self.declare_parameter("navigation_localization_timeout", 3.0)
        self.declare_parameter("max_voxel_points", 12000)
        self.declare_parameter("enable_preview", True)
        self.declare_parameter(
            "rgb_preview_topic",
            "/camera/camera/color/image_raw/compressed",
        )
        self.declare_parameter("cloud_preview_topic", "/rtabmap/cloud_map")
        self.declare_parameter("max_cloud_points", 1800)

        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        bind_address = str(self.get_parameter("bind_address").value)
        http_port = int(self.get_parameter("http_port").value)
        auto_stop_existing = bool(
            self.get_parameter("auto_stop_existing_web_control").value
        )
        publish_rate = float(self.get_parameter("publish_rate").value)
        self.command_timeout = float(
            self.get_parameter("command_timeout").value
        )
        self.output_enabled = bool(self.get_parameter("enable_output").value)
        self.limits = VelocityCommand(
            float(self.get_parameter("max_linear_x").value),
            float(self.get_parameter("max_linear_y").value),
            float(self.get_parameter("max_angular_z").value),
        )
        package_share = Path(
            get_package_share_directory("luxi_web_control")
        ).resolve()
        web_root = str(self.get_parameter("web_root").value)
        if not web_root:
            web_root = str(package_share / "web")
        self.web_root = Path(web_root).resolve()

        workspace_root = package_share.parents[3]
        d435_setup = str(self.get_parameter("mapping_d435_setup").value)
        workspace_setup = str(
            self.get_parameter("mapping_workspace_setup").value
        )
        mapping_log_path = str(self.get_parameter("mapping_log_path").value)
        self.mapping = MappingController(
            enabled=bool(self.get_parameter("enable_mapping_control").value),
            package=str(self.get_parameter("mapping_launch_package").value),
            launch_file=str(self.get_parameter("mapping_launch_file").value),
            rmw_implementation=str(
                self.get_parameter("mapping_rmw_implementation").value
            ),
            d435_setup=Path(
                d435_setup
                or workspace_root / "device/D435i/ros2_ws/install/setup.bash"
            ).resolve(),
            workspace_setup=Path(
                workspace_setup or workspace_root / "install/setup.bash"
            ).resolve(),
            log_path=Path(
                mapping_log_path
                or workspace_root / "log/luxi_web_control_rtabmap.log"
            ).resolve(),
        )
        navigation_d435_setup = str(self.get_parameter("navigation_d435_setup").value)
        navigation_workspace_setup = str(
            self.get_parameter("navigation_workspace_setup").value
        )
        navigation_log_path = str(self.get_parameter("navigation_log_path").value)
        maps_root = str(self.get_parameter("maps_root").value)
        self.maps_root = Path(maps_root or workspace_root / "maps").resolve()
        self.octomap_points_executable = (
            workspace_root / "install/luxi_voxel_navigation/lib/"
            "luxi_voxel_navigation/octomap_to_points"
        ).resolve()
        self.navigation_goal_topic = str(
            self.get_parameter("navigation_goal_topic").value
        )
        self.navigation_marker_topic = str(
            self.get_parameter("navigation_marker_topic").value
        )
        self.navigation_path_topic = str(
            self.get_parameter("navigation_path_topic").value
        )
        self.navigation_localization_pose_topic = str(
            self.get_parameter("navigation_localization_pose_topic").value
        )
        self.navigation_localization_max_variance = float(
            self.get_parameter("navigation_localization_max_variance").value
        )
        self.navigation_localization_timeout = float(
            self.get_parameter("navigation_localization_timeout").value
        )
        self.max_voxel_points = int(self.get_parameter("max_voxel_points").value)
        self.navigation = NavigationController(
            enabled=bool(self.get_parameter("enable_navigation_control").value),
            package=str(self.get_parameter("navigation_launch_package").value),
            launch_file=str(self.get_parameter("navigation_launch_file").value),
            rmw_implementation=str(
                self.get_parameter("navigation_rmw_implementation").value
            ),
            d435_setup=Path(
                navigation_d435_setup
                or workspace_root / "device/D435i/ros2_ws/install/setup.bash"
            ).resolve(),
            workspace_setup=Path(
                navigation_workspace_setup or workspace_root / "install/setup.bash"
            ).resolve(),
            octomap_library_path=(
                workspace_root / "3parts/octomap/install/lib"
            ).resolve(),
            log_path=Path(
                navigation_log_path
                or workspace_root / "log/luxi_web_control_navigation.log"
            ).resolve(),
        )
        self.preview_enabled = bool(self.get_parameter("enable_preview").value)
        self.rgb_preview_topic = str(
            self.get_parameter("rgb_preview_topic").value
        )
        self.cloud_preview_topic = str(
            self.get_parameter("cloud_preview_topic").value
        )
        self.max_cloud_points = int(
            self.get_parameter("max_cloud_points").value
        )

        self._validate_parameters(http_port, publish_rate)
        replacement_message = ""
        if auto_stop_existing and http_port != 0:
            replaced, replacement_message = stop_existing_web_control(
                http_port
            )
            if not replaced:
                raise RuntimeError(replacement_message)
        self._lock = threading.Lock()
        self._command = VelocityCommand()
        self._last_command_time: Optional[float] = None
        self._timed_out = False
        self._estop_active = False
        self._closed = False
        self._preview_lock = threading.Lock()
        self._rgb_image: Optional[bytes] = None
        self._rgb_content_type = "image/jpeg"
        self._rgb_received_at: Optional[float] = None
        self._cloud_points = []
        self._cloud_frame_id = ""
        self._cloud_received_at: Optional[float] = None
        self._navigation_lock = threading.Lock()
        self._voxel_points = []
        self._voxel_frame_id = "map"
        self._voxel_resolution = 0.0
        self._voxel_received_at: Optional[float] = None
        self._navigation_voxel_map_id: Optional[str] = None
        self._navigation_voxel_error = ""
        self._localization_ready = False
        self._localization_variance: Optional[float] = None
        self._localization_received_at: Optional[float] = None
        self._navigation_cloud_points = []
        self._navigation_cloud_map_id: Optional[str] = None
        self._navigation_cloud_error = ""
        self._planned_path_points = []
        self._path_frame_id = "map"
        self._path_received_at: Optional[float] = None

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, qos)
        self.navigation_goal_publisher = self.create_publisher(
            PoseStamped, self.navigation_goal_topic, qos
        )
        navigation_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.navigation_marker_subscription = self.create_subscription(
            Marker,
            self.navigation_marker_topic,
            self._on_navigation_marker,
            navigation_qos,
        )
        self.navigation_path_subscription = self.create_subscription(
            NavigationPath,
            self.navigation_path_topic,
            self._on_navigation_path,
            navigation_qos,
        )
        localization_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.navigation_localization_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            self.navigation_localization_pose_topic,
            self._on_navigation_localization_pose,
            localization_qos,
        )
        if self.preview_enabled:
            image_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            cloud_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.rgb_subscription = self.create_subscription(
                CompressedImage,
                self.rgb_preview_topic,
                self._on_rgb_preview,
                image_qos,
            )
            self.cloud_subscription = self.create_subscription(
                PointCloud2,
                self.cloud_preview_topic,
                self._on_cloud_preview,
                cloud_qos,
            )
        self.timer = self.create_timer(
            1.0 / publish_rate,
            self._on_publish_timer,
        )

        self.http_server = ControlHTTPServer((bind_address, http_port), self)
        self.http_port = int(self.http_server.server_address[1])
        self.access_urls = make_access_urls(bind_address, self.http_port)
        self._http_thread = threading.Thread(
            target=self.http_server.serve_forever,
            kwargs={"poll_interval": 0.1},
            name="luxi-web-control-http",
            daemon=True,
        )
        self._http_thread.start()
        url_lines = "\n  ".join(self.access_urls)
        self.get_logger().info(
            f"Web control is publishing Twist on {self.cmd_vel_topic}\n"
            f"Open one of these addresses in a browser:\n  {url_lines}"
        )
        if replacement_message:
            self.get_logger().info(replacement_message)
        if not self.output_enabled:
            self.get_logger().warning(
                "enable_output is false; only zero velocity is published"
            )

    def _validate_parameters(
        self,
        http_port: int,
        publish_rate: float,
    ) -> None:
        if not 0 <= http_port <= 65535:
            raise ValueError("http_port must be between 0 and 65535")
        if publish_rate <= 0.0:
            raise ValueError("publish_rate must be greater than zero")
        if self.command_timeout <= 0.0:
            raise ValueError("command_timeout must be greater than zero")
        if not 100 <= self.max_cloud_points <= 20000:
            raise ValueError("max_cloud_points must be between 100 and 20000")
        if not 100 <= self.max_voxel_points <= 50000:
            raise ValueError("max_voxel_points must be between 100 and 50000")
        if self.navigation_localization_max_variance <= 0.0:
            raise ValueError("navigation_localization_max_variance must be positive")
        if self.navigation_localization_timeout <= 0.0:
            raise ValueError("navigation_localization_timeout must be positive")
        if not self.cmd_vel_topic:
            raise ValueError("cmd_vel_topic must not be empty")
        invalid_limits = any(
            value < 0.0 or not math.isfinite(value)
            for value in self.limits.as_tuple()
        )
        if invalid_limits:
            raise ValueError("velocity limits must be finite and non-negative")
        if not self.web_root.is_dir():
            raise ValueError(f"web_root is not a directory: {self.web_root}")

    def _twist(self, command: VelocityCommand) -> Twist:
        message = Twist()
        if self.output_enabled and not self._estop_active:
            message.linear.x = command.linear_x
            message.linear.y = command.linear_y
            message.angular.z = command.angular_z
        return message

    def _publish(self, command: VelocityCommand) -> None:
        self.publisher.publish(self._twist(command))

    def _on_publish_timer(self) -> None:
        now = time.monotonic()
        with self._lock:
            if (
                self._last_command_time is not None
                and now - self._last_command_time > self.command_timeout
            ):
                self._command = VelocityCommand()
                self._last_command_time = None
                self._timed_out = True
            command = self._command
        self._publish(command)

    def _on_rgb_preview(self, message: CompressedImage) -> None:
        """Keep the newest compressed camera image for HTTP preview requests."""
        image_format = message.format.lower()
        content_type = "image/png" if "png" in image_format else "image/jpeg"
        with self._preview_lock:
            self._rgb_image = bytes(message.data)
            self._rgb_content_type = content_type
            self._rgb_received_at = time.monotonic()

    def _on_cloud_preview(self, message: PointCloud2) -> None:
        """Keep a bounded sparse RGB point cloud for browser rendering."""
        points = extract_sparse_cloud(message, self.max_cloud_points)
        with self._preview_lock:
            self._cloud_points = points
            self._cloud_frame_id = message.header.frame_id
            self._cloud_received_at = time.monotonic()

    def _on_navigation_marker(self, message: Marker) -> None:
        """Cache a bounded occupied-voxel marker for the browser top-down view."""
        if message.type != Marker.CUBE_LIST:
            return
        points = message.points
        stride = max(1, (len(points) + self.max_voxel_points - 1) // self.max_voxel_points)
        cached = [
            (round(point.x, 3), round(point.y, 3), round(point.z, 3))
            for point in points[::stride]
            if all(math.isfinite(value) for value in (point.x, point.y, point.z))
        ]
        with self._navigation_lock:
            self._voxel_points = cached
            self._voxel_frame_id = message.header.frame_id or "map"
            self._voxel_resolution = float(message.scale.x)
            self._voxel_received_at = time.monotonic()

    def _on_navigation_path(self, message: NavigationPath) -> None:
        """Cache the latest global path; the planner publishes it transiently."""
        points = [
            (round(pose.pose.position.x, 3), round(pose.pose.position.y, 3),
             round(pose.pose.position.z, 3))
            for pose in message.poses
            if all(math.isfinite(value) for value in (
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z))
        ]
        with self._navigation_lock:
            self._planned_path_points = points
            self._path_frame_id = message.header.frame_id or "map"
            self._path_received_at = time.monotonic()

    def _on_navigation_localization_pose(
        self, message: PoseWithCovarianceStamped
    ) -> None:
        covariance = list(message.pose.covariance)
        values = (
            (covariance[0], covariance[7], covariance[35])
            if len(covariance) >= 36 else ()
        )
        with self._navigation_lock:
            self._localization_ready = localization_covariance_ready(
                covariance, self.navigation_localization_max_variance)
            self._localization_variance = (
                max(values) if values and all(math.isfinite(value) for value in values)
                else None
            )
            self._localization_received_at = time.monotonic()

    def _clear_cloud_preview(self) -> None:
        """Discard map data which belongs to a previous mapping session."""
        with self._preview_lock:
            self._cloud_points = []
            self._cloud_frame_id = ""
            self._cloud_received_at = None

    def _clear_navigation_preview(self) -> None:
        with self._navigation_lock:
            self._voxel_points = []
            self._voxel_received_at = None
            self._navigation_voxel_map_id = None
            self._navigation_voxel_error = ""
            self._localization_ready = False
            self._localization_variance = None
            self._localization_received_at = None
            self._navigation_cloud_points = []
            self._navigation_cloud_map_id = None
            self._navigation_cloud_error = ""
            self._planned_path_points = []
            self._path_received_at = None

    def rgb_preview(self) -> Tuple[Optional[bytes], str]:
        """Return the latest compressed RGB frame and its MIME type."""
        with self._preview_lock:
            return self._rgb_image, self._rgb_content_type

    def cloud_preview(self) -> Dict[str, Any]:
        """Return a bounded point-cloud snapshot for the browser Canvas."""
        with self._preview_lock:
            preview = self._cloud_preview_summary_locked()
            preview["points"] = list(self._cloud_points)
            return preview

    def voxel_preview(self) -> Dict[str, Any]:
        """Return occupied voxels in the selected static map for Canvas rendering."""
        with self._navigation_lock:
            return {
                "map_id": self._navigation_voxel_map_id,
                "frame_id": self._voxel_frame_id,
                "resolution": self._voxel_resolution,
                "point_count": len(self._voxel_points),
                "error": self._navigation_voxel_error or None,
                "age_seconds": None if self._voxel_received_at is None
                else round(time.monotonic() - self._voxel_received_at, 2),
                "points": list(self._voxel_points),
            }

    def _load_navigation_voxels(self, map_id: str, octomap_path: str) -> str:
        """Load saved occupied voxels immediately, without waiting for ROS launch."""
        command = [
            str(self.octomap_points_executable), str(octomap_path),
            str(self.max_voxel_points),
        ]
        environment = os.environ.copy()
        library_path = str(self.navigation.octomap_library_path)
        environment["LD_LIBRARY_PATH"] = (
            library_path + ":" + environment.get("LD_LIBRARY_PATH", "")
        )
        try:
            result = subprocess.run(
                command, check=True, capture_output=True, text=True,
                timeout=12.0, env=environment,
            )
            resolution, points = parse_octomap_point_output(result.stdout)
        except (OSError, subprocess.SubprocessError, UnicodeDecodeError,
                ValueError) as exc:
            error = str(exc)
            with self._navigation_lock:
                self._voxel_points = []
                self._navigation_voxel_map_id = map_id
                self._navigation_voxel_error = error
                self._voxel_received_at = None
            return error
        with self._navigation_lock:
            self._voxel_points = points
            self._voxel_frame_id = "map"
            self._voxel_resolution = resolution
            self._voxel_received_at = time.monotonic()
            self._navigation_voxel_map_id = map_id
            self._navigation_voxel_error = ""
        return ""

    def navigation_cloud_preview(self) -> Dict[str, Any]:
        """Return the selected saved map's bounded RGB point cloud."""
        with self._navigation_lock:
            return {
                "map_id": self._navigation_cloud_map_id,
                "point_count": len(self._navigation_cloud_points),
                "error": self._navigation_cloud_error or None,
                "points": list(self._navigation_cloud_points),
            }

    def _load_navigation_cloud(self, map_id: str, cloud_path: Optional[str]) -> str:
        if not cloud_path:
            with self._navigation_lock:
                self._navigation_cloud_points = []
                self._navigation_cloud_map_id = map_id
                self._navigation_cloud_error = "no exported colored PLY is available"
            return self._navigation_cloud_error
        try:
            points = extract_colored_ply_points(
                Path(cloud_path), self.max_cloud_points)
        except (OSError, UnicodeDecodeError, ValueError, struct.error) as exc:
            with self._navigation_lock:
                self._navigation_cloud_points = []
                self._navigation_cloud_map_id = map_id
                self._navigation_cloud_error = str(exc)
            return self._navigation_cloud_error
        with self._navigation_lock:
            self._navigation_cloud_points = points
            self._navigation_cloud_map_id = map_id
            self._navigation_cloud_error = ""
        return ""

    def path_preview(self) -> Dict[str, Any]:
        """Return the latest A* global path for Canvas rendering."""
        with self._navigation_lock:
            return {
                "frame_id": self._path_frame_id,
                "point_count": len(self._planned_path_points),
                "age_seconds": None if self._path_received_at is None
                else round(time.monotonic() - self._path_received_at, 2),
                "points": list(self._planned_path_points),
            }

    def preview_status(self) -> Dict[str, Any]:
        """Return lightweight preview availability without cloud point data."""
        with self._preview_lock:
            rgb_received_at = self._rgb_received_at
            return {
                "enabled": self.preview_enabled,
                "rgb_topic": self.rgb_preview_topic,
                "rgb_age_seconds": (
                    None if rgb_received_at is None
                    else round(time.monotonic() - rgb_received_at, 2)
                ),
                "cloud_topic": self.cloud_preview_topic,
                "cloud": self._cloud_preview_summary_locked(),
            }

    def _cloud_preview_summary_locked(self) -> Dict[str, Any]:
        """Return cloud metadata while the preview lock is held."""
        received_at = self._cloud_received_at
        return {
            "frame_id": self._cloud_frame_id,
            "age_seconds": (
                None if received_at is None
                else round(time.monotonic() - received_at, 2)
            ),
            "point_count": len(self._cloud_points),
        }

    def accept_command(self, command: VelocityCommand) -> Tuple[bool, str]:
        """Store and immediately publish a validated browser command."""
        with self._lock:
            if self._estop_active and command.moving:
                return False, "emergency stop is active"
            self._command = command
            self._last_command_time = (
                time.monotonic() if command.moving else None
            )
            self._timed_out = False
        self._publish(command)
        return True, ""

    def stop_motion(self) -> None:
        """Clear motion and immediately publish zero velocity."""
        with self._lock:
            self._command = VelocityCommand()
            self._last_command_time = None
            self._timed_out = False
        self._publish(VelocityCommand())

    def set_estop(self, active: bool) -> None:
        """Set the sticky software emergency stop; activation always stops."""
        with self._lock:
            self._estop_active = active
            self._command = VelocityCommand()
            self._last_command_time = None
            self._timed_out = False
        self._publish(VelocityCommand())

    def start_mapping(self) -> Tuple[bool, str]:
        """Start the managed RTAB-Map RGB-D mapping launch."""
        started, message = self.mapping.start()
        if started:
            self._clear_cloud_preview()
        return started, message

    def stop_mapping(self) -> Tuple[bool, str]:
        """Stop the managed RTAB-Map RGB-D mapping launch."""
        stopped, message = self.mapping.stop()
        if stopped:
            self._clear_cloud_preview()
        return stopped, message

    def mapping_status(self) -> Dict[str, Any]:
        """Return the state of the mapping process owned by this node."""
        return self.mapping.status()

    def navigation_maps(self) -> list:
        """Discover selectable pairs without exposing arbitrary filesystem paths."""
        return discover_navigation_maps(self.maps_root)

    def load_navigation_map(self, map_id: str) -> Tuple[bool, str]:
        """Load static layers, then start localization/planning when available."""
        record = next(
            (item for item in self.navigation_maps() if item["id"] == map_id),
            None,
        )
        if record is None:
            return False, f"map {map_id} does not exist under {self.maps_root}"
        if not record["loadable"]:
            return False, f"map {map_id} needs both .db and .bt files"
        self.navigation.stop()
        self._clear_navigation_preview()
        cloud_error = self._load_navigation_cloud(map_id, record.get("cloud_path"))
        voxel_error = self._load_navigation_voxels(map_id, record["octomap_path"])
        started, message = self.navigation.start(
            map_id,
            Path(record["database_path"]),
            Path(record["octomap_path"]),
        )
        preview_errors = []
        for label, error in (("colored cloud", cloud_error),
                             ("OctoMap voxels", voxel_error)):
            if error:
                preview_errors.append(label + ": " + error)
        if not started:
            return True, "map layers loaded; navigation unavailable: " + message + (
                "; " + "; ".join(preview_errors) if preview_errors else "")
        return True, message + (
            "; " + "; ".join(preview_errors) if preview_errors else "")

    def stop_navigation(self) -> Tuple[bool, str]:
        """Stop localization/planning while retaining the currently loaded map."""
        stopped, message = self.navigation.stop()
        return stopped, message

    def set_navigation_goal(self, x: float, y: float, z: float) -> Tuple[bool, str]:
        """Publish a map-frame goal only after RTAB-Map localization is ready."""
        navigation = self.navigation_status()
        if navigation["state"] != "running":
            return False, "load a map and wait for navigation to start first"
        if not navigation["localization_ready"]:
            return False, "wait for RTAB-Map localization before selecting a goal"
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        with self._navigation_lock:
            goal.header.frame_id = self._voxel_frame_id or "map"
            self._planned_path_points = []
            self._path_received_at = None
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        goal.pose.orientation.w = 1.0
        self.navigation_goal_publisher.publish(goal)
        return True, "goal sent to voxel A* planner"

    def navigation_status(self) -> Dict[str, Any]:
        """Return selected-map navigation state without large preview payloads."""
        status = self.navigation.status()
        with self._navigation_lock:
            age = None if self._localization_received_at is None else round(
                time.monotonic() - self._localization_received_at, 2)
            ready = self._localization_ready and age is not None and (
                age <= self.navigation_localization_timeout)
            status["localization_ready"] = (
                status["state"] == "running" and ready)
            status["localization_variance"] = self._localization_variance
            status["localization_age_seconds"] = age
        return status

    def status(self) -> Dict[str, Any]:
        """Return a thread-safe, JSON-ready controller status snapshot."""
        now = time.monotonic()
        with self._lock:
            command = self._command
            age = (
                None
                if self._last_command_time is None
                else max(0.0, now - self._last_command_time)
            )
            estop_active = self._estop_active
            timed_out = self._timed_out
        if not self.output_enabled:
            state = "disabled"
        elif estop_active:
            state = "estop"
        elif timed_out:
            state = "timeout"
        elif command.moving:
            state = "moving"
        else:
            state = "idle"
        return {
            "ok": True,
            "node": self.get_name(),
            "state": state,
            "cmd_vel_topic": self.cmd_vel_topic,
            "http_port": self.http_port,
            "access_urls": self.access_urls,
            "output_enabled": self.output_enabled,
            "estop_active": estop_active,
            "timed_out": timed_out,
            "command_age": age,
            "command_timeout": self.command_timeout,
            "command": command.as_dict(),
            "limits": self.limits.as_dict(),
            "subscriber_count": self.publisher.get_subscription_count(),
            "mapping": self.mapping_status(),
            "navigation": self.navigation_status(),
            "preview": self.preview_status(),
        }

    def close(self) -> None:
        """Stop the HTTP service and leave the robot with a zero command."""
        if self._closed:
            return
        self._closed = True
        try:
            self.stop_mapping()
            self.stop_navigation()
            if rclpy.ok():
                for _ in range(3):
                    self.stop_motion()
                    time.sleep(0.02)
        finally:
            self.http_server.shutdown()
            self.http_server.server_close()
            if self._http_thread.is_alive():
                self._http_thread.join(timeout=1.0)


def main(args: Optional[list] = None) -> None:
    """Run the web control ROS node."""
    # Keep ROS alive during Ctrl+C so close() can publish zero first.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node: Optional[WebControlNode] = None
    try:
        node = WebControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
