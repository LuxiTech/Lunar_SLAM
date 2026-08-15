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
import sqlite3
import struct
import subprocess
import threading
import time
from typing import Any, Dict, Optional, Tuple
from urllib.parse import parse_qs, urlsplit

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path as NavigationPath
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import BatteryState, CompressedImage, PointCloud2, PointField
from std_msgs.msg import Bool, Float32, Float64, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker


MAX_REQUEST_BYTES = 2 * 1024 * 1024
SIOCGIFADDR = 0x8915
MAP_IDENTIFIER = re.compile(r"^map\d+$")
CONTROL_CLIENT_IDENTIFIER = re.compile(r"^[A-Za-z0-9._:-]{8,128}$")
LEGACY_CONTROL_CLIENT_ID = "legacy-http-client"
CONTROL_DEFERRED_GET_PATHS = frozenset({
    "/api/preview/cloud",
    "/api/navigation/voxels",
    "/api/navigation/cloud",
    "/api/navigation/path",
    "/api/navigation/terrain",
})
PLY_SCALAR_FORMATS = {
    "char": "b", "int8": "b", "uchar": "B", "uint8": "B",
    "short": "h", "int16": "h", "ushort": "H", "uint16": "H",
    "int": "i", "int32": "i", "uint": "I", "uint32": "I",
    "float": "f", "float32": "f", "double": "d", "float64": "d",
}
MAPPING_NODE_PATHS = frozenset({
    "/luxi_visual_frontend",
    "/rtabmap/rtabmap",
})
ROBOT_NAMESPACE_PATTERN = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
CAMERA_QUATERNION_ARGUMENTS = ("camera_qx", "camera_qy", "camera_qz", "camera_qw")


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Tuple[float, ...]:
    """Return a normalized ROS quaternion for intrinsic roll/pitch/yaw."""
    half_roll = roll * 0.5
    half_pitch = pitch * 0.5
    half_yaw = yaw * 0.5
    cr, sr = math.cos(half_roll), math.sin(half_roll)
    cp, sp = math.cos(half_pitch), math.sin(half_pitch)
    cy, sy = math.cos(half_yaw), math.sin(half_yaw)
    quaternion = (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )
    norm = math.sqrt(sum(value * value for value in quaternion))
    if not math.isfinite(norm) or norm < 1.0e-9:
        raise ValueError("camera calibration produced an invalid quaternion")
    return tuple(value / norm for value in quaternion)


def camera_calibration_overrides(
    calibration: Dict[str, Any], camera_yaw_degrees: float
) -> Dict[str, str]:
    """Convert a completed gravity calibration into ROS launch arguments."""
    if not calibration.get("calibrated"):
        return {}
    try:
        roll = math.radians(float(calibration["roll_degrees"]))
        pitch = math.radians(float(calibration["pitch_degrees"]))
        yaw = math.radians(float(camera_yaw_degrees))
    except (KeyError, TypeError, ValueError, OverflowError) as exc:
        raise ValueError("saved camera calibration is incomplete") from exc
    if not all(math.isfinite(value) for value in (roll, pitch, yaw)):
        raise ValueError("saved camera calibration contains non-finite angles")
    quaternion = quaternion_from_rpy(roll, pitch, yaw)
    return {
        name: format(value, ".12g")
        for name, value in zip(CAMERA_QUATERNION_ARGUMENTS, quaternion)
    }


def replace_launch_arguments(
    arguments: list[str], overrides: Dict[str, str]
) -> list[str]:
    """Replace named ``name:=value`` arguments without duplicating them."""
    names = set(overrides)
    result = [
        argument for argument in arguments
        if argument.partition(":=")[0] not in names
    ]
    result.extend(f"{name}:={value}" for name, value in overrides.items())
    return result


def launch_argument_value(
    arguments: list[str], name: str, default: str
) -> str:
    """Read the last value of a ROS launch argument from a sequence."""
    prefix = f"{name}:="
    for argument in reversed(arguments):
        if argument.startswith(prefix):
            return argument[len(prefix):]
    return default


def load_camera_calibration(path: Path, robot_namespace: str) -> Dict[str, Any]:
    """Load and validate one robot-specific persisted mount calibration."""
    if not path.is_file():
        return {}
    try:
        calibration = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot read camera calibration {path}: {exc}") from exc
    if not isinstance(calibration, dict):
        raise ValueError(f"camera calibration {path} must contain a JSON object")
    saved_namespace = str(calibration.get("robot_namespace", ""))
    if saved_namespace and saved_namespace != robot_namespace:
        raise ValueError(
            f"camera calibration belongs to {saved_namespace}, not {robot_namespace}"
        )
    camera_calibration_overrides(
        calibration, float(calibration.get("camera_yaw_degrees", -90.0))
    )
    return calibration


def normalize_robot_namespace(value: str) -> str:
    """Return one safe ROS namespace token for the selected robot."""
    namespace = str(value).strip().strip("/")
    if not ROBOT_NAMESPACE_PATTERN.fullmatch(namespace):
        raise ValueError(
            "robot_namespace must be one ROS name token using only letters, "
            "digits and underscores"
        )
    return namespace


def robot_resource_name(robot_namespace: str, relative_name: str) -> str:
    """Build an absolute topic or service name below a robot namespace."""
    namespace = normalize_robot_namespace(robot_namespace)
    return f"/{namespace}/{relative_name.strip('/')}"


def classify_d1_posture(fsm_state: str) -> str:
    """Classify the D1's reported locomotion FSM without hiding its raw value."""
    normalized = fsm_state.strip().lower()
    if not normalized:
        return "unknown"
    if normalized in {"idle", "transform_down"}:
        return "prone"
    if normalized == "transform_up":
        return "standing_up"
    if (
        normalized in {"loco", "car", "joint_pd"}
        or normalized.startswith("rl_")
    ):
        return "standing"
    return "unknown"


def d1_feedback_request_timed_out(
    future: Any,
    requested_at: Optional[float],
    now: float,
    timeout: float,
) -> bool:
    """
    Return whether an unanswered D1 service request must be retried.

    A ROS service future can remain pending indefinitely when the remote DDS
    participant is restarted. Treating that future as permanently in flight
    prevents every later controller/SDK status poll and leaves web control
    stuck offline even after the robot services recover.
    """
    return bool(
        future is not None
        and not future.done()
        and requested_at is not None
        and now - requested_at >= timeout
    )


def parse_body_height(
    payload: Dict[str, Any], minimum: float, maximum: float
) -> float:
    """Validate a browser body-height request without silently changing it."""
    value = payload.get("height")
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("height must be a number")
    height = float(value)
    if not math.isfinite(height):
        raise ValueError("height must be finite")
    if height < minimum or height > maximum:
        raise ValueError(
            f"height level must be between {minimum:.0f} and {maximum:.0f}"
        )
    return height


def normalize_battery_percentage(value: float) -> Optional[float]:
    """Accept both ROS's 0..1 convention and the D1 driver's 0..100 value."""
    if not math.isfinite(value) or value < 0.0:
        return None
    percentage = value * 100.0 if value <= 1.0 else value
    return round(min(100.0, percentage), 1)


def parse_control_client_id(payload: Dict[str, Any]) -> str:
    """Return an optional bounded browser ID used to arbitrate control."""
    value = payload.get("client_id", "")
    if value is None or value == "":
        return ""
    if not isinstance(value, str) or not CONTROL_CLIENT_IDENTIFIER.fullmatch(value):
        raise ValueError("client_id must be 8-128 safe ASCII characters")
    return value


def validate_d1_http_bind_address(bind_address: str) -> None:
    """Limit the D1 web controller to its wired and operator LANs."""
    try:
        address = ipaddress.ip_address(bind_address)
    except ValueError as exc:
        raise ValueError("D1 bind_address must be a concrete IPv4 address") from exc
    allowed_networks = (
        ipaddress.ip_network("192.168.123.0/24"),
        ipaddress.ip_network("192.168.137.0/24"),
    )
    allowed = address.version == 4 and (
        address.is_loopback
        or any(address in network for network in allowed_networks)
    )
    if not allowed:
        raise ValueError(
            "D1 web control may bind only to 127.0.0.1, 192.168.123.x "
            "or 192.168.137.x"
        )


def resolve_d1_http_bind_addresses(
    bind_address: str,
    lan_addresses: Optional[list] = None,
) -> list:
    """Resolve a wildcard to concrete D1 wired/operator LAN addresses."""
    if bind_address != "0.0.0.0":
        validate_d1_http_bind_address(bind_address)
        return [bind_address]

    addresses = (
        discover_lan_ipv4_addresses()
        if lan_addresses is None
        else lan_addresses
    )
    allowed_networks = (
        ipaddress.ip_network("192.168.123.0/24"),
        ipaddress.ip_network("192.168.137.0/24"),
    )
    candidates = []
    for value in addresses:
        try:
            parsed = ipaddress.ip_address(value)
        except ValueError:
            continue
        if parsed.version == 4 and any(
            parsed in network for network in allowed_networks
        ):
            candidates.append(str(parsed))
    if not candidates:
        raise ValueError(
            "bind_address 0.0.0.0 cannot be used because this host has no "
            "192.168.123.x or 192.168.137.x address; connect an allowed LAN "
            "first"
        )
    return sorted(
        set(candidates),
        key=lambda value: ipaddress.ip_address(value).packed,
    )


def resolve_d1_http_bind_address(
    bind_address: str,
    lan_addresses: Optional[list] = None,
) -> str:
    """Return the first allowed address for callers needing one endpoint."""
    return resolve_d1_http_bind_addresses(bind_address, lan_addresses)[0]


def mapping_graph_conflicts(
    nodes: list[Tuple[str, str]],
) -> list[str]:
    """Return active SLAM nodes that would collide with a new mapping launch."""
    paths = {
        f"/{name}" if namespace == "/" else f"{namespace.rstrip('/')}/{name}"
        for name, namespace in nodes
    }
    return sorted(paths.intersection(MAPPING_NODE_PATHS))


def mapping_topic_conflicts(publisher_counts: Dict[str, int]) -> list[str]:
    """Return sensor topics already owned before the managed launch starts."""
    return [
        f"{topic}={count}个发布者"
        for topic, count in publisher_counts.items()
        if count > 0
    ]


def sanitized_subprocess_environment(
    prepend_library_paths: Tuple[str, ...] = (),
) -> Dict[str, str]:
    """Exclude camera-SDK libraries from non-camera child processes."""
    environment = os.environ.copy()
    mvs_root = Path(environment.get("MVS_ROOT", "/opt/MVS")).resolve()
    blocked_paths = {
        (mvs_root / "lib/aarch64").resolve(),
        (mvs_root / "lib/64").resolve(),
    }
    inherited_paths = [
        entry
        for entry in environment.get("LD_LIBRARY_PATH", "").split(os.pathsep)
        if entry and Path(entry).resolve() not in blocked_paths
    ]
    environment["LD_LIBRARY_PATH"] = os.pathsep.join([
        *(entry for entry in prepend_library_paths if entry),
        *inherited_paths,
    ])
    return environment


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
        matched = re.match(
            r"^(map\d+)_(?:octomap|filtered(?:_.+)?)$", parent.name
        )
        if matched:
            return matched.group(1)
    return ""


def _is_filtered_export_path(candidate: Path, octo_directory: Path) -> bool:
    """Return whether a saved layer belongs to a filtered map export."""
    if candidate.stem.endswith("_filtered_cloud"):
        return True
    for parent in candidate.parents:
        if parent == octo_directory.parent:
            break
        if re.match(r"^map\d+_filtered(?:_.+)?$", parent.name):
            return True
    return False


def discover_navigation_maps(maps_root: Path) -> list:
    """Return saved map layers and optional HLoc indices grouped by mapNNN."""
    rtab_directory = maps_root / "rtab_maps"
    octo_directory = maps_root / "octo_maps"
    hloc_directory = maps_root / "hloc_maps"
    databases = {
        candidate.stem: candidate.resolve()
        for candidate in rtab_directory.glob("map*.db")
        if MAP_IDENTIFIER.fullmatch(candidate.stem)
    }
    octomaps: Dict[str, Path] = {}
    filtered_octomaps: Dict[str, Path] = {}
    for candidate in octo_directory.rglob("*.bt"):
        map_id = _map_id_from_export_path(candidate, octo_directory)
        if map_id:
            collection = (
                filtered_octomaps
                if _is_filtered_export_path(candidate, octo_directory)
                else octomaps
            )
            previous = collection.get(map_id)
            if previous is None or candidate.stat().st_mtime > previous.stat().st_mtime:
                collection[map_id] = candidate.resolve()
    clouds: Dict[str, Path] = {}
    filtered_clouds: Dict[str, Path] = {}
    for candidate in octo_directory.rglob("*_cloud.ply"):
        map_id = _map_id_from_export_path(candidate, octo_directory)
        if map_id:
            collection = (
                filtered_clouds
                if _is_filtered_export_path(candidate, octo_directory)
                else clouds
            )
            previous = collection.get(map_id)
            if previous is None or candidate.stat().st_mtime > previous.stat().st_mtime:
                collection[map_id] = candidate.resolve()
    maps = []
    map_ids = set(databases) | set(octomaps) | set(filtered_octomaps)
    for map_id in sorted(map_ids, key=lambda value: int(value[3:])):
        database = databases.get(map_id)
        octomap = octomaps.get(map_id)
        cloud = clouds.get(map_id)
        filtered_octomap = filtered_octomaps.get(map_id)
        filtered_cloud = filtered_clouds.get(map_id)
        hloc_map = (hloc_directory / map_id).resolve()
        hloc_ready = (hloc_map / "metadata.yaml").is_file()
        maps.append({
            "id": map_id,
            "database_path": str(database) if database else None,
            "octomap_path": str(octomap) if octomap else None,
            "cloud_path": str(cloud) if cloud else None,
            "filtered_octomap_path": (
                str(filtered_octomap) if filtered_octomap else None
            ),
            "filtered_cloud_path": str(filtered_cloud) if filtered_cloud else None,
            "hloc_map_directory": str(hloc_map) if hloc_ready else None,
            "convertible": database is not None,
            "loadable": database is not None and octomap is not None,
            "localizable": (
                database is not None
                and octomap is not None
                and cloud is not None
                and hloc_ready
            ),
            "filtered_loadable": (
                database is not None
                and filtered_octomap is not None
                and filtered_cloud is not None
            ),
            "filtered_localizable": (
                database is not None
                and filtered_octomap is not None
                and filtered_cloud is not None
                and hloc_ready
            ),
        })
    return maps


def rtabmap_database_conversion_error(database: Path) -> str:
    """Explain why an RTAB-Map database cannot produce an optimized map."""
    try:
        connection = sqlite3.connect(
            f"file:{database.resolve()}?mode=ro", uri=True, timeout=1.0
        )
        try:
            tables = {
                row[0] for row in connection.execute(
                    "SELECT name FROM sqlite_master WHERE type='table'"
                )
            }
            if not {"Node", "Link"}.issubset(tables):
                return ""
            node_count = connection.execute(
                "SELECT COUNT(*) FROM Node"
            ).fetchone()[0]
            odometry_links = connection.execute(
                "SELECT COUNT(*) FROM Link "
                "WHERE type=0 AND from_id != to_id"
            ).fetchone()[0]
        finally:
            connection.close()
    except sqlite3.Error:
        # Leave non-SQLite or older databases to RTAB-Map's own validator.
        return ""
    if node_count == 0:
        return "database contains no mapped nodes"
    if odometry_links == 0:
        return (
            f"database has {node_count} nodes but no odometry links; "
            "camera exposure or visual odometry failed during capture, "
            "so this map must be recorded again"
        )
    return ""


def extract_colored_ply_points(path: Path, max_points: int) -> list:
    """Read XYZRGB vertices from PLY, or sample them when max_points is positive."""
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
        stride = (
            1 if max_points <= 0
            else max(1, (vertex_count + max_points - 1) // max_points)
        )
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


def parse_terrain_point_output(output: str) -> Tuple[float, list, list]:
    """Parse traversable costs and segmented obstacles from the C++ tool."""
    lines = output.splitlines()
    if not lines:
        raise ValueError("terrain converter returned no data")
    header = lines[0].split()
    if len(header) != 2 or header[0] != "resolution":
        raise ValueError("invalid terrain converter header")
    try:
        resolution = float(header[1])
    except ValueError as exc:
        raise ValueError("invalid terrain resolution") from exc
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise ValueError("invalid terrain resolution")
    traversable = []
    obstacles = []
    for line in lines[1:]:
        values = line.split()
        if len(values) != 5 or values[0] not in {"traversable", "obstacle"}:
            raise ValueError("invalid terrain point row")
        try:
            x, y, z, cost = (float(value) for value in values[1:])
        except ValueError as exc:
            raise ValueError("invalid terrain point row") from exc
        if not all(math.isfinite(value) for value in (x, y, z, cost)):
            continue
        if values[0] == "traversable":
            traversable.append((
                round(x, 3), round(y, 3), round(z, 3),
                round(max(0.0, min(1.0, cost)), 4),
            ))
        else:
            obstacles.append((round(x, 3), round(y, 3), round(z, 3)))
    return round(resolution, 4), traversable, obstacles


def localization_covariance_ready(covariance: list, maximum: float) -> bool:
    """Return whether the coarse localizer supplied a finite, confident pose."""
    if len(covariance) < 36 or not math.isfinite(maximum) or maximum <= 0.0:
        return False
    values = (covariance[0], covariance[7], covariance[35])
    return all(math.isfinite(value) and 0.0 <= value <= maximum
               for value in values)


def localization_pose_summary(message: PoseWithCovarianceStamped) -> Optional[dict]:
    """Return a finite map pose in the compact form consumed by the browser."""
    pose = message.pose.pose
    values = (
        pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w,
    )
    if not all(math.isfinite(value) for value in values):
        return None
    quaternion_norm = math.sqrt(sum(value * value for value in values[3:]))
    if quaternion_norm < 1.0e-6:
        return None
    x, y, z, qx, qy, qz, qw = values
    qx, qy, qz, qw = (
        value / quaternion_norm for value in (qx, qy, qz, qw)
    )
    yaw = math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )
    return {
        "x": round(x, 4),
        "y": round(y, 4),
        "z": round(z, 4),
        "yaw": round(yaw, 5),
        "yaw_degrees": round(math.degrees(yaw), 2),
    }


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
    timeout: float = 30.0,
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
    """Extract finite XYZRGB samples; non-positive max_points disables sampling."""
    field_by_name = {field.name: field for field in message.fields}
    required_fields = ("x", "y", "z")
    if any(name not in field_by_name for name in required_fields):
        return []
    if message.point_step <= 0 or message.width <= 0 or message.height <= 0:
        return []

    total_points = message.width * message.height
    stride = (
        1
        if max_points <= 0
        else max(1, (total_points + max_points - 1) // max_points)
    )
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


class HlocIndexBuilder:
    """Build one GPU HLoc reference index from a saved RTAB database."""

    def __init__(
        self,
        enabled: bool,
        exporter: Path,
        model_builder: Path,
        timeout: float,
    ) -> None:
        """Store the two installed tools and construction policy."""
        self.enabled = enabled
        self.exporter = exporter
        self.model_builder = model_builder
        self.timeout = timeout

    def build(
        self,
        map_id: str,
        database: Path,
        output_directory: Path,
    ) -> Tuple[bool, str]:
        """Create reference frames and neural features unless already ready."""
        metadata = output_directory / "metadata.yaml"
        if metadata.is_file():
            return True, f"map {map_id} HLoc index already exists"
        if not self.enabled:
            return False, "automatic HLoc index construction is disabled"
        if not MAP_IDENTIFIER.fullmatch(map_id):
            return False, "map_id must use the mapNNN format"
        if not database.is_file():
            return False, f"HLoc source database is missing: {database}"
        for label, executable in (
            ("HLoc RTAB exporter", self.exporter),
            ("HLoc model builder", self.model_builder),
        ):
            if not executable.is_file():
                return False, f"{label} is missing: {executable}"

        output_directory.mkdir(parents=True, exist_ok=True)
        environment = sanitized_subprocess_environment()
        environment["LUXI_HLOC_RUNTIME"] = "gpu"
        environment["PYTHONUNBUFFERED"] = "1"
        commands = (
            [
                str(self.exporter),
                "--database", str(database),
                "--output", str(output_directory),
            ],
            [
                str(self.model_builder),
                "--map-directory", str(output_directory),
                "--source-database", str(database),
                "--resize-max", "640",
                "--max-keypoints", "1024",
                "--overwrite",
            ],
        )
        try:
            outputs = []
            for command in commands:
                result = subprocess.run(
                    command,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    check=True,
                    timeout=self.timeout,
                    env=environment,
                )
                outputs.append(result.stdout or "")
        except subprocess.TimeoutExpired:
            return False, (
                f"map {map_id} HLoc construction exceeded "
                f"{self.timeout:.0f} seconds"
            )
        except (OSError, subprocess.SubprocessError) as exc:
            detail = getattr(exc, "stdout", "") or str(exc)
            return False, (
                f"map {map_id} HLoc construction failed: "
                f"{detail.strip()[-800:]}"
            )
        if not metadata.is_file():
            detail = "\n".join(outputs).strip()
            return False, (
                f"map {map_id} HLoc builder produced no metadata.yaml"
                + (f": {detail[-800:]}" if detail else "")
            )
        return True, f"map {map_id} HLoc index built with CUDA"


class SemanticAnnotationStore:
    """Validate and persist offline map annotations through the C++ tool."""

    def __init__(
        self,
        executable: Path,
        output_root: Path,
        octomap_library_path: Path,
        timeout: float,
    ) -> None:
        self.executable = executable
        self.output_root = output_root
        self.octomap_library_path = octomap_library_path
        self.timeout = timeout
        self._lock = threading.Lock()

    def _run(self, arguments: list, input_text: Optional[str] = None) -> dict:
        if not self.executable.is_file():
            raise RuntimeError(
                f"semantic annotation tool is missing: {self.executable}"
            )
        environment = sanitized_subprocess_environment(
            (str(self.octomap_library_path),)
        )
        try:
            result = subprocess.run(
                [str(self.executable), *arguments],
                input=input_text,
                capture_output=True,
                text=True,
                check=True,
                timeout=self.timeout,
                env=environment,
            )
            output = json.loads(result.stdout)
        except subprocess.TimeoutExpired as exc:
            raise RuntimeError("semantic annotation operation timed out") from exc
        except (OSError, subprocess.SubprocessError, json.JSONDecodeError) as exc:
            detail = getattr(exc, "stderr", "") or str(exc)
            raise RuntimeError(detail.strip()[-800:]) from exc
        if not isinstance(output, dict):
            raise RuntimeError("semantic annotation tool returned invalid JSON")
        return output

    def load(self, map_id: str, octomap_path: Path) -> dict:
        """Return validated annotations and an automatically estimated ground."""
        with self._lock:
            summary = self._run(["inspect", str(octomap_path)])
            output = self.output_root / map_id / "annotations.json"
            if output.is_file():
                annotation = self._run(
                    ["validate", str(octomap_path), map_id, str(output)]
                )
                saved = True
            else:
                annotation = {
                    "schema_version": 1,
                    "map_id": map_id,
                    "frame_id": "map",
                    "ground": {
                        "z": summary["suggested_ground_z"],
                        "minimum_height": 0.15,
                    },
                    "occupied_labels": [],
                    "pits": [],
                }
                saved = False
            return {
                "annotation": annotation,
                "summary": summary,
                "saved": saved,
                "path": str(output),
            }

    def save(self, map_id: str, octomap_path: Path, annotation: dict) -> dict:
        """Validate against occupied voxels and atomically save canonical JSON."""
        output = self.output_root / map_id / "annotations.json"
        with self._lock:
            canonical = self._run(
                [
                    "save",
                    str(octomap_path),
                    map_id,
                    "-",
                    str(output),
                ],
                json.dumps(annotation, ensure_ascii=False),
            )
        return {
            "annotation": canonical,
            "saved": True,
            "path": str(output),
        }


class D1ControlManager:
    """Run the validated D1 stand-up and lie-down procedures asynchronously."""

    def __init__(
        self,
        enabled: bool,
        start_script: Path,
        stop_script: Path,
        bridge_pid_file: Path,
        log_path: Path,
        transition_timeout: float = 90.0,
        robot_namespace: str = "d15041873",
        workspace_root: Optional[Path] = None,
    ) -> None:
        self.enabled = enabled
        self.start_script = start_script
        self.stop_script = stop_script
        self.bridge_pid_file = bridge_pid_file
        self.log_path = log_path
        self.transition_timeout = transition_timeout
        self.robot_namespace = normalize_robot_namespace(robot_namespace)
        self.workspace_root = workspace_root
        self._lock = threading.Lock()
        self._worker: Optional[threading.Thread] = None
        self._state = "active" if self._bridge_running() else "inactive"
        self._last_error = ""

    def _bridge_running(self) -> bool:
        try:
            pid = int(self.bridge_pid_file.read_text(encoding="ascii").strip())
            os.kill(pid, 0)
            return True
        except (OSError, ValueError):
            return False

    def set_active(self, active: bool, force: bool = False) -> Tuple[bool, str]:
        """Start a background transition unless already in the requested state."""
        with self._lock:
            if not self.enabled:
                return False, "D1 control is disabled"
            if self._worker is not None and self._worker.is_alive():
                return False, "D1 control is already transitioning"
            currently_active = self._bridge_running()
            if active == currently_active and not force:
                self._state = "active" if active else "inactive"
                self._last_error = ""
                return True, "D1 control is already in the requested state"
            self._state = "enabling" if active else "disabling"
            self._last_error = ""
            self._worker = threading.Thread(
                target=self._run_transition,
                args=(active, force),
                daemon=True,
            )
            self._worker.start()
        return True, "D1 control transition started"

    def _run_transition(self, active: bool, force: bool = False) -> None:
        script = self.start_script if active else self.stop_script
        try:
            if not script.is_file():
                raise FileNotFoundError(f"D1 control script is missing: {script}")
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            with self.log_path.open("a", encoding="utf-8") as log_file:
                log_file.write(
                    "\n===== D1 control "
                    + ("enable" if active else "disable")
                    + " requested by luxi_web_control =====\n"
                )
                environment = sanitized_subprocess_environment()
                environment["ROBOT_NS"] = self.robot_namespace
                if active and force:
                    # A forced web shutdown can leave the local bridge alive
                    # while the remote SDK is disabled and the robot is prone.
                    # Tell the start script to recycle that bridge instead of
                    # treating its PID as proof that control is ready.
                    environment["SLAM_D1_RECOVER_EXISTING"] = "true"
                if self.workspace_root is not None:
                    environment["SLAM_D1_WORKSPACE"] = str(self.workspace_root)
                result = subprocess.run(
                    [str(script), "--yes"],
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                    timeout=self.transition_timeout,
                    check=False,
                    env=environment,
                )
            if result.returncode != 0:
                raise RuntimeError(
                    f"D1 control script exited with code {result.returncode}"
                )
            if self._bridge_running() != active:
                raise RuntimeError("D1 bridge state does not match the request")
            state = "active" if active else "inactive"
            error = ""
        except (OSError, subprocess.SubprocessError, RuntimeError) as exc:
            state = "failed"
            error = str(exc)
        with self._lock:
            self._state = state
            self._last_error = error

    def status(self) -> Dict[str, Any]:
        """Return the transition state and independently observed bridge state."""
        with self._lock:
            state = self._state
            error = self._last_error
            transitioning = self._worker is not None and self._worker.is_alive()
        bridge_running = self._bridge_running()
        if not self.enabled:
            state = "disabled"
        elif not transitioning and state not in ("failed",):
            state = "active" if bridge_running else "inactive"
        return {
            "enabled": self.enabled,
            "robot_namespace": self.robot_namespace,
            "state": state,
            "active": bridge_running,
            "transitioning": transitioning,
            "last_error": error,
            "log_path": str(self.log_path),
        }


class MappingController:
    """Own the stable, maximum-performance and fallback USB map launches."""

    MODES = {
        "crestereo": (
            "usb_crestereo_rtabmap.launch.py",
            "crestereo_cuda_graph+luxi_direct_odom",
        ),
        "crestereo_max": (
            "usb_crestereo_max_performance_rtabmap.launch.py",
            "crestereo_10hz+superpoint_trt+lightglue_graph",
        ),
        "vpi": ("usb_rtabmap.launch.py", "vpi_ofa_pva_vic+luxi"),
    }
    MODE_ERROR = "mapping mode must be 'crestereo', 'crestereo_max' or 'vpi'"

    def __init__(
        self,
        enabled: bool,
        package: str,
        launch_file: str,
        rmw_implementation: str,
        sensor_setup: Optional[Path],
        workspace_setup: Path,
        log_path: Path,
        launch_arguments: Optional[Tuple[str, ...]] = None,
        crestereo_use_imu: Optional[bool] = None,
        planar_motion: bool = True,
    ) -> None:
        self.enabled = enabled
        self.package = package
        self.launch_file = launch_file
        self.rmw_implementation = rmw_implementation
        self.sensor_setup = sensor_setup
        self.workspace_setup = workspace_setup
        self.log_path = log_path
        self.launch_arguments = (
            launch_arguments if launch_arguments is not None else (
                "new_map:=true",
                "rviz:=false",
                "rtabmap_viz:=false",
            )
        )
        self.crestereo_use_imu = crestereo_use_imu
        self.planar_motion = planar_motion
        self._lock = threading.Lock()
        self._process: Optional[subprocess.Popen] = None
        self._started_at: Optional[float] = None
        self._last_exit_code: Optional[int] = None
        self._last_error = ""
        self._stop_requested = False
        self._mode = "crestereo"

    def set_camera_calibration(self, overrides: Dict[str, str]) -> None:
        """Apply persisted mount rotation to all subsequent map launches."""
        with self._lock:
            if self._process is not None and self._process.poll() is None:
                raise RuntimeError("cannot change camera calibration while mapping")
            self.launch_arguments = tuple(replace_launch_arguments(
                list(self.launch_arguments), overrides
            ))

    def _command(self, mode: Optional[str] = None) -> list:
        """Build a shell-free, source-aware RTAB-Map launch command."""
        selected_mode = mode or self._mode
        if selected_mode not in self.MODES:
            raise ValueError(self.MODE_ERROR)
        launch_file, _ = self.MODES[selected_mode]
        if self.package != "lunar_usb_rtabmap_bringup":
            launch_file = self.launch_file
        source_commands = [
            f"source {shlex.quote(str(self.workspace_setup))}",
            "export ROS_LOCALHOST_ONLY=0",
        ]
        if self.sensor_setup is not None:
            source_commands.insert(
                1, f"source {shlex.quote(str(self.sensor_setup))}"
            )
        if self.rmw_implementation:
            source_commands.append(
                "export RMW_IMPLEMENTATION="
                + shlex.quote(self.rmw_implementation)
            )
        launch_arguments = list(self.launch_arguments)
        if self.package != "lunar_usb_rtabmap_bringup":
            launch_arguments = [
                argument for argument in launch_arguments
                if not argument.startswith("planar_motion:=")
            ]
            launch_arguments.append(
                f"planar_motion:={'true' if self.planar_motion else 'false'}"
            )
        if (
            selected_mode.startswith("crestereo")
            and self.crestereo_use_imu is not None
        ):
            launch_arguments = [
                argument for argument in launch_arguments
                if not argument.startswith("use_imu:=")
            ]
            launch_arguments.append(
                f"use_imu:={'true' if self.crestereo_use_imu else 'false'}"
            )
        launch_command = shlex.join([
            "ros2",
            "launch",
            self.package,
            launch_file,
            *launch_arguments,
        ])
        script = "set -e; " + "; ".join(source_commands)
        script += f"; exec {launch_command}"
        return ["/bin/bash", "-c", script]

    def start(self, mode: str = "crestereo") -> Tuple[bool, str]:
        """Start a fresh managed RTAB-Map process when prerequisites exist."""
        with self._lock:
            if mode not in self.MODES:
                return False, self.MODE_ERROR
            if not self.enabled:
                return False, "mapping control is disabled"
            if self._process is not None and self._process.poll() is None:
                return False, "RTAB-Map mapping is already running"
            setup_files = [self.workspace_setup]
            if self.sensor_setup is not None:
                setup_files.insert(0, self.sensor_setup)
            missing = [path for path in setup_files if not path.is_file()]
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
                        self._command(mode),
                        stdout=log_file,
                        stderr=subprocess.STDOUT,
                        start_new_session=True,
                        env=sanitized_subprocess_environment(),
                    )
            except OSError as exc:
                self._process = None
                self._last_error = str(exc)
                return False, f"unable to start RTAB-Map: {exc}"
            self._started_at = time.monotonic()
            self._last_exit_code = None
            self._last_error = ""
            self._stop_requested = False
            self._mode = mode
            labels = {
                "crestereo": "CREStereo",
                "crestereo_max": "CREStereo MAX",
                "vpi": "VPI",
            }
            label = labels[mode]
            return True, f"RTAB-Map {label}/Luxi launch process started"

    def stop(self) -> Tuple[bool, str]:
        """Gracefully stop only this controller's RTAB-Map process."""
        with self._lock:
            process = self._process
            if process is None or process.poll() is not None:
                self._update_exit_state_locked()
                return True, "RTAB-Map mapping is already stopped"
            self._stop_requested = True
            try:
                # Signal only the top-level ROS launch process. It will stop
                # each child once and in dependency order. Signalling the
                # whole process group here makes launch forward a second
                # SIGINT to RTAB-Map, which can skip its graceful DB save.
                process.send_signal(signal.SIGINT)
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
        if not self._stop_requested:
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
        # Surface actionable hardware causes ahead of launch's generic
        # wrapper exception so the web page says why mapping was rejected.
        for marker in ("IMU NO DATA", "IMU HEALTH FAIL", "H30 NO DATA"):
            for line in reversed(lines):
                clean_line = line.strip()
                if marker in clean_line:
                    return clean_line
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
        elif error:
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
            "mode": self._mode,
            "modes": list(self.MODES),
            "default_mode": "crestereo",
            "frontend": self.MODES[self._mode][1],
            "planar_motion": self.planar_motion,
        }


class NavigationController:
    """Own selected-map localization and static OctoMap planning processes."""

    def __init__(
        self,
        enabled: bool,
        package: str,
        launch_file: str,
        rmw_implementation: str,
        sensor_setup: Optional[Path],
        workspace_setup: Path,
        octomap_library_path: Path,
        log_path: Path,
        launch_arguments: Tuple[str, ...] = (),
    ) -> None:
        self.enabled = enabled
        self.package = package
        self.launch_file = launch_file
        self.rmw_implementation = rmw_implementation
        self.sensor_setup = sensor_setup
        self.workspace_setup = workspace_setup
        self.octomap_library_path = octomap_library_path
        self.log_path = log_path
        self.launch_arguments = launch_arguments
        self._lock = threading.Lock()
        self._process: Optional[subprocess.Popen] = None
        self._started_at: Optional[float] = None
        self._last_exit_code: Optional[int] = None
        self._last_error = ""
        self._stop_requested = False
        self._map_id = ""

    def set_camera_calibration(self, overrides: Dict[str, str]) -> None:
        """Apply persisted mount rotation to subsequent localization runs."""
        with self._lock:
            if self._process is not None and self._process.poll() is None:
                raise RuntimeError("cannot change camera calibration during navigation")
            self.launch_arguments = tuple(replace_launch_arguments(
                list(self.launch_arguments), overrides
            ))

    def _command(
        self,
        database_path: Path,
        octomap_path: Path,
        cloud_path: Path,
        hloc_map_directory: Path,
        semantic_path: Path,
    ) -> list:
        # Source the algorithm workspace first, then the camera workspace as
        # the overlay.  Both workspaces may contain lunar_usb_rtabmap_bringup;
        # the device workspace owns the current USB launch files and must win.
        source_commands = [
            f"source {shlex.quote(str(self.workspace_setup))}",
        ]
        if self.sensor_setup is not None:
            source_commands.append(
                f"source {shlex.quote(str(self.sensor_setup))}"
            )
        source_commands.extend([
            "export ROS_LOCALHOST_ONLY=0",
            "export RMW_IMPLEMENTATION=" + shlex.quote(self.rmw_implementation),
            "export LD_LIBRARY_PATH=" + shlex.quote(str(self.octomap_library_path))
            + ':${LD_LIBRARY_PATH:-}',
        ])
        launch_command = shlex.join([
            "ros2", "launch", self.package, self.launch_file,
            f"database_path:={database_path}",
            f"octomap_path:={octomap_path}",
            f"cloud_path:={cloud_path}",
            f"hloc_map_directory:={hloc_map_directory}",
            f"semantic_path:={semantic_path}",
            "cmd_vel_topic:=/navigation/cmd_vel",
            *self.launch_arguments,
        ])
        script = "set -e; " + "; ".join(source_commands)
        return ["/bin/bash", "-c", script + f"; exec {launch_command}"]

    def start(
        self,
        map_id: str,
        database_path: Path,
        octomap_path: Path,
        cloud_path: Path,
        hloc_map_directory: Path,
        semantic_path: Path,
    ) -> Tuple[bool, str]:
        with self._lock:
            if not self.enabled:
                return False, "navigation control is disabled"
            if self._process is not None and self._process.poll() is None:
                return False, "selected-map navigation is already running"
            prerequisites = [
                self.workspace_setup,
                database_path,
                octomap_path,
                cloud_path,
                hloc_map_directory / "metadata.yaml",
            ]
            if self.sensor_setup is not None:
                prerequisites.insert(0, self.sensor_setup)
            missing = [
                path for path in prerequisites
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
                        self._command(
                            database_path,
                            octomap_path,
                            cloud_path,
                            hloc_map_directory,
                            semantic_path,
                        ),
                        stdout=log_file, stderr=subprocess.STDOUT,
                        start_new_session=True,
                        env=sanitized_subprocess_environment())
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
        parsed_url = urlsplit(self.path)
        path = parsed_url.path
        if path == "/api/status":
            self._send_json(HTTPStatus.OK, self.server.control_node.status())
            return
        if (
            path in CONTROL_DEFERRED_GET_PATHS
            and self.server.control_node.control_session_active()
        ):
            self._send_json(
                HTTPStatus.CONFLICT,
                {
                    "ok": False,
                    "paused": True,
                    "error": "preview paused while manual control is active",
                },
            )
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
            if not self.server.control_node.cloud_preview_enabled:
                self._send_error_json(
                    HTTPStatus.NOT_FOUND,
                    "live cloud preview is disabled",
                )
                return
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
        if path == "/api/navigation/terrain":
            self._send_json(
                HTTPStatus.OK,
                {"ok": True, "terrain": self.server.control_node.terrain_preview()},
            )
            return
        if path == "/api/semantic/annotations":
            map_id = parse_qs(parsed_url.query).get("map_id", [""])[0]
            try:
                result = self.server.control_node.semantic_annotations(map_id)
            except (RuntimeError, ValueError) as exc:
                self._send_error_json(HTTPStatus.BAD_REQUEST, str(exc))
                return
            self._send_json(HTTPStatus.OK, {"ok": True, **result})
            return

        assets = {
            "/": ("index.html", "text/html; charset=utf-8"),
            "/index.html": ("index.html", "text/html; charset=utf-8"),
            "/app.js": ("app.js", "text/javascript; charset=utf-8"),
            "/map_projection.js": (
                "map_projection.js",
                "text/javascript; charset=utf-8",
            ),
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
            cache_control="no-store",
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
                client_id = parse_control_client_id(payload)
                accepted, reason = node.accept_command(command, client_id)
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
            try:
                client_id = parse_control_client_id(payload)
            except ValueError as exc:
                self._send_error_json(HTTPStatus.BAD_REQUEST, str(exc))
                return
            force = payload.get("force", False)
            if not isinstance(force, bool):
                self._send_error_json(
                    HTTPStatus.BAD_REQUEST, "force must be boolean"
                )
                return
            stopped = node.stop_motion(
                client_id=client_id,
                force=force,
            )
            self._send_json(
                HTTPStatus.OK,
                {"ok": True, "stopped": stopped, "ignored": not stopped},
            )
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
        if path == "/api/robot/control":
            active = payload.get("active")
            if not isinstance(active, bool):
                self._send_error_json(
                    HTTPStatus.BAD_REQUEST,
                    "active must be boolean",
                )
                return
            node.stop_motion()
            accepted, message = node.set_d1_control(active)
            if not accepted:
                self._send_error_json(HTTPStatus.CONFLICT, message)
                return
            self._send_json(
                HTTPStatus.ACCEPTED,
                {
                    "ok": True,
                    "message": message,
                    "robot_control": node.d1_control_status(),
                },
            )
            return
        if path == "/api/robot/height":
            try:
                height = parse_body_height(
                    payload,
                    node.d1_body_height_minimum,
                    node.d1_body_height_maximum,
                )
            except ValueError as exc:
                self._send_error_json(HTTPStatus.BAD_REQUEST, str(exc))
                return
            node.stop_motion()
            accepted, message = node.set_d1_body_height(height)
            if not accepted:
                self._send_error_json(HTTPStatus.CONFLICT, message)
                return
            self._send_json(
                HTTPStatus.OK,
                {
                    "ok": True,
                    "message": message,
                    "robot_control": node.d1_control_status(),
                },
            )
            return
        if path == "/api/imu/calibrate":
            started, message = node.start_imu_level_calibration()
            if not started:
                self._send_error_json(HTTPStatus.CONFLICT, message)
                return
            self._send_json(
                HTTPStatus.ACCEPTED,
                {
                    "ok": True,
                    "message": message,
                    "imu_calibration": node.imu_calibration_status(),
                },
            )
            return
        if path == "/api/mapping/start":
            mode = payload.get("mode", "crestereo")
            if not isinstance(mode, str) or mode not in MappingController.MODES:
                self._send_error_json(
                    HTTPStatus.BAD_REQUEST,
                    MappingController.MODE_ERROR,
                )
                return
            started, message = node.start_mapping(mode)
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
            filtered = payload.get("filtered", False)
            if not isinstance(filtered, bool):
                self._send_error_json(
                    HTTPStatus.BAD_REQUEST, "filtered must be boolean"
                )
                return
            loaded, message = node.load_navigation_map(map_id, filtered)
            if not loaded:
                self._send_error_json(HTTPStatus.CONFLICT, message)
                return
            self._send_json(HTTPStatus.ACCEPTED, {
                "ok": True, "message": message,
                "maps": node.navigation_maps(),
                "navigation": node.navigation_status(),
                "map_variant": "filtered" if filtered else "original",
            })
            return
        if path == "/api/navigation/localize":
            map_id = payload.get("map_id")
            if not isinstance(map_id, str):
                self._send_error_json(HTTPStatus.BAD_REQUEST, "map_id must be a string")
                return
            started, message = node.start_navigation_localization(map_id)
            if not started:
                self._send_error_json(HTTPStatus.CONFLICT, message)
                return
            self._send_json(HTTPStatus.ACCEPTED, {
                "ok": True,
                "message": message,
                "maps": node.navigation_maps(),
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
        if path == "/api/navigation/start":
            started, message = node.start_navigation_motion()
            if not started:
                self._send_error_json(HTTPStatus.CONFLICT, message)
                return
            self._send_json(HTTPStatus.ACCEPTED, {
                "ok": True,
                "message": message,
                "navigation": node.navigation_status(),
            })
            return
        if path == "/api/navigation/halt":
            node.halt_navigation_motion()
            self._send_json(HTTPStatus.OK, {
                "ok": True,
                "message": "navigation motion stopped",
                "navigation": node.navigation_status(),
            })
            return
        if path == "/api/semantic/save":
            try:
                result = node.save_semantic_annotations(payload)
            except (RuntimeError, ValueError) as exc:
                self._send_error_json(HTTPStatus.BAD_REQUEST, str(exc))
                return
            self._send_json(HTTPStatus.OK, {"ok": True, **result})
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
        self.declare_parameter("restrict_http_to_d1_lan", False)
        self.declare_parameter("http_port", 8080)
        self.declare_parameter("auto_stop_existing_web_control", True)
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("command_timeout", 0.8)
        self.declare_parameter("max_linear_x", 0.25)
        self.declare_parameter("max_linear_y", 0.0)
        self.declare_parameter("max_angular_z", 0.8)
        self.declare_parameter("enable_output", True)
        self.declare_parameter("web_root", "")
        self.declare_parameter("robot_namespace", "d15041873")
        self.declare_parameter("enable_d1_control", False)
        self.declare_parameter("d1_start_script", "")
        self.declare_parameter("d1_stop_script", "")
        self.declare_parameter("d1_bridge_pid_file", "")
        self.declare_parameter("d1_control_log_path", "")
        self.declare_parameter("d1_transition_timeout", 90.0)
        # Empty values follow robot_namespace. Explicit absolute names remain
        # available for unusual vendor deployments.
        self.declare_parameter("d1_fsm_topic", "")
        self.declare_parameter("d1_controller_status_service", "")
        self.declare_parameter("d1_parameter_service", "")
        self.declare_parameter("d1_battery1_topic", "")
        self.declare_parameter("d1_battery2_topic", "")
        self.declare_parameter("d1_body_height_command_topic", "")
        self.declare_parameter("d1_body_height_status_topic", "")
        self.declare_parameter("d1_body_height_minimum", 0.0)
        self.declare_parameter("d1_body_height_maximum", 9.0)
        self.declare_parameter("d1_body_height_default", 0.0)
        self.declare_parameter("d1_feedback_timeout", 3.0)
        self.declare_parameter(
            "imu_level_calibration_service", "/sensors/imu/calibrate_level"
        )
        self.declare_parameter(
            "imu_level_status_topic", "/sensors/imu/level_calibration_status"
        )
        # USB/H30 calibration is started on demand because the mapping process
        # normally owns the H30 serial device.  Keeping it short-lived avoids
        # two yesense drivers competing for /dev/imu-H30.
        self.declare_parameter("imu_level_standalone_enabled", False)
        self.declare_parameter(
            "imu_level_calibration_launch_package",
            "lunar_usb_rtabmap_bringup",
        )
        self.declare_parameter(
            "imu_level_calibration_launch_file",
            "usb_imu_level_calibration.launch.py",
        )
        self.declare_parameter("imu_level_calibration_sensor_setup", "")
        self.declare_parameter("imu_level_calibration_workspace_setup", "")
        self.declare_parameter("imu_level_calibration_log_path", "")
        self.declare_parameter("imu_level_calibration_store_path", "")
        self.declare_parameter("imu_level_calibration_start_timeout", 12.0)
        self.declare_parameter("imu_level_calibration_result_timeout", 20.0)
        self.declare_parameter("imu_level_calibration_samples", 200)
        self.declare_parameter("imu_level_camera_yaw_degrees", -90.0)
        self.declare_parameter("enable_mapping_control", True)
        self.declare_parameter(
            "mapping_launch_package", "lunar_usb_rtabmap_bringup"
        )
        self.declare_parameter("mapping_launch_file", "usb_rtabmap.launch.py")
        self.declare_parameter("mapping_launch_arguments", [
            "new_map:=true",
            "rviz:=false",
            "rtabmap_viz:=false",
            "use_imu:=true",
            "planar_mode:=false",
        ])
        self.declare_parameter("mapping_robot_camera_profiles", [""])
        # H30 rotation is the default for both model and VPI mapping. The
        # launch health gate rejects silent or invalid IMU streams.
        self.declare_parameter("mapping_crestereo_use_imu", True)
        self.declare_parameter(
            "mapping_rmw_implementation",
            "rmw_fastrtps_cpp",
        )
        self.declare_parameter("mapping_sensor_setup", "")
        self.declare_parameter("mapping_d435_setup", "")
        self.declare_parameter("mapping_workspace_setup", "")
        self.declare_parameter("mapping_log_path", "")
        self.declare_parameter("mapping_planar_motion", True)
        self.declare_parameter("mapping_rgbd_topic", "/sensors/rgbd/rgbd_image")
        self.declare_parameter("mapping_imu_topic", "/sensors/imu/data")
        self.declare_parameter("enable_navigation_control", True)
        self.declare_parameter("navigation_launch_package", "luxi_3d_navigation")
        self.declare_parameter("navigation_launch_file", "saved_map_navigation.launch.py")
        self.declare_parameter("navigation_launch_arguments", [""])
        self.declare_parameter("navigation_rmw_implementation", "rmw_cyclonedds_cpp")
        self.declare_parameter("navigation_sensor_setup", "")
        self.declare_parameter("navigation_d435_setup", "")
        self.declare_parameter("navigation_workspace_setup", "")
        self.declare_parameter("navigation_log_path", "")
        self.declare_parameter("maps_root", "")
        self.declare_parameter("map_conversion_timeout", 300.0)
        self.declare_parameter("auto_build_hloc_index", True)
        self.declare_parameter("hloc_index_build_timeout", 900.0)
        self.declare_parameter("navigation_goal_topic", "/navigation/goal_pose")
        self.declare_parameter("navigation_marker_topic", "/navigation/occupied_voxels")
        self.declare_parameter("navigation_path_topic", "/navigation/planned_path")
        self.declare_parameter(
            "navigation_planning_status_topic", "/navigation/planning_status"
        )
        self.declare_parameter("navigation_start_topic", "/navigation/start")
        self.declare_parameter("navigation_stop_topic", "/navigation/stop")
        self.declare_parameter("navigation_active_topic", "/navigation/active")
        self.declare_parameter(
            "navigation_follower_state_topic",
            "/navigation/follower_state",
        )
        self.declare_parameter(
            "navigation_emergency_stop_topic",
            "/navigation/emergency_stop",
        )
        self.declare_parameter(
            "navigation_localization_pose_topic", "/luxi_hloc/coarse_pose"
        )
        self.declare_parameter(
            "navigation_refined_pose_topic", "/luxi_location/pose"
        )
        self.declare_parameter(
            "navigation_terrain_pose_topic", "/navigation/terrain_pose"
        )
        self.declare_parameter(
            "navigation_refined_fitness_topic", "/luxi_location/fitness"
        )
        self.declare_parameter(
            "navigation_refined_status_topic", "/luxi_location/status"
        )
        self.declare_parameter("navigation_hloc_status_topic", "/luxi_hloc/status")
        self.declare_parameter(
            "navigation_hloc_diagnostics_topic", "/luxi_hloc/diagnostics"
        )
        self.declare_parameter("navigation_localization_max_variance", 0.5)
        self.declare_parameter("navigation_localization_timeout", 3.0)
        self.declare_parameter("navigation_icp_verification_timeout", 20.0)
        self.declare_parameter("navigation_icp_minimum_fitness", 0.25)
        self.declare_parameter("max_voxel_points", 12000)
        self.declare_parameter("max_terrain_points", 12000)
        self.declare_parameter("navigation_robot_radius", 0.10)
        self.declare_parameter("navigation_costmap_margin", 0.60)
        self.declare_parameter("navigation_ground_normal_radius", 0.30)
        self.declare_parameter("navigation_ground_max_slope_degrees", 35.0)
        self.declare_parameter("navigation_obstacle_min_height", 0.15)
        self.declare_parameter("navigation_terrain_load_timeout", 90.0)
        self.declare_parameter("enable_preview", True)
        self.declare_parameter("enable_cloud_preview", False)
        self.declare_parameter(
            "rgb_preview_topic",
            "/sensors/rgbd/color/image_raw/compressed",
        )
        self.declare_parameter("cloud_preview_topic", "/rtabmap/cloud_map")
        self.declare_parameter("max_cloud_points", 5000)
        self.declare_parameter("max_saved_cloud_points", 30000)
        self.declare_parameter("semantic_annotation_timeout", 15.0)
        self.declare_parameter("semantic_annotation_executable", "")
        self.declare_parameter("semantic_maps_root", "")

        self.cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self.imu_level_calibration_service = str(
            self.get_parameter("imu_level_calibration_service").value
        )
        self.imu_level_status_topic = str(
            self.get_parameter("imu_level_status_topic").value
        )
        bind_address = str(self.get_parameter("bind_address").value)
        restrict_http_to_d1_lan = bool(
            self.get_parameter("restrict_http_to_d1_lan").value
        )
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
        self.robot_namespace = normalize_robot_namespace(
            str(self.get_parameter("robot_namespace").value)
        )
        robot_pid_token = self.robot_namespace.replace("/", "_")
        self.imu_level_standalone_enabled = bool(
            self.get_parameter("imu_level_standalone_enabled").value
        )
        self.imu_level_calibration_launch_package = str(
            self.get_parameter("imu_level_calibration_launch_package").value
        ).strip()
        self.imu_level_calibration_launch_file = str(
            self.get_parameter("imu_level_calibration_launch_file").value
        ).strip()

        def resolve_calibration_path(parameter_name: str, fallback: Path) -> Path:
            configured = str(self.get_parameter(parameter_name).value).strip()
            if configured:
                configured = configured.replace(
                    "{robot_namespace}", self.robot_namespace
                )
                path = Path(os.path.expandvars(configured)).expanduser()
                if not path.is_absolute():
                    path = workspace_root / path
                return path.resolve()
            return fallback.resolve()

        self.imu_level_calibration_sensor_setup = resolve_calibration_path(
            "imu_level_calibration_sensor_setup",
            workspace_root / "device/USBCameraSDK/ros2_ws/install/setup.bash",
        )
        self.imu_level_calibration_workspace_setup = resolve_calibration_path(
            "imu_level_calibration_workspace_setup",
            workspace_root / "install/setup.bash",
        )
        self.imu_level_calibration_log_path = resolve_calibration_path(
            "imu_level_calibration_log_path",
            workspace_root / "log/luxi_web_control_imu_calibration.log",
        )
        self.imu_level_calibration_store_path = resolve_calibration_path(
            "imu_level_calibration_store_path",
            workspace_root / "maps/calibration"
            / f"{self.robot_namespace}_camera_mount.json",
        )
        self.imu_level_calibration_start_timeout = float(
            self.get_parameter("imu_level_calibration_start_timeout").value
        )
        self.imu_level_calibration_result_timeout = float(
            self.get_parameter("imu_level_calibration_result_timeout").value
        )
        self.imu_level_calibration_samples = int(
            self.get_parameter("imu_level_calibration_samples").value
        )
        self.imu_level_camera_yaw_degrees = float(
            self.get_parameter("imu_level_camera_yaw_degrees").value
        )
        try:
            self._persisted_imu_calibration = load_camera_calibration(
                self.imu_level_calibration_store_path,
                self.robot_namespace,
            )
        except ValueError as exc:
            self.get_logger().error(str(exc))
            self._persisted_imu_calibration = {}
        d1_start_script = str(self.get_parameter("d1_start_script").value)
        d1_stop_script = str(self.get_parameter("d1_stop_script").value)
        d1_bridge_pid_file = str(
            self.get_parameter("d1_bridge_pid_file").value
        )
        d1_control_log_path = str(
            self.get_parameter("d1_control_log_path").value
        )
        self.d1_control = D1ControlManager(
            enabled=bool(self.get_parameter("enable_d1_control").value),
            start_script=Path(
                d1_start_script
                or workspace_root
                / "project/slam_d1_bridge/scripts/start_slam_d1_bridge.sh"
            ).resolve(),
            stop_script=Path(
                d1_stop_script
                or workspace_root
                / "project/slam_d1_bridge/scripts/stop_slam_d1_bridge.sh"
            ).resolve(),
            bridge_pid_file=Path(
                d1_bridge_pid_file
                or f"/tmp/slam_d1_bridge_{robot_pid_token}.pid"
            ).resolve(),
            log_path=Path(
                d1_control_log_path
                or workspace_root
                / f"log/luxi_web_control_d1_{robot_pid_token}.log"
            ).resolve(),
            transition_timeout=float(
                self.get_parameter("d1_transition_timeout").value
            ),
            robot_namespace=self.robot_namespace,
            workspace_root=workspace_root,
        )
        configured_fsm_topic = str(
            self.get_parameter("d1_fsm_topic").value
        ).strip()
        configured_status_service = str(
            self.get_parameter("d1_controller_status_service").value
        ).strip()
        configured_parameter_service = str(
            self.get_parameter("d1_parameter_service").value
        ).strip()
        self.d1_fsm_topic = configured_fsm_topic or robot_resource_name(
            self.robot_namespace, "rl_controller/fsm"
        )
        self.d1_controller_status_service = (
            configured_status_service
            or robot_resource_name(
                self.robot_namespace, "command/get_controller_status"
            )
        )
        self.d1_parameter_service = (
            configured_parameter_service
            or robot_resource_name(
                self.robot_namespace, "teleop_command/get_parameters"
            )
        )
        configured_battery_topics = tuple(
            str(self.get_parameter(name).value).strip()
            for name in ("d1_battery1_topic", "d1_battery2_topic")
        )
        self.d1_battery_topics = tuple(
            configured or robot_resource_name(
                self.robot_namespace, f"status/battery{index}"
            )
            for index, configured in enumerate(
                configured_battery_topics, start=1
            )
        )
        configured_height_command = str(
            self.get_parameter("d1_body_height_command_topic").value
        ).strip()
        configured_height_status = str(
            self.get_parameter("d1_body_height_status_topic").value
        ).strip()
        self.d1_body_height_command_topic = (
            configured_height_command
            or robot_resource_name(
                self.robot_namespace, "command/body_height"
            )
        )
        self.d1_body_height_status_topic = (
            configured_height_status
            or robot_resource_name(
                self.robot_namespace, "status/body_height"
            )
        )
        self.d1_body_height_minimum = float(
            self.get_parameter("d1_body_height_minimum").value
        )
        self.d1_body_height_maximum = float(
            self.get_parameter("d1_body_height_maximum").value
        )
        self.d1_body_height_default = float(
            self.get_parameter("d1_body_height_default").value
        )
        if not (
            math.isfinite(self.d1_body_height_minimum)
            and math.isfinite(self.d1_body_height_maximum)
            and self.d1_body_height_minimum < self.d1_body_height_maximum
            and self.d1_body_height_minimum <= self.d1_body_height_default
            <= self.d1_body_height_maximum
        ):
            raise ValueError("invalid D1 body-height limits")
        self.d1_feedback_timeout = float(
            self.get_parameter("d1_feedback_timeout").value
        )
        self._d1_status_lock = threading.Lock()
        self._d1_fsm_state = ""
        self._d1_fsm_received_at: Optional[float] = None
        self._d1_controller_mode = ""
        self._d1_controller_received_at: Optional[float] = None
        self._d1_sdk_active: Optional[bool] = None
        self._d1_sdk_received_at: Optional[float] = None
        self._d1_feedback_error = ""
        self._d1_batteries = [None, None]
        self._d1_battery_received_at = [None, None]
        self._d1_body_height = self.d1_body_height_default
        self._d1_body_height_target = self.d1_body_height_default
        self._d1_body_height_received_at: Optional[float] = None
        self._d1_controller_future = None
        self._d1_controller_requested_at: Optional[float] = None
        self._d1_parameter_future = None
        self._d1_parameter_requested_at: Optional[float] = None
        mapping_sensor_setup = str(
            self.get_parameter("mapping_sensor_setup").value
        )
        legacy_mapping_setup = str(
            self.get_parameter("mapping_d435_setup").value
        )
        mapping_sensor_setup = mapping_sensor_setup or legacy_mapping_setup
        if mapping_sensor_setup:
            expanded_setup = Path(
                os.path.expandvars(mapping_sensor_setup)
            ).expanduser()
            if not expanded_setup.is_absolute():
                expanded_setup = workspace_root / expanded_setup
            mapping_sensor_setup = str(expanded_setup.resolve())
        workspace_setup = str(
            self.get_parameter("mapping_workspace_setup").value
        )
        mapping_log_path = str(self.get_parameter("mapping_log_path").value)
        mapping_launch_arguments = [
            str(argument) for argument in
            self.get_parameter("mapping_launch_arguments").value
        ]
        selected_camera_profile: Optional[Path] = None
        for profile_entry in self.get_parameter(
            "mapping_robot_camera_profiles"
        ).value:
            profile_entry = str(profile_entry).strip()
            if not profile_entry:
                continue
            namespace, separator, configured_path = profile_entry.partition("=")
            if not separator or not configured_path.strip():
                raise ValueError(
                    "mapping_robot_camera_profiles entries must be NAMESPACE=PATH"
                )
            if normalize_robot_namespace(namespace) != self.robot_namespace:
                continue
            camera_profile = Path(
                os.path.expandvars(configured_path.strip())
            ).expanduser()
            if not camera_profile.is_absolute():
                camera_profile = workspace_root / camera_profile
            camera_profile = camera_profile.resolve()
            if not camera_profile.is_file():
                raise FileNotFoundError(
                    f"camera profile for {self.robot_namespace} is missing: "
                    f"{camera_profile}"
                )
            mapping_launch_arguments = [
                argument for argument in mapping_launch_arguments
                if not argument.startswith("camera_params:=")
            ]
            mapping_launch_arguments.append(f"camera_params:={camera_profile}")
            selected_camera_profile = camera_profile
            break
        self._imu_level_camera_translation = tuple(
            launch_argument_value(mapping_launch_arguments, name, default)
            for name, default in (
                ("camera_x", "0.20"),
                ("camera_y", "0.044982"),
                ("camera_z", "0.20"),
            )
        )
        if self._persisted_imu_calibration:
            mapping_launch_arguments = replace_launch_arguments(
                mapping_launch_arguments,
                camera_calibration_overrides(
                    self._persisted_imu_calibration,
                    self.imu_level_camera_yaw_degrees,
                ),
            )
        self.mapping = MappingController(
            enabled=bool(self.get_parameter("enable_mapping_control").value),
            package=str(self.get_parameter("mapping_launch_package").value),
            launch_file=str(self.get_parameter("mapping_launch_file").value),
            rmw_implementation=str(
                self.get_parameter("mapping_rmw_implementation").value
            ),
            sensor_setup=(
                Path(mapping_sensor_setup)
                if mapping_sensor_setup else None
            ),
            workspace_setup=Path(
                workspace_setup or workspace_root / "install/setup.bash"
            ).resolve(),
            log_path=Path(
                mapping_log_path
                or workspace_root / "log/luxi_web_control_rtabmap.log"
            ).resolve(),
            launch_arguments=tuple(mapping_launch_arguments),
            crestereo_use_imu=bool(
                self.get_parameter("mapping_crestereo_use_imu").value
            ),
            planar_motion=bool(
                self.get_parameter("mapping_planar_motion").value
            ),
        )
        self.mapping_rgbd_topic = str(
            self.get_parameter("mapping_rgbd_topic").value
        )
        self.mapping_imu_topic = str(
            self.get_parameter("mapping_imu_topic").value
        )
        navigation_sensor_setup = str(
            self.get_parameter("navigation_sensor_setup").value
        )
        legacy_navigation_setup = str(
            self.get_parameter("navigation_d435_setup").value
        )
        navigation_sensor_setup = (
            navigation_sensor_setup or legacy_navigation_setup
        )
        if navigation_sensor_setup:
            expanded_navigation_setup = Path(
                os.path.expandvars(navigation_sensor_setup)
            ).expanduser()
            if not expanded_navigation_setup.is_absolute():
                expanded_navigation_setup = (
                    workspace_root / expanded_navigation_setup
                )
            navigation_sensor_setup = str(
                expanded_navigation_setup.resolve()
            )
        navigation_workspace_setup = str(
            self.get_parameter("navigation_workspace_setup").value
        )
        navigation_log_path = str(self.get_parameter("navigation_log_path").value)
        navigation_launch_arguments = [
            str(argument).strip()
            for argument in self.get_parameter("navigation_launch_arguments").value
            if str(argument).strip()
        ]
        if selected_camera_profile is not None:
            navigation_launch_arguments = [
                argument for argument in navigation_launch_arguments
                if not argument.startswith("camera_params:=")
            ]
            navigation_launch_arguments.append(
                f"camera_params:={selected_camera_profile}"
            )
        if self._persisted_imu_calibration:
            navigation_launch_arguments = replace_launch_arguments(
                navigation_launch_arguments,
                camera_calibration_overrides(
                    self._persisted_imu_calibration,
                    self.imu_level_camera_yaw_degrees,
                ),
            )
        maps_root = str(self.get_parameter("maps_root").value)
        self.maps_root = Path(maps_root or workspace_root / "maps").resolve()
        self.map_export_executable = (
            workspace_root / "tools/export_rtabmap_octomap.sh"
        ).resolve()
        self.map_conversion_timeout = float(
            self.get_parameter("map_conversion_timeout").value
        )
        self.hloc_index_builder = HlocIndexBuilder(
            enabled=bool(self.get_parameter("auto_build_hloc_index").value),
            exporter=(
                workspace_root / "install/luxi_hloc/lib/luxi_hloc/"
                "rtab_hloc_exporter"
            ).resolve(),
            model_builder=(
                workspace_root / "install/luxi_hloc/lib/luxi_hloc/"
                "build_reference_model.py"
            ).resolve(),
            timeout=float(
                self.get_parameter("hloc_index_build_timeout").value
            ),
        )
        annotation_executable = str(
            self.get_parameter("semantic_annotation_executable").value
        )
        semantic_maps_root = str(
            self.get_parameter("semantic_maps_root").value
        )
        self.semantic_annotation_store = SemanticAnnotationStore(
            executable=Path(
                annotation_executable
                or workspace_root
                / "install/luxi_semantic_annotation/lib/"
                "luxi_semantic_annotation/semantic_annotation_tool"
            ).resolve(),
            output_root=Path(
                semantic_maps_root
                or workspace_root / "maps/semantic_maps"
            ).resolve(),
            octomap_library_path=(
                workspace_root / "3parts/octomap/install/lib"
            ).resolve(),
            timeout=float(
                self.get_parameter("semantic_annotation_timeout").value
            ),
        )
        self.octomap_points_executable = (
            workspace_root / "install/luxi_voxel_navigation/lib/"
            "luxi_voxel_navigation/octomap_to_points"
        ).resolve()
        self.terrain_points_executable = (
            workspace_root / "install/luxi_3d_navigation/lib/"
            "luxi_3d_navigation/terrain_map_to_points"
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
        self.navigation_planning_status_topic = str(
            self.get_parameter("navigation_planning_status_topic").value
        )
        self.navigation_start_topic = str(
            self.get_parameter("navigation_start_topic").value
        )
        self.navigation_stop_topic = str(
            self.get_parameter("navigation_stop_topic").value
        )
        self.navigation_active_topic = str(
            self.get_parameter("navigation_active_topic").value
        )
        self.navigation_follower_state_topic = str(
            self.get_parameter("navigation_follower_state_topic").value
        )
        self.navigation_emergency_stop_topic = str(
            self.get_parameter("navigation_emergency_stop_topic").value
        )
        self.navigation_localization_pose_topic = str(
            self.get_parameter("navigation_localization_pose_topic").value
        )
        self.navigation_refined_pose_topic = str(
            self.get_parameter("navigation_refined_pose_topic").value
        )
        self.navigation_terrain_pose_topic = str(
            self.get_parameter("navigation_terrain_pose_topic").value
        )
        self.navigation_refined_fitness_topic = str(
            self.get_parameter("navigation_refined_fitness_topic").value
        )
        self.navigation_refined_status_topic = str(
            self.get_parameter("navigation_refined_status_topic").value
        )
        self.navigation_hloc_status_topic = str(
            self.get_parameter("navigation_hloc_status_topic").value
        )
        self.navigation_hloc_diagnostics_topic = str(
            self.get_parameter("navigation_hloc_diagnostics_topic").value
        )
        self.navigation_localization_max_variance = float(
            self.get_parameter("navigation_localization_max_variance").value
        )
        self.navigation_localization_timeout = float(
            self.get_parameter("navigation_localization_timeout").value
        )
        self.navigation_icp_verification_timeout = float(
            self.get_parameter("navigation_icp_verification_timeout").value
        )
        self.navigation_icp_minimum_fitness = float(
            self.get_parameter("navigation_icp_minimum_fitness").value
        )
        self.max_voxel_points = int(self.get_parameter("max_voxel_points").value)
        self.max_terrain_points = int(
            self.get_parameter("max_terrain_points").value
        )
        self.navigation_robot_radius = float(
            self.get_parameter("navigation_robot_radius").value
        )
        self.navigation_costmap_margin = float(
            self.get_parameter("navigation_costmap_margin").value
        )
        self.navigation_ground_normal_radius = float(
            self.get_parameter("navigation_ground_normal_radius").value
        )
        self.navigation_ground_max_slope_degrees = float(
            self.get_parameter("navigation_ground_max_slope_degrees").value
        )
        self.navigation_obstacle_min_height = float(
            self.get_parameter("navigation_obstacle_min_height").value
        )
        self.navigation_terrain_load_timeout = float(
            self.get_parameter("navigation_terrain_load_timeout").value
        )
        self.navigation = NavigationController(
            enabled=bool(self.get_parameter("enable_navigation_control").value),
            package=str(self.get_parameter("navigation_launch_package").value),
            launch_file=str(self.get_parameter("navigation_launch_file").value),
            rmw_implementation=str(
                self.get_parameter("navigation_rmw_implementation").value
            ),
            sensor_setup=(
                Path(navigation_sensor_setup).resolve()
                if navigation_sensor_setup else None
            ),
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
            launch_arguments=tuple(navigation_launch_arguments),
        )
        self.preview_enabled = bool(self.get_parameter("enable_preview").value)
        self.cloud_preview_enabled = bool(
            self.get_parameter("enable_cloud_preview").value
        )
        self.rgb_preview_topic = str(
            self.get_parameter("rgb_preview_topic").value
        )
        self.cloud_preview_topic = str(
            self.get_parameter("cloud_preview_topic").value
        )
        self.max_cloud_points = int(
            self.get_parameter("max_cloud_points").value
        )
        self.max_saved_cloud_points = int(
            self.get_parameter("max_saved_cloud_points").value
        )

        self._validate_parameters(http_port, publish_rate)
        bind_addresses = [bind_address]
        if restrict_http_to_d1_lan:
            bind_addresses = resolve_d1_http_bind_addresses(bind_address)
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
        self._control_owner_id = ""
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
        self._navigation_map_operation_lock = threading.Lock()
        self._voxel_points = []
        self._voxel_frame_id = "map"
        self._voxel_resolution = 0.0
        self._voxel_received_at: Optional[float] = None
        self._navigation_voxel_map_id: Optional[str] = None
        self._navigation_voxel_variant: Optional[str] = None
        self._navigation_voxel_error = ""
        self._terrain_traversable_points = []
        self._terrain_obstacle_points = []
        self._terrain_resolution = 0.0
        self._terrain_map_id: Optional[str] = None
        self._terrain_variant: Optional[str] = None
        self._terrain_error = ""
        self._localization_ready = False
        self._localization_variance: Optional[float] = None
        self._localization_received_at: Optional[float] = None
        self._coarse_localization_pose: Optional[dict] = None
        self._coarse_gate_ready = False
        self._refined_localization_pose: Optional[dict] = None
        self._refined_localization_received_at: Optional[float] = None
        self._terrain_pose: Optional[dict] = None
        self._terrain_pose_received_at: Optional[float] = None
        self._refined_localization_fitness: Optional[float] = None
        self._refined_localization_status = ""
        self._refined_localization_verified_at: Optional[float] = None
        self._hloc_status = ""
        self._hloc_status_received_at: Optional[float] = None
        self._hloc_diagnostics: Optional[dict] = None
        self._hloc_diagnostics_received_at: Optional[float] = None
        self._navigation_cloud_points = []
        self._navigation_cloud_map_id: Optional[str] = None
        self._navigation_cloud_variant: Optional[str] = None
        self._navigation_cloud_error = ""
        self._planned_path_points = []
        self._last_valid_path_points = []
        self._last_valid_path_frame_id = "map"
        self._path_frame_id = "map"
        self._path_received_at: Optional[float] = None
        self._last_valid_path_received_at: Optional[float] = None
        self._planning_state = "idle"
        self._planning_error = ""
        self._navigation_active = False
        self._navigation_follower_state = "stopped"
        self._imu_calibration_lock = threading.Lock()
        if self._persisted_imu_calibration:
            self._imu_calibration = {
                **self._persisted_imu_calibration,
                "state": "calibrated",
                "message": "已加载此机器人的安装角度校准",
                "calibrated": True,
                "sample_count": 0,
                "required_samples": self.imu_level_calibration_samples,
                "persisted": True,
            }
        else:
            self._imu_calibration = {
                "state": "offline",
                "message": "点击校准后将自动启动 USB/H30 校准链路",
                "sample_count": 0,
                "required_samples": self.imu_level_calibration_samples,
                "calibrated": False,
                "roll_degrees": None,
                "pitch_degrees": None,
                "persisted": False,
            }
        self._imu_calibration_received_at: Optional[float] = None
        self._imu_calibration_future = None
        self._imu_calibration_process: Optional[subprocess.Popen] = None
        self._imu_calibration_started_at: Optional[float] = None
        self._imu_calibration_worker: Optional[threading.Thread] = None
        self._imu_calibration_stop_requested = False

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(Twist, self.cmd_vel_topic, qos)
        self.imu_calibration_client = self.create_client(
            Trigger, self.imu_level_calibration_service
        )
        imu_calibration_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.imu_calibration_subscription = self.create_subscription(
            String,
            self.imu_level_status_topic,
            self._on_imu_calibration_status,
            imu_calibration_qos,
        )
        self.imu_calibration_timer = self.create_timer(
            0.25, self._poll_imu_calibration_process
        )
        self.d1_fsm_subscription = None
        self.d1_controller_status_client = None
        self.d1_parameter_client = None
        self.d1_body_height_publisher = None
        self.d1_body_height_subscription = None
        self.d1_battery_subscriptions = []
        self.d1_feedback_timer = None
        if self.d1_control.enabled:
            d1_feedback_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.d1_fsm_subscription = self.create_subscription(
                String,
                self.d1_fsm_topic,
                self._on_d1_fsm,
                d1_feedback_qos,
            )
            for index, battery_topic in enumerate(self.d1_battery_topics):
                self.d1_battery_subscriptions.append(
                    self.create_subscription(
                        BatteryState,
                        battery_topic,
                        lambda message, pack=index: self._on_d1_battery(
                            pack, message
                        ),
                        d1_feedback_qos,
                    )
                )
            self.d1_body_height_publisher = self.create_publisher(
                Float64, self.d1_body_height_command_topic, qos
            )
            self.d1_body_height_subscription = self.create_subscription(
                Float64,
                self.d1_body_height_status_topic,
                self._on_d1_body_height,
                d1_feedback_qos,
            )
            self.d1_controller_status_client = self.create_client(
                Trigger, self.d1_controller_status_service
            )
            self.d1_parameter_client = self.create_client(
                GetParameters, self.d1_parameter_service
            )
            self.d1_feedback_timer = self.create_timer(
                1.0, self._poll_d1_feedback
            )
        self.navigation_goal_publisher = self.create_publisher(
            PoseStamped, self.navigation_goal_topic, qos
        )
        self.navigation_start_publisher = self.create_publisher(
            Bool, self.navigation_start_topic, qos
        )
        self.navigation_stop_publisher = self.create_publisher(
            Bool, self.navigation_stop_topic, qos
        )
        self.navigation_emergency_stop_publisher = self.create_publisher(
            Bool, self.navigation_emergency_stop_topic, qos
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
        self.navigation_planning_status_subscription = self.create_subscription(
            String,
            self.navigation_planning_status_topic,
            self._on_navigation_planning_status,
            navigation_qos,
        )
        self.navigation_active_subscription = self.create_subscription(
            Bool,
            self.navigation_active_topic,
            self._on_navigation_active,
            navigation_qos,
        )
        self.navigation_follower_state_subscription = self.create_subscription(
            String,
            self.navigation_follower_state_topic,
            self._on_navigation_follower_state,
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
        self.navigation_refined_pose_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            self.navigation_refined_pose_topic,
            self._on_navigation_refined_pose,
            localization_qos,
        )
        self.navigation_terrain_pose_subscription = self.create_subscription(
            PoseStamped,
            self.navigation_terrain_pose_topic,
            self._on_navigation_terrain_pose,
            localization_qos,
        )
        self.navigation_refined_fitness_subscription = self.create_subscription(
            Float32,
            self.navigation_refined_fitness_topic,
            self._on_navigation_refined_fitness,
            qos,
        )
        self.navigation_refined_status_subscription = self.create_subscription(
            String,
            self.navigation_refined_status_topic,
            self._on_navigation_refined_status,
            qos,
        )
        self.navigation_hloc_status_subscription = self.create_subscription(
            String,
            self.navigation_hloc_status_topic,
            self._on_navigation_hloc_status,
            qos,
        )
        self.navigation_hloc_diagnostics_subscription = self.create_subscription(
            String,
            self.navigation_hloc_diagnostics_topic,
            self._on_navigation_hloc_diagnostics,
            qos,
        )
        if self.preview_enabled:
            image_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.rgb_subscription = self.create_subscription(
                CompressedImage,
                self.rgb_preview_topic,
                self._on_rgb_preview,
                image_qos,
            )
        if self.preview_enabled and self.cloud_preview_enabled:
            cloud_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
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

        self.http_servers = []
        requested_port = http_port
        try:
            for address in bind_addresses:
                server = ControlHTTPServer((address, requested_port), self)
                self.http_servers.append(server)
                if requested_port == 0:
                    requested_port = int(server.server_address[1])
        except OSError:
            for server in self.http_servers:
                server.server_close()
            raise
        self.http_server = self.http_servers[0]
        self.http_port = int(self.http_server.server_address[1])
        self.access_urls = [
            f"http://{server.server_address[0]}:{server.server_address[1]}"
            for server in self.http_servers
        ]
        self._http_threads = []
        for index, server in enumerate(self.http_servers):
            thread = threading.Thread(
                target=server.serve_forever,
                kwargs={"poll_interval": 0.1},
                name=f"luxi-web-control-http-{index}",
                daemon=True,
            )
            self._http_threads.append(thread)
            thread.start()
        self._http_thread = self._http_threads[0]
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
        if self.d1_feedback_timeout <= 0.0:
            raise ValueError("d1_feedback_timeout must be greater than zero")
        if self.d1_control.transition_timeout <= 0.0:
            raise ValueError("d1_transition_timeout must be greater than zero")
        if self.imu_level_calibration_start_timeout <= 0.0:
            raise ValueError("imu_level_calibration_start_timeout must be positive")
        if self.imu_level_calibration_result_timeout <= 0.0:
            raise ValueError("imu_level_calibration_result_timeout must be positive")
        if self.imu_level_calibration_samples <= 0:
            raise ValueError("imu_level_calibration_samples must be positive")
        if not math.isfinite(self.imu_level_camera_yaw_degrees):
            raise ValueError("imu_level_camera_yaw_degrees must be finite")
        if self.max_cloud_points != 0 and not 100 <= self.max_cloud_points <= 20000:
            raise ValueError(
                "max_cloud_points must be zero or between 100 and 20000"
            )
        if self.max_saved_cloud_points < 0:
            raise ValueError("max_saved_cloud_points must be zero or positive")
        if not 100 <= self.max_voxel_points <= 50000:
            raise ValueError("max_voxel_points must be between 100 and 50000")
        if self.navigation_localization_max_variance <= 0.0:
            raise ValueError("navigation_localization_max_variance must be positive")
        if self.navigation_localization_timeout <= 0.0:
            raise ValueError("navigation_localization_timeout must be positive")
        if self.map_conversion_timeout <= 0.0:
            raise ValueError("map_conversion_timeout must be positive")
        if self.hloc_index_builder.timeout <= 0.0:
            raise ValueError("hloc_index_build_timeout must be positive")
        if self.navigation_terrain_load_timeout <= 0.0:
            raise ValueError("navigation_terrain_load_timeout must be positive")
        if self.semantic_annotation_store.timeout <= 0.0:
            raise ValueError("semantic_annotation_timeout must be positive")
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
                self._control_owner_id = ""
                self._timed_out = True
            command = self._command
        self._publish(command)

    def _on_d1_fsm(self, message: String) -> None:
        """Cache the robot controller's transient-local FSM feedback."""
        new_state = message.data.strip()
        with self._d1_status_lock:
            old_posture = classify_d1_posture(self._d1_fsm_state)
            new_posture = classify_d1_posture(new_state)
            self._d1_fsm_state = new_state
            self._d1_fsm_received_at = time.monotonic()
            self._d1_feedback_error = ""
        posture_changed = (
            old_posture != "unknown"
            and new_posture != "unknown"
            and old_posture != new_posture
        )
        if posture_changed:
            with self._imu_calibration_lock:
                if self._imu_calibration.get("state") == "calibrated":
                    self._imu_calibration = {
                        **self._imu_calibration,
                        "state": "stale",
                        "calibrated": False,
                        "message": "机器人姿态已变化，请在站立静止后重新校准 IMU",
                    }

    def _on_imu_calibration_status(self, message: String) -> None:
        """Cache the C++ level calibrator's transient-local status."""
        try:
            status = json.loads(message.data)
            if not isinstance(status, dict) or not isinstance(status.get("state"), str):
                raise ValueError("invalid IMU calibration status")
        except (json.JSONDecodeError, ValueError):
            self.get_logger().warning("Ignoring invalid IMU calibration status")
            return
        with self._imu_calibration_lock:
            self._imu_calibration = {
                **status,
                "persisted": False,
            }
            self._imu_calibration_received_at = time.monotonic()
        if status.get("state") == "calibrated" and status.get("calibrated"):
            try:
                self._persist_imu_calibration(status)
            except (OSError, RuntimeError, ValueError) as exc:
                self.get_logger().error(f"Cannot save IMU calibration: {exc}")
                with self._imu_calibration_lock:
                    self._imu_calibration = {
                        **self._imu_calibration,
                        "state": "failed",
                        "message": f"校准结果保存失败：{exc}",
                        "persisted": False,
                    }
            finally:
                self._stop_standalone_imu_calibration_async()

    def _standalone_imu_calibration_error(self) -> str:
        """Return an actionable reason why the temporary USB/H30 launch cannot run."""
        if not self.imu_level_standalone_enabled:
            return "USB/H30 独立校准未启用"
        if not self.imu_level_calibration_launch_package:
            return "校准 launch package 未配置"
        if not self.imu_level_calibration_launch_file:
            return "校准 launch file 未配置"
        missing = [
            path for path in (
                self.imu_level_calibration_workspace_setup,
                self.imu_level_calibration_sensor_setup,
            )
            if not path.is_file()
        ]
        if missing:
            return "校准工作空间未构建：" + ", ".join(str(path) for path in missing)
        return ""

    def _imu_calibration_command(self) -> list[str]:
        """Build the short-lived H30-only level calibration launch command."""
        camera_x, camera_y, camera_z = self._imu_level_camera_translation
        source_commands = [
            f"source {shlex.quote(str(self.imu_level_calibration_workspace_setup))}",
            f"source {shlex.quote(str(self.imu_level_calibration_sensor_setup))}",
            "export ROS_LOCALHOST_ONLY=0",
        ]
        if self.mapping.rmw_implementation:
            source_commands.append(
                "export RMW_IMPLEMENTATION="
                + shlex.quote(self.mapping.rmw_implementation)
            )
        launch_command = shlex.join([
            "ros2", "launch",
            self.imu_level_calibration_launch_package,
            self.imu_level_calibration_launch_file,
            f"calibration_service:={self.imu_level_calibration_service}",
            f"status_topic:={self.imu_level_status_topic}",
            f"calibration_samples:={self.imu_level_calibration_samples}",
            f"camera_x:={camera_x}",
            f"camera_y:={camera_y}",
            f"camera_z:={camera_z}",
            "camera_yaw:=" + format(
                math.radians(self.imu_level_camera_yaw_degrees), ".12g"
            ),
        ])
        return [
            "/bin/bash", "-c",
            "set -e; " + "; ".join(source_commands)
            + f"; exec {launch_command}",
        ]

    def _request_imu_calibration_service(self) -> None:
        """Issue Trigger after either an existing or temporary service is ready."""
        with self._imu_calibration_lock:
            self._imu_calibration = {
                **self._imu_calibration,
                "state": "waiting_stationary",
                "message": "等待机器人在水平面保持静止",
                "sample_count": 0,
                "required_samples": self.imu_level_calibration_samples,
                "persisted": False,
            }
        future = self.imu_calibration_client.call_async(Trigger.Request())
        self._imu_calibration_future = future
        future.add_done_callback(self._on_imu_calibration_response)

    def _wait_for_standalone_imu_calibrator(self) -> None:
        deadline = time.monotonic() + self.imu_level_calibration_start_timeout
        while time.monotonic() < deadline:
            with self._imu_calibration_lock:
                process = self._imu_calibration_process
            if process is None:
                return
            exit_code = process.poll()
            if exit_code is not None:
                with self._imu_calibration_lock:
                    self._imu_calibration = {
                        **self._imu_calibration,
                        "state": "failed",
                        "message": f"H30 校准进程提前退出（code={exit_code}），请检查校准日志",
                    }
                return
            if self.imu_calibration_client.wait_for_service(timeout_sec=0.2):
                self._request_imu_calibration_service()
                return
        with self._imu_calibration_lock:
            self._imu_calibration = {
                **self._imu_calibration,
                "state": "failed",
                "message": "H30 校准服务启动超时，请检查串口和校准日志",
            }
        self._stop_standalone_imu_calibration_async()

    def _start_standalone_imu_calibration(self) -> Tuple[bool, str]:
        error = self._standalone_imu_calibration_error()
        if error:
            return False, error
        with self._imu_calibration_lock:
            process = self._imu_calibration_process
            if process is not None and process.poll() is None:
                return False, "H30 安装角度校准已经在启动或采集中"
            self._imu_calibration = {
                **self._imu_calibration,
                "state": "starting",
                "message": "正在独立启动 H30，请保持机器人静止",
                "sample_count": 0,
                "required_samples": self.imu_level_calibration_samples,
                "persisted": False,
            }
            self._imu_calibration_started_at = time.monotonic()
            self._imu_calibration_stop_requested = False
        try:
            self.imu_level_calibration_log_path.parent.mkdir(
                parents=True, exist_ok=True
            )
            with self.imu_level_calibration_log_path.open(
                "a", encoding="utf-8"
            ) as log_file:
                log_file.write(
                    "\n===== USB/H30 mount calibration started by "
                    "luxi_web_control =====\n"
                )
                process = subprocess.Popen(
                    self._imu_calibration_command(),
                    stdout=log_file,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                    env=sanitized_subprocess_environment(),
                )
        except OSError as exc:
            with self._imu_calibration_lock:
                self._imu_calibration = {
                    **self._imu_calibration,
                    "state": "failed",
                    "message": f"无法启动 H30 校准：{exc}",
                }
            return False, f"无法启动 H30 校准：{exc}"
        with self._imu_calibration_lock:
            self._imu_calibration_process = process
            self._imu_calibration_worker = threading.Thread(
                target=self._wait_for_standalone_imu_calibrator,
                name="luxi-imu-calibration-start",
                daemon=True,
            )
            worker = self._imu_calibration_worker
        worker.start()
        return True, "USB/H30 安装角度校准正在启动，请勿移动机器人"

    def _stop_standalone_imu_calibration(self) -> None:
        with self._imu_calibration_lock:
            process = self._imu_calibration_process
            if process is None:
                return
            self._imu_calibration_stop_requested = True
        if process.poll() is None:
            try:
                process.send_signal(signal.SIGINT)
                process.wait(timeout=5.0)
            except ProcessLookupError:
                pass
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGTERM)
                    process.wait(timeout=2.0)
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    if process.poll() is None:
                        os.killpg(process.pid, signal.SIGKILL)
        with self._imu_calibration_lock:
            if self._imu_calibration_process is process:
                self._imu_calibration_process = None
                self._imu_calibration_started_at = None

    def _stop_standalone_imu_calibration_async(self) -> None:
        with self._imu_calibration_lock:
            process = self._imu_calibration_process
            already_stopping = self._imu_calibration_stop_requested
            if process is None or already_stopping:
                return
            self._imu_calibration_stop_requested = True
        threading.Thread(
            target=self._stop_standalone_imu_calibration,
            name="luxi-imu-calibration-stop",
            daemon=True,
        ).start()

    def _persist_imu_calibration(self, status: Dict[str, Any]) -> None:
        """Atomically save the mount and update future map/navigation launches."""
        overrides = camera_calibration_overrides(
            status, self.imu_level_camera_yaw_degrees
        )
        saved = {
            "version": 1,
            "robot_namespace": self.robot_namespace,
            "calibrated": True,
            "roll_degrees": float(status["roll_degrees"]),
            "pitch_degrees": float(status["pitch_degrees"]),
            "installation_roll_degrees": float(status.get(
                "installation_roll_degrees",
                float(status["roll_degrees"]) + 90.0,
            )),
            "installation_pitch_degrees": float(status.get(
                "installation_pitch_degrees", status["pitch_degrees"]
            )),
            "camera_yaw_degrees": self.imu_level_camera_yaw_degrees,
            "camera_quaternion": {
                name.removeprefix("camera_"): float(value)
                for name, value in overrides.items()
            },
            "updated_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        }
        self.imu_level_calibration_store_path.parent.mkdir(
            parents=True, exist_ok=True
        )
        temporary_path = self.imu_level_calibration_store_path.with_suffix(
            self.imu_level_calibration_store_path.suffix + ".tmp"
        )
        temporary_path.write_text(
            json.dumps(saved, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        os.replace(temporary_path, self.imu_level_calibration_store_path)
        self.mapping.set_camera_calibration(overrides)
        self.navigation.set_camera_calibration(overrides)
        self._persisted_imu_calibration = saved
        with self._imu_calibration_lock:
            self._imu_calibration = {
                **saved,
                "state": "calibrated",
                "message": "安装角度已保存，后续建图和定位会自动使用",
                "sample_count": int(status.get("sample_count", 0)),
                "required_samples": int(status.get(
                    "required_samples", self.imu_level_calibration_samples
                )),
                "persisted": True,
            }
            self._imu_calibration_received_at = time.monotonic()

    def _poll_imu_calibration_process(self) -> None:
        with self._imu_calibration_lock:
            process = self._imu_calibration_process
            started_at = self._imu_calibration_started_at
            state = self._imu_calibration.get("state")
            stop_requested = self._imu_calibration_stop_requested
        if process is None:
            return
        exit_code = process.poll()
        if exit_code is not None:
            with self._imu_calibration_lock:
                self._imu_calibration_process = None
                self._imu_calibration_started_at = None
                if not stop_requested and state not in {"calibrated", "failed"}:
                    self._imu_calibration = {
                        **self._imu_calibration,
                        "state": "failed",
                        "message": f"H30 校准进程退出（code={exit_code}）",
                    }
            return
        if (
            started_at is not None
            and state in {"starting", "waiting_stationary", "collecting"}
            and time.monotonic() - started_at
            > self.imu_level_calibration_result_timeout
        ):
            with self._imu_calibration_lock:
                self._imu_calibration = {
                    **self._imu_calibration,
                    "state": "failed",
                    "message": "安装角度校准超时；请确认 H30 出流且机器人完全静止",
                }
            self._stop_standalone_imu_calibration_async()

    def _on_imu_calibration_response(self, future) -> None:
        try:
            response = future.result()
            if response is None or not response.success:
                raise RuntimeError(
                    response.message if response is not None else "empty response"
                )
        except Exception as exc:  # noqa: BLE001 - ROS futures expose transport errors.
            with self._imu_calibration_lock:
                self._imu_calibration = {
                    **self._imu_calibration,
                    "state": "failed",
                    "message": f"IMU 校准请求失败：{exc}",
                }
            self._stop_standalone_imu_calibration_async()

    def imu_calibration_status(self) -> Dict[str, Any]:
        """Return the latest C++ calibrator state for the web page."""
        with self._imu_calibration_lock:
            status = dict(self._imu_calibration)
            received_at = self._imu_calibration_received_at
        status["service_available"] = self.imu_calibration_client.service_is_ready()
        status["status_age_seconds"] = (
            None if received_at is None
            else round(max(0.0, time.monotonic() - received_at), 2)
        )
        status["busy"] = status.get("state") in {
            "starting", "waiting_stationary", "collecting"
        }
        standalone_error = self._standalone_imu_calibration_error()
        status["standalone_available"] = not bool(standalone_error)
        status["standalone_error"] = standalone_error or None
        status["store_path"] = str(self.imu_level_calibration_store_path)
        status["log_path"] = str(self.imu_level_calibration_log_path)
        d1_status = self.d1_control_status()
        status["robot_standing"] = bool(
            not d1_status["enabled"] or d1_status["posture"] == "standing"
        )
        mapping_running = self.mapping.status()["state"] == "running"
        navigation_status = self.navigation.status()
        navigation_running = navigation_status["state"] == "running"
        status["can_start"] = (
            (status["service_available"] or status["standalone_available"])
            and not status["busy"]
            and status["robot_standing"]
            and not mapping_running and not navigation_running
        )
        return status

    def start_imu_level_calibration(self) -> Tuple[bool, str]:
        """Request stationary level calibration only outside mapping/navigation."""
        if self.mapping_status()["state"] == "running":
            return False, "请先停止建图，再校准 IMU"
        navigation = self.navigation_status()
        if navigation["state"] == "running" or navigation["active"]:
            return False, "请先停止定位和导航，再校准 IMU"
        d1_status = self.d1_control_status()
        if d1_status["enabled"] and d1_status["posture"] != "standing":
            return False, "请先让机器人站立并保持静止，再校准 IMU"
        if self.imu_calibration_status()["busy"]:
            return False, "IMU 正在校准，请保持机器人静止"
        self.stop_motion()
        if self.imu_calibration_client.service_is_ready():
            self._request_imu_calibration_service()
            return True, "IMU 水平校准已启动，请勿移动机器人"
        return self._start_standalone_imu_calibration()

    def _poll_d1_feedback(self) -> None:
        """Poll the vendor's morphology and SDK-mode services at 1 Hz."""
        now = time.monotonic()
        request_timeout = max(1.0, self.d1_feedback_timeout)
        if d1_feedback_request_timed_out(
            self._d1_controller_future,
            self._d1_controller_requested_at,
            now,
            request_timeout,
        ):
            stale_future = self._d1_controller_future
            self._d1_controller_future = None
            self._d1_controller_requested_at = None
            stale_future.cancel()
            with self._d1_status_lock:
                self._d1_feedback_error = (
                    "D1 controller feedback timed out; retrying"
                )
        if d1_feedback_request_timed_out(
            self._d1_parameter_future,
            self._d1_parameter_requested_at,
            now,
            request_timeout,
        ):
            stale_future = self._d1_parameter_future
            self._d1_parameter_future = None
            self._d1_parameter_requested_at = None
            stale_future.cancel()
            with self._d1_status_lock:
                self._d1_feedback_error = "D1 SDK feedback timed out; retrying"

        if (
            self.d1_controller_status_client is not None
            and self.d1_controller_status_client.service_is_ready()
            and self._d1_controller_future is None
        ):
            future = self.d1_controller_status_client.call_async(
                Trigger.Request()
            )
            self._d1_controller_future = future
            self._d1_controller_requested_at = now
            future.add_done_callback(self._on_d1_controller_status)
        if (
            self.d1_parameter_client is not None
            and self.d1_parameter_client.service_is_ready()
            and self._d1_parameter_future is None
        ):
            request = GetParameters.Request()
            request.names = ["use_sdk"]
            future = self.d1_parameter_client.call_async(request)
            self._d1_parameter_future = future
            self._d1_parameter_requested_at = now
            future.add_done_callback(self._on_d1_parameters)

    def _on_d1_controller_status(self, future) -> None:
        if future is not self._d1_controller_future:
            return
        self._d1_controller_future = None
        self._d1_controller_requested_at = None
        try:
            response = future.result()
            if response is None or not response.success:
                raise RuntimeError(
                    response.message if response is not None
                    else "empty controller status response"
                )
            with self._d1_status_lock:
                self._d1_controller_mode = response.message.strip()
                self._d1_controller_received_at = time.monotonic()
                self._d1_feedback_error = ""
        except Exception as exc:  # ROS futures surface transport errors here.
            with self._d1_status_lock:
                self._d1_feedback_error = str(exc)

    def _on_d1_parameters(self, future) -> None:
        if future is not self._d1_parameter_future:
            return
        self._d1_parameter_future = None
        self._d1_parameter_requested_at = None
        try:
            response = future.result()
            if (
                response is None
                or len(response.values) != 1
                or response.values[0].type != ParameterType.PARAMETER_BOOL
            ):
                raise RuntimeError("invalid D1 parameter response")
            with self._d1_status_lock:
                self._d1_sdk_active = bool(response.values[0].bool_value)
                self._d1_sdk_received_at = time.monotonic()
                self._d1_feedback_error = ""
        except Exception as exc:  # ROS futures surface transport errors here.
            with self._d1_status_lock:
                self._d1_feedback_error = str(exc)

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
            if points:
                self._last_valid_path_points = list(points)
                self._last_valid_path_frame_id = self._path_frame_id
                self._last_valid_path_received_at = self._path_received_at
                self._planning_state = "ready"
                self._planning_error = ""
            elif self._planning_state != "failed":
                self._planning_state = "failed"
                self._planning_error = "规划器未生成可执行路径"

    def _on_navigation_planning_status(self, message: String) -> None:
        errors = {
            "failed_map_unavailable": "三维地图尚未加载完成",
            "failed_localization_unavailable": "无法获取机器人在地图中的定位",
            "failed_start_or_goal_unsupported": "机器人起点或目标点附近没有可通行地形",
            "failed_no_path": "起点与目标点之间不存在连通的可通行路径",
        }
        status = message.data[:80]
        with self._navigation_lock:
            if status == "planning":
                self._planning_state = "pending"
                self._planning_error = ""
            elif status == "ready":
                if self._planned_path_points:
                    self._planning_state = "ready"
                    self._planning_error = ""
            elif status in errors:
                self._planned_path_points = []
                self._planning_state = "failed"
                self._planning_error = errors[status]
            elif status == "waiting_map":
                self._planning_state = "idle"
                self._planning_error = ""

    def _on_navigation_active(self, message: Bool) -> None:
        """Track whether the C++ path follower currently owns navigation."""
        with self._navigation_lock:
            self._navigation_active = bool(message.data)

    def _on_navigation_follower_state(self, message: String) -> None:
        """Expose the bounded C++ follower state to the browser."""
        with self._navigation_lock:
            self._navigation_follower_state = message.data[:80]

    def _on_navigation_localization_pose(
        self, message: PoseWithCovarianceStamped
    ) -> None:
        covariance = list(message.pose.covariance)
        pose = localization_pose_summary(message)
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
            self._coarse_localization_pose = pose

    def _on_navigation_refined_pose(
        self, message: PoseWithCovarianceStamped
    ) -> None:
        pose = localization_pose_summary(message)
        if pose is None:
            return
        with self._navigation_lock:
            self._coarse_gate_ready = True
            self._refined_localization_pose = pose
            self._refined_localization_received_at = time.monotonic()

    def _on_navigation_terrain_pose(self, message: PoseStamped) -> None:
        wrapped = PoseWithCovarianceStamped()
        wrapped.header = message.header
        wrapped.pose.pose = message.pose
        pose = localization_pose_summary(wrapped)
        if pose is None:
            return
        with self._navigation_lock:
            self._terrain_pose = pose
            self._terrain_pose_received_at = time.monotonic()

    def _on_navigation_refined_fitness(self, message: Float32) -> None:
        if not math.isfinite(message.data):
            return
        with self._navigation_lock:
            self._refined_localization_fitness = round(float(message.data), 4)
            if message.data >= self.navigation_icp_minimum_fitness:
                self._refined_localization_verified_at = time.monotonic()

    def _on_navigation_refined_status(self, message: String) -> None:
        with self._navigation_lock:
            self._refined_localization_status = message.data[:300]
            if "HLoc consistency 3/3" in message.data:
                self._coarse_gate_ready = True
            elif (
                "consistent HLoc poses" in message.data
                or "restarting HLoc" in message.data
            ):
                self._coarse_gate_ready = False
                self._refined_localization_pose = None
                self._refined_localization_received_at = None
                self._refined_localization_verified_at = None

    def _on_navigation_hloc_status(self, message: String) -> None:
        """Expose the coarse localizer's actual state instead of hiding it."""
        with self._navigation_lock:
            self._hloc_status = message.data[:300]
            self._hloc_status_received_at = time.monotonic()

    def _on_navigation_hloc_diagnostics(self, message: String) -> None:
        """Cache bounded JSON diagnostics from the HLoc query."""
        try:
            diagnostics = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        if not isinstance(diagnostics, dict):
            return
        allowed = {
            "accepted", "reason", "reference", "retrieval_score",
            "elapsed_seconds", "candidates_tested", "device", "matches",
            "landmarks", "inliers", "inlier_ratio", "depth_verified",
            "median_depth_residual", "reprojection_rmse",
        }
        bounded = {}
        for key in allowed:
            if key not in diagnostics:
                continue
            value = diagnostics[key]
            if isinstance(value, float) and not math.isfinite(value):
                value = None
            elif isinstance(value, str):
                value = value[:300]
            bounded[key] = value
        with self._navigation_lock:
            self._hloc_diagnostics = bounded
            self._hloc_diagnostics_received_at = time.monotonic()

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
            self._navigation_voxel_variant = None
            self._navigation_voxel_error = ""
            self._terrain_traversable_points = []
            self._terrain_obstacle_points = []
            self._terrain_resolution = 0.0
            self._terrain_map_id = None
            self._terrain_variant = None
            self._terrain_error = ""
            self._localization_ready = False
            self._localization_variance = None
            self._localization_received_at = None
            self._coarse_localization_pose = None
            self._coarse_gate_ready = False
            self._refined_localization_pose = None
            self._refined_localization_received_at = None
            self._terrain_pose = None
            self._terrain_pose_received_at = None
            self._refined_localization_fitness = None
            self._refined_localization_status = ""
            self._refined_localization_verified_at = None
            self._hloc_status = ""
            self._hloc_status_received_at = None
            self._hloc_diagnostics = None
            self._hloc_diagnostics_received_at = None
            self._navigation_cloud_points = []
            self._navigation_cloud_map_id = None
            self._navigation_cloud_variant = None
            self._navigation_cloud_error = ""
            self._planned_path_points = []
            self._last_valid_path_points = []
            self._last_valid_path_frame_id = "map"
            self._path_received_at = None
            self._last_valid_path_received_at = None
            self._planning_state = "idle"
            self._planning_error = ""
            self._navigation_active = False
            self._navigation_follower_state = "stopped"

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
                "variant": self._navigation_voxel_variant,
                "frame_id": self._voxel_frame_id,
                "resolution": self._voxel_resolution,
                "point_count": len(self._voxel_points),
                "error": self._navigation_voxel_error or None,
                "age_seconds": None if self._voxel_received_at is None
                else round(time.monotonic() - self._voxel_received_at, 2),
                "points": list(self._voxel_points),
            }

    def _load_navigation_voxels(
        self, map_id: str, octomap_path: str, variant: str = "original"
    ) -> str:
        """Load saved occupied voxels immediately, without waiting for ROS launch."""
        command = [
            str(self.octomap_points_executable), str(octomap_path),
            str(self.max_voxel_points),
        ]
        library_path = str(self.navigation.octomap_library_path)
        environment = sanitized_subprocess_environment(
            (library_path,)
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
                self._navigation_voxel_variant = variant
                self._navigation_voxel_error = error
                self._voxel_received_at = None
            return error
        with self._navigation_lock:
            self._voxel_points = points
            self._voxel_frame_id = "map"
            self._voxel_resolution = resolution
            self._voxel_received_at = time.monotonic()
            self._navigation_voxel_map_id = map_id
            self._navigation_voxel_variant = variant
            self._navigation_voxel_error = ""
        return ""

    def navigation_cloud_preview(self) -> Dict[str, Any]:
        """Return the selected saved map's RGB point cloud."""
        with self._navigation_lock:
            return {
                "map_id": self._navigation_cloud_map_id,
                "variant": self._navigation_cloud_variant,
                "point_count": len(self._navigation_cloud_points),
                "error": self._navigation_cloud_error or None,
                "points": list(self._navigation_cloud_points),
            }

    def terrain_preview(self) -> Dict[str, Any]:
        """Return terrain segmentation and edge costs for browser rendering."""
        with self._navigation_lock:
            return {
                "map_id": self._terrain_map_id,
                "variant": self._terrain_variant,
                "resolution": self._terrain_resolution,
                "robot_radius": self.navigation_robot_radius,
                "costmap_margin": self.navigation_costmap_margin,
                "ground_normal_radius": self.navigation_ground_normal_radius,
                "ground_max_slope_degrees": (
                    self.navigation_ground_max_slope_degrees
                ),
                "obstacle_min_height": self.navigation_obstacle_min_height,
                "traversable_count": len(self._terrain_traversable_points),
                "obstacle_count": len(self._terrain_obstacle_points),
                "error": self._terrain_error or None,
                "traversable_points": list(self._terrain_traversable_points),
                "obstacle_points": list(self._terrain_obstacle_points),
            }

    def _load_navigation_terrain(
        self, map_id: str, octomap_path: str, cloud_path: Optional[str],
        variant: str = "original"
    ) -> str:
        """Build bounded terrain layers through the shared C++ implementation."""
        if not cloud_path:
            return "no colored point cloud is available for terrain fitting"
        command = [
            str(self.terrain_points_executable),
            str(octomap_path),
            str(self.max_terrain_points),
            str(self.navigation_robot_radius),
            str(self.navigation_costmap_margin),
            str(cloud_path),
            str(self.navigation_ground_normal_radius),
            str(self.navigation_ground_max_slope_degrees),
            str(self.navigation_obstacle_min_height),
        ]
        environment = sanitized_subprocess_environment(
            (str(self.navigation.octomap_library_path),)
        )
        try:
            result = subprocess.run(
                command,
                check=True,
                capture_output=True,
                text=True,
                timeout=self.navigation_terrain_load_timeout,
                env=environment,
            )
            resolution, traversable, obstacles = parse_terrain_point_output(
                result.stdout
            )
        except (OSError, subprocess.SubprocessError, UnicodeDecodeError,
                ValueError) as exc:
            error = str(exc)
            with self._navigation_lock:
                self._terrain_traversable_points = []
                self._terrain_obstacle_points = []
                self._terrain_map_id = map_id
                self._terrain_variant = variant
                self._terrain_error = error
            return error
        with self._navigation_lock:
            self._terrain_traversable_points = traversable
            self._terrain_obstacle_points = obstacles
            self._terrain_resolution = resolution
            self._terrain_map_id = map_id
            self._terrain_variant = variant
            self._terrain_error = ""
        return ""

    def _load_navigation_cloud(
        self, map_id: str, cloud_path: Optional[str], variant: str = "original"
    ) -> str:
        if not cloud_path:
            with self._navigation_lock:
                self._navigation_cloud_points = []
                self._navigation_cloud_map_id = map_id
                self._navigation_cloud_variant = variant
                self._navigation_cloud_error = "no exported colored PLY is available"
            return self._navigation_cloud_error
        try:
            points = extract_colored_ply_points(
                Path(cloud_path), self.max_saved_cloud_points)
        except (OSError, UnicodeDecodeError, ValueError, struct.error) as exc:
            with self._navigation_lock:
                self._navigation_cloud_points = []
                self._navigation_cloud_map_id = map_id
                self._navigation_cloud_variant = variant
                self._navigation_cloud_error = str(exc)
            return self._navigation_cloud_error
        with self._navigation_lock:
            self._navigation_cloud_points = points
            self._navigation_cloud_map_id = map_id
            self._navigation_cloud_variant = variant
            self._navigation_cloud_error = ""
        return ""

    def path_preview(self) -> Dict[str, Any]:
        """Return the latest A* global path for Canvas rendering."""
        with self._navigation_lock:
            valid = bool(self._planned_path_points)
            points = (
                self._planned_path_points if valid else self._last_valid_path_points
            )
            received_at = (
                self._path_received_at if valid else self._last_valid_path_received_at
            )
            return {
                "frame_id": (
                    self._path_frame_id if valid else self._last_valid_path_frame_id
                ),
                "point_count": len(points),
                "active_point_count": len(self._planned_path_points),
                "valid": valid,
                "stale": bool(points) and not valid,
                "planning_state": self._planning_state,
                "error": self._planning_error or None,
                "age_seconds": None if received_at is None
                else round(time.monotonic() - received_at, 2),
                "points": list(points),
            }

    def preview_status(self) -> Dict[str, Any]:
        """Return lightweight preview availability without cloud point data."""
        with self._preview_lock:
            rgb_received_at = self._rgb_received_at
            return {
                "enabled": self.preview_enabled,
                "cloud_enabled": self.cloud_preview_enabled,
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

    def accept_command(
        self, command: VelocityCommand, client_id: str = ""
    ) -> Tuple[bool, str]:
        """Store and immediately publish a validated browser command."""
        d1_status = self.d1_control_status()
        if (
            d1_status["enabled"]
            and not d1_status["control_ready"]
            and command.moving
        ):
            return False, "D1 is not standing with SDK control and bridge ready"
        with self._lock:
            if self._estop_active and command.moving:
                return False, "emergency stop is active"
            now = time.monotonic()
            requester_id = client_id or LEGACY_CONTROL_CLIENT_ID
            owner_is_live = (
                bool(self._control_owner_id)
                and self._last_command_time is not None
                and now - self._last_command_time <= self.command_timeout
            )
            if (
                command.moving
                and owner_is_live
                and self._control_owner_id != requester_id
            ):
                return False, "control is in use by another browser"
            self._command = command
            self._last_command_time = (
                now if command.moving else None
            )
            if command.moving:
                self._control_owner_id = requester_id
            elif not owner_is_live or self._control_owner_id == requester_id:
                self._control_owner_id = ""
            self._timed_out = False
        self._publish(command)
        return True, ""

    def control_session_active(self) -> bool:
        """Return whether a browser currently owns the manual-control lease."""
        with self._lock:
            return bool(self._control_owner_id)

    def stop_motion(self, client_id: str = "", force: bool = True) -> bool:
        """Clear motion and immediately publish zero velocity."""
        with self._lock:
            requester_id = client_id or LEGACY_CONTROL_CLIENT_ID
            if (
                not force
                and self._control_owner_id
                and self._control_owner_id != requester_id
            ):
                return False
            self._command = VelocityCommand()
            self._last_command_time = None
            self._control_owner_id = ""
            self._timed_out = False
        self._publish(VelocityCommand())
        return True

    def set_estop(self, active: bool) -> None:
        """Set the sticky software emergency stop; activation always stops."""
        with self._lock:
            self._estop_active = active
            self._command = VelocityCommand()
            self._last_command_time = None
            self._control_owner_id = ""
            self._timed_out = False
        self._publish(VelocityCommand())
        message = Bool()
        message.data = active
        self.navigation_emergency_stop_publisher.publish(message)
        if active:
            self.halt_navigation_motion()

    def set_d1_control(self, active: bool) -> Tuple[bool, str]:
        """Start a safe asynchronous D1 enable or disable transition."""
        self.stop_motion()
        if not active:
            self.halt_navigation_motion()
        status = self.d1_control_status()
        if active and not status["feedback_online"]:
            return False, "D1 state feedback is offline; refusing to stand up"
        if active and status["control_ready"]:
            return True, "D1 is already standing and ready"
        if not active and status["posture"] == "prone" and not status["bridge_active"]:
            return True, "D1 is already prone with control released"
        force = bool(
            (active and status["bridge_active"] and not status["control_ready"])
            or (not active and not status["bridge_active"])
        )
        return self.d1_control.set_active(active, force=force)

    def _on_d1_battery(self, pack: int, message: BatteryState) -> None:
        def finite(value: float) -> Optional[float]:
            return round(float(value), 2) if math.isfinite(value) else None

        statuses = {
            BatteryState.POWER_SUPPLY_STATUS_UNKNOWN: "unknown",
            BatteryState.POWER_SUPPLY_STATUS_CHARGING: "charging",
            BatteryState.POWER_SUPPLY_STATUS_DISCHARGING: "discharging",
            BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING: "not_charging",
            BatteryState.POWER_SUPPLY_STATUS_FULL: "full",
        }
        snapshot = {
            "pack": pack + 1,
            "percentage": normalize_battery_percentage(message.percentage),
            "voltage": finite(message.voltage),
            "current": finite(message.current),
            "temperature": finite(message.temperature),
            "status": statuses.get(message.power_supply_status, "unknown"),
        }
        with self._d1_status_lock:
            self._d1_batteries[pack] = snapshot
            self._d1_battery_received_at[pack] = time.monotonic()

    def _on_d1_body_height(self, message: Float64) -> None:
        if not math.isfinite(message.data):
            return
        with self._d1_status_lock:
            self._d1_body_height = float(message.data)
            self._d1_body_height_received_at = time.monotonic()

    def set_d1_body_height(self, height: float) -> Tuple[bool, str]:
        """Publish a validated height target only while D1 control is ready."""
        status = self.d1_control_status()
        if not status["enabled"]:
            return False, "D1 control is disabled"
        if not status["control_ready"]:
            return False, "D1 must be standing with SDK control and bridge ready"
        if str(status.get("controller_mode") or "").strip().lower() != "biped":
            return False, (
                "当前不是已验证的双足 LQR 模式；为避免触发四足形态策略，"
                "拒绝腿高指令"
            )
        if self.d1_body_height_publisher is None:
            return False, "D1 body-height publisher is unavailable"
        message = Float64()
        message.data = height
        with self._d1_status_lock:
            self._d1_body_height_target = height
        self.d1_body_height_publisher.publish(message)
        percentage = 100.0 * (
            (height - self.d1_body_height_minimum)
            / (self.d1_body_height_maximum - self.d1_body_height_minimum)
        )
        return True, (
            f"D1 single-unit biped height set to level {height:.0f} "
            f"({percentage:.0f}%)"
        )

    def d1_control_status(self) -> Dict[str, Any]:
        """Combine actual D1 FSM/SDK feedback with the local bridge state."""
        managed = self.d1_control.status()
        now = time.monotonic()
        with self._d1_status_lock:
            fsm_state = self._d1_fsm_state
            fsm_received_at = self._d1_fsm_received_at
            controller_mode = self._d1_controller_mode
            controller_received_at = self._d1_controller_received_at
            sdk_active = self._d1_sdk_active
            sdk_received_at = self._d1_sdk_received_at
            feedback_error = self._d1_feedback_error
            batteries = list(self._d1_batteries)
            battery_received_at = list(self._d1_battery_received_at)
            body_height = self._d1_body_height
            body_height_target = self._d1_body_height_target
            body_height_received_at = self._d1_body_height_received_at

        def age(received_at: Optional[float]) -> Optional[float]:
            return None if received_at is None else round(now - received_at, 2)

        fsm_age = age(fsm_received_at)
        controller_age = age(controller_received_at)
        sdk_age = age(sdk_received_at)
        fsm_online = bool(
            managed["enabled"]
            and fsm_state
            and fsm_age is not None
            and self.count_publishers(self.d1_fsm_topic) > 0
        )
        controller_online = bool(
            managed["enabled"]
            and controller_age is not None
            and controller_age <= self.d1_feedback_timeout
        )
        sdk_online = bool(
            managed["enabled"]
            and sdk_age is not None
            and sdk_age <= self.d1_feedback_timeout
        )
        feedback_online = fsm_online and controller_online and sdk_online
        battery_packs = []
        for index, snapshot in enumerate(batteries):
            battery_age = age(battery_received_at[index])
            online = bool(
                managed["enabled"]
                and snapshot is not None
                and battery_age is not None
                and battery_age <= self.d1_feedback_timeout
                and self.count_publishers(self.d1_battery_topics[index]) > 0
            )
            battery_packs.append({
                **(snapshot or {"pack": index + 1}),
                "online": online,
                "age_seconds": battery_age,
            })
        online_packs = [pack for pack in battery_packs if pack["online"]]
        percentages = [
            pack["percentage"] for pack in online_packs
            if pack.get("percentage") is not None
        ]
        height_age = age(body_height_received_at)
        height_online = bool(
            managed["enabled"]
            and height_age is not None
            and height_age <= self.d1_feedback_timeout
            and self.count_publishers(self.d1_body_height_status_topic) > 0
        )
        height_supported = bool(
            controller_online
            and str(controller_mode).strip().lower() == "biped"
        )
        if not controller_online:
            height_reason = "控制器形态状态离线，无法确认高度功能"
        elif height_supported:
            height_reason = "双足 LQR 腿高速率控制已验证"
        else:
            height_reason = (
                "当前不是已验证的双足 LQR 模式"
            )
        posture = classify_d1_posture(fsm_state) if fsm_online else "offline"
        bridge_active = bool(managed["active"])
        transitioning = bool(managed["transitioning"])
        control_ready = bool(
            feedback_online
            and posture == "standing"
            and sdk_online
            and sdk_active is True
            and bridge_active
            and not transitioning
        )
        # `enabled` means the feature is configured; `active` is reserved for
        # actual, feedback-verified web control. Keeping these distinct stops a
        # prone or merely standing robot from appearing "on" in the browser.
        switch_active = control_ready

        if not managed["enabled"]:
            state = "disabled"
        elif transitioning:
            state = managed["state"]
        elif managed["state"] == "failed":
            state = "failed"
        elif control_ready:
            state = "active"
        elif not feedback_online:
            state = "offline"
        elif posture == "prone" and sdk_active is False:
            state = "recovery_required" if bridge_active else "inactive"
        elif posture == "standing_up":
            state = "standing_up"
        elif posture == "standing":
            state = "standing_uncontrolled"
        else:
            state = "unknown"

        return {
            **managed,
            "state": state,
            "active": switch_active,
            "bridge_active": bridge_active,
            "control_ready": control_ready,
            "feedback_online": feedback_online,
            "fsm_online": fsm_online,
            "posture": posture,
            "fsm_state": fsm_state or None,
            "fsm_age_seconds": fsm_age,
            "controller_online": controller_online,
            "controller_mode": controller_mode or None,
            "controller_age_seconds": controller_age,
            "sdk_online": sdk_online,
            "sdk_active": sdk_active,
            "sdk_age_seconds": sdk_age,
            "feedback_error": feedback_error or None,
            "battery": {
                "online": bool(online_packs),
                "percentage": min(percentages) if percentages else None,
                "packs": battery_packs,
            },
            "body_height": {
                "supported": height_supported,
                "reason": height_reason,
                "online": height_online,
                "current": round(body_height, 3) if height_online else None,
                "target": round(body_height_target, 3),
                "minimum": self.d1_body_height_minimum,
                "maximum": self.d1_body_height_maximum,
                "age_seconds": height_age,
            },
        }

    def start_mapping(self, mode: str = "crestereo") -> Tuple[bool, str]:
        """Start the managed RTAB-Map RGB-D mapping launch."""
        d1_status = self.d1_control_status()
        if d1_status["enabled"] and d1_status["posture"] != "standing":
            return False, "请先让机器人站立，再校准 IMU 并开始建图"
        calibration = self.imu_calibration_status()
        if (calibration["service_available"] or calibration["standalone_available"]) and (
            calibration.get("state") != "calibrated" or calibration["busy"]
        ):
            return False, "请先在水平面完成 IMU 一键校准"
        if self.mapping.status()["state"] != "running":
            # The managed USB launch owns both canonical publishers.  Their
            # correct stopped-state count is therefore zero; requiring one
            # here prevents the very process that creates them from starting.
            publisher_counts = {
                self.mapping_rgbd_topic: self.count_publishers(
                    self.mapping_rgbd_topic
                ),
                self.mapping_imu_topic: self.count_publishers(
                    self.mapping_imu_topic
                ),
            }
            active_inputs = mapping_topic_conflicts(publisher_counts)
            if active_inputs:
                return False, (
                    "建图启动前检测到传感器话题已被占用："
                    + "，".join(active_inputs)
                    + "；请停止重复或残留的 D435i/USB/IMU 进程"
                )
            conflicts = mapping_graph_conflicts(
                self.get_node_names_and_namespaces()
            )
            if conflicts:
                return False, (
                    "mapping nodes are already active outside web control: "
                    + ", ".join(conflicts)
                )
        started, message = self.mapping.start(mode)
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

    def _semantic_map_record(self, map_id: str) -> Dict[str, Any]:
        if not MAP_IDENTIFIER.fullmatch(map_id):
            raise ValueError("map_id must use the mapNNN format")
        record = next(
            (item for item in self.navigation_maps() if item["id"] == map_id),
            None,
        )
        if record is None or not record.get("octomap_path"):
            raise ValueError(f"map {map_id} has no saved OctoMap")
        return record

    def semantic_annotations(self, map_id: str) -> dict:
        """Load annotations for one discovered OctoMap."""
        record = self._semantic_map_record(map_id)
        return self.semantic_annotation_store.load(
            map_id, Path(record["octomap_path"])
        )

    def save_semantic_annotations(self, annotation: dict) -> dict:
        """Validate and save annotations without modifying geometry maps."""
        map_id = annotation.get("map_id")
        if not isinstance(map_id, str):
            raise ValueError("annotation map_id must be a string")
        record = self._semantic_map_record(map_id)
        return self.semantic_annotation_store.save(
            map_id, Path(record["octomap_path"]), annotation
        )

    def _convert_navigation_map(
        self, record: Dict[str, Any], filtered: bool = False
    ) -> Tuple[bool, str]:
        """Export a saved RTAB-Map database using the project's trusted tool."""
        map_id = record["id"]
        database_path = record.get("database_path")
        if not database_path:
            return False, f"map {map_id} has no RTAB-Map .db file to convert"
        if not self.map_export_executable.is_file():
            return False, f"map export tool is missing: {self.map_export_executable}"
        if self.mapping_status()["state"] == "running":
            return False, "stop mapping and save the database before converting it"
        database_error = rtabmap_database_conversion_error(
            Path(database_path)
        )
        if database_error:
            return False, f"map {map_id} cannot be converted: {database_error}"

        output_directory = (
            self.maps_root / "octo_maps"
            / f"{map_id}_{'filtered_' if filtered else ''}octomap"
        ).resolve()
        output_directory.mkdir(parents=True, exist_ok=True)
        library_path = str(self.navigation.octomap_library_path)
        environment = sanitized_subprocess_environment(
            (library_path,)
        )
        try:
            command = [str(self.map_export_executable)]
            if filtered:
                command.append("--filter")
            command.extend([str(Path(database_path)), str(output_directory)])
            result = subprocess.run(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                check=True,
                timeout=self.map_conversion_timeout,
                env=environment,
            )
        except subprocess.TimeoutExpired:
            return False, (
                f"map {map_id} conversion exceeded "
                f"{self.map_conversion_timeout:.0f} seconds"
            )
        except (OSError, subprocess.SubprocessError) as exc:
            detail = getattr(exc, "stdout", "") or str(exc)
            return False, f"map {map_id} conversion failed: {detail.strip()[-800:]}"

        converted = next(
            (item for item in self.navigation_maps() if item["id"] == map_id),
            None,
        )
        loadable_key = "filtered_loadable" if filtered else "loadable"
        cloud_key = "filtered_cloud_path" if filtered else "cloud_path"
        if not converted or not converted[loadable_key] or not converted[cloud_key]:
            output = (result.stdout or "").strip()
            return False, (
                f"map {map_id} export did not produce both colored PLY and .bt files"
                + (f": {output[-800:]}" if output else "")
            )
        variant = "filtered" if filtered else "original"
        return True, f"map {map_id} {variant} layers converted for browser display"

    def load_navigation_map(
        self, map_id: str, filtered: bool = False
    ) -> Tuple[bool, str]:
        """
        Build missing display assets, then load browser map layers.

        HLoc feature construction is deliberately not part of this synchronous
        request. Large maps can contain hundreds of reference images, and
        loading NetVLAD/SuperPoint while the web/ROS stack is resident can
        exhaust an NX before the already-exported cloud is displayed.
        """
        if not MAP_IDENTIFIER.fullmatch(map_id):
            return False, "map_id must use the mapNNN format"
        if not self._navigation_map_operation_lock.acquire(blocking=False):
            return False, "another map is already being converted or loaded"
        try:
            record = next(
                (item for item in self.navigation_maps() if item["id"] == map_id),
                None,
            )
            if record is None:
                return False, f"map {map_id} does not exist under {self.maps_root}"
            if not record["database_path"]:
                return False, f"map {map_id} has no RTAB-Map .db file to convert"
            if self.mapping_status()["state"] == "running":
                return False, (
                    "stop mapping and save the database before loading or "
                    "building its HLoc index"
                )

            # A localization launch also owns RTAB-Map processes. Stop it before
            # calling the offline export tool, which correctly refuses live maps.
            self.navigation.stop()
            operation_messages = []
            loadable_key = "filtered_loadable" if filtered else "loadable"
            cloud_key = "filtered_cloud_path" if filtered else "cloud_path"
            octomap_key = "filtered_octomap_path" if filtered else "octomap_path"
            if not record[loadable_key] or not record[cloud_key]:
                converted, message = self._convert_navigation_map(record, filtered)
                if not converted:
                    return False, message
                operation_messages.append(message)
                record = next(
                    (item for item in self.navigation_maps() if item["id"] == map_id),
                    None,
                )
                if record is None or not record[loadable_key]:
                    return False, f"map {map_id} is still missing its exported .bt file"

            hloc_warning = ""
            if not record.get("hloc_map_directory"):
                hloc_warning = (
                    "HLoc: index not built; map display is available, "
                    "automatic localization remains disabled"
                )

            self._clear_navigation_preview()
            variant = "filtered" if filtered else "original"
            cloud_error = self._load_navigation_cloud(
                map_id, record.get(cloud_key), variant
            )
            voxel_error = self._load_navigation_voxels(
                map_id, record[octomap_key], variant
            )
            terrain_error = self._load_navigation_terrain(
                map_id, record[octomap_key], record.get(cloud_key), variant
            )
            preview_errors = []
            for label, error in (("colored cloud", cloud_error),
                                 ("OctoMap voxels", voxel_error),
                                 ("terrain costmap", terrain_error)):
                if error:
                    preview_errors.append(label + ": " + error)
            details = operation_messages + preview_errors
            if hloc_warning:
                details.append(hloc_warning)
            return True, f"map {map_id} {variant} layers loaded" + (
                "; " + "; ".join(details) if details else "")
        finally:
            self._navigation_map_operation_lock.release()

    def start_navigation_localization(self, map_id: str) -> Tuple[bool, str]:
        """Start HLoc coarse localization followed by ICP refinement."""
        calibration = self.imu_calibration_status()
        if (calibration["service_available"] or calibration["standalone_available"]) and (
            calibration.get("state") != "calibrated" or calibration["busy"]
        ):
            return False, "请先在水平面完成 IMU 一键校准"
        if not MAP_IDENTIFIER.fullmatch(map_id):
            return False, "map_id must use the mapNNN format"
        record = next(
            (item for item in self.navigation_maps() if item["id"] == map_id),
            None,
        )
        if record is None:
            return False, f"map {map_id} does not exist under {self.maps_root}"
        with self._navigation_lock:
            loaded_variant = self._navigation_cloud_variant
            loaded_layers_match = (
                self._navigation_cloud_map_id == map_id
                and self._navigation_voxel_map_id == map_id
                and self._navigation_voxel_variant == loaded_variant
            )
        if not loaded_layers_match or loaded_variant not in ("original", "filtered"):
            return False, f"load map {map_id} before starting localization"
        filtered = loaded_variant == "filtered"
        octomap_key = "filtered_octomap_path" if filtered else "octomap_path"
        cloud_key = "filtered_cloud_path" if filtered else "cloud_path"
        if not record.get("hloc_map_directory"):
            if self.mapping_status()["state"] == "running":
                return False, "请先停止建图，再构建定位索引"
            if not self._navigation_map_operation_lock.acquire(blocking=False):
                return False, "another map is already being converted or prepared"
            try:
                hloc_directory = (
                    self.maps_root / "hloc_maps" / map_id
                ).resolve()
                built, message = self.hloc_index_builder.build(
                    map_id,
                    Path(record["database_path"]),
                    hloc_directory,
                )
                if not built:
                    return False, message
                record = next(
                    (
                        item for item in self.navigation_maps()
                        if item["id"] == map_id
                    ),
                    None,
                )
                if record is None or not record.get("hloc_map_directory"):
                    return False, (
                        f"map {map_id} HLoc builder produced no usable index"
                    )
            finally:
                self._navigation_map_operation_lock.release()
        required = ("database_path", octomap_key, cloud_key, "hloc_map_directory")
        if any(not record.get(name) for name in required):
            return False, (
                f"map {map_id} needs display layers and a built HLoc index first"
            )
        with self._navigation_lock:
            if (
                self._navigation_cloud_map_id != map_id
                or self._navigation_voxel_map_id != map_id
                or self._navigation_cloud_variant != loaded_variant
                or self._navigation_voxel_variant != loaded_variant
            ):
                return False, f"load map {map_id} before starting localization"
            self._localization_ready = False
            self._localization_variance = None
            self._localization_received_at = None
            self._coarse_localization_pose = None
            self._coarse_gate_ready = False
            self._refined_localization_pose = None
            self._refined_localization_received_at = None
            self._terrain_pose = None
            self._terrain_pose_received_at = None
            self._refined_localization_fitness = None
            self._refined_localization_status = ""
            self._refined_localization_verified_at = None
            self._planned_path_points = []
            self._path_received_at = None
            self._planning_state = "idle"
            self._planning_error = ""
        return self.navigation.start(
            map_id,
            Path(record["database_path"]),
            Path(record[octomap_key]),
            Path(record[cloud_key]),
            Path(record["hloc_map_directory"]),
            self.semantic_annotation_store.output_root / map_id / "annotations.json",
        )

    def stop_navigation(self) -> Tuple[bool, str]:
        """Stop localization/planning while retaining the currently loaded map."""
        self.halt_navigation_motion()
        stopped, message = self.navigation.stop()
        if stopped:
            with self._navigation_lock:
                self._localization_ready = False
                self._localization_variance = None
                self._localization_received_at = None
                self._coarse_localization_pose = None
                self._coarse_gate_ready = False
                self._refined_localization_pose = None
                self._refined_localization_received_at = None
                self._terrain_pose = None
                self._terrain_pose_received_at = None
                self._refined_localization_fitness = None
                self._refined_localization_status = ""
                self._refined_localization_verified_at = None
                self._hloc_status = ""
                self._hloc_status_received_at = None
                self._hloc_diagnostics = None
                self._hloc_diagnostics_received_at = None
                self._navigation_active = False
                self._navigation_follower_state = "stopped"
        return stopped, message

    def start_navigation_motion(self) -> Tuple[bool, str]:
        """Start low-speed path following only after localization and planning."""
        navigation = self.navigation_status()
        robot_control = self.d1_control_status()
        with self._lock:
            estop_active = self._estop_active
        if estop_active:
            return False, "release emergency stop before starting navigation"
        if robot_control["enabled"] and not robot_control["control_ready"]:
            return False, "请先开启机器人控制并等待状态变为站立且可控制"
        if navigation["state"] != "running":
            return False, "start map localization before navigation"
        if not navigation["planning_localization_ready"]:
            return False, "wait for a recent accepted ICP localization before starting"
        if not navigation["path_ready"]:
            return False, "select a reachable goal and wait for a valid path"
        if navigation["active"]:
            return False, "navigation is already active"
        message = Bool()
        message.data = True
        self.navigation_start_publisher.publish(message)
        with self._navigation_lock:
            self._navigation_follower_state = "starting"
        return True, "navigation start command sent"

    def halt_navigation_motion(self) -> None:
        """Stop path following while leaving localization and the path loaded."""
        message = Bool()
        message.data = True
        self.navigation_stop_publisher.publish(message)
        with self._navigation_lock:
            self._navigation_active = False
            self._navigation_follower_state = "stopped"

    def set_navigation_goal(self, x: float, y: float, z: float) -> Tuple[bool, str]:
        """Publish a map-frame goal only after HLoc and ICP localization is ready."""
        navigation = self.navigation_status()
        if navigation["state"] != "running":
            return False, "load a map and wait for navigation to start first"
        if not navigation["planning_localization_ready"]:
            return False, "wait for a recent accepted ICP localization before selecting a goal"
        self.halt_navigation_motion()
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        with self._navigation_lock:
            goal.header.frame_id = self._voxel_frame_id or "map"
            self._planned_path_points = []
            self._path_received_at = None
            self._planning_state = "pending"
            self._planning_error = ""
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
            status["map_variant"] = (
                self._navigation_cloud_variant
                if self._navigation_cloud_variant == self._navigation_voxel_variant
                else None
            )
            coarse_age = None if self._localization_received_at is None else round(
                time.monotonic() - self._localization_received_at, 2)
            refined_age = (
                None if self._refined_localization_received_at is None else round(
                    time.monotonic() - self._refined_localization_received_at, 2
                )
            )
            terrain_pose_age = (
                None if self._terrain_pose_received_at is None else round(
                    time.monotonic() - self._terrain_pose_received_at, 2
                )
            )
            icp_verification_age = (
                None if self._refined_localization_verified_at is None else round(
                    time.monotonic() - self._refined_localization_verified_at, 2
                )
            )
            coarse_ready = (
                self._coarse_gate_ready
                and self._localization_ready
                and coarse_age is not None
                and coarse_age <= self.navigation_localization_timeout
            )
            refined_ready = (
                self._refined_localization_pose is not None
                and refined_age is not None
                and refined_age <= self.navigation_localization_timeout
            )
            terrain_pose_ready = (
                self._terrain_pose is not None
                and terrain_pose_age is not None
                and terrain_pose_age <= self.navigation_localization_timeout
            )
            icp_verified = (
                icp_verification_age is not None
                and icp_verification_age <= self.navigation_icp_verification_timeout
            )
            running = status["state"] == "running"
            status["path_ready"] = running and bool(
                self._planned_path_points
            )
            status["path_point_count"] = len(self._planned_path_points)
            status["planning_state"] = self._planning_state
            status["planning_error"] = self._planning_error or None
            status["active"] = running and self._navigation_active
            status["follower_state"] = (
                self._navigation_follower_state if running else "stopped"
            )
            status["coarse_localization_ready"] = running and coarse_ready
            status["coarse_consistency_ready"] = (
                running and self._coarse_gate_ready
            )
            status["refined_localization_ready"] = running and refined_ready
            status["localization_ready"] = running and refined_ready
            status["planning_localization_ready"] = (
                running and refined_ready and icp_verified
            )
            status["icp_verified"] = running and icp_verified
            status["icp_verification_age_seconds"] = icp_verification_age
            status["localization_variance"] = self._localization_variance
            status["localization_age_seconds"] = refined_age
            status["terrain_pose_age_seconds"] = terrain_pose_age
            status["coarse_localization_age_seconds"] = coarse_age
            status["localization_fitness"] = self._refined_localization_fitness
            status["localization_status"] = self._refined_localization_status or None
            hloc_status_age = (
                None if self._hloc_status_received_at is None else round(
                    time.monotonic() - self._hloc_status_received_at, 2
                )
            )
            hloc_diagnostics_age = (
                None if self._hloc_diagnostics_received_at is None else round(
                    time.monotonic() - self._hloc_diagnostics_received_at, 2
                )
            )
            status["hloc_status"] = self._hloc_status or None
            status["hloc_status_age_seconds"] = hloc_status_age
            status["hloc_diagnostics"] = (
                dict(self._hloc_diagnostics) if self._hloc_diagnostics else None
            )
            status["hloc_diagnostics_age_seconds"] = hloc_diagnostics_age
            if running and refined_ready:
                pose = dict(self._refined_localization_pose)
                if terrain_pose_ready:
                    pose.update(self._terrain_pose)
                    pose["localization_z"] = self._refined_localization_pose["z"]
                    pose["source"] = "terrain_constrained"
                else:
                    pose["source"] = "icp"
                status["localization_stage"] = "localized"
                status["pose"] = pose
            elif running and coarse_ready and self._coarse_localization_pose:
                status["localization_stage"] = "refining"
                status["pose"] = {
                    **self._coarse_localization_pose,
                    "source": "hloc",
                }
            elif running:
                status["localization_stage"] = "searching"
                status["pose"] = None
            else:
                status["localization_stage"] = "stopped"
                status["pose"] = None
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
            control_session_active = bool(self._control_owner_id)
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
            "robot_namespace": self.robot_namespace,
            "state": state,
            "cmd_vel_topic": self.cmd_vel_topic,
            "http_port": self.http_port,
            "access_urls": self.access_urls,
            "output_enabled": self.output_enabled,
            "estop_active": estop_active,
            "timed_out": timed_out,
            "command_age": age,
            "command_timeout": self.command_timeout,
            "control_session_active": control_session_active,
            "command": command.as_dict(),
            "limits": self.limits.as_dict(),
            "subscriber_count": self.publisher.get_subscription_count(),
            "robot_control": self.d1_control_status(),
            "imu_calibration": self.imu_calibration_status(),
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
            self._stop_standalone_imu_calibration()
            self.stop_mapping()
            self.stop_navigation()
            if rclpy.ok():
                for _ in range(3):
                    self.stop_motion()
                    time.sleep(0.02)
        finally:
            for server in self.http_servers:
                server.shutdown()
                server.server_close()
            for thread in self._http_threads:
                if thread.is_alive():
                    thread.join(timeout=1.0)


def main(args: Optional[list] = None) -> None:
    """Run the web control ROS node."""
    # Keep ROS alive during Ctrl+C so close() can publish zero first.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    # ros2 launch children inherit SIGINT as ignored. Restore Python's handler
    # so automatic port takeover can run close() instead of waiting forever.
    signal.signal(signal.SIGINT, signal.default_int_handler)
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
