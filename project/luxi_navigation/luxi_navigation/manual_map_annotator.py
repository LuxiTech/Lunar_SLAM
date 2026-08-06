#!/usr/bin/env python3
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

"""Annotate polygon obstacles by clicking points in RViz."""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from pathlib import Path
import re
import tempfile
from typing import Optional

from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
import yaml


ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")


def _default_output_path() -> str:
    configured = os.environ.get("LUXI_WORKSPACE_ROOT", "").strip()
    if configured:
        workspace = Path(configured).expanduser().resolve()
    else:
        workspace = next(
            candidate
            for candidate in Path(__file__).resolve().parents
            if (candidate / "project").is_dir() and (candidate / "maps").is_dir()
        )
    return str(workspace / "maps" / "occupancy_maps" / "semantic_obstacles.yaml")


@dataclass
class Obstacle:
    """A named polygon obstacle stored in map coordinates."""

    obstacle_id: str
    label: str
    polygon: list[tuple[float, float]] = field(default_factory=list)


def polygon_area(polygon: list[tuple[float, float]]) -> float:
    """Return the absolute area of a two-dimensional polygon."""
    if len(polygon) < 3:
        return 0.0
    twice_area = 0.0
    for index, point in enumerate(polygon):
        next_point = polygon[(index + 1) % len(polygon)]
        twice_area += point[0] * next_point[1] - next_point[0] * point[1]
    return abs(twice_area) * 0.5


def point_in_polygon(
    x_value: float, y_value: float, polygon: list[tuple[float, float]]
) -> bool:
    """Return whether a point lies inside or on the edge of a polygon."""
    inside = False
    for index, first in enumerate(polygon):
        second = polygon[(index + 1) % len(polygon)]
        delta_x = second[0] - first[0]
        delta_y = second[1] - first[1]
        cross = (x_value - first[0]) * delta_y - (y_value - first[1]) * delta_x
        if abs(cross) <= 1.0e-9:
            dot = (x_value - first[0]) * (x_value - second[0]) + (
                y_value - first[1]
            ) * (y_value - second[1])
            if dot <= 1.0e-9:
                return True

        intersects = (first[1] > y_value) != (second[1] > y_value)
        if intersects:
            boundary_x = delta_x * (y_value - first[1]) / delta_y + first[0]
            if x_value < boundary_x:
                inside = not inside
    return inside


def quaternion_yaw(grid: OccupancyGrid) -> float:
    """Extract the yaw component from an occupancy grid origin."""
    orientation = grid.info.origin.orientation
    numerator = 2.0 * (
        orientation.w * orientation.z + orientation.x * orientation.y
    )
    denominator = 1.0 - 2.0 * (
        orientation.y * orientation.y + orientation.z * orientation.z
    )
    return math.atan2(numerator, denominator)


def polygon_in_grid_coordinates(
    grid: OccupancyGrid, polygon: list[tuple[float, float]]
) -> list[tuple[float, float]]:
    """Transform map-coordinate vertices into continuous grid coordinates."""
    yaw = quaternion_yaw(grid)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    origin = grid.info.origin.position
    resolution = float(grid.info.resolution)
    transformed = []
    for x_value, y_value in polygon:
        delta_x = x_value - origin.x
        delta_y = y_value - origin.y
        local_x = cosine * delta_x + sine * delta_y
        local_y = -sine * delta_x + cosine * delta_y
        transformed.append((local_x / resolution, local_y / resolution))
    return transformed


def rasterize_obstacles(
    source_map: OccupancyGrid, obstacles: list[Obstacle]
) -> OccupancyGrid:
    """Rasterize polygon obstacles into a map-aligned binary occupancy mask."""
    width = int(source_map.info.width)
    height = int(source_map.info.height)
    if width <= 0 or height <= 0 or source_map.info.resolution <= 0.0:
        raise ValueError("source map must have positive dimensions and resolution")

    mask = OccupancyGrid()
    mask.header = source_map.header
    mask.info = source_map.info
    mask.data = [0] * (width * height)

    for obstacle in obstacles:
        polygon = polygon_in_grid_coordinates(source_map, obstacle.polygon)
        minimum_x = max(0, math.floor(min(point[0] for point in polygon)))
        maximum_x = min(width - 1, math.ceil(max(point[0] for point in polygon)))
        minimum_y = max(0, math.floor(min(point[1] for point in polygon)))
        maximum_y = min(height - 1, math.ceil(max(point[1] for point in polygon)))
        for grid_y in range(minimum_y, maximum_y + 1):
            for grid_x in range(minimum_x, maximum_x + 1):
                if point_in_polygon(grid_x + 0.5, grid_y + 0.5, polygon):
                    mask.data[grid_y * width + grid_x] = 100
    return mask


def annotation_document(
    map_id: str, frame_id: str, obstacles: list[Obstacle]
) -> dict:
    """Create the stable YAML representation of all annotations."""
    return {
        "schema_version": 1,
        "map_id": map_id,
        "frame_id": frame_id,
        "obstacles": [
            {
                "id": obstacle.obstacle_id,
                "label": obstacle.label,
                "type": "obstacle",
                "polygon": [
                    [round(point[0], 6), round(point[1], 6)]
                    for point in obstacle.polygon
                ],
            }
            for obstacle in obstacles
        ],
    }


def save_annotation_file(
    output_path: Path, map_id: str, frame_id: str, obstacles: list[Obstacle]
) -> None:
    """Atomically write the annotation YAML file."""
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path: Optional[str] = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            dir=output_path.parent,
            prefix=f".{output_path.name}.",
            suffix=".tmp",
            delete=False,
        ) as temporary_file:
            temporary_path = temporary_file.name
            yaml.safe_dump(
                annotation_document(map_id, frame_id, obstacles),
                temporary_file,
                allow_unicode=True,
                sort_keys=False,
            )
            temporary_file.flush()
            os.fsync(temporary_file.fileno())
        os.replace(temporary_path, output_path)
    finally:
        if temporary_path is not None and os.path.exists(temporary_path):
            os.unlink(temporary_path)


def load_annotation_file(
    input_path: Path, expected_frame: str
) -> tuple[str, list[Obstacle]]:
    """Load and validate an annotation YAML file."""
    if not input_path.exists():
        return "", []
    with input_path.open("r", encoding="utf-8") as input_file:
        document = yaml.safe_load(input_file) or {}
    if document.get("schema_version") != 1:
        raise ValueError("unsupported or missing schema_version")
    frame_id = str(document.get("frame_id", ""))
    if frame_id != expected_frame:
        raise ValueError(
            f"annotation frame '{frame_id}' does not match '{expected_frame}'"
        )

    obstacles = []
    seen_ids = set()
    for item in document.get("obstacles", []):
        obstacle_id = str(item.get("id", ""))
        if not ID_PATTERN.fullmatch(obstacle_id) or obstacle_id in seen_ids:
            raise ValueError(f"invalid or duplicate obstacle id '{obstacle_id}'")
        polygon = []
        for point in item.get("polygon", []):
            if not isinstance(point, list) or len(point) != 2:
                raise ValueError(f"invalid polygon point in '{obstacle_id}'")
            x_value, y_value = float(point[0]), float(point[1])
            if not math.isfinite(x_value) or not math.isfinite(y_value):
                raise ValueError(f"non-finite polygon point in '{obstacle_id}'")
            polygon.append((x_value, y_value))
        if len(polygon) < 3 or polygon_area(polygon) <= 0.0:
            raise ValueError(f"obstacle '{obstacle_id}' has an invalid polygon")
        seen_ids.add(obstacle_id)
        obstacles.append(
            Obstacle(obstacle_id, str(item.get("label", obstacle_id)), polygon)
        )
    return str(document.get("map_id", "")), obstacles


class ManualMapAnnotator(Node):
    """Manage RViz-clicked obstacle polygons and publish semantic products."""

    def __init__(
        self, parameter_overrides: Optional[list[Parameter]] = None
    ) -> None:
        super().__init__(
            "manual_map_annotator",
            parameter_overrides=parameter_overrides or [],
        )
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("map_id", "")
        self.declare_parameter(
            "output_path",
            _default_output_path(),
        )
        self.declare_parameter("map_topic", "/rtabmap/map")
        self.declare_parameter("clicked_point_topic", "/clicked_point")
        self.declare_parameter("markers_topic", "/semantic_annotation/markers")
        self.declare_parameter("obstacle_mask_topic", "/semantic_annotation/obstacle_mask")
        self.declare_parameter("active_id", "")
        self.declare_parameter("active_label", "")
        self.declare_parameter("autosave", True)
        self.declare_parameter("minimum_polygon_area", 0.01)
        self.declare_parameter("minimum_vertex_spacing", 0.02)
        self.declare_parameter("map_transient_local", False)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.map_id = str(self.get_parameter("map_id").value)
        self.output_path = Path(
            os.path.abspath(
                os.path.expanduser(str(self.get_parameter("output_path").value))
            )
        )
        self.minimum_polygon_area = float(
            self.get_parameter("minimum_polygon_area").value
        )
        self.minimum_vertex_spacing = float(
            self.get_parameter("minimum_vertex_spacing").value
        )
        self.autosave = bool(self.get_parameter("autosave").value)
        if not self.frame_id:
            raise ValueError("frame_id cannot be empty")
        if self.minimum_polygon_area <= 0.0 or self.minimum_vertex_spacing < 0.0:
            raise ValueError("polygon area must be positive and spacing non-negative")

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=(
                DurabilityPolicy.TRANSIENT_LOCAL
                if bool(self.get_parameter("map_transient_local").value)
                else DurabilityPolicy.VOLATILE
            ),
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, str(self.get_parameter("markers_topic").value), latched_qos
        )
        self.mask_publisher = self.create_publisher(
            OccupancyGrid,
            str(self.get_parameter("obstacle_mask_topic").value),
            latched_qos,
        )
        self.create_subscription(
            PointStamped,
            str(self.get_parameter("clicked_point_topic").value),
            self._clicked_point,
            10,
        )
        self.create_subscription(
            OccupancyGrid,
            str(self.get_parameter("map_topic").value),
            self._map_callback,
            map_qos,
        )

        self.create_service(Trigger, "~/start_polygon", self._start_polygon)
        self.create_service(Trigger, "~/finish_polygon", self._finish_polygon)
        self.create_service(Trigger, "~/undo_vertex", self._undo_vertex)
        self.create_service(Trigger, "~/cancel_polygon", self._cancel_polygon)
        self.create_service(Trigger, "~/delete_obstacle", self._delete_obstacle)
        self.create_service(Trigger, "~/save", self._save_service)
        self.create_service(Trigger, "~/reload", self._reload_service)
        self.create_service(Trigger, "~/list", self._list_service)

        self.obstacles: list[Obstacle] = []
        self.draft: Optional[Obstacle] = None
        self.source_map: Optional[OccupancyGrid] = None
        self._load_from_disk()
        self.publish_markers()
        self.get_logger().info(
            f"Manual annotator ready: frame={self.frame_id} "
            f"map_topic={self.get_parameter('map_topic').value} "
            f"output={self.output_path} obstacles={len(self.obstacles)}"
        )

    def _next_obstacle_id(self) -> str:
        existing = {obstacle.obstacle_id for obstacle in self.obstacles}
        index = 1
        while f"obstacle_{index:03d}" in existing:
            index += 1
        return f"obstacle_{index:03d}"

    def _start_polygon(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if self.draft is not None:
            response.success = False
            response.message = "A polygon draft is already active."
            return response
        configured_id = str(self.get_parameter("active_id").value).strip()
        obstacle_id = configured_id or self._next_obstacle_id()
        if not ID_PATTERN.fullmatch(obstacle_id):
            response.success = False
            response.message = "active_id must use letters, digits, '.', '_' or '-'."
            return response
        if any(item.obstacle_id == obstacle_id for item in self.obstacles):
            response.success = False
            response.message = f"Obstacle id '{obstacle_id}' already exists."
            return response
        configured_label = str(self.get_parameter("active_label").value).strip()
        self.draft = Obstacle(obstacle_id, configured_label or obstacle_id)
        self.publish_markers()
        response.success = True
        response.message = f"Started '{obstacle_id}'. Click at least three vertices."
        return response

    def _finish_polygon(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if self.draft is None:
            response.success = False
            response.message = "No active polygon draft."
            return response
        if len(self.draft.polygon) < 3:
            response.success = False
            response.message = "A polygon requires at least three vertices."
            return response
        area = polygon_area(self.draft.polygon)
        if area < self.minimum_polygon_area:
            response.success = False
            response.message = (
                f"Polygon area {area:.4f} m^2 is below "
                f"{self.minimum_polygon_area:.4f} m^2."
            )
            return response

        completed = self.draft
        self.obstacles.append(completed)
        self.draft = None
        if self.autosave:
            try:
                self._save_to_disk()
            except (OSError, ValueError, yaml.YAMLError) as error:
                self.obstacles.pop()
                self.draft = completed
                self.publish_markers()
                response.success = False
                response.message = f"Autosave failed; draft was preserved: {error}"
                return response
        self.publish_products()
        response.success = True
        response.message = (
            f"Saved '{completed.obstacle_id}' with "
            f"{len(completed.polygon)} vertices and area {area:.3f} m^2."
        )
        return response

    def _undo_vertex(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if self.draft is None or not self.draft.polygon:
            response.success = False
            response.message = "The active draft has no vertex to undo."
            return response
        removed = self.draft.polygon.pop()
        self.publish_markers()
        response.success = True
        response.message = f"Removed vertex ({removed[0]:.3f}, {removed[1]:.3f})."
        return response

    def _cancel_polygon(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if self.draft is None:
            response.success = False
            response.message = "No active polygon draft."
            return response
        obstacle_id = self.draft.obstacle_id
        self.draft = None
        self.publish_markers()
        response.success = True
        response.message = f"Cancelled draft '{obstacle_id}'."
        return response

    def _delete_obstacle(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        obstacle_id = str(self.get_parameter("active_id").value).strip()
        for index, obstacle in enumerate(self.obstacles):
            if obstacle.obstacle_id == obstacle_id:
                self.obstacles.pop(index)
                if self.autosave:
                    try:
                        self._save_to_disk()
                    except (OSError, ValueError, yaml.YAMLError) as error:
                        self.obstacles.insert(index, obstacle)
                        response.success = False
                        response.message = (
                            f"Delete autosave failed; obstacle was preserved: {error}"
                        )
                        return response
                self.publish_products()
                response.success = True
                response.message = f"Deleted obstacle '{obstacle_id}'."
                return response
        response.success = False
        response.message = f"Obstacle '{obstacle_id}' does not exist."
        return response

    def _save_service(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        try:
            self._save_to_disk()
        except (OSError, ValueError, yaml.YAMLError) as error:
            response.success = False
            response.message = f"Save failed: {error}"
            return response
        response.success = True
        response.message = f"Saved {len(self.obstacles)} obstacles to {self.output_path}."
        return response

    def _reload_service(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        if self.draft is not None:
            response.success = False
            response.message = "Cancel or finish the active draft before reloading."
            return response
        try:
            self._load_from_disk()
        except (OSError, ValueError, yaml.YAMLError) as error:
            response.success = False
            response.message = f"Reload failed: {error}"
            return response
        self.publish_products()
        response.success = True
        response.message = f"Reloaded {len(self.obstacles)} obstacles."
        return response

    def _list_service(
        self, _request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        response.success = True
        response.message = ", ".join(
            obstacle.obstacle_id for obstacle in self.obstacles
        ) or "No saved obstacles."
        return response

    def _clicked_point(self, message: PointStamped) -> None:
        if self.draft is None:
            self.get_logger().warn(
                "Ignoring clicked point: call ~/start_polygon first."
            )
            return
        if message.header.frame_id != self.frame_id:
            self.get_logger().error(
                f"Ignoring point in frame '{message.header.frame_id}'; "
                f"expected '{self.frame_id}'."
            )
            return
        point = (float(message.point.x), float(message.point.y))
        if not math.isfinite(point[0]) or not math.isfinite(point[1]):
            self.get_logger().error("Ignoring non-finite clicked point.")
            return
        if self.draft.polygon:
            previous = self.draft.polygon[-1]
            spacing = math.hypot(point[0] - previous[0], point[1] - previous[1])
            if spacing < self.minimum_vertex_spacing:
                self.get_logger().warn(
                    f"Ignoring vertex only {spacing:.3f} m from the previous one."
                )
                return
        self.draft.polygon.append(point)
        self.publish_markers()
        self.get_logger().info(
            f"Added vertex {len(self.draft.polygon)} to '{self.draft.obstacle_id}': "
            f"({point[0]:.3f}, {point[1]:.3f})"
        )

    def _map_callback(self, message: OccupancyGrid) -> None:
        if message.header.frame_id != self.frame_id:
            self.get_logger().error(
                f"Ignoring map in frame '{message.header.frame_id}'; "
                f"expected '{self.frame_id}'."
            )
            return
        self.source_map = message
        self.publish_mask()

    def _load_from_disk(self) -> None:
        loaded_map_id, obstacles = load_annotation_file(
            self.output_path, self.frame_id
        )
        if self.map_id and loaded_map_id and loaded_map_id != self.map_id:
            raise ValueError(
                f"annotation map_id '{loaded_map_id}' does not match '{self.map_id}'"
            )
        if not self.map_id:
            self.map_id = loaded_map_id
        self.obstacles = obstacles

    def _save_to_disk(self) -> None:
        save_annotation_file(
            self.output_path, self.map_id, self.frame_id, self.obstacles
        )

    def publish_products(self) -> None:
        """Publish all visual and grid representations."""
        self.publish_markers()
        self.publish_mask()

    def publish_mask(self) -> None:
        """Publish a binary obstacle mask when a source map is available."""
        if self.source_map is None:
            return
        mask = rasterize_obstacles(self.source_map, self.obstacles)
        mask.header.stamp = self.get_clock().now().to_msg()
        self.mask_publisher.publish(mask)

    def publish_markers(self) -> None:
        """Publish saved obstacles and the active polygon draft."""
        timestamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self.frame_id
        clear.header.stamp = timestamp
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for index, obstacle in enumerate(self.obstacles):
            markers.markers.extend(
                self._obstacle_markers(
                    obstacle, index * 3 + 1, timestamp, (1.0, 0.1, 0.1, 0.9), True
                )
            )
        if self.draft is not None:
            markers.markers.extend(
                self._obstacle_markers(
                    self.draft,
                    100000,
                    timestamp,
                    (1.0, 0.8, 0.0, 1.0),
                    False,
                )
            )
        self.marker_publisher.publish(markers)

    def _obstacle_markers(
        self,
        obstacle: Obstacle,
        marker_id: int,
        timestamp,
        color: tuple[float, float, float, float],
        close_polygon: bool,
    ) -> list[Marker]:
        line = Marker()
        line.header.frame_id = self.frame_id
        line.header.stamp = timestamp
        line.ns = "semantic_obstacles"
        line.id = marker_id
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0
        line.scale.x = 0.04
        line.color.r, line.color.g, line.color.b, line.color.a = color
        line.points = [Point(x=point[0], y=point[1], z=0.05) for point in obstacle.polygon]
        if close_polygon and line.points:
            line.points.append(line.points[0])

        vertices = Marker()
        vertices.header = line.header
        vertices.ns = line.ns
        vertices.id = marker_id + 1
        vertices.type = Marker.SPHERE_LIST
        vertices.action = Marker.ADD
        vertices.pose.orientation.w = 1.0
        vertices.scale.x = 0.09
        vertices.scale.y = 0.09
        vertices.scale.z = 0.09
        vertices.color.r, vertices.color.g, vertices.color.b, vertices.color.a = color
        vertices.points = list(line.points[:-1] if close_polygon and line.points else line.points)

        text = Marker()
        text.header = line.header
        text.ns = line.ns
        text.id = marker_id + 2
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.orientation.w = 1.0
        if obstacle.polygon:
            text.pose.position.x = sum(point[0] for point in obstacle.polygon) / len(
                obstacle.polygon
            )
            text.pose.position.y = sum(point[1] for point in obstacle.polygon) / len(
                obstacle.polygon
            )
        text.pose.position.z = 0.2
        text.scale.z = 0.18
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = obstacle.label
        return [line, vertices, text]


def main(args: Optional[list[str]] = None) -> None:
    """Run the manual map annotation node."""
    rclpy.init(args=args)
    node = ManualMapAnnotator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == "__main__":
    main()
