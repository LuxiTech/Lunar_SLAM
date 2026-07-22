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

"""Unit tests for semantic obstacle geometry and persistence."""

from nav_msgs.msg import OccupancyGrid
import pytest

from luxi_navigation.manual_map_annotator import (
    Obstacle,
    load_annotation_file,
    point_in_polygon,
    polygon_area,
    rasterize_obstacles,
    save_annotation_file,
)


def make_grid() -> OccupancyGrid:
    grid = OccupancyGrid()
    grid.header.frame_id = "map"
    grid.info.width = 5
    grid.info.height = 5
    grid.info.resolution = 1.0
    grid.info.origin.orientation.w = 1.0
    grid.data = [0] * 25
    return grid


def test_polygon_geometry():
    square = [(1.0, 1.0), (3.0, 1.0), (3.0, 3.0), (1.0, 3.0)]
    assert polygon_area(square) == pytest.approx(4.0)
    assert point_in_polygon(2.0, 2.0, square)
    assert point_in_polygon(1.0, 2.0, square)
    assert not point_in_polygon(4.0, 2.0, square)


def test_obstacle_rasterization():
    obstacle = Obstacle(
        "box",
        "box",
        [(1.0, 1.0), (3.0, 1.0), (3.0, 3.0), (1.0, 3.0)],
    )
    mask = rasterize_obstacles(make_grid(), [obstacle])
    occupied = [index for index, value in enumerate(mask.data) if value == 100]
    assert occupied == [6, 7, 11, 12]


def test_yaml_round_trip(tmp_path):
    path = tmp_path / "semantic_obstacles.yaml"
    obstacle = Obstacle(
        "rock_001",
        "岩石",
        [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)],
    )
    save_annotation_file(path, "map003", "map", [obstacle])
    map_id, loaded = load_annotation_file(path, "map")
    assert map_id == "map003"
    assert loaded == [obstacle]
    assert not list(tmp_path.glob("*.tmp"))
