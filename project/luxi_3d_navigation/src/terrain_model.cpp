#include "luxi_3d_navigation/terrain_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>
#include <unordered_set>

namespace luxi_3d_navigation
{

TerrainModel::TerrainModel(
  const octomap::OcTree & tree, TerrainParameters parameters,
  std::vector<Polygon2D> pit_polygons, std::optional<TerrainObservation> observation)
: tree_(tree), parameters_(parameters), pit_polygons_(std::move(pit_polygons)),
  observation_(std::move(observation))
{
  double min_x;
  double min_y;
  double min_z;
  double max_x;
  double max_y;
  double max_z;
  tree_.getMetricMin(min_x, min_y, min_z);
  tree_.getMetricMax(max_x, max_y, max_z);
  const double r = resolution();
  min_x_ = static_cast<int>(std::floor(min_x / r)) - 1;
  min_y_ = static_cast<int>(std::floor(min_y / r)) - 1;
  min_z_ = static_cast<int>(std::floor(min_z / r)) - 1;
  max_x_ = static_cast<int>(std::floor(max_x / r)) + 1;
  max_y_ = static_cast<int>(std::floor(max_y / r)) + 1;
  max_z_ = static_cast<int>(std::floor(max_z / r)) + 1;
  buildLayers();
}

double TerrainModel::resolution() const
{
  return tree_.getResolution();
}

GridCell3D TerrainModel::worldToGrid(double x, double y, double z) const
{
  const double r = resolution();
  return GridCell3D{
    static_cast<int>(std::floor(x / r)), static_cast<int>(std::floor(y / r)),
    static_cast<int>(std::floor(z / r))};
}

octomap::point3d TerrainModel::gridToWorld(const GridCell3D & cell) const
{
  const double r = resolution();
  return octomap::point3d(
    static_cast<float>((static_cast<double>(cell.x) + 0.5) * r),
    static_cast<float>((static_cast<double>(cell.y) + 0.5) * r),
    static_cast<float>((static_cast<double>(cell.z) + 0.5) * r));
}

bool TerrainModel::inside(const GridCell3D & cell) const
{
  return cell.x >= min_x_ && cell.x <= max_x_ && cell.y >= min_y_ && cell.y <= max_y_ &&
         cell.z >= min_z_ && cell.z <= max_z_;
}

bool TerrainModel::occupied(const GridCell3D & cell) const
{
  if (!inside(cell)) {
    return false;
  }
  const auto point = gridToWorld(cell);
  const auto * node = tree_.search(point);
  return node != nullptr && tree_.isNodeOccupied(node);
}

bool TerrainModel::supportOccupied(const GridCell3D & cell) const
{
  if (observation_) {
    return observation_->ground_cells.find(cell) != observation_->ground_cells.end();
  }
  return occupied(cell);
}

bool TerrainModel::collisionOccupied(const GridCell3D & cell) const
{
  if (observation_) {
    return observation_->obstacle_cells.find(cell) != observation_->obstacle_cells.end();
  }
  return occupied(cell);
}

bool TerrainModel::supported(const GridCell3D & cell) const
{
  const int xy_radius = parameters_.strict_direct_support ? 0 :
    std::max(0, parameters_.support_xy_radius_cells);
  const int depth = parameters_.strict_direct_support ? 1 :
    std::max(1, parameters_.support_depth_cells);
  for (int dz = 1; dz <= depth; ++dz) {
    for (int dy = -xy_radius; dy <= xy_radius; ++dy) {
      for (int dx = -xy_radius; dx <= xy_radius; ++dx) {
        if (supportOccupied(GridCell3D{cell.x + dx, cell.y + dy, cell.z - dz})) {
          return true;
        }
      }
    }
  }
  return false;
}

bool TerrainModel::pointInPolygon(double x, double y, const Polygon2D & polygon)
{
  bool inside_polygon = false;
  if (polygon.size() < 3U) {
    return false;
  }
  for (std::size_t i = 0U, j = polygon.size() - 1U; i < polygon.size(); j = i++) {
    const auto & a = polygon[i];
    const auto & b = polygon[j];
    const bool crosses = ((a.second > y) != (b.second > y)) &&
      (x < (b.first - a.first) * (y - a.second) / (b.second - a.second) + a.first);
    if (crosses) {
      inside_polygon = !inside_polygon;
    }
  }
  return inside_polygon;
}

bool TerrainModel::inPitFootprint(const GridCell3D & cell) const
{
  if (pit_polygons_.empty()) {
    return false;
  }
  const auto center = gridToWorld(cell);
  const int radius_cells = std::max(
    0, static_cast<int>(std::ceil(parameters_.robot_radius / resolution())));
  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const double offset_m = std::hypot(
        static_cast<double>(dx), static_cast<double>(dy)) * resolution();
      if (offset_m > parameters_.robot_radius + 1e-9) {
        continue;
      }
      const double x = center.x() + static_cast<double>(dx) * resolution();
      const double y = center.y() + static_cast<double>(dy) * resolution();
      for (const auto & polygon : pit_polygons_) {
        if (pointInPolygon(x, y, polygon)) {
          return true;
        }
      }
    }
  }
  return false;
}

bool TerrainModel::collides(const GridCell3D & cell) const
{
  const int radius_cells = std::max(
    0, static_cast<int>(std::ceil(parameters_.robot_radius / resolution())));
  const int height_cells = std::max(
    1, static_cast<int>(std::ceil(parameters_.robot_height / resolution())));
  for (int dz = 0; dz < height_cells; ++dz) {
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        const double offset_m = std::hypot(
          static_cast<double>(dx), static_cast<double>(dy)) * resolution();
        if (offset_m > parameters_.robot_radius + 1e-9) {
          continue;
        }
        if (collisionOccupied(GridCell3D{cell.x + dx, cell.y + dy, cell.z + dz})) {
          return true;
        }
      }
    }
  }
  return false;
}

bool TerrainModel::isTraversable(const GridCell3D & cell) const
{
  return inside(cell) && supported(cell) && !collides(cell) && !inPitFootprint(cell);
}

void TerrainModel::buildLayers()
{
  std::unordered_set<GridCell3D, GridCell3DHash> occupied_cells;
  std::unordered_set<GridCell3D, GridCell3DHash> candidates;
  const double r = resolution();
  constexpr double kEpsilon = 1e-6;
  if (observation_) {
    for (const auto & ground : observation_->ground_cells) {
      occupied_cells.insert(ground);
      candidates.insert(GridCell3D{ground.x, ground.y, ground.z + 1});
    }
    occupied_cells.insert(
      observation_->obstacle_cells.begin(), observation_->obstacle_cells.end());
  } else {
    for (auto iterator = tree_.begin_leafs(); iterator != tree_.end_leafs(); ++iterator) {
      if (!tree_.isNodeOccupied(*iterator)) {
        continue;
      }
      const double half_size = iterator.getSize() * 0.5;
      const int leaf_min_x = static_cast<int>(std::floor(
        (static_cast<double>(iterator.getX()) - half_size + kEpsilon) / r));
      const int leaf_min_y = static_cast<int>(std::floor(
        (static_cast<double>(iterator.getY()) - half_size + kEpsilon) / r));
      const int leaf_min_z = static_cast<int>(std::floor(
        (static_cast<double>(iterator.getZ()) - half_size + kEpsilon) / r));
      const int leaf_max_x = static_cast<int>(std::ceil(
        (static_cast<double>(iterator.getX()) + half_size - kEpsilon) / r)) - 1;
      const int leaf_max_y = static_cast<int>(std::ceil(
        (static_cast<double>(iterator.getY()) + half_size - kEpsilon) / r)) - 1;
      const int leaf_max_z = static_cast<int>(std::ceil(
        (static_cast<double>(iterator.getZ()) + half_size - kEpsilon) / r)) - 1;
      for (int x = leaf_min_x; x <= leaf_max_x; ++x) {
        for (int y = leaf_min_y; y <= leaf_max_y; ++y) {
          for (int z = leaf_min_z; z <= leaf_max_z; ++z) {
            occupied_cells.insert(GridCell3D{x, y, z});
          }
          candidates.insert(GridCell3D{x, y, leaf_max_z + 1});
        }
      }
    }
  }

  for (const auto & candidate : candidates) {
    if (!isTraversable(candidate)) {
      continue;
    }
    surface_cells_.insert(candidate);
  }

  const int margin_cells = std::max(
    0, static_cast<int>(std::ceil(parameters_.costmap_margin / r)));
  const int height_search_cells = std::max(
    1, static_cast<int>(std::ceil(parameters_.max_step_height / r)));
  const int point_cloud_hole_tolerance = observation_ ? 1 : 0;
  std::unordered_set<GridCell3D, GridCell3DHash> nearby_surface_cells;
  const std::size_t nearby_cell_count = static_cast<std::size_t>(
    (2 * point_cloud_hole_tolerance + 1) *
    (2 * point_cloud_hole_tolerance + 1) *
    (2 * height_search_cells + 1));
  nearby_surface_cells.reserve(surface_cells_.size() * nearby_cell_count);
  for (const auto & surface : surface_cells_) {
    for (
      int dx = -point_cloud_hole_tolerance;
      dx <= point_cloud_hole_tolerance; ++dx)
    {
      for (
        int dy = -point_cloud_hole_tolerance;
        dy <= point_cloud_hole_tolerance; ++dy)
      {
        for (int dz = -height_search_cells; dz <= height_search_cells; ++dz) {
          nearby_surface_cells.insert(
            GridCell3D{surface.x + dx, surface.y + dy, surface.z + dz});
        }
      }
    }
  }
  std::vector<std::tuple<int, int, double>> edge_offsets;
  edge_offsets.reserve(static_cast<std::size_t>(
    (2 * margin_cells + 1) * (2 * margin_cells + 1)));
  for (int dx = -margin_cells; dx <= margin_cells; ++dx) {
    for (int dy = -margin_cells; dy <= margin_cells; ++dy) {
      const double distance = std::hypot(static_cast<double>(dx), static_cast<double>(dy));
      if (distance >= 1.0 && distance <= static_cast<double>(margin_cells)) {
        edge_offsets.emplace_back(dx, dy, distance);
      }
    }
  }
  std::sort(
    edge_offsets.begin(), edge_offsets.end(),
    [](const auto & lhs, const auto & rhs) {return std::get<2>(lhs) < std::get<2>(rhs);});
  for (const auto & cell : surface_cells_) {
    double nearest_edge = std::numeric_limits<double>::infinity();
    for (const auto & [dx, dy, distance] : edge_offsets) {
      const bool neighbor_surface = nearby_surface_cells.find(
        GridCell3D{cell.x + dx, cell.y + dy, cell.z}) != nearby_surface_cells.end();
      if (!neighbor_surface) {
        nearest_edge = std::max(
          1.0, distance - static_cast<double>(point_cloud_hole_tolerance));
        break;
      }
    }
    double cost = 0.0;
    if (margin_cells > 0 && std::isfinite(nearest_edge)) {
      cost = std::max(
        0.0, (static_cast<double>(margin_cells) + 1.0 - nearest_edge) /
        (static_cast<double>(margin_cells) + 1.0));
    }
    traversal_costs_[cell] = cost;
    layers_.traversable_cells.push_back(TerrainCellCost{cell, cost});
  }

  if (observation_) {
    layers_.obstacle_cells.assign(
      observation_->obstacle_cells.begin(), observation_->obstacle_cells.end());
  } else {
    std::unordered_map<GridCell3D, int, GridCell3DHash> lowest_surface_z;
    for (const auto & surface : surface_cells_) {
      const GridCell3D column{surface.x, surface.y, 0};
      const auto found = lowest_surface_z.find(column);
      if (found == lowest_surface_z.end() || surface.z < found->second) {
        lowest_surface_z[column] = surface.z;
      }
    }
    const int obstacle_search_cells = std::max(
      1, static_cast<int>(std::ceil(parameters_.robot_radius / r)) + 1);
    for (const auto & cell : occupied_cells) {
      int nearby_surface_z = std::numeric_limits<int>::max();
      for (int dx = -obstacle_search_cells; dx <= obstacle_search_cells; ++dx) {
        for (int dy = -obstacle_search_cells; dy <= obstacle_search_cells; ++dy) {
          if (std::hypot(static_cast<double>(dx), static_cast<double>(dy)) >
            static_cast<double>(obstacle_search_cells))
          {
            continue;
          }
          const auto found = lowest_surface_z.find(GridCell3D{cell.x + dx, cell.y + dy, 0});
          if (found != lowest_surface_z.end()) {
            nearby_surface_z = std::min(nearby_surface_z, found->second);
          }
        }
      }
      if (nearby_surface_z != std::numeric_limits<int>::max() && cell.z < nearby_surface_z) {
        continue;
      }
      layers_.obstacle_cells.push_back(cell);
    }
  }
  const auto cell_less = [](const auto & lhs, const auto & rhs) {
      if (lhs.x != rhs.x) {
        return lhs.x < rhs.x;
      }
      if (lhs.y != rhs.y) {
        return lhs.y < rhs.y;
      }
      return lhs.z < rhs.z;
    };
  std::sort(layers_.obstacle_cells.begin(), layers_.obstacle_cells.end(), cell_less);
  std::sort(
    layers_.traversable_cells.begin(), layers_.traversable_cells.end(),
    [&cell_less](const auto & lhs, const auto & rhs) {return cell_less(lhs.cell, rhs.cell);});
}

double TerrainModel::traversalCost(const GridCell3D & cell) const
{
  const auto found = traversal_costs_.find(cell);
  if (found != traversal_costs_.end()) {
    return found->second;
  }
  return isTraversable(cell) ? 1.0 : 0.0;
}

const TerrainLayers & TerrainModel::layers() const
{
  return layers_;
}

bool TerrainModel::transitionAllowed(const GridCell3D & from, const GridCell3D & to) const
{
  const int dx = to.x - from.x;
  const int dy = to.y - from.y;
  const int dz = to.z - from.z;
  const double horizontal = std::hypot(static_cast<double>(dx), static_cast<double>(dy));
  if (horizontal == 0.0) {
    return false;
  }
  const double vertical_m = std::abs(static_cast<double>(dz)) * resolution();
  if (vertical_m > parameters_.max_step_height + 1e-9) {
    return false;
  }
  constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;
  const double slope_degrees = std::atan2(std::abs(static_cast<double>(dz)), horizontal) *
    kRadiansToDegrees;
  return slope_degrees <= parameters_.max_slope_degrees + 1e-9;
}

std::optional<GridCell3D> TerrainModel::snapToTerrain(const GridCell3D & seed) const
{
  const int radius = std::max(0, parameters_.snap_radius_cells);
  std::optional<GridCell3D> best;
  double best_score = std::numeric_limits<double>::infinity();
  for (int dz = -radius; dz <= radius; ++dz) {
    for (int dy = -radius; dy <= radius; ++dy) {
      for (int dx = -radius; dx <= radius; ++dx) {
        const GridCell3D candidate{seed.x + dx, seed.y + dy, seed.z + dz};
        if (surface_cells_.find(candidate) == surface_cells_.end()) {
          continue;
        }
        const double score = static_cast<double>(dx * dx + dy * dy) +
          0.25 * static_cast<double>(dz * dz);
        if (score < best_score) {
          best = candidate;
          best_score = score;
        }
      }
    }
  }
  return best;
}

std::optional<GridCell3D> TerrainModel::snapGoalToTerrain(const GridCell3D & seed) const
{
  return snapToTerrainAtXY(seed);
}

std::optional<GridCell3D> TerrainModel::snapToTerrainAtXY(const GridCell3D & seed) const
{
  const int radius = std::max(0, parameters_.snap_radius_cells);
  std::optional<GridCell3D> best;
  std::tuple<int, int, int> best_score{
    std::numeric_limits<int>::max(), std::numeric_limits<int>::max(),
    std::numeric_limits<int>::max()};
  for (const auto & candidate : surface_cells_) {
    const int dx = candidate.x - seed.x;
    const int dy = candidate.y - seed.y;
    if (std::abs(dx) > radius || std::abs(dy) > radius) {
      continue;
    }
    const std::tuple<int, int, int> score{
      dx * dx + dy * dy, std::abs(candidate.z - seed.z), candidate.z};
    if (score < best_score) {
      best = candidate;
      best_score = score;
    }
  }
  return best;
}

std::vector<GridCell3D> TerrainModel::plan(
  const GridCell3D & start, const GridCell3D & goal) const
{
  return planAvoidingColumns(start, goal, {});
}

std::vector<GridCell3D> TerrainModel::planAvoidingColumns(
  const GridCell3D & start, const GridCell3D & goal,
  const GridColumnSet & blocked_columns,
  std::optional<GridPlanningBounds> bounds) const
{
  return planAstar3D(
    start, goal,
    [this, &blocked_columns, &bounds, &start, &goal](const auto & cell) {
      if (bounds &&
        (cell.x < bounds->min_x || cell.x > bounds->max_x ||
        cell.y < bounds->min_y || cell.y > bounds->max_y))
      {
        return false;
      }
      const GridCell3D column{cell.x, cell.y, 0};
      const bool endpoint = cell == start || cell == goal;
      return surface_cells_.find(cell) != surface_cells_.end() &&
             (endpoint || blocked_columns.find(column) == blocked_columns.end());
    },
    [this](const auto & from, const auto & to) {return transitionAllowed(from, to);},
    [this](const auto & cell) {
      return parameters_.costmap_weight * traversalCost(cell);
    }, parameters_.max_iterations);
}

}  // namespace luxi_3d_navigation
