#include "luxi_3d_navigation/terrain_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace luxi_3d_navigation
{

TerrainModel::TerrainModel(
  const octomap::OcTree & tree, TerrainParameters parameters,
  std::vector<Polygon2D> pit_polygons)
: tree_(tree), parameters_(parameters), pit_polygons_(std::move(pit_polygons))
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

bool TerrainModel::supported(const GridCell3D & cell) const
{
  const int xy_radius = parameters_.strict_direct_support ? 0 :
    std::max(0, parameters_.support_xy_radius_cells);
  const int depth = parameters_.strict_direct_support ? 1 :
    std::max(1, parameters_.support_depth_cells);
  for (int dz = 1; dz <= depth; ++dz) {
    for (int dy = -xy_radius; dy <= xy_radius; ++dy) {
      for (int dx = -xy_radius; dx <= xy_radius; ++dx) {
        if (occupied(GridCell3D{cell.x + dx, cell.y + dy, cell.z - dz})) {
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
        if (occupied(GridCell3D{cell.x + dx, cell.y + dy, cell.z + dz})) {
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
        if (!isTraversable(candidate)) {
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

std::vector<GridCell3D> TerrainModel::plan(
  const GridCell3D & start, const GridCell3D & goal) const
{
  return planAstar3D(
    start, goal,
    [this](const auto & cell) {return isTraversable(cell);},
    [this](const auto & from, const auto & to) {return transitionAllowed(from, to);},
    [](const auto &) {return 0.0;}, parameters_.max_iterations);
}

}  // namespace luxi_3d_navigation
