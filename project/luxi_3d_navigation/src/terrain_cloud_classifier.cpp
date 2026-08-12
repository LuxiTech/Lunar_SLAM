#include "luxi_3d_navigation/terrain_cloud_classifier.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>

#include "pcl/features/normal_3d.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/io/ply_io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/search/kdtree.h"

namespace luxi_3d_navigation
{
namespace
{

GridCell3D worldToGrid(const pcl::PointXYZ & point, double resolution)
{
  return GridCell3D{
    static_cast<int>(std::floor(static_cast<double>(point.x) / resolution)),
    static_cast<int>(std::floor(static_cast<double>(point.y) / resolution)),
    static_cast<int>(std::floor(static_cast<double>(point.z) / resolution))};
}

void validateParameters(const TerrainCloudParameters & parameters)
{
  if (
    parameters.resolution <= 0.0 || parameters.normal_radius <= parameters.resolution ||
    parameters.maximum_ground_slope_degrees <= 0.0 ||
    parameters.maximum_ground_slope_degrees >= 90.0 || parameters.obstacle_min_height <= 0.0)
  {
    throw std::invalid_argument("terrain cloud classifier parameters are invalid");
  }
}

}  // namespace

TerrainObservation classifyTerrainPoints(
  const std::vector<TerrainCloudPoint> & points,
  const TerrainCloudParameters & parameters)
{
  validateParameters(parameters);
  auto input = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  auto input_with_normals =
    std::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>();
  input->reserve(points.size());
  input_with_normals->reserve(points.size());
  bool all_points_have_normals = true;
  for (const auto & point : points) {
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
      input->push_back(pcl::PointXYZ(point.x, point.y, point.z));
      pcl::PointXYZRGBNormal point_with_normal;
      point_with_normal.x = point.x;
      point_with_normal.y = point.y;
      point_with_normal.z = point.z;
      point_with_normal.normal_x = point.normal_x;
      point_with_normal.normal_y = point.normal_y;
      point_with_normal.normal_z = point.normal_z;
      point_with_normal.rgba = 0U;
      point_with_normal.curvature = 0.0F;
      input_with_normals->push_back(point_with_normal);
      const double normal_norm = std::sqrt(
        static_cast<double>(point.normal_x) * point.normal_x +
        static_cast<double>(point.normal_y) * point.normal_y +
        static_cast<double>(point.normal_z) * point.normal_z);
      all_points_have_normals = all_points_have_normals &&
        std::isfinite(normal_norm) && normal_norm >= 1e-6;
    }
  }
  if (input->empty()) {
    throw std::runtime_error("terrain point cloud contains no finite points");
  }

  auto downsampled = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  const float leaf_size = static_cast<float>(parameters.resolution);
  auto normals = std::make_shared<pcl::PointCloud<pcl::Normal>>();
  if (all_points_have_normals) {
    auto downsampled_with_normals =
      std::make_shared<pcl::PointCloud<pcl::PointXYZRGBNormal>>();
    pcl::VoxelGrid<pcl::PointXYZRGBNormal> voxel_filter;
    voxel_filter.setInputCloud(input_with_normals);
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_filter.setDownsampleAllData(true);
    voxel_filter.filter(*downsampled_with_normals);
    downsampled->reserve(downsampled_with_normals->size());
    normals->reserve(downsampled_with_normals->size());
    for (const auto & point : *downsampled_with_normals) {
      downsampled->push_back(pcl::PointXYZ(point.x, point.y, point.z));
      pcl::Normal normal;
      normal.normal_x = point.normal_x;
      normal.normal_y = point.normal_y;
      normal.normal_z = point.normal_z;
      normals->push_back(normal);
    }
  } else {
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(input);
    voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_filter.filter(*downsampled);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    auto search = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
    normal_estimator.setInputCloud(downsampled);
    normal_estimator.setSearchMethod(search);
    normal_estimator.setRadiusSearch(parameters.normal_radius);
    normal_estimator.compute(*normals);
  }

  constexpr double kPi = 3.14159265358979323846;
  const double minimum_vertical_normal = std::cos(
    parameters.maximum_ground_slope_degrees * kPi / 180.0);
  std::vector<bool> ground_candidates(downsampled->size(), false);
  std::size_t ground_candidate_count = 0U;
  for (std::size_t index = 0; index < downsampled->size(); ++index) {
    const auto & normal = (*normals)[index];
    const double norm = std::sqrt(
      static_cast<double>(normal.normal_x) * normal.normal_x +
      static_cast<double>(normal.normal_y) * normal.normal_y +
      static_cast<double>(normal.normal_z) * normal.normal_z);
    if (!std::isfinite(norm) || norm < 1e-6) {
      continue;
    }
    const double vertical_normal = std::abs(static_cast<double>(normal.normal_z) / norm);
    if (vertical_normal < minimum_vertical_normal) {
      continue;
    }
    ground_candidates[index] = true;
    ++ground_candidate_count;
  }
  if (ground_candidate_count == 0U) {
    throw std::runtime_error("terrain point cloud has no ground-like surface");
  }

  const double maximum_ground_gradient = std::tan(
    parameters.maximum_ground_slope_degrees * kPi / 180.0);
  const int continuity_radius_cells = std::max(
    2, static_cast<int>(std::ceil(parameters.normal_radius / parameters.resolution)));
  auto connectivity_search = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
  connectivity_search->setInputCloud(downsampled);
  const double connectivity_search_radius =
    static_cast<double>(continuity_radius_cells + 2) * parameters.resolution;
  std::vector<bool> visited(downsampled->size(), false);
  std::vector<std::size_t> largest_component;
  for (std::size_t seed = 0U; seed < downsampled->size(); ++seed) {
    if (!ground_candidates[seed] || visited[seed]) {
      continue;
    }
    std::vector<std::size_t> component;
    std::queue<std::size_t> pending;
    visited[seed] = true;
    pending.push(seed);
    while (!pending.empty()) {
      const std::size_t current_index = pending.front();
      pending.pop();
      component.push_back(current_index);
      const auto & current_point = (*downsampled)[current_index];
      const auto current_cell = worldToGrid(current_point, parameters.resolution);
      std::vector<int> neighbor_indices;
      std::vector<float> squared_distances;
      connectivity_search->radiusSearch(
        current_point, connectivity_search_radius,
        neighbor_indices, squared_distances);
      for (const int raw_index : neighbor_indices) {
        const std::size_t next_index = static_cast<std::size_t>(raw_index);
        if (next_index == current_index || !ground_candidates[next_index] ||
          visited[next_index])
        {
          continue;
        }
        const auto & next_point = (*downsampled)[next_index];
        const auto next_cell = worldToGrid(next_point, parameters.resolution);
        const int dx = next_cell.x - current_cell.x;
        const int dy = next_cell.y - current_cell.y;
        const int dz = next_cell.z - current_cell.z;
        if (std::abs(dz) > 1 ||
          dx * dx + dy * dy > continuity_radius_cells * continuity_radius_cells)
        {
          continue;
        }
        const double horizontal = std::hypot(
          static_cast<double>(next_point.x - current_point.x),
          static_cast<double>(next_point.y - current_point.y));
        const double maximum_height_change = std::min(
          parameters.resolution * 1.1,
          parameters.resolution * 0.5 + maximum_ground_gradient * horizontal);
        if (std::abs(static_cast<double>(next_point.z - current_point.z)) >
          maximum_height_change)
        {
          continue;
        }
        visited[next_index] = true;
        pending.push(next_index);
      }
    }
    if (component.size() > largest_component.size()) {
      largest_component = std::move(component);
    }
  }

  std::vector<bool> ground_flags(downsampled->size(), false);
  for (const std::size_t index : largest_component) {
    ground_flags[index] = true;
  }

  std::unordered_map<GridCell3D, std::size_t, GridCell3DHash> surface_indices;
  for (std::size_t index = 0; index < downsampled->size(); ++index) {
    if (!ground_flags[index]) {
      continue;
    }
    const auto cell = worldToGrid((*downsampled)[index], parameters.resolution);
    const GridCell3D column{cell.x, cell.y, 0};
    const auto found = surface_indices.find(column);
    if (found == surface_indices.end() ||
      (*downsampled)[index].z > (*downsampled)[found->second].z)
    {
      surface_indices[column] = index;
    }
  }

  TerrainObservation observation;
  std::unordered_map<GridCell3D, std::vector<double>, GridCell3DHash> ground_heights;
  for (const auto & entry : surface_indices) {
    const auto & point = (*downsampled)[entry.second];
    const auto cell = worldToGrid(point, parameters.resolution);
    observation.ground_cells.insert(cell);
    ground_heights[entry.first].push_back(point.z);
  }

  const int search_cells = std::max(
    1, static_cast<int>(std::ceil(parameters.normal_radius / parameters.resolution)));
  for (std::size_t index = 0; index < downsampled->size(); ++index) {
    if (ground_flags[index]) {
      continue;
    }
    const auto & point = (*downsampled)[index];
    const auto cell = worldToGrid(point, parameters.resolution);
    double support_height = -std::numeric_limits<double>::infinity();
    double support_horizontal = std::numeric_limits<double>::infinity();
    for (int dx = -search_cells; dx <= search_cells; ++dx) {
      for (int dy = -search_cells; dy <= search_cells; ++dy) {
        if (std::hypot(static_cast<double>(dx), static_cast<double>(dy)) > search_cells) {
          continue;
        }
        const auto found = ground_heights.find(GridCell3D{cell.x + dx, cell.y + dy, 0});
        if (found == ground_heights.end()) {
          continue;
        }
        for (const double height : found->second) {
          if (height <= static_cast<double>(point.z) + parameters.resolution * 0.5 &&
            height > support_height)
          {
            support_height = std::max(support_height, height);
            support_horizontal = std::hypot(
              static_cast<double>(dx), static_cast<double>(dy)) * parameters.resolution;
          }
        }
      }
    }
    double minimum_obstacle_height = parameters.obstacle_min_height;
    if (ground_candidates[index] && std::isfinite(support_horizontal)) {
      minimum_obstacle_height += maximum_ground_gradient * support_horizontal;
    }
    if (std::isfinite(support_height) &&
      static_cast<double>(point.z) - support_height >= minimum_obstacle_height)
    {
      observation.obstacle_cells.insert(cell);
    }
  }
  for (const auto & ground : observation.ground_cells) {
    observation.obstacle_cells.erase(ground);
  }
  return observation;
}

TerrainObservation classifyTerrainCloudFile(
  const std::string & path,
  const TerrainCloudParameters & parameters)
{
  pcl::PointCloud<pcl::PointXYZRGBNormal> cloud;
  if (pcl::io::loadPLYFile(path, cloud) < 0) {
    throw std::runtime_error("cannot read terrain PLY: " + path);
  }
  std::vector<TerrainCloudPoint> points;
  points.reserve(cloud.size());
  for (const auto & point : cloud) {
    points.push_back(TerrainCloudPoint{
      point.x, point.y, point.z,
      point.normal_x, point.normal_y, point.normal_z});
  }
  return classifyTerrainPoints(points, parameters);
}

}  // namespace luxi_3d_navigation
