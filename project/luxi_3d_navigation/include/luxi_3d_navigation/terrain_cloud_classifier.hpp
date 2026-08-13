#pragma once

#include <limits>
#include <string>
#include <vector>

#include "luxi_3d_navigation/terrain_model.hpp"

namespace luxi_3d_navigation
{

struct TerrainCloudPoint
{
  float x{};
  float y{};
  float z{};
  float normal_x{std::numeric_limits<float>::quiet_NaN()};
  float normal_y{std::numeric_limits<float>::quiet_NaN()};
  float normal_z{std::numeric_limits<float>::quiet_NaN()};
};

struct TerrainCloudParameters
{
  double resolution{0.05};
  double normal_radius{0.30};
  double maximum_ground_slope_degrees{35.0};
  double obstacle_min_height{0.15};
};

TerrainObservation classifyTerrainPoints(
  const std::vector<TerrainCloudPoint> & points,
  const TerrainCloudParameters & parameters);

TerrainObservation classifyTerrainCloudFile(
  const std::string & path,
  const TerrainCloudParameters & parameters);

}  // namespace luxi_3d_navigation
