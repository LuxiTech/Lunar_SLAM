#include <cmath>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"

#include "luxi_3d_navigation/terrain_cloud_classifier.hpp"

TEST(TerrainCloudClassifier, UsesFiveCentimeterDefaultResolution)
{
  const luxi_3d_navigation::TerrainCloudParameters parameters;
  EXPECT_DOUBLE_EQ(parameters.resolution, 0.05);
}

TEST(TerrainCloudClassifier, SeparatesTiltedGroundFromVerticalWall)
{
  std::vector<luxi_3d_navigation::TerrainCloudPoint> points;
  for (int x = 0; x <= 40; ++x) {
    for (int y = 0; y <= 24; ++y) {
      const float world_x = 0.05F * static_cast<float>(x);
      const float world_y = 0.05F * static_cast<float>(y);
      const float world_z = 0.20F * world_x + 0.003F * static_cast<float>((x + y) % 3 - 1);
      points.push_back({world_x, world_y, world_z});
    }
  }
  for (int y = 0; y <= 24; ++y) {
    for (int z = 4; z <= 18; ++z) {
      points.push_back({1.0F, 0.05F * static_cast<float>(y), 0.20F + 0.05F * z});
    }
  }

  luxi_3d_navigation::TerrainCloudParameters parameters;
  parameters.resolution = 0.10;
  parameters.normal_radius = 0.25;
  parameters.maximum_ground_slope_degrees = 30.0;
  parameters.obstacle_min_height = 0.15;
  const auto observation = luxi_3d_navigation::classifyTerrainPoints(points, parameters);

  EXPECT_GT(observation.ground_cells.size(), 150U);
  EXPECT_GT(observation.obstacle_cells.size(), 20U);
  bool found_clear_ground = false;
  for (const auto & cell : observation.ground_cells) {
    if (cell.x == 5 && cell.y == 5) {
      found_clear_ground = true;
      EXPECT_EQ(observation.obstacle_cells.count(cell), 0U);
    }
  }
  EXPECT_TRUE(found_clear_ground);
}

TEST(TerrainCloudClassifier, DoesNotTreatRaisedHorizontalObstacleAsGround)
{
  std::vector<luxi_3d_navigation::TerrainCloudPoint> points;
  for (int x = -20; x <= 20; ++x) {
    for (int y = -20; y <= 20; ++y) {
      const float world_x = 0.05F * static_cast<float>(x);
      const float world_y = 0.05F * static_cast<float>(y);
      if (world_x >= 0.40F && world_y >= 0.40F) {
        continue;
      }
      points.push_back({world_x, world_y, 0.0F});
    }
  }
  for (int x = 8; x <= 20; ++x) {
    for (int y = 8; y <= 20; ++y) {
      points.push_back({
        0.05F * static_cast<float>(x),
        0.05F * static_cast<float>(y), 0.35F});
    }
  }
  for (int axis = 8; axis <= 20; ++axis) {
    for (int z = 0; z <= 7; ++z) {
      const float along = 0.05F * static_cast<float>(axis);
      const float height = 0.05F * static_cast<float>(z);
      points.push_back({0.40F, along, height});
      points.push_back({along, 0.40F, height});
    }
  }

  luxi_3d_navigation::TerrainCloudParameters parameters;
  parameters.resolution = 0.10;
  parameters.normal_radius = 0.25;
  parameters.maximum_ground_slope_degrees = 35.0;
  parameters.obstacle_min_height = 0.15;
  const auto observation = luxi_3d_navigation::classifyTerrainPoints(points, parameters);

  std::size_t raised_ground = 0U;
  std::size_t raised_obstacles = 0U;
  for (const auto & cell : observation.ground_cells) {
    if (cell.x >= 4 && cell.y >= 4 && cell.z >= 2) {
      ++raised_ground;
    }
  }
  for (const auto & cell : observation.obstacle_cells) {
    if (cell.x >= 4 && cell.y >= 4 && cell.z >= 2) {
      ++raised_obstacles;
    }
  }
  EXPECT_EQ(raised_ground, 0U);
  EXPECT_GT(raised_obstacles, 10U);
}

TEST(TerrainCloudClassifier, UsesFiniteNormalsAlreadyStoredInMapCloud)
{
  std::vector<luxi_3d_navigation::TerrainCloudPoint> points;
  for (int x = -10; x <= 10; ++x) {
    for (int y = -10; y <= 10; ++y) {
      points.push_back({
        0.05F * static_cast<float>(x),
        0.05F * static_cast<float>(y), 0.0F,
        1.0F, 0.0F, 0.0F});
    }
  }

  luxi_3d_navigation::TerrainCloudParameters parameters;
  parameters.resolution = 0.10;
  parameters.normal_radius = 0.25;
  parameters.maximum_ground_slope_degrees = 35.0;
  parameters.obstacle_min_height = 0.15;

  EXPECT_THROW(
    luxi_3d_navigation::classifyTerrainPoints(points, parameters),
    std::runtime_error);
}

TEST(TerrainCloudClassifier, AnchorsGroundToOriginInsteadOfLargerFurnitureTop)
{
  std::vector<luxi_3d_navigation::TerrainCloudPoint> points;
  // A small floor patch around the robot origin.
  for (int x = -10; x <= 10; ++x) {
    for (int y = -10; y <= 10; ++y) {
      points.push_back({
        0.05F * static_cast<float>(x),
        0.05F * static_cast<float>(y), -0.05F});
    }
  }
  // A much larger disconnected horizontal surface must not win merely by area.
  for (int x = 20; x <= 60; ++x) {
    for (int y = -20; y <= 20; ++y) {
      points.push_back({
        0.05F * static_cast<float>(x),
        0.05F * static_cast<float>(y), 0.55F});
    }
  }

  luxi_3d_navigation::TerrainCloudParameters parameters;
  parameters.resolution = 0.10;
  parameters.normal_radius = 0.25;
  parameters.maximum_ground_slope_degrees = 35.0;
  parameters.obstacle_min_height = 0.15;
  const auto observation = luxi_3d_navigation::classifyTerrainPoints(points, parameters);

  EXPECT_GT(observation.ground_cells.size(), 80U);
  for (const auto & cell : observation.ground_cells) {
    EXPECT_LT(cell.z, 1);
  }
}

TEST(TerrainCloudClassifier, DoesNotClimbAStackOfHorizontalStereoReturns)
{
  std::vector<luxi_3d_navigation::TerrainCloudPoint> points;
  for (int x = -20; x <= 20; ++x) {
    for (int y = -20; y <= 20; ++y) {
      points.push_back({
        0.05F * static_cast<float>(x),
        0.05F * static_cast<float>(y), -0.05F});
    }
  }
  // Mimic dense-stereo edge bleeding: several horizontal-looking returns in
  // the same XY columns rise toward an object top.
  for (int x = 4; x <= 12; ++x) {
    for (int y = -4; y <= 4; ++y) {
      for (int z = 0; z <= 8; ++z) {
        points.push_back({
          0.05F * static_cast<float>(x),
          0.05F * static_cast<float>(y), 0.05F * static_cast<float>(z)});
      }
    }
  }

  luxi_3d_navigation::TerrainCloudParameters parameters;
  parameters.resolution = 0.10;
  parameters.normal_radius = 0.25;
  parameters.maximum_ground_slope_degrees = 35.0;
  parameters.obstacle_min_height = 0.15;
  const auto observation = luxi_3d_navigation::classifyTerrainPoints(points, parameters);

  for (const auto & cell : observation.ground_cells) {
    EXPECT_LT(cell.z, 1);
  }
  std::size_t elevated_obstacles = 0U;
  for (const auto & cell : observation.obstacle_cells) {
    if (cell.z >= 1) {
      ++elevated_obstacles;
    }
  }
  EXPECT_GT(elevated_obstacles, 10U);
}
