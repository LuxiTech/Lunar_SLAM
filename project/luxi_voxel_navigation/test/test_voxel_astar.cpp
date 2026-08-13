#include <gtest/gtest.h>

#include "luxi_voxel_navigation/octomap_defaults.hpp"
#include "luxi_voxel_navigation/voxel_astar.hpp"

TEST(VoxelDefaults, UsesFiveCentimeterOctomapResolution)
{
  EXPECT_DOUBLE_EQ(luxi_voxel_navigation::kDefaultOctomapResolution, 0.05);
}

TEST(VoxelAstar, FindsRouteAroundObstacle)
{
  constexpr int width = 6;
  constexpr int height = 5;
  std::vector<std::uint8_t> blocked(width * height, 0);
  for (int y = 0; y < height - 1; ++y) {
    blocked[luxi_voxel_navigation::gridOffset(width, {2, y})] = 1;
  }
  const auto path = luxi_voxel_navigation::planAstar(
    width, height, blocked, {0, 0}, {5, 0}, false);
  ASSERT_FALSE(path.empty());
  EXPECT_EQ(path.front(), (luxi_voxel_navigation::GridCell{0, 0}));
  EXPECT_EQ(path.back(), (luxi_voxel_navigation::GridCell{5, 0}));
  for (const auto & cell : path) {
    EXPECT_EQ(blocked[luxi_voxel_navigation::gridOffset(width, cell)], 0);
  }
}

TEST(VoxelAstar, RejectsBlockedEndpoint)
{
  std::vector<std::uint8_t> blocked(9, 0);
  blocked[0] = 1;
  EXPECT_TRUE(luxi_voxel_navigation::planAstar(3, 3, blocked, {0, 0}, {2, 2}, true).empty());
}
