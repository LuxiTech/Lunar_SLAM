#include <gtest/gtest.h>

#include "luxi_3d_navigation/rolling_voxel_grid.hpp"

namespace
{

luxi_3d_navigation::RollingVoxelGrid makeGrid()
{
  luxi_3d_navigation::RollingVoxelGridParameters parameters;
  parameters.resolution = 0.1;
  parameters.occupied_threshold = 2;
  parameters.occupied_ttl = 1.0;
  parameters.stale_entry_ttl = 2.0;
  parameters.size_x = 4.0;
  parameters.size_y = 4.0;
  parameters.size_z = 2.0;
  return luxi_3d_navigation::RollingVoxelGrid(parameters);
}

TEST(RollingVoxelGrid, RequiresRepeatedHitsAndExpiresOccupiedVoxel)
{
  auto grid = makeGrid();
  const luxi_3d_navigation::Point3D origin{0.0, 0.0, 0.0};
  const std::vector<luxi_3d_navigation::RayObservation> frame{
    {{0.55, 0.05, 0.05}, true}};
  grid.integrateFrame(origin, frame, 1.0);
  EXPECT_TRUE(grid.occupiedPoints(1.0).empty());
  grid.integrateFrame(origin, frame, 1.1);
  ASSERT_EQ(grid.occupiedPoints(1.1).size(), 1U);
  EXPECT_TRUE(grid.occupiedPoints(2.2).empty());
}

TEST(RollingVoxelGrid, FreeRayClearsPreviouslyOccupiedEndpoint)
{
  auto grid = makeGrid();
  const luxi_3d_navigation::Point3D origin{0.0, 0.0, 0.0};
  const std::vector<luxi_3d_navigation::RayObservation> hit{{{0.55, 0.05, 0.05}, true}};
  grid.integrateFrame(origin, hit, 1.0);
  grid.integrateFrame(origin, hit, 1.1);
  ASSERT_EQ(grid.occupiedPoints(1.1).size(), 1U);

  const std::vector<luxi_3d_navigation::RayObservation> clear{{{0.85, 0.05, 0.05}, false}};
  grid.integrateFrame(origin, clear, 1.2);
  EXPECT_TRUE(grid.occupiedPoints(1.2).empty());
}

TEST(RollingVoxelGrid, PrunesEntriesOutsideSlidingWindow)
{
  auto grid = makeGrid();
  const luxi_3d_navigation::Point3D origin{0.0, 0.0, 0.0};
  const std::vector<luxi_3d_navigation::RayObservation> hit{{{0.55, 0.05, 0.05}, true}};
  grid.integrateFrame(origin, hit, 1.0);
  EXPECT_GT(grid.entryCount(), 0U);
  grid.prune({10.0, 0.0, 0.0}, 1.1);
  EXPECT_EQ(grid.entryCount(), 0U);
}

TEST(RollingVoxelGrid, ClearRemovesAccumulatedHistory)
{
  auto grid = makeGrid();
  const luxi_3d_navigation::Point3D origin{0.0, 0.0, 0.0};
  const std::vector<luxi_3d_navigation::RayObservation> hit{{{0.55, 0.05, 0.05}, true}};
  grid.integrateFrame(origin, hit, 1.0);
  grid.integrateFrame(origin, hit, 1.1);
  ASSERT_FALSE(grid.occupiedPoints(1.1).empty());

  grid.clear();

  EXPECT_EQ(grid.entryCount(), 0U);
  EXPECT_TRUE(grid.occupiedPoints(1.1).empty());
}

}  // namespace
