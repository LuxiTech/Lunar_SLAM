#include "gtest/gtest.h"
#include "octomap/OcTree.h"

#include "luxi_3d_navigation/terrain_model.hpp"

namespace
{

octomap::OcTree makeGround(int length)
{
  octomap::OcTree tree(0.1);
  for (int x = 0; x < length; ++x) {
    tree.updateNode(octomap::point3d(0.05F + 0.1F * x, 0.05F, -0.05F), true);
  }
  tree.updateInnerOccupancy();
  return tree;
}

luxi_3d_navigation::TerrainParameters testParameters()
{
  luxi_3d_navigation::TerrainParameters parameters;
  parameters.robot_radius = 0.0;
  parameters.robot_height = 0.1;
  parameters.strict_direct_support = true;
  parameters.support_depth_cells = 1;
  parameters.snap_radius_cells = 2;
  return parameters;
}

}  // namespace

TEST(TerrainModel, PlansOnDirectlySupportedGround)
{
  auto tree = makeGround(5);
  luxi_3d_navigation::TerrainModel terrain(tree, testParameters());
  const auto start = terrain.worldToGrid(0.05, 0.05, 0.05);
  const auto goal = terrain.worldToGrid(0.45, 0.05, 0.05);
  EXPECT_TRUE(terrain.isTraversable(start));
  EXPECT_EQ(terrain.plan(start, goal).size(), 5U);
}

TEST(TerrainModel, RequiresGroundSupport)
{
  auto tree = makeGround(1);
  luxi_3d_navigation::TerrainModel terrain(tree, testParameters());
  EXPECT_FALSE(terrain.isTraversable(terrain.worldToGrid(0.25, 0.05, 0.05)));
}

TEST(TerrainModel, RejectsSemanticPitFootprint)
{
  auto tree = makeGround(5);
  const luxi_3d_navigation::Polygon2D pit{
    {0.18, -0.02}, {0.32, -0.02}, {0.32, 0.12}, {0.18, 0.12}};
  luxi_3d_navigation::TerrainModel terrain(tree, testParameters(), {pit});
  EXPECT_FALSE(terrain.isTraversable(terrain.worldToGrid(0.25, 0.05, 0.05)));
  EXPECT_TRUE(terrain.plan(
    terrain.worldToGrid(0.05, 0.05, 0.05),
    terrain.worldToGrid(0.45, 0.05, 0.05)).empty());
}

TEST(TerrainModel, EnforcesStepAndSlopeLimits)
{
  auto tree = makeGround(1);
  tree.updateNode(octomap::point3d(0.15F, 0.05F, 0.05F), true);
  tree.updateInnerOccupancy();
  auto parameters = testParameters();
  parameters.max_step_height = 0.05;
  parameters.max_slope_degrees = 20.0;
  luxi_3d_navigation::TerrainModel terrain(tree, parameters);
  EXPECT_FALSE(terrain.transitionAllowed(
    terrain.worldToGrid(0.05, 0.05, 0.05),
    terrain.worldToGrid(0.15, 0.05, 0.15)));
}
