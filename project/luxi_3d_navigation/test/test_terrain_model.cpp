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

octomap::OcTree makeGroundPatch(int width, int height)
{
  octomap::OcTree tree(0.1);
  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      tree.updateNode(
        octomap::point3d(0.05F + 0.1F * x, 0.05F + 0.1F * y, -0.05F), true);
    }
  }
  tree.updateInnerOccupancy();
  return tree;
}

octomap::OcTree makeThickGroundPatch(int width, int height)
{
  octomap::OcTree tree(0.1);
  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      for (int z = -2; z <= -1; ++z) {
        tree.updateNode(
          octomap::point3d(
            0.05F + 0.1F * x, 0.05F + 0.1F * y, 0.05F + 0.1F * z), true);
      }
    }
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

TEST(TerrainModel, UsesTenCentimeterDefaultRobotRadius)
{
  const luxi_3d_navigation::TerrainParameters parameters;
  EXPECT_DOUBLE_EQ(parameters.robot_radius, 0.10);
}

TEST(TerrainModel, RequiresGroundSupport)
{
  auto tree = makeGround(1);
  luxi_3d_navigation::TerrainModel terrain(tree, testParameters());
  EXPECT_FALSE(terrain.isTraversable(terrain.worldToGrid(0.25, 0.05, 0.05)));
}

TEST(TerrainModel, SnappingReturnsTheClassifiedSurfaceInsteadOfAirAboveIt)
{
  auto tree = makeGround(1);
  auto parameters = testParameters();
  parameters.strict_direct_support = false;
  parameters.support_depth_cells = 2;
  luxi_3d_navigation::TerrainModel terrain(tree, parameters);

  const auto air_cell = terrain.worldToGrid(0.05, 0.05, 0.15);
  ASSERT_TRUE(terrain.isTraversable(air_cell));
  const auto snapped = terrain.snapToTerrain(air_cell);

  ASSERT_TRUE(snapped.has_value());
  EXPECT_EQ(*snapped, terrain.worldToGrid(0.05, 0.05, 0.05));
}

TEST(TerrainModel, GoalSnappingUsesXYToReachTerrainFarFromZeroHeight)
{
  auto tree = makeGround(5);
  luxi_3d_navigation::TerrainModel terrain(tree, testParameters());
  const auto target_xy = terrain.worldToGrid(0.45, 0.05, 4.05);

  EXPECT_FALSE(terrain.snapToTerrain(target_xy).has_value());
  const auto grounded = terrain.snapGoalToTerrain(target_xy);

  ASSERT_TRUE(grounded.has_value());
  EXPECT_EQ(*grounded, terrain.worldToGrid(0.45, 0.05, 0.05));
}

TEST(TerrainModel, PlannedPathStaysOnAContinuousRampSurface)
{
  octomap::OcTree tree(0.1);
  for (int x = 0; x < 5; ++x) {
    tree.updateNode(
      octomap::point3d(0.05F + 0.1F * x, 0.05F, -0.05F + 0.1F * x), true);
  }
  tree.updateInnerOccupancy();
  auto parameters = testParameters();
  parameters.max_step_height = 0.11;
  parameters.max_slope_degrees = 50.0;
  luxi_3d_navigation::TerrainModel terrain(tree, parameters);
  const auto start = terrain.worldToGrid(0.05, 0.05, 0.05);
  const auto goal = terrain.snapGoalToTerrain(
    terrain.worldToGrid(0.45, 0.05, 0.05));

  ASSERT_TRUE(goal.has_value());
  const auto path = terrain.plan(start, *goal);
  ASSERT_EQ(path.size(), 5U);
  for (std::size_t index = 1U; index < path.size(); ++index) {
    EXPECT_EQ(path[index].x - path[index - 1U].x, 1);
    EXPECT_EQ(path[index].z - path[index - 1U].z, 1);
  }
}

TEST(TerrainModel, ReturnPathGroundsPlanarOdometryOnTheElevatedRampSurface)
{
  octomap::OcTree tree(0.1);
  for (int x = 0; x < 20; ++x) {
    tree.updateNode(
      octomap::point3d(0.05F + 0.1F * x, 0.05F, -0.05F + 0.1F * x), true);
  }
  tree.updateInnerOccupancy();
  auto parameters = testParameters();
  parameters.max_step_height = 0.11;
  parameters.max_slope_degrees = 50.0;
  parameters.snap_radius_cells = 6;
  luxi_3d_navigation::TerrainModel terrain(tree, parameters);

  const auto stale_odometry_height = terrain.worldToGrid(1.95, 0.05, 0.05);
  EXPECT_FALSE(terrain.snapToTerrain(stale_odometry_height).has_value());
  const auto grounded_start = terrain.snapToTerrainAtXY(stale_odometry_height);

  ASSERT_TRUE(grounded_start.has_value());
  const auto return_goal = terrain.snapToTerrainAtXY(
    terrain.worldToGrid(0.05, 0.05, 0.05));
  ASSERT_TRUE(return_goal.has_value());
  const auto path = terrain.plan(*grounded_start, *return_goal);
  ASSERT_EQ(path.size(), 20U);
  for (std::size_t index = 1U; index < path.size(); ++index) {
    EXPECT_EQ(path[index].x - path[index - 1U].x, -1);
    EXPECT_EQ(path[index].z - path[index - 1U].z, -1);
  }
}

TEST(TerrainModel, HonorsMetricRobotRadiusAtVoxelBoundary)
{
  auto tree = makeGround(1);
  tree.updateNode(octomap::point3d(0.25F, 0.05F, 0.05F), true);
  tree.updateInnerOccupancy();
  auto parameters = testParameters();
  parameters.robot_radius = 0.18;
  luxi_3d_navigation::TerrainModel terrain(tree, parameters);
  EXPECT_TRUE(terrain.isTraversable(terrain.worldToGrid(0.05, 0.05, 0.05)));
}

TEST(TerrainModel, DetectsObstacleInsideMetricRobotRadius)
{
  auto tree = makeGround(1);
  tree.updateNode(octomap::point3d(0.15F, 0.05F, 0.05F), true);
  tree.updateInnerOccupancy();
  auto parameters = testParameters();
  parameters.robot_radius = 0.18;
  luxi_3d_navigation::TerrainModel terrain(tree, parameters);
  EXPECT_FALSE(terrain.isTraversable(terrain.worldToGrid(0.05, 0.05, 0.05)));
}

TEST(TerrainModel, UsesConfiguredTwentyCentimeterRadiusAtBoundary)
{
  auto tree = makeGround(1);
  tree.updateNode(octomap::point3d(0.25F, 0.05F, 0.05F), true);
  tree.updateInnerOccupancy();
  auto parameters = testParameters();
  parameters.robot_radius = 0.20;
  luxi_3d_navigation::TerrainModel terrain(tree, parameters);
  EXPECT_FALSE(terrain.isTraversable(terrain.worldToGrid(0.05, 0.05, 0.05)));
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

TEST(TerrainModel, SegmentsObstaclesAndBuildsMonotonicEdgeCost)
{
  auto tree = makeGroundPatch(41, 41);
  for (int z = 0; z < 4; ++z) {
    tree.updateNode(octomap::point3d(2.05F, 2.05F, 0.05F + 0.1F * z), true);
  }
  tree.updateInnerOccupancy();
  auto parameters = testParameters();
  parameters.robot_radius = 0.20;
  parameters.costmap_margin = 0.60;
  luxi_3d_navigation::TerrainModel terrain(tree, parameters);

  const auto near_obstacle = terrain.worldToGrid(2.35, 2.05, 0.05);
  const auto farther_away = terrain.worldToGrid(2.75, 2.05, 0.05);
  const auto clear_center = terrain.worldToGrid(3.15, 2.05, 0.05);
  EXPECT_TRUE(terrain.isTraversable(near_obstacle));
  EXPECT_GT(terrain.traversalCost(near_obstacle), terrain.traversalCost(farther_away));
  EXPECT_GT(terrain.traversalCost(farther_away), terrain.traversalCost(clear_center));
  EXPECT_DOUBLE_EQ(terrain.traversalCost(clear_center), 0.0);
  EXPECT_FALSE(terrain.layers().obstacle_cells.empty());
  EXPECT_FALSE(terrain.layers().traversable_cells.empty());
}

TEST(TerrainModel, DoesNotSegmentGroundThicknessAsObstacle)
{
  auto tree = makeThickGroundPatch(9, 9);
  luxi_3d_navigation::TerrainModel terrain(tree, testParameters());

  EXPECT_FALSE(terrain.layers().traversable_cells.empty());
  EXPECT_TRUE(terrain.layers().obstacle_cells.empty());
}

TEST(TerrainModel, UsesPointCloudTerrainObservationForPlanningLayers)
{
  auto tree = makeGroundPatch(7, 7);
  tree.updateNode(octomap::point3d(0.35F, 0.35F, 0.05F), true);
  tree.updateInnerOccupancy();
  luxi_3d_navigation::TerrainObservation observation;
  for (int x = 0; x < 7; ++x) {
    for (int y = 0; y < 7; ++y) {
      observation.ground_cells.insert({x, y, -1});
    }
  }
  observation.obstacle_cells.insert({3, 3, 0});
  luxi_3d_navigation::TerrainModel terrain(
    tree, testParameters(), {}, observation);

  ASSERT_EQ(terrain.layers().obstacle_cells.size(), 1U);
  EXPECT_EQ(terrain.layers().obstacle_cells.front(), (luxi_3d_navigation::GridCell3D{3, 3, 0}));
  EXPECT_TRUE(terrain.isTraversable({1, 1, 0}));
  EXPECT_FALSE(terrain.isTraversable({3, 3, 0}));
}

TEST(TerrainModel, DoesNotTurnSinglePointCloudHoleIntoCostmapEdge)
{
  auto tree = makeGroundPatch(11, 11);
  luxi_3d_navigation::TerrainObservation observation;
  for (int x = 0; x < 11; ++x) {
    for (int y = 0; y < 11; ++y) {
      if (x != 5 || y != 5) {
        observation.ground_cells.insert({x, y, -1});
      }
    }
  }
  auto parameters = testParameters();
  parameters.costmap_margin = 0.40;
  luxi_3d_navigation::TerrainModel terrain(
    tree, parameters, {}, observation);

  EXPECT_DOUBLE_EQ(terrain.traversalCost({5, 4, 0}), 0.0);
  EXPECT_GT(terrain.traversalCost({1, 5, 0}), 0.0);
}

TEST(TerrainModel, PrefersInteriorRouteOverMapEdge)
{
  auto tree = makeGroundPatch(17, 11);
  auto parameters = testParameters();
  parameters.robot_radius = 0.0;
  parameters.costmap_margin = 0.40;
  parameters.costmap_weight = 8.0;
  luxi_3d_navigation::TerrainModel terrain(tree, parameters);
  const auto path = terrain.plan(
    terrain.worldToGrid(0.15, 0.15, 0.05),
    terrain.worldToGrid(1.55, 0.15, 0.05));

  ASSERT_FALSE(path.empty());
  EXPECT_TRUE(std::any_of(path.begin(), path.end(), [](const auto & cell) {
    return cell.y >= 4;
  }));
}
