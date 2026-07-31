#include <unordered_set>

#include "gtest/gtest.h"
#include "luxi_3d_navigation/astar_3d.hpp"

using luxi_3d_navigation::GridCell3D;
using luxi_3d_navigation::GridCell3DHash;

TEST(Astar3D, PlansAcrossHeightChange)
{
  std::unordered_set<GridCell3D, GridCell3DHash> cells;
  for (int x = 0; x < 5; ++x) {
    cells.insert(GridCell3D{x, 0, x < 2 ? 0 : 1});
  }
  const auto path = luxi_3d_navigation::planAstar3D(
    GridCell3D{0, 0, 0}, GridCell3D{4, 0, 1},
    [&cells](const auto & cell) {return cells.count(cell) != 0U;},
    [](const auto & from, const auto & to) {
      return !(from.x == to.x && from.y == to.y) && std::abs(from.z - to.z) <= 1;
    },
    [](const auto &) {return 0.0;}, 100U);
  ASSERT_EQ(path.size(), 5U);
  EXPECT_EQ(path.front().z, 0);
  EXPECT_EQ(path.back().z, 1);
}

TEST(Astar3D, RejectsUnsupportedGap)
{
  const auto path = luxi_3d_navigation::planAstar3D(
    GridCell3D{0, 0, 0}, GridCell3D{2, 0, 0},
    [](const auto & cell) {return cell == GridCell3D{0, 0, 0} || cell == GridCell3D{2, 0, 0};},
    [](const auto &, const auto &) {return true;},
    [](const auto &) {return 0.0;}, 100U);
  EXPECT_TRUE(path.empty());
}

TEST(Astar3D, AvoidsHighCostCell)
{
  const auto path = luxi_3d_navigation::planAstar3D(
    GridCell3D{0, 0, 0}, GridCell3D{2, 0, 0},
    [](const auto & cell) {
      return cell.z == 0 && cell.x >= 0 && cell.x <= 2 && cell.y >= 0 && cell.y <= 1;
    },
    [](const auto & from, const auto & to) {return !(from.x == to.x && from.y == to.y);},
    [](const auto & cell) {return cell == GridCell3D{1, 0, 0} ? 10.0 : 0.0;}, 100U);
  ASSERT_FALSE(path.empty());
  EXPECT_EQ(path[1].y, 1);
}
