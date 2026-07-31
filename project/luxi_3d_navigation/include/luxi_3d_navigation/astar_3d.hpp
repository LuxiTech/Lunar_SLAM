#pragma once

#include <cstddef>
#include <functional>
#include <vector>

namespace luxi_3d_navigation
{

struct GridCell3D
{
  int x{};
  int y{};
  int z{};

  bool operator==(const GridCell3D & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct GridCell3DHash
{
  std::size_t operator()(const GridCell3D & cell) const;
};

using CellPredicate = std::function<bool(const GridCell3D &)>;
using TransitionPredicate = std::function<bool(const GridCell3D &, const GridCell3D &)>;
using CellCost = std::function<double(const GridCell3D &)>;

std::vector<GridCell3D> planAstar3D(
  const GridCell3D & start,
  const GridCell3D & goal,
  const CellPredicate & traversable,
  const TransitionPredicate & transition_allowed,
  const CellCost & cell_cost,
  std::size_t max_iterations);

double gridDistance(const GridCell3D & lhs, const GridCell3D & rhs);

}  // namespace luxi_3d_navigation
