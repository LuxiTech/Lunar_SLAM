#ifndef LUXI_VOXEL_NAVIGATION__VOXEL_ASTAR_HPP_
#define LUXI_VOXEL_NAVIGATION__VOXEL_ASTAR_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

namespace luxi_voxel_navigation
{

struct GridCell
{
  int x;
  int y;

  bool operator==(const GridCell & other) const;
};

std::vector<GridCell> planAstar(
  int width,
  int height,
  const std::vector<std::uint8_t> & blocked,
  GridCell start,
  GridCell goal,
  bool allow_diagonal);

std::size_t gridOffset(int width, GridCell cell);
bool isInsideGrid(int width, int height, GridCell cell);

}  // namespace luxi_voxel_navigation

#endif  // LUXI_VOXEL_NAVIGATION__VOXEL_ASTAR_HPP_
