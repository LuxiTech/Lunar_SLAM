#include "luxi_voxel_navigation/voxel_astar.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

namespace luxi_voxel_navigation
{
namespace
{
struct QueueEntry
{
  GridCell cell;
  double score;
};

struct QueueCompare
{
  bool operator()(const QueueEntry & left, const QueueEntry & right) const
  {
    return left.score > right.score;
  }
};

double heuristic(const GridCell & left, const GridCell & right)
{
  return std::hypot(
    static_cast<double>(left.x - right.x),
    static_cast<double>(left.y - right.y));
}
}  // namespace

bool GridCell::operator==(const GridCell & other) const
{
  return x == other.x && y == other.y;
}

std::size_t gridOffset(int width, GridCell cell)
{
  return static_cast<std::size_t>(cell.y * width + cell.x);
}

bool isInsideGrid(int width, int height, GridCell cell)
{
  return cell.x >= 0 && cell.y >= 0 && cell.x < width && cell.y < height;
}

std::vector<GridCell> planAstar(
  int width,
  int height,
  const std::vector<std::uint8_t> & blocked,
  GridCell start,
  GridCell goal,
  bool allow_diagonal)
{
  if (width <= 0 || height <= 0 ||
    blocked.size() != static_cast<std::size_t>(width * height) ||
    !isInsideGrid(width, height, start) || !isInsideGrid(width, height, goal) ||
    blocked[gridOffset(width, start)] != 0 || blocked[gridOffset(width, goal)] != 0)
  {
    return {};
  }

  const std::vector<GridCell> offsets = allow_diagonal ?
    std::vector<GridCell>{{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}} :
    std::vector<GridCell>{{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
  const auto cell_count = static_cast<std::size_t>(width * height);
  const double infinity = std::numeric_limits<double>::infinity();
  std::vector<double> cost(cell_count, infinity);
  std::vector<int> parent(cell_count, -1);
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueCompare> open;

  cost[gridOffset(width, start)] = 0.0;
  open.push(QueueEntry{start, heuristic(start, goal)});
  while (!open.empty()) {
    const QueueEntry current = open.top();
    open.pop();
    const std::size_t current_offset = gridOffset(width, current.cell);
    if (current.score > cost[current_offset] + heuristic(current.cell, goal) + 1.0e-9) {
      continue;
    }
    if (current.cell == goal) {
      std::vector<GridCell> path;
      for (int offset = static_cast<int>(gridOffset(width, goal)); offset >= 0; offset = parent[offset]) {
        path.push_back(GridCell{offset % width, offset / width});
        if (offset == static_cast<int>(gridOffset(width, start))) {
          break;
        }
      }
      std::reverse(path.begin(), path.end());
      return path;
    }

    for (const GridCell & delta : offsets) {
      const GridCell next{current.cell.x + delta.x, current.cell.y + delta.y};
      if (!isInsideGrid(width, height, next) || blocked[gridOffset(width, next)] != 0) {
        continue;
      }
      if (delta.x != 0 && delta.y != 0) {
        const GridCell side_x{current.cell.x + delta.x, current.cell.y};
        const GridCell side_y{current.cell.x, current.cell.y + delta.y};
        if (blocked[gridOffset(width, side_x)] != 0 || blocked[gridOffset(width, side_y)] != 0) {
          continue;
        }
      }
      const std::size_t next_offset = gridOffset(width, next);
      const double next_cost = cost[current_offset] + std::hypot(delta.x, delta.y);
      if (next_cost >= cost[next_offset]) {
        continue;
      }
      cost[next_offset] = next_cost;
      parent[next_offset] = static_cast<int>(current_offset);
      open.push(QueueEntry{next, next_cost + heuristic(next, goal)});
    }
  }
  return {};
}

}  // namespace luxi_voxel_navigation
