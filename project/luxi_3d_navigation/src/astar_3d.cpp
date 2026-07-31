#include "luxi_3d_navigation/astar_3d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace luxi_3d_navigation
{
namespace
{

struct OpenEntry
{
  GridCell3D cell;
  double g;
  double f;
};

struct GreaterF
{
  bool operator()(const OpenEntry & lhs, const OpenEntry & rhs) const
  {
    return lhs.f > rhs.f;
  }
};

}  // namespace

std::size_t GridCell3DHash::operator()(const GridCell3D & cell) const
{
  const auto h1 = std::hash<int>{}(cell.x);
  const auto h2 = std::hash<int>{}(cell.y);
  const auto h3 = std::hash<int>{}(cell.z);
  return h1 ^ (h2 << 1U) ^ (h3 << 2U);
}

double gridDistance(const GridCell3D & lhs, const GridCell3D & rhs)
{
  const double dx = static_cast<double>(lhs.x - rhs.x);
  const double dy = static_cast<double>(lhs.y - rhs.y);
  const double dz = static_cast<double>(lhs.z - rhs.z);
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::vector<GridCell3D> planAstar3D(
  const GridCell3D & start,
  const GridCell3D & goal,
  const CellPredicate & traversable,
  const TransitionPredicate & transition_allowed,
  const CellCost & cell_cost,
  std::size_t max_iterations)
{
  if (!traversable(start) || !traversable(goal) || max_iterations == 0U) {
    return {};
  }

  std::priority_queue<OpenEntry, std::vector<OpenEntry>, GreaterF> open;
  std::unordered_map<GridCell3D, double, GridCell3DHash> scores;
  std::unordered_map<GridCell3D, GridCell3D, GridCell3DHash> parents;
  std::unordered_set<GridCell3D, GridCell3DHash> closed;
  scores[start] = 0.0;
  open.push(OpenEntry{start, 0.0, gridDistance(start, goal)});

  std::size_t iterations = 0U;
  while (!open.empty() && iterations++ < max_iterations) {
    const OpenEntry current = open.top();
    open.pop();
    if (closed.find(current.cell) != closed.end()) {
      continue;
    }
    if (current.cell == goal) {
      std::vector<GridCell3D> path{goal};
      GridCell3D cursor = goal;
      while (!(cursor == start)) {
        const auto parent = parents.find(cursor);
        if (parent == parents.end()) {
          return {};
        }
        cursor = parent->second;
        path.push_back(cursor);
      }
      std::reverse(path.begin(), path.end());
      return path;
    }
    closed.insert(current.cell);

    for (int dz = -1; dz <= 1; ++dz) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          const GridCell3D next{
            current.cell.x + dx, current.cell.y + dy, current.cell.z + dz};
          if (closed.find(next) != closed.end() || !traversable(next) ||
            !transition_allowed(current.cell, next))
          {
            continue;
          }
          const double extra_cost = std::max(0.0, cell_cost(next));
          const double tentative = current.g + gridDistance(current.cell, next) + extra_cost;
          const auto previous = scores.find(next);
          if (previous != scores.end() && tentative >= previous->second) {
            continue;
          }
          scores[next] = tentative;
          parents[next] = current.cell;
          open.push(OpenEntry{next, tentative, tentative + gridDistance(next, goal)});
        }
      }
    }
  }
  return {};
}

}  // namespace luxi_3d_navigation
