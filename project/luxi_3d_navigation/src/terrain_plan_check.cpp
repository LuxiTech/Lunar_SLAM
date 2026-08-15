#include <algorithm>
#include <chrono>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <queue>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "octomap/AbstractOcTree.h"
#include "octomap/OcTree.h"

#include "luxi_3d_navigation/terrain_model.hpp"
#include "luxi_3d_navigation/terrain_cloud_classifier.hpp"

int main(int argc, char ** argv)
{
  const auto started_at = std::chrono::steady_clock::now();
  if (argc < 2 || argc > 4) {
    std::cerr << "usage: terrain_plan_check MAP.bt|MAP.ot [cloud.ply] [--dynamic-test]\n";
    return 2;
  }
  const std::string map_path = argv[1];
  const bool dynamic_test = std::string(argv[argc - 1]) == "--dynamic-test";
  const int cloud_argument = argc >= 3 && std::string(argv[2]) != "--dynamic-test" ? 2 : -1;
  std::unique_ptr<octomap::OcTree> tree;
  if (map_path.size() >= 3U && map_path.substr(map_path.size() - 3U) == ".bt") {
    tree = std::make_unique<octomap::OcTree>(0.1);
    if (!tree->readBinary(map_path)) {
      tree.reset();
    }
  } else {
    std::unique_ptr<octomap::AbstractOcTree> abstract_tree(
      octomap::AbstractOcTree::read(map_path));
    auto * raw_tree = dynamic_cast<octomap::OcTree *>(abstract_tree.get());
    if (raw_tree != nullptr) {
      tree.reset(static_cast<octomap::OcTree *>(abstract_tree.release()));
    }
  }
  if (!tree) {
    std::cerr << "cannot read OcTree: " << argv[1] << '\n';
    return 2;
  }

  luxi_3d_navigation::TerrainParameters parameters;
  std::optional<luxi_3d_navigation::TerrainObservation> observation;
  if (cloud_argument > 0) {
    const auto classification_started_at = std::chrono::steady_clock::now();
    luxi_3d_navigation::TerrainCloudParameters cloud_parameters;
    cloud_parameters.resolution = tree->getResolution();
    observation = luxi_3d_navigation::classifyTerrainCloudFile(
      argv[cloud_argument], cloud_parameters);
    std::cout << "classification_seconds=" << std::chrono::duration<double>(
      std::chrono::steady_clock::now() - classification_started_at).count() << '\n';
  }
  const auto terrain_started_at = std::chrono::steady_clock::now();
  luxi_3d_navigation::TerrainModel terrain(
    *tree, parameters, {}, std::move(observation));
  std::cout << "terrain_seconds=" << std::chrono::duration<double>(
    std::chrono::steady_clock::now() - terrain_started_at).count() << '\n';
  std::map<std::pair<int, int>, luxi_3d_navigation::GridCell3D> lowest_cells;
  for (const auto & entry : terrain.layers().traversable_cells) {
    const auto & cell = entry.cell;
    const auto key = std::make_pair(cell.x, cell.y);
    const auto found = lowest_cells.find(key);
    if (found == lowest_cells.end() || cell.z < found->second.z) {
      lowest_cells[key] = cell;
    }
  }
  std::vector<luxi_3d_navigation::GridCell3D> cells;
  for (const auto & entry : lowest_cells) {
    cells.push_back(entry.second);
  }
  std::cout << "map=" << argv[1] << " resolution=" << tree->getResolution()
            << " supported_cells=" << cells.size() << '\n';
  if (cells.size() < 2U) {
    std::cerr << "no usable ground-supported terrain\n";
    return 1;
  }

  std::unordered_set<luxi_3d_navigation::GridCell3D,
    luxi_3d_navigation::GridCell3DHash> available(cells.begin(), cells.end());
  std::unordered_set<luxi_3d_navigation::GridCell3D,
    luxi_3d_navigation::GridCell3DHash> visited;
  std::vector<luxi_3d_navigation::GridCell3D> largest_component;
  for (const auto & seed : cells) {
    if (visited.count(seed) != 0U) {
      continue;
    }
    std::vector<luxi_3d_navigation::GridCell3D> component;
    std::queue<luxi_3d_navigation::GridCell3D> pending;
    pending.push(seed);
    visited.insert(seed);
    while (!pending.empty()) {
      const auto current = pending.front();
      pending.pop();
      component.push_back(current);
      for (int dz = -1; dz <= 1; ++dz) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            const luxi_3d_navigation::GridCell3D next{
              current.x + dx, current.y + dy, current.z + dz};
            if ((dx == 0 && dy == 0 && dz == 0) || available.count(next) == 0U ||
              visited.count(next) != 0U || !terrain.transitionAllowed(current, next))
            {
              continue;
            }
            visited.insert(next);
            pending.push(next);
          }
        }
      }
    }
    if (component.size() > largest_component.size()) {
      largest_component = std::move(component);
    }
  }
  std::cout << "largest_connected_component=" << largest_component.size() << '\n';
  if (largest_component.size() < 2U) {
    std::cerr << "no connected ground-supported route was found\n";
    return 1;
  }
  const auto start = largest_component.front();
  const auto goal = *std::max_element(
    largest_component.begin(), largest_component.end(), [&start](const auto & lhs, const auto & rhs) {
      return luxi_3d_navigation::gridDistance(lhs, start) <
             luxi_3d_navigation::gridDistance(rhs, start);
    });
  const auto path = terrain.plan(start, goal);
  if (path.size() < 2U) {
    std::cerr << "largest component exists but A* did not reproduce its route\n";
    return 1;
  }
  const auto start_world = terrain.gridToWorld(path.front());
  const auto goal_world = terrain.gridToWorld(path.back());
  std::cout << "path_cells=" << path.size() << " start=[" << start_world.x() << ','
            << start_world.y() << ',' << start_world.z() << "] goal=[" << goal_world.x()
            << ',' << goal_world.y() << ',' << goal_world.z() << "]\n";
  std::cout << "total_seconds=" << std::chrono::duration<double>(
    std::chrono::steady_clock::now() - started_at).count() << '\n';
  if (!dynamic_test) {
    return 0;
  }

  const std::size_t midpoint_index = path.size() / 2U;
  const int obstacle_radius_cells = std::max(
    1, static_cast<int>(std::ceil(0.20 / terrain.resolution())));
  luxi_3d_navigation::GridColumnSet blocked;
  const auto obstacle = path[midpoint_index];
  for (int dx = -obstacle_radius_cells; dx <= obstacle_radius_cells; ++dx) {
    for (int dy = -obstacle_radius_cells; dy <= obstacle_radius_cells; ++dy) {
      if (std::hypot(static_cast<double>(dx), static_cast<double>(dy)) <=
        static_cast<double>(obstacle_radius_cells))
      {
        blocked.insert(luxi_3d_navigation::GridCell3D{obstacle.x + dx, obstacle.y + dy, 0});
      }
    }
  }
  const std::size_t segment_cells = static_cast<std::size_t>(std::max(
    10, static_cast<int>(std::ceil(1.5 / terrain.resolution()))));
  const auto local_start = path[midpoint_index > segment_cells ? midpoint_index - segment_cells : 0U];
  const auto local_goal = path[std::min(path.size() - 1U, midpoint_index + segment_cells)];
  const int margin_cells = std::max(
    1, static_cast<int>(std::ceil(1.0 / terrain.resolution())));
  const luxi_3d_navigation::GridPlanningBounds bounds{
    std::min(local_start.x, local_goal.x) - margin_cells,
    std::max(local_start.x, local_goal.x) + margin_cells,
    std::min(local_start.y, local_goal.y) - margin_cells,
    std::max(local_start.y, local_goal.y) + margin_cells};
  const auto detour = terrain.planAvoidingColumns(local_start, local_goal, blocked, bounds);
  if (detour.empty()) {
    std::cout << "dynamic_result=safe_no_path obstacle_radius=0.20\n";
    return 0;
  }
  for (const auto & cell : detour) {
    if (blocked.count(luxi_3d_navigation::GridCell3D{cell.x, cell.y, 0}) != 0U) {
      std::cerr << "dynamic detour intersects the synthetic obstacle\n";
      return 1;
    }
  }
  std::cout << "dynamic_result=detour detour_cells=" << detour.size()
            << " obstacle_radius=0.20\n";
  return 0;
}
