#include <algorithm>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "octomap/AbstractOcTree.h"
#include "octomap/OcTree.h"

#include "luxi_3d_navigation/terrain_model.hpp"
#include "luxi_3d_navigation/terrain_cloud_classifier.hpp"

int main(int argc, char ** argv)
{
  if (argc < 2 || argc > 3) {
    std::cerr << "usage: terrain_plan_check MAP.bt|MAP.ot [cloud.ply]\n";
    return 2;
  }
  const std::string map_path = argv[1];
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
  if (argc == 3) {
    luxi_3d_navigation::TerrainCloudParameters cloud_parameters;
    cloud_parameters.resolution = tree->getResolution();
    observation = luxi_3d_navigation::classifyTerrainCloudFile(argv[2], cloud_parameters);
  }
  luxi_3d_navigation::TerrainModel terrain(
    *tree, parameters, {}, std::move(observation));
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

  const auto start = cells.front();
  std::sort(cells.begin(), cells.end(), [&start](const auto & lhs, const auto & rhs) {
    return luxi_3d_navigation::gridDistance(lhs, start) >
           luxi_3d_navigation::gridDistance(rhs, start);
  });
  for (std::size_t index = 0U; index < std::min<std::size_t>(cells.size(), 100U); ++index) {
    const auto path = terrain.plan(start, cells[index]);
    if (path.size() >= 2U) {
      const auto start_world = terrain.gridToWorld(path.front());
      const auto goal_world = terrain.gridToWorld(path.back());
      std::cout << "path_cells=" << path.size() << " start=[" << start_world.x() << ','
                << start_world.y() << ',' << start_world.z() << "] goal=[" << goal_world.x()
                << ',' << goal_world.y() << ',' << goal_world.z() << "]\n";
      return 0;
    }
  }
  std::cerr << "supported cells exist but no connected test route was found\n";
  return 1;
}
