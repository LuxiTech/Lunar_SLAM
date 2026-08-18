#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "octomap/AbstractOcTree.h"
#include "octomap/OcTree.h"

#include "luxi_3d_navigation/terrain_model.hpp"
#include "luxi_3d_navigation/terrain_cloud_classifier.hpp"

namespace
{

std::unique_ptr<octomap::OcTree> loadTree(const std::string & path)
{
  if (path.size() >= 3U && path.substr(path.size() - 3U) == ".bt") {
    auto tree = std::make_unique<octomap::OcTree>(0.1);
    if (!tree->readBinary(path)) {
      return nullptr;
    }
    return tree;
  }
  std::unique_ptr<octomap::AbstractOcTree> abstract_tree(octomap::AbstractOcTree::read(path));
  auto * raw_tree = dynamic_cast<octomap::OcTree *>(abstract_tree.get());
  if (raw_tree == nullptr) {
    return nullptr;
  }
  return std::unique_ptr<octomap::OcTree>(
    static_cast<octomap::OcTree *>(abstract_tree.release()));
}

template<typename Collection, typename Emit>
void emitBounded(const Collection & values, std::size_t limit, Emit emit)
{
  const std::size_t stride = limit == 0U ? 1U : std::max<std::size_t>(
    1U, (values.size() + limit - 1U) / limit);
  for (std::size_t index = 0U; index < values.size(); index += stride) {
    emit(values[index]);
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2 || argc > 9) {
    std::cerr <<
      "usage: terrain_map_to_points MAP.bt|MAP.ot [max_points] [robot_radius] "
      "[costmap_margin] [cloud.ply] [normal_radius] [max_ground_slope] "
      "[obstacle_min_height]\n";
    return 2;
  }
  try {
    const std::size_t max_points = argc >= 3 ?
      static_cast<std::size_t>(std::stoul(argv[2])) : 12000U;
    luxi_3d_navigation::TerrainParameters parameters;
    parameters.robot_radius = argc >= 4 ? std::stod(argv[3]) : 0.25;
    parameters.costmap_margin = argc >= 5 ? std::stod(argv[4]) : 0.60;
    parameters.costmap_weight = 8.0;
    if (parameters.robot_radius < 0.0 || parameters.costmap_margin < 0.0) {
      throw std::runtime_error("robot radius and costmap margin must be non-negative");
    }
    auto tree = loadTree(argv[1]);
    if (!tree) {
      throw std::runtime_error(std::string("cannot read OcTree: ") + argv[1]);
    }
    std::optional<luxi_3d_navigation::TerrainObservation> observation;
    if (argc >= 6 && std::string(argv[5]).size() > 0U) {
      luxi_3d_navigation::TerrainCloudParameters cloud_parameters;
      cloud_parameters.resolution = tree->getResolution();
      cloud_parameters.normal_radius = argc >= 7 ? std::stod(argv[6]) : 0.30;
      cloud_parameters.maximum_ground_slope_degrees = argc >= 8 ? std::stod(argv[7]) : 35.0;
      cloud_parameters.obstacle_min_height = argc >= 9 ? std::stod(argv[8]) : 0.15;
      observation = luxi_3d_navigation::classifyTerrainCloudFile(argv[5], cloud_parameters);
    }
    luxi_3d_navigation::TerrainModel terrain(
      *tree, parameters, {}, std::move(observation));
    std::cout << "resolution " << terrain.resolution() << '\n';
    emitBounded(
      terrain.layers().traversable_cells, max_points,
      [&terrain](const auto & entry) {
        const auto point = terrain.gridToWorld(entry.cell);
        std::cout << "traversable " << point.x() << ' ' << point.y() << ' ' << point.z() <<
          ' ' << entry.cost << '\n';
      });
    emitBounded(
      terrain.layers().obstacle_cells, max_points,
      [&terrain](const auto & cell) {
        const auto point = terrain.gridToWorld(cell);
        std::cout << "obstacle " << point.x() << ' ' << point.y() << ' ' << point.z() << " 1\n";
      });
  } catch (const std::exception & error) {
    std::cerr << "terrain_map_to_points: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
