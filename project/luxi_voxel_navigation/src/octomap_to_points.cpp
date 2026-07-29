#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "octomap/AbstractOcTree.h"
#include "octomap/OcTree.h"

namespace
{
std::size_t parseMaximum(const char * value)
{
  try {
    const auto parsed = std::stoll(value);
    if (parsed <= 0) {
      throw std::runtime_error("maximum point count must be positive");
    }
    return static_cast<std::size_t>(parsed);
  } catch (const std::invalid_argument &) {
    throw std::runtime_error("maximum point count must be an integer");
  } catch (const std::out_of_range &) {
    throw std::runtime_error("maximum point count is out of range");
  }
}

std::unique_ptr<octomap::OcTree> loadTree(const std::string & path)
{
  if (path.size() >= 3 && path.substr(path.size() - 3) == ".bt") {
    auto tree = std::make_unique<octomap::OcTree>(0.1);
    if (tree->readBinary(path)) {
      return tree;
    }
    return nullptr;
  }
  std::unique_ptr<octomap::AbstractOcTree> raw_tree(octomap::AbstractOcTree::read(path));
  auto * tree = dynamic_cast<octomap::OcTree *>(raw_tree.release());
  return std::unique_ptr<octomap::OcTree>(tree);
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 3) {
    std::cerr << "Usage: octomap_to_points INPUT.bt MAX_POINTS\n";
    return 2;
  }
  try {
    const std::size_t maximum = parseMaximum(argv[2]);
    auto tree = loadTree(argv[1]);
    if (!tree) {
      throw std::runtime_error(std::string("cannot load OctoMap: ") + argv[1]);
    }

    struct Voxel { double x; double y; double z; double size; };
    std::vector<Voxel> occupied;
    for (auto iterator = tree->begin_leafs(); iterator != tree->end_leafs(); ++iterator) {
      if (!tree->isNodeOccupied(*iterator)) {
        continue;
      }
      occupied.push_back({iterator.getX(), iterator.getY(), iterator.getZ(), iterator.getSize()});
    }
    const std::size_t stride = std::max<std::size_t>(
      1, (occupied.size() + maximum - 1) / maximum);
    std::cout << "resolution " << tree->getResolution() << '\n';
    for (std::size_t index = 0; index < occupied.size(); index += stride) {
      const auto & voxel = occupied[index];
      std::cout << voxel.x << ' ' << voxel.y << ' ' << voxel.z << ' ' << voxel.size << '\n';
    }
  } catch (const std::exception & error) {
    std::cerr << "octomap_to_points: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
