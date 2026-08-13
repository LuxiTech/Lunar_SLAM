#include <cmath>
#include <filesystem>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

#include <pcl/common/common.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "octomap/OcTree.h"

#include "luxi_voxel_navigation/octomap_defaults.hpp"

namespace
{
double parseDouble(const char * value, const char * name)
{
  try {
    return std::stod(value);
  } catch (const std::exception &) {
    throw std::runtime_error(std::string("invalid ") + name + ": " + value);
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 3 || argc > 6) {
    std::cerr << "Usage: ply_to_octomap INPUT.ply OUTPUT.bt [resolution] [min_z] [max_z]\n";
    return 2;
  }
  try {
    const double resolution = argc >= 4 ? parseDouble(argv[3], "resolution") :
      luxi_voxel_navigation::kDefaultOctomapResolution;
    const double min_z = argc >= 5 ? parseDouble(argv[4], "min_z") : -std::numeric_limits<double>::infinity();
    const double max_z = argc >= 6 ? parseDouble(argv[5], "max_z") : std::numeric_limits<double>::infinity();
    if (resolution <= 0.0 || min_z > max_z) {
      throw std::runtime_error("resolution must be positive and min_z must not exceed max_z");
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPLYFile(argv[1], cloud) < 0) {
      throw std::runtime_error(std::string("cannot read PLY: ") + argv[1]);
    }
    octomap::OcTree tree(resolution);
    std::size_t accepted = 0;
    for (const pcl::PointXYZ & point : cloud.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
        point.z < min_z || point.z > max_z)
      {
        continue;
      }
      tree.updateNode(octomap::point3d(point.x, point.y, point.z), true);
      ++accepted;
    }
    if (accepted == 0) {
      throw std::runtime_error("the PLY contains no finite points in the requested z range");
    }
    tree.updateInnerOccupancy();
    const std::filesystem::path output(argv[2]);
    std::filesystem::create_directories(output.parent_path());
    if (!tree.writeBinary(output.string())) {
      throw std::runtime_error(std::string("cannot write OctoMap: ") + output.string());
    }
    std::cout << "OctoMap written: " << output << " source_points=" << cloud.size()
              << " accepted=" << accepted << " occupied_voxels=" << tree.size() << '\n';
  } catch (const std::exception & error) {
    std::cerr << "ply_to_octomap: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
