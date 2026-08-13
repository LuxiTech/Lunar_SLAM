#pragma once

#include <cstddef>
#include <filesystem>
#include <limits>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace luxi_map_tools
{

using MapPoint = pcl::PointXYZRGBNormal;
using MapCloud = pcl::PointCloud<MapPoint>;

struct FilterParameters
{
  int mean_k{20};
  double standard_deviation{2.0};
  double radius{0.12};
  int minimum_neighbors{4};
  double cluster_tolerance{0.12};
  int minimum_cluster_size{20};
  double minimum_z{-std::numeric_limits<double>::infinity()};
  double maximum_z{std::numeric_limits<double>::infinity()};
};

struct FilterReport
{
  std::size_t input_points{};
  std::size_t finite_points{};
  std::size_t after_height_filter{};
  std::size_t after_statistical_filter{};
  std::size_t after_radius_filter{};
  std::size_t output_points{};
};

FilterParameters loadFilterParameters(
  const std::filesystem::path & path,
  FilterParameters parameters = FilterParameters{});

void validateFilterParameters(const FilterParameters & parameters);

MapCloud::Ptr filterMapCloud(
  const MapCloud::ConstPtr & input, const FilterParameters & parameters,
  FilterReport & report);

}  // namespace luxi_map_tools
