#include "map_cloud_filter.hpp"

#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include <pcl/common/point_tests.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <yaml-cpp/yaml.h>

namespace luxi_map_tools
{

FilterParameters loadFilterParameters(
  const std::filesystem::path & path, FilterParameters parameters)
{
  const YAML::Node document = YAML::LoadFile(path.string());
  const YAML::Node values = document["map_cloud_filter"];
  if (!values || !values.IsMap()) {
    throw std::runtime_error(
            "config must contain a map_cloud_filter mapping: " + path.string());
  }
  const std::unordered_set<std::string> allowed_keys{
    "mean_k", "standard_deviation", "radius", "minimum_neighbors",
    "cluster_tolerance", "minimum_cluster_size", "minimum_z", "maximum_z"};
  for (const auto & entry : values) {
    const auto key = entry.first.as<std::string>();
    if (allowed_keys.count(key) == 0U) {
      throw std::runtime_error("unknown filter config key: " + key);
    }
  }

  if (values["mean_k"]) {
    parameters.mean_k = values["mean_k"].as<int>();
  }
  if (values["standard_deviation"]) {
    parameters.standard_deviation = values["standard_deviation"].as<double>();
  }
  if (values["radius"]) {
    parameters.radius = values["radius"].as<double>();
  }
  if (values["minimum_neighbors"]) {
    parameters.minimum_neighbors = values["minimum_neighbors"].as<int>();
  }
  if (values["cluster_tolerance"]) {
    parameters.cluster_tolerance = values["cluster_tolerance"].as<double>();
  }
  if (values["minimum_cluster_size"]) {
    parameters.minimum_cluster_size = values["minimum_cluster_size"].as<int>();
  }
  if (values["minimum_z"]) {
    parameters.minimum_z = values["minimum_z"].as<double>();
  }
  if (values["maximum_z"]) {
    parameters.maximum_z = values["maximum_z"].as<double>();
  }
  validateFilterParameters(parameters);
  return parameters;
}

void validateFilterParameters(const FilterParameters & parameters)
{
  if (
    parameters.mean_k < 0 || parameters.standard_deviation <= 0.0 ||
    parameters.radius < 0.0 || parameters.minimum_neighbors < 0 ||
    parameters.cluster_tolerance < 0.0 || parameters.minimum_cluster_size < 0 ||
    parameters.minimum_z > parameters.maximum_z)
  {
    throw std::runtime_error("filter parameters are outside their valid ranges");
  }
}

MapCloud::Ptr filterMapCloud(
  const MapCloud::ConstPtr & input, const FilterParameters & parameters,
  FilterReport & report)
{
  report.input_points = input->size();
  auto finite = std::make_shared<MapCloud>();
  finite->reserve(input->size());
  for (const auto & point : *input) {
    if (pcl::isFinite(point)) {
      finite->push_back(point);
    }
  }
  finite->is_dense = true;
  report.finite_points = finite->size();

  auto current = std::make_shared<MapCloud>();
  current->reserve(finite->size());
  for (const auto & point : *finite) {
    if (point.z >= parameters.minimum_z && point.z <= parameters.maximum_z) {
      current->push_back(point);
    }
  }
  current->is_dense = true;
  report.after_height_filter = current->size();

  if (parameters.mean_k > 0 && current->size() > static_cast<std::size_t>(parameters.mean_k)) {
    pcl::StatisticalOutlierRemoval<MapPoint> filter;
    filter.setInputCloud(current);
    filter.setMeanK(parameters.mean_k);
    filter.setStddevMulThresh(parameters.standard_deviation);
    auto filtered = std::make_shared<MapCloud>();
    filter.filter(*filtered);
    current = filtered;
  }
  report.after_statistical_filter = current->size();

  if (parameters.radius > 0.0 && parameters.minimum_neighbors > 0 && !current->empty()) {
    pcl::RadiusOutlierRemoval<MapPoint> filter;
    filter.setInputCloud(current);
    filter.setRadiusSearch(parameters.radius);
    filter.setMinNeighborsInRadius(parameters.minimum_neighbors);
    auto filtered = std::make_shared<MapCloud>();
    filter.filter(*filtered);
    current = filtered;
  }
  report.after_radius_filter = current->size();

  if (
    parameters.cluster_tolerance > 0.0 && parameters.minimum_cluster_size > 1 &&
    !current->empty())
  {
    auto search = std::make_shared<pcl::search::KdTree<MapPoint>>();
    search->setInputCloud(current);
    pcl::EuclideanClusterExtraction<MapPoint> extraction;
    extraction.setInputCloud(current);
    extraction.setSearchMethod(search);
    extraction.setClusterTolerance(parameters.cluster_tolerance);
    extraction.setMinClusterSize(parameters.minimum_cluster_size);
    extraction.setMaxClusterSize(static_cast<int>(current->size()));
    std::vector<pcl::PointIndices> clusters;
    extraction.extract(clusters);

    auto filtered = std::make_shared<MapCloud>();
    filtered->reserve(current->size());
    for (const auto & cluster : clusters) {
      for (const int index : cluster.indices) {
        filtered->push_back((*current)[static_cast<std::size_t>(index)]);
      }
    }
    filtered->is_dense = true;
    current = filtered;
  }
  report.output_points = current->size();
  return current;
}

}  // namespace luxi_map_tools
