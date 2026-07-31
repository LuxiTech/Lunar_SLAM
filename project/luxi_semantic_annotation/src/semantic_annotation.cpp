#include "luxi_semantic_annotation/semantic_annotation.hpp"

#include <octomap/AbstractOcTree.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <regex>
#include <set>
#include <stdexcept>

namespace luxi_semantic_annotation
{
namespace
{

constexpr std::size_t kMaximumOccupiedLabels = 20000;
constexpr std::size_t kMaximumPitRegions = 1000;
constexpr std::size_t kMaximumPitVertices = 5000;

double finite_number(const nlohmann::json & value, const std::string & name)
{
  if (!value.is_number()) {
    throw std::runtime_error(name + " must be a number");
  }
  const double result = value.get<double>();
  if (!std::isfinite(result)) {
    throw std::runtime_error(name + " must be finite");
  }
  return result;
}

std::string required_string(const nlohmann::json & value, const std::string & name)
{
  if (!value.is_string() || value.get<std::string>().empty()) {
    throw std::runtime_error(name + " must be a non-empty string");
  }
  return value.get<std::string>();
}

std::string voxel_key(const octomap::OcTreeKey & key)
{
  return std::to_string(key.k[0]) + ":" + std::to_string(key.k[1]) + ":" +
         std::to_string(key.k[2]);
}

}  // namespace

std::unique_ptr<octomap::OcTree> load_octomap(const std::filesystem::path & path)
{
  if (!std::filesystem::is_regular_file(path)) {
    throw std::runtime_error("OctoMap does not exist: " + path.string());
  }
  if (path.extension() == ".bt") {
    auto tree = std::make_unique<octomap::OcTree>(0.1);
    if (!tree->readBinary(path.string())) {
      throw std::runtime_error("cannot read binary OctoMap: " + path.string());
    }
    return tree;
  }
  octomap::AbstractOcTree * raw = octomap::AbstractOcTree::read(path.string());
  auto * tree = dynamic_cast<octomap::OcTree *>(raw);
  if (tree == nullptr) {
    delete raw;
    throw std::runtime_error("OctoMap is not an OcTree: " + path.string());
  }
  return std::unique_ptr<octomap::OcTree>(tree);
}

OctomapSummary inspect_octomap(const octomap::OcTree & tree)
{
  OctomapSummary summary;
  summary.resolution = tree.getResolution();
  summary.minimum_z = std::numeric_limits<double>::infinity();
  summary.maximum_z = -std::numeric_limits<double>::infinity();
  struct HeightBin
  {
    std::size_t count{0};
    double sum{0.0};
  };
  std::map<long long, HeightBin> height_histogram;
  for (auto iterator = tree.begin_leafs(); iterator != tree.end_leafs(); ++iterator) {
    if (!tree.isNodeOccupied(*iterator)) {
      continue;
    }
    const double z = iterator.getZ();
    ++summary.occupied_voxel_count;
    summary.minimum_z = std::min(summary.minimum_z, z);
    summary.maximum_z = std::max(summary.maximum_z, z);
    auto & bin = height_histogram[std::llround(z / summary.resolution)];
    ++bin.count;
    bin.sum += z;
  }
  if (summary.occupied_voxel_count == 0) {
    throw std::runtime_error("OctoMap contains no occupied leaf voxels");
  }
  const auto ground_bin = std::max_element(
    height_histogram.begin(), height_histogram.end(),
    [](const auto & left, const auto & right) {
      if (left.second.count != right.second.count) {
        return left.second.count < right.second.count;
      }
      return left.first > right.first;
    });
  summary.suggested_ground_z = ground_bin->second.sum / ground_bin->second.count;
  return summary;
}

nlohmann::json summary_json(const OctomapSummary & summary)
{
  return {
    {"resolution", summary.resolution},
    {"suggested_ground_z", summary.suggested_ground_z},
    {"minimum_z", summary.minimum_z},
    {"maximum_z", summary.maximum_z},
    {"occupied_voxel_count", summary.occupied_voxel_count},
  };
}

nlohmann::json validate_annotation(
  const nlohmann::json & input,
  const octomap::OcTree & tree,
  const std::string & map_id)
{
  if (!std::regex_match(map_id, std::regex("^map[0-9]+$"))) {
    throw std::runtime_error("map_id must use the mapNNN format");
  }
  if (!input.is_object()) {
    throw std::runtime_error("annotation root must be an object");
  }
  if (input.value("map_id", std::string()) != map_id) {
    throw std::runtime_error("annotation map_id does not match selected map");
  }
  if (input.value("schema_version", 0) != 1) {
    throw std::runtime_error("unsupported annotation schema_version");
  }
  if (input.value("frame_id", std::string()) != "map") {
    throw std::runtime_error("annotation frame_id must be map");
  }
  if (!input.contains("ground") || !input["ground"].is_object()) {
    throw std::runtime_error("ground must be an object");
  }
  const double ground_z = finite_number(input["ground"].at("z"), "ground.z");
  const double minimum_height =
    finite_number(input["ground"].at("minimum_height"), "ground.minimum_height");
  if (minimum_height < 0.0) {
    throw std::runtime_error("ground.minimum_height cannot be negative");
  }

  const auto labels = input.value("occupied_labels", nlohmann::json::array());
  if (!labels.is_array() || labels.size() > kMaximumOccupiedLabels) {
    throw std::runtime_error("occupied_labels must be a bounded array");
  }
  nlohmann::json canonical_labels = nlohmann::json::array();
  std::set<std::string> selected_voxels;
  for (const auto & label : labels) {
    if (!label.is_object()) {
      throw std::runtime_error("occupied label must be an object");
    }
    const std::string type = required_string(label.at("type"), "occupied label type");
    if (type != "rock" && type != "wall") {
      throw std::runtime_error("occupied label type must be rock or wall");
    }
    const double x = finite_number(label.at("x"), "occupied label x");
    const double y = finite_number(label.at("y"), "occupied label y");
    const double z = finite_number(label.at("z"), "occupied label z");
    if (z + 1e-9 < ground_z + minimum_height) {
      throw std::runtime_error("occupied label is below the configured height threshold");
    }
    octomap::OcTreeKey key;
    if (!tree.coordToKeyChecked(octomap::point3d(x, y, z), key)) {
      throw std::runtime_error("occupied label is outside the OctoMap bounds");
    }
    const octomap::OcTreeNode * node = tree.search(key);
    if (node == nullptr || !tree.isNodeOccupied(node)) {
      throw std::runtime_error("occupied label does not reference an occupied voxel");
    }
    if (!selected_voxels.insert(voxel_key(key)).second) {
      throw std::runtime_error("an occupied voxel cannot have multiple labels");
    }
    const double size = finite_number(
      label.value("size", tree.getResolution()), "occupied label size");
    if (size <= 0.0) {
      throw std::runtime_error("occupied label size must be positive");
    }
    canonical_labels.push_back({
      {"type", type},
      {"x", x},
      {"y", y},
      {"z", z},
      {"size", size},
    });
  }

  const auto pits = input.value("pits", nlohmann::json::array());
  if (!pits.is_array() || pits.size() > kMaximumPitRegions) {
    throw std::runtime_error("pits must be a bounded array");
  }
  nlohmann::json canonical_pits = nlohmann::json::array();
  std::set<std::string> pit_ids;
  for (const auto & pit : pits) {
    if (!pit.is_object()) {
      throw std::runtime_error("pit region must be an object");
    }
    const std::string id = required_string(pit.at("id"), "pit id");
    if (pit.value("type", std::string()) != "pit") {
      throw std::runtime_error("pit region type must be pit");
    }
    if (!pit_ids.insert(id).second) {
      throw std::runtime_error("pit ids must be unique");
    }
    const double depth = finite_number(pit.at("depth"), "pit depth");
    if (depth <= 0.0) {
      throw std::runtime_error("pit depth must be positive");
    }
    const auto & polygon = pit.at("polygon");
    if (!polygon.is_array() || polygon.size() < 3 ||
      polygon.size() > kMaximumPitVertices)
    {
      throw std::runtime_error("pit polygon must contain between 3 and 5000 vertices");
    }
    nlohmann::json canonical_polygon = nlohmann::json::array();
    double twice_area = 0.0;
    for (const auto & vertex : polygon) {
      if (!vertex.is_array() || vertex.size() != 2) {
        throw std::runtime_error("pit polygon vertex must be [x, y]");
      }
      canonical_polygon.push_back({
        finite_number(vertex[0], "pit polygon x"),
        finite_number(vertex[1], "pit polygon y"),
      });
    }
    for (std::size_t index = 0; index < canonical_polygon.size(); ++index) {
      const auto & current = canonical_polygon[index];
      const auto & next = canonical_polygon[(index + 1) % canonical_polygon.size()];
      twice_area += current[0].get<double>() * next[1].get<double>() -
        next[0].get<double>() * current[1].get<double>();
    }
    if (std::abs(twice_area) < 1.0e-8) {
      throw std::runtime_error("pit polygon must enclose a non-zero area");
    }
    canonical_pits.push_back({
      {"id", id},
      {"type", "pit"},
      {"depth", depth},
      {"polygon", canonical_polygon},
    });
  }

  return {
    {"schema_version", 1},
    {"map_id", map_id},
    {"frame_id", "map"},
    {"ground", {{"z", ground_z}, {"minimum_height", minimum_height}}},
    {"occupied_labels", canonical_labels},
    {"pits", canonical_pits},
  };
}

void save_annotation_atomic(
  const std::filesystem::path & output,
  const nlohmann::json & annotation)
{
  std::filesystem::create_directories(output.parent_path());
  const std::filesystem::path temporary = output.string() + ".tmp";
  {
    std::ofstream stream(temporary, std::ios::trunc);
    if (!stream) {
      throw std::runtime_error("cannot open temporary annotation file");
    }
    stream << annotation.dump(2) << '\n';
    if (!stream) {
      throw std::runtime_error("cannot write temporary annotation file");
    }
  }
  std::error_code error;
  std::filesystem::rename(temporary, output, error);
  if (error) {
    std::filesystem::remove(output, error);
    error.clear();
    std::filesystem::rename(temporary, output, error);
  }
  if (error) {
    std::filesystem::remove(temporary);
    throw std::runtime_error("cannot replace annotation file: " + error.message());
  }
}

}  // namespace luxi_semantic_annotation
