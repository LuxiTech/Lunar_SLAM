#pragma once

#include <nlohmann/json.hpp>
#include <octomap/OcTree.h>

#include <cstddef>
#include <filesystem>
#include <memory>
#include <string>

namespace luxi_semantic_annotation
{

struct OctomapSummary
{
  double resolution{0.0};
  double suggested_ground_z{0.0};
  double minimum_z{0.0};
  double maximum_z{0.0};
  std::size_t occupied_voxel_count{0};
};

std::unique_ptr<octomap::OcTree> load_octomap(const std::filesystem::path & path);
OctomapSummary inspect_octomap(const octomap::OcTree & tree);
nlohmann::json summary_json(const OctomapSummary & summary);
nlohmann::json validate_annotation(
  const nlohmann::json & input,
  const octomap::OcTree & tree,
  const std::string & map_id);
void save_annotation_atomic(
  const std::filesystem::path & output,
  const nlohmann::json & annotation);

}  // namespace luxi_semantic_annotation
