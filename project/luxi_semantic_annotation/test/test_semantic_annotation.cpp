#include "luxi_semantic_annotation/semantic_annotation.hpp"

#include <gtest/gtest.h>
#include <octomap/OcTree.h>

#include <filesystem>
#include <fstream>

namespace
{

octomap::OcTree make_tree()
{
  octomap::OcTree tree(0.1);
  tree.updateNode(octomap::point3d(0.0F, 0.0F, 0.0F), true);
  tree.updateNode(octomap::point3d(0.0F, 0.0F, 0.3F), true);
  tree.updateNode(octomap::point3d(0.2F, 0.0F, 0.3F), true);
  tree.updateInnerOccupancy();
  return tree;
}

nlohmann::json valid_annotation()
{
  return {
    {"schema_version", 1},
    {"map_id", "map015"},
    {"frame_id", "map"},
    {"ground", {{"z", 0.0}, {"minimum_height", 0.15}}},
    {"occupied_labels", {{
      {"type", "rock"}, {"x", 0.0}, {"y", 0.0}, {"z", 0.3}, {"size", 0.1},
    }}},
    {"pits", {{
      {"id", "pit_001"},
      {"type", "pit"},
      {"depth", 0.4},
      {"polygon", {{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}}},
    }}},
  };
}

}  // namespace

TEST(SemanticAnnotation, InspectsGroundHeight)
{
  const auto tree = make_tree();
  const auto summary = luxi_semantic_annotation::inspect_octomap(tree);
  EXPECT_DOUBLE_EQ(summary.resolution, 0.1);
  EXPECT_EQ(summary.occupied_voxel_count, 3U);
  EXPECT_NEAR(summary.suggested_ground_z, 0.35, 1e-6);
}

TEST(SemanticAnnotation, AcceptsOccupiedLabelsAndPitPolygons)
{
  const auto tree = make_tree();
  const auto validated = luxi_semantic_annotation::validate_annotation(
    valid_annotation(), tree, "map015");
  EXPECT_EQ(validated["occupied_labels"].size(), 1U);
  EXPECT_EQ(validated["pits"].size(), 1U);
}

TEST(SemanticAnnotation, RejectsLabelsBelowGroundThreshold)
{
  const auto tree = make_tree();
  auto annotation = valid_annotation();
  annotation["occupied_labels"][0]["z"] = 0.0;
  EXPECT_THROW(
    luxi_semantic_annotation::validate_annotation(annotation, tree, "map015"),
    std::runtime_error);
}

TEST(SemanticAnnotation, RejectsUnoccupiedLabels)
{
  const auto tree = make_tree();
  auto annotation = valid_annotation();
  annotation["occupied_labels"][0]["x"] = 4.0;
  EXPECT_THROW(
    luxi_semantic_annotation::validate_annotation(annotation, tree, "map015"),
    std::runtime_error);
}

TEST(SemanticAnnotation, RejectsInvalidGeometryMetadata)
{
  const auto tree = make_tree();
  auto annotation = valid_annotation();
  annotation["occupied_labels"][0]["size"] = -0.1;
  EXPECT_THROW(
    luxi_semantic_annotation::validate_annotation(annotation, tree, "map015"),
    std::runtime_error);

  annotation = valid_annotation();
  annotation["pits"][0]["polygon"] = {{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}};
  EXPECT_THROW(
    luxi_semantic_annotation::validate_annotation(annotation, tree, "map015"),
    std::runtime_error);
}

TEST(SemanticAnnotation, SavesCanonicalJsonAtomically)
{
  const auto tree = make_tree();
  const auto annotation = luxi_semantic_annotation::validate_annotation(
    valid_annotation(), tree, "map015");
  const auto directory =
    std::filesystem::temp_directory_path() / "luxi_semantic_annotation_test";
  const auto output = directory / "annotations.json";
  luxi_semantic_annotation::save_annotation_atomic(output, annotation);
  std::ifstream stream(output);
  nlohmann::json loaded;
  stream >> loaded;
  EXPECT_EQ(loaded, annotation);
  std::filesystem::remove_all(directory);
}
