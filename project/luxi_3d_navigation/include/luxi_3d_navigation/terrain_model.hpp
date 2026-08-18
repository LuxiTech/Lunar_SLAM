#pragma once

#include <cstddef>
#include <optional>
#include <utility>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "octomap/OcTree.h"

#include "luxi_3d_navigation/astar_3d.hpp"

namespace luxi_3d_navigation
{

struct TerrainParameters
{
  double robot_radius{0.25};
  double robot_height{0.35};
  double max_step_height{0.15};
  double max_slope_degrees{50.0};
  int support_xy_radius_cells{1};
  int support_depth_cells{2};
  bool strict_direct_support{false};
  int snap_radius_cells{12};
  std::size_t max_iterations{500000U};
  double costmap_margin{0.60};
  double costmap_weight{8.0};
};

using Polygon2D = std::vector<std::pair<double, double>>;
using GridColumnSet = std::unordered_set<GridCell3D, GridCell3DHash>;

struct GridPlanningBounds
{
  int min_x{};
  int max_x{};
  int min_y{};
  int max_y{};
};

struct TerrainCellCost
{
  GridCell3D cell;
  double cost{};
};

struct TerrainLayers
{
  std::vector<GridCell3D> obstacle_cells;
  std::vector<TerrainCellCost> traversable_cells;
};

struct TerrainObservation
{
  std::unordered_set<GridCell3D, GridCell3DHash> ground_cells;
  std::unordered_set<GridCell3D, GridCell3DHash> obstacle_cells;
};

class TerrainModel
{
public:
  TerrainModel(
    const octomap::OcTree & tree, TerrainParameters parameters,
    std::vector<Polygon2D> pit_polygons = {},
    std::optional<TerrainObservation> observation = std::nullopt);

  GridCell3D worldToGrid(double x, double y, double z) const;
  octomap::point3d gridToWorld(const GridCell3D & cell) const;
  bool isTraversable(const GridCell3D & cell) const;
  bool transitionAllowed(const GridCell3D & from, const GridCell3D & to) const;
  std::optional<GridCell3D> snapToTerrain(const GridCell3D & seed) const;
  std::optional<GridCell3D> snapToTerrainAtXY(const GridCell3D & seed) const;
  std::optional<GridCell3D> snapGoalToTerrain(const GridCell3D & seed) const;
  std::vector<GridCell3D> plan(const GridCell3D & start, const GridCell3D & goal) const;
  std::vector<GridCell3D> planAvoidingColumns(
    const GridCell3D & start, const GridCell3D & goal,
    const GridColumnSet & blocked_columns,
    std::optional<GridPlanningBounds> bounds = std::nullopt) const;
  double traversalCost(const GridCell3D & cell) const;
  const TerrainLayers & layers() const;
  double resolution() const;

private:
  bool inside(const GridCell3D & cell) const;
  bool occupied(const GridCell3D & cell) const;
  bool supportOccupied(const GridCell3D & cell) const;
  bool collisionOccupied(const GridCell3D & cell) const;
  bool supported(const GridCell3D & cell) const;
  bool collides(const GridCell3D & cell) const;
  bool inPitFootprint(const GridCell3D & cell) const;
  void buildLayers();
  static bool pointInPolygon(double x, double y, const Polygon2D & polygon);

  const octomap::OcTree & tree_;
  TerrainParameters parameters_;
  std::vector<Polygon2D> pit_polygons_;
  std::optional<TerrainObservation> observation_;
  int min_x_{};
  int min_y_{};
  int min_z_{};
  int max_x_{};
  int max_y_{};
  int max_z_{};
  TerrainLayers layers_;
  std::unordered_set<GridCell3D, GridCell3DHash> surface_cells_;
  std::unordered_map<GridCell3D, double, GridCell3DHash> traversal_costs_;
};

}  // namespace luxi_3d_navigation
