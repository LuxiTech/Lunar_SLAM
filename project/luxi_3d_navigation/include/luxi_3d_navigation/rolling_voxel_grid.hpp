#pragma once

#include <cstddef>
#include <unordered_map>
#include <vector>

namespace luxi_3d_navigation
{

struct Point3D
{
  double x{};
  double y{};
  double z{};
};

struct VoxelIndex
{
  int x{};
  int y{};
  int z{};

  bool operator==(const VoxelIndex & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelIndexHash
{
  std::size_t operator()(const VoxelIndex & voxel) const;
};

struct RayObservation
{
  Point3D endpoint;
  bool endpoint_is_hit{true};
  bool clear_ray{true};
};

struct RollingVoxelGridParameters
{
  double resolution{0.05};
  int hit_increment{1};
  int miss_decrement{1};
  int occupied_threshold{2};
  int minimum_score{-3};
  int maximum_score{5};
  double occupied_ttl{1.5};
  double stale_entry_ttl{3.0};
  double size_x{6.0};
  double size_y{6.0};
  double size_z{1.5};
};

class RollingVoxelGrid
{
public:
  explicit RollingVoxelGrid(RollingVoxelGridParameters parameters = {});

  void integrateFrame(
    const Point3D & origin, const std::vector<RayObservation> & observations,
    double now_seconds);
  void prune(const Point3D & center, double now_seconds);
  std::vector<Point3D> occupiedPoints(double now_seconds) const;
  bool occupied(const VoxelIndex & voxel, double now_seconds) const;
  VoxelIndex pointToVoxel(const Point3D & point) const;
  Point3D voxelCenter(const VoxelIndex & voxel) const;
  std::size_t entryCount() const;

private:
  struct Entry
  {
    int score{};
    double last_observed{};
    double last_hit{};
  };

  RollingVoxelGridParameters parameters_;
  std::unordered_map<VoxelIndex, Entry, VoxelIndexHash> entries_;
};

}  // namespace luxi_3d_navigation
