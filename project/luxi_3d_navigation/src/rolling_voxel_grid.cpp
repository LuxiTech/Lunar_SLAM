#include "luxi_3d_navigation/rolling_voxel_grid.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_set>

namespace luxi_3d_navigation
{

std::size_t VoxelIndexHash::operator()(const VoxelIndex & voxel) const
{
  const auto h1 = std::hash<int>{}(voxel.x);
  const auto h2 = std::hash<int>{}(voxel.y);
  const auto h3 = std::hash<int>{}(voxel.z);
  return h1 ^ (h2 << 1U) ^ (h3 << 2U);
}

RollingVoxelGrid::RollingVoxelGrid(RollingVoxelGridParameters parameters)
: parameters_(parameters)
{
  if (parameters_.resolution <= 0.0 || parameters_.hit_increment <= 0 ||
    parameters_.miss_decrement <= 0 ||
    parameters_.minimum_score >= parameters_.occupied_threshold ||
    parameters_.occupied_threshold > parameters_.maximum_score ||
    parameters_.occupied_ttl <= 0.0 ||
    parameters_.stale_entry_ttl < parameters_.occupied_ttl ||
    parameters_.size_x <= 0.0 || parameters_.size_y <= 0.0 || parameters_.size_z <= 0.0)
  {
    throw std::invalid_argument("rolling voxel grid parameters are invalid");
  }
}

VoxelIndex RollingVoxelGrid::pointToVoxel(const Point3D & point) const
{
  return VoxelIndex{
    static_cast<int>(std::floor(point.x / parameters_.resolution)),
    static_cast<int>(std::floor(point.y / parameters_.resolution)),
    static_cast<int>(std::floor(point.z / parameters_.resolution))};
}

Point3D RollingVoxelGrid::voxelCenter(const VoxelIndex & voxel) const
{
  return Point3D{
    (static_cast<double>(voxel.x) + 0.5) * parameters_.resolution,
    (static_cast<double>(voxel.y) + 0.5) * parameters_.resolution,
    (static_cast<double>(voxel.z) + 0.5) * parameters_.resolution};
}

void RollingVoxelGrid::integrateFrame(
  const Point3D & origin, const std::vector<RayObservation> & observations,
  double now_seconds)
{
  std::unordered_set<VoxelIndex, VoxelIndexHash> misses;
  std::unordered_set<VoxelIndex, VoxelIndexHash> hits;
  struct EndpointFlags
  {
    bool hit{false};
    bool clear_ray{false};
  };
  std::unordered_map<VoxelIndex, EndpointFlags, VoxelIndexHash> unique_endpoints;
  for (const auto & observation : observations) {
    const auto voxel = pointToVoxel(observation.endpoint);
    auto & flags = unique_endpoints[voxel];
    flags.hit = flags.hit || observation.endpoint_is_hit;
    flags.clear_ray = flags.clear_ray || observation.clear_ray;
  }
  const double step = parameters_.resolution;
  for (const auto & [endpoint_voxel, flags] : unique_endpoints) {
    const auto endpoint = voxelCenter(endpoint_voxel);
    const double dx = endpoint.x - origin.x;
    const double dy = endpoint.y - origin.y;
    const double dz = endpoint.z - origin.z;
    const double length = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (!std::isfinite(length) || length < parameters_.resolution * 0.5) {
      continue;
    }
    if (flags.clear_ray) {
      const int sample_count = std::max(1, static_cast<int>(std::ceil(length / step)));
      for (int sample = 0; sample < sample_count; ++sample) {
        const double fraction = static_cast<double>(sample) / static_cast<double>(sample_count);
        misses.insert(pointToVoxel(Point3D{
          origin.x + fraction * dx, origin.y + fraction * dy, origin.z + fraction * dz}));
      }
    }
    if (flags.hit) {
      hits.insert(endpoint_voxel);
    }
  }

  for (const auto & voxel : hits) {
    misses.erase(voxel);
  }
  for (const auto & voxel : misses) {
    auto & entry = entries_[voxel];
    entry.score = std::max(parameters_.minimum_score, entry.score - parameters_.miss_decrement);
    entry.last_observed = now_seconds;
  }
  for (const auto & voxel : hits) {
    auto & entry = entries_[voxel];
    entry.score = std::min(parameters_.maximum_score, entry.score + parameters_.hit_increment);
    entry.last_observed = now_seconds;
    entry.last_hit = now_seconds;
  }
}

void RollingVoxelGrid::prune(const Point3D & center, double now_seconds)
{
  const double half_x = parameters_.size_x * 0.5;
  const double half_y = parameters_.size_y * 0.5;
  const double half_z = parameters_.size_z * 0.5;
  for (auto iterator = entries_.begin(); iterator != entries_.end();) {
    const auto point = voxelCenter(iterator->first);
    const bool outside = std::abs(point.x - center.x) > half_x ||
      std::abs(point.y - center.y) > half_y || std::abs(point.z - center.z) > half_z;
    const bool stale = now_seconds - iterator->second.last_observed >
      parameters_.stale_entry_ttl;
    if (outside || stale) {
      iterator = entries_.erase(iterator);
    } else {
      ++iterator;
    }
  }
}

bool RollingVoxelGrid::occupied(const VoxelIndex & voxel, double now_seconds) const
{
  const auto found = entries_.find(voxel);
  return found != entries_.end() && found->second.score >= parameters_.occupied_threshold &&
         now_seconds - found->second.last_hit <= parameters_.occupied_ttl;
}

std::vector<Point3D> RollingVoxelGrid::occupiedPoints(double now_seconds) const
{
  std::vector<Point3D> result;
  result.reserve(entries_.size());
  for (const auto & [voxel, entry] : entries_) {
    (void)entry;
    if (occupied(voxel, now_seconds)) {
      result.push_back(voxelCenter(voxel));
    }
  }
  return result;
}

std::size_t RollingVoxelGrid::entryCount() const
{
  return entries_.size();
}

void RollingVoxelGrid::clear()
{
  entries_.clear();
}

}  // namespace luxi_3d_navigation
