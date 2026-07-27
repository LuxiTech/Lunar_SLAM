#include "luxi_location/icp_localizer.hpp"

#include <open3d/Open3D.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace luxi_location
{

IcpLocalizer::IcpLocalizer(IcpParameters parameters)
: parameters_(std::move(parameters))
{
}

bool IcpLocalizer::load_map(const std::string & path, std::string & error)
{
  auto loaded = std::make_shared<open3d::geometry::PointCloud>();
  if (!open3d::io::ReadPointCloud(path, *loaded) || loaded->IsEmpty()) {
    error = "failed to read a non-empty point cloud map: " + path;
    return false;
  }

  map_ = loaded->VoxelDownSample(parameters_.map_voxel_size);
  coarse_map_ = loaded->VoxelDownSample(parameters_.coarse_voxel_size);
  if (!map_ || map_->points_.size() < static_cast<std::size_t>(parameters_.minimum_scan_points)) {
    error = "point cloud map is too small after voxel downsampling";
    map_.reset();
    coarse_map_.reset();
    return false;
  }
  error.clear();
  return true;
}

IcpResult IcpLocalizer::register_scan(
  const open3d::geometry::PointCloud & scan,
  const Eigen::Matrix4d & initial_pose,
  const bool initial_alignment) const
{
  IcpResult output;
  output.pose = initial_pose;
  if (!map_ || !coarse_map_) {
    output.reason = "map is not loaded";
    return output;
  }

  const auto fine_scan = scan.VoxelDownSample(parameters_.scan_voxel_size);
  if (!fine_scan ||
    fine_scan->points_.size() < static_cast<std::size_t>(parameters_.minimum_scan_points))
  {
    output.reason = "scan has too few points";
    return output;
  }

  const auto coarse_scan = scan.VoxelDownSample(parameters_.coarse_voxel_size);
  const open3d::pipelines::registration::TransformationEstimationPointToPoint estimation(false);
  const open3d::pipelines::registration::ICPConvergenceCriteria coarse_criteria(
    1e-5, 1e-5, parameters_.coarse_iterations);
  const auto coarse = open3d::pipelines::registration::RegistrationICP(
    *coarse_scan, *coarse_map_, parameters_.coarse_max_correspondence_distance,
    initial_pose, estimation, coarse_criteria);

  const open3d::pipelines::registration::ICPConvergenceCriteria fine_criteria(
    1e-6, 1e-6, parameters_.fine_iterations);
  const auto fine = open3d::pipelines::registration::RegistrationICP(
    *fine_scan, *map_, parameters_.fine_max_correspondence_distance,
    coarse.transformation_, estimation, fine_criteria);

  const double result_yaw = yaw(fine.transformation_);
  output.pose = planar_pose(
    fine.transformation_(0, 3), fine.transformation_(1, 3),
    initial_pose(2, 3), result_yaw);
  output.fitness = fine.fitness_;
  output.rmse = fine.inlier_rmse_;

  const Eigen::Matrix4d correction = initial_pose.inverse() * output.pose;
  output.translation_correction = correction.block<2, 1>(0, 3).norm();
  output.yaw_correction = std::abs(normalize_angle(yaw(output.pose) - yaw(initial_pose)));

  const double maximum_translation = initial_alignment ?
    parameters_.initial_maximum_translation_correction :
    parameters_.maximum_translation_correction;
  const double maximum_yaw = initial_alignment ?
    parameters_.initial_maximum_yaw_correction :
    parameters_.maximum_yaw_correction;

  if (output.fitness < parameters_.minimum_fitness) {
    output.reason = "fitness is below the acceptance threshold";
  } else if (!std::isfinite(output.rmse) || output.rmse > parameters_.maximum_rmse) {
    output.reason = "RMSE is above the acceptance threshold";
  } else if (output.translation_correction > maximum_translation) {
    output.reason = "translation correction is above the acceptance threshold";
  } else if (output.yaw_correction > maximum_yaw) {
    output.reason = "yaw correction is above the acceptance threshold";
  } else {
    output.accepted = true;
    output.reason = "accepted";
  }
  return output;
}

const open3d::geometry::PointCloud & IcpLocalizer::map() const
{
  if (!map_) {
    throw std::runtime_error("map is not loaded");
  }
  return *map_;
}

Eigen::Matrix4d IcpLocalizer::planar_pose(
  const double x, const double y, const double z, const double yaw_angle)
{
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  const double cosine = std::cos(yaw_angle);
  const double sine = std::sin(yaw_angle);
  pose(0, 0) = cosine;
  pose(0, 1) = -sine;
  pose(1, 0) = sine;
  pose(1, 1) = cosine;
  pose(0, 3) = x;
  pose(1, 3) = y;
  pose(2, 3) = z;
  return pose;
}

double IcpLocalizer::yaw(const Eigen::Matrix4d & pose)
{
  return std::atan2(pose(1, 0), pose(0, 0));
}

double IcpLocalizer::normalize_angle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

}  // namespace luxi_location
