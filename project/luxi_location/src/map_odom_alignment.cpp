#include "luxi_location/map_odom_alignment.hpp"

#include "luxi_location/icp_localizer.hpp"

#include <cmath>
#include <stdexcept>

namespace luxi_location
{
namespace
{

Eigen::Matrix4d planar(const Eigen::Matrix4d & pose)
{
  return IcpLocalizer::planar_pose(
    pose(0, 3), pose(1, 3), pose(2, 3), IcpLocalizer::yaw(pose));
}

Eigen::Matrix4d planar_inverse(const Eigen::Matrix4d & pose)
{
  const double yaw = IcpLocalizer::yaw(pose);
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  return IcpLocalizer::planar_pose(
    -cosine * pose(0, 3) - sine * pose(1, 3),
    sine * pose(0, 3) - cosine * pose(1, 3),
    -pose(2, 3), -yaw);
}

bool finite_pose(const Eigen::Matrix4d & pose)
{
  return pose.allFinite() && std::isfinite(IcpLocalizer::yaw(pose));
}

}  // namespace

MapOdomAlignment::MapOdomAlignment(
  const double correction_gain, const double maximum_translation_correction,
  const double maximum_yaw_correction)
: correction_gain_(correction_gain),
  maximum_translation_correction_(maximum_translation_correction),
  maximum_yaw_correction_(maximum_yaw_correction)
{
  if (correction_gain_ < 0.0 || correction_gain_ > 1.0 ||
    maximum_translation_correction_ <= 0.0 || maximum_yaw_correction_ <= 0.0)
  {
    throw std::invalid_argument("map-odom alignment parameters are invalid");
  }
}

void MapOdomAlignment::initialize(
  const Eigen::Matrix4d & map_from_base,
  const Eigen::Matrix4d & odom_from_base)
{
  if (!finite_pose(map_from_base) || !finite_pose(odom_from_base)) {
    throw std::invalid_argument("map-odom initialization pose is invalid");
  }
  map_from_odom_ = planar(map_from_base) * planar_inverse(odom_from_base);
  map_from_odom_ = planar(map_from_odom_);
  initialized_ = true;
}

void MapOdomAlignment::reset()
{
  initialized_ = false;
  map_from_odom_ = Eigen::Matrix4d::Identity();
}

bool MapOdomAlignment::initialized() const
{
  return initialized_;
}

Eigen::Matrix4d MapOdomAlignment::predict(const Eigen::Matrix4d & odom_from_base) const
{
  if (!initialized_) {
    throw std::logic_error("map-odom alignment is not initialized");
  }
  if (!finite_pose(odom_from_base)) {
    throw std::invalid_argument("odometry pose is invalid");
  }
  return planar(map_from_odom_ * planar(odom_from_base));
}

MapOdomCorrection MapOdomAlignment::correct(
  const Eigen::Matrix4d & measured_map_from_base,
  const Eigen::Matrix4d & odom_from_base)
{
  MapOdomCorrection decision;
  if (!initialized_ || !finite_pose(measured_map_from_base) || !finite_pose(odom_from_base)) {
    decision.reason = "map-odom correction state is invalid";
    return decision;
  }

  const Eigen::Matrix4d predicted = predict(odom_from_base);
  decision.translation_residual =
    (measured_map_from_base.block<2, 1>(0, 3) - predicted.block<2, 1>(0, 3)).norm();
  const double yaw_difference = IcpLocalizer::normalize_angle(
    IcpLocalizer::yaw(measured_map_from_base) - IcpLocalizer::yaw(predicted));
  decision.yaw_residual = std::abs(yaw_difference);
  if (decision.translation_residual > maximum_translation_correction_) {
    decision.reason = "ICP translation disagrees with local odometry";
    return decision;
  }
  if (decision.yaw_residual > maximum_yaw_correction_) {
    decision.reason = "ICP yaw disagrees with local odometry";
    return decision;
  }

  const Eigen::Matrix4d measured_map_from_odom =
    planar(measured_map_from_base) * planar_inverse(odom_from_base);
  const double corrected_yaw = IcpLocalizer::yaw(map_from_odom_) +
    correction_gain_ * IcpLocalizer::normalize_angle(
    IcpLocalizer::yaw(measured_map_from_odom) - IcpLocalizer::yaw(map_from_odom_));
  map_from_odom_ = IcpLocalizer::planar_pose(
    map_from_odom_(0, 3) + correction_gain_ *
    (measured_map_from_odom(0, 3) - map_from_odom_(0, 3)),
    map_from_odom_(1, 3) + correction_gain_ *
    (measured_map_from_odom(1, 3) - map_from_odom_(1, 3)),
    map_from_odom_(2, 3) + correction_gain_ *
    (measured_map_from_odom(2, 3) - map_from_odom_(2, 3)),
    corrected_yaw);
  decision.accepted = true;
  decision.reason = "accepted odometry-constrained ICP correction";
  return decision;
}

const Eigen::Matrix4d & MapOdomAlignment::map_from_odom() const
{
  if (!initialized_) {
    throw std::logic_error("map-odom alignment is not initialized");
  }
  return map_from_odom_;
}

}  // namespace luxi_location
