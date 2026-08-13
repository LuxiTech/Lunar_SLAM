#include "luxi_location/command_gated_odometry.hpp"

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

}  // namespace

Eigen::Matrix4d CommandGatedOdometry::update(
  const Eigen::Matrix4d & raw_odom_from_base, const bool moving)
{
  if (!raw_odom_from_base.allFinite()) {
    throw std::invalid_argument("raw odometry pose contains non-finite values");
  }
  const Eigen::Matrix4d raw = planar(raw_odom_from_base);
  if (!initialized_) {
    last_raw_pose_ = raw;
    filtered_pose_ = raw;
    initialized_ = true;
    return filtered_pose_;
  }
  if (moving) {
    const Eigen::Matrix4d raw_inverse = planar_inverse(last_raw_pose_);
    const Eigen::Matrix4d delta = (raw_inverse * raw).eval();
    const Eigen::Matrix4d next = (filtered_pose_ * delta).eval();
    filtered_pose_ = planar(next);
  }
  last_raw_pose_ = raw;
  return filtered_pose_;
}

void CommandGatedOdometry::reset()
{
  initialized_ = false;
  last_raw_pose_ = Eigen::Matrix4d::Identity();
  filtered_pose_ = Eigen::Matrix4d::Identity();
}

}  // namespace luxi_location
