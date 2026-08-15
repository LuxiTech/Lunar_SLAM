#include "luxi_location/localization_supervisor.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <stdexcept>

namespace luxi_location
{
namespace
{

double yaw(const Eigen::Matrix4d & pose)
{
  return std::atan2(pose(1, 0), pose(0, 0));
}

double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

bool finite_planar_pose(const Eigen::Matrix4d & pose)
{
  return std::isfinite(pose(0, 3)) && std::isfinite(pose(1, 3)) &&
         std::isfinite(yaw(pose));
}

}  // namespace

bool should_retain_odometry_after_rejected_icp(
  const bool initial_alignment,
  const bool accepted,
  const bool relocalize_on_tracking_icp_failure,
  const bool alignment_initialized,
  const double fitness,
  const double minimum_fitness,
  const double static_point_ratio,
  const double minimum_static_point_ratio_for_relocalization)
{
  const bool scan_is_dynamically_occluded =
    std::isfinite(static_point_ratio) &&
    static_point_ratio < minimum_static_point_ratio_for_relocalization;
  return !initial_alignment && !accepted &&
         (!relocalize_on_tracking_icp_failure || scan_is_dynamically_occluded) &&
         alignment_initialized &&
         std::isfinite(fitness) && fitness >= minimum_fitness;
}

LocalizationSupervisor::LocalizationSupervisor(
  LocalizationSupervisorParameters parameters)
: parameters_(parameters)
{
  if (parameters_.consistent_pose_count <= 0 ||
    parameters_.maximum_translation_difference < 0.0 ||
    parameters_.maximum_yaw_difference < 0.0 ||
    parameters_.failures_before_relocalization <= 0)
  {
    throw std::invalid_argument("localization supervisor parameters are invalid");
  }
}

std::optional<Eigen::Matrix4d> LocalizationSupervisor::add_coarse_pose(
  const Eigen::Matrix4d & pose)
{
  if (phase_ != LocalizationPhase::kSearching || !finite_planar_pose(pose)) {
    return std::nullopt;
  }
  if (!coarse_pose_anchor_.has_value()) {
    coarse_pose_anchor_ = pose;
    consistent_pose_count_ = 1;
  } else {
    const Eigen::Vector2d difference =
      pose.block<2, 1>(0, 3) - coarse_pose_anchor_->block<2, 1>(0, 3);
    const double yaw_difference = std::abs(normalize_angle(yaw(pose) - yaw(*coarse_pose_anchor_)));
    if (difference.norm() <= parameters_.maximum_translation_difference &&
      yaw_difference <= parameters_.maximum_yaw_difference)
    {
      ++consistent_pose_count_;
    } else {
      coarse_pose_anchor_ = pose;
      consistent_pose_count_ = 1;
    }
  }

  if (consistent_pose_count_ < parameters_.consistent_pose_count) {
    return std::nullopt;
  }
  phase_ = LocalizationPhase::kWaitingForIcp;
  consecutive_icp_failures_ = 0;
  return pose;
}

HlocAction LocalizationSupervisor::report_icp_result(bool accepted)
{
  if (phase_ == LocalizationPhase::kSearching) {
    return HlocAction::kNone;
  }
  if (accepted) {
    consecutive_icp_failures_ = 0;
    if (phase_ == LocalizationPhase::kWaitingForIcp) {
      phase_ = LocalizationPhase::kTracking;
      return HlocAction::kDisable;
    }
    return HlocAction::kNone;
  }

  ++consecutive_icp_failures_;
  if (consecutive_icp_failures_ < parameters_.failures_before_relocalization) {
    return HlocAction::kNone;
  }
  phase_ = LocalizationPhase::kSearching;
  consecutive_icp_failures_ = 0;
  reset_coarse_poses();
  return HlocAction::kEnable;
}

void LocalizationSupervisor::force_relocalization()
{
  phase_ = LocalizationPhase::kSearching;
  consecutive_icp_failures_ = 0;
  reset_coarse_poses();
}

LocalizationPhase LocalizationSupervisor::phase() const
{
  return phase_;
}

int LocalizationSupervisor::consistent_pose_count() const
{
  return consistent_pose_count_;
}

int LocalizationSupervisor::consecutive_icp_failures() const
{
  return consecutive_icp_failures_;
}

void LocalizationSupervisor::reset_coarse_poses()
{
  coarse_pose_anchor_.reset();
  consistent_pose_count_ = 0;
}

}  // namespace luxi_location
