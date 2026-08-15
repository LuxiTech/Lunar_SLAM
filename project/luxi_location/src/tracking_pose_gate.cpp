#include "luxi_location/tracking_pose_gate.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace luxi_location
{
namespace
{

double yaw(const Eigen::Matrix4d & pose)
{
  return std::atan2(pose(1, 0), pose(0, 0));
}

double angle_difference(const double first, const double second)
{
  return std::atan2(std::sin(first - second), std::cos(first - second));
}

bool finite_planar_pose(const Eigen::Matrix4d & pose)
{
  return std::isfinite(pose(0, 3)) && std::isfinite(pose(1, 3)) &&
         std::isfinite(pose(2, 3)) && std::isfinite(yaw(pose));
}

}  // namespace

TrackingPoseGate::TrackingPoseGate(TrackingPoseGateParameters parameters)
: parameters_(std::move(parameters))
{
  if (parameters_.maximum_linear_speed < 0.0 ||
    parameters_.maximum_angular_speed < 0.0 ||
    parameters_.translation_margin < 0.0 || parameters_.yaw_margin < 0.0 ||
    parameters_.maximum_interval <= 0.0 ||
    parameters_.maximum_relocalization_translation < 0.0 ||
    parameters_.maximum_relocalization_yaw < 0.0)
  {
    throw std::invalid_argument("tracking pose gate parameters are invalid");
  }
}

void TrackingPoseGate::reset(const Eigen::Matrix4d & pose, const double stamp_seconds)
{
  if (!finite_planar_pose(pose) || !std::isfinite(stamp_seconds)) {
    throw std::invalid_argument("tracking pose gate reference is invalid");
  }
  accepted_pose_ = pose;
  accepted_stamp_seconds_ = stamp_seconds;
}

TrackingPoseDecision TrackingPoseGate::evaluate(
  const Eigen::Matrix4d & pose, const double stamp_seconds)
{
  TrackingPoseDecision decision;
  if (!finite_planar_pose(pose) || !std::isfinite(stamp_seconds)) {
    decision.reason = "pose contains non-finite values";
    return decision;
  }
  if (!accepted_pose_.has_value()) {
    reset(pose, stamp_seconds);
    decision.accepted = true;
    decision.reason = "accepted initial tracking pose";
    return decision;
  }

  const double interval = stamp_seconds - accepted_stamp_seconds_;
  if (interval <= 0.0) {
    decision.reason = "pose timestamp is not newer than the accepted pose";
    return decision;
  }
  decision.translation_delta =
    (pose.block<2, 1>(0, 3) - accepted_pose_->block<2, 1>(0, 3)).norm();
  decision.yaw_delta = std::abs(angle_difference(yaw(pose), yaw(*accepted_pose_)));
  const double bounded_interval = std::min(interval, parameters_.maximum_interval);
  const double maximum_translation =
    parameters_.translation_margin + parameters_.maximum_linear_speed * bounded_interval;
  const double maximum_yaw =
    parameters_.yaw_margin + parameters_.maximum_angular_speed * bounded_interval;
  if (decision.translation_delta > maximum_translation) {
    decision.reason = "translation jump exceeds physical limit";
    return decision;
  }
  if (decision.yaw_delta > maximum_yaw) {
    decision.reason = "yaw jump exceeds physical limit";
    return decision;
  }

  reset(pose, stamp_seconds);
  decision.accepted = true;
  decision.reason = "accepted";
  return decision;
}

TrackingPoseDecision TrackingPoseGate::evaluate_relocalization(
  const Eigen::Matrix4d & pose, const double stamp_seconds)
{
  if (!accepted_pose_.has_value()) {
    return evaluate(pose, stamp_seconds);
  }
  return evaluate_relocalization(pose, *accepted_pose_, stamp_seconds);
}

TrackingPoseDecision TrackingPoseGate::evaluate_relocalization(
  const Eigen::Matrix4d & pose, const Eigen::Matrix4d & continuity_reference,
  const double stamp_seconds)
{
  TrackingPoseDecision decision;
  if (!finite_planar_pose(pose) || !finite_planar_pose(continuity_reference) ||
    !std::isfinite(stamp_seconds))
  {
    decision.reason = "pose contains non-finite values";
    return decision;
  }
  decision.translation_delta =
    (pose.block<2, 1>(0, 3) - continuity_reference.block<2, 1>(0, 3)).norm();
  decision.yaw_delta = std::abs(angle_difference(yaw(pose), yaw(continuity_reference)));
  if (decision.translation_delta > parameters_.maximum_relocalization_translation) {
    decision.reason = "relocalized translation is inconsistent with the last trusted pose";
    return decision;
  }
  if (decision.yaw_delta > parameters_.maximum_relocalization_yaw) {
    decision.reason = "relocalized yaw is inconsistent with the last trusted pose";
    return decision;
  }
  reset(pose, stamp_seconds);
  decision.accepted = true;
  decision.reason = "accepted relocalization";
  return decision;
}

}  // namespace luxi_location
