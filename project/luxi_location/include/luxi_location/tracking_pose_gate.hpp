#pragma once

#include <Eigen/Core>

#include <optional>
#include <string>

namespace luxi_location
{

struct TrackingPoseGateParameters
{
  double maximum_linear_speed{0.20};
  double maximum_angular_speed{0.70};
  double translation_margin{0.08};
  double yaw_margin{0.14};
  double maximum_interval{2.0};
  double maximum_relocalization_translation{0.40};
  double maximum_relocalization_yaw{0.52};
};

struct TrackingPoseDecision
{
  bool accepted{false};
  double translation_delta{0.0};
  double yaw_delta{0.0};
  std::string reason;
};

class TrackingPoseGate
{
public:
  explicit TrackingPoseGate(TrackingPoseGateParameters parameters = {});

  void reset(const Eigen::Matrix4d & pose, double stamp_seconds);
  TrackingPoseDecision evaluate(const Eigen::Matrix4d & pose, double stamp_seconds);
  TrackingPoseDecision evaluate_relocalization(
    const Eigen::Matrix4d & pose, double stamp_seconds);

private:
  TrackingPoseGateParameters parameters_;
  std::optional<Eigen::Matrix4d> accepted_pose_;
  double accepted_stamp_seconds_{0.0};
};

}  // namespace luxi_location
