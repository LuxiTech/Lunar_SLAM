#pragma once

#include <Eigen/Core>

#include <optional>

namespace luxi_location
{

enum class LocalizationPhase
{
  kSearching,
  kWaitingForIcp,
  kTracking,
};

enum class HlocAction
{
  kNone,
  kDisable,
  kEnable,
};

struct LocalizationSupervisorParameters
{
  int consistent_pose_count{3};
  double maximum_translation_difference{0.50};
  double maximum_yaw_difference{0.35};
  int failures_before_relocalization{5};
};

bool should_retain_odometry_after_rejected_icp(
  bool initial_alignment,
  bool accepted,
  bool relocalize_on_tracking_icp_failure,
  bool alignment_initialized,
  double fitness,
  double minimum_fitness,
  double static_point_ratio,
  double minimum_static_point_ratio_for_relocalization);

class LocalizationSupervisor
{
public:
  explicit LocalizationSupervisor(LocalizationSupervisorParameters parameters = {});

  std::optional<Eigen::Matrix4d> add_coarse_pose(const Eigen::Matrix4d & pose);
  HlocAction report_icp_result(bool accepted);
  void force_relocalization();

  LocalizationPhase phase() const;
  int consistent_pose_count() const;
  int consecutive_icp_failures() const;

private:
  void reset_coarse_poses();

  LocalizationSupervisorParameters parameters_;
  LocalizationPhase phase_{LocalizationPhase::kSearching};
  std::optional<Eigen::Matrix4d> coarse_pose_anchor_;
  int consistent_pose_count_{0};
  int consecutive_icp_failures_{0};
};

}  // namespace luxi_location
