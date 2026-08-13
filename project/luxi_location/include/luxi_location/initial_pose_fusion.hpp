#pragma once

#include <Eigen/Core>

namespace luxi_location
{

inline Eigen::Matrix4d initialPoseForMapAlignment(
  const Eigen::Matrix4d & trusted_pose, const Eigen::Matrix4d & icp_pose,
  const bool preserve_trusted_translation)
{
  if (!preserve_trusted_translation) {
    return icp_pose;
  }
  Eigen::Matrix4d fused = icp_pose;
  fused.block<3, 1>(0, 3) = trusted_pose.block<3, 1>(0, 3);
  return fused;
}

}  // namespace luxi_location
