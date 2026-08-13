#pragma once

#include <Eigen/Core>

namespace luxi_location
{

class CommandGatedOdometry
{
public:
  Eigen::Matrix4d update(const Eigen::Matrix4d & raw_odom_from_base, bool moving);
  void reset();

private:
  bool initialized_{false};
  Eigen::Matrix4d last_raw_pose_{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d filtered_pose_{Eigen::Matrix4d::Identity()};
};

}  // namespace luxi_location
