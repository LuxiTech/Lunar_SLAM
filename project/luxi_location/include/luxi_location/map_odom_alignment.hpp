#pragma once

#include <Eigen/Core>

#include <string>

namespace luxi_location
{

struct MapOdomCorrection
{
  bool accepted{false};
  bool applied{false};
  double translation_residual{0.0};
  double yaw_residual{0.0};
  std::string reason;
};

class MapOdomAlignment
{
public:
  MapOdomAlignment(
    double correction_gain, double maximum_translation_correction,
    double maximum_yaw_correction);

  void initialize(
    const Eigen::Matrix4d & map_from_base,
    const Eigen::Matrix4d & odom_from_base);
  void reset();
  bool initialized() const;
  Eigen::Matrix4d predict(const Eigen::Matrix4d & odom_from_base) const;
  MapOdomCorrection correct(
    const Eigen::Matrix4d & measured_map_from_base,
    const Eigen::Matrix4d & odom_from_base,
    bool apply_correction = true);
  const Eigen::Matrix4d & map_from_odom() const;

private:
  double correction_gain_;
  double maximum_translation_correction_;
  double maximum_yaw_correction_;
  bool initialized_{false};
  Eigen::Matrix4d map_from_odom_{Eigen::Matrix4d::Identity()};
};

}  // namespace luxi_location
