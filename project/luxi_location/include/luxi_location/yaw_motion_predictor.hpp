#pragma once

#include <Eigen/Core>

#include <optional>

namespace luxi_location
{

class YawMotionPredictor
{
public:
  explicit YawMotionPredictor(double maximum_prediction);

  void update(double yaw);
  void anchor();
  void reset();
  Eigen::Matrix4d predict(const Eigen::Matrix4d & pose) const;

private:
  double maximum_prediction_;
  std::optional<double> latest_yaw_;
  std::optional<double> anchor_yaw_;
};

}  // namespace luxi_location
