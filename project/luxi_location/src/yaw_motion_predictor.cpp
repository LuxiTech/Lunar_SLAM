#include "luxi_location/yaw_motion_predictor.hpp"

#include <cmath>
#include <stdexcept>

namespace luxi_location
{

YawMotionPredictor::YawMotionPredictor(const double maximum_prediction)
: maximum_prediction_(maximum_prediction)
{
  if (maximum_prediction_ < 0.0) {
    throw std::invalid_argument("maximum yaw prediction must not be negative");
  }
}

void YawMotionPredictor::update(const double yaw)
{
  if (std::isfinite(yaw)) {
    latest_yaw_ = yaw;
  }
}

void YawMotionPredictor::anchor()
{
  anchor_yaw_ = latest_yaw_;
}

void YawMotionPredictor::reset()
{
  anchor_yaw_.reset();
}

Eigen::Matrix4d YawMotionPredictor::predict(const Eigen::Matrix4d & pose) const
{
  if (!latest_yaw_.has_value() || !anchor_yaw_.has_value()) {
    return pose;
  }
  const double delta = std::atan2(
    std::sin(*latest_yaw_ - *anchor_yaw_),
    std::cos(*latest_yaw_ - *anchor_yaw_));
  if (std::abs(delta) > maximum_prediction_) {
    return pose;
  }

  const double pose_yaw = std::atan2(pose(1, 0), pose(0, 0));
  const double predicted_yaw = pose_yaw + delta;
  Eigen::Matrix4d predicted = pose;
  predicted(0, 0) = std::cos(predicted_yaw);
  predicted(0, 1) = -std::sin(predicted_yaw);
  predicted(1, 0) = std::sin(predicted_yaw);
  predicted(1, 1) = std::cos(predicted_yaw);
  return predicted;
}

}  // namespace luxi_location
