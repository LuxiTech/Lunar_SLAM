#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rtabmap_msgs/msg/rgbd_image.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace
{

int64_t stamp_nanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

class SensorSyncCheck : public rclcpp::Node
{
public:
  SensorSyncCheck()
  : Node("luxi_sensor_sync_check"), started_(std::chrono::steady_clock::now())
  {
    const auto rgbd_topic = declare_parameter<std::string>(
      "rgbd_topic", "/sensors/rgbd/rgbd_image");
    const auto imu_topic = declare_parameter<std::string>(
      "imu_topic", "/sensors/imu/data");
    required_sync_count_ = static_cast<int>(
      std::max<int64_t>(1, declare_parameter<int64_t>("required_sync_count", 5)));
    required_imu_count_ = static_cast<int>(
      std::max<int64_t>(1, declare_parameter<int64_t>("required_imu_count", 20)));
    require_imu_ = declare_parameter<bool>("require_imu", true);
    timeout_sec_ = std::max(1.0, declare_parameter<double>("timeout_sec", 60.0));
    imu_no_data_timeout_sec_ = std::max(
      1.0, declare_parameter<double>("imu_no_data_timeout_sec", 8.0));
    max_rgbd_skew_ns_ = static_cast<int64_t>(
      std::max(0.0, declare_parameter<double>("max_rgbd_skew_sec", 0.05)) * 1e9);
    max_imu_delta_ns_ = static_cast<int64_t>(
      std::max(0.0, declare_parameter<double>("max_imu_delta_sec", 0.10)) * 1e9);

    rgbd_sub_ = create_subscription<rtabmap_msgs::msg::RGBDImage>(
      rgbd_topic, rclcpp::SensorDataQoS(),
      std::bind(&SensorSyncCheck::rgbd_callback, this, std::placeholders::_1));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr message) {
        ++imu_count_;
        const auto & orientation = message->orientation;
        const auto & angular_velocity = message->angular_velocity;
        const auto & acceleration = message->linear_acceleration;
        const double orientation_norm = std::sqrt(
          orientation.x * orientation.x + orientation.y * orientation.y +
          orientation.z * orientation.z + orientation.w * orientation.w);
        const bool valid =
          message->orientation_covariance[0] >= 0.0 &&
          message->angular_velocity_covariance[0] >= 0.0 &&
          message->linear_acceleration_covariance[0] >= 0.0 &&
          std::isfinite(orientation_norm) && orientation_norm > 0.5 &&
          orientation_norm < 1.5 &&
          std::isfinite(angular_velocity.x) &&
          std::isfinite(angular_velocity.y) &&
          std::isfinite(angular_velocity.z) &&
          std::isfinite(acceleration.x) && std::isfinite(acceleration.y) &&
          std::isfinite(acceleration.z);
        if (!valid) {
          ++invalid_imu_count_;
          return;
        }
        ++valid_imu_count_;
        imu_stamps_ns_.push_back(stamp_nanoseconds(message->header.stamp));
        if (imu_stamps_ns_.size() > 2000) {
          imu_stamps_ns_.pop_front();
        }
      });
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&SensorSyncCheck::check_result, this));

    RCLCPP_INFO(
      get_logger(),
      "Waiting for consecutive atomic RGB-D packets on %s%s",
      rgbd_topic.c_str(),
      require_imu_ ? (" and IMU on " + imu_topic).c_str() : "");
  }

  bool passed() const {return passed_;}

private:
  void rgbd_callback(const rtabmap_msgs::msg::RGBDImage::ConstSharedPtr & message)
  {
    ++total_rgbd_count_;
    const int64_t outer_stamp = stamp_nanoseconds(message->header.stamp);
    const int64_t rgb_stamp = stamp_nanoseconds(message->rgb.header.stamp);
    const int64_t depth_stamp = stamp_nanoseconds(message->depth.header.stamp);
    const int64_t rgb_info_stamp = stamp_nanoseconds(message->rgb_camera_info.header.stamp);
    const int64_t depth_info_stamp = stamp_nanoseconds(message->depth_camera_info.header.stamp);
    const int64_t earliest = std::min(
      {outer_stamp, rgb_stamp, depth_stamp, rgb_info_stamp, depth_info_stamp});
    const int64_t latest = std::max(
      {outer_stamp, rgb_stamp, depth_stamp, rgb_info_stamp, depth_info_stamp});
    const int64_t skew = latest - earliest;
    int64_t imu_delta = std::numeric_limits<int64_t>::max();
    for (const int64_t imu_stamp : imu_stamps_ns_) {
      imu_delta = std::min(imu_delta, std::abs(rgb_stamp - imu_stamp));
    }
    last_camera_imu_delta_ns_ = imu_delta;

    const bool stamp_monotonic = rgb_stamp > last_rgbd_stamp_ns_;
    last_rgbd_stamp_ns_ = std::max(last_rgbd_stamp_ns_, rgb_stamp);
    const bool imu_count_ready = !require_imu_ || valid_imu_count_ >= required_imu_count_;
    const bool imu_time_ready = !require_imu_ || imu_delta <= max_imu_delta_ns_;
    if (skew <= max_rgbd_skew_ns_ && stamp_monotonic && imu_count_ready && imu_time_ready) {
      ++consecutive_valid_count_;
      largest_valid_rgbd_skew_ns_ = std::max(largest_valid_rgbd_skew_ns_, skew);
    } else {
      consecutive_valid_count_ = 0;
      ++invalid_rgbd_count_;
    }
  }

  void check_result()
  {
    const int64_t imu_delta = last_camera_imu_delta_ns_;

    if (consecutive_valid_count_ >= required_sync_count_) {
      passed_ = true;
      RCLCPP_INFO(
        get_logger(),
        "SYNC PASS: consecutive=%d total=%d invalid=%d imu_valid=%d imu_invalid=%d "
        "valid_max_rgbd_skew=%.3f ms "
        "camera_imu_delta=%.3f ms",
        consecutive_valid_count_, total_rgbd_count_, invalid_rgbd_count_, valid_imu_count_,
        invalid_imu_count_,
        largest_valid_rgbd_skew_ns_ / 1e6, imu_delta / 1e6);
      rclcpp::shutdown();
      return;
    }

    const double elapsed = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - started_).count();
    if (
      require_imu_ && elapsed >= 1.0 && valid_imu_count_ == 0 &&
      invalid_imu_count_ >= required_imu_count_)
    {
      RCLCPP_ERROR(
        get_logger(),
        "IMU HEALTH FAIL: received %d packets but none had valid orientation and "
        "three-axis angular velocity and acceleration; refusing to start odometry",
        invalid_imu_count_);
      rclcpp::shutdown();
      return;
    }
    if (require_imu_ && elapsed >= imu_no_data_timeout_sec_ && imu_count_ == 0) {
      RCLCPP_ERROR(
        get_logger(),
        "IMU NO DATA: received zero packets for %.1f s; refusing to start odometry. "
        "The H30 serial device may be enumerated but not streaming.",
        elapsed);
      rclcpp::shutdown();
      return;
    }
    if (elapsed >= timeout_sec_) {
      RCLCPP_ERROR(
        get_logger(),
        "SYNC FAIL after %.1f s: consecutive=%d/%d total=%d invalid=%d "
        "imu_valid=%d/%d imu_invalid=%d "
        "camera_imu_delta=%s",
        elapsed, consecutive_valid_count_, required_sync_count_, total_rgbd_count_,
        invalid_rgbd_count_, valid_imu_count_,
        require_imu_ ? required_imu_count_ : 0,
        invalid_imu_count_,
        imu_delta == std::numeric_limits<int64_t>::max() ?
        "unavailable" : (std::to_string(imu_delta / 1e6) + " ms").c_str());
      rclcpp::shutdown();
    }
  }

  rclcpp::Subscription<rtabmap_msgs::msg::RGBDImage>::SharedPtr rgbd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::steady_clock::time_point started_;
  int required_sync_count_{5};
  int required_imu_count_{20};
  int consecutive_valid_count_{0};
  int total_rgbd_count_{0};
  int invalid_rgbd_count_{0};
  int imu_count_{0};
  int valid_imu_count_{0};
  int invalid_imu_count_{0};
  int64_t max_rgbd_skew_ns_{50000000};
  int64_t max_imu_delta_ns_{100000000};
  int64_t largest_valid_rgbd_skew_ns_{0};
  int64_t last_rgbd_stamp_ns_{-1};
  int64_t last_camera_imu_delta_ns_{std::numeric_limits<int64_t>::max()};
  std::deque<int64_t> imu_stamps_ns_;
  double timeout_sec_{60.0};
  double imu_no_data_timeout_sec_{8.0};
  bool require_imu_{true};
  bool passed_{false};
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SensorSyncCheck>();
  rclcpp::spin(node);
  const bool passed = node->passed();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return passed ? 0 : 1;
}
