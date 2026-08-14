#ifndef LUXI_ADAPTER__IMU_LEVEL_CALIBRATOR_HPP_
#define LUXI_ADAPTER__IMU_LEVEL_CALIBRATOR_HPP_

#include <cstddef>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace luxi_adapter
{

struct MountAngles
{
  double roll{0.0};
  double pitch{0.0};
};

MountAngles estimate_mount_angles(const tf2::Vector3 & measured_up_in_camera);
bool is_stationary_imu_sample(
  const tf2::Vector3 & angular_velocity,
  const tf2::Vector3 & acceleration,
  double gravity,
  double gravity_tolerance,
  double maximum_angular_speed);

class ImuLevelCalibrator : public rclcpp::Node
{
public:
  explicit ImuLevelCalibrator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr message);
  void calibration_callback(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);
  void begin_calibration();
  void reset_samples(const char * reason);
  void publish_status(const std::string & state, const std::string & message);
  void publish_calibrated_transform();

  std::string base_frame_;
  std::string camera_frame_;
  std::size_t required_samples_;
  double camera_x_;
  double camera_y_;
  double camera_z_;
  double camera_yaw_;
  double gravity_;
  double gravity_tolerance_;
  double maximum_angular_speed_;
  double maximum_tilt_;
  std::size_t sample_count_{0};
  tf2::Vector3 measured_up_sum_{0.0, 0.0, 0.0};
  bool calibrated_{false};
  bool calibration_requested_{false};
  double calibrated_roll_{0.0};
  double calibrated_pitch_{0.0};
  std::string calibration_state_{"idle"};
  std::string calibration_message_{"请将机器人放在水平面并点击校准"};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibration_service_;
};

}  // namespace luxi_adapter

#endif  // LUXI_ADAPTER__IMU_LEVEL_CALIBRATOR_HPP_
