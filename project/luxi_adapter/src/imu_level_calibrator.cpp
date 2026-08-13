#include "luxi_adapter/imu_level_calibrator.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"

namespace luxi_adapter
{

MountAngles estimate_mount_angles(const tf2::Vector3 & measured_up_in_camera)
{
  if (measured_up_in_camera.length2() < 1.0e-12) {
    throw std::invalid_argument("Measured up vector must be non-zero");
  }
  const tf2::Vector3 up = measured_up_in_camera.normalized();
  return MountAngles{
    std::atan2(up.y(), up.z()),
    std::atan2(-up.x(), std::hypot(up.y(), up.z()))};
}

bool is_stationary_imu_sample(
  const tf2::Vector3 & angular_velocity,
  const tf2::Vector3 & acceleration,
  double gravity,
  double gravity_tolerance,
  double maximum_angular_speed)
{
  return angular_velocity.length() <= maximum_angular_speed &&
         std::abs(acceleration.length() - gravity) <= gravity_tolerance;
}

ImuLevelCalibrator::ImuLevelCalibrator(const rclcpp::NodeOptions & options)
: Node("imu_level_calibrator", options),
  tf_buffer_(get_clock()),
  tf_listener_(tf_buffer_),
  static_broadcaster_(this)
{
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  camera_frame_ = declare_parameter<std::string>("camera_frame", "camera_link");
  camera_x_ = declare_parameter<double>("camera_x", 0.0);
  camera_y_ = declare_parameter<double>("camera_y", 0.0);
  camera_z_ = declare_parameter<double>("camera_z", 0.0);
  camera_yaw_ = declare_parameter<double>("camera_yaw", 0.0);
  const auto sample_count = declare_parameter<int>("calibration_samples", 200);
  gravity_ = declare_parameter<double>("gravity", 9.80665);
  gravity_tolerance_ = declare_parameter<double>("gravity_tolerance", 0.8);
  maximum_angular_speed_ = declare_parameter<double>("maximum_angular_speed", 0.05);
  const double maximum_tilt_degrees = declare_parameter<double>("maximum_tilt_degrees", 40.0);
  const std::string imu_topic =
    declare_parameter<std::string>("imu_topic", "/sensors/imu/data_raw");
  const std::string calibration_service = declare_parameter<std::string>(
    "calibration_service", "/sensors/imu/calibrate_level");
  const std::string status_topic = declare_parameter<std::string>(
    "status_topic", "/sensors/imu/level_calibration_status");
  const bool auto_start = declare_parameter<bool>("auto_start", false);

  if (sample_count <= 0) {
    throw std::invalid_argument("calibration_samples must be positive");
  }
  required_samples_ = static_cast<std::size_t>(sample_count);
  maximum_tilt_ = maximum_tilt_degrees * M_PI / 180.0;
  imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, rclcpp::SensorDataQoS(),
    std::bind(&ImuLevelCalibrator::imu_callback, this, std::placeholders::_1));
  status_publisher_ = create_publisher<std_msgs::msg::String>(
    status_topic, rclcpp::QoS(1).reliable().transient_local());
  calibration_service_ = create_service<std_srvs::srv::Trigger>(
    calibration_service,
    std::bind(
      &ImuLevelCalibrator::calibration_callback, this,
      std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(
    get_logger(),
    "IMU leveling ready on %s; %zu stationary samples are required",
    calibration_service.c_str(), required_samples_);
  if (auto_start) {
    begin_calibration();
  } else {
    publish_status(calibration_state_, calibration_message_);
  }
}

void ImuLevelCalibrator::calibration_callback(
  const std_srvs::srv::Trigger::Request::SharedPtr,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  begin_calibration();
  response->success = true;
  response->message = "IMU leveling started; keep the robot stationary on level ground";
}

void ImuLevelCalibrator::begin_calibration()
{
  sample_count_ = 0;
  measured_up_sum_.setValue(0.0, 0.0, 0.0);
  calibration_requested_ = true;
  calibration_state_ = "waiting_stationary";
  calibration_message_ = "等待机器人在水平面保持静止";
  publish_status(calibration_state_, calibration_message_);
  RCLCPP_INFO(
    get_logger(), "IMU leveling requested; waiting for %zu stationary samples",
    required_samples_);
}

void ImuLevelCalibrator::reset_samples(const char * reason)
{
  if (sample_count_ > 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "IMU leveling restarted after %zu samples: %s", sample_count_, reason);
  }
  sample_count_ = 0;
  measured_up_sum_.setValue(0.0, 0.0, 0.0);
  calibration_state_ = "waiting_stationary";
  calibration_message_ = reason;
  publish_status(calibration_state_, calibration_message_);
}

void ImuLevelCalibrator::publish_status(
  const std::string & state, const std::string & message)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3)
         << "{\"state\":\"" << state
         << "\",\"message\":\"" << message
         << "\",\"sample_count\":" << sample_count_
         << ",\"required_samples\":" << required_samples_
         << ",\"calibrated\":" << (calibrated_ ? "true" : "false");
  if (calibrated_) {
    stream << ",\"roll_degrees\":" << calibrated_roll_ * 180.0 / M_PI
           << ",\"pitch_degrees\":" << calibrated_pitch_ * 180.0 / M_PI;
  } else {
    stream << ",\"roll_degrees\":null,\"pitch_degrees\":null";
  }
  stream << "}";
  std_msgs::msg::String status;
  status.data = stream.str();
  status_publisher_->publish(status);
}

void ImuLevelCalibrator::imu_callback(const sensor_msgs::msg::Imu::SharedPtr message)
{
  if (!calibration_requested_) {
    return;
  }

  const tf2::Vector3 angular_velocity(
    message->angular_velocity.x,
    message->angular_velocity.y,
    message->angular_velocity.z);
  const tf2::Vector3 acceleration(
    message->linear_acceleration.x,
    message->linear_acceleration.y,
    message->linear_acceleration.z);
  if (!is_stationary_imu_sample(
      angular_velocity, acceleration, gravity_, gravity_tolerance_,
      maximum_angular_speed_))
  {
    reset_samples("robot is moving or acceleration is not stationary gravity");
    return;
  }

  geometry_msgs::msg::TransformStamped camera_from_imu;
  try {
    camera_from_imu = tf_buffer_.lookupTransform(
      camera_frame_, message->header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException & exception) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Waiting for %s -> %s IMU extrinsic: %s",
      camera_frame_.c_str(), message->header.frame_id.c_str(), exception.what());
    reset_samples("IMU extrinsic is unavailable");
    return;
  }

  const auto & rotation = camera_from_imu.transform.rotation;
  const tf2::Quaternion camera_from_imu_rotation(
    rotation.x, rotation.y, rotation.z, rotation.w);
  measured_up_sum_ += tf2::quatRotate(camera_from_imu_rotation, acceleration.normalized());
  ++sample_count_;
  calibration_state_ = "collecting";
  calibration_message_ = "正在采集静止 IMU 样本";
  if (sample_count_ == 1 || sample_count_ % 20 == 0) {
    publish_status(calibration_state_, calibration_message_);
  }
  if (sample_count_ >= required_samples_) {
    publish_calibrated_transform();
  }
}

void ImuLevelCalibrator::publish_calibrated_transform()
{
  const MountAngles mount = estimate_mount_angles(measured_up_sum_);
  const double tilt = std::hypot(mount.roll, mount.pitch);
  if (tilt > maximum_tilt_) {
    RCLCPP_ERROR(
      get_logger(),
      "Refusing IMU leveling: estimated mount tilt %.2f deg exceeds %.2f deg",
      tilt * 180.0 / M_PI, maximum_tilt_ * 180.0 / M_PI);
    reset_samples("estimated mount tilt is implausible");
    return;
  }

  tf2::Quaternion rotation;
  rotation.setRPY(mount.roll, mount.pitch, camera_yaw_);
  rotation.normalize();
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = now();
  transform.header.frame_id = base_frame_;
  transform.child_frame_id = camera_frame_;
  transform.transform.translation.x = camera_x_;
  transform.transform.translation.y = camera_y_;
  transform.transform.translation.z = camera_z_;
  transform.transform.rotation.x = rotation.x();
  transform.transform.rotation.y = rotation.y();
  transform.transform.rotation.z = rotation.z();
  transform.transform.rotation.w = rotation.w();
  static_broadcaster_.sendTransform(transform);
  calibrated_ = true;
  calibration_requested_ = false;
  calibrated_roll_ = mount.roll;
  calibrated_pitch_ = mount.pitch;
  calibration_state_ = "calibrated";
  calibration_message_ = "水平校准完成";
  publish_status(calibration_state_, calibration_message_);

  RCLCPP_INFO(
    get_logger(),
    "IMU leveling complete: camera mount roll=%.2f deg pitch=%.2f deg yaw=%.2f deg, "
    "translation=(%.3f, %.3f, %.3f) m",
    mount.roll * 180.0 / M_PI, mount.pitch * 180.0 / M_PI,
    camera_yaw_ * 180.0 / M_PI, camera_x_, camera_y_, camera_z_);
}

}  // namespace luxi_adapter
