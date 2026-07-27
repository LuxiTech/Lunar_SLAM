#include "luxi_adapter/sensor_adapter.hpp"

#include <stdexcept>
#include <utility>

namespace luxi_adapter
{

std::string SensorAdapter::require_topic(const std::string & parameter_name, const std::string & topic)
{
  if (topic.empty()) {
    throw std::invalid_argument("Parameter '" + parameter_name + "' must not be empty.");
  }
  if (topic.front() != '/') {
    throw std::invalid_argument("Parameter '" + parameter_name + "' must be an absolute ROS topic.");
  }
  return topic;
}

SensorAdapter::SensorAdapter(const rclcpp::NodeOptions & options)
: Node("luxi_adapter", options)
{
  const auto color_input = require_topic(
    "color_input_topic",
    declare_parameter<std::string>("color_input_topic", "/camera/camera/color/image_raw"));
  const auto depth_input = require_topic(
    "depth_input_topic",
    declare_parameter<std::string>("depth_input_topic", "/camera/camera/aligned_depth_to_color/image_raw"));
  const auto camera_info_input = require_topic(
    "camera_info_input_topic",
    declare_parameter<std::string>("camera_info_input_topic", "/camera/camera/color/camera_info"));
  const auto imu_input = require_topic(
    "imu_input_topic",
    declare_parameter<std::string>("imu_input_topic", "/camera/camera/imu"));
  const auto compressed_color_input = require_topic(
    "compressed_color_input_topic",
    declare_parameter<std::string>(
      "compressed_color_input_topic", "/camera/camera/color/image_raw/compressed"));

  const auto color_output = require_topic(
    "color_output_topic",
    declare_parameter<std::string>("color_output_topic", "/sensors/rgbd/color/image_raw"));
  const auto depth_output = require_topic(
    "depth_output_topic",
    declare_parameter<std::string>("depth_output_topic", "/sensors/rgbd/depth/image_raw"));
  const auto camera_info_output = require_topic(
    "camera_info_output_topic",
    declare_parameter<std::string>("camera_info_output_topic", "/sensors/rgbd/color/camera_info"));
  const auto imu_output = require_topic(
    "imu_output_topic",
    declare_parameter<std::string>("imu_output_topic", "/sensors/imu/data_raw"));
  const auto compressed_color_output = require_topic(
    "compressed_color_output_topic",
    declare_parameter<std::string>(
      "compressed_color_output_topic", "/sensors/rgbd/color/image_raw/compressed"));
  const auto enable_imu = declare_parameter<bool>("enable_imu", true);

  const auto qos = rclcpp::SensorDataQoS();
  color_publisher_ = create_publisher<sensor_msgs::msg::Image>(color_output, qos);
  depth_publisher_ = create_publisher<sensor_msgs::msg::Image>(depth_output, qos);
  camera_info_publisher_ = create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_output, qos);
  compressed_color_publisher_ = create_publisher<sensor_msgs::msg::CompressedImage>(
    compressed_color_output, qos);

  color_subscription_ = create_subscription<sensor_msgs::msg::Image>(
    color_input, qos,
    [this](sensor_msgs::msg::Image::ConstSharedPtr message) { color_publisher_->publish(*message); });
  depth_subscription_ = create_subscription<sensor_msgs::msg::Image>(
    depth_input, qos,
    [this](sensor_msgs::msg::Image::ConstSharedPtr message) { depth_publisher_->publish(*message); });
  camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    camera_info_input, qos,
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
      camera_info_publisher_->publish(*message);
    });
  compressed_color_subscription_ = create_subscription<sensor_msgs::msg::CompressedImage>(
    compressed_color_input, qos,
    [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr message) {
      compressed_color_publisher_->publish(*message);
    });

  if (enable_imu) {
    raw_imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(imu_output, qos);
    raw_imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_input, qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr message) { raw_imu_publisher_->publish(*message); });
  }

  RCLCPP_INFO(
    get_logger(),
    "Hardware inputs are adapted to color=%s depth=%s camera_info=%s preview=%s%s",
    color_output.c_str(), depth_output.c_str(), camera_info_output.c_str(),
    compressed_color_output.c_str(),
    enable_imu ? " and raw IMU=/sensors/imu/data_raw" : "; IMU adaptation is disabled");
}

}  // namespace luxi_adapter
