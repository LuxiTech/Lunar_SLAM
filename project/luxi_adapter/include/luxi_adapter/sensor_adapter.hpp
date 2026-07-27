#ifndef LUXI_ADAPTER__SENSOR_ADAPTER_HPP_
#define LUXI_ADAPTER__SENSOR_ADAPTER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace luxi_adapter
{

/// Relays one hardware profile into the project-wide RGB-D/IMU contract.
class SensorAdapter : public rclcpp::Node
{
public:
  explicit SensorAdapter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  static std::string require_topic(const std::string & parameter_name, const std::string & topic);

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_color_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr raw_imu_publisher_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_color_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr raw_imu_subscription_;
};

}  // namespace luxi_adapter

#endif  // LUXI_ADAPTER__SENSOR_ADAPTER_HPP_
