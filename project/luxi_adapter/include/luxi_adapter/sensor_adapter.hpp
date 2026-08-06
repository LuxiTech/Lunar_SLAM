#ifndef LUXI_ADAPTER__SENSOR_ADAPTER_HPP_
#define LUXI_ADAPTER__SENSOR_ADAPTER_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rtabmap_msgs/msg/rgbd_image.hpp"
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
  void publish_rgbd_input(rtabmap_msgs::msg::RGBDImage::UniquePtr message);
  void publish_split_outputs(const rtabmap_msgs::msg::RGBDImage & message);
  void try_publish_separate_rgbd();

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr color_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_color_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr raw_imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr orientation_imu_publisher_;
  rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr rgbd_publisher_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_color_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr raw_imu_subscription_;
  rclcpp::Subscription<rtabmap_msgs::msg::RGBDImage>::SharedPtr rgbd_subscription_;

  std::mutex separate_input_mutex_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_color_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_depth_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_camera_info_;
  int64_t last_rgbd_stamp_ns_{-1};
  int64_t maximum_sensor_time_difference_ns_{50000000};
};

}  // namespace luxi_adapter

#endif  // LUXI_ADAPTER__SENSOR_ADAPTER_HPP_
