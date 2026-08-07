#include "luxi_adapter/sensor_adapter.hpp"

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <utility>

#include "cv_bridge/cv_bridge.h"
#include "opencv2/imgcodecs.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/image_encodings.hpp"

namespace luxi_adapter
{

namespace
{

int64_t stamp_nanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

void normalize_rgbd_headers(rtabmap_msgs::msg::RGBDImage & message)
{
  const auto stamp = stamp_nanoseconds(message.rgb.header.stamp) > 0 ?
    message.rgb.header.stamp : message.header.stamp;
  const std::string frame = !message.rgb.header.frame_id.empty() ?
    message.rgb.header.frame_id : message.header.frame_id;
  message.header.stamp = stamp;
  message.header.frame_id = frame;
  message.rgb.header.stamp = stamp;
  message.rgb.header.frame_id = frame;
  message.depth.header.stamp = stamp;
  message.depth.header.frame_id = frame;
  message.rgb_camera_info.header.stamp = stamp;
  message.rgb_camera_info.header.frame_id = frame;
  message.depth_camera_info.header.stamp = stamp;
  message.depth_camera_info.header.frame_id = frame;
}

}  // namespace

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
  const auto input_mode = declare_parameter<std::string>("input_mode", "separate");
  if (input_mode != "separate" && input_mode != "rgbd") {
    throw std::invalid_argument("Parameter 'input_mode' must be 'separate' or 'rgbd'.");
  }
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
  const auto rgbd_input = declare_parameter<std::string>("rgbd_input_topic", "");
  const auto rgbd_output = require_topic(
    "rgbd_output_topic",
    declare_parameter<std::string>("rgbd_output_topic", "/sensors/rgbd/rgbd_image"));

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
  const auto orientation_imu_output = declare_parameter<std::string>(
    "imu_orientation_output_topic", "");
  const auto compressed_color_output = require_topic(
    "compressed_color_output_topic",
    declare_parameter<std::string>(
      "compressed_color_output_topic", "/sensors/rgbd/color/image_raw/compressed"));
  const auto enable_imu = declare_parameter<bool>("enable_imu", true);
  const auto enable_compressed_color = declare_parameter<bool>("enable_compressed_color", true);
  generate_compressed_color_from_raw_ = declare_parameter<bool>(
    "generate_compressed_color_from_raw", false);
  compressed_color_jpeg_quality_ = declare_parameter<int>(
    "compressed_color_jpeg_quality", 80);
  const auto compressed_color_rate = declare_parameter<double>("compressed_color_rate", 5.0);
  if (compressed_color_jpeg_quality_ < 1 || compressed_color_jpeg_quality_ > 100) {
    throw std::invalid_argument("compressed_color_jpeg_quality must be in [1, 100].");
  }
  if (compressed_color_rate <= 0.0) {
    throw std::invalid_argument("compressed_color_rate must be positive.");
  }
  compressed_color_period_ns_ = static_cast<int64_t>(1.0e9 / compressed_color_rate);
  const auto reliable_image_output = declare_parameter<bool>("reliable_image_output", false);
  const auto maximum_sensor_time_difference = declare_parameter<double>(
    "maximum_sensor_time_difference", 0.05);
  if (maximum_sensor_time_difference < 0.0) {
    throw std::invalid_argument("Parameter 'maximum_sensor_time_difference' must be non-negative.");
  }
  maximum_sensor_time_difference_ns_ = static_cast<int64_t>(
    maximum_sensor_time_difference * 1.0e9);

  const auto sensor_qos = rclcpp::SensorDataQoS();
  const auto image_qos = reliable_image_output ?
    rclcpp::QoS(2).reliable() : rclcpp::QoS(sensor_qos);
  color_publisher_ = create_publisher<sensor_msgs::msg::Image>(color_output, image_qos);
  depth_publisher_ = create_publisher<sensor_msgs::msg::Image>(depth_output, image_qos);
  camera_info_publisher_ = create_publisher<sensor_msgs::msg::CameraInfo>(
    camera_info_output, image_qos);
  rgbd_publisher_ = create_publisher<rtabmap_msgs::msg::RGBDImage>(rgbd_output, image_qos);
  if (enable_compressed_color) {
    compressed_color_publisher_ = create_publisher<sensor_msgs::msg::CompressedImage>(
      compressed_color_output, image_qos);
  }

  if (input_mode == "rgbd") {
    rgbd_subscription_ = create_subscription<rtabmap_msgs::msg::RGBDImage>(
      require_topic("rgbd_input_topic", rgbd_input), rclcpp::QoS(2).reliable(),
      [this](rtabmap_msgs::msg::RGBDImage::UniquePtr message) {
        publish_rgbd_input(std::move(message));
      });
  } else {
    color_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      color_input, sensor_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
        color_publisher_->publish(*message);
        publish_generated_compressed_color(message);
        {
          std::lock_guard<std::mutex> lock(separate_input_mutex_);
          latest_color_ = message;
        }
        try_publish_separate_rgbd();
      });
    depth_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      depth_input, sensor_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
        depth_publisher_->publish(*message);
        {
          std::lock_guard<std::mutex> lock(separate_input_mutex_);
          latest_depth_ = message;
        }
        try_publish_separate_rgbd();
      });
    camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_input, sensor_qos,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
        camera_info_publisher_->publish(*message);
        {
          std::lock_guard<std::mutex> lock(separate_input_mutex_);
          latest_camera_info_ = message;
        }
        try_publish_separate_rgbd();
      });
  }
  if (enable_compressed_color && !generate_compressed_color_from_raw_) {
    compressed_color_subscription_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      compressed_color_input, sensor_qos,
      [this](sensor_msgs::msg::CompressedImage::ConstSharedPtr message) {
        compressed_color_publisher_->publish(*message);
      });
  }

  if (enable_imu) {
    raw_imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(imu_output, sensor_qos);
    if (!orientation_imu_output.empty()) {
      orientation_imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
        require_topic("imu_orientation_output_topic", orientation_imu_output), sensor_qos);
    }
    raw_imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_input, sensor_qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr message) {
        if (raw_imu_publisher_->get_subscription_count() > 0) {
          raw_imu_publisher_->publish(*message);
        }
        if (orientation_imu_publisher_ && orientation_imu_publisher_->get_subscription_count() > 0) {
          orientation_imu_publisher_->publish(*message);
        }
      });
  }

  RCLCPP_INFO(
    get_logger(),
    "Hardware %s input is adapted to rgbd=%s color=%s depth=%s camera_info=%s preview=%s%s%s",
    input_mode.c_str(), rgbd_output.c_str(), color_output.c_str(), depth_output.c_str(),
    camera_info_output.c_str(), compressed_color_output.c_str(),
    enable_compressed_color ? "" : " (disabled)",
    enable_imu ? " and IMU relay is enabled" : "; IMU adaptation is disabled");
}

void SensorAdapter::publish_generated_compressed_color(
  const sensor_msgs::msg::Image::ConstSharedPtr & message)
{
  if (!generate_compressed_color_from_raw_ || !compressed_color_publisher_ ||
    compressed_color_publisher_->get_subscription_count() == 0)
  {
    return;
  }
  const int64_t stamp_ns = stamp_nanoseconds(message->header.stamp);
  if (last_compressed_color_stamp_ns_ >= 0 && stamp_ns > last_compressed_color_stamp_ns_ &&
    stamp_ns - last_compressed_color_stamp_ns_ < compressed_color_period_ns_)
  {
    return;
  }
  try {
    const auto color = cv_bridge::toCvShare(message, sensor_msgs::image_encodings::BGR8);
    sensor_msgs::msg::CompressedImage preview;
    preview.header = message->header;
    preview.format = "jpeg";
    if (!cv::imencode(
        ".jpg", color->image, preview.data,
        {cv::IMWRITE_JPEG_QUALITY, compressed_color_jpeg_quality_}))
    {
      throw std::runtime_error("OpenCV JPEG encoder returned false");
    }
    compressed_color_publisher_->publish(preview);
    last_compressed_color_stamp_ns_ = stamp_ns;
  } catch (const std::exception & error) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Unable to generate compressed RGB preview: %s", error.what());
  }
}

void SensorAdapter::publish_split_outputs(const rtabmap_msgs::msg::RGBDImage & message)
{
  if (camera_info_publisher_->get_subscription_count() > 0) {
    camera_info_publisher_->publish(message.rgb_camera_info);
  }
  if (depth_publisher_->get_subscription_count() > 0) {
    depth_publisher_->publish(message.depth);
  }
  if (color_publisher_->get_subscription_count() > 0) {
    color_publisher_->publish(message.rgb);
  }
}

void SensorAdapter::publish_rgbd_input(rtabmap_msgs::msg::RGBDImage::UniquePtr message)
{
  normalize_rgbd_headers(*message);
  publish_split_outputs(*message);
  rgbd_publisher_->publish(std::move(message));
}

void SensorAdapter::try_publish_separate_rgbd()
{
  sensor_msgs::msg::Image::ConstSharedPtr color;
  sensor_msgs::msg::Image::ConstSharedPtr depth;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info;
  {
    std::lock_guard<std::mutex> lock(separate_input_mutex_);
    color = latest_color_;
    depth = latest_depth_;
    camera_info = latest_camera_info_;
  }
  if (!color || !depth || !camera_info) {
    return;
  }

  const int64_t color_stamp = stamp_nanoseconds(color->header.stamp);
  const int64_t depth_stamp = stamp_nanoseconds(depth->header.stamp);
  const int64_t info_stamp = stamp_nanoseconds(camera_info->header.stamp);
  const int64_t earliest = std::min({color_stamp, depth_stamp, info_stamp});
  const int64_t latest = std::max({color_stamp, depth_stamp, info_stamp});
  {
    std::lock_guard<std::mutex> lock(separate_input_mutex_);
    if (color_stamp <= last_rgbd_stamp_ns_ || latest - earliest > maximum_sensor_time_difference_ns_) {
      return;
    }
    last_rgbd_stamp_ns_ = color_stamp;
  }

  auto message = std::make_unique<rtabmap_msgs::msg::RGBDImage>();
  message->header = color->header;
  message->rgb = *color;
  message->depth = *depth;
  message->rgb_camera_info = *camera_info;
  message->depth_camera_info = *camera_info;
  normalize_rgbd_headers(*message);
  rgbd_publisher_->publish(std::move(message));
}

}  // namespace luxi_adapter

RCLCPP_COMPONENTS_REGISTER_NODE(luxi_adapter::SensorAdapter)
