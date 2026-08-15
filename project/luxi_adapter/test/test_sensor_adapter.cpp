#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "luxi_adapter/sensor_adapter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rtabmap_msgs/msg/rgbd_image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace
{

using namespace std::chrono_literals;

class SensorAdapterTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }
};

TEST_F(SensorAdapterTest, RelaysAllCanonicalSensorMessages)
{
  const std::vector<rclcpp::Parameter> parameters{
    {"color_input_topic", "/adapter_test/input/color"},
    {"depth_input_topic", "/adapter_test/input/depth"},
    {"camera_info_input_topic", "/adapter_test/input/camera_info"},
    {"imu_input_topic", "/adapter_test/input/imu"},
    {"compressed_color_input_topic", "/adapter_test/input/compressed_color"},
    {"color_output_topic", "/adapter_test/output/color"},
    {"depth_output_topic", "/adapter_test/output/depth"},
    {"camera_info_output_topic", "/adapter_test/output/camera_info"},
    {"imu_output_topic", "/adapter_test/output/imu"},
    {"imu_orientation_output_topic", "/adapter_test/output/imu_orientation"},
    {"compressed_color_output_topic", "/adapter_test/output/compressed_color"},
    {"rgbd_output_topic", "/adapter_test/output/rgbd"},
    {"enable_imu", true},
  };
  auto adapter = std::make_shared<luxi_adapter::SensorAdapter>(
    rclcpp::NodeOptions().parameter_overrides(parameters));
  auto test_node = std::make_shared<rclcpp::Node>("sensor_adapter_test_client");
  const auto qos = rclcpp::SensorDataQoS();
  auto color_input = test_node->create_publisher<sensor_msgs::msg::Image>(
    "/adapter_test/input/color", qos);
  auto depth_input = test_node->create_publisher<sensor_msgs::msg::Image>(
    "/adapter_test/input/depth", qos);
  auto info_input = test_node->create_publisher<sensor_msgs::msg::CameraInfo>(
    "/adapter_test/input/camera_info", qos);
  auto imu_input = test_node->create_publisher<sensor_msgs::msg::Imu>(
    "/adapter_test/input/imu", qos);
  auto compressed_color_input = test_node->create_publisher<sensor_msgs::msg::CompressedImage>(
    "/adapter_test/input/compressed_color", qos);

  std::atomic<int> color_count{0};
  std::atomic<int> depth_count{0};
  std::atomic<int> info_count{0};
  std::atomic<int> imu_count{0};
  std::atomic<int> orientation_imu_count{0};
  std::atomic<int> compressed_color_count{0};
  std::atomic<int> rgbd_count{0};
  auto color_output = test_node->create_subscription<sensor_msgs::msg::Image>(
    "/adapter_test/output/color", qos,
    [&color_count](sensor_msgs::msg::Image::ConstSharedPtr) { ++color_count; });
  auto depth_output = test_node->create_subscription<sensor_msgs::msg::Image>(
    "/adapter_test/output/depth", qos,
    [&depth_count](sensor_msgs::msg::Image::ConstSharedPtr) { ++depth_count; });
  auto info_output = test_node->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/adapter_test/output/camera_info", qos,
    [&info_count](sensor_msgs::msg::CameraInfo::ConstSharedPtr) { ++info_count; });
  auto imu_output = test_node->create_subscription<sensor_msgs::msg::Imu>(
    "/adapter_test/output/imu", qos,
    [&imu_count](sensor_msgs::msg::Imu::ConstSharedPtr) { ++imu_count; });
  auto orientation_imu_output = test_node->create_subscription<sensor_msgs::msg::Imu>(
    "/adapter_test/output/imu_orientation", qos,
    [&orientation_imu_count](sensor_msgs::msg::Imu::ConstSharedPtr) { ++orientation_imu_count; });
  auto compressed_color_output = test_node->create_subscription<sensor_msgs::msg::CompressedImage>(
    "/adapter_test/output/compressed_color", qos,
    [&compressed_color_count](sensor_msgs::msg::CompressedImage::ConstSharedPtr) {
      ++compressed_color_count;
    });
  auto rgbd_output = test_node->create_subscription<rtabmap_msgs::msg::RGBDImage>(
    "/adapter_test/output/rgbd", qos,
    [&rgbd_count](rtabmap_msgs::msg::RGBDImage::ConstSharedPtr) {++rgbd_count;});

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(adapter);
  executor.add_node(test_node);
  sensor_msgs::msg::Image image;
  image.header.frame_id = "camera_color_optical_frame";
  image.height = 1;
  image.width = 1;
  image.encoding = "mono8";
  image.step = 1;
  image.data = {42};
  sensor_msgs::msg::CameraInfo camera_info;
  camera_info.header = image.header;
  sensor_msgs::msg::Imu imu;
  imu.header.frame_id = "camera_imu_frame";
  sensor_msgs::msg::CompressedImage compressed_image;
  compressed_image.header = image.header;
  compressed_image.format = "jpeg";
  compressed_image.data = {0xff, 0xd8, 0xff, 0xd9};

  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (std::chrono::steady_clock::now() < deadline &&
    (color_count == 0 || depth_count == 0 || info_count == 0 || imu_count == 0 ||
    orientation_imu_count == 0 ||
    compressed_color_count == 0 || rgbd_count == 0))
  {
    color_input->publish(image);
    depth_input->publish(image);
    info_input->publish(camera_info);
    imu_input->publish(imu);
    compressed_color_input->publish(compressed_image);
    executor.spin_some();
    std::this_thread::sleep_for(20ms);
  }
  EXPECT_GT(color_count, 0);
  EXPECT_GT(depth_count, 0);
  EXPECT_GT(info_count, 0);
  EXPECT_GT(imu_count, 0);
  EXPECT_GT(orientation_imu_count, 0);
  EXPECT_GT(compressed_color_count, 0);
  EXPECT_GT(rgbd_count, 0);
  executor.remove_node(test_node);
  executor.remove_node(adapter);
}

TEST_F(SensorAdapterTest, RejectsRelativeTopics)
{
  EXPECT_THROW(
    luxi_adapter::SensorAdapter(
      rclcpp::NodeOptions().parameter_overrides(
        {rclcpp::Parameter("color_input_topic", "relative/color")})),
    std::invalid_argument);
}

TEST_F(SensorAdapterTest, GeneratesCompressedPreviewFromRawColor)
{
  const std::vector<rclcpp::Parameter> parameters{
    {"color_input_topic", "/adapter_preview_test/input/color"},
    {"depth_input_topic", "/adapter_preview_test/input/depth"},
    {"camera_info_input_topic", "/adapter_preview_test/input/camera_info"},
    {"compressed_color_input_topic", "/adapter_preview_test/input/compressed_color"},
    {"color_output_topic", "/adapter_preview_test/output/color"},
    {"depth_output_topic", "/adapter_preview_test/output/depth"},
    {"camera_info_output_topic", "/adapter_preview_test/output/camera_info"},
    {"compressed_color_output_topic", "/adapter_preview_test/output/compressed_color"},
    {"rgbd_output_topic", "/adapter_preview_test/output/rgbd"},
    {"enable_imu", false},
    {"generate_compressed_color_from_raw", true},
  };
  auto adapter = std::make_shared<luxi_adapter::SensorAdapter>(
    rclcpp::NodeOptions().parameter_overrides(parameters));
  auto test_node = std::make_shared<rclcpp::Node>("sensor_adapter_preview_test_client");
  const auto qos = rclcpp::SensorDataQoS();
  auto color_input = test_node->create_publisher<sensor_msgs::msg::Image>(
    "/adapter_preview_test/input/color", qos);
  std::atomic<bool> received_valid_jpeg{false};
  auto compressed_output = test_node->create_subscription<sensor_msgs::msg::CompressedImage>(
    "/adapter_preview_test/output/compressed_color", qos,
    [&received_valid_jpeg](sensor_msgs::msg::CompressedImage::ConstSharedPtr message) {
      received_valid_jpeg = message->format == "jpeg" && message->data.size() >= 4 &&
        message->data[0] == 0xff && message->data[1] == 0xd8 &&
        message->data[message->data.size() - 2] == 0xff &&
        message->data.back() == 0xd9;
    });

  sensor_msgs::msg::Image image;
  image.header.stamp.sec = 1;
  image.header.frame_id = "camera_color_optical_frame";
  image.height = 2;
  image.width = 2;
  image.encoding = "rgb8";
  image.step = 6;
  image.data = {
    255, 0, 0, 0, 255, 0,
    0, 0, 255, 255, 255, 255,
  };

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(adapter);
  executor.add_node(test_node);
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (std::chrono::steady_clock::now() < deadline && !received_valid_jpeg) {
    color_input->publish(image);
    executor.spin_some();
    std::this_thread::sleep_for(20ms);
  }

  EXPECT_TRUE(received_valid_jpeg);
  executor.remove_node(test_node);
  executor.remove_node(adapter);
}

TEST_F(SensorAdapterTest, SplitsSynchronizedRgbdInputIntoCanonicalTopics)
{
  const std::vector<rclcpp::Parameter> parameters{
    {"input_mode", "rgbd"},
    {"rgbd_input_topic", "/adapter_rgbd_test/input"},
    {"compressed_color_input_topic", "/adapter_rgbd_test/input/compressed_color"},
    {"color_output_topic", "/adapter_rgbd_test/output/color"},
    {"depth_output_topic", "/adapter_rgbd_test/output/depth"},
    {"camera_info_output_topic", "/adapter_rgbd_test/output/camera_info"},
    {"rgbd_output_topic", "/adapter_rgbd_test/output/rgbd"},
    {"compressed_color_output_topic", "/adapter_rgbd_test/output/compressed_color"},
    {"enable_imu", false},
    {"generate_compressed_color_from_raw", true},
  };
  auto adapter = std::make_shared<luxi_adapter::SensorAdapter>(
    rclcpp::NodeOptions().parameter_overrides(parameters));
  auto test_node = std::make_shared<rclcpp::Node>("sensor_adapter_rgbd_test_client");
  auto rgbd_input = test_node->create_publisher<rtabmap_msgs::msg::RGBDImage>(
    "/adapter_rgbd_test/input", rclcpp::QoS(2).reliable());

  std::atomic<int> color_count{0};
  std::atomic<int> depth_count{0};
  std::atomic<int> info_count{0};
  std::atomic<int> compressed_count{0};
  builtin_interfaces::msg::Time received_stamp;
  builtin_interfaces::msg::Time received_info_stamp;
  std::atomic<int> rgbd_count{0};
  const auto sensor_qos = rclcpp::SensorDataQoS();
  auto color_output = test_node->create_subscription<sensor_msgs::msg::Image>(
    "/adapter_rgbd_test/output/color", sensor_qos,
    [&color_count, &received_stamp](sensor_msgs::msg::Image::ConstSharedPtr message) {
      ++color_count;
      received_stamp = message->header.stamp;
    });
  auto depth_output = test_node->create_subscription<sensor_msgs::msg::Image>(
    "/adapter_rgbd_test/output/depth", sensor_qos,
    [&depth_count](sensor_msgs::msg::Image::ConstSharedPtr) { ++depth_count; });
  auto info_output = test_node->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/adapter_rgbd_test/output/camera_info", sensor_qos,
    [&info_count](sensor_msgs::msg::CameraInfo::ConstSharedPtr) { ++info_count; });
  auto rgbd_output = test_node->create_subscription<rtabmap_msgs::msg::RGBDImage>(
    "/adapter_rgbd_test/output/rgbd", sensor_qos,
    [&rgbd_count, &received_info_stamp](rtabmap_msgs::msg::RGBDImage::ConstSharedPtr message) {
      ++rgbd_count;
      received_info_stamp = message->rgb_camera_info.header.stamp;
    });
  auto compressed_output = test_node->create_subscription<sensor_msgs::msg::CompressedImage>(
    "/adapter_rgbd_test/output/compressed_color", sensor_qos,
    [&compressed_count](sensor_msgs::msg::CompressedImage::ConstSharedPtr message) {
      if (message->format == "jpeg" && message->data.size() >= 4 &&
        message->data[0] == 0xff && message->data[1] == 0xd8 &&
        message->data[message->data.size() - 2] == 0xff && message->data.back() == 0xd9)
      {
        ++compressed_count;
      }
    });

  rtabmap_msgs::msg::RGBDImage rgbd;
  rgbd.header.stamp.sec = 123;
  rgbd.header.stamp.nanosec = 456;
  rgbd.rgb.header = rgbd.header;
  rgbd.rgb.height = 1;
  rgbd.rgb.width = 1;
  rgbd.rgb.encoding = "mono8";
  rgbd.rgb.step = 1;
  rgbd.rgb.data = {42};
  rgbd.depth = rgbd.rgb;
  rgbd.depth.encoding = "16UC1";
  rgbd.depth.step = 2;
  rgbd.depth.data = {0xe8, 0x03};
  rgbd.rgb_camera_info.header = rgbd.header;
  rgbd.rgb_camera_info.header.stamp.sec = 124;
  rgbd.rgb_camera_info.width = 1;
  rgbd.rgb_camera_info.height = 1;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(adapter);
  executor.add_node(test_node);
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (std::chrono::steady_clock::now() < deadline &&
    (color_count == 0 || depth_count == 0 || info_count == 0 ||
    rgbd_count == 0 || compressed_count == 0))
  {
    rgbd_input->publish(rgbd);
    executor.spin_some();
    std::this_thread::sleep_for(20ms);
  }

  EXPECT_GT(color_count, 0);
  EXPECT_GT(depth_count, 0);
  EXPECT_GT(info_count, 0);
  EXPECT_GT(rgbd_count, 0);
  EXPECT_GT(compressed_count, 0);
  EXPECT_EQ(received_stamp.sec, 123);
  EXPECT_EQ(received_stamp.nanosec, 456u);
  EXPECT_EQ(received_info_stamp.sec, 123);
  EXPECT_EQ(received_info_stamp.nanosec, 456u);
  executor.remove_node(test_node);
  executor.remove_node(adapter);
}

}  // namespace
