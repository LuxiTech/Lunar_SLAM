#include <atomic>
#include <chrono>
#include <memory>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "luxi_adapter/sensor_adapter.hpp"
#include "rclcpp/rclcpp.hpp"
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
    compressed_color_count == 0))
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

}  // namespace
