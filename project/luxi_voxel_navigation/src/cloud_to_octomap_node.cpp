#include <cmath>
#include <filesystem>
#include <memory>
#include <string>

#include "octomap/OcTree.h"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "luxi_voxel_navigation/octomap_defaults.hpp"

class CloudToOctomapNode : public rclcpp::Node
{
public:
  CloudToOctomapNode()
  : Node("cloud_to_octomap")
  {
    declare_parameter<std::string>("cloud_topic", "/rtabmap/cloud_map");
    declare_parameter<std::string>("octomap_topic", "/navigation/octomap");
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<std::string>("save_bt_path", "");
    declare_parameter<double>(
      "resolution", luxi_voxel_navigation::kDefaultOctomapResolution);
    declare_parameter<double>("min_z", -1.0e9);
    declare_parameter<double>("max_z", 1.0e9);

    const auto cloud_topic = get_parameter("cloud_topic").as_string();
    const auto octomap_topic = get_parameter("octomap_topic").as_string();
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic, rclcpp::SensorDataQoS(),
      std::bind(&CloudToOctomapNode::onCloud, this, std::placeholders::_1));
    octomap_pub_ = create_publisher<octomap_msgs::msg::Octomap>(
      octomap_topic, rclcpp::QoS(1).reliable().transient_local());
    RCLCPP_INFO(
      get_logger(), "cloud_to_octomap: cloud=%s octomap=%s resolution=%.3f",
      cloud_topic.c_str(), octomap_topic.c_str(), get_parameter("resolution").as_double());
  }

private:
  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr message)
  {
    const double resolution = get_parameter("resolution").as_double();
    if (resolution <= 0.0) {
      RCLCPP_ERROR(get_logger(), "resolution must be positive");
      return;
    }
    const double min_z = get_parameter("min_z").as_double();
    const double max_z = get_parameter("max_z").as_double();
    if (min_z > max_z) {
      RCLCPP_ERROR(get_logger(), "min_z must not exceed max_z");
      return;
    }
    const auto has_field = [message](const std::string & name) {
        for (const auto & field : message->fields) {
          if (field.name == name) {
            return true;
          }
        }
        return false;
      };
    if (!has_field("x") || !has_field("y") || !has_field("z")) {
      RCLCPP_ERROR(get_logger(), "PointCloud2 must contain x, y, and z fields");
      return;
    }

    octomap::OcTree tree(resolution);
    std::size_t accepted = 0;
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(*message, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(*message, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(*message, "z");
      for (; x != x.end(); ++x, ++y, ++z) {
        if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z) ||
          *z < min_z || *z > max_z)
        {
          continue;
        }
        tree.updateNode(octomap::point3d(*x, *y, *z), true);
        ++accepted;
      }
    } catch (const std::runtime_error & error) {
      RCLCPP_ERROR(get_logger(), "Unable to read point cloud XYZ fields: %s", error.what());
      return;
    }
    if (accepted == 0) {
      RCLCPP_WARN(get_logger(), "No finite points passed the configured z filter");
      return;
    }

    tree.updateInnerOccupancy();
    octomap_msgs::msg::Octomap octomap_message;
    if (!octomap_msgs::binaryMapToMsg(tree, octomap_message)) {
      RCLCPP_ERROR(get_logger(), "Failed to serialize OctoMap");
      return;
    }
    octomap_message.header = message->header;
    if (octomap_message.header.frame_id.empty()) {
      octomap_message.header.frame_id = get_parameter("frame_id").as_string();
    }
    octomap_pub_->publish(octomap_message);
    saveTree(tree);
    RCLCPP_INFO(
      get_logger(), "Published OctoMap: source_points=%u accepted=%zu occupied_voxels=%zu",
      message->width * message->height, accepted, tree.size());
  }

  void saveTree(octomap::OcTree & tree) const
  {
    const std::string path = get_parameter("save_bt_path").as_string();
    if (path.empty()) {
      return;
    }
    const std::filesystem::path output(path);
    std::filesystem::create_directories(output.parent_path());
    if (!tree.writeBinary(path)) {
      RCLCPP_ERROR(get_logger(), "Unable to save OctoMap: %s", path.c_str());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudToOctomapNode>());
  rclcpp::shutdown();
  return 0;
}
