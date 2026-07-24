#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "octomap/AbstractOcTree.h"
#include "octomap/OcTree.h"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"

class OctomapFileLoaderNode : public rclcpp::Node
{
public:
  OctomapFileLoaderNode() : Node("octomap_file_loader")
  {
    declare_parameter<std::string>("initial_octomap_path", "");
    declare_parameter<std::string>("load_path_topic", "/navigation/load_octomap_path");
    declare_parameter<std::string>("octomap_topic", "/navigation/octomap");
    declare_parameter<std::string>("marker_topic", "/navigation/occupied_voxels");
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<int>("max_marker_points", 12000);

    octomap_pub_ = create_publisher<octomap_msgs::msg::Octomap>(
      get_parameter("octomap_topic").as_string(), rclcpp::QoS(1).reliable().transient_local());
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      get_parameter("marker_topic").as_string(), rclcpp::QoS(1).reliable().transient_local());
    load_sub_ = create_subscription<std_msgs::msg::String>(
      get_parameter("load_path_topic").as_string(), rclcpp::QoS(10).reliable(),
      [this](const std_msgs::msg::String::SharedPtr message) { load(message->data); });

    const auto initial_path = get_parameter("initial_octomap_path").as_string();
    if (!initial_path.empty()) {
      load(initial_path);
    }
  }

private:
  void load(const std::string & path)
  {
    std::unique_ptr<octomap::OcTree> loaded_tree;
    if (path.size() >= 3 && path.substr(path.size() - 3) == ".bt") {
      loaded_tree = std::make_unique<octomap::OcTree>(0.1);
      if (!loaded_tree->readBinary(path)) {
        loaded_tree.reset();
      }
    } else {
      std::unique_ptr<octomap::AbstractOcTree> raw_tree(
        octomap::AbstractOcTree::read(path));
      auto * raw_octree = dynamic_cast<octomap::OcTree *>(raw_tree.release());
      loaded_tree.reset(raw_octree);
    }
    if (loaded_tree == nullptr) {
      RCLCPP_ERROR(get_logger(), "Cannot load OcTree from '%s'", path.c_str());
      return;
    }
    octree_ = std::move(loaded_tree);
    octomap_msgs::msg::Octomap map_message;
    if (!octomap_msgs::binaryMapToMsg(*octree_, map_message)) {
      RCLCPP_ERROR(get_logger(), "Cannot serialize OctoMap '%s'", path.c_str());
      return;
    }
    map_message.header.stamp = now();
    map_message.header.frame_id = get_parameter("frame_id").as_string();
    octomap_pub_->publish(map_message);
    publishMarker(map_message.header.frame_id);
    RCLCPP_INFO(
      get_logger(), "Loaded OctoMap: %s (%zu nodes, %.3f m)", path.c_str(),
      octree_->size(), octree_->getResolution());
  }

  void publishMarker(const std::string & frame_id)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = frame_id;
    marker.ns = "occupied_voxels";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    const double resolution = octree_->getResolution();
    marker.scale.x = resolution;
    marker.scale.y = resolution;
    marker.scale.z = resolution;
    marker.color.r = 0.15F;
    marker.color.g = 0.85F;
    marker.color.b = 0.95F;
    marker.color.a = 0.65F;

    std::vector<geometry_msgs::msg::Point> occupied;
    for (auto iterator = octree_->begin_leafs(); iterator != octree_->end_leafs(); ++iterator) {
      if (!octree_->isNodeOccupied(*iterator)) {
        continue;
      }
      geometry_msgs::msg::Point point;
      point.x = iterator.getX();
      point.y = iterator.getY();
      point.z = iterator.getZ();
      occupied.push_back(point);
    }
    const auto maximum = static_cast<std::size_t>(std::max<int64_t>(
      1, get_parameter("max_marker_points").as_int()));
    const auto stride = std::max<std::size_t>(1, (occupied.size() + maximum - 1) / maximum);
    marker.points.reserve((occupied.size() + stride - 1) / stride);
    for (std::size_t index = 0; index < occupied.size(); index += stride) {
      marker.points.push_back(occupied[index]);
    }
    marker_pub_->publish(marker);
  }

  std::unique_ptr<octomap::OcTree> octree_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr load_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OctomapFileLoaderNode>());
  rclcpp::shutdown();
  return 0;
}
