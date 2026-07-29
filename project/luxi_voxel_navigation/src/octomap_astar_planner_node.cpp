#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "octomap/OcTree.h"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "luxi_voxel_navigation/voxel_astar.hpp"

class OctomapAstarPlannerNode : public rclcpp::Node
{
public:
  OctomapAstarPlannerNode()
  : Node("octomap_astar_planner"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    declare_parameter<std::string>("octomap_topic", "/navigation/octomap");
    declare_parameter<std::string>("goal_topic", "/navigation/goal_pose");
    declare_parameter<std::string>("path_topic", "/navigation/planned_path");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<double>("obstacle_min_z", -0.10);
    declare_parameter<double>("obstacle_max_z", 1.00);
    declare_parameter<double>("robot_radius", 0.18);
    declare_parameter<double>("map_padding_m", 0.50);
    declare_parameter<int>("snap_radius_cells", 8);
    declare_parameter<int>("max_grid_cells", 250000);
    declare_parameter<bool>("allow_diagonal", true);

    octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
      get_parameter("octomap_topic").as_string(), rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&OctomapAstarPlannerNode::onOctomap, this, std::placeholders::_1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      get_parameter("goal_topic").as_string(), rclcpp::QoS(10).reliable(),
      std::bind(&OctomapAstarPlannerNode::onGoal, this, std::placeholders::_1));
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      get_parameter("path_topic").as_string(), rclcpp::QoS(1).reliable().transient_local());
  }

private:
  void onOctomap(const octomap_msgs::msg::Octomap::SharedPtr message)
  {
    std::unique_ptr<octomap::AbstractOcTree> tree(octomap_msgs::msgToMap(*message));
    auto * occupancy_tree = dynamic_cast<octomap::OcTree *>(tree.release());
    if (occupancy_tree == nullptr) {
      RCLCPP_ERROR(get_logger(), "Expected an OcTree OctoMap message");
      return;
    }
    octree_.reset(occupancy_tree);
    if (!message->header.frame_id.empty()) {
      map_frame_ = message->header.frame_id;
    }
    RCLCPP_INFO(get_logger(), "OctoMap ready: occupied_voxels=%zu frame=%s", octree_->size(), map_frame_.c_str());
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr goal)
  {
    if (!octree_) {
      RCLCPP_WARN(get_logger(), "Ignoring goal: no OctoMap has arrived");
      return;
    }
    try {
      const auto transform = tf_buffer_.lookupTransform(
        map_frame_, get_parameter("base_frame").as_string(), tf2::TimePointZero);
      plan(
        transform.transform.translation.x, transform.transform.translation.y,
        goal->pose.position.x, goal->pose.position.y);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN(get_logger(), "Cannot obtain localization transform: %s", error.what());
    }
  }

  bool snapFree(
    int width, int height, const std::vector<std::uint8_t> & blocked,
    luxi_voxel_navigation::GridCell input, luxi_voxel_navigation::GridCell & output) const
  {
    const int radius = std::max(
      0, static_cast<int>(get_parameter("snap_radius_cells").as_int()));
    for (int distance = 0; distance <= radius; ++distance) {
      for (int dy = -distance; dy <= distance; ++dy) {
        for (int dx = -distance; dx <= distance; ++dx) {
          if (std::max(std::abs(dx), std::abs(dy)) != distance) {
            continue;
          }
          const luxi_voxel_navigation::GridCell candidate{input.x + dx, input.y + dy};
          if (luxi_voxel_navigation::isInsideGrid(width, height, candidate) &&
            blocked[luxi_voxel_navigation::gridOffset(width, candidate)] == 0)
          {
            output = candidate;
            return true;
          }
        }
      }
    }
    return false;
  }

  void plan(double start_x, double start_y, double goal_x, double goal_y)
  {
    double min_x, min_y, min_z, max_x, max_y, max_z;
    octree_->getMetricMin(min_x, min_y, min_z);
    octree_->getMetricMax(max_x, max_y, max_z);
    const double resolution = octree_->getResolution();
    const double padding = std::max(0.0, get_parameter("map_padding_m").as_double());
    const int origin_x = static_cast<int>(std::floor((min_x - padding) / resolution));
    const int origin_y = static_cast<int>(std::floor((min_y - padding) / resolution));
    const int max_cell_x = static_cast<int>(std::floor((max_x + padding) / resolution));
    const int max_cell_y = static_cast<int>(std::floor((max_y + padding) / resolution));
    const int width = max_cell_x - origin_x + 1;
    const int height = max_cell_y - origin_y + 1;
    if (width <= 0 || height <= 0 || width * height > get_parameter("max_grid_cells").as_int()) {
      RCLCPP_ERROR(get_logger(), "Refusing invalid or oversized navigation grid: %d x %d", width, height);
      return;
    }
    std::vector<std::uint8_t> blocked(static_cast<std::size_t>(width * height), 0);
    const double obstacle_min_z = get_parameter("obstacle_min_z").as_double();
    const double obstacle_max_z = get_parameter("obstacle_max_z").as_double();
    for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
      if (!octree_->isNodeOccupied(*it) || it.getZ() < obstacle_min_z || it.getZ() > obstacle_max_z) {
        continue;
      }
      const int cell_x = static_cast<int>(std::floor(it.getX() / resolution)) - origin_x;
      const int cell_y = static_cast<int>(std::floor(it.getY() / resolution)) - origin_y;
      const luxi_voxel_navigation::GridCell cell{cell_x, cell_y};
      if (luxi_voxel_navigation::isInsideGrid(width, height, cell)) {
        blocked[luxi_voxel_navigation::gridOffset(width, cell)] = 1;
      }
    }
    inflateObstacles(width, height, resolution, blocked);
    const auto to_grid = [origin_x, origin_y, resolution](double x, double y) {
        return luxi_voxel_navigation::GridCell{
          static_cast<int>(std::floor(x / resolution)) - origin_x,
          static_cast<int>(std::floor(y / resolution)) - origin_y};
      };
    luxi_voxel_navigation::GridCell start;
    luxi_voxel_navigation::GridCell goal;
    if (!snapFree(width, height, blocked, to_grid(start_x, start_y), start) ||
      !snapFree(width, height, blocked, to_grid(goal_x, goal_y), goal))
    {
      RCLCPP_ERROR(get_logger(), "Start or goal has no nearby free voxel cell");
      return;
    }
    const auto cells = luxi_voxel_navigation::planAstar(
      width, height, blocked, start, goal, get_parameter("allow_diagonal").as_bool());
    if (cells.empty()) {
      RCLCPP_WARN(get_logger(), "No voxel path found");
      return;
    }
    publishPath(cells, origin_x, origin_y, resolution);
  }

  void inflateObstacles(
    int width, int height, double resolution, std::vector<std::uint8_t> & blocked) const
  {
    const int radius = static_cast<int>(std::ceil(get_parameter("robot_radius").as_double() / resolution));
    if (radius <= 0) {
      return;
    }
    const std::vector<std::uint8_t> original = blocked;
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        const luxi_voxel_navigation::GridCell obstacle{x, y};
        if (original[luxi_voxel_navigation::gridOffset(width, obstacle)] == 0) {
          continue;
        }
        for (int dy = -radius; dy <= radius; ++dy) {
          for (int dx = -radius; dx <= radius; ++dx) {
            if (dx * dx + dy * dy > radius * radius) {
              continue;
            }
            const luxi_voxel_navigation::GridCell cell{x + dx, y + dy};
            if (luxi_voxel_navigation::isInsideGrid(width, height, cell)) {
              blocked[luxi_voxel_navigation::gridOffset(width, cell)] = 1;
            }
          }
        }
      }
    }
  }

  void publishPath(
    const std::vector<luxi_voxel_navigation::GridCell> & cells,
    int origin_x, int origin_y, double resolution)
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = map_frame_;
    for (std::size_t index = 0; index < cells.size(); ++index) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = (static_cast<double>(cells[index].x + origin_x) + 0.5) * resolution;
      pose.pose.position.y = (static_cast<double>(cells[index].y + origin_y) + 0.5) * resolution;
      pose.pose.orientation.w = 1.0;
      if (index + 1 < cells.size()) {
        const double yaw = std::atan2(
          static_cast<double>(cells[index + 1].y - cells[index].y),
          static_cast<double>(cells[index + 1].x - cells[index].x));
        tf2::Quaternion orientation;
        orientation.setRPY(0.0, 0.0, yaw);
        pose.pose.orientation = tf2::toMsg(orientation);
      }
      path.poses.push_back(pose);
    }
    path_pub_->publish(path);
    RCLCPP_INFO(get_logger(), "Published voxel path with %zu poses", path.poses.size());
  }

  std::unique_ptr<octomap::OcTree> octree_;
  std::string map_frame_{"map"};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OctomapAstarPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
