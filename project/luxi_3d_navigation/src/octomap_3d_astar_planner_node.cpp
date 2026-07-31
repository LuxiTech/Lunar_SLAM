#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nlohmann/json.hpp"
#include "octomap/OcTree.h"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "luxi_3d_navigation/terrain_model.hpp"

class Octomap3DAstarPlannerNode : public rclcpp::Node
{
public:
  Octomap3DAstarPlannerNode()
  : Node("octomap_3d_astar_planner"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    declare_parameter<std::string>("octomap_topic", "/navigation/octomap");
    declare_parameter<std::string>("goal_topic", "/navigation/goal_pose");
    declare_parameter<std::string>("path_topic", "/navigation/planned_path");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<std::string>("semantic_path", "");
    declare_parameter<double>("robot_radius", 0.18);
    declare_parameter<double>("robot_height", 0.35);
    declare_parameter<double>("max_step_height", 0.15);
    declare_parameter<double>("max_slope_degrees", 50.0);
    declare_parameter<int>("ground_support_xy_radius_cells", 1);
    declare_parameter<int>("ground_support_depth_cells", 2);
    declare_parameter<bool>("strict_direct_ground_support", false);
    declare_parameter<int>("snap_search_radius_cells", 12);
    declare_parameter<int>("max_iterations", 500000);

    pit_polygons_ = loadPitPolygons(get_parameter("semantic_path").as_string());
    octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
      get_parameter("octomap_topic").as_string(), rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&Octomap3DAstarPlannerNode::onOctomap, this, std::placeholders::_1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      get_parameter("goal_topic").as_string(), rclcpp::QoS(10).reliable(),
      std::bind(&Octomap3DAstarPlannerNode::onGoal, this, std::placeholders::_1));
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      get_parameter("path_topic").as_string(), rclcpp::QoS(1).reliable().transient_local());
  }

private:
  std::vector<luxi_3d_navigation::Polygon2D> loadPitPolygons(const std::string & path)
  {
    std::vector<luxi_3d_navigation::Polygon2D> result;
    if (path.empty()) {
      return result;
    }
    try {
      std::ifstream stream(path);
      if (!stream) {
        RCLCPP_WARN(get_logger(), "Semantic annotation is unavailable: %s", path.c_str());
        return result;
      }
      const auto document = nlohmann::json::parse(stream);
      for (const auto & pit : document.value("pits", nlohmann::json::array())) {
        if (pit.value("type", std::string()) != "pit") {
          continue;
        }
        luxi_3d_navigation::Polygon2D polygon;
        for (const auto & vertex : pit.value("polygon", nlohmann::json::array())) {
          if (vertex.is_array() && vertex.size() == 2U) {
            polygon.emplace_back(vertex[0].get<double>(), vertex[1].get<double>());
          }
        }
        if (polygon.size() >= 3U) {
          result.push_back(std::move(polygon));
        }
      }
      RCLCPP_INFO(get_logger(), "Loaded %zu semantic pit exclusion regions", result.size());
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Cannot parse semantic annotation '%s': %s", path.c_str(), error.what());
    }
    return result;
  }

  luxi_3d_navigation::TerrainParameters parameters() const
  {
    luxi_3d_navigation::TerrainParameters result;
    result.robot_radius = get_parameter("robot_radius").as_double();
    result.robot_height = get_parameter("robot_height").as_double();
    result.max_step_height = get_parameter("max_step_height").as_double();
    result.max_slope_degrees = get_parameter("max_slope_degrees").as_double();
    result.support_xy_radius_cells =
      static_cast<int>(get_parameter("ground_support_xy_radius_cells").as_int());
    result.support_depth_cells =
      static_cast<int>(get_parameter("ground_support_depth_cells").as_int());
    result.strict_direct_support = get_parameter("strict_direct_ground_support").as_bool();
    result.snap_radius_cells =
      static_cast<int>(get_parameter("snap_search_radius_cells").as_int());
    result.max_iterations = static_cast<std::size_t>(
      std::max<int64_t>(1, get_parameter("max_iterations").as_int()));
    return result;
  }

  void onOctomap(const octomap_msgs::msg::Octomap::SharedPtr message)
  {
    std::unique_ptr<octomap::AbstractOcTree> abstract_tree(octomap_msgs::msgToMap(*message));
    auto * tree = dynamic_cast<octomap::OcTree *>(abstract_tree.get());
    if (tree == nullptr) {
      RCLCPP_ERROR(get_logger(), "Expected an OcTree OctoMap message");
      return;
    }
    octree_.reset(static_cast<octomap::OcTree *>(abstract_tree.release()));
    terrain_ = std::make_unique<luxi_3d_navigation::TerrainModel>(
      *octree_, parameters(), pit_polygons_);
    if (!message->header.frame_id.empty()) {
      map_frame_ = message->header.frame_id;
    }
    RCLCPP_INFO(
      get_logger(), "Ground-supported 3D map ready: nodes=%zu resolution=%.3f frame=%s",
      octree_->size(), octree_->getResolution(), map_frame_.c_str());
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr goal)
  {
    if (!terrain_) {
      RCLCPP_WARN(get_logger(), "Ignoring goal: no OctoMap has arrived");
      return;
    }
    try {
      const auto transform = tf_buffer_.lookupTransform(
        map_frame_, get_parameter("base_frame").as_string(), tf2::TimePointZero);
      plan(
        transform.transform.translation.x, transform.transform.translation.y,
        transform.transform.translation.z, *goal);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN(get_logger(), "Cannot obtain the localized robot pose: %s", error.what());
    }
  }

  void plan(double start_x, double start_y, double start_z, const geometry_msgs::msg::PoseStamped & goal)
  {
    const auto start = terrain_->snapToTerrain(terrain_->worldToGrid(start_x, start_y, start_z));
    const auto target = terrain_->snapToTerrain(terrain_->worldToGrid(
      goal.pose.position.x, goal.pose.position.y, goal.pose.position.z));
    if (!start || !target) {
      RCLCPP_ERROR(
        get_logger(), "No supported terrain near start or goal (snap radius=%ld cells)",
        get_parameter("snap_search_radius_cells").as_int());
      publishPath({});
      return;
    }
    const auto cells = terrain_->plan(*start, *target);
    if (cells.empty()) {
      RCLCPP_WARN(get_logger(), "No ground-supported 3D A* path found");
      publishPath({});
      return;
    }
    publishPath(cells);
  }

  void publishPath(const std::vector<luxi_3d_navigation::GridCell3D> & cells)
  {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = map_frame_;
    for (std::size_t index = 0U; index < cells.size(); ++index) {
      const auto point = terrain_->gridToWorld(cells[index]);
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = point.x();
      pose.pose.position.y = point.y();
      pose.pose.position.z = point.z();
      pose.pose.orientation.w = 1.0;
      if (index + 1U < cells.size()) {
        const auto next = terrain_->gridToWorld(cells[index + 1U]);
        tf2::Quaternion orientation;
        orientation.setRPY(0.0, 0.0, std::atan2(next.y() - point.y(), next.x() - point.x()));
        pose.pose.orientation = tf2::toMsg(orientation);
      }
      path.poses.push_back(std::move(pose));
    }
    path_pub_->publish(path);
    RCLCPP_INFO(get_logger(), "Published ground-supported 3D A* path with %zu poses", path.poses.size());
  }

  std::unique_ptr<octomap::OcTree> octree_;
  std::unique_ptr<luxi_3d_navigation::TerrainModel> terrain_;
  std::vector<luxi_3d_navigation::Polygon2D> pit_polygons_;
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
  rclcpp::spin(std::make_shared<Octomap3DAstarPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
