#include <fstream>
#include <chrono>
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
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"

#include "luxi_3d_navigation/terrain_model.hpp"
#include "luxi_3d_navigation/terrain_cloud_classifier.hpp"

class Octomap3DAstarPlannerNode : public rclcpp::Node
{
public:
  Octomap3DAstarPlannerNode()
  : Node("octomap_3d_astar_planner"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    declare_parameter<std::string>("octomap_topic", "/navigation/octomap");
    declare_parameter<std::string>("goal_topic", "/navigation/goal_pose");
    declare_parameter<std::string>("path_topic", "/navigation/planned_path");
    declare_parameter<std::string>("planning_status_topic", "/navigation/planning_status");
    declare_parameter<std::string>("terrain_pose_topic", "/navigation/terrain_pose");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<std::string>("semantic_path", "");
    declare_parameter<std::string>("cloud_path", "");
    declare_parameter<double>("robot_radius", 0.10);
    declare_parameter<double>("robot_height", 0.35);
    declare_parameter<double>("body_reference_height", 0.35);
    declare_parameter<double>("max_step_height", 0.15);
    declare_parameter<double>("max_slope_degrees", 50.0);
    declare_parameter<int>("ground_support_xy_radius_cells", 1);
    declare_parameter<int>("ground_support_depth_cells", 2);
    declare_parameter<bool>("strict_direct_ground_support", false);
    declare_parameter<int>("snap_search_radius_cells", 12);
    declare_parameter<int>("max_iterations", 500000);
    declare_parameter<double>("costmap_margin", 0.60);
    declare_parameter<double>("costmap_weight", 8.0);
    declare_parameter<double>("ground_normal_radius", 0.30);
    declare_parameter<double>("ground_max_slope_degrees", 35.0);
    declare_parameter<double>("obstacle_min_height", 0.15);
    declare_parameter<std::string>(
      "terrain_obstacle_topic", "/navigation/terrain/obstacles");
    declare_parameter<std::string>(
      "terrain_traversable_topic", "/navigation/terrain/traversable");
    declare_parameter<std::string>(
      "terrain_costmap_topic", "/navigation/terrain/costmap");

    pit_polygons_ = loadPitPolygons(get_parameter("semantic_path").as_string());
    octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
      get_parameter("octomap_topic").as_string(), rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&Octomap3DAstarPlannerNode::onOctomap, this, std::placeholders::_1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      get_parameter("goal_topic").as_string(), rclcpp::QoS(10).reliable(),
      std::bind(&Octomap3DAstarPlannerNode::onGoal, this, std::placeholders::_1));
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      get_parameter("path_topic").as_string(), rclcpp::QoS(1).reliable().transient_local());
    planning_status_pub_ = create_publisher<std_msgs::msg::String>(
      get_parameter("planning_status_topic").as_string(),
      rclcpp::QoS(1).reliable().transient_local());
    terrain_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      get_parameter("terrain_pose_topic").as_string(), 10);
    obstacle_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      get_parameter("terrain_obstacle_topic").as_string(),
      rclcpp::QoS(1).reliable().transient_local());
    traversable_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      get_parameter("terrain_traversable_topic").as_string(),
      rclcpp::QoS(1).reliable().transient_local());
    costmap_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      get_parameter("terrain_costmap_topic").as_string(),
      rclcpp::QoS(1).reliable().transient_local());
    terrain_pose_timer_ = create_wall_timer(
      std::chrono::milliseconds(200), [this]() {publishTerrainPose();});
    publishPlanningStatus("waiting_map");
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
    result.costmap_margin = get_parameter("costmap_margin").as_double();
    result.costmap_weight = get_parameter("costmap_weight").as_double();
    return result;
  }

  visualization_msgs::msg::Marker layerMarker(
    const std::string & name, float red, float green, float blue, float alpha) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = map_frame_;
    marker.ns = name;
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = terrain_->resolution();
    marker.scale.y = terrain_->resolution();
    marker.scale.z = terrain_->resolution();
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;
    return marker;
  }

  void publishTerrainLayers()
  {
    const auto & layers = terrain_->layers();
    auto obstacles = layerMarker("terrain_obstacles", 0.95F, 0.18F, 0.16F, 0.85F);
    obstacles.points.reserve(layers.obstacle_cells.size());
    for (const auto & cell : layers.obstacle_cells) {
      const auto point = terrain_->gridToWorld(cell);
      geometry_msgs::msg::Point output;
      output.x = point.x();
      output.y = point.y();
      output.z = point.z();
      obstacles.points.push_back(output);
    }

    auto traversable = layerMarker("terrain_traversable", 0.20F, 0.90F, 0.52F, 0.30F);
    auto costmap = layerMarker("terrain_costmap", 1.0F, 0.75F, 0.10F, 0.72F);
    traversable.points.reserve(layers.traversable_cells.size());
    for (const auto & entry : layers.traversable_cells) {
      const auto point = terrain_->gridToWorld(entry.cell);
      geometry_msgs::msg::Point output;
      output.x = point.x();
      output.y = point.y();
      output.z = point.z();
      traversable.points.push_back(output);
      if (entry.cost <= 0.0) {
        continue;
      }
      costmap.points.push_back(output);
      std_msgs::msg::ColorRGBA color;
      color.r = 1.0F;
      color.g = static_cast<float>(1.0 - entry.cost);
      color.b = 0.08F;
      color.a = static_cast<float>(0.35 + 0.55 * entry.cost);
      costmap.colors.push_back(color);
    }
    obstacle_pub_->publish(obstacles);
    traversable_pub_->publish(traversable);
    costmap_pub_->publish(costmap);
    RCLCPP_INFO(
      get_logger(), "Terrain layers: obstacles=%zu traversable=%zu cost_edges=%zu",
      obstacles.points.size(), traversable.points.size(), costmap.points.size());
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
    const auto cloud_path = get_parameter("cloud_path").as_string();
    if (!cloud_path.empty() && !terrain_observation_) {
      try {
        luxi_3d_navigation::TerrainCloudParameters cloud_parameters;
        cloud_parameters.resolution = octree_->getResolution();
        cloud_parameters.normal_radius = get_parameter("ground_normal_radius").as_double();
        cloud_parameters.maximum_ground_slope_degrees =
          get_parameter("ground_max_slope_degrees").as_double();
        cloud_parameters.obstacle_min_height = get_parameter("obstacle_min_height").as_double();
        terrain_observation_ = luxi_3d_navigation::classifyTerrainCloudFile(
          cloud_path, cloud_parameters);
        RCLCPP_INFO(
          get_logger(), "Point-cloud terrain fitted: ground=%zu obstacles=%zu",
          terrain_observation_->ground_cells.size(),
          terrain_observation_->obstacle_cells.size());
      } catch (const std::exception & error) {
        RCLCPP_ERROR(
          get_logger(), "Cannot fit point-cloud terrain '%s': %s",
          cloud_path.c_str(), error.what());
      }
    }
    terrain_ = std::make_unique<luxi_3d_navigation::TerrainModel>(
      *octree_, parameters(), pit_polygons_, terrain_observation_);
    if (!message->header.frame_id.empty()) {
      map_frame_ = message->header.frame_id;
    }
    publishTerrainLayers();
    RCLCPP_INFO(
      get_logger(), "Ground-supported 3D map ready: nodes=%zu resolution=%.3f frame=%s",
      octree_->size(), octree_->getResolution(), map_frame_.c_str());
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr goal)
  {
    publishPlanningStatus("planning");
    if (!terrain_) {
      RCLCPP_WARN(get_logger(), "Ignoring goal: no OctoMap has arrived");
      publishPath({});
      publishPlanningStatus("failed_map_unavailable");
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
      publishPath({});
      publishPlanningStatus("failed_localization_unavailable");
    }
  }

  void publishTerrainPose()
  {
    if (!terrain_) {
      return;
    }
    try {
      const auto transform = tf_buffer_.lookupTransform(
        map_frame_, get_parameter("base_frame").as_string(), tf2::TimePointZero);
      const double terrain_z = transform.transform.translation.z -
        get_parameter("body_reference_height").as_double();
      const auto ground = terrain_->snapToTerrainAtXY(terrain_->worldToGrid(
        transform.transform.translation.x, transform.transform.translation.y, terrain_z));
      if (!ground) {
        return;
      }
      const auto point = terrain_->gridToWorld(*ground);
      geometry_msgs::msg::PoseStamped pose;
      pose.header.stamp = now();
      pose.header.frame_id = map_frame_;
      pose.pose.position.x = transform.transform.translation.x;
      pose.pose.position.y = transform.transform.translation.y;
      pose.pose.position.z = point.z();
      pose.pose.orientation = transform.transform.rotation;
      terrain_pose_pub_->publish(pose);
    } catch (const tf2::TransformException &) {
      return;
    }
  }

  void plan(double start_x, double start_y, double start_z, const geometry_msgs::msg::PoseStamped & goal)
  {
    const double terrain_start_z =
      start_z - get_parameter("body_reference_height").as_double();
    const auto start = terrain_->snapToTerrainAtXY(
      terrain_->worldToGrid(start_x, start_y, terrain_start_z));
    const auto target = terrain_->snapGoalToTerrain(terrain_->worldToGrid(
      goal.pose.position.x, goal.pose.position.y, goal.pose.position.z));
    if (!start || !target) {
      RCLCPP_ERROR(
        get_logger(), "No supported terrain near start or goal (snap radius=%ld cells)",
        get_parameter("snap_search_radius_cells").as_int());
      publishPath({});
      publishPlanningStatus("failed_start_or_goal_unsupported");
      return;
    }
    const auto cells = terrain_->plan(*start, *target);
    if (cells.empty()) {
      RCLCPP_WARN(get_logger(), "No ground-supported 3D A* path found");
      publishPath({});
      publishPlanningStatus("failed_no_path");
      return;
    }
    publishPath(cells);
    publishPlanningStatus("ready");
  }

  void publishPlanningStatus(const std::string & status)
  {
    std_msgs::msg::String message;
    message.data = status;
    planning_status_pub_->publish(message);
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
  std::optional<luxi_3d_navigation::TerrainObservation> terrain_observation_;
  std::vector<luxi_3d_navigation::Polygon2D> pit_polygons_;
  std::string map_frame_{"map"};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr planning_status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr terrain_pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traversable_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr costmap_pub_;
  rclcpp::TimerBase::SharedPtr terrain_pose_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Octomap3DAstarPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
