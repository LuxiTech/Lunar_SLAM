#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
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
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "luxi_3d_navigation/terrain_cloud_classifier.hpp"
#include "luxi_3d_navigation/terrain_model.hpp"

namespace luxi_3d_navigation
{

class LocalPathReplannerNode : public rclcpp::Node
{
public:
  LocalPathReplannerNode()
  : Node("local_path_replanner"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    declare_parameter<std::string>("octomap_topic", "/navigation/octomap");
    declare_parameter<std::string>("global_path_topic", "/navigation/global_path");
    declare_parameter<std::string>("output_path_topic", "/navigation/planned_path");
    declare_parameter<std::string>(
      "obstacle_points_topic", "/navigation/local_obstacles/points");
    declare_parameter<std::string>(
      "obstacle_state_topic", "/navigation/local_obstacles/state");
    declare_parameter<std::string>("status_topic", "/navigation/local_replan/status");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<std::string>("semantic_path", "");
    declare_parameter<std::string>("cloud_path", "");
    declare_parameter<double>("body_reference_height", 0.35);
    declare_parameter<double>("dynamic_inflation_radius", 0.25);
    declare_parameter<double>("dynamic_replan_hysteresis", 0.05);
    declare_parameter<double>("active_path_blocked_hold", 0.50);
    declare_parameter<double>("rejoin_min_distance", 0.80);
    declare_parameter<double>("planning_horizon", 3.0);
    declare_parameter<double>("local_window_margin", 1.0);
    declare_parameter<double>("replan_rate", 2.0);
    declareTerrainParameters();

    path_pub_ = create_publisher<nav_msgs::msg::Path>(
      get_parameter("output_path_topic").as_string(),
      rclcpp::QoS(1).reliable().transient_local());
    status_pub_ = create_publisher<std_msgs::msg::String>(
      get_parameter("status_topic").as_string(),
      rclcpp::QoS(1).reliable().transient_local());
    octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
      get_parameter("octomap_topic").as_string(),
      rclcpp::QoS(1).reliable().transient_local(),
      [this](const octomap_msgs::msg::Octomap::SharedPtr message) {onOctomap(*message);});
    global_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      get_parameter("global_path_topic").as_string(),
      rclcpp::QoS(1).reliable().transient_local(),
      [this](const nav_msgs::msg::Path::SharedPtr message) {
        global_path_ = *message;
        active_path_.poses.clear();
        active_path_is_detour_ = false;
        active_path_blocked_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
        dirty_ = true;
        if (!global_path_.poses.empty()) {
          publishStatus("global_path_ready");
        }
      });
    obstacle_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      get_parameter("obstacle_points_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::PointCloud2::SharedPtr message) {
        onObstacles(*message);
      });
    obstacle_state_sub_ = create_subscription<std_msgs::msg::String>(
      get_parameter("obstacle_state_topic").as_string(),
      rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr message) {
        if (obstacle_state_ != message->data) {
          obstacle_state_ = message->data;
          dirty_ = true;
        }
      });
    const double rate = std::max(0.2, get_parameter("replan_rate").as_double());
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate), [this]() {updatePlan();});
    publishStatus("waiting_map");
  }

private:
  void declareTerrainParameters()
  {
    declare_parameter<double>("robot_radius", 0.10);
    declare_parameter<double>("robot_height", 0.35);
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
  }

  TerrainParameters terrainParameters() const
  {
    TerrainParameters result;
    result.robot_radius = get_parameter("robot_radius").as_double();
    result.robot_height = get_parameter("robot_height").as_double();
    result.max_step_height = get_parameter("max_step_height").as_double();
    result.max_slope_degrees = get_parameter("max_slope_degrees").as_double();
    result.support_xy_radius_cells = static_cast<int>(
      get_parameter("ground_support_xy_radius_cells").as_int());
    result.support_depth_cells = static_cast<int>(
      get_parameter("ground_support_depth_cells").as_int());
    result.strict_direct_support = get_parameter("strict_direct_ground_support").as_bool();
    result.snap_radius_cells = static_cast<int>(
      get_parameter("snap_search_radius_cells").as_int());
    result.max_iterations = static_cast<std::size_t>(std::max<int64_t>(
      1, get_parameter("max_iterations").as_int()));
    result.costmap_margin = get_parameter("costmap_margin").as_double();
    result.costmap_weight = get_parameter("costmap_weight").as_double();
    return result;
  }

  std::vector<Polygon2D> loadPitPolygons(const std::string & path) const
  {
    std::vector<Polygon2D> result;
    if (path.empty()) {
      return result;
    }
    try {
      std::ifstream stream(path);
      if (!stream) {
        return result;
      }
      const auto document = nlohmann::json::parse(stream);
      for (const auto & pit : document.value("pits", nlohmann::json::array())) {
        if (pit.value("type", std::string()) != "pit") {
          continue;
        }
        Polygon2D polygon;
        for (const auto & vertex : pit.value("polygon", nlohmann::json::array())) {
          if (vertex.is_array() && vertex.size() == 2U) {
            polygon.emplace_back(vertex[0].get<double>(), vertex[1].get<double>());
          }
        }
        if (polygon.size() >= 3U) {
          result.push_back(std::move(polygon));
        }
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Cannot load semantic exclusions: %s", error.what());
    }
    return result;
  }

  void onOctomap(const octomap_msgs::msg::Octomap & message)
  {
    std::unique_ptr<octomap::AbstractOcTree> abstract_tree(octomap_msgs::msgToMap(message));
    auto * tree = dynamic_cast<octomap::OcTree *>(abstract_tree.get());
    if (tree == nullptr) {
      publishStatus("failed_map_type");
      return;
    }
    octree_.reset(static_cast<octomap::OcTree *>(abstract_tree.release()));
    std::optional<TerrainObservation> observation;
    const auto cloud_path = get_parameter("cloud_path").as_string();
    if (!cloud_path.empty()) {
      try {
        TerrainCloudParameters parameters;
        parameters.resolution = octree_->getResolution();
        parameters.normal_radius = get_parameter("ground_normal_radius").as_double();
        parameters.maximum_ground_slope_degrees =
          get_parameter("ground_max_slope_degrees").as_double();
        parameters.obstacle_min_height = get_parameter("obstacle_min_height").as_double();
        observation = classifyTerrainCloudFile(cloud_path, parameters);
      } catch (const std::exception & error) {
        RCLCPP_ERROR(get_logger(), "Cannot classify terrain cloud: %s", error.what());
      }
    }
    terrain_ = std::make_unique<TerrainModel>(
      *octree_, terrainParameters(),
      loadPitPolygons(get_parameter("semantic_path").as_string()), std::move(observation));
    active_path_.poses.clear();
    active_path_is_detour_ = false;
    active_path_blocked_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    map_frame_ = message.header.frame_id.empty() ?
      get_parameter("map_frame").as_string() : message.header.frame_id;
    dirty_ = true;
    publishStatus("map_ready");
    RCLCPP_INFO(
      get_logger(), "Local replanner map ready: resolution=%.3f frame=%s",
      terrain_->resolution(), map_frame_.c_str());
  }

  void onObstacles(const sensor_msgs::msg::PointCloud2 & cloud)
  {
    dynamic_points_.clear();
    try {
      sensor_msgs::PointCloud2ConstIterator<float> x(cloud, "x");
      sensor_msgs::PointCloud2ConstIterator<float> y(cloud, "y");
      sensor_msgs::PointCloud2ConstIterator<float> z(cloud, "z");
      const auto count = static_cast<std::size_t>(cloud.width) * cloud.height;
      dynamic_points_.reserve(count);
      for (std::size_t index = 0U; index < count; ++index, ++x, ++y, ++z) {
        if (std::isfinite(*x) && std::isfinite(*y) && std::isfinite(*z)) {
          geometry_msgs::msg::Point point;
          point.x = *x;
          point.y = *y;
          point.z = *z;
          dynamic_points_.push_back(point);
        }
      }
      obstacle_frame_ = cloud.header.frame_id;
      obstacle_stamp_ = cloud.header.stamp;
      dirty_ = true;
    } catch (const std::runtime_error & error) {
      RCLCPP_WARN(get_logger(), "Invalid dynamic obstacle cloud: %s", error.what());
      obstacle_state_ = "error";
      dirty_ = true;
    }
  }

  GridColumnSet blockedColumns(double extra_inflation = 0.0)
  {
    GridColumnSet result;
    if (!terrain_ || dynamic_points_.empty()) {
      return result;
    }
    tf2::Transform map_from_obstacle;
    map_from_obstacle.setIdentity();
    if (!obstacle_frame_.empty() && obstacle_frame_ != map_frame_) {
      const auto transform = tf_buffer_.lookupTransform(
        map_frame_, obstacle_frame_, obstacle_stamp_, rclcpp::Duration::from_seconds(0.05));
      tf2::fromMsg(transform.transform, map_from_obstacle);
    }
    const double inflation = std::max(
      0.0, get_parameter("dynamic_inflation_radius").as_double() + extra_inflation);
    const int inflation_cells = std::max(
      0, static_cast<int>(std::ceil(inflation / terrain_->resolution())));
    for (const auto & point : dynamic_points_) {
      const auto transformed = map_from_obstacle * tf2::Vector3(point.x, point.y, point.z);
      const auto cell = terrain_->worldToGrid(transformed.x(), transformed.y(), transformed.z());
      for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
        for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
          if (std::hypot(static_cast<double>(dx), static_cast<double>(dy)) <=
            static_cast<double>(inflation_cells) + 1e-9)
          {
            result.insert(GridCell3D{cell.x + dx, cell.y + dy, 0});
          }
        }
      }
    }
    return result;
  }

  static std::size_t nearestPathIndex(
    const nav_msgs::msg::Path & path, double x, double y, double z)
  {
    std::size_t nearest = 0U;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0U; index < path.poses.size(); ++index) {
      const auto & point = path.poses[index].pose.position;
      const double distance = std::hypot(std::hypot(point.x - x, point.y - y), point.z - z);
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest = index;
      }
    }
    return nearest;
  }

  bool remainingPathBlocked(
    const nav_msgs::msg::Path & path, std::size_t start_index,
    const GridColumnSet & blocked, double x, double y) const
  {
    const double horizon = get_parameter("planning_horizon").as_double();
    for (std::size_t index = start_index; index < path.poses.size(); ++index) {
      const auto & point = path.poses[index].pose.position;
      if (std::hypot(point.x - x, point.y - y) > horizon) {
        break;
      }
      const auto cell = terrain_->worldToGrid(point.x, point.y, point.z);
      if (blocked.count(GridCell3D{cell.x, cell.y, 0}) != 0U) {
        return true;
      }
    }
    return false;
  }

  void updatePlan()
  {
    if (!dirty_) {
      if (!last_status_.empty()) {
        publishStatus(last_status_);
      }
      return;
    }
    if (global_path_.poses.empty()) {
      return;
    }
    if (!terrain_) {
      publishStatus("waiting_map");
      return;
    }
    if (obstacle_state_ == "stale" || obstacle_state_ == "tf_unavailable" ||
      obstacle_state_ == "camera_info_missing" || obstacle_state_ == "error")
    {
      publishStatus("sensor_stale");
      return;
    }
    try {
      const auto transform = tf_buffer_.lookupTransform(
        map_frame_, get_parameter("base_frame").as_string(), tf2::TimePointZero);
      const double robot_x = transform.transform.translation.x;
      const double robot_y = transform.transform.translation.y;
      const double robot_ground_z = transform.transform.translation.z -
        get_parameter("body_reference_height").as_double();
      const auto start = terrain_->snapToTerrainAtXY(
        terrain_->worldToGrid(robot_x, robot_y, robot_ground_z));
      if (!start) {
        publishStatus("failed_start_unsupported");
        return;
      }
      const auto blocked = blockedColumns();
      const std::size_t nearest = nearestPathIndex(
        global_path_, robot_x, robot_y, robot_ground_z);
      if (blocked.empty() ||
        !remainingPathBlocked(global_path_, nearest, blocked, robot_x, robot_y))
      {
        // Do not republish an unchanged global path for every depth frame: a
        // path publication is a meaningful update to the follower.
        if (active_path_.poses.empty() || active_path_is_detour_) {
          active_path_ = global_path_;
          active_path_is_detour_ = false;
          path_pub_->publish(global_path_);
        }
        active_path_blocked_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
        publishStatus("clear");
        dirty_ = false;
        return;
      }

      // A fresh depth frame should not force a stop-and-replan cycle when the
      // currently published detour is still clear.  The safety gate continues
      // to receive "ready", while a genuinely new obstacle intersecting the
      // active path immediately falls through to the fail-closed replanning
      // branch below.
      if (active_path_is_detour_ && !active_path_.poses.empty()) {
        const std::size_t active_nearest = nearestPathIndex(
          active_path_, robot_x, robot_y, robot_ground_z);
        if (!remainingPathBlocked(
            active_path_, active_nearest, blocked, robot_x, robot_y))
        {
          active_path_blocked_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
          publishStatus("ready");
          dirty_ = false;
          return;
        }
        const auto current_time = now();
        if (obstacle_state_ != "blocked") {
          if (active_path_blocked_since_.nanoseconds() == 0) {
            active_path_blocked_since_ = current_time;
          }
          if ((current_time - active_path_blocked_since_).seconds() <
            std::max(0.0, get_parameter("active_path_blocked_hold").as_double()))
          {
            publishStatus("ready");
            dirty_ = false;
            return;
          }
        }
      }

      active_path_blocked_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      publishStatus("planning");
      // Plan with a small extra margin, but validate the active path against
      // the configured safety inflation.  This hysteresis prevents centimetre
      // scale depth noise from repeatedly invalidating an otherwise safe path.
      const auto planning_blocked = blockedColumns(
        get_parameter("dynamic_replan_hysteresis").as_double());
      const double minimum = get_parameter("rejoin_min_distance").as_double();
      const double horizon = get_parameter("planning_horizon").as_double();
      std::optional<std::size_t> rejoin_index;
      for (std::size_t index = nearest + 1U; index < global_path_.poses.size(); ++index) {
        const auto & point = global_path_.poses[index].pose.position;
        const double distance = std::hypot(point.x - robot_x, point.y - robot_y);
        if (distance > horizon) {
          break;
        }
        const auto cell = terrain_->worldToGrid(point.x, point.y, point.z);
        if (distance >= minimum &&
          planning_blocked.count(GridCell3D{cell.x, cell.y, 0}) == 0U)
        {
          rejoin_index = index;
        }
      }
      if (!rejoin_index) {
        publishStatus("no_path");
        return;
      }
      const auto & rejoin_pose = global_path_.poses[*rejoin_index].pose.position;
      const auto goal = terrain_->snapToTerrainAtXY(
        terrain_->worldToGrid(rejoin_pose.x, rejoin_pose.y, rejoin_pose.z));
      if (!goal) {
        publishStatus("no_path");
        return;
      }
      const int margin_cells = std::max(1, static_cast<int>(std::ceil(
        get_parameter("local_window_margin").as_double() / terrain_->resolution())));
      const GridPlanningBounds bounds{
        std::min(start->x, goal->x) - margin_cells,
        std::max(start->x, goal->x) + margin_cells,
        std::min(start->y, goal->y) - margin_cells,
        std::max(start->y, goal->y) + margin_cells};
      const auto local_cells = terrain_->planAvoidingColumns(
        *start, *goal, planning_blocked, bounds);
      if (local_cells.empty()) {
        publishStatus("no_path");
        return;
      }
      nav_msgs::msg::Path output;
      output.header.stamp = now();
      output.header.frame_id = map_frame_;
      for (std::size_t index = 0U; index < local_cells.size(); ++index) {
        const auto point = terrain_->gridToWorld(local_cells[index]);
        geometry_msgs::msg::PoseStamped pose;
        pose.header = output.header;
        pose.pose.position.x = point.x();
        pose.pose.position.y = point.y();
        pose.pose.position.z = point.z();
        pose.pose.orientation.w = 1.0;
        if (index + 1U < local_cells.size()) {
          const auto next = terrain_->gridToWorld(local_cells[index + 1U]);
          tf2::Quaternion orientation;
          orientation.setRPY(0.0, 0.0, std::atan2(next.y() - point.y(), next.x() - point.x()));
          pose.pose.orientation = tf2::toMsg(orientation);
        }
        output.poses.push_back(std::move(pose));
      }
      output.poses.insert(
        output.poses.end(), global_path_.poses.begin() + static_cast<std::ptrdiff_t>(*rejoin_index + 1U),
        global_path_.poses.end());
      active_path_ = output;
      active_path_is_detour_ = true;
      path_pub_->publish(output);
      publishStatus("ready");
      dirty_ = false;
      RCLCPP_INFO(
        get_logger(), "Dynamic detour: local=%zu rejoin=%zu output=%zu blocked_columns=%zu",
        local_cells.size(), *rejoin_index, output.poses.size(), planning_blocked.size());
    } catch (const tf2::TransformException & error) {
      publishStatus("localization_unavailable");
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Local replanning TF unavailable: %s", error.what());
    }
  }

  void publishStatus(const std::string & status)
  {
    std_msgs::msg::String message;
    message.data = status;
    status_pub_->publish(message);
    last_status_ = status;
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<octomap::OcTree> octree_;
  std::unique_ptr<TerrainModel> terrain_;
  nav_msgs::msg::Path global_path_;
  nav_msgs::msg::Path active_path_;
  std::vector<geometry_msgs::msg::Point> dynamic_points_;
  std::string map_frame_{"map"};
  std::string obstacle_frame_;
  builtin_interfaces::msg::Time obstacle_stamp_;
  std::string obstacle_state_{"stale"};
  std::string last_status_;
  bool active_path_is_detour_{false};
  rclcpp::Time active_path_blocked_since_{0, 0, RCL_ROS_TIME};
  bool dirty_{true};
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr obstacle_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace luxi_3d_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<luxi_3d_navigation::LocalPathReplannerNode>());
  rclcpp::shutdown();
  return 0;
}
