#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "luxi_3d_navigation/localization_recovery_controller.hpp"
#include "luxi_3d_navigation/path_follower_control.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class TerrainPathFollowerNode : public rclcpp::Node
{
public:
  TerrainPathFollowerNode()
  : Node("terrain_path_follower"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    declare_parameter<std::string>("path_topic", "/navigation/planned_path");
    declare_parameter<std::string>("start_topic", "/navigation/start");
    declare_parameter<std::string>("stop_topic", "/navigation/stop");
    declare_parameter<std::string>("cmd_vel_topic", "/navigation/cmd_vel");
    declare_parameter<std::string>("active_topic", "/navigation/active");
    declare_parameter<std::string>("state_topic", "/navigation/follower_state");
    declare_parameter<std::string>("terrain_pose_topic", "/navigation/terrain_pose");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<double>("control_rate", 15.0);
    declare_parameter<double>("lookahead_m", 0.25);
    declare_parameter<double>("goal_tolerance_m", 0.12);
    declare_parameter<double>("minimum_safe_goal_tolerance_m", 0.12);
    declare_parameter<double>("linear_gain", 0.6);
    declare_parameter<double>("angular_gain", 1.2);
    declare_parameter<double>("max_linear_speed", 0.10);
    declare_parameter<double>("minimum_linear_speed", 0.10);
    declare_parameter<double>("max_angular_speed", 0.35);
    declare_parameter<double>("angular_deadband", 0.15);
    declare_parameter<double>("linear_heading_tolerance", 0.35);
    declare_parameter<double>("localization_timeout", 1.0);
    declare_parameter<std::string>("localization_health_topic", "/luxi_location/health");
    declare_parameter<std::string>(
      "localization_recovery_active_topic", "/navigation/localization_recovery_active");
    declare_parameter<std::string>(
      "relocalization_request_topic", "/luxi_location/relocalization_request");
    declare_parameter<double>("dead_reckoning_duration", 0.80);
    declare_parameter<double>("localization_recovery_timeout", 45.0);
    declare_parameter<double>("localization_recovery_angular_speed", 0.20);
    declare_parameter<double>("localization_recovery_confirmation_time", 1.0);
    declare_parameter<double>("max_path_deviation_m", 0.50);

    luxi_3d_navigation::LocalizationRecoveryParameters recovery_parameters;
    recovery_parameters.dead_reckoning_duration =
      get_parameter("dead_reckoning_duration").as_double();
    recovery_parameters.recovery_timeout =
      get_parameter("localization_recovery_timeout").as_double();
    recovery_parameters.healthy_confirmation_time =
      get_parameter("localization_recovery_confirmation_time").as_double();
    localization_recovery_controller_ =
      std::make_unique<luxi_3d_navigation::LocalizationRecoveryController>(
      recovery_parameters);

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      get_parameter("cmd_vel_topic").as_string(), 10);
    active_pub_ = create_publisher<std_msgs::msg::Bool>(
      get_parameter("active_topic").as_string(),
      rclcpp::QoS(1).reliable().transient_local());
    state_pub_ = create_publisher<std_msgs::msg::String>(
      get_parameter("state_topic").as_string(),
      rclcpp::QoS(1).reliable().transient_local());
    recovery_active_pub_ = create_publisher<std_msgs::msg::Bool>(
      get_parameter("localization_recovery_active_topic").as_string(),
      rclcpp::QoS(1).reliable().transient_local());
    relocalization_request_pub_ = create_publisher<std_msgs::msg::Bool>(
      get_parameter("relocalization_request_topic").as_string(), 10);
    terrain_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      get_parameter("terrain_pose_topic").as_string(), 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
        terrain_pose_ = *message;
      });
    localization_health_sub_ = create_subscription<std_msgs::msg::String>(
      get_parameter("localization_health_topic").as_string(),
      rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr message) {
        localization_health_ = message->data;
        localization_health_received_at_ = steadyNow();
      });

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      get_parameter("path_topic").as_string(), rclcpp::QoS(1).reliable().transient_local(),
      [this](const nav_msgs::msg::Path::SharedPtr message) {
        path_ = message->poses;
        if (path_.empty()) {
          setState(false, "waiting_path");
        } else if (active_) {
          publishState(state_);
          RCLCPP_INFO(
            get_logger(), "Updated active terrain path with %zu poses", path_.size());
        } else {
          setState(false, "plan_ready");
          RCLCPP_INFO(
            get_logger(), "Received %zu terrain path poses; waiting for explicit start",
            path_.size());
        }
      });
    start_sub_ = create_subscription<std_msgs::msg::Bool>(
      get_parameter("start_topic").as_string(), 10,
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        if (message->data) {
          localization_recovery_controller_->reset();
          last_path_turn_direction_ = 1.0;
        }
        setState(
          message->data && !path_.empty(),
          message->data && !path_.empty() ? "active" :
          (path_.empty() ? "waiting_path" : "stopped"));
      });
    stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      get_parameter("stop_topic").as_string(), 10,
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        if (message->data) {
          localization_recovery_controller_->reset();
          setState(false, "stopped");
        }
      });
    const double rate = std::max(1.0, get_parameter("control_rate").as_double());
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate), [this]() {control();});
    publishState("waiting_path");
  }

  ~TerrainPathFollowerNode() override
  {
    publishStop();
  }

private:
  void control()
  {
    if (!active_ || path_.empty()) {
      return;
    }
    const double now_seconds = steadyNow();
    const double health_timeout = get_parameter("localization_timeout").as_double();
    const bool health_fresh = localization_health_received_at_ >= 0.0 &&
      now_seconds - localization_health_received_at_ <= health_timeout;
    const std::string health = health_fresh ? localization_health_ : "searching";
    const auto recovery_action = localization_recovery_controller_->update(
      health, now_seconds);
    if (recovery_action == luxi_3d_navigation::LocalizationRecoveryAction::kStop) {
      setState(false, "localization_lost");
      RCLCPP_ERROR(
        get_logger(), "Localization recovery reached its bounded timeout; navigation stopped");
      return;
    }
    if (recovery_action == luxi_3d_navigation::LocalizationRecoveryAction::kRotate) {
      publishRecoveryRotation();
      return;
    }
    if (recovery_action == luxi_3d_navigation::LocalizationRecoveryAction::kHold) {
      publishRecoveryActive(false);
      publishStop();
      updateActiveState(
        health == "verifying" ? "localization_recovery_verifying" :
        "localization_recovery_confirming");
      return;
    }
    publishRecoveryActive(false);
    if (recovery_action == luxi_3d_navigation::LocalizationRecoveryAction::kDeadReckon) {
      updateActiveState("localization_dead_reckoning");
    } else if (state_ != "active") {
      updateActiveState("active");
      RCLCPP_INFO(get_logger(), "Confirmed localization recovery; resuming navigation");
    }

    try {
      const auto transform = tf_buffer_.lookupTransform(
        get_parameter("map_frame").as_string(), get_parameter("base_frame").as_string(),
        tf2::TimePointZero);
      const rclcpp::Time transform_stamp(transform.header.stamp);
      const double transform_age = (now() - transform_stamp).seconds();
      if (transform_stamp.nanoseconds() == 0 || transform_age < -0.1 ||
        transform_age > get_parameter("localization_timeout").as_double())
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Localization transform is stale (age=%.3fs)", transform_age);
        handleLocalizationFault();
        return;
      }
      if (!terrain_pose_) {
        handleLocalizationFault();
        return;
      }
      const rclcpp::Time terrain_stamp(terrain_pose_->header.stamp);
      const double terrain_age = (now() - terrain_stamp).seconds();
      if (terrain_stamp.nanoseconds() == 0 || terrain_age < -0.1 ||
        terrain_age > get_parameter("localization_timeout").as_double())
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Terrain-constrained pose is stale (age=%.3fs)", terrain_age);
        handleLocalizationFault();
        return;
      }
      follow(
        terrain_pose_->pose.position.x, terrain_pose_->pose.position.y,
        terrain_pose_->pose.position.z,
        tf2::getYaw(transform.transform.rotation));
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Localization unavailable: %s", error.what());
      handleLocalizationFault();
    }
  }

  void follow(double robot_x, double robot_y, double robot_z, double robot_yaw)
  {
    const auto & goal = path_.back().pose.position;
    const auto & previous = path_.size() > 1U ?
      path_[path_.size() - 2U].pose.position : goal;
    if (luxi_3d_navigation::pathGoalReached3D(
        robot_x, robot_y, robot_z, previous.x, previous.y,
        goal.x, goal.y, goal.z,
        get_parameter("goal_tolerance_m").as_double(),
        get_parameter("minimum_safe_goal_tolerance_m").as_double()))
    {
      setState(false, "goal_reached");
      RCLCPP_INFO(get_logger(), "3D terrain navigation goal reached");
      return;
    }

    std::size_t nearest = 0U;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0U; index < path_.size(); ++index) {
      const auto & point = path_[index].pose.position;
      const double distance = std::hypot(
        std::hypot(point.x - robot_x, point.y - robot_y), point.z - robot_z);
      if (distance < nearest_distance) {
        nearest = index;
        nearest_distance = distance;
      }
    }
    if (nearest_distance > get_parameter("max_path_deviation_m").as_double()) {
      RCLCPP_WARN(
        get_logger(), "Robot is %.3fm away from the planned path; stopping",
        nearest_distance);
      setState(false, "path_deviation");
      return;
    }
    const double lookahead = get_parameter("lookahead_m").as_double();
    const auto * target = &goal;
    for (std::size_t index = nearest; index < path_.size(); ++index) {
      const auto & point = path_[index].pose.position;
      if (std::hypot(point.x - robot_x, point.y - robot_y) >= lookahead) {
        target = &point;
        break;
      }
    }

    const double dx = target->x - robot_x;
    const double dy = target->y - robot_y;
    const double heading = std::atan2(dy, dx) - robot_yaw;
    const double normalized_heading = std::atan2(std::sin(heading), std::cos(heading));
    last_path_turn_direction_ = luxi_3d_navigation::updatedTurnDirection(
      normalized_heading, last_path_turn_direction_);
    const auto control = luxi_3d_navigation::pathFollowerCommand(
      std::hypot(dx, dy), normalized_heading,
      get_parameter("linear_gain").as_double(),
      get_parameter("angular_gain").as_double(),
      get_parameter("max_linear_speed").as_double(),
      get_parameter("max_angular_speed").as_double(),
      get_parameter("angular_deadband").as_double(),
      get_parameter("linear_heading_tolerance").as_double(),
      get_parameter("minimum_linear_speed").as_double());
    geometry_msgs::msg::Twist command;
    command.linear.x = control.linear_x;
    command.angular.z = control.angular_z;
    cmd_pub_->publish(command);
  }

  void publishStop() const
  {
    if (cmd_pub_) {
      cmd_pub_->publish(geometry_msgs::msg::Twist());
    }
  }

  static double steadyNow()
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  void handleLocalizationFault()
  {
    const double now_seconds = steadyNow();
    const auto action = localization_recovery_controller_->update("searching", now_seconds);
    if (action == luxi_3d_navigation::LocalizationRecoveryAction::kRotate) {
      publishRecoveryRotation();
      return;
    }
    publishRecoveryActive(false);
    publishStop();
    updateActiveState("localization_recovery_waiting");
  }

  void publishRecoveryRotation()
  {
    if (state_ != "localization_recovery_spin") {
      recovery_turn_direction_ = last_path_turn_direction_;
      std_msgs::msg::Bool request;
      request.data = true;
      relocalization_request_pub_->publish(request);
      RCLCPP_WARN(
        get_logger(),
        "Localization recovery rotation started: direction=%s angular_speed=%.3f rad/s; HLoc restart requested",
        recovery_turn_direction_ > 0.0 ? "left" : "right",
        get_parameter("localization_recovery_angular_speed").as_double());
    }
    geometry_msgs::msg::Twist command;
    command.angular.z = std::clamp(
      get_parameter("localization_recovery_angular_speed").as_double(), 0.0,
      get_parameter("max_angular_speed").as_double()) *
      recovery_turn_direction_;
    publishRecoveryActive(true);
    cmd_pub_->publish(command);
    updateActiveState("localization_recovery_spin");
  }

  void publishRecoveryActive(const bool active) const
  {
    if (!recovery_active_pub_) {
      return;
    }
    std_msgs::msg::Bool message;
    message.data = active;
    recovery_active_pub_->publish(message);
  }

  void updateActiveState(const std::string & state)
  {
    if (state_ == state) {
      return;
    }
    state_ = state;
    publishState(state_);
  }

  void publishState(const std::string & state) const
  {
    std_msgs::msg::Bool active;
    active.data = active_;
    active_pub_->publish(active);
    std_msgs::msg::String message;
    message.data = state;
    state_pub_->publish(message);
  }

  void setState(bool active, const std::string & state)
  {
    active_ = active;
    state_ = state;
    if (!active_) {
      publishRecoveryActive(false);
      publishStop();
    }
    publishState(state);
  }

  bool active_{false};
  std::string state_{"waiting_path"};
  std::vector<geometry_msgs::msg::PoseStamped> path_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr terrain_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr localization_health_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr recovery_active_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr relocalization_request_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::optional<geometry_msgs::msg::PoseStamped> terrain_pose_;
  std::string localization_health_{"searching"};
  double localization_health_received_at_{-1.0};
  double last_path_turn_direction_{1.0};
  double recovery_turn_direction_{1.0};
  std::unique_ptr<luxi_3d_navigation::LocalizationRecoveryController>
    localization_recovery_controller_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TerrainPathFollowerNode>());
  rclcpp::shutdown();
  return 0;
}
