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
#include "nav_msgs/msg/odometry.hpp"
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
    declare_parameter<std::string>("odometry_topic", "/navigation/odom");
    declare_parameter<std::string>(
      "obstacle_state_topic", "/navigation/local_obstacles/state");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<double>("control_rate", 15.0);
    declare_parameter<double>("lookahead_m", 0.25);
    declare_parameter<double>("goal_tolerance_m", 0.12);
    declare_parameter<double>("goal_yaw_tolerance", 0.15);
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
    declare_parameter<std::string>(
      "replan_request_topic", "/navigation/replan_request");
    declare_parameter<double>("dead_reckoning_duration", 0.80);
    declare_parameter<double>("localization_recovery_timeout", 45.0);
    declare_parameter<double>("localization_recovery_angular_speed", 0.10);
    declare_parameter<double>("localization_recovery_confirmation_time", 1.0);
    declare_parameter<double>("replan_after_relocalization_timeout", 15.0);
    declare_parameter<double>("max_path_deviation_m", 0.50);
    declare_parameter<double>("traction_progress_timeout", 2.5);
    declare_parameter<double>("traction_progress_distance", 0.04);
    declare_parameter<double>("traction_boost_speed", 0.15);
    declare_parameter<double>("traction_boost_timeout", 2.0);
    declare_parameter<double>("traction_observation_timeout", 0.80);
    declare_parameter<double>("obstacle_recovery_angular_speed", 0.20);
    declare_parameter<double>("obstacle_recovery_timeout", 10.0);

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
    luxi_3d_navigation::TractionBoostParameters traction_parameters;
    traction_parameters.progress_timeout =
      get_parameter("traction_progress_timeout").as_double();
    traction_parameters.progress_distance =
      get_parameter("traction_progress_distance").as_double();
    traction_parameters.boost_timeout =
      get_parameter("traction_boost_timeout").as_double();
    traction_boost_controller_ =
      std::make_unique<luxi_3d_navigation::TractionBoostController>(traction_parameters);

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
    replan_request_pub_ = create_publisher<std_msgs::msg::Bool>(
      get_parameter("replan_request_topic").as_string(), 10);
    terrain_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      get_parameter("terrain_pose_topic").as_string(), 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
        terrain_pose_ = *message;
      });
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("odometry_topic").as_string(), 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {
        odometry_ = *message;
        odometry_received_at_ = steadyNow();
      });
    obstacle_state_sub_ = create_subscription<std_msgs::msg::String>(
      get_parameter("obstacle_state_topic").as_string(),
      rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr message) {
        obstacle_state_ = message->data;
        obstacle_state_received_at_ = steadyNow();
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
          if (replan_pending_ && active_) {
            publishStop();
            updateActiveState("replanning_after_relocalization");
          } else {
            setState(false, "waiting_path");
          }
        } else if (active_) {
          const bool recovered_replan = replan_pending_;
          replan_pending_ = false;
          replan_requested_at_ = -1.0;
          updateActiveState("active");
          RCLCPP_INFO(
            get_logger(),
            recovered_replan ?
            "Relocalization replan ready with %zu poses; resuming navigation" :
            "Updated active terrain path with %zu poses",
            path_.size());
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
          traction_boost_controller_->reset();
          traction_boost_active_ = false;
          obstacle_recovery_started_at_ = -1.0;
          last_path_turn_direction_ = 1.0;
          replan_pending_ = false;
          replan_requested_at_ = -1.0;
          forced_relocalization_ = false;
          forced_relocalization_observed_fault_ = false;
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
          traction_boost_controller_->reset();
          traction_boost_active_ = false;
          obstacle_recovery_started_at_ = -1.0;
          replan_pending_ = false;
          replan_requested_at_ = -1.0;
          forced_relocalization_ = false;
          forced_relocalization_observed_fault_ = false;
          path_.clear();
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
    if (!active_) {
      return;
    }
    const double now_seconds = steadyNow();
    if (replan_pending_) {
      publishRecoveryActive(false);
      publishStop();
      if (replan_requested_at_ >= 0.0 &&
        now_seconds - replan_requested_at_ >=
        get_parameter("replan_after_relocalization_timeout").as_double())
      {
        RCLCPP_ERROR(
          get_logger(), "No new path arrived after relocalization; navigation stopped");
        replan_pending_ = false;
        setState(false, "replan_after_relocalization_timeout");
      }
      return;
    }
    if (path_.empty()) {
      return;
    }
    const double health_timeout = get_parameter("localization_timeout").as_double();
    const bool health_fresh = localization_health_received_at_ >= 0.0 &&
      now_seconds - localization_health_received_at_ <= health_timeout;
    const std::string observed_health = health_fresh ? localization_health_ : "searching";
    if (forced_relocalization_ && observed_health != "tracking") {
      forced_relocalization_observed_fault_ = true;
    }
    const std::string health = forced_relocalization_ &&
      !forced_relocalization_observed_fault_ ? "searching" : observed_health;
    const auto recovery_action = localization_recovery_controller_->update(
      health, now_seconds);
    if (recovery_action == luxi_3d_navigation::LocalizationRecoveryAction::kStop) {
      traction_boost_controller_->reset();
      traction_boost_active_ = false;
      setState(false, "localization_lost");
      RCLCPP_ERROR(
        get_logger(), "Localization recovery reached its bounded timeout; navigation stopped");
      return;
    }
    if (recovery_action == luxi_3d_navigation::LocalizationRecoveryAction::kRotate) {
      traction_boost_controller_->reset();
      traction_boost_active_ = false;
      publishRecoveryRotation();
      return;
    }
    if (recovery_action == luxi_3d_navigation::LocalizationRecoveryAction::kHold) {
      traction_boost_controller_->reset();
      traction_boost_active_ = false;
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
    } else if (state_.rfind("localization_recovery_", 0U) == 0U ||
      state_ == "localization_dead_reckoning")
    {
      requestReplanAfterRelocalization();
      return;
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
        tf2::getYaw(transform.transform.rotation),
        recovery_action == luxi_3d_navigation::LocalizationRecoveryAction::kTrack);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Localization unavailable: %s", error.what());
      handleLocalizationFault();
    }
  }

  void follow(
    double robot_x, double robot_y, double robot_z, double robot_yaw,
    const bool localization_tracking)
  {
    const auto & goal_pose = path_.back().pose;
    const auto & goal = goal_pose.position;
    const auto & previous = path_.size() > 1U ?
      path_[path_.size() - 2U].pose.position : goal;
    if (luxi_3d_navigation::pathGoalReached3D(
        robot_x, robot_y, robot_z, previous.x, previous.y,
        goal.x, goal.y, goal.z,
        get_parameter("goal_tolerance_m").as_double(),
        get_parameter("minimum_safe_goal_tolerance_m").as_double()))
    {
      traction_boost_controller_->reset();
      traction_boost_active_ = false;
      const double goal_heading_error = luxi_3d_navigation::normalizedAngle(
        tf2::getYaw(goal_pose.orientation) - robot_yaw);
      if (std::abs(goal_heading_error) > get_parameter("goal_yaw_tolerance").as_double()) {
        last_path_turn_direction_ = luxi_3d_navigation::updatedTurnDirection(
          goal_heading_error, last_path_turn_direction_);
        geometry_msgs::msg::Twist command;
        command.angular.z = std::clamp(
          goal_heading_error * get_parameter("angular_gain").as_double(),
          -get_parameter("max_angular_speed").as_double(),
          get_parameter("max_angular_speed").as_double());
        cmd_pub_->publish(command);
        updateActiveState("aligning_goal_heading");
        return;
      }
      setState(false, "goal_reached");
      RCLCPP_INFO(get_logger(), "3D terrain navigation goal position and heading reached");
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
        get_logger(),
        "Robot is %.3fm away from the planned path; requesting relocalization",
        nearest_distance);
      beginForcedRelocalization();
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

    const double observation_timeout =
      get_parameter("traction_observation_timeout").as_double();
    const double now_seconds = steadyNow();
    const bool odometry_fresh = odometry_.has_value() && odometry_received_at_ >= 0.0 &&
      now_seconds - odometry_received_at_ <= observation_timeout;
    const bool obstacle_clear = obstacle_state_ == "clear" &&
      obstacle_state_received_at_ >= 0.0 &&
      now_seconds - obstacle_state_received_at_ <= observation_timeout;
    const bool obstacle_blocked = obstacle_state_ == "blocked" &&
      obstacle_state_received_at_ >= 0.0 &&
      now_seconds - obstacle_state_received_at_ <= observation_timeout;
    if (localization_tracking && obstacle_blocked) {
      traction_boost_controller_->reset();
      traction_boost_active_ = false;
      if (obstacle_recovery_started_at_ < 0.0) {
        obstacle_recovery_started_at_ = now_seconds;
        RCLCPP_WARN(
          get_logger(),
          "Forward corridor blocked; requesting a bounded in-place turn toward %s",
          last_path_turn_direction_ > 0.0 ? "left" : "right");
      }
      if (now_seconds - obstacle_recovery_started_at_ >=
        get_parameter("obstacle_recovery_timeout").as_double())
      {
        RCLCPP_ERROR(
          get_logger(), "Obstacle did not clear during in-place recovery; navigation stopped");
        setState(false, "obstacle_recovery_timeout");
        return;
      }
      command.linear.x = 0.0;
      command.linear.y = 0.0;
      command.angular.z = std::clamp(
        get_parameter("obstacle_recovery_angular_speed").as_double(), 0.0,
        get_parameter("max_angular_speed").as_double()) * last_path_turn_direction_;
      updateActiveState("obstacle_recovery_spin");
      cmd_pub_->publish(command);
      return;
    }
    if (obstacle_recovery_started_at_ >= 0.0) {
      obstacle_recovery_started_at_ = -1.0;
      if (state_ == "obstacle_recovery_spin") {
        updateActiveState("active");
        RCLCPP_INFO(get_logger(), "Forward corridor cleared; resuming the updated path");
      }
    }
    const bool boost_eligible = localization_tracking && odometry_fresh && obstacle_clear &&
      command.linear.x > 0.0 && std::abs(command.angular.z) < 0.05;
    const auto boost_action = traction_boost_controller_->update(
      boost_eligible,
      odometry_fresh ? odometry_->pose.pose.position.x : 0.0,
      odometry_fresh ? odometry_->pose.pose.position.y : 0.0,
      now_seconds);
    if (boost_action == luxi_3d_navigation::TractionBoostAction::kFailed) {
      traction_boost_active_ = false;
      RCLCPP_ERROR(
        get_logger(),
        "Robot made no odometry progress during the bounded traction boost; navigation stopped");
      setState(false, "stuck_no_progress");
      return;
    }
    if (boost_action == luxi_3d_navigation::TractionBoostAction::kBoost) {
      const double boost_speed = std::clamp(
        get_parameter("traction_boost_speed").as_double(),
        get_parameter("max_linear_speed").as_double(), 0.20);
      if (!traction_boost_active_) {
        RCLCPP_WARN(
          get_logger(),
          "No obstacle and no odometry progress; temporarily boosting %.3f -> %.3f m/s",
          command.linear.x, boost_speed);
      }
      traction_boost_active_ = true;
      command.linear.x = std::max(command.linear.x, boost_speed);
      updateActiveState("traction_boost");
    } else if (traction_boost_active_) {
      if (boost_eligible) {
        RCLCPP_INFO(get_logger(), "Odometry progress recovered; returning to normal speed");
      }
      traction_boost_active_ = false;
      updateActiveState("active");
    }
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

  void beginForcedRelocalization()
  {
    if (!forced_relocalization_) {
      forced_relocalization_ = true;
      forced_relocalization_observed_fault_ = false;
      localization_recovery_controller_->reset();
      std_msgs::msg::Bool request;
      request.data = true;
      relocalization_request_pub_->publish(request);
    }
    publishRecoveryActive(false);
    publishStop();
    updateActiveState("localization_recovery_waiting");
  }

  void requestReplanAfterRelocalization()
  {
    forced_relocalization_ = false;
    forced_relocalization_observed_fault_ = false;
    replan_pending_ = true;
    replan_requested_at_ = steadyNow();
    path_.clear();
    publishRecoveryActive(false);
    publishStop();
    updateActiveState("replanning_after_relocalization");
    std_msgs::msg::Bool request;
    request.data = true;
    replan_request_pub_->publish(request);
    RCLCPP_INFO(
      get_logger(),
      "Localization recovered; requesting a fresh path from the corrected pose to the original goal");
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
      traction_boost_controller_->reset();
      traction_boost_active_ = false;
      obstacle_recovery_started_at_ = -1.0;
      replan_pending_ = false;
      replan_requested_at_ = -1.0;
      forced_relocalization_ = false;
      forced_relocalization_observed_fault_ = false;
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
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr terrain_pose_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr localization_health_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr obstacle_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr recovery_active_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr relocalization_request_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr replan_request_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::optional<geometry_msgs::msg::PoseStamped> terrain_pose_;
  std::optional<nav_msgs::msg::Odometry> odometry_;
  double odometry_received_at_{-1.0};
  std::string obstacle_state_{"unknown"};
  double obstacle_state_received_at_{-1.0};
  std::string localization_health_{"searching"};
  double localization_health_received_at_{-1.0};
  double last_path_turn_direction_{1.0};
  double recovery_turn_direction_{1.0};
  bool traction_boost_active_{false};
  double obstacle_recovery_started_at_{-1.0};
  bool replan_pending_{false};
  double replan_requested_at_{-1.0};
  bool forced_relocalization_{false};
  bool forced_relocalization_observed_fault_{false};
  std::unique_ptr<luxi_3d_navigation::LocalizationRecoveryController>
    localization_recovery_controller_;
  std::unique_ptr<luxi_3d_navigation::TractionBoostController>
    traction_boost_controller_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TerrainPathFollowerNode>());
  rclcpp::shutdown();
  return 0;
}
