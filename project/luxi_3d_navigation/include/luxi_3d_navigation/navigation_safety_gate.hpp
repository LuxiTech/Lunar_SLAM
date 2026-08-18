#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include "geometry_msgs/msg/twist.hpp"

namespace luxi_3d_navigation
{

struct SafetyGateParameters
{
  double command_timeout{0.20};
  double obstacle_timeout{0.35};
  // A missing/invalid obstacle layer always stops output immediately.  Only
  // latch that stop after the fault persists for this long, so a brief USB
  // camera dropout can recover without requiring navigation to be restarted.
  double obstacle_hard_stop_latch_delay{0.0};
  double planner_timeout{1.0};
  double localization_timeout{1.5};
  double slow_scale{0.50};
  double minimum_linear_speed{0.0};
  double healthy_resume_hold{0.50};
  double dead_reckoning_duration{0.80};
  double dead_reckoning_scale{0.50};
  double maximum_recovery_angular_speed{0.20};
  double minimum_obstacle_rotation_clearance{0.30};
  double maximum_obstacle_recovery_angular_speed{0.20};
};

struct SafetyGateResult
{
  geometry_msgs::msg::Twist command;
  std::string state{"inactive"};
  bool limited{true};
  bool emergency_stop{false};
};

class NavigationSafetyGate
{
public:
  explicit NavigationSafetyGate(SafetyGateParameters parameters = {})
  : parameters_(parameters)
  {
  }

  void setNavigationActive(bool active, double now_seconds)
  {
    if (!active) {
      hard_stop_latched_ = false;
      healthy_since_ = -1.0;
      recovery_active_ = false;
      localization_fault_started_at_ = -1.0;
      obstacle_fault_started_at_ = -1.0;
    } else if (!active_) {
      healthy_since_ = -1.0;
    }
    active_ = active;
    last_active_change_ = now_seconds;
  }

  void updateCommand(const geometry_msgs::msg::Twist & command, double now_seconds)
  {
    command_ = command;
    command_time_ = now_seconds;
    have_command_ = true;
  }

  void updateObstacleState(const std::string & state, double now_seconds)
  {
    obstacle_state_ = state;
    obstacle_time_ = now_seconds;
    have_obstacle_state_ = true;
  }

  void updateRotationClearance(double distance, double now_seconds)
  {
    rotation_clearance_ = distance;
    rotation_clearance_time_ = now_seconds;
    have_rotation_clearance_ = std::isfinite(distance);
  }

  void updatePlannerState(const std::string & state, double now_seconds)
  {
    planner_state_ = state;
    planner_time_ = now_seconds;
    have_planner_state_ = true;
  }

  void updateLocalizationState(const std::string & state, double now_seconds)
  {
    if (state == "tracking") {
      localization_fault_started_at_ = -1.0;
    } else if (localization_state_ == "tracking" || localization_fault_started_at_ < 0.0) {
      localization_fault_started_at_ = now_seconds;
    }
    localization_state_ = state;
    localization_time_ = now_seconds;
    have_localization_state_ = true;
  }

  void updateRecoveryActive(bool active, double now_seconds)
  {
    recovery_active_ = active;
    recovery_time_ = now_seconds;
  }

  SafetyGateResult evaluate(double now_seconds, bool monitor_only = false)
  {
    SafetyGateResult result;
    if (!active_) {
      result.state = "inactive";
      return result;
    }
    const bool command_stale = !have_command_ ||
      now_seconds - command_time_ > parameters_.command_timeout;
    const bool obstacle_stale = !have_obstacle_state_ ||
      now_seconds - obstacle_time_ > parameters_.obstacle_timeout;
    const bool planner_stale = !have_planner_state_ ||
      now_seconds - planner_time_ > parameters_.planner_timeout;
    const bool localization_stale = !have_localization_state_ ||
      now_seconds - localization_time_ > parameters_.localization_timeout;
    const bool recognized_obstacle_state = obstacle_state_ == "clear" ||
      obstacle_state_ == "slow" || obstacle_state_ == "blocked";
    const bool obstacle_hard_fault = obstacle_stale || !recognized_obstacle_state;
    if (obstacle_hard_fault) {
      if (obstacle_fault_started_at_ < 0.0) {
        obstacle_fault_started_at_ = now_seconds;
      }
      if (now_seconds - obstacle_fault_started_at_ >=
        std::max(0.0, parameters_.obstacle_hard_stop_latch_delay))
      {
        hard_stop_latched_ = true;
      }
    } else {
      obstacle_fault_started_at_ = -1.0;
    }

    if (monitor_only) {
      result.command = command_stale ? geometry_msgs::msg::Twist() : command_;
      result.state = obstacle_hard_fault ? "monitor_sensor_fault" :
        (localization_stale || localization_state_ != "tracking" ?
        "monitor_localization_fault" : "monitor_only");
      result.limited = command_stale || localization_stale || localization_state_ != "tracking";
      return result;
    }
    if (hard_stop_latched_) {
      result.state = "hard_stop";
      result.emergency_stop = true;
      return result;
    }
    if (obstacle_hard_fault) {
      healthy_since_ = -1.0;
      result.state = obstacle_stale ? "sensor_stale_stop" : "sensor_fault_stop";
      return result;
    }
    if (command_stale) {
      result.state = "command_stale";
      return result;
    }
    if (recovery_active_ &&
      now_seconds - recovery_time_ <= parameters_.command_timeout &&
      obstacle_state_ != "blocked")
    {
      // A stale localization heartbeat/TF is itself one reason the follower
      // requests recovery. Permit only the explicitly marked, angular-only
      // command; fresh obstacle sensing remains mandatory and a blocked
      // footprint still stops the robot.
      result.command.angular.z = std::clamp(
        command_.angular.z, -parameters_.maximum_recovery_angular_speed,
        parameters_.maximum_recovery_angular_speed);
      result.state = "localization_recovery_spin";
      return result;
    }
    const bool localization_fault = localization_stale || localization_state_ != "tracking";
    if (localization_fault) {
      healthy_since_ = -1.0;
      const bool dead_reckoning_state = localization_state_ == "degraded" ||
        localization_state_ == "dead_reckoning";
      const bool within_dead_reckoning_window = localization_fault_started_at_ >= 0.0 &&
        now_seconds - localization_fault_started_at_ <= parameters_.dead_reckoning_duration;
      const bool planner_ready_during_recovery = !planner_stale &&
        (planner_state_ == "clear" || planner_state_ == "ready");
      if (!localization_stale && dead_reckoning_state && within_dead_reckoning_window &&
        planner_ready_during_recovery && obstacle_state_ != "blocked")
      {
        const double scale = std::clamp(parameters_.dead_reckoning_scale, 0.0, 1.0);
        result.command = command_;
        const double original_x = result.command.linear.x;
        const double original_y = result.command.linear.y;
        result.command.linear.x *= scale;
        result.command.linear.y *= scale;
        result.command.angular.z *= scale;
        preserveMinimumLinearCommand(original_x, result.command.linear.x);
        preserveMinimumLinearCommand(original_y, result.command.linear.y);
        result.state = "localization_dead_reckoning";
        return result;
      }
      result.state = localization_stale ? "localization_stale" :
        "localization_" + localization_state_;
      return result;
    }
    if (planner_stale) {
      result.state = "planner_stale";
      return result;
    }
    const bool planner_ready = planner_state_ == "clear" || planner_state_ == "ready";
    const bool rotation_clearance_fresh = have_rotation_clearance_ &&
      now_seconds - rotation_clearance_time_ <= parameters_.obstacle_timeout;
    const bool angular_only_command =
      std::abs(command_.linear.x) <= 1.0e-6 &&
      std::abs(command_.linear.y) <= 1.0e-6 &&
      std::abs(command_.angular.z) > 1.0e-6;
    if (obstacle_state_ == "blocked" && planner_ready && rotation_clearance_fresh &&
      rotation_clearance_ >= parameters_.minimum_obstacle_rotation_clearance &&
      angular_only_command)
    {
      // The forward corridor is blocked, but every measured collision-height
      // point is outside the full rotation envelope. Permit a bounded in-place
      // turn only; translation remains prohibited and stale depth still fails closed.
      healthy_since_ = -1.0;
      result.command.angular.z = std::clamp(
        command_.angular.z,
        -parameters_.maximum_obstacle_recovery_angular_speed,
        parameters_.maximum_obstacle_recovery_angular_speed);
      result.state = "obstacle_recovery_spin";
      return result;
    }
    const bool blocked = obstacle_state_ == "blocked" || !planner_ready;
    if (blocked) {
      healthy_since_ = -1.0;
      result.state = planner_state_ == "no_path" ? "no_path" : "blocked";
      return result;
    }
    if (healthy_since_ < 0.0) {
      healthy_since_ = now_seconds;
    }
    if (now_seconds - healthy_since_ < parameters_.healthy_resume_hold) {
      result.state = "recovery_hold";
      return result;
    }

    result.command = command_;
    result.limited = false;
    if (obstacle_state_ == "slow") {
      const double original_x = result.command.linear.x;
      const double original_y = result.command.linear.y;
      result.command.linear.x *= std::clamp(parameters_.slow_scale, 0.0, 1.0);
      result.command.linear.y *= std::clamp(parameters_.slow_scale, 0.0, 1.0);
      preserveMinimumLinearCommand(original_x, result.command.linear.x);
      preserveMinimumLinearCommand(original_y, result.command.linear.y);
      result.state = "slow";
      result.limited = true;
    } else {
      result.state = "clear";
    }
    return result;
  }

private:
  void preserveMinimumLinearCommand(double original, double & scaled) const
  {
    const double minimum = std::max(0.0, parameters_.minimum_linear_speed);
    if (std::abs(original) > 0.0 && std::abs(scaled) < minimum) {
      scaled = std::copysign(std::min(std::abs(original), minimum), original);
    }
  }

  SafetyGateParameters parameters_;
  bool active_{false};
  bool have_command_{false};
  bool have_obstacle_state_{false};
  bool have_rotation_clearance_{false};
  bool have_planner_state_{false};
  bool have_localization_state_{false};
  bool hard_stop_latched_{false};
  double command_time_{};
  double obstacle_time_{};
  double rotation_clearance_time_{};
  double rotation_clearance_{std::numeric_limits<double>::infinity()};
  double planner_time_{};
  double localization_time_{};
  double localization_fault_started_at_{-1.0};
  double obstacle_fault_started_at_{-1.0};
  double recovery_time_{};
  double last_active_change_{};
  double healthy_since_{-1.0};
  std::string obstacle_state_{"stale"};
  std::string planner_state_{"waiting"};
  std::string localization_state_{"searching"};
  bool recovery_active_{false};
  geometry_msgs::msg::Twist command_;
};

}  // namespace luxi_3d_navigation
