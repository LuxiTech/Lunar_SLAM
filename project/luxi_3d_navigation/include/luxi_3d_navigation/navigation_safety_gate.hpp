#pragma once

#include <algorithm>
#include <cmath>
#include <string>

#include "geometry_msgs/msg/twist.hpp"

namespace luxi_3d_navigation
{

struct SafetyGateParameters
{
  double command_timeout{0.20};
  double obstacle_timeout{0.35};
  double planner_timeout{1.0};
  double slow_scale{0.50};
  double minimum_linear_speed{0.0};
  double healthy_resume_hold{0.50};
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

  void updatePlannerState(const std::string & state, double now_seconds)
  {
    planner_state_ = state;
    planner_time_ = now_seconds;
    have_planner_state_ = true;
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
    const bool recognized_obstacle_state = obstacle_state_ == "clear" ||
      obstacle_state_ == "slow" || obstacle_state_ == "blocked";
    const bool obstacle_hard_fault = obstacle_stale || !recognized_obstacle_state;
    if (obstacle_hard_fault) {
      hard_stop_latched_ = true;
    }

    if (monitor_only) {
      result.command = command_stale ? geometry_msgs::msg::Twist() : command_;
      result.state = obstacle_hard_fault ? "monitor_sensor_fault" : "monitor_only";
      result.limited = command_stale;
      return result;
    }
    if (hard_stop_latched_) {
      result.state = "hard_stop";
      result.emergency_stop = true;
      return result;
    }
    if (command_stale) {
      result.state = "command_stale";
      return result;
    }
    if (planner_stale) {
      result.state = "planner_stale";
      return result;
    }
    const bool planner_ready = planner_state_ == "clear" || planner_state_ == "ready";
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
  bool have_planner_state_{false};
  bool hard_stop_latched_{false};
  double command_time_{};
  double obstacle_time_{};
  double planner_time_{};
  double last_active_change_{};
  double healthy_since_{-1.0};
  std::string obstacle_state_{"stale"};
  std::string planner_state_{"waiting"};
  geometry_msgs::msg::Twist command_;
};

}  // namespace luxi_3d_navigation
