// Copyright 2026 lunar
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <algorithm>
#include <cmath>
#include <optional>
#include <stdexcept>

namespace luxi_3d_navigation
{

struct PathFollowerCommand
{
  double linear_x{0.0};
  double angular_z{0.0};
};

enum class TractionBoostAction
{
  kNormal,
  kBoost,
  kFailed,
};

struct TractionBoostParameters
{
  double progress_timeout{2.5};
  double progress_distance{0.04};
  double boost_timeout{2.0};
};

// Detects a commanded-but-stationary robot from local odometry.  The caller
// remains responsible for allowing a boost only while localization and the
// near-field obstacle layer are healthy.
class TractionBoostController
{
public:
  explicit TractionBoostController(TractionBoostParameters parameters = {})
  : parameters_(parameters)
  {
    if (parameters_.progress_timeout <= 0.0 || parameters_.progress_distance <= 0.0 ||
      parameters_.boost_timeout <= 0.0)
    {
      throw std::invalid_argument("traction boost parameters are invalid");
    }
  }

  TractionBoostAction update(
    const bool eligible, const double x, const double y, const double now_seconds)
  {
    if (!eligible || !std::isfinite(x) || !std::isfinite(y) ||
      !std::isfinite(now_seconds))
    {
      reset();
      return TractionBoostAction::kNormal;
    }
    if (!anchor_time_.has_value()) {
      anchor_x_ = x;
      anchor_y_ = y;
      anchor_time_ = now_seconds;
      return TractionBoostAction::kNormal;
    }
    if (std::hypot(x - anchor_x_, y - anchor_y_) >= parameters_.progress_distance) {
      reset();
      anchor_x_ = x;
      anchor_y_ = y;
      anchor_time_ = now_seconds;
      return TractionBoostAction::kNormal;
    }
    if (boost_started_at_.has_value()) {
      if (now_seconds - *boost_started_at_ >= parameters_.boost_timeout) {
        reset();
        return TractionBoostAction::kFailed;
      }
      return TractionBoostAction::kBoost;
    }
    if (now_seconds - *anchor_time_ >= parameters_.progress_timeout) {
      boost_started_at_ = now_seconds;
      return TractionBoostAction::kBoost;
    }
    return TractionBoostAction::kNormal;
  }

  void reset()
  {
    anchor_time_.reset();
    boost_started_at_.reset();
  }

private:
  TractionBoostParameters parameters_;
  double anchor_x_{0.0};
  double anchor_y_{0.0};
  std::optional<double> anchor_time_;
  std::optional<double> boost_started_at_;
};

inline double normalizedAngle(const double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

inline double updatedTurnDirection(
  const double heading_error, const double previous_direction,
  const double minimum_heading = 1e-3)
{
  if (std::isfinite(heading_error) && std::abs(heading_error) > minimum_heading) {
    return std::copysign(1.0, heading_error);
  }
  if (std::isfinite(previous_direction) && std::abs(previous_direction) > 0.0) {
    return std::copysign(1.0, previous_direction);
  }
  return 1.0;
}

inline bool pathGoalReached(
  const double robot_x, const double robot_y,
  const double previous_x, const double previous_y,
  const double goal_x, const double goal_y,
  const double configured_tolerance, const double minimum_safe_tolerance)
{
  const double tolerance = std::max(configured_tolerance, minimum_safe_tolerance);
  if (std::hypot(goal_x - robot_x, goal_y - robot_y) <= tolerance) {
    return true;
  }
  const double segment_x = goal_x - previous_x;
  const double segment_y = goal_y - previous_y;
  if (segment_x * segment_x + segment_y * segment_y <= 1e-12) {
    return false;
  }
  return (robot_x - goal_x) * segment_x + (robot_y - goal_y) * segment_y >= 0.0;
}

inline bool pathGoalReached3D(
  const double robot_x, const double robot_y, const double robot_z,
  const double previous_x, const double previous_y,
  const double goal_x, const double goal_y, const double goal_z,
  const double configured_tolerance, const double minimum_safe_tolerance)
{
  const double tolerance = std::max(configured_tolerance, minimum_safe_tolerance);
  return std::abs(goal_z - robot_z) <= tolerance && pathGoalReached(
    robot_x, robot_y, previous_x, previous_y, goal_x, goal_y,
    configured_tolerance, minimum_safe_tolerance);
}

inline PathFollowerCommand pathFollowerCommand(
  const double distance, const double heading_error, const double linear_gain,
  const double angular_gain, const double maximum_linear_speed,
  const double maximum_angular_speed, const double angular_deadband,
  const double linear_heading_tolerance, const double minimum_linear_speed = 0.0)
{
  PathFollowerCommand command;
  const double absolute_heading = std::abs(heading_error);
  if (absolute_heading > angular_deadband) {
    command.angular_z = std::clamp(
      heading_error * angular_gain, -maximum_angular_speed, maximum_angular_speed);
  }
  if (absolute_heading <= linear_heading_tolerance) {
    command.linear_x = std::clamp(
      distance * linear_gain, -maximum_linear_speed, maximum_linear_speed);
    const double effective_minimum = std::clamp(
      minimum_linear_speed, 0.0, maximum_linear_speed);
    if (std::abs(command.linear_x) > 0.0 &&
      std::abs(command.linear_x) < effective_minimum)
    {
      command.linear_x = std::copysign(effective_minimum, command.linear_x);
    }
  }
  return command;
}

}  // namespace luxi_3d_navigation
