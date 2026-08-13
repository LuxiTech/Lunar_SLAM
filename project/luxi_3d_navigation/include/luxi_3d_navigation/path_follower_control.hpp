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

namespace luxi_3d_navigation
{

struct PathFollowerCommand
{
  double linear_x{0.0};
  double angular_z{0.0};
};

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

inline PathFollowerCommand pathFollowerCommand(
  const double distance, const double heading_error, const double linear_gain,
  const double angular_gain, const double maximum_linear_speed,
  const double maximum_angular_speed, const double angular_deadband,
  const double linear_heading_tolerance)
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
  }
  return command;
}

}  // namespace luxi_3d_navigation
