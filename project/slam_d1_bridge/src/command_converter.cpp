// Copyright 2026 ROS 2 Developer
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

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "slam_d1_bridge/command_converter.hpp"

namespace slam_d1_bridge
{

CommandConverter::CommandConverter(CommandLimits limits)
  : limits_(std::move(limits))
{
    if (!std::isfinite(limits_.max_linear_x) || limits_.max_linear_x < 0.0 ||
      !std::isfinite(limits_.max_angular_z) || limits_.max_angular_z < 0.0)
    {
        throw std::invalid_argument("command limits must be finite and non-negative");
    }
}

bool CommandConverter::is_valid(const geometry_msgs::msg::Twist & twist) const
{
    return std::isfinite(twist.linear.x) && std::isfinite(twist.linear.y) &&
           std::isfinite(twist.linear.z) && std::isfinite(twist.angular.x) &&
           std::isfinite(twist.angular.y) && std::isfinite(twist.angular.z);
}

ddt_msgs::msg::UserCommand CommandConverter::convert(
    const geometry_msgs::msg::Twist & twist,
    const std::string & fsm_mode) const
{
    auto command = make_stop(fsm_mode);
    command.twist.linear.x = std::clamp(
        twist.linear.x, -limits_.max_linear_x, limits_.max_linear_x);
    command.twist.angular.z = std::clamp(
        twist.angular.z, -limits_.max_angular_z, limits_.max_angular_z);
    return command;
}

ddt_msgs::msg::UserCommand CommandConverter::make_stop(const std::string & fsm_mode)
{
    ddt_msgs::msg::UserCommand command;
    command.header.frame_id = "cmd";
    command.fsm_mode = fsm_mode;
    command.pose.orientation.w = 1.0;
    return command;
}

}  // namespace slam_d1_bridge
