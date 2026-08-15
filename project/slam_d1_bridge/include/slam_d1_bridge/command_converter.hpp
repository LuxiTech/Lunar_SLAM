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

#pragma once

#include <string>

#include "ddt_msgs/msg/user_command.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace slam_d1_bridge
{

struct CommandLimits
{
    double max_linear_x{0.5};
    double max_angular_z{0.5};
};

class CommandConverter final
{
public:
    explicit CommandConverter(CommandLimits limits);

    [[nodiscard]] bool is_valid(const geometry_msgs::msg::Twist & twist) const;

    [[nodiscard]] ddt_msgs::msg::UserCommand convert(
        const geometry_msgs::msg::Twist & twist,
        const std::string & fsm_mode,
        double body_height_rate = 0.0) const;

    [[nodiscard]] static ddt_msgs::msg::UserCommand make_stop(
        const std::string & fsm_mode,
        double body_height_rate = 0.0);

private:
    CommandLimits limits_;
};

}  // namespace slam_d1_bridge
