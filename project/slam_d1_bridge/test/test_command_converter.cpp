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

#include <limits>

#include "gtest/gtest.h"

#include "slam_d1_bridge/command_converter.hpp"

namespace slam_d1_bridge
{

TEST(CommandConverter, MapsAndClampsSupportedAxes)
{
    CommandConverter converter(CommandLimits{0.5, 0.4});
    geometry_msgs::msg::Twist twist;
    twist.linear.x = 1.0;
    twist.linear.y = 0.3;
    twist.linear.z = 0.2;
    twist.angular.x = 0.1;
    twist.angular.y = 0.2;
    twist.angular.z = -1.0;

    const auto command = converter.convert(twist, "loco", 0.03);

    EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.5);
    EXPECT_DOUBLE_EQ(command.twist.linear.y, 0.0);
    EXPECT_DOUBLE_EQ(command.twist.linear.z, 0.03);
    EXPECT_DOUBLE_EQ(command.twist.angular.x, 0.0);
    EXPECT_DOUBLE_EQ(command.twist.angular.y, 0.0);
    EXPECT_DOUBLE_EQ(command.twist.angular.z, -0.4);
    EXPECT_EQ(command.fsm_mode, "loco");
    EXPECT_DOUBLE_EQ(command.pose.orientation.w, 1.0);
    EXPECT_DOUBLE_EQ(command.pose.position.z, 0.0);
}

TEST(CommandConverter, RejectsNonFiniteInput)
{
    CommandConverter converter(CommandLimits{0.5, 0.5});
    geometry_msgs::msg::Twist twist;
    twist.linear.x = std::numeric_limits<double>::quiet_NaN();

    EXPECT_FALSE(converter.is_valid(twist));
}

TEST(CommandConverter, StopHasSafeDefaults)
{
    const auto command = CommandConverter::make_stop("", -0.12);

    EXPECT_DOUBLE_EQ(command.twist.linear.x, 0.0);
    EXPECT_DOUBLE_EQ(command.twist.linear.y, 0.0);
    EXPECT_DOUBLE_EQ(command.twist.linear.z, -0.12);
    EXPECT_DOUBLE_EQ(command.twist.angular.z, 0.0);
    EXPECT_DOUBLE_EQ(command.pose.orientation.w, 1.0);
    EXPECT_DOUBLE_EQ(command.pose.position.z, 0.0);
}

}  // namespace slam_d1_bridge
