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

#include <gtest/gtest.h>

#include "geometry_msgs/msg/twist.hpp"
#include "luxi_3d_navigation/twist_direction_adapter.hpp"

namespace
{

TEST(TwistDirectionAdapter, InvertsOnlyYawForLeKiwi)
{
  geometry_msgs::msg::Twist left;
  left.linear.x = 0.12;
  left.linear.y = -0.03;
  left.angular.x = 0.01;
  left.angular.y = -0.02;
  left.angular.z = 0.40;

  const auto adapted_left = luxi_3d_navigation::adaptTwist(left, true);
  EXPECT_DOUBLE_EQ(adapted_left.linear.x, 0.12);
  EXPECT_DOUBLE_EQ(adapted_left.linear.y, -0.03);
  EXPECT_DOUBLE_EQ(adapted_left.angular.x, 0.01);
  EXPECT_DOUBLE_EQ(adapted_left.angular.y, -0.02);
  EXPECT_DOUBLE_EQ(adapted_left.angular.z, -0.40);

  geometry_msgs::msg::Twist right;
  right.angular.z = -0.35;
  const auto adapted_right = luxi_3d_navigation::adaptTwist(right, true);
  EXPECT_DOUBLE_EQ(adapted_right.angular.z, 0.35);
}

TEST(TwistDirectionAdapter, PreservesStandardRosDirectionWhenDisabled)
{
  geometry_msgs::msg::Twist command;
  command.angular.z = 0.25;

  const auto adapted = luxi_3d_navigation::adaptTwist(command, false);
  EXPECT_DOUBLE_EQ(adapted.angular.z, 0.25);
}

}  // namespace
