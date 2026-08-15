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

#include "luxi_3d_navigation/path_follower_control.hpp"

TEST(PathFollowerControl, SmallHeadingErrorDrivesStraight)
{
  const auto command = luxi_3d_navigation::pathFollowerCommand(
    0.20, 0.10, 0.6, 1.2, 0.03, 0.10, 0.15, 0.35);
  EXPECT_DOUBLE_EQ(command.linear_x, 0.03);
  EXPECT_DOUBLE_EQ(command.angular_z, 0.0);
}

TEST(PathFollowerControl, ModerateHeadingErrorSteersWhileAdvancing)
{
  const auto command = luxi_3d_navigation::pathFollowerCommand(
    0.20, -0.20, 0.6, 1.2, 0.03, 0.10, 0.15, 0.35);
  EXPECT_DOUBLE_EQ(command.linear_x, 0.03);
  EXPECT_DOUBLE_EQ(command.angular_z, -0.10);
}

TEST(PathFollowerControl, LargeHeadingErrorRotatesBeforeAdvancing)
{
  const auto command = luxi_3d_navigation::pathFollowerCommand(
    0.20, 0.50, 0.6, 1.2, 0.03, 0.10, 0.15, 0.35);
  EXPECT_DOUBLE_EQ(command.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(command.angular_z, 0.10);
}

TEST(PathFollowerControl, PreservesD1MinimumEffectiveWalkingSpeed)
{
  const auto command = luxi_3d_navigation::pathFollowerCommand(
    0.13, 0.0, 0.6, 1.2, 0.10, 0.35, 0.15, 0.35, 0.10);
  EXPECT_DOUBLE_EQ(command.linear_x, 0.10);

  const auto rotate_only = luxi_3d_navigation::pathFollowerCommand(
    0.13, 0.50, 0.6, 1.2, 0.10, 0.35, 0.15, 0.35, 0.10);
  EXPECT_DOUBLE_EQ(rotate_only.linear_x, 0.0);
}

TEST(PathFollowerControl, D1SafetyToleranceOverridesUnsafeRuntimeValue)
{
  EXPECT_TRUE(luxi_3d_navigation::pathGoalReached(
    0.0, 0.384, 0.0, 0.45, 0.0, 0.50, 0.02, 0.12));
}

TEST(PathFollowerControl, StopsAfterCrossingGoalPlane)
{
  EXPECT_TRUE(luxi_3d_navigation::pathGoalReached(
    0.02, 0.56, 0.0, 0.45, 0.0, 0.50, 0.02, 0.12));
}

TEST(PathFollowerControl, KeepsDrivingBeforeSafetyTolerance)
{
  EXPECT_FALSE(luxi_3d_navigation::pathGoalReached(
    0.0, 0.27, 0.0, 0.45, 0.0, 0.50, 0.02, 0.12));
}

TEST(PathFollowerControl, SinglePointPathStillRequiresTolerance)
{
  EXPECT_FALSE(luxi_3d_navigation::pathGoalReached(
    0.0, 0.0, 0.0, 0.50, 0.0, 0.50, 0.02, 0.12));
}

TEST(PathFollowerControl, SameXYOnAnotherTerrainLevelIsNotTheGoal)
{
  EXPECT_FALSE(luxi_3d_navigation::pathGoalReached3D(
    1.0, 2.0, 0.0, 0.9, 2.0, 1.0, 2.0, 1.0, 0.12, 0.12));
  EXPECT_TRUE(luxi_3d_navigation::pathGoalReached3D(
    1.0, 2.0, 0.95, 0.9, 2.0, 1.0, 2.0, 1.0, 0.12, 0.12));
}
