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
#include "luxi_3d_navigation/velocity_command_mux.hpp"

namespace
{

geometry_msgs::msg::Twist command(double linear_x, double angular_z)
{
  geometry_msgs::msg::Twist result;
  result.linear.x = linear_x;
  result.angular.z = angular_z;
  return result;
}

TEST(VelocityCommandMux, NavigationOwnsOutputOnlyWhileActiveAndFresh)
{
  luxi_3d_navigation::VelocityCommandMux mux(0.6, 0.3);
  EXPECT_FALSE(mux.updateManual(command(0.05, 0.1), 1.0));
  EXPECT_DOUBLE_EQ(mux.output(1.1).linear.x, 0.05);

  mux.setNavigationActive(true, 2.0);
  mux.updateNavigation(command(0.10, 0.25), 2.05);
  const auto navigation = mux.output(2.10);
  EXPECT_DOUBLE_EQ(navigation.linear.x, 0.10);
  EXPECT_DOUBLE_EQ(navigation.angular.z, 0.25);
  EXPECT_FALSE(mux.navigationTimedOut(2.30));
  EXPECT_TRUE(mux.navigationTimedOut(2.36));
  EXPECT_DOUBLE_EQ(mux.output(2.36).linear.x, 0.0);
}

TEST(VelocityCommandMux, NonzeroManualCommandCancelsNavigation)
{
  luxi_3d_navigation::VelocityCommandMux mux(0.6, 0.3);
  mux.setNavigationActive(true, 1.0);
  mux.updateNavigation(command(0.10, 0.20), 1.02);

  EXPECT_FALSE(mux.updateManual(command(0.0, 0.0), 1.03));
  EXPECT_TRUE(mux.navigationActive());
  EXPECT_TRUE(mux.updateManual(command(0.04, -0.10), 1.04));
  EXPECT_FALSE(mux.navigationActive());
  EXPECT_DOUBLE_EQ(mux.output(1.05).angular.z, -0.10);
}

TEST(VelocityCommandMux, EmergencyStopLatchesZeroAndCancelsNavigation)
{
  luxi_3d_navigation::VelocityCommandMux mux(0.6, 0.3);
  mux.setNavigationActive(true, 1.0);
  mux.updateNavigation(command(0.10, 0.20), 1.02);

  mux.setEmergencyStop(true);
  EXPECT_FALSE(mux.navigationActive());
  EXPECT_DOUBLE_EQ(mux.output(1.03).linear.x, 0.0);
  EXPECT_DOUBLE_EQ(mux.output(1.03).angular.z, 0.0);
}

}  // namespace
