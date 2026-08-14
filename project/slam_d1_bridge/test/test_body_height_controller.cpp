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
#include <stdexcept>

#include "gtest/gtest.h"

#include "slam_d1_bridge/body_height_controller.hpp"

namespace slam_d1_bridge
{

TEST(BodyHeightController, ClampsTargetsAndLimitsChangeRate)
{
    BodyHeightController controller(BodyHeightLimits{0.0, 9.0, 0.0, 1.0});

    EXPECT_DOUBLE_EQ(controller.set_target(50.0), 9.0);
    EXPECT_NEAR(controller.step(0.10), 0.10, 1.0e-12);
    EXPECT_NEAR(controller.step(8.90), 9.0, 1.0e-12);
    EXPECT_DOUBLE_EQ(controller.set_target(-0.50), 0.0);
    EXPECT_NEAR(controller.step(1.0), 8.0, 1.0e-12);
}

TEST(BodyHeightController, RejectsInvalidInput)
{
    BodyHeightController controller(BodyHeightLimits{});
    EXPECT_THROW(
        controller.set_target(std::numeric_limits<double>::quiet_NaN()),
        std::invalid_argument);
    EXPECT_THROW(controller.step(-0.1), std::invalid_argument);
    EXPECT_THROW(
        BodyHeightController(BodyHeightLimits{9.0, 0.0, 0.0, 1.0}),
        std::invalid_argument);
}

}  // namespace slam_d1_bridge
