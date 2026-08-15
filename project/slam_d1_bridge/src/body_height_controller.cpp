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

#include "slam_d1_bridge/body_height_controller.hpp"

namespace slam_d1_bridge
{

BodyHeightController::BodyHeightController(BodyHeightLimits limits)
  : limits_(std::move(limits)),
    current_(limits_.default_value),
    target_(limits_.default_value)
{
    if (!std::isfinite(limits_.minimum) || !std::isfinite(limits_.maximum) ||
      limits_.minimum >= limits_.maximum ||
      !std::isfinite(limits_.default_value) ||
      limits_.default_value < limits_.minimum || limits_.default_value > limits_.maximum ||
      !std::isfinite(limits_.maximum_rate) || limits_.maximum_rate <= 0.0)
    {
        throw std::invalid_argument("invalid body-height limits");
    }
}

double BodyHeightController::set_target(double requested_height)
{
    if (!std::isfinite(requested_height)) {
        throw std::invalid_argument("body height must be finite");
    }
    target_ = std::clamp(requested_height, limits_.minimum, limits_.maximum);
    return target_;
}

double BodyHeightController::step(double elapsed_seconds)
{
    if (!std::isfinite(elapsed_seconds) || elapsed_seconds < 0.0) {
        throw std::invalid_argument("elapsed time must be finite and non-negative");
    }
    const double maximum_change = limits_.maximum_rate * elapsed_seconds;
    current_ += std::clamp(target_ - current_, -maximum_change, maximum_change);
    return current_;
}

double BodyHeightController::current() const noexcept
{
    return current_;
}

double BodyHeightController::target() const noexcept
{
    return target_;
}

}  // namespace slam_d1_bridge
