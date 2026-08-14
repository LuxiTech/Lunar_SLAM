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

namespace slam_d1_bridge
{

struct BodyHeightLimits
{
    double minimum{0.0};
    double maximum{9.0};
    double default_value{0.0};
    double maximum_rate{1.0};
};

class BodyHeightController final
{
public:
    explicit BodyHeightController(BodyHeightLimits limits);

    double set_target(double requested_height);
    double step(double elapsed_seconds);

    [[nodiscard]] double current() const noexcept;
    [[nodiscard]] double target() const noexcept;

private:
    BodyHeightLimits limits_;
    double current_;
    double target_;
};

}  // namespace slam_d1_bridge
