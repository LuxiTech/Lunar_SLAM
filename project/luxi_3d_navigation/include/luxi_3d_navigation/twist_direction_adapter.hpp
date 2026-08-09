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

#ifndef LUXI_3D_NAVIGATION__TWIST_DIRECTION_ADAPTER_HPP_
#define LUXI_3D_NAVIGATION__TWIST_DIRECTION_ADAPTER_HPP_

#include "geometry_msgs/msg/twist.hpp"

namespace luxi_3d_navigation
{

inline geometry_msgs::msg::Twist adaptTwist(
  const geometry_msgs::msg::Twist & input, const bool invert_angular_z)
{
  auto output = input;
  if (invert_angular_z) {
    output.angular.z = -output.angular.z;
  }
  return output;
}

}  // namespace luxi_3d_navigation

#endif  // LUXI_3D_NAVIGATION__TWIST_DIRECTION_ADAPTER_HPP_
