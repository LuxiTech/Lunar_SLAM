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

#ifndef LUXI_3D_NAVIGATION__VELOCITY_COMMAND_MUX_HPP_
#define LUXI_3D_NAVIGATION__VELOCITY_COMMAND_MUX_HPP_

#include <cmath>

#include "geometry_msgs/msg/twist.hpp"

namespace luxi_3d_navigation
{

class VelocityCommandMux
{
public:
  VelocityCommandMux(double manual_timeout, double navigation_timeout)
  : manual_timeout_(manual_timeout), navigation_timeout_(navigation_timeout)
  {
  }

  bool updateManual(const geometry_msgs::msg::Twist & command, double now_seconds)
  {
    manual_command_ = command;
    manual_received_at_ = now_seconds;
    manual_valid_ = true;
    if (navigation_active_ && isMoving(command)) {
      navigation_active_ = false;
      navigation_valid_ = false;
      return true;
    }
    return false;
  }

  void updateNavigation(const geometry_msgs::msg::Twist & command, double now_seconds)
  {
    navigation_command_ = command;
    navigation_received_at_ = now_seconds;
    navigation_valid_ = true;
  }

  void setNavigationActive(bool active, double now_seconds)
  {
    if (active && !navigation_active_) {
      navigation_valid_ = false;
      navigation_activated_at_ = now_seconds;
    }
    navigation_active_ = active && !emergency_stop_;
  }

  void setEmergencyStop(bool active)
  {
    emergency_stop_ = active;
    if (active) {
      navigation_active_ = false;
      navigation_valid_ = false;
    }
  }

  bool navigationActive() const
  {
    return navigation_active_;
  }

  bool navigationTimedOut(double now_seconds) const
  {
    if (!navigation_active_) {
      return false;
    }
    const double reference = navigation_valid_ ?
      navigation_received_at_ : navigation_activated_at_;
    return now_seconds - reference > navigation_timeout_;
  }

  geometry_msgs::msg::Twist output(double now_seconds) const
  {
    if (emergency_stop_) {
      return geometry_msgs::msg::Twist();
    }
    if (navigation_active_) {
      if (!navigation_valid_ || now_seconds - navigation_received_at_ > navigation_timeout_) {
        return geometry_msgs::msg::Twist();
      }
      return navigation_command_;
    }
    if (!manual_valid_ || now_seconds - manual_received_at_ > manual_timeout_) {
      return geometry_msgs::msg::Twist();
    }
    return manual_command_;
  }

private:
  static bool isMoving(const geometry_msgs::msg::Twist & command)
  {
    constexpr double epsilon = 1.0e-6;
    return std::abs(command.linear.x) > epsilon ||
           std::abs(command.linear.y) > epsilon ||
           std::abs(command.linear.z) > epsilon ||
           std::abs(command.angular.x) > epsilon ||
           std::abs(command.angular.y) > epsilon ||
           std::abs(command.angular.z) > epsilon;
  }

  double manual_timeout_;
  double navigation_timeout_;
  bool manual_valid_{false};
  bool navigation_valid_{false};
  bool navigation_active_{false};
  bool emergency_stop_{false};
  double manual_received_at_{0.0};
  double navigation_received_at_{0.0};
  double navigation_activated_at_{0.0};
  geometry_msgs::msg::Twist manual_command_;
  geometry_msgs::msg::Twist navigation_command_;
};

}  // namespace luxi_3d_navigation

#endif  // LUXI_3D_NAVIGATION__VELOCITY_COMMAND_MUX_HPP_
