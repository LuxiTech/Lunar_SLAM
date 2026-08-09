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

#include <algorithm>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "luxi_3d_navigation/twist_direction_adapter.hpp"
#include "luxi_3d_navigation/velocity_command_mux.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace luxi_3d_navigation
{

class VelocityCommandMuxNode : public rclcpp::Node
{
public:
  VelocityCommandMuxNode()
  : Node("velocity_command_mux"),
    mux_(
      declare_parameter<double>("manual_command_timeout", 0.6),
      declare_parameter<double>("navigation_command_timeout", 0.3))
  {
    const auto manual_topic = declare_parameter<std::string>(
      "manual_input_topic", "/lekiwi/cmd_vel_standard");
    const auto navigation_topic = declare_parameter<std::string>(
      "navigation_input_topic", "/navigation/cmd_vel");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/cmd_vel");
    const auto active_topic = declare_parameter<std::string>(
      "navigation_active_topic", "/navigation/active");
    const auto stop_topic = declare_parameter<std::string>(
      "navigation_stop_topic", "/navigation/stop");
    const auto emergency_stop_topic = declare_parameter<std::string>(
      "emergency_stop_topic", "/navigation/emergency_stop");
    invert_angular_z_ = declare_parameter<bool>("invert_angular_z", false);
    const double publish_rate = declare_parameter<double>("publish_rate", 20.0);

    if (manual_topic.empty() || navigation_topic.empty() || output_topic.empty()) {
      throw std::invalid_argument("velocity mux topics must not be empty");
    }
    if (manual_topic == output_topic || navigation_topic == output_topic) {
      throw std::invalid_argument("velocity mux input and output topics must differ");
    }

    output_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic, 10);
    stop_pub_ = create_publisher<std_msgs::msg::Bool>(stop_topic, 10);
    manual_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      manual_topic, 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr message) {
        if (mux_.updateManual(*message, steadySeconds())) {
          publishNavigationStop("manual command override");
        }
      });
    navigation_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      navigation_topic, 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr message) {
        mux_.updateNavigation(*message, steadySeconds());
      });
    active_sub_ = create_subscription<std_msgs::msg::Bool>(
      active_topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        mux_.setNavigationActive(message->data, steadySeconds());
      });
    emergency_stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      emergency_stop_topic, 10,
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        mux_.setEmergencyStop(message->data);
        if (message->data) {
          publishNavigationStop("emergency stop");
        }
      });

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate));
    timer_ = create_wall_timer(period, [this]() {publishSelectedCommand();});
    RCLCPP_INFO(
      get_logger(), "Velocity mux: manual=%s navigation=%s output=%s invert_angular_z=%s",
      manual_topic.c_str(), navigation_topic.c_str(), output_topic.c_str(),
      invert_angular_z_ ? "true" : "false");
  }

  ~VelocityCommandMuxNode() override
  {
    if (output_pub_) {
      output_pub_->publish(geometry_msgs::msg::Twist());
    }
  }

private:
  static double steadySeconds()
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  void publishNavigationStop(const char * reason)
  {
    std_msgs::msg::Bool stop;
    stop.data = true;
    stop_pub_->publish(stop);
    RCLCPP_WARN(get_logger(), "Navigation stopped: %s", reason);
  }

  void publishSelectedCommand()
  {
    const double now_seconds = steadySeconds();
    if (mux_.navigationTimedOut(now_seconds)) {
      mux_.setNavigationActive(false, now_seconds);
      publishNavigationStop("navigation command timeout");
    }
    output_pub_->publish(adaptTwist(mux_.output(now_seconds), invert_angular_z_));
  }

  bool invert_angular_z_{false};
  VelocityCommandMux mux_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr manual_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr navigation_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr active_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace luxi_3d_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<luxi_3d_navigation::VelocityCommandMuxNode>());
  rclcpp::shutdown();
  return 0;
}
