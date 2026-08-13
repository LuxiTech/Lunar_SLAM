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

#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "luxi_3d_navigation/twist_direction_adapter.hpp"
#include "rclcpp/rclcpp.hpp"

namespace luxi_3d_navigation
{

class TwistDirectionAdapterNode : public rclcpp::Node
{
public:
  TwistDirectionAdapterNode()
  : Node("twist_direction_adapter")
  {
    const auto input_topic = declare_parameter<std::string>(
      "input_topic", "/lekiwi/cmd_vel_standard");
    const auto output_topic = declare_parameter<std::string>("output_topic", "/cmd_vel");
    invert_angular_z_ = declare_parameter<bool>("invert_angular_z", false);

    if (input_topic.empty() || output_topic.empty()) {
      throw std::invalid_argument("input_topic and output_topic must not be empty");
    }
    if (input_topic == output_topic) {
      throw std::invalid_argument("input_topic and output_topic must differ");
    }

    publisher_ = create_publisher<geometry_msgs::msg::Twist>(output_topic, 10);
    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      input_topic, 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr message) {
        publisher_->publish(adaptTwist(*message, invert_angular_z_));
      });

    RCLCPP_INFO(
      get_logger(), "Twist direction adapter: %s -> %s, invert_angular_z=%s",
      input_topic.c_str(), output_topic.c_str(), invert_angular_z_ ? "true" : "false");
  }

private:
  bool invert_angular_z_{false};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

}  // namespace luxi_3d_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<luxi_3d_navigation::TwistDirectionAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
