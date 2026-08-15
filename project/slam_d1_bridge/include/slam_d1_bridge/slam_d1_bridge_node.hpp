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

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "ddt_msgs/msg/user_command.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/float64.hpp"

#include "slam_d1_bridge/body_height_controller.hpp"
#include "slam_d1_bridge/command_converter.hpp"

namespace slam_d1_bridge
{

class SlamD1BridgeNode final : public rclcpp::Node
{
public:
    explicit SlamD1BridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~SlamD1BridgeNode() override;

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr message);
    void slam_pose_callback(
        const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message);
    void body_height_callback(const std_msgs::msg::Float64::SharedPtr message);
    void battery1_callback(const sensor_msgs::msg::BatteryState::SharedPtr message);
    void battery2_callback(const sensor_msgs::msg::BatteryState::SharedPtr message);
    void publish_timer_callback();
    void publish_command(ddt_msgs::msg::UserCommand command);

    [[nodiscard]] bool is_pose_valid(
        const geometry_msgs::msg::PoseWithCovarianceStamped & pose) const;

    std::string input_topic_;
    std::string slam_pose_topic_;
    std::string output_topic_;
    std::string body_height_command_topic_;
    std::string body_height_status_topic_;
    std::string battery1_input_topic_;
    std::string battery2_input_topic_;
    std::string battery1_status_topic_;
    std::string battery2_status_topic_;
    std::string startup_fsm_mode_;
    std::string fsm_mode_;
    std::string height_fsm_mode_;
    int startup_fsm_duration_ms_;
    int publish_period_ms_;
    int command_timeout_ms_;
    double lateral_velocity_tolerance_;
    double body_height_linear_z_scale_;

    CommandConverter converter_;
    BodyHeightController body_height_controller_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      slam_pose_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr body_height_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery1_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery2_subscription_;
    rclcpp::Publisher<ddt_msgs::msg::UserCommand>::SharedPtr command_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr body_height_status_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery1_status_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery2_status_publisher_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    std::mutex state_mutex_;
    geometry_msgs::msg::Twist latest_twist_;
    std::chrono::steady_clock::time_point last_command_time_;
    std::chrono::steady_clock::time_point node_started_at_{std::chrono::steady_clock::now()};
    bool has_valid_command_{false};
    bool height_control_active_{false};
    geometry_msgs::msg::PoseWithCovarianceStamped latest_slam_pose_;
    std::chrono::steady_clock::time_point last_pose_time_;
    bool has_slam_pose_{false};
    bool slam_pose_valid_{false};
};

}  // namespace slam_d1_bridge
