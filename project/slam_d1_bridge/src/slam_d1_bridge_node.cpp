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

#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>

#include "slam_d1_bridge/slam_d1_bridge_node.hpp"

namespace slam_d1_bridge
{

namespace
{

constexpr int DEFAULT_PUBLISH_PERIOD_MS = 50;
constexpr int DEFAULT_COMMAND_TIMEOUT_MS = 300;
constexpr double DEFAULT_MAX_LINEAR_X = 0.5;
constexpr double DEFAULT_MAX_ANGULAR_Z = 0.5;
constexpr double DEFAULT_LATERAL_VELOCITY_TOLERANCE = 0.001;
constexpr double DEFAULT_BODY_HEIGHT_MINIMUM = 0.0;
constexpr double DEFAULT_BODY_HEIGHT_MAXIMUM = 9.0;
constexpr double DEFAULT_BODY_HEIGHT = 0.0;
constexpr double DEFAULT_BODY_HEIGHT_RATE = 1.0;
constexpr double DEFAULT_BODY_HEIGHT_LINEAR_Z_SCALE = 0.03;
constexpr double MIN_QUATERNION_NORM = 1.0e-6;

}  // namespace

SlamD1BridgeNode::SlamD1BridgeNode(const rclcpp::NodeOptions & options)
  : Node("slam_d1_bridge", options),
    input_topic_(declare_parameter<std::string>("input_topic", "cmd_vel")),
    slam_pose_topic_(declare_parameter<std::string>("slam_pose_topic", "slam/pose")),
    output_topic_(declare_parameter<std::string>("output_topic", "command/user_command")),
    body_height_command_topic_(declare_parameter<std::string>(
          "body_height_command_topic", "command/body_height")),
    body_height_status_topic_(declare_parameter<std::string>(
          "body_height_status_topic", "status/body_height")),
    battery1_input_topic_(declare_parameter<std::string>(
          "battery1_input_topic", "system_status_broadcaster/battery1")),
    battery2_input_topic_(declare_parameter<std::string>(
          "battery2_input_topic", "system_status_broadcaster/battery2")),
    battery1_status_topic_(declare_parameter<std::string>(
          "battery1_status_topic", "status/battery1")),
    battery2_status_topic_(declare_parameter<std::string>(
          "battery2_status_topic", "status/battery2")),
    startup_fsm_mode_(declare_parameter<std::string>("startup_fsm_mode", "")),
    fsm_mode_(declare_parameter<std::string>("fsm_mode", "")),
    height_fsm_mode_(declare_parameter<std::string>("height_fsm_mode", "")),
    startup_fsm_duration_ms_(declare_parameter<int>("startup_fsm_duration_ms", 0)),
    publish_period_ms_(declare_parameter<int>("publish_period_ms", DEFAULT_PUBLISH_PERIOD_MS)),
    command_timeout_ms_(declare_parameter<int>("command_timeout_ms", DEFAULT_COMMAND_TIMEOUT_MS)),
    lateral_velocity_tolerance_(declare_parameter<double>(
          "lateral_velocity_tolerance", DEFAULT_LATERAL_VELOCITY_TOLERANCE)),
    body_height_linear_z_scale_(declare_parameter<double>(
          "body_height_linear_z_scale", DEFAULT_BODY_HEIGHT_LINEAR_Z_SCALE)),
    converter_(CommandLimits{
        declare_parameter<double>("max_linear_x", DEFAULT_MAX_LINEAR_X),
        declare_parameter<double>("max_angular_z", DEFAULT_MAX_ANGULAR_Z)}),
    body_height_controller_(BodyHeightLimits{
        declare_parameter<double>("body_height_minimum", DEFAULT_BODY_HEIGHT_MINIMUM),
        declare_parameter<double>("body_height_maximum", DEFAULT_BODY_HEIGHT_MAXIMUM),
        declare_parameter<double>("body_height_default", DEFAULT_BODY_HEIGHT),
        declare_parameter<double>("body_height_maximum_rate", DEFAULT_BODY_HEIGHT_RATE)})
{
    if (input_topic_.empty() || slam_pose_topic_.empty() || output_topic_.empty() ||
      body_height_command_topic_.empty() || body_height_status_topic_.empty() ||
      battery1_input_topic_.empty() || battery2_input_topic_.empty() ||
      battery1_status_topic_.empty() || battery2_status_topic_.empty())
    {
        throw std::invalid_argument("topic parameters must not be empty");
    }
    if (publish_period_ms_ <= 0 || command_timeout_ms_ <= 0) {
        throw std::invalid_argument("publish_period_ms and command_timeout_ms must be positive");
    }
    if (startup_fsm_duration_ms_ < 0) {
        throw std::invalid_argument("startup_fsm_duration_ms must be non-negative");
    }
    if (!std::isfinite(lateral_velocity_tolerance_) || lateral_velocity_tolerance_ < 0.0) {
        throw std::invalid_argument("lateral_velocity_tolerance must be finite and non-negative");
    }
    if (!std::isfinite(body_height_linear_z_scale_) || body_height_linear_z_scale_ <= 0.0) {
        throw std::invalid_argument("body_height_linear_z_scale must be finite and positive");
    }

    const auto input_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    const auto output_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    const auto status_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    // The deployed D1 broadcaster is RELIABLE + TRANSIENT_LOCAL. Matching it
    // lets a newly started bridge receive the retained battery sample at once.
    const auto battery_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
        input_topic_, input_qos,
        std::bind(&SlamD1BridgeNode::cmd_vel_callback, this, std::placeholders::_1));
    slam_pose_subscription_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        slam_pose_topic_, input_qos,
        std::bind(&SlamD1BridgeNode::slam_pose_callback, this, std::placeholders::_1));
    body_height_subscription_ = create_subscription<std_msgs::msg::Float64>(
        body_height_command_topic_, input_qos,
        std::bind(&SlamD1BridgeNode::body_height_callback, this, std::placeholders::_1));
    battery1_subscription_ = create_subscription<sensor_msgs::msg::BatteryState>(
        battery1_input_topic_, battery_qos,
        std::bind(&SlamD1BridgeNode::battery1_callback, this, std::placeholders::_1));
    battery2_subscription_ = create_subscription<sensor_msgs::msg::BatteryState>(
        battery2_input_topic_, battery_qos,
        std::bind(&SlamD1BridgeNode::battery2_callback, this, std::placeholders::_1));
    command_publisher_ = create_publisher<ddt_msgs::msg::UserCommand>(output_topic_, output_qos);
    body_height_status_publisher_ = create_publisher<std_msgs::msg::Float64>(
        body_height_status_topic_, status_qos);
    battery1_status_publisher_ = create_publisher<sensor_msgs::msg::BatteryState>(
        battery1_status_topic_, battery_qos);
    battery2_status_publisher_ = create_publisher<sensor_msgs::msg::BatteryState>(
        battery2_status_topic_, battery_qos);
    publish_timer_ = create_wall_timer(
        std::chrono::milliseconds(publish_period_ms_),
        std::bind(&SlamD1BridgeNode::publish_timer_callback, this));

    RCLCPP_INFO(
        get_logger(),
        "Bridge ready: '%s' -> '%s', pose='%s', height='%s', period=%d ms, timeout=%d ms",
        input_topic_.c_str(), output_topic_.c_str(), slam_pose_topic_.c_str(),
        body_height_command_topic_.c_str(), publish_period_ms_, command_timeout_ms_);
}

SlamD1BridgeNode::~SlamD1BridgeNode()
{
    if (command_publisher_) {
        const std::string & stop_fsm_mode =
          height_control_active_ && !height_fsm_mode_.empty() ? height_fsm_mode_ : fsm_mode_;
        publish_command(CommandConverter::make_stop(stop_fsm_mode));
    }
}

void SlamD1BridgeNode::body_height_callback(const std_msgs::msg::Float64::SharedPtr message)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!std::isfinite(message->data)) {
        RCLCPP_ERROR(get_logger(), "Rejected non-finite body-height command");
        return;
    }
    const double accepted = body_height_controller_.set_target(message->data);
    height_control_active_ = true;
    if (accepted != message->data) {
        RCLCPP_WARN(
            get_logger(), "Clamped biped height level from %.3f to %.3f",
            message->data, accepted);
    }
}

void SlamD1BridgeNode::battery1_callback(
    const sensor_msgs::msg::BatteryState::SharedPtr message)
{
    battery1_status_publisher_->publish(*message);
}

void SlamD1BridgeNode::battery2_callback(
    const sensor_msgs::msg::BatteryState::SharedPtr message)
{
    battery2_status_publisher_->publish(*message);
}

void SlamD1BridgeNode::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr message)
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!converter_.is_valid(*message)) {
        has_valid_command_ = false;
        latest_twist_ = geometry_msgs::msg::Twist{};
        RCLCPP_ERROR_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Rejected cmd_vel containing NaN or infinity; commanding stop");
        return;
    }

    latest_twist_ = *message;
    last_command_time_ = std::chrono::steady_clock::now();
    has_valid_command_ = true;

    if (std::abs(message->linear.y) > lateral_velocity_tolerance_) {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(), 2000,
            "D1 single-unit mode does not support lateral velocity; "
            "linear.y=%.3f is forced to zero",
            message->linear.y);
    }
}

void SlamD1BridgeNode::slam_pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
{
    const bool valid = is_pose_valid(*message);
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_slam_pose_ = *message;
        last_pose_time_ = std::chrono::steady_clock::now();
        has_slam_pose_ = true;
        slam_pose_valid_ = valid;
    }

    if (!valid) {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(), 5000,
            "Received invalid SLAM pose; pose is cached but does not affect "
            "phase-one motion control");
    }
}

void SlamD1BridgeNode::publish_timer_callback()
{
    geometry_msgs::msg::Twist twist;
    bool command_is_fresh = false;
    double body_height = 0.0;
    double body_height_level_rate = 0.0;
    bool height_control_active = false;
    const auto now = std::chrono::steady_clock::now();
    const bool use_startup_mode = !startup_fsm_mode_.empty() &&
      now - node_started_at_ < std::chrono::milliseconds(startup_fsm_duration_ms_);
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        command_is_fresh = has_valid_command_ &&
          now - last_command_time_ <= std::chrono::milliseconds(command_timeout_ms_);
        if (command_is_fresh) {
            twist = latest_twist_;
        }
        height_control_active = height_control_active_;
        const double elapsed_seconds = static_cast<double>(publish_period_ms_) / 1000.0;
        const double previous_height = body_height_controller_.current();
        if (height_control_active && !use_startup_mode) {
            body_height = body_height_controller_.step(elapsed_seconds);
            body_height_level_rate = (body_height - previous_height) / elapsed_seconds;
        } else {
            body_height = previous_height;
        }
    }

    const std::string & requested_fsm_mode =
      height_control_active && !height_fsm_mode_.empty() ? height_fsm_mode_ : fsm_mode_;
    const std::string & active_fsm_mode =
      use_startup_mode ? startup_fsm_mode_ : requested_fsm_mode;
    const double body_height_linear_z =
      body_height_level_rate * body_height_linear_z_scale_;
    auto command = command_is_fresh ?
      converter_.convert(twist, active_fsm_mode, body_height_linear_z) :
      CommandConverter::make_stop(active_fsm_mode, body_height_linear_z);
    publish_command(std::move(command));
    std_msgs::msg::Float64 height_status;
    height_status.data = body_height;
    body_height_status_publisher_->publish(height_status);
}

void SlamD1BridgeNode::publish_command(ddt_msgs::msg::UserCommand command)
{
    command.header.stamp = get_clock()->now();
    command_publisher_->publish(std::move(command));
}

bool SlamD1BridgeNode::is_pose_valid(
    const geometry_msgs::msg::PoseWithCovarianceStamped & pose) const
{
    const auto & position = pose.pose.pose.position;
    const auto & orientation = pose.pose.pose.orientation;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(position.z) || !std::isfinite(orientation.x) ||
      !std::isfinite(orientation.y) || !std::isfinite(orientation.z) ||
      !std::isfinite(orientation.w))
    {
        return false;
    }

    const double norm_squared = orientation.x * orientation.x +
      orientation.y * orientation.y + orientation.z * orientation.z +
      orientation.w * orientation.w;
    return norm_squared > MIN_QUATERNION_NORM * MIN_QUATERNION_NORM;
}

}  // namespace slam_d1_bridge
