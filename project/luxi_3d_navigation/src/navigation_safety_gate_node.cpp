#include <algorithm>
#include <chrono>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include "luxi_3d_navigation/navigation_safety_gate.hpp"

namespace luxi_3d_navigation
{

class NavigationSafetyGateNode : public rclcpp::Node
{
public:
  NavigationSafetyGateNode()
  : Node("navigation_safety_gate"), gate_(parameters())
  {
    monitor_only_ = declare_parameter<bool>("monitor_only", false);
    const auto input_topic = declare_parameter<std::string>(
      "input_topic", "/navigation/cmd_vel_raw");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/navigation/cmd_vel");
    const auto active_topic = declare_parameter<std::string>(
      "active_topic", "/navigation/active");
    const auto obstacle_topic = declare_parameter<std::string>(
      "obstacle_state_topic", "/navigation/local_obstacles/state");
    const auto planner_topic = declare_parameter<std::string>(
      "planner_state_topic", "/navigation/local_replan/status");
    const auto state_topic = declare_parameter<std::string>(
      "state_topic", "/navigation/safety_gate/state");
    const auto limited_topic = declare_parameter<std::string>(
      "limited_topic", "/navigation/safety_limited");
    const auto hard_stop_topic = declare_parameter<std::string>(
      "hard_stop_topic", "/navigation/safety_hard_stop");
    const double publish_rate = std::max(1.0, declare_parameter<double>("publish_rate", 20.0));

    output_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic, 10);
    state_pub_ = create_publisher<std_msgs::msg::String>(
      state_topic, rclcpp::QoS(1).reliable().transient_local());
    limited_pub_ = create_publisher<std_msgs::msg::Bool>(
      limited_topic, rclcpp::QoS(1).reliable().transient_local());
    hard_stop_pub_ = create_publisher<std_msgs::msg::Bool>(
      hard_stop_topic, rclcpp::QoS(1).reliable().transient_local());
    command_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      input_topic, 10, [this](const geometry_msgs::msg::Twist::SharedPtr message) {
        gate_.updateCommand(*message, steadyNow());
      });
    active_sub_ = create_subscription<std_msgs::msg::Bool>(
      active_topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        gate_.setNavigationActive(message->data, steadyNow());
      });
    obstacle_sub_ = create_subscription<std_msgs::msg::String>(
      obstacle_topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr message) {
        gate_.updateObstacleState(message->data, steadyNow());
      });
    planner_sub_ = create_subscription<std_msgs::msg::String>(
      planner_topic, rclcpp::QoS(1).reliable().transient_local(),
      [this](const std_msgs::msg::String::SharedPtr message) {
        gate_.updatePlannerState(message->data, steadyNow());
      });
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate), [this]() {publish();});
    RCLCPP_INFO(
      get_logger(), "Navigation safety gate: input=%s output=%s monitor_only=%s",
      input_topic.c_str(), output_topic.c_str(), monitor_only_ ? "true" : "false");
  }

  ~NavigationSafetyGateNode() override
  {
    if (output_pub_) {
      output_pub_->publish(geometry_msgs::msg::Twist());
    }
  }

private:
  SafetyGateParameters parameters()
  {
    SafetyGateParameters result;
    result.command_timeout = declare_parameter<double>("command_timeout", 0.20);
    result.obstacle_timeout = declare_parameter<double>("obstacle_timeout", 0.35);
    result.planner_timeout = declare_parameter<double>("planner_timeout", 1.0);
    result.slow_scale = declare_parameter<double>("slow_scale", 0.50);
    result.minimum_linear_speed = declare_parameter<double>("minimum_linear_speed", 0.0);
    result.healthy_resume_hold = declare_parameter<double>("healthy_resume_hold", 0.50);
    return result;
  }

  static double steadyNow()
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  void publish()
  {
    const auto result = gate_.evaluate(steadyNow(), monitor_only_);
    output_pub_->publish(result.command);
    std_msgs::msg::String state;
    state.data = result.state;
    state_pub_->publish(state);
    std_msgs::msg::Bool limited;
    limited.data = result.limited;
    limited_pub_->publish(limited);
    std_msgs::msg::Bool hard_stop;
    hard_stop.data = result.emergency_stop;
    hard_stop_pub_->publish(hard_stop);
  }

  NavigationSafetyGate gate_;
  bool monitor_only_{false};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr limited_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hard_stop_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr active_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr obstacle_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr planner_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace luxi_3d_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<luxi_3d_navigation::NavigationSafetyGateNode>());
  rclcpp::shutdown();
  return 0;
}
