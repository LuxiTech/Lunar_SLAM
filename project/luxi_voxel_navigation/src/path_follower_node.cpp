#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class PathFollowerNode : public rclcpp::Node
{
public:
  PathFollowerNode()
  : Node("path_follower"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    declare_parameter<std::string>("path_topic", "/navigation/planned_path");
    declare_parameter<std::string>("start_topic", "/navigation/start");
    declare_parameter<std::string>("stop_topic", "/navigation/stop");
    declare_parameter<std::string>("cmd_vel_topic", "/navigation/cmd_vel");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<double>("control_rate", 15.0);
    declare_parameter<double>("lookahead_m", 0.25);
    declare_parameter<double>("goal_tolerance_m", 0.12);
    declare_parameter<double>("linear_gain", 0.6);
    declare_parameter<double>("angular_gain", 1.2);
    declare_parameter<double>("max_linear_speed", 0.10);
    declare_parameter<double>("max_angular_speed", 0.35);

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      get_parameter("path_topic").as_string(), rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&PathFollowerNode::onPath, this, std::placeholders::_1));
    start_sub_ = create_subscription<std_msgs::msg::Bool>(
      get_parameter("start_topic").as_string(), 10,
      std::bind(&PathFollowerNode::onStart, this, std::placeholders::_1));
    stop_sub_ = create_subscription<std_msgs::msg::Bool>(
      get_parameter("stop_topic").as_string(), 10,
      std::bind(&PathFollowerNode::onStop, this, std::placeholders::_1));
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      get_parameter("cmd_vel_topic").as_string(), 10);
    const double rate = std::max(1.0, get_parameter("control_rate").as_double());
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate), std::bind(&PathFollowerNode::onTimer, this));
  }

  ~PathFollowerNode() override
  {
    publishStop();
  }

private:
  void onPath(const nav_msgs::msg::Path::SharedPtr message)
  {
    path_ = message->poses;
    active_ = false;
    publishStop();
    RCLCPP_INFO(get_logger(), "Received %zu path poses; waiting for explicit start", path_.size());
  }

  void onStart(const std_msgs::msg::Bool::SharedPtr message)
  {
    if (!message->data) {
      active_ = false;
      publishStop();
      return;
    }
    if (path_.empty()) {
      RCLCPP_WARN(get_logger(), "Start requested before a path is available");
      return;
    }
    active_ = true;
    RCLCPP_INFO(get_logger(), "Voxel path following enabled");
  }

  void onStop(const std_msgs::msg::Bool::SharedPtr message)
  {
    if (message->data) {
      active_ = false;
      publishStop();
      RCLCPP_INFO(get_logger(), "Voxel path following stopped");
    }
  }

  void onTimer()
  {
    if (!active_ || path_.empty()) {
      return;
    }
    try {
      const auto transform = tf_buffer_.lookupTransform(
        get_parameter("map_frame").as_string(), get_parameter("base_frame").as_string(),
        tf2::TimePointZero);
      follow(transform.transform.translation.x, transform.transform.translation.y,
        tf2::getYaw(transform.transform.rotation));
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Localization unavailable: %s", error.what());
      publishStop();
    }
  }

  void follow(double robot_x, double robot_y, double robot_yaw)
  {
    const auto & goal = path_.back().pose.position;
    if (std::hypot(goal.x - robot_x, goal.y - robot_y) <= get_parameter("goal_tolerance_m").as_double()) {
      active_ = false;
      publishStop();
      RCLCPP_INFO(get_logger(), "Voxel navigation goal reached");
      return;
    }
    const double lookahead = get_parameter("lookahead_m").as_double();
    const geometry_msgs::msg::Point * target = &goal;
    for (const auto & pose : path_) {
      if (std::hypot(pose.pose.position.x - robot_x, pose.pose.position.y - robot_y) >= lookahead) {
        target = &pose.pose.position;
        break;
      }
    }
    const double dx = target->x - robot_x;
    const double dy = target->y - robot_y;
    const double heading = std::atan2(dy, dx) - robot_yaw;
    const double normalized_heading = std::atan2(std::sin(heading), std::cos(heading));
    geometry_msgs::msg::Twist command;
    command.angular.z = clamp(
      normalized_heading * get_parameter("angular_gain").as_double(),
      get_parameter("max_angular_speed").as_double());
    if (std::abs(normalized_heading) < 0.8) {
      command.linear.x = clamp(
        std::hypot(dx, dy) * get_parameter("linear_gain").as_double(),
        get_parameter("max_linear_speed").as_double());
    }
    cmd_pub_->publish(command);
  }

  static double clamp(double value, double limit)
  {
    return std::max(-limit, std::min(limit, value));
  }

  void publishStop() const
  {
    cmd_pub_->publish(geometry_msgs::msg::Twist());
  }

  bool active_{false};
  std::vector<geometry_msgs::msg::PoseStamped> path_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stop_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathFollowerNode>());
  rclcpp::shutdown();
  return 0;
}
