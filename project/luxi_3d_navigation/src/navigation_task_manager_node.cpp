#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace luxi_3d_navigation
{

class NavigationTaskManagerNode : public rclcpp::Node
{
public:
  NavigationTaskManagerNode()
  : Node("navigation_task_manager")
  {
    declare_parameter<std::string>("task_goal_topic", "/navigation/task/goal_pose");
    declare_parameter<std::string>("task_home_topic", "/navigation/task/home_pose");
    declare_parameter<std::string>("task_start_topic", "/navigation/task/start");
    declare_parameter<std::string>("task_cancel_topic", "/navigation/task/cancel");
    declare_parameter<std::string>("task_return_home_topic", "/navigation/task/return_home");
    declare_parameter<std::string>("task_status_topic", "/navigation/task/status");
    declare_parameter<std::string>("task_home_state_topic", "/navigation/task/home_state");
    declare_parameter<std::string>("planner_goal_topic", "/navigation/goal_pose");
    declare_parameter<std::string>("follower_start_topic", "/navigation/start");
    declare_parameter<std::string>("follower_stop_topic", "/navigation/stop");
    declare_parameter<std::string>("clear_path_topic", "/navigation/clear_path");
    declare_parameter<std::string>("path_topic", "/navigation/planned_path");
    declare_parameter<std::string>("planning_status_topic", "/navigation/planning_status");
    declare_parameter<std::string>("local_planner_status_topic", "/navigation/local_replan/status");
    declare_parameter<std::string>("follower_state_topic", "/navigation/follower_state");
    declare_parameter<std::string>("follower_active_topic", "/navigation/active");

    const auto commands = rclcpp::QoS(10).reliable();
    const auto state = rclcpp::QoS(1).reliable().transient_local();
    planner_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      get_parameter("planner_goal_topic").as_string(), commands);
    follower_start_pub_ = create_publisher<std_msgs::msg::Bool>(
      get_parameter("follower_start_topic").as_string(), commands);
    follower_stop_pub_ = create_publisher<std_msgs::msg::Bool>(
      get_parameter("follower_stop_topic").as_string(), commands);
    clear_path_pub_ = create_publisher<std_msgs::msg::Bool>(
      get_parameter("clear_path_topic").as_string(), commands);
    status_pub_ = create_publisher<std_msgs::msg::String>(
      get_parameter("task_status_topic").as_string(), state);
    home_state_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      get_parameter("task_home_state_topic").as_string(), state);

    task_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      get_parameter("task_goal_topic").as_string(), commands,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
        beginGoal(*message, false);
      });
    task_home_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      get_parameter("task_home_topic").as_string(), commands,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
        home_ = *message;
        home_state_pub_->publish(*message);
        if (!task_in_progress_) {
          publishStatus("home_saved");
        }
        RCLCPP_INFO(get_logger(), "Return pose saved in frame %s", message->header.frame_id.c_str());
      });
    task_start_sub_ = create_subscription<std_msgs::msg::Bool>(
      get_parameter("task_start_topic").as_string(), commands,
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        if (!message->data) {
          return;
        }
        if (!goal_) {
          publishStatus("failed_goal_unavailable");
          return;
        }
        execute_pending_ = true;
        task_in_progress_ = true;
        maybeStartFollower();
      });
    task_cancel_sub_ = create_subscription<std_msgs::msg::Bool>(
      get_parameter("task_cancel_topic").as_string(), commands,
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        if (message->data) {
          cancelTask();
        }
      });
    task_return_sub_ = create_subscription<std_msgs::msg::Bool>(
      get_parameter("task_return_home_topic").as_string(), commands,
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        if (!message->data) {
          return;
        }
        if (!home_) {
          publishStatus("failed_home_unavailable");
          return;
        }
        beginGoal(*home_, true);
      });
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      get_parameter("path_topic").as_string(), state,
      [this](const nav_msgs::msg::Path::SharedPtr message) {
        path_ready_ = !message->poses.empty();
        if (path_ready_) {
          maybeStartFollower();
        } else if (task_in_progress_) {
          publishWaitingForPath();
        }
      });
    planning_status_sub_ = create_subscription<std_msgs::msg::String>(
      get_parameter("planning_status_topic").as_string(), state,
      [this](const std_msgs::msg::String::SharedPtr message) {
        if (message->data.rfind("failed_", 0) == 0U && task_in_progress_) {
          task_in_progress_ = false;
          execute_pending_ = false;
          path_ready_ = false;
          publishStatus(message->data);
        }
      });
    local_status_sub_ = create_subscription<std_msgs::msg::String>(
      get_parameter("local_planner_status_topic").as_string(), state,
      [this](const std_msgs::msg::String::SharedPtr message) {
        local_planner_state_ = message->data;
        if (task_in_progress_ && localPlannerBlocked()) {
          path_ready_ = false;
          publishWaitingForPath();
        } else {
          maybeStartFollower();
        }
      });
    follower_state_sub_ = create_subscription<std_msgs::msg::String>(
      get_parameter("follower_state_topic").as_string(), state,
      [this](const std_msgs::msg::String::SharedPtr message) {onFollowerState(message->data);});
    follower_active_sub_ = create_subscription<std_msgs::msg::Bool>(
      get_parameter("follower_active_topic").as_string(), state,
      [this](const std_msgs::msg::Bool::SharedPtr message) {
        follower_active_ = message->data;
      });
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {maybeStartFollower();});
    publishStatus("idle");
  }

private:
  bool localPlannerBlocked() const
  {
    return local_planner_state_ == "no_path" || local_planner_state_ == "sensor_stale" ||
           local_planner_state_ == "localization_unavailable" ||
           local_planner_state_ == "planning";
  }

  void beginGoal(const geometry_msgs::msg::PoseStamped & goal, const bool returning)
  {
    publishFollowerStop();
    goal_ = goal;
    returning_ = returning;
    execute_pending_ = returning;
    task_in_progress_ = true;
    path_ready_ = false;
    local_planner_state_ = "planning";
    planner_goal_pub_->publish(goal);
    publishStatus(returning ? "return_planning" : "planning");
    RCLCPP_INFO(
      get_logger(), "%s goal accepted at (%.3f, %.3f)",
      returning ? "Return" : "Navigation", goal.pose.position.x, goal.pose.position.y);
  }

  void maybeStartFollower()
  {
    if (!execute_pending_ || !path_ready_ || follower_active_ || localPlannerBlocked()) {
      if (execute_pending_ && localPlannerBlocked()) {
        publishWaitingForPath();
      }
      return;
    }
    std_msgs::msg::Bool start;
    start.data = true;
    follower_start_pub_->publish(start);
    execute_pending_ = false;
    publishStatus(returning_ ? "return_starting" : "starting");
  }

  void cancelTask()
  {
    publishFollowerStop();
    publishClearPath();
    goal_.reset();
    returning_ = false;
    execute_pending_ = false;
    task_in_progress_ = false;
    path_ready_ = false;
    follower_active_ = false;
    publishStatus("cancelled");
    RCLCPP_INFO(get_logger(), "Navigation task cancelled and path cleared");
  }

  void onFollowerState(const std::string & state)
  {
    if (state == "goal_reached") {
      task_in_progress_ = false;
      execute_pending_ = false;
      path_ready_ = false;
      publishStatus(returning_ ? "home_reached" : "goal_reached");
      publishClearPath();
      returning_ = false;
      return;
    }
    if (state.rfind("localization_recovery_", 0) == 0U ||
      state.rfind("replanning_", 0) == 0U || state == "localization_dead_reckoning")
    {
      publishStatus("recovering_localization");
      return;
    }
    if (state == "replan_after_relocalization_timeout" || state == "localization_lost" ||
      state == "obstacle_recovery_timeout" || state == "stuck_no_progress")
    {
      task_in_progress_ = false;
      execute_pending_ = false;
      publishStatus("failed_" + state);
      publishClearPath();
      return;
    }
    if (state == "active" || state == "aligning_goal_heading") {
      task_in_progress_ = true;
      publishStatus(returning_ ? "return_active" : "active");
    }
  }

  void publishFollowerStop()
  {
    std_msgs::msg::Bool stop;
    stop.data = true;
    follower_stop_pub_->publish(stop);
  }

  void publishClearPath()
  {
    std_msgs::msg::Bool clear;
    clear.data = true;
    clear_path_pub_->publish(clear);
  }

  void publishWaitingForPath()
  {
    publishStatus(returning_ ? "return_waiting_for_safe_path" : "waiting_for_safe_path");
  }

  void publishStatus(const std::string & status)
  {
    if (status == status_) {
      return;
    }
    status_ = status;
    std_msgs::msg::String message;
    message.data = status;
    status_pub_->publish(message);
  }

  std::optional<geometry_msgs::msg::PoseStamped> goal_;
  std::optional<geometry_msgs::msg::PoseStamped> home_;
  std::string status_;
  std::string local_planner_state_;
  bool returning_{false};
  bool execute_pending_{false};
  bool task_in_progress_{false};
  bool path_ready_{false};
  bool follower_active_{false};
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr planner_goal_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr follower_start_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr follower_stop_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr clear_path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr home_state_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr task_goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr task_home_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr task_start_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr task_cancel_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr task_return_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr planning_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr local_status_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr follower_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr follower_active_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace luxi_3d_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<luxi_3d_navigation::NavigationTaskManagerNode>());
  rclcpp::shutdown();
  return 0;
}
