#ifndef SDK_COMMAND_NODE_HPP
#define SDK_COMMAND_NODE_HPP

#include "d1_utils/topic_names.hpp"
#include "ddt_msgs/msg/user_command.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include <atomic>
#include <fcntl.h>
#include <mutex>
#include <termios.h>
#include <thread>
#include <unistd.h>

class SDKCmdNode : public rclcpp::Node {
public:
  explicit SDKCmdNode(const rclcpp::NodeOptions &options);
  ~SDKCmdNode();

private:
  // 定时器回调函数，用于发布当前控制命令
  void controlTimerCallback();
  //订阅回调函数，用更新当前关节状态
  // void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  // 键盘监听线程函数
  void keyboardListener();
  // 处理按键输入
  void handleKeyPress(char key);
  // 更新姿态
  void updatePose(double pitch_deg, double roll_deg);

  // 重置控制参数

  void resetControlParams();

  // 发布器
  rclcpp::Publisher<ddt_msgs::msg::UserCommand>::SharedPtr
      user_command_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  // 控制变量
  ddt_msgs::msg::UserCommand current_command_;
  std::string current_fsm_state_;

  // 关节状态
  sensor_msgs::msg::JointState joints_;

  // 键盘监听线程
  std::thread keyboard_thread_;
  std::atomic<bool> running_;
  std::mutex control_mutex_;

  // 参数
  double max_linear_vel_;
  double max_angular_vel_;
};

#endif // SDK_COMMAND_NODE_HPP