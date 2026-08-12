#include "sdk_command/sdk_command_node.hpp"
#include "d1_utils/d1_utils.hpp"
#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fstream>

SDKCmdNode::SDKCmdNode(const rclcpp::NodeOptions &options)
    : Node("user_command_node", options), running_(true) {
  this->declare_parameter("max_linear_velocity", 2.0);
  this->declare_parameter("max_angular_velocity", 1.0);
  this->get_parameter("max_linear_velocity", max_linear_vel_);
  this->get_parameter("max_angular_velocity", max_angular_vel_);

  RCLCPP_INFO(this->get_logger(), "User SDK with Keyboard Control!");
  RCLCPP_INFO(this->get_logger(), "Control Keys:\n"
                                  "  w/s: forward/backward\n"
                                  "  a/d: left/right\n"
                                  "  q/e: rotate left/right\n"
                                  "  1-5: change robot states\n"
                                  "  space: stop\n"
                                  "  r: reset pose\n"
                                  "  x: exit");

  // 初始化发布器
  user_command_publisher_ = this->create_publisher<ddt_msgs::msg::UserCommand>(
      ros_topic::manager_use_command,
      rclcpp::QoS(rclcpp::KeepLast(10))
          .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL));

  // 初始化控制变量
  current_command_.twist.linear.x = 0.0;
  current_command_.twist.linear.y = 0.0;
  current_command_.twist.linear.z = 0.0;
  current_command_.twist.angular.x = 0.0;
  current_command_.twist.angular.y = 0.0;
  current_command_.twist.angular.z = 0.0;

  // 初始化姿态
  updatePose(0.0, 0.0); // 水平姿态

  // 启动键盘监听线程
  keyboard_thread_ = std::thread(&SDKCmdNode::keyboardListener, this);

  // 启动控制命令发布定时器
  auto period = std::chrono::duration<double>(0.05); // 20Hz
  timer_ = this->create_wall_timer(
      period, std::bind(&SDKCmdNode::controlTimerCallback, this));
}

SDKCmdNode::~SDKCmdNode() {
  running_ = false;
  if (keyboard_thread_.joinable()) {
    keyboard_thread_.join();
  }
  RCLCPP_INFO(this->get_logger(), "SDK Command Node shutting down.");
}

void SDKCmdNode::controlTimerCallback() {
  std::lock_guard<std::mutex> lock(control_mutex_);

  // 发布当前控制命令
  auto cmd_msg = std::make_unique<ddt_msgs::msg::UserCommand>();

  // 将当前控制命令数据复制到消息对象
  cmd_msg->twist = current_command_.twist;
  cmd_msg->pose = current_command_.pose;
  cmd_msg->fsm_mode = current_fsm_state_;
  cmd_msg->header.stamp = this->get_clock()->now();
  cmd_msg->header.frame_id = "cmd";

  user_command_publisher_->publish(std::move(cmd_msg));
}

void SDKCmdNode::keyboardListener() {
  int tty_fd = open("/dev/tty", O_RDWR | O_NOCTTY);
  if (tty_fd < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open /dev/tty: %s", strerror(errno));
    return;
  }

  struct termios oldt, newt;
  tcgetattr(tty_fd, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(tty_fd, TCSANOW, &newt);

  int flags = fcntl(tty_fd, F_GETFL, 0);
  fcntl(tty_fd, F_SETFL, flags | O_NONBLOCK);

  RCLCPP_INFO(this->get_logger(), "Keyboard listener started.");

  while (running_ && rclcpp::ok()) {
    char key;
    int nread = read(tty_fd, &key, 1);

    if (nread > 0) {
      handleKeyPress(key);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  tcsetattr(tty_fd, TCSANOW, &oldt);
  close(tty_fd);
}

void SDKCmdNode::handleKeyPress(char key) {
  std::lock_guard<std::mutex> lock(control_mutex_);

  switch (key) {
  case 'w': // 前进
    current_command_.twist.linear.x = 0.5;
    current_command_.twist.linear.y = 0.0;
    RCLCPP_INFO(this->get_logger(), "Forward");
    break;

  case 's': // 后退
    current_command_.twist.linear.x = -0.5;
    current_command_.twist.linear.y = 0.0;
    RCLCPP_INFO(this->get_logger(), "Backward");
    break;

  case 'a': // 左移
    current_command_.twist.linear.y = 0.5;
    current_command_.twist.linear.x = 0.0;
    RCLCPP_INFO(this->get_logger(), "Left");
    break;

  case 'd': // 右移
    current_command_.twist.linear.y = -0.5;
    current_command_.twist.linear.x = 0.0;
    RCLCPP_INFO(this->get_logger(), "Right");
    break;

  case 'q': // 左转
    current_command_.twist.angular.z = 0.5;
    RCLCPP_INFO(this->get_logger(), "Rotate Left");
    break;

  case 'e': // 右转
    current_command_.twist.angular.z = -0.5;
    RCLCPP_INFO(this->get_logger(), "Rotate Right");
    break;

  case ' ': // 停止
    current_command_.twist.linear.x = 0.0;
    current_command_.twist.linear.y = 0.0;
    current_command_.twist.angular.z = 0.0;
    RCLCPP_INFO(this->get_logger(), "Stop");
    break;

  case '1': // 状态1
    current_fsm_state_ = "rl_3";
    RCLCPP_INFO(this->get_logger(), "State: rl_3");
    break;

  case '2': // 状态2
    current_fsm_state_ = "car";
    RCLCPP_INFO(this->get_logger(), "State: car");
    break;

  case '4': // 状态4
    current_fsm_state_ = "transform_down";
    RCLCPP_INFO(this->get_logger(), "State: transform_down");
    break;

  case '5': // 状态5
    current_fsm_state_ = "transform_up";
    RCLCPP_INFO(this->get_logger(), "State: transform_up");
    break;

  case '6': // 状态6
    current_fsm_state_ = "loco";
    RCLCPP_INFO(this->get_logger(), "State: loco");
    break;
  case 'r': // 重置姿态
    current_command_.pose.position.z = 0.2;
    RCLCPP_INFO(this->get_logger(), "Reset pose to horizontal");
    break;
  case 't': // 重置姿态
    updatePose(0.0, -3.0);
    RCLCPP_INFO(this->get_logger(), "Reset pose to horizontal");
    break;
  case 'u': // 重置姿态
    updatePose(8.0, 0.0);
    RCLCPP_INFO(this->get_logger(), "Reset pose to horizontal");
    break;
  case 'i': // 重置姿态
    updatePose(-8.0, 0.0);
    RCLCPP_INFO(this->get_logger(), "Reset pose to horizontal");
    break;
  case 'p':
    updatePose(-8.0, -3.0);
    RCLCPP_INFO(this->get_logger(), "Reset pose to horizontal");
    break;
  case 'x': // 退出
    RCLCPP_INFO(this->get_logger(), "Exiting...");
    running_ = false;
    rclcpp::shutdown();
    break;

  default:
    break;
  }
}

void SDKCmdNode::updatePose(double pitch_deg, double roll_deg) {
  auto pitch_cmd = d1_utils::pitchToQuaternion(pitch_deg);
  auto roll_cmd = d1_utils::rollToQuaternion(roll_deg);
  auto body_pose = d1_utils::multiplyQuaternions(pitch_cmd, roll_cmd);

  current_command_.pose.position.x = body_pose.x;
  current_command_.pose.position.y = body_pose.y;
  current_command_.pose.position.z = body_pose.z;
  current_command_.pose.orientation.w = body_pose.w;
}
void SDKCmdNode::resetControlParams() {
  current_command_.twist.linear.x = 0.0;
  current_command_.twist.linear.y = 0.0;
  current_command_.twist.angular.z = 0.0;
}