基本流程就是：
把 slam_d1_bridge 源码目录复制到 SLAM NX；
在 SLAM NX 上编译；
source install/setup.bash；
设置 ROS 环境并运行启动脚本。
但有一个前提：SLAM NX 必须有 ddt_msgs。如果没有，源码还需要一起复制 D1 SDK 中的 ddt_msgs 包。
# SLAM NX
source /opt/ros/humble/setup.bash
# 如果 ddt_msgs 在其他工作空间，还要先 source 它

cd ~/ros2_ws
colcon build --symlink-install \
    --packages-select slam_d1_bridge \
    --cmake-args -DBUILD_TESTING=OFF

source install/setup.bash

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROBOT_NS=d15041873
编译前可检查：
ros2 interface show ddt_msgs/msg/UserCommand
确认 SLAM NX 能看到机器人和 /cmd_vel 后，再运行：
ros2 run slam_d1_bridge start_slam_d1_bridge.sh
