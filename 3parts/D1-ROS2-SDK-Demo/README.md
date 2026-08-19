# D1-ROS2-SDK-Demo
## 功能实现

* 基于D1-ROS2-SDK 实现键盘输入实现机器人实机的控制

## 快速开始
 
1.创建ros工程文件夹

```bash 
#make sure you have build all dependence.

sudo apt-get install python3-colcon-common-extensions
mkdir -p ~/d1_ws/src
cd ~/d1_ws/src

#clone API source code
git clone https://github.com/DDTRobot/D1-ROS2-SDK-Demo.git

cd ~/d1_ws
colcon build
source install/setup.bash
# please read sdk_command_node.cpp
ros2 launch d1_bringup sdk_launch.py

```
说明：遥控器菜单页`08 SDK Mode`带*后遥控不发送command话题，可由用户通过sdk发送话题来控制机器状态机切换，速度控制

**注意:** 机器人始终会保持最后一条指令的执行动作,若将机器人保持停止运动,需要将控制参数重置为0。