# 调试 Guide：联调前网络与测试流程

## 1. 适用范围

本文用于 SLAM NX 与自有 NX 首次联调，机器人实际 namespace 为：

```text
d15041873
```

本文的前半段可以在机器人断电时完成，不会向 D1 发送控制命令。只有最后的
`start_slam_d1_bridge.sh` 才会启用 SDK、站立并启动实际桥接控制。

## 2. 网络拓扑

```text
SLAM NX ──直连网线── 自有 NX ──公司 Wi-Fi── D1
192.168.123.x/24       192.168.123.50/24
                        192.168.0.x/24
```

自有 NX 的有线接口为 `eno1`。SLAM NX 与自有 NX 使用 `192.168.123.0/24`，D1
继续使用已验证的公司 Wi-Fi 网络。

## 3. 重要说明

如果机器人是断电状态，不能执行：

```bash
ros2 run slam_d1_bridge start_slam_d1_bridge.sh
```

开始脚本会检查以下 D1 接口：

```text
/d15041873/command/user_command
/d15041873/teleop_command
```

机器人断电时这些接口不存在，脚本会安全退出，不会启动实际桥接流程。

机器人断电时应先使用 `test_d1` namespace 验证桥接转换。

## 4. 连接网线并确认自有 NX 地址

在自有 NX 上执行：

```bash
ip -4 addr show eno1
```

应看到：

```text
192.168.123.50/24
```

如果网线刚插入但还没有地址，检查 NetworkManager 配置并激活连接：

```bash
nmcli device status
sudo nmcli connection up slam-link
ip -4 addr show eno1
```

`eno1` 不配置默认网关和 DNS；自有 NX 的默认路由继续走 Wi-Fi。

## 5. 双向 ping

假设 SLAM NX 的地址为 `<SLAM_NX_IP>`，在自有 NX 上执行：

```bash
ping <SLAM_NX_IP>
```

同时让 SLAM NX 执行：

```bash
ping 192.168.123.50
```

两端 ping 不通时，不进入 ROS 2 排查。先检查网线、IP 地址、子网掩码、接口状态和
防火墙。

## 6. 配置 ROS 2 环境

在自有 NX 上执行：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/Desktop/lunar_-slam/install/setup.bash

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

SLAM NX 侧也应确认使用相同的：

```text
ROS_DOMAIN_ID=42
ROS_LOCALHOST_ONLY=0
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

两张网卡都获得 IP 后，再启动 ROS 2 节点。

## 7. 机器人断电时查看 SLAM 话题

机器人保持断电，在自有 NX 上执行：

```bash
ros2 topic list -t | grep -E '/cmd_vel|/slam/pose'
```

检查速度命令：

```bash
ros2 topic info /cmd_vel -v
ros2 topic echo --once /cmd_vel
```

检查 SLAM 位姿：

```bash
ros2 topic info /slam/pose -v
ros2 topic echo --once /slam/pose
```

先确认消息类型：

```text
/cmd_vel     geometry_msgs/msg/Twist
/slam/pose   geometry_msgs/msg/PoseWithCovarianceStamped
```

同时记录实际发布频率、QoS、`header.frame_id` 和静止时的速度值。期望静止时
`/cmd_vel` 为零速度，`/slam/pose` 的四元数为有效有限值。

## 8. 机器人断电时验证桥接转换

机器人仍保持断电，只启动测试 namespace：

```bash
ros2 launch slam_d1_bridge slam_d1_bridge.launch.py \
    namespace:=test_d1
```

该节点：

- 订阅全局 `/cmd_vel` 和 `/slam/pose`；
- 发布到 `/test_d1/command/user_command`；
- 不会发布到真实 D1 的 `/d15041873/command/user_command`；
- 不会设置 `use_sdk`；
- 不会站立或控制机器人。

另开终端查看转换结果：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/Desktop/lunar_-slam/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 topic echo /test_d1/command/user_command
```

验证以下行为：

- `linear.x` 被正确映射并限制在 `[-0.5, 0.5] m/s`；
- `angular.z` 被正确映射并限制在 `[-0.5, 0.5] rad/s`；
- `linear.y` 始终为 `0`；
- 其他 Twist 分量始终为 `0`；
- `pose.orientation` 为单位四元数；
- `/cmd_vel` 停止超过 300 ms 后，输出持续为零速度。

测试结束后用 `Ctrl+C` 停止 `test_d1` 桥节点。

## 9. 机器人上电后的实际启动

确认 SLAM 通信和测试 namespace 转换均正常后，再给机器人上电，并检查 D1：

```bash
ros2 topic info -v /d15041873/command/user_command
ros2 param get /d15041873/teleop_command use_sdk
```

确认：

- `/d15041873/command/user_command` 存在订阅端；
- 当前没有其他控制节点同时发布 `UserCommand`；
- 现场有人持遥控器或物理急停；
- SLAM 当前没有输出非零运动命令。

在启动脚本前，`ros2 topic info -v` 中的 `Publisher count` 应为 `0`。当前已验证的
`http_ros_gateway` 会发布同一个话题，必须先停止网关；开始脚本检测到已有发布者时会
拒绝启动。

然后才执行：

```bash
ros2 run slam_d1_bridge start_slam_d1_bridge.sh
```

开始脚本会依次：

```text
use_sdk=true
→ transform_up 持续约 3 秒
→ loco + 零速度
→ 启动 slam_d1_bridge
```

这是第一次会实际影响机器人的步骤。启动后先确认桥输出为零速度，再允许 SLAM 发送
极低速测试命令。

## 10. 安全限制

- 机器人断电阶段只做网络、话题和 `test_d1` 转换验证；
- 不要在机器人断电时反复执行 start 脚本；
- 不要同时运行 `http_ros_gateway` 和 `slam_d1_bridge`；
- 结束脚本检测到 `http_ros_gateway` 仍在发布时会拒绝继续，避免两个控制源竞争；
- 首次实机速度建议不超过 `0.1 m/s`，角速度不超过 `0.2 rad/s`；
- 任何人喊停时立即停止 SLAM 输出并执行结束流程；
- 软件停止不能替代物理急停。
