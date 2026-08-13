# D1 机器人控制接入与测试

## 1. 已确认接口

本方案以 `d1/` 中的交付资料和厂家官方
`DDTRobot/D1-ROS2-SDK-Demo` 为准。当前设备参数为：

| 项目 | 值 |
| --- | --- |
| D1 有线 IP | `192.168.123.49/24` |
| 控制机有线 IP | `192.168.123.51/24` |
| ROS Domain ID | `42` |
| RMW | `rmw_fastrtps_cpp` |
| 机器人 namespace | `d15041873` |
| 厂家控制输入 | `/d15041873/command/user_command` |
| 消息类型 | `ddt_msgs/msg/UserCommand` |

厂家消息的有效字段为 `fsm_mode`、`pose` 和 `twist`。本工程只将 `/cmd_vel` 的
`linear.x` 与 `angular.z` 映射到 `UserCommand.twist`，不把 SLAM 地图位姿写入机身
姿态字段。

控制链如下：

```text
网页手动 /d1/cmd_vel_standard ─┐
                               ├─ C++ velocity mux ─ /cmd_vel
导航      /navigation/cmd_vel ─┘                         │
                                                         ▼
                                             C++ slam_d1_bridge
                                                         │
                                                         ▼
                                /d15041873/command/user_command
                                                         │
                                                         ▼
                                           D1 teleop_command
```

厂家示例定义 `linear.x > 0` 为前进，`angular.z > 0` 为左转。D1 链路保持 ROS 标准
方向，不反转 `angular.z`。

## 2. 源码与构建

厂家桥已适配到：

```text
project/slam_d1_bridge
```

缺失的厂家消息包保存在：

```text
3parts/D1-ROS2-SDK-Demo/ddt_msgs
```

当前使用的官方 SDK 提交为
`ffe6281dbebcfb732ea3413c76ef35877b6a06fe`。构建时显式加入第三方消息路径，避免
扫描 `3parts` 下其他大型依赖：

```bash
cd /home/nvidia/Desktop/lunar_slam
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --base-paths project 3parts/D1-ROS2-SDK-Demo/ddt_msgs \
  --packages-up-to slam_d1_bridge luxi_web_control
source install/setup.bash
ros2 interface show ddt_msgs/msg/UserCommand
```

## 3. 只读连接检查

```bash
ip route get 192.168.123.49
ping -c 3 192.168.123.49

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 topic info -v /d15041873/command/user_command
ros2 node info /d15041873/teleop_command
```

启动桥前的期望结果是 D1 命令话题有一个 `teleop_command` 订阅者且没有发布者。
出现其他发布者时，不得再启动桥；尤其不能同时运行 `http_ros_gateway`。

## 4. 隔离转换测试

先使用测试 namespace，避免接触真实命令话题：

```bash
export ROS_DOMAIN_ID=142
ros2 launch slam_d1_bridge slam_d1_bridge.launch.py namespace:=test_d1
```

另一个终端观察：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/Desktop/lunar_slam/install/setup.bash
export ROS_DOMAIN_ID=142
ros2 topic echo /test_d1/command/user_command
```

验证规则：

- `linear.x` 限制到 `[-0.5, 0.5] m/s`；
- `angular.z` 限制到 `[-0.5, 0.5] rad/s`；
- `linear.y` 和其他未支持分量强制为零；
- NaN 或无穷输入使输出归零；
- `/cmd_vel` 超过 300 ms 未更新后持续发布零速度；
- 输出频率为 20 Hz。

## 5. 网页与导航输入

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch luxi_web_control lekiwi_web_control.launch.py \
  bind_address:=0.0.0.0 http_port:=8080
```

启动文件名为历史兼容名称，当前内部手动话题已是 `/d1/cmd_vel_standard`，最终仍输出
`/cmd_vel`。网页空闲、停止、急停或命令超时时，速度 mux 持续输出零速度。

## 6. 实机控制流程

以下流程会让机器人站起或趴下，只能在场地清空、有人持遥控器或物理急停时执行。
启动前必须确认 `/cmd_vel` 为零，并关闭厂家键盘 Demo 和 `http_ros_gateway`。

网页右侧“机器人控制”开关是日常入口：

- 开启：先锁定运动输出，再执行 `use_sdk=true → transform_up → loco`，随后启动 C++
  `/cmd_vel` 桥；
- 关闭：先清零并停止导航，再停止桥、执行 `transform_down`，最后恢复
  `use_sdk=false`；
- 切换期间开关和非零速度命令均被锁定；页面刷新不会自动改变机器人状态；
- 开启约需 10 秒，关闭约需 15 秒，必须等待状态显示完成。

推荐使用一键脚本同时启动网页、开放 SDK 控制并启动 D1 桥：

```bash
/home/nvidia/Desktop/lunar_slam/scripts/start_d1_web_control.sh
```

首次受监护联调可附加 `--motion-test`，依次执行 1 秒、最大名义位移 5 cm 的前后运动和
1 秒的左右低速转动：

```bash
/home/nvidia/Desktop/lunar_slam/scripts/start_d1_web_control.sh --motion-test
```

脚本异常时会先发送网页停止命令，再调用 D1 停止脚本让机器人趴下。正常完成后网页和
D1 桥保持运行，可继续通过网页控制。

也可以手工执行底层启动命令：

```bash
export ROBOT_NS=d15041873
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export SLAM_D1_WORKSPACE=/home/nvidia/Desktop/lunar_slam

ros2 run slam_d1_bridge start_slam_d1_bridge.sh
```

脚本要求人工确认，然后按厂家流程执行：

```text
teleop_command.use_sdk=true
→ transform_up（约 3 秒）
→ loco + 零速度
→ 启动 C++ bridge（20 Hz）
```

停止时必须使用：

```bash
ros2 run slam_d1_bridge stop_slam_d1_bridge.sh
```

停止脚本先结束桥、发送零速度，再执行 `transform_down` 并恢复 `use_sdk=false`。

## 7. 首次非零运动验收

软件零速度测试不能证明转向和制动的实车动力学已经验收。首次非零测试必须人工监护，
建议依次验证：

1. `+0.05 m/s` 前进不超过 1 秒，然后停止；
2. `-0.05 m/s` 后退不超过 1 秒，然后停止；
3. `+0.10 rad/s` 左转不超过 1 秒，然后停止；
4. `-0.10 rad/s` 右转不超过 1 秒，然后停止；
5. 非零输入时停止上游，确认 300 ms 后桥输出归零；
6. 网页急停，确认 `/cmd_vel` 与 `UserCommand.twist` 同时归零。

未经现场安全确认，不执行上述非零测试。厂家明确说明机器人会保持最后一条命令，因此
持续零速度输出、300 ms 桥超时和物理急停缺一不可。
