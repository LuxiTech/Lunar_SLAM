# slam_d1_bridge

`slam_d1_bridge` 将网页或导航输出的标准 `/cmd_vel` 转换为 D1 的
`ddt_msgs/msg/UserCommand`。日常使用请通过仓库的一键脚本启动，它会完成 D1 连通性
检查、SDK 控制、站立、速度桥和网页服务。

## 当前设备

| 设备 | `ROBOT_NS` | D1 有线地址 | 相机配置 |
|---|---|---|---|
| 第一台 D1 | `d15041873` | `192.168.123.49` | 默认 USB 双目配置 |
| 第二台 D1（当前） | `d15042176` | `192.168.123.49` | `stereo_camera_d15042176.yaml` |

两台 D1 使用相同的固定有线地址，一次只能直连一台。程序不会按 IP 或 DDS 自动选择
机器人，必须显式传入正确的 `ROBOT_NS`，避免控制错设备。

## 一键启动当前 D1

执行以下命令会让机器人站立。开始前将 D1 放在平地、清空周围区域，并由现场人员持有
遥控器或物理急停：

```bash
cd /home/nvidia/Desktop/lunar_-slam
source /opt/ros/humble/setup.bash
source install/setup.bash

./scripts/start_d1_web_control.sh \
  --robot-ns d15042176 \
  --robot-ip 192.168.123.49
```

确认提示后，当前 NX 从局域网访问：

```text
http://192.168.0.187:8080
```

`192.168.0.187` 是当前 NX 的 Wi-Fi 地址，可能随网络变化；以启动脚本最后打印的 URL
为准。`192.168.123.51` 是 NX 到 D1 的隔离控制网口，通常不是手机或电脑的网页入口。

网页建图顺序：

1. 确认 USB 双目和 H30 已连接，没有其他进程占用相机。
2. 在“深度链路”中选择 CREStereo；需要最高帧率时选择 CREStereo 极致，VPI 为备用。
3. 点击“开始建图”，确认 RGB、深度/点云和里程计持续更新后再缓慢移动 D1。
4. 完成后点击“停止建图”，等待 RTAB-Map 数据库保存结束。

选择 `d15042176` 后，网页会自动加载该机器人对应的 USB 相机 profile。CREStereo 默认
使用 H30；H30 无串口数据、六轴异常或四元数无效时应修复传感器，不能用
`use_imu:=false` 绕过后进行机器狗动态建图。

## 安全停止

优先在网页中停止行驶和建图，再关闭“机器人控制”。若需要从终端让当前 D1 趴下并释放
SDK，必须继续携带同一个 namespace：

```bash
ROBOT_NS=d15042176 \
  ros2 run slam_d1_bridge stop_slam_d1_bridge.sh
```

停止脚本依次停止桥、持续发送零速度、执行 `transform_down`，最后恢复
`use_sdk=false`。不要关闭终端来代替安全停止。

## 启动过程与门禁

一键脚本会执行：

```text
有线 ping 与 DDS 订阅检查
  -> use_sdk=true
  -> transform_up
  -> loco + 零速度
  -> slam_d1_bridge
  -> 网页 /cmd_vel
```

控制链为：

```text
网页手动 /d1/cmd_vel_standard ─┐
                               ├─ velocity mux ─ /cmd_vel
导航      /navigation/cmd_vel ─┘                    │
                                                    ▼
                                           slam_d1_bridge
                                                    │
                                                    ▼
                         /d15042176/command/user_command
```

桥只使用 `linear.x` 和 `angular.z`，超限值会被裁剪；未支持的横向速度被清零。输入超过
300 ms 未更新、出现 NaN/Inf 或上游停止时，桥会持续输出零速度。`http_ros_gateway`
不能与本桥同时运行，否则两个节点会竞争 D1 命令话题。

当能发现目标 namespace、但 `/command/user_command` 没有订阅者时，一键脚本只会在
远端确认 namespace 一致且 FSM 为 `idle` 后尝试重建 `d1_bringup` 的 DDS participant。
该恢复不发送运动命令，并要求 NX 已配置免密 SSH：

```bash
ssh -o BatchMode=yes robot@192.168.123.49 true
```

维护时可加 `--no-dds-recovery` 禁用自动恢复。`--motion-test` 会产生低速实机运动，只能
在现场受监护验收时使用。

## 常见检查

```bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

ping -c 3 192.168.123.49
ros2 topic info -v /d15042176/command/user_command
ros2 node info /d15042176/teleop_command
curl -s http://127.0.0.1:8080/api/status
```

启动桥前，厂家命令话题应为 `Subscription count: 1`、`Publisher count: 0`。常见问题：

- 能 ping 但发现不到 namespace：检查 D1 的 `ROS_DOMAIN_ID=42`、Fast DDS 和
  `d1_bringup.service`。
- 能发现 `d15042176` 但没有命令订阅者：确认免密 SSH；保持 D1 趴下后重新运行一键
  脚本，让安全 DDS 恢复流程处理。
- 网页提示目标 namespace 不一致：8080 上已有另一台机器人的网页进程，先安全关闭
  原实例再切换。
- 网页可开但相机不启动：检查所选 namespace 是否为 `d15042176`、USB 占用和
  `log/luxi_web_control_rtabmap.log`。
- 桥启动日志：`/tmp/slam_d1_bridge_d15042176.log`。
- 网页启动日志：`log/d1_web_control.log`。

## 构建

```bash
cd /home/nvidia/Desktop/lunar_-slam
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --base-paths project 3parts/D1-ROS2-SDK-Demo/ddt_msgs \
  --packages-up-to slam_d1_bridge luxi_web_control
source install/setup.bash
```

只启动底层桥（不启动网页）时：

```bash
export ROBOT_NS=d15042176
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export SLAM_D1_WORKSPACE=/home/nvidia/Desktop/lunar_-slam

ros2 run slam_d1_bridge start_slam_d1_bridge.sh
```

## 参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `input_topic` | `cmd_vel` | 速度输入，launch 映射到全局 `/cmd_vel` |
| `slam_pose_topic` | `slam/pose` | SLAM 位姿输入；当前仅缓存校验，不参与运动控制 |
| `output_topic` | `command/user_command` | namespace 下的 D1 命令输出 |
| `publish_period_ms` | `50` | 20 Hz 输出周期 |
| `command_timeout_ms` | `300` | 速度命令超时后归零 |
| `max_linear_x` | `0.5` | 最大前后速度，m/s |
| `max_angular_z` | `0.5` | 最大转向角速度，rad/s |
| `lateral_velocity_tolerance` | `0.001` | 不支持的 `linear.y` 告警阈值 |
| `fsm_mode` | `loco` | 正常桥接时的 D1 FSM 字段 |

更完整的网络部署、首次运动验收和故障恢复见
[D1 机器人控制接入与测试](../../docs/d1_robot_control.md)。

## 网页电池与机身高度控制

合并后的桥会把厂家 `battery1`、`battery2` 转发到当前机器人 namespace 下的
`status/battery1`、`status/battery2`，网页显示两组电池并以较低电量作为总览。

双足 LQR 模式下，网页高度滑块使用 `0..9` 档；9 档对应旧 `0..30` 范围的 30%。桥将
档位误差转换为零平面速度下的 `linear.z=±0.03`，变化率限制为每秒 1 档。机器人未站立、
SDK/桥未就绪或 `controller_mode` 不是 `biped` 时，服务端会拒绝高度命令。该值是控制
档位，不是独立高度传感器的米制读数。

相关话题会自动跟随 `ROBOT_NS`，例如第二台机器人使用：

```text
/d15042176/command/body_height
/d15042176/status/body_height
/d15042176/status/battery1
/d15042176/status/battery2
```
