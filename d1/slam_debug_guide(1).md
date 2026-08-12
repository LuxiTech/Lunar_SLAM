# SLAM-D1 有线以太网联调指南

## 1. 适用范围

本文用于以下联调方案：

- `slam_d1_bridge` 部署在 SLAM NX 上；
- SLAM NX 通过以太网直连 D1 的千兆网口；
- SLAM 发布最终速度话题 `/cmd_vel`；
- 桥接节点将 `/cmd_vel` 转换为 D1 的
  `/d15041873/command/user_command`；
- 机器人实际 namespace 为 `d15041873`，ROS Domain ID 为 `42`；
- D1 `eth0` 使用静态地址 `192.168.123.49/24`。

桥接节点不需要知道底层是 Wi-Fi 还是以太网，只要 ROS 2 能够通过有线网卡发现 D1
即可。D1 的 Wi-Fi 继续用于默认路由和日常维护；Type-C 仅保留为应急维护通道，
不作为本次 SLAM 数据链路。

第一期只使用速度命令。SLAM pose 订阅是预留接口，pose 暂时没有发布或暂时不可用，
不影响速度转换和桥接运行。

## 2. 网络拓扑

```text
SLAM NX（运行 slam_d1_bridge）
有线网卡：192.168.123.<未占用地址>/24
        │
        │ 千兆以太网直连
        │
D1 eth0：192.168.123.49/24
D1 wlan0：192.168.0.61/24（默认路由/维护）
```

有线联调链路使用 `192.168.123.0/24`。SLAM NX 的有线地址必须与机器人 `.49` 同网段。
当前自有 NX 的测试地址是 `192.168.123.50/24`；如果 SLAM NX 与自有 NX 同时接入该
网段，SLAM NX 必须选择另一个未占用地址，不能复用 `.50`。

本方案不依赖以下网络作为 SLAM 控制链路：

- D1 公司 Wi-Fi 网段 `192.168.0.0/24`；
- Type-C USB 网络 `192.168.42.0/24`。

如果还需要从自有 NX 观察 SLAM 或 D1 话题，可以另行通过 Wi-Fi 或另一条以太网连接；
桥接节点本身运行在 SLAM NX 上，通过 `192.168.123.0/24` 有线链路访问 D1。

## 3. 有线连接要求

D1 端已经使用 `systemd-networkd` 持久化配置：

```text
/etc/systemd/network/05-d1-ethernet-123.network
```

其核心配置为：

```ini
[Match]
Name=eth0

[Link]
RequiredForOnline=no

[Network]
DHCP=no
Address=192.168.123.49/24
IPv6AcceptRA=no
```

该配置不设置网关和 DNS，因此 D1 默认路由仍走 Wi-Fi。机器人以太网地址重启后保持为
`192.168.123.49/24`。

SLAM NX 侧使用一个未占用的 `192.168.123.x/24` 地址，并且不设置默认网关。例如在
没有其他设备使用 `.50` 时：

```text
SLAM NX：192.168.123.50/24
D1：    192.168.123.49/24
```

如果 `192.168.123.50` 已被自有 NX 或其他设备使用，必须为 SLAM NX 选择另一个地址，
避免 IP 冲突。IP 冲突会导致 ARP/DDS 发现不稳定。

## 4. 连接检查

### 4.1 SLAM NX 检查有线网卡

在 SLAM NX 上执行：

```bash
ip -brief address
ip route get 192.168.123.49
```

确认以太网接口处于 `UP`，有 `192.168.123.x/24` 地址，并且到 `192.168.123.49` 的
路由走有线网卡。接口名可能是 `eth0`、`eno1`、`enP...` 或其他名称，不要把接口名
写死。

如果 SLAM NX 没有地址，可临时配置（将 `<SLAM_IF>` 替换为实际接口）：

```bash
sudo ip link set dev <SLAM_IF> up
sudo ip addr replace 192.168.123.50/24 dev <SLAM_IF>
```

持久化方式应使用 SLAM NX 自己的 NetworkManager、systemd-networkd 或发行版网络配置，
不要修改 D1 端的配置文件。

### 4.2 测试 D1 有线链路

在 SLAM NX 上执行：

```bash
ping -c 3 192.168.123.49
ip neigh show 192.168.123.49
```

如果 D1 的 SSH 服务监听以太网，也可以测试：

```bash
ssh robot@192.168.123.49
```

日常维护仍可通过 D1 Wi-Fi 地址（当前观测为 `192.168.0.61`）或 Type-C 地址
`192.168.42.1` 登录；这些地址不属于本次 SLAM 控制链路。SSH 密码使用现场交付的
凭据，不写入本文档。

如果没有地址或 ping 不通，依次检查：

- 以太网线是否插在 D1 的千兆网口和 SLAM NX 的有线网口；
- D1 `eth0` 是否为 `UP/RUNNING`，并具有 `192.168.123.49/24`；
- SLAM NX 是否使用了同网段且不冲突的地址；
- 两端是否被其他网络服务改回 DHCP 或自动地址；
- D1 是否已经上电；
- 是否有其他网络接口把到 `192.168.123.49` 的路由抢走。

## 5. D1 端 ROS 2 配置

### 5.1 必须使用跨机通信设置

在 D1 上，运行中的 `d1_bringup.service` 必须使用：

```text
ROS_DOMAIN_ID=42
ROS_LOCALHOST_ONLY=0
```

手册中用于机器人本机查看话题的示例包含 `ROS_LOCALHOST_ONLY=1`。该设置会限制
ROS 2 只使用本机回环接口，外部 SLAM NX 通过以太网时不能使用它。

通过 Wi-Fi 或维护通道登录 D1 后检查配置文件：

```bash
grep -E '^(ROS_DOMAIN_ID|ROS_LOCALHOST_ONLY)=' /opt/d1_ros2/ros2.env
```

不要只查看 SSH 终端自己的 `printenv`，还要确认启动控制器的 systemd 服务实际继承
的环境：

```bash
pid=$(systemctl show d1_bringup.service -p MainPID --value)
sudo sh -c "
tr '\0' '\n' < /proc/${pid}/environ |
grep -E '^(ROS_DOMAIN_ID|ROS_LOCALHOST_ONLY|RMW_IMPLEMENTATION)'
"
```

如果服务仍是 `ROS_LOCALHOST_ONLY=1`，应在机器人趴下、停止运动并有人持急停的条件下：

1. 备份 `/opt/d1_ros2/ros2.env`；
2. 将 `ROS_LOCALHOST_ONLY` 改为 `0`；
3. 重启 `d1_bringup.service`；
4. 重新检查服务环境和 ROS 话题。

### 5.2 确认机器人实际 namespace

```bash
source /opt/ros/humble/setup.bash
source /opt/d1_ros2/setup.bash
source /opt/d1_ros2/namespace.sh
echo "ROBOT_NS=${ROBOT_NS}"
```

本文和桥接脚本使用的实际值是：

```text
ROBOT_NS=d15041873
```

不要使用早期资料中的 `d13007137`。

## 6. SLAM NX 的 ROS 2 环境

在运行桥接节点的终端中执行：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

如果 SLAM NX 工作空间路径不同，将第二行改为实际路径。不要保留由 SLAM 环境预先
设置的 `ROS_LOCALHOST_ONLY=1`。

为避免 ROS 2 CLI 使用旧的发现缓存，执行：

```bash
ros2 daemon stop
ros2 daemon start
```

如果 SLAM NX 同时连接了其他 Wi-Fi 或网络接口，先检查：

```bash
ip route get 192.168.123.49
```

首次联调可以临时关闭无关的网络接口，确保 DDS 通过有线网卡发现 D1。不要在没有验证
前修改 Fast DDS 配置或替换 RMW 实现。

## 7. 部署和编译桥接节点

### 7.1 复制源码

将以下源码包复制到 SLAM NX 的 ROS 2 工作空间：

```text
/home/nvidia/ros2_ws/src/slam_d1_bridge
```

复制可以使用已有的 Wi-Fi、以太网、U 盘或其他文件传输方式。不要复制 `build`、
`install` 和 `log` 目录。

### 7.2 确认 ddt_msgs

桥接节点依赖 D1 的消息包 `ddt_msgs`。在 SLAM NX 上执行：

```bash
source /opt/ros/humble/setup.bash
ros2 interface show ddt_msgs/msg/UserCommand
```

如果找不到该接口，需要把 D1 SDK 中的 `ddt_msgs` 源码包一并复制到工作空间，或先
加载已经安装该消息包的工作空间。

### 7.3 编译

```bash
source /opt/ros/humble/setup.bash
cd /home/nvidia/ros2_ws

colcon build --symlink-install \
    --packages-select slam_d1_bridge \
    --cmake-args -DBUILD_TESTING=OFF

source install/setup.bash
```

如果 `ddt_msgs` 也放在当前工作空间源码中，可以使用：

```bash
colcon build --symlink-install \
    --packages-up-to slam_d1_bridge \
    --cmake-args -DBUILD_TESTING=OFF
```

## 8. 编译后只读验证

### 8.1 验证 SLAM 速度输入

桥接节点第一期订阅最终速度话题：

```text
/cmd_vel  geometry_msgs/msg/Twist
```

在 SLAM NX 上执行：

```bash
ros2 topic info -v /cmd_vel
ros2 topic echo --once /cmd_vel
```

确认 SLAM 或其速度 mux 确实发布 `/cmd_vel`。`/navigation/cmd_vel` 和
`/lekiwi/cmd_vel_standard` 是其他上游话题，第一期桥接节点不同时订阅它们。

### 8.2 验证 D1 话题发现

```bash
ros2 topic info -v /d15041873/command/user_command
```

成功标准：

- 能解析消息类型 `ddt_msgs/msg/UserCommand`；
- 能看到 D1 的 `teleop_command` 订阅者；
- 在桥接节点启动前，`Publisher count` 为 `0`；
- 没有其他节点（尤其是 `http_ros_gateway`）发布该话题。

还可以读取只读状态确认跨机 ROS 2 通信：

```bash
ros2 topic echo --once /d15041873/joint_states
ros2 topic echo --once /d15041873/imu_sensor_broadcaster/imu
```

### 8.3 SLAM pose（可选）

```bash
ros2 topic info -v /slam/pose
ros2 topic echo --once /slam/pose
```

当前没有 `/slam/pose` 不会阻止桥接节点运行，也不会停止速度输出。该数据仅缓存并
为后续闭环使用预留，第一期不参与运动控制。

## 9. 机器人断电时的转换测试

机器人断电或 D1 ROS 话题不可见时，不能运行实际启动脚本。可以在 SLAM NX 上使用
测试 namespace 验证转换逻辑：

```bash
ros2 launch slam_d1_bridge slam_d1_bridge.launch.py \
    namespace:=test_d1
```

另开终端：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 topic echo /test_d1/command/user_command
```

该测试只发布 `/test_d1/command/user_command`，不会访问真实的
`/d15041873/command/user_command`，不会设置 SDK 模式，也不会使机器人站立。

验证以下转换行为：

- `linear.x` 限制在 `[-0.5, 0.5] m/s`；
- `angular.z` 限制在 `[-0.5, 0.5] rad/s`；
- `linear.y` 始终为 `0`，单机模式不使用横移；
- 其他 Twist 分量始终为 `0`；
- 输出 pose 的 `orientation.w` 为 `1.0`；
- `/cmd_vel` 超过 `300 ms` 没有新命令后，输出持续为零速度。

测试结束后使用 `Ctrl+C` 停止测试节点。

## 10. 实机启动流程

只有完成有线网络、ROS 发现、D1 订阅者和转换测试后，才进行以下步骤。

### 10.1 启动前安全检查

- D1 已上电并处于稳定、可控状态；
- 现场有人持遥控器或物理急停；
- 运动区域清空；
- 当前没有运行 `http_ros_gateway`；
- `/cmd_vel` 当前为零速度或尚未发布非零速度；
- `/d15041873/command/user_command` 当前没有其他发布者。

### 10.2 启动桥接

在 SLAM NX 上执行：

```bash
export ROBOT_NS=d15041873
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ros2 run slam_d1_bridge start_slam_d1_bridge.sh
```

如果工作空间不是 `/home/nvidia/ros2_ws`，先设置：

```bash
export SLAM_D1_WORKSPACE=/实际工作空间路径
```

开始脚本会依次执行：

```text
检查 D1 UserCommand 订阅者和现有发布者
→ use_sdk=true
→ transform_up 约 3 秒
→ loco + 零速度约 1 秒
→ 后台启动 slam_d1_bridge
```

`transform_up` 是第一次会实际影响机器人的步骤。启动后先确认桥接输出为零速度，
再允许 SLAM 发送低速测试命令。

### 10.3 停止桥接

测试结束后，在 SLAM NX 上执行：

```bash
ros2 run slam_d1_bridge stop_slam_d1_bridge.sh
```

结束脚本会停止桥接、发送零速度、执行 `transform_down`，最后释放 SDK 模式。若检测
到 `http_ros_gateway` 仍在发布控制话题，脚本会拒绝继续，需先停止网关。

## 11. 当前桥接限制

| 项目 | 当前值 |
| --- | --- |
| 输入话题 | `/cmd_vel` |
| 输出话题 | `/d15041873/command/user_command` |
| 输出频率 | 20 Hz |
| 命令超时 | 300 ms 后输出零速度 |
| 最大前后速度 | `±0.5 m/s` |
| 最大角速度 | `±0.5 rad/s` |
| 横向速度 | 固定为 `0` |
| 机身 pose 控制 | 第一阶段不使用，仅发送中性单位四元数 |
| SLAM pose | 可选订阅、缓存，不参与第一阶段运动控制 |

这些是桥接层的限幅，不会修改 SLAM 的原始命令。SLAM 团队应将其视为机器人实际可
接受的第一阶段接口限制。

首次实机测试建议进一步限制上游命令：线速度不超过 `0.1 m/s`，角速度不超过
`0.2 rad/s`。软件停止不能替代物理急停。

## 12. 故障排查

### 有线网卡没有 IP 地址

```bash
ip -brief address
ip link
ip route get 192.168.123.49
```

检查网线、两端网口、D1 电源和链路状态。D1 端检查：

```bash
ip -4 addr show dev eth0
networkctl status eth0 --no-pager
```

期望 D1 `eth0` 为 `192.168.123.49/24`。SLAM NX 端应使用同网段的未占用地址。

### 能 ping，但看不到 D1 ROS 话题

依次确认：

1. SLAM NX 和 D1 的 `ROS_DOMAIN_ID` 都是 `42`；
2. D1 的 `d1_bringup.service` 使用 `ROS_LOCALHOST_ONLY=0`；
3. SLAM NX 使用 `ROS_LOCALHOST_ONLY=0`；
4. 两端 RMW 实现兼容，当前优先使用 `rmw_fastrtps_cpp`；
5. `ros2 daemon stop` 后重新启动 ROS 2 CLI；
6. 多网卡时到 `192.168.123.49` 的路由确实走有线网卡。

### 找不到 `/d15041873/command/user_command`

确认 D1 已上电、bringup 正常运行，并使用实际 namespace `d15041873`。不要使用旧资料
中的 `d13007137`。

### 找不到 `ddt_msgs/msg/UserCommand`

在 SLAM NX 安装或编译 D1 SDK 的 `ddt_msgs` 包，然后重新 source 工作空间并重新编译
`slam_d1_bridge`。

### start 脚本拒绝启动

常见原因：

- 没有检测到 D1 的 `UserCommand` 订阅者；
- `http_ros_gateway` 或其他节点仍在发布同一控制话题；
- bridge 已经运行；
- 工作空间没有编译或 `SLAM_D1_WORKSPACE` 路径错误。

先读取脚本提示，不要用第二个控制节点绕过检查。

## 13. 参考资料

- `/home/nvidia/wangzheng/D1-Development-Manual/pages/Quick_Start.md`
- `/home/nvidia/ros2_ws/src/slam_d1_bridge/README.md`
- `/home/nvidia/ros2_ws/src/slam_d1_bridge/docs/概要设计.md`
- `/home/nvidia/ros2_ws/src/http_ros_gateway/docs/robot_modifications.md`
