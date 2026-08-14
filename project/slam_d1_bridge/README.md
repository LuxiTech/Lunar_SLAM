# slam_d1_bridge：D1 有线局域网控制

`slam_d1_bridge` 把导航和网页输出的 `geometry_msgs/msg/Twist` 转换为 D1 厂家接口
`ddt_msgs/msg/UserCommand`。当前部署只允许通过 `192.168.123.0/24` 有线局域网控制，
不会通过 Wi-Fi、VPN、Docker 或其他网卡发送、发现 D1 控制数据。

## 1. 固定配置

| 项目 | 值 |
| --- | --- |
| D1 有线地址 | `192.168.123.49/24`，接口 `eth0` |
| ROS Domain | `42` |
| ROS RMW | `rmw_fastrtps_cpp` |
| D1 namespace | `d15041873` |
| 厂家命令话题 | `/d15041873/command/user_command` |
| 标准速度话题 | `/cmd_vel` |
| 网页高度命令 | `/d15041873/command/body_height` |
| 桥接高度状态 | `/d15041873/status/body_height` |
| 桥接电池状态 | `/d15041873/status/battery1`、`battery2` |
| D1 SSH 用户名 | `robot` |
| D1 SSH 密码 | `ddt` |

上面的密码按现场部署要求记录在仓库中，因此该仓库必须保持私有，不能上传到公开仓库、
公开制品或外部日志。条件允许时应改为 SSH 密钥并更换默认密码。

## 2. 为什么其他设备能控制，而本机曾经不能

其他控制设备通常只有一张 `192.168.123.x/24` 网卡，Fast DDS 的组播发现和单播数据
自然都从这张有线网卡发送。本机同时存在：

- `eno1`：`192.168.123.66/24`，连接 D1；
- Wi-Fi：连接办公网络并提供默认路由；
- `Meta` VPN/代理接口；
- Docker 等虚拟接口。

原配置只设置了 `ROS_DOMAIN_ID=42` 和 `SUBNET` 发现，没有限制 Fast DDS 使用哪张网卡。
本机的策略路由把 `239.255.0.1` DDS 组播送到 `Meta`，所以可以 `ping`、可以 SSH，
却看不到 D1 的控制订阅者。D1 本身也同时启用了 `eth0` 和 `wlan0`，启动服务还在等待
`wlan0`，进一步造成发现端点不稳定。厂家控制器启动较慢时，原来的 5 秒发现和 8 秒
服务超时还会把“正在发现”误判为“机器人离线”。

本仓库现在从三个层面解决该问题：

1. `setup_d1_lan_dds.sh` 根据到 `192.168.123.49` 的路由，自动选择当前设备自己的
   `192.168.123.x/24` 地址；
2. 生成 Fast DDS `interfaceWhiteList`，只保留回环地址和该有线地址，并关闭内置的其他
   传输接口；
3. D1 使用固定白名单 `127.0.0.1 + 192.168.123.49`，并用 systemd 禁用无线网卡。

因此其他设备重新拉取最新代码后，只要分配一个不冲突的 `192.168.123.x/24` 地址并重新
构建，就会自动生成适合该设备的 DDS 配置，不需要复制本机的 `.66` 地址，也不需要添加
临时组播路由。

Fast DDS 2.6 的白名单机制会同时约束发现流量和用户数据，配置依据见
[eProsima Interface Whitelist](https://fast-dds.docs.eprosima.com/en/2.6.x/fastdds/transport/whitelist.html)。

## 3. 网络拓扑和多设备规则

```text
控制机 A  192.168.123.50/24 ─┐
控制机 B  192.168.123.51/24 ─┼─ 有线交换机 ─ D1 eth0 192.168.123.49/24
控制机 C  192.168.123.66/24 ─┘                D1 wlan0：禁用
```

每台设备必须使用唯一地址。有线控制接口不要设置默认网关；设备可以继续使用自己的
Wi-Fi 上网，但 D1 ROS 2 进程不会使用该 Wi-Fi。

多台设备可以安装并启动网页，但同一时间只能有一台设备取得运动控制权：

- 启动脚本要求厂家话题已有 D1 订阅者且没有其他发布者；
- 已有 `slam_d1_bridge` 或 `http_ros_gateway` 发布时，新实例会拒绝启动；
- 同一网页服务上的每个浏览器都有独立控制租约；一个页面正在持续发运动命令时，旧标签页
  的失焦/关闭停车请求不会再覆盖它，第二个页面会收到“另一浏览器正在控制”；当前页面
  停止或 0.8 秒命令超时后，其他设备可以正常接管；
- 接管前应先在原控制机网页关闭“机器人控制”，确认机器人趴下、SDK 已释放；
- 不允许在两台设备上同时点击开启。DDS 发布者检查可避免正常情况下的重复控制，但不能
  替代现场操作协调和物理急停。

## 4. 首次配置 D1：只允许有线控制

以下操作只需执行一次。执行前确认 D1 已趴下，并保持有线 SSH 可用。

在控制机仓库根目录运行：

```bash
scp project/slam_d1_bridge/config/fastdds_d1_robot_lan_only.xml \
  robot@192.168.123.49:/tmp/
scp project/slam_d1_bridge/config/d1_robot_ros2.env \
  robot@192.168.123.49:/tmp/
scp project/slam_d1_bridge/config/d1_bringup_lan_only.conf \
  robot@192.168.123.49:/tmp/
scp project/slam_d1_bridge/config/d1_disable_wifi.service \
  robot@192.168.123.49:/tmp/
```

登录 D1 后安装配置：

```bash
ssh robot@192.168.123.49

sudo install -o robot -g robot -m 0644 \
  /tmp/fastdds_d1_robot_lan_only.xml \
  /opt/d1_ros2/fastdds_d1_robot_lan_only.xml
sudo install -o robot -g robot -m 0644 \
  /tmp/d1_robot_ros2.env /opt/d1_ros2/ros2.env
sudo install -o root -g root -m 0644 \
  /tmp/d1_bringup_lan_only.conf \
  /etc/systemd/system/d1_bringup.service.d/network-wait.conf
sudo install -o root -g root -m 0644 \
  /tmp/d1_disable_wifi.service /etc/systemd/system/d1-disable-wifi.service

sudo systemctl daemon-reload
sudo systemctl enable --now d1-disable-wifi.service
sudo systemctl restart d1_bringup.service
```

验证：

```bash
rfkill list
ip -4 -o addr show eth0
systemctl is-active d1-disable-wifi.service d1_bringup.service
grep -E '^(ROS_DOMAIN_ID|RMW_IMPLEMENTATION|FASTRTPS_DEFAULT_PROFILES_FILE)=' \
  /opt/d1_ros2/ros2.env
```

期望 Wi-Fi 显示 `Soft blocked: yes`，`eth0` 为 `192.168.123.49/24`，两个服务均为
`active`。如需恢复 D1 Wi-Fi，只能通过有线 SSH 执行：

```bash
sudo systemctl disable --now d1-disable-wifi.service
```

## 5. 在任意控制设备上部署

先给有线网卡设置一个没有冲突的地址，例如 `192.168.123.50/24`。不要照抄已经被其他
设备使用的地址。

```bash
cd /path/to/lunar_slam
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --base-paths project 3parts/D1-ROS2-SDK-Demo/ddt_msgs \
  --packages-up-to slam_d1_bridge luxi_web_control
source install/setup.bash
```

检查自动选择结果：

```bash
source install/slam_d1_bridge/lib/slam_d1_bridge/setup_d1_lan_dds.sh
echo "${D1_LAN_INTERFACE} ${D1_LAN_ADDRESS}"
echo "${FASTRTPS_DEFAULT_PROFILES_FILE}"
```

输出地址必须属于 `192.168.123.0/24`。脚本发现路由走 Wi-Fi、VPN 或其他网段时会直接
拒绝控制，而不是退回不安全的自动网卡选择。

## 6. 网页控制

推荐从仓库根目录运行一键脚本：

```bash
./scripts/start_d1_web_control.sh
```

它会自动加载有线 DDS 配置、检查 D1 唯一订阅端、启动网页和速度仲裁，并按安全流程
执行 `use_sdk=true -> transform_up -> loco`。脚本显示的 URL 使用当前设备实际的
`192.168.123.x` 地址。

如果只想先启动网页、由页面上的“机器人控制”开关决定何时站立：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
source install/slam_d1_bridge/lib/slam_d1_bridge/setup_d1_lan_dds.sh
export ROS_DOMAIN_ID=42

ros2 launch luxi_web_control lekiwi_web_control.launch.py \
  "bind_address:=${D1_LAN_ADDRESS}" http_port:=8080
```

打开 `http://<本机192.168.123.x>:8080`。开启控制前必须清空场地并由现场人员持物理
急停；关闭开关会先清零、停止桥、执行 `transform_down`，最后恢复 `use_sdk=false`。
D1 专用 launch 默认只监听 `127.0.0.1`；一键脚本可以显式绑定本机
`192.168.123.x` 地址。需要让操作端通过局域网访问时，可传入 `bind_address:=0.0.0.0`：
程序会分别监听本机实际的 D1 有线地址 `192.168.123.x` 和操作端局域网地址
`192.168.137.x`（例如本机的 `192.168.123.66` 与 `192.168.137.132`），而不监听其他
网卡。显式传入其他网段地址仍会被拒绝；如果本机没有任一允许网段，通配地址会明确
报错。

为避免手机网页预览抢占遥控心跳，D1 默认只在遥控盘上方回传压缩 RGB 图像，不订阅、
不显示也不通过 HTTP 返回建图实时点云；RTAB-Map 内部建图和地图保存不受影响。手动运动
期间，网页会取消体素、地形和保存点云等大响应，服务端也会拒绝其他浏览器拉取这些数据，
松手停车后再恢复地图显示。控制请求超过 250 ms 会放弃等待并在下一个 100 ms 心跳重试，
服务端看门狗为 0.8 秒；松手、页面真正隐藏、急停仍立即停车。升级后需要在手机浏览器中
强制刷新一次页面，确保加载新的 JavaScript 和布局。

网页的“机器人状态”栏目在桥启动后显示两组电池反馈。顶部百分比取在线电池包中的较低
值，避免一组电池偏低时被平均值掩盖；下方保留每组电池的百分比和电压。D1 驱动现场
版本的 `percentage` 使用 `0..100`，网页也兼容标准 ROS `0..1` 表示。超过 3 秒没有新
消息或桥断开时显示“电量离线”，不会继续展示过期数值。

腿部高度滑动条针对现场 `controller_mode=biped` 的 D1 单体双足 LQR。控制
有效刻度为 `0..9` 档，网页归一化显示为 `0..100%`；新的 100% 对应旧
`0..30` 刻度的 30% 位置。它是可重复的时间积分档位，
不是米制高度传感器的读数。每变化 1 档，桥以 `linear.z=0.03` 持续约 1 秒；
`0 -> 9` 对应约 9 秒抬升，`9 -> 0` 对应约 9 秒降低。
网页只在控制器明确返回 `biped` 时开放滑块；不会自动切换四足形态或触发
`rl_4`，避免误触发对接/形态转换。

网页显示的是桥的**单体双足高度百分比指令**，不是额外高度传感器的测量值。机器人未站立、
SDK 未开启、桥未就绪或不是双足 LQR 模式时，请求都会被服务端拒绝。

### 建图后无法自动定位的判定

自动定位不是仅凭地图文件存在就判定成功。HLoc 必须先通过图像检索、局部特征、PnP 和深度
一致性检查，随后还要连续得到 3 个一致位姿并通过 ICP；任一步失败都保持零速度。网页现在
直接显示 HLoc 原因、匹配数和候选参考帧。常见状态：

- `MATCHES_LOW`：画面主要是无纹理地面、白墙，或者拍摄角度变化太大；
- `LANDMARKS_LOW`：匹配特征缺少有效深度；
- `DEPTH_RESIDUAL_HIGH`：当前几何结构与候选位置不一致；
- `RETRIEVAL_SCORE_LOW`：当前位置不在地图覆盖范围或视野被遮挡。

出现这些状态时不要降低安全门限后直接导航。建图时应缓慢运动，让相邻关键帧重叠，并让
门框、墙角、箱体等稳定且有区分度的物体进入画面；避免整段只拍反光地面。建图结束前回看
起点或其他已走过的特征区域形成闭环。定位时先停止平移，在安全空间缓慢转动相机/机器人，
直到网页依次显示 HLoc 粗定位、ICP 精定位。外观相似但实际位置不同的门或走廊必须依靠局部
几何和深度消歧，不能只用相似图检索结果作为位姿。

### 2026-08-14 实机高度验证

先前用 `UserCommand.pose.position.z` 下发的测试只有 `0.00098 rad` 关节抖动，该字段
不是双足 LQR 高度接口。解析当前机器实际运行的 `librl_controller.so` 和
`libtita_mcu_controller.so` 后确认：`loco` 将 `Twist.linear.z` 以增量模式交给 LQR，
底层执行 `height += linear.z * dt` 并应用内部限位。

正确轴实测使用零平面速度、`linear.z=±0.03`，并用机器实时 TF 取四条链
`base_link -> *_foot` 的变换，计算值为
`mean(-T_base_to_foot.translation.z)`。得到的运动学参考距离从 `0.2901 m` 变为
`0.4595 m`，差值 `0.1689 m`。这个数字依赖 URDF、关节反馈和轮心坐标，不是地面到
机身的独立实测传感器高度，因此不再用它作为网页刻度。测试后确认
`inactive / prone / SDK=false`。

## 7. 桥接行为和安全限制

- `linear.x` 和 `angular.z` 被转换到厂家 `UserCommand.twist`；
- 两个轴默认限制为绝对值 `0.5`；网页现场配置进一步限制为 `0.10 m/s`；
- `linear.y` 和其他不支持的轴强制归零；
- 非有限输入被拒绝；
- 输入超过 300 ms 未更新时持续发布零速度；
- 高度滑块是 `0..9` 的单体双足目标档位，该上限对应旧范围的 30%；
  桥将误差转成 `UserCommand.twist.linear.z`
  速率并连续发送，双足 LQR 按持续时间积分；
- 档位变化率为 `1 档/s`，经实机验证的 `linear.z` 幅值为 `0.03`；
- 腿高调节全程保持 `loco`，不激活 `rl_4`，不切换机器人形态；
- 厂家 `battery1`、`battery2` 只由桥转发到稳定的 `/status/battery*` 接口，网页不直接
  依赖厂家内部话题；
- 桥启动时会拒绝与其他厂家命令发布者并行运行；
- 桥本身不会私自取得 SDK 权限，站立和趴下由启停脚本管理。

首次受监护短距离验收建议使用 `0.05 m/s` 持续 2 秒，名义位移约 10 cm；随后立即
停止并检查 300 ms 超时归零。不要在斜坡、人员附近或没有物理急停时测试。

## 8. 常用检查

```bash
ping -c 3 192.168.123.49

ros2 topic info --no-daemon --spin-time 30 \
  /d15041873/command/user_command --verbose

ros2 service call /d15041873/command/get_controller_status \
  std_srvs/srv/Trigger '{}'

ros2 topic echo --once /d15041873/status/battery1
ros2 topic echo --once /d15041873/status/body_height
```

启动桥前，厂家命令话题应为 `Publisher count: 0`、`Subscription count: 1`。控制正常
关闭后应看到 D1 `fsm_state=idle`、`posture=prone`、`sdk_active=false`。

如只做不接触实机的接口回归，可使用独立 ROS Domain 启动桥，向厂家电池输入话题发布
模拟 `BatteryState`，再验证 `/status/battery*`、`/status/body_height` 和
`UserCommand.twist.linear.z`。仓库测试同时覆盖高度输入类型/范围、两种电量百分比
格式、限幅、变化率、停车消息保持高度以及网页 HTTP 拒绝条件。

## 9. 停止和故障恢复

正常停止：

```bash
ros2 run slam_d1_bridge stop_slam_d1_bridge.sh
```

该脚本先停止桥、发布零速度、持续发送 `transform_down`，再释放 SDK。任何无法确认的
状态都应先使用物理急停；不要通过启动第二台控制机来“抢占”失控的发布者。

脚本默认等待 DDS 发现 30 秒、服务响应 30 秒，网页允许整个启停流程使用 90 秒。现场
网络更慢时可临时覆盖脚本参数：

```bash
export D1_DISCOVERY_SPIN_TIME=45.0
export D1_SERVICE_TIMEOUT=45s
```
