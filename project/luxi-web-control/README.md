# Luxi Web Control

一个与小车型号无关的 ROS 2 网页遥控节点。浏览器通过同源 HTTP API 发送运动目标，
节点限幅并发布标准 `geometry_msgs/msg/Twist`，默认话题为 `/cmd_vel`。不依赖
rosbridge、前端框架或厂商 SDK。它还可管理本项目 RTAB-Map 建图 launch 的启动和
停止，方便在同一网页中完成“开始建图 → 遥控采集 → 停止保存”。

## 安全行为

- 服务端限制最大线速度和角速度，浏览器不能绕过限制。
- 浏览器拖动虚拟摇杆时以 10 Hz 刷新目标；节点以 20 Hz 发布。
- 超过 `command_timeout`（默认 0.6 秒）未收到运动目标即发布零速度。
- 松开按键、窗口失焦、切换页面、点击停止或急停都会发送零速度。
- 软件急停是粘滞的；解除急停后仍保持停止，必须重新拖动摇杆才会运动。
- 节点退出时连续发布三次零速度。

软件急停不能代替实体急停。首次联调应架空驱动轮或在开阔区域使用低速参数。

## 完整启动流程（D435i + LeKiwi + 网页）

首次构建在工作区根目录执行一次：

```bash
cd /home/lunar/project/lunar_slam
source /opt/ros/humble/setup.bash
sudo apt-get install ros-humble-rtabmap-launch
colcon build --packages-select \
  luxi_adapter luxi_location luxi_rtab_map luxi_voxel_navigation luxi_web_control \
  --symlink-install
```

随后按下列顺序使用两个终端。网页遥控本身只需终端二；要使用网页 RGB 预览、建图或
基于相机的地图定位，必须先保持终端一的硬件 profile 运行。

### 终端一：启动唯一硬件 profile

当前默认 profile 是 D435i，选择、话题名和驱动工作区均由
`luxi_adapter/config/sensor_bringup.yaml` 管理。不要再手工启动
`lunar_realsense_bringup`，否则会与适配层重复占用同一相机。

```bash
cd /home/lunar/project/lunar_slam
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch luxi_adapter sensor_bringup.launch.py
```

第一次插入或热插拔 D435i 后，驱动可能需要数十秒重新枚举。保持此终端运行，确认已
出现 `RealSense Node Is Up!`；网页建图会最多等待 60 秒，以等待统一传感器数据。

可在另一个终端确认适配层已准备好：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 topic echo --once /sensors/rgbd/color/image_raw
ros2 topic echo --once /sensors/rgbd/depth/image_raw
ros2 topic echo --once /sensors/imu/data
```

### 终端二：启动网页与底盘控制

```bash
cd /home/lunar/project/lunar_slam
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch luxi_web_control lekiwi_web_control.launch.py \
  bind_address:=0.0.0.0 http_port:=8080
```

查看主机局域网地址：

```bash
hostname -I
```

启动日志会列出当前设备可访问的局域网 URL。同一局域网内的手机或电脑打开其中
与自己同网段的地址，例如 `http://192.168.123.66:8080`。页面使用虚拟摇杆控制：
上下对应前进/后退，左右对应左转/右转，松手自动回中停车；同时保留 W/A/S/D 和
方向键，空格键触发急停。

网页功能与前置条件：

| 网页功能 | 需要先启动的内容 |
|---|---|
| 遥控、急停 | 终端二；底盘在线且订阅 `/cmd_vel` |
| RGB 预览 | 终端一的 `luxi_adapter` profile |
| 开始建图、停止保存 | 终端一与终端二；由网页启动 RTAB-Map 算法 |
| 加载已有地图、显示 OctoMap | 终端二；选择已保存的 `.db` 后网页会自动调用 `tools/export_rtabmap_octomap.sh` 生成彩色 PLY 与 `.bt`，再加载显示 |
| RTAB-Map 粗定位、ICP 精定位、选择目标点 | 终端一、终端二与已保存的 `.db`、彩色 PLY、`.bt` |

关闭时先在网页点击“停止建图”（若正在建图），再在两个终端分别按 `Ctrl-C`；网页不
会自动停止硬件 profile。

也可以直接运行节点并覆盖安全参数：

```bash
ros2 run luxi_web_control web_control_node --ros-args \
  -p max_linear_x:=0.10 \
  -p max_angular_z:=0.35 \
  -p command_timeout:=0.6
```

修改参数后若再次出现 `Package 'luxi_web_control' not found`，通常是当前终端没有
加载工作区。重新执行 `source /home/lunar/project/lunar_slam/install/setup.bash`。

如果提示 `Address already in use`，说明已有网页控制服务正在使用该端口。当前服务可
直接通过浏览器访问，无需再次启动。默认 `auto_stop_existing_web_control:=true` 时，
新实例会自动向同一用户、同一 `luxi_web_control` 网页节点发送 `SIGINT`，等待其发送
零速度并释放端口后再启动。它不会停止其他程序占用的端口。需要两个独立实例时，使用
其他端口，例如 `http_port:=8081`。

## 与不同小车连接

只要底盘订阅 `geometry_msgs/msg/Twist` 即可。不同话题可以通过参数指定：

```bash
ros2 launch luxi_web_control web_control.launch.py cmd_vel_topic:=/base_controller/cmd_vel
```

若底盘需要 ROS remap，也可使用：

```bash
ros2 run luxi_web_control web_control_node --ros-args \
  -r /cmd_vel:=/robot/cmd_vel
```

### 已连接的 LeKiwi 小车（192.168.123.49）

树莓派的 `lekiwi-base.service` 已设为开机自启，直接由
`/lekiwi_base_node` 订阅 `/cmd_vel`。该底盘使用 Fast DDS、Domain 0 和子网发现；
推荐在控制电脑上一键启动：

```bash
ros2 launch luxi_web_control lekiwi_web_control.launch.py
```

它会自动设置所需 DDS 环境变量。若使用通用 launch，则应在本机设置：

```bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_LOCALHOST_ONLY
```

确认发现底盘订阅者：

```bash
ros2 topic info /cmd_vel --verbose
```

在树莓派上可以同时观察网页节点发出的指令：

```bash
ros2 topic echo /cmd_vel geometry_msgs/msg/Twist
```

网页顶部“ROS 订阅者”应大于 0。为 0 时网页服务仍可访问，但速度消息还没有接入
底盘，需要检查两端的 ROS domain、RMW、网卡防火墙和话题名。

## 网页控制 RTAB-Map 建图

网页中的“开始建图”只管理算法建图进程，**不会启动或关闭相机硬件**。因此必须先按
“终端一”启动唯一选定的 `luxi_adapter` profile，再按“终端二”启动网页。D435i 的
完整硬件启动命令为：

```bash
cd /home/lunar/project/lunar_slam
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch luxi_adapter sensor_bringup.launch.py
```

打开网页后按以下顺序操作：

1. 确认“ROS 订阅者”为 `1` 或更大。
2. 点击“开始建图”，状态变为“建图中”。网页会以无 RViz 模式启动
   `luxi_rtab_map/rgbd_mapping.launch.py`。
3. 使用虚拟摇杆缓慢运动并采集环境。
4. 点击“停止建图”。网页会向它启动的建图进程发送 `SIGINT`，RTAB-Map 正常关闭并
   保存数据库。

网页下方会同时显示两块只读预览：当前硬件 profile 的 RGB 图像，以及来自
`/rtabmap/cloud_map` 的稀疏彩色点云。RGB 在相机驱动运行后即可显示；点云需要建图
成功启动并收到 RTAB-Map 地图数据后才会出现。点云为浏览器实时查看而抽样的最多
1800 个点，并不是完整地图导出；其显示采用固定等轴视角，适合确认重建是否持续更新。

每一次新建图默认写入 `/home/lunar/project/lunar_slam/maps/rtab_maps/mapNNN.db`。如果启动失败，
网页会显示失败状态；详细日志位于
`/home/lunar/project/lunar_slam/log/luxi_web_control_rtabmap.log`。常见原因是硬件
profile 尚未运行、热插拔后仍在恢复，或没有 RGB-D/IMU 数据。

网页只停止它自己启动的 RTAB-Map 进程，不会停止手工终端中已经运行的建图任务。

网页还会列出 `maps/rtab_maps/mapNNN.db`。选择地图后，网页会自动调用
`tools/export_rtabmap_octomap.sh` 补齐彩色 PLY 和 `.bt`，并立即加载显示；这一步不会
启动相机定位。地图画布支持拖动旋转视角和滚轮缩放。

要进行无需手工点击初始位姿的自动定位：

1. 选择地图，等待彩色点云和 OctoMap 图层加载完成。
2. 点击“自动定位”，缓慢移动或原地转动机器人，让相机看到建图时记录过的区域。
3. GPU HLoc 使用 NetVLAD 检索候选、SuperPoint + LightGlue 匹配，并通过
   PnP/RANSAC 和当前深度验证后，将 `/luxi_hloc/coarse_pose` 交给后级。
4. `luxi_location` 使用当前深度点云和保存的彩色 PLY 进行 Open3D ICP 精配准，结果
   发布到 `/luxi_location/pose`。
5. 页面显示“已定位”后，以紫色箭头显示机器人位置和朝向，同时显示 `x/y/yaw` 与
   ICP fitness；“选择目标点”此时才会启用。

因此粗定位失败时不会盲目启动 ICP，ICP 失败时也不会开放导航目标。没有对应
`maps/hloc_maps/mapNNN/metadata.yaml` 的地图只能显示，网页会禁用“自动定位”。
点击“停止定位”会结束 HLoc、ICP、OctoMap 和规划进程，但保留已加载的地图图层。该功能不会自动
启用路径跟随，默认也不会向 `/cmd_vel` 发送导航速度。

若只需浏览已保存的 OctoMap，可以不启动终端一；但要让 RTAB-Map 使用当前相机进行
定位、获得可信位姿并启用“选择目标点”，仍必须启动硬件 profile。

## 参数

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `cmd_vel_topic` | `/cmd_vel` | Twist 输出话题 |
| `bind_address` | `0.0.0.0` | HTTP 监听地址 |
| `http_port` | `8080` | HTTP 端口，测试时可设为 0 自动分配 |
| `auto_stop_existing_web_control` | `true` | 自动替换同用户、同包的旧网页控制实例 |
| `publish_rate` | `20.0` | Twist 发布频率（Hz） |
| `command_timeout` | `0.6` | 运动命令失效时间（秒） |
| `max_linear_x` | `0.25` | 前后最大速度（m/s） |
| `max_linear_y` | `0.0` | 横移最大速度，差速小车保持 0 |
| `max_angular_z` | `0.8` | 最大角速度（rad/s） |
| `enable_output` | `true` | 设为 false 时仅发布零速度 |
| `web_root` | 安装目录 | 自定义网页资源目录，主要用于开发测试 |
| `enable_mapping_control` | `true` | 是否显示并允许 RTAB-Map 建图开关 |
| `mapping_launch_package` | `luxi_rtab_map` | 被网页管理的建图 ROS 包 |
| `mapping_launch_file` | `rgbd_mapping.launch.py` | 被网页管理的建图 launch 文件 |
| `enable_preview` | `true` | 是否订阅并提供 RGB、稀疏点云预览 |
| `rgb_preview_topic` | `/sensors/rgbd/color/image_raw/compressed` | 适配层统一的压缩 RGB 话题 |
| `cloud_preview_topic` | `/rtabmap/cloud_map` | RTAB-Map 彩色点云话题 |
| `max_cloud_points` | `1800` | 单次浏览器点云预览的最大抽样点数 |
| `navigation_localization_pose_topic` | `/luxi_hloc/coarse_pose` | HLoc 粗定位输入 |
| `navigation_refined_pose_topic` | `/luxi_location/pose` | ICP 精定位结果 |
| `navigation_refined_fitness_topic` | `/luxi_location/fitness` | ICP 匹配得分 |

默认监听所有网卡且没有用户认证，适合受信任的机器人局域网。不要把 8080 端口直接
暴露到互联网；需要跨公网使用时，应在前方增加带认证和 TLS 的网关。

## HTTP 接口

- `GET /api/status`：控制器状态、限速、订阅者数量。
- `POST /api/cmd_vel`：JSON 字段 `linear_x`、`linear_y`、`angular_z`。
- `POST /api/stop`：立即归零。
- `POST /api/estop`：`{"active": true}` 锁定，`false` 解除。
- `POST /api/mapping/start`：启动受网页管理的 RTAB-Map 建图进程。
- `POST /api/mapping/stop`：停止受网页管理的 RTAB-Map 建图进程并保存数据库。
- `POST /api/navigation/load_map`：转换并加载地图显示图层，不启动定位。
- `POST /api/navigation/localize`：以所选地图启动 GPU HLoc 粗定位和 ICP 精定位。
- `POST /api/navigation/stop`：停止定位、规划相关进程。
- `GET /api/preview/rgb`：最新压缩 RGB 图像，未收到相机数据时返回 404。
- `GET /api/preview/cloud`：抽样后的 XYZRGB 点云 JSON，用于网页 Canvas 预览。

即使外部程序直接调用 API，服务端限幅、急停和超时看门狗仍然生效。
