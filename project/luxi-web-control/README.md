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

## 构建与启动（LeKiwi 小车）

在工作区根目录执行：

```bash
cd /home/lunar/project/lunar_slam
source /opt/ros/humble/setup.bash
colcon build --packages-select luxi_web_control luxi_rtab_map --symlink-install
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

网页中的“开始建图”只管理算法建图进程，**不会启动或关闭 D435i 相机驱动**。这是为了
避免网页按钮重复占用相机设备。开始建图前，先在单独终端启动相机：

```bash
cd /home/lunar/project/lunar_slam
source /opt/ros/humble/setup.bash
source device/D435i/ros2_ws/install/setup.bash
ros2 launch lunar_realsense_bringup d435i.launch.py \
  rviz:=false enable_imu:=true unite_imu_method:=2 enable_pointcloud:=false
```

然后启动网页控制（LeKiwi 小车使用专用 launch）：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch luxi_web_control lekiwi_web_control.launch.py
```

打开网页后按以下顺序操作：

1. 确认“ROS 订阅者”为 `1` 或更大。
2. 点击“开始建图”，状态变为“建图中”。网页会以无 RViz 模式启动
   `luxi_rtab_map/rgbd_mapping.launch.py`。
3. 使用虚拟摇杆缓慢运动并采集环境。
4. 点击“停止建图”。网页会向它启动的建图进程发送 `SIGINT`，RTAB-Map 正常关闭并
   保存数据库。

网页下方会同时显示两块只读预览：D435i 当前 RGB 图像，以及来自
`/rtabmap/cloud_map` 的稀疏彩色点云。RGB 在相机驱动运行后即可显示；点云需要建图
成功启动并收到 RTAB-Map 地图数据后才会出现。点云为浏览器实时查看而抽样的最多
1800 个点，并不是完整地图导出；其显示采用固定等轴视角，适合确认重建是否持续更新。

每一次新建图默认写入 `/home/lunar/project/lunar_slam/maps/mapNNN.db`。如果启动失败，
网页会显示失败状态；详细日志位于
`/home/lunar/project/lunar_slam/log/luxi_web_control_rtabmap.log`。常见原因是 D435i
驱动尚未运行、没有相机 RGB-D 数据，或没有加载 D435i 工作区。

网页只停止它自己启动的 RTAB-Map 进程，不会停止手工终端中已经运行的建图任务。

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
| `rgb_preview_topic` | `/camera/camera/color/image_raw/compressed` | D435i 压缩 RGB 话题 |
| `cloud_preview_topic` | `/rtabmap/cloud_map` | RTAB-Map 彩色点云话题 |
| `max_cloud_points` | `1800` | 单次浏览器点云预览的最大抽样点数 |

默认监听所有网卡且没有用户认证，适合受信任的机器人局域网。不要把 8080 端口直接
暴露到互联网；需要跨公网使用时，应在前方增加带认证和 TLS 的网关。

## HTTP 接口

- `GET /api/status`：控制器状态、限速、订阅者数量。
- `POST /api/cmd_vel`：JSON 字段 `linear_x`、`linear_y`、`angular_z`。
- `POST /api/stop`：立即归零。
- `POST /api/estop`：`{"active": true}` 锁定，`false` 解除。
- `POST /api/mapping/start`：启动受网页管理的 RTAB-Map 建图进程。
- `POST /api/mapping/stop`：停止受网页管理的 RTAB-Map 建图进程并保存数据库。
- `GET /api/preview/rgb`：最新压缩 RGB 图像，未收到相机数据时返回 404。
- `GET /api/preview/cloud`：抽样后的 XYZRGB 点云 JSON，用于网页 Canvas 预览。

即使外部程序直接调用 API，服务端限幅、急停和超时看门狗仍然生效。
