# Luxi Web Control

一个与小车型号无关的 ROS 2 网页遥控节点。浏览器通过同源 HTTP API 发送运动目标，
节点限幅并发布标准 `geometry_msgs/msg/Twist`，默认话题为 `/cmd_vel`。不依赖
rosbridge、前端框架或厂商 SDK。它还可管理本项目 RTAB-Map 建图 launch 的启动和
停止，方便在同一网页中完成“开始建图 → 遥控采集 → 停止保存”。


## 设备无关的启动结构

系统分成两层，切换相机只发生在第一层：

```text
HIK 双目 + H30 IMU ─┐
                    ├─ luxi_adapter ─ /sensors/* ─ luxi_visual_frontend
D435i RGB-D + IMU ──┘                                  └─ luxi_rtab_map
                                                               └─ Web/RViz
```

HIK 和 D435i 必须输出同一组算法接口。网页、视觉前端和 RTAB 后端不读取厂商话题，也
不再默认加载 D435i 的安装目录：

- `/sensors/rgbd/rgbd_image`：原子 RGB-D 包，供视觉前端使用；
- `/sensors/rgbd/color/image_raw`、`/sensors/rgbd/depth/image_raw`；
- `/sensors/rgbd/color/camera_info`；
- `/sensors/rgbd/color/image_raw/compressed`：网页 RGB 预览；
- `/sensors/imu/data_raw`、`/sensors/imu/data`。

不要同时手工启动 HIK/D435i 厂商驱动、`hik_mapping.launch.py` 或另一套
`sensor_bringup.launch.py`，否则会重复占用相机或重复发布 TF/话题。

## 首次构建

```bash
cd /home/nvidia/Desktop/lunar_slam
source /opt/ros/humble/setup.bash
colcon build --packages-select \
  hikrobot_camera_driver hik_bringup stereo_depth \
  luxi_adapter luxi_visual_frontend luxi_rtab_map \
  luxi_location luxi_semantic_annotation luxi_voxel_navigation \
  luxi_3d_navigation luxi_web_control \
  --symlink-install
```

## 完整启动流程（当前测试设备：HIK）

按顺序保持两个终端运行。终端一只负责选定的硬件及适配层；终端二负责网页、底盘以及
由网页启动/停止的设备无关建图进程。

### 启动前：结束旧进程

每次重新启动系统或切换 HIK/D435i 前，先在一个终端执行下面的清理命令。它会先请求
网页正常停止建图和定位，使 RTAB-Map 有机会保存数据库；随后只结束当前用户启动的
Luxi 网页、建图和相机链路，防止重复占用 8080 端口、D435i USB、HIK 相机、GPU 以及
重复发布 TF/话题。

```bash
bash -lc '
set +e
curl -fsS -X POST -H "Content-Type: application/json" -d "{}" \
  http://127.0.0.1:8080/api/mapping/stop >/dev/null 2>&1
curl -fsS -X POST -H "Content-Type: application/json" -d "{}" \
  http://127.0.0.1:8080/api/navigation/stop >/dev/null 2>&1
sleep 2
current_uid=$(id -u)
process_pattern="[/](luxi_web_control/lib/luxi_web_control/web_control_node|luxi_visual_frontend/lib/luxi_visual_frontend/visual_odometry_node|rtabmap_slam/rtabmap|luxi_adapter/lib/luxi_adapter/sensor_adapter_node|realsense2_camera/lib/realsense2_camera/realsense2_camera_node|imu_filter_madgwick/lib/imu_filter_madgwick/imu_filter_madgwick_node|yesense_std_ros2/lib/yesense_std_ros2/yesense_node_publisher|stereo_depth/lib/stereo_depth/stereo_depth_node|hikrobot_camera_driver/lib/hikrobot_camera_driver/stereo_node)|__node:=[l]uxi_sensor_container|[s]ensor_bringup\.launch\.py|[d]435i\.launch\.py|[s]tereo_camera_bringup\.launch\.py|[l]ekiwi_web_control\.launch\.py|[w]eb_control\.launch\.py"
pkill -INT -u "$current_uid" -f "$process_pattern"
sleep 3
if pgrep -u "$current_uid" -af "$process_pattern"; then
  echo "仍有 Luxi 旧进程，请先检查上面列出的 PID。"
  exit 1
fi
echo "Luxi 旧进程已清理，可以启动硬件和网页。"
'
```

上面整个代码块是一条命令，可直接完整复制执行。如果命令列出残留 PID 并返回失败，
不要再次启动；先确认残留 PID 属于本工作区并正常结束。
不要直接使用不带匹配条件的 `killall python3`、`killall component_container_mt`，它们会
误停桌面或其他 ROS 任务。

### 终端一：选择并启动硬件

HIK 双目 + H30 IMU（当前推荐测试命令）：

```bash
cd /home/nvidia/Desktop/lunar_slam
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch luxi_adapter sensor_bringup.launch.py hardware:=hik
```

该 profile 启动双目采集、H30、1024×750 深度计算和 `luxi_adapter`。默认
`external_trigger:=true`，应保证相机触发线和触发源工作；仅做无外触发台架检查时才显式
传入 `external_trigger:=false`。

D435i 使用完全相同的入口，只替换一个参数：

```bash
cd /home/nvidia/Desktop/lunar_slam
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch luxi_adapter sensor_bringup.launch.py hardware:=d435i
```

不传 `hardware` 时为 `auto`，会读取
`project/luxi_adapter/config/sensor_bringup.yaml`，当前该默认配置选择 D435i。为了让现场
操作明确且不受默认值变化影响，建议始终写出 `hardware:=hik` 或 `hardware:=d435i`。



### 终端二：启动网页

连接 LeKiwi 底盘时使用：

```bash
cd /home/nvidia/Desktop/lunar_slam
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch luxi_web_control lekiwi_web_control.launch.py \
  bind_address:=0.0.0.0 http_port:=8080
```

只在本机测试网页、无需 LeKiwi DDS 设置时可使用：

```bash
ros2 launch luxi_web_control web_control.launch.py \
  bind_address:=127.0.0.1 http_port:=8080
```

用 `hostname -I` 查看主机地址；同一局域网浏览器打开日志列出的地址，例如
`http://192.168.123.66:8080`。网页启动后应先看到 HIK RGB 预览，再点击“开始建图”。
状态变为“建图中”后，页面点云窗口应出现 `/rtabmap/cloud_map` 的完整彩色点云。采集
结束必须点击“停止建图”，等待 RTAB 正常保存数据库后再关闭终端。

| 网页功能 | 前置条件 |
|---|---|
| 遥控、急停 | 网页运行；底盘订阅 `/cmd_vel` |
| RGB 预览 | 任一硬件 profile 正常发布统一压缩 RGB |
| 开始建图、停止保存 | 硬件 profile 与网页运行；同步门禁通过 |
| 实时点云预览 | 建图运行且 `/rtabmap/cloud_map` 已发布 |
| 加载/标注已有地图 | 网页运行且已有 `.db`；不要求相机在线 |
| 地图定位、选择导航目标 | 硬件、网页及已导出的地图层均可用 |

关闭顺序：网页“停止建图” → 等待状态停止 → 终端二 `Ctrl-C` → 终端一 `Ctrl-C`。
网页只管理自己启动的算法进程，不会停止硬件 profile。

若提示 `Address already in use`，先访问现有 8080 服务；也可改用
`http_port:=8081`。若提示 `Package 'luxi_web_control' not found`，重新执行
`source /home/nvidia/Desktop/lunar_slam/install/setup.bash`。

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
“终端一”启动唯一选定的 `luxi_adapter` profile，再按“终端二”启动网页。硬件型号不
会改变这里使用的算法 launch。

打开网页后按以下顺序操作：

1. 确认“ROS 订阅者”为 `1` 或更大。
2. 点击“开始建图”，状态变为“建图中”。网页会以无 RViz 模式启动
   `luxi_rtab_map/rgbd_mapping_learned.launch.py`。该流程使用
   SuperPoint + LightGlue 外部里程计，并将 SuperPoint 局部特征交给 RTAB 后端。
3. 使用虚拟摇杆缓慢运动并采集环境。
4. 点击“停止建图”。网页会向它启动的建图进程发送 `SIGINT`，RTAB-Map 正常关闭并
   保存数据库。

网页下方会同时显示两块只读预览：当前硬件 profile 的 RGB 图像，以及来自
`/rtabmap/cloud_map` 的彩色点云。RGB 在相机驱动运行后即可显示；点云需要建图
成功启动并收到 RTAB-Map 地图数据后才会出现。当前 `max_cloud_points=0`，网页不再对
实时点云抽样；其显示采用固定等轴视角，适合确认重建是否持续更新。大地图若导致网页
延迟升高，可将该参数恢复为正数以限制单帧预览点数，这不会改变地图数据库或导出结果。

每一次新建图默认写入 `/home/nvidia/Desktop/lunar_slam/maps/rtab_maps/mapNNN.db`。如果启动失败，
网页会显示失败状态；详细日志位于
`/home/nvidia/Desktop/lunar_slam/log/luxi_web_control_rtabmap.log`。常见原因是硬件
profile 尚未运行、热插拔后仍在恢复，或没有 RGB-D/IMU 数据。

网页只停止它自己启动的 RTAB-Map 进程，不会停止手工终端中已经运行的建图任务。
启动前还会检查 ROS 图中的 `/luxi_visual_frontend` 和 `/rtabmap/rtabmap`。如果终端已
经启动建图，网页会返回冲突并拒绝创建第二套学习前端和 RTAB-Map；此时应继续使用终端
中的任务，或者先正常结束它，再从网页启动。该保护用于防止 GPU 瞬时满载、CPU 翻倍、
重复 TF 发布和额外内存占用。

网页还会列出 `maps/rtab_maps/mapNNN.db`。选择地图后，网页会自动调用
`tools/export_rtabmap_octomap.sh` 补齐彩色 PLY 和 `.bt`，并在缺失时调用
`luxi_hloc` 导出器和 CUDA 模型构建器生成 HLoc 索引，随后立即加载显示；这一步不会
启动相机定位。保存地图的彩色 PLY 默认读取并显示全部有效顶点，不使用实时预览的
1800 点抽样上限。地图画布支持拖动旋转视角和滚轮缩放。默认视角遵循 ROS REP-103：
`+X`（机器人前方）朝屏幕上方，`+Y`（机器人左方）朝屏幕左侧，画布左下角同时显示
方向标记。首次打开时默认选择编号最大的可用地图；定时刷新列表不会改变用户已经选择
的地图。水平拖动采用轨道视角语义：向右拖动时观察视角向右环绕，地图内容向左旋转；
该手势只改变观察角度，不修改地图坐标。

### 已有地图的离线语义标注

选择并加载地图后，地图画布上方会出现语义标注工具。操作顺序如下：

“显示图层”中的“语义标注”复选框可以单独显示或隐藏岩石、墙和坑覆盖层；关闭它只
影响网页显示，不会删除已经加载或保存的标注。选择标注画笔时该图层会自动重新开启。

1. 检查自动估计的“地面 Z”，必要时手工修正；“最小高度”决定岩石/墙画笔可选择的
   最低占用体素。
2. 选择“岩石画笔”或“墙体画笔”后在体素上拖动；选择“橡皮擦”可删除标签，点击
   已闭合的坑区域可删除整块坑标注。
3. 标注坑时依次点击地面上的边界点，设置坑深，再点击“闭合坑区域”。未闭合的草稿
   不允许保存。
4. 点击“保存标注”。服务端会用 `luxi_semantic_annotation` 核对每个岩石/墙标签确实
   对应 `.bt` 中高于阈值的占用体素，再原子写入：
   `maps/semantic_maps/mapNNN/annotations.json`。

原始 `.bt`、RTAB-Map `.db` 和彩色 PLY 始终保持只读。坑之所以记录为二维地面多边形
加深度，是因为当前 `.bt` 仅能可靠提供占用体素，坑内部没有可供画笔附着的占用节点。
“撤销”保留最近 50 次操作，“重载标注”可放弃尚未保存的修改。

要进行无需手工点击初始位姿的自动定位：

1. 选择地图，等待彩色点云和 OctoMap 图层加载完成。
2. 点击“自动定位”，缓慢移动或原地转动机器人，让相机看到建图时记录过的区域。
3. GPU HLoc 使用 NetVLAD 检索候选、SuperPoint + LightGlue 匹配，并通过
   PnP/RANSAC 和当前深度验证。只有连续 3 帧粗位姿相差不超过 0.5 m、20°，才交给
   ICP。
4. `luxi_location` 使用当前深度点云和保存的彩色 PLY 进行 Open3D ICP 精配准，结果
   发布到 `/luxi_location/pose`。首次成功后自动停用 HLoc，避免持续占用 GPU 和重复
   注入粗位姿；ICP 连续失败 5 帧才清除旧位姿并重新启用 HLoc。
5. 页面显示“已定位”后，以紫色箭头显示机器人位置和朝向，同时显示 `x/y/yaw` 与
   ICP fitness；“选择目标点”此时才会启用。

因此粗定位失败时不会盲目启动 ICP，ICP 失败时也不会开放导航目标。默认情况下，没有
`maps/hloc_maps/mapNNN/metadata.yaml` 的地图会在加载时自动构建；构建失败时地图仍可
显示，但网页会报告具体错误并禁用“自动定位”。
点击“停止定位”会结束 HLoc、ICP、OctoMap 和规划进程，但保留已加载的地图图层。该功能不会自动
启用路径跟随，默认也不会向 `/cmd_vel` 发送导航速度。

定位成功后，网页目标会交给 `luxi_3d_navigation`。它在 OctoMap 的 26 邻域中进行
三维 A* 搜索，并要求每个路径体素具有地面支撑、足够的机器人净空且不超过配置的
台阶和坡度；语义标注中的坑多边形会作为不可通行区域。网页点击仍提供 x/y，规划器
会自动吸附到附近可行走表面的 z。发布的路径保留三维高度，但地面底盘只跟随 x/y
和偏航；仍需显式向 `/navigation/start` 发布 `true` 才会开始运动。

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
| `mapping_launch_file` | `rgbd_mapping_learned.launch.py` | 被网页管理的学习型前端建图 launch 文件 |
| `mapping_sensor_setup` | 空 | 可选的额外设备环境；统一工作区构建时保持为空 |
| `auto_build_hloc_index` | `true` | 加载地图时是否自动构建缺失的 GPU HLoc 索引 |
| `hloc_index_build_timeout` | `900.0` | HLoc 导出和单个模型构建步骤的超时秒数 |
| `enable_preview` | `true` | 是否订阅并提供 RGB、实时点云预览 |
| `rgb_preview_topic` | `/sensors/rgbd/color/image_raw/compressed` | 适配层统一的压缩 RGB 话题 |
| `cloud_preview_topic` | `/rtabmap/cloud_map` | RTAB-Map 彩色点云话题 |
| `max_cloud_points` | `0` | 单次浏览器点云预览的最大抽样点数；`0` 表示不抽样 |
| `max_saved_cloud_points` | `30000` | 浏览器 Canvas 的保存点云显示上限；只限制预览，不改变 PLY、OctoMap 或定位精度 |
| `semantic_annotation_timeout` | `15.0` | 单次标注检查或保存的超时秒数 |
| `semantic_maps_root` | `maps/semantic_maps` | 独立语义标注输出目录 |
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
- `GET /api/semantic/annotations?map_id=mapNNN`：检查 OctoMap 并加载独立标注。
- `POST /api/semantic/save`：校验并原子保存所选地图的语义标注 JSON。

即使外部程序直接调用 API，服务端限幅、急停和超时看门狗仍然生效。
