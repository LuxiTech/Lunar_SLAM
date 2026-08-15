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
cd /home/nvidia/Desktop/lunar_slam
bash scripts/stop_luxi_system.sh
```

脚本先请求网页停止运动、建图和导航，等待数据库保存，然后只向当前用户启动且命令行
匹配本工程的进程发送 `SIGINT`。若进程继承了“忽略 SIGINT”的状态，5 秒后会升级为
`SIGTERM`。最后只有在相关 PID 全部退出且 8080 已释放时才报告成功；若列出残留 PID，
不要再次启动，应先检查这些 PID。
不要直接使用不带匹配条件的 `killall python3`、`killall component_container_mt`，它们会
误停桌面或其他 ROS 任务。

切换 HIK/D435i 或重新启动整套系统时必须执行上述完整清理。若只是重复执行网页 launch，
新网页节点会先让旧网页正常停止建图和导航、释放 8080，并联动结束旧的速度仲裁节点；
最多等待 30 秒供 RTAB-Map 保存数据库。不要在旧实例未退出时改用 8081 绕过检查，否则
两套网页和速度仲裁节点会同时发布 ROS 话题。

### 终端一：选择并启动硬件

HIK 双目 + H30 IMU（当前推荐测试命令）：

```bash
cd /home/nvidia/Desktop/lunar_-slam
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch luxi_adapter sensor_bringup.launch.py hardware:=hik
```

该 profile 启动双目采集、H30、1024×750 深度计算和 `luxi_adapter`。默认
`external_trigger:=true`，应保证相机触发线和触发源工作；仅做无外触发台架检查时才显式
传入 `external_trigger:=false`。

D435i 使用完全相同的入口，只替换一个参数：

```bash
cd /home/nvidia/Desktop/lunar_-slam
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch luxi_adapter sensor_bringup.launch.py hardware:=d435i
```

不传 `hardware` 时为 `auto`，会读取
`project/luxi_adapter/config/sensor_bringup.yaml`，当前该默认配置选择 D435i。为了让现场
操作明确且不受默认值变化影响，建议始终写出 `hardware:=hik` 或 `hardware:=d435i`。



### 终端二：启动网页

连接 LeKiwi 底盘时使用：

```bash
cd /home/nvidia/Desktop/lunar_-slam
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

若提示 `previous web controller did not release port 8080`，说明旧版本或异常进程没有
完成退出；执行本页“启动前：结束旧进程”的完整清理命令，确认没有残留 PID 后仍使用
8080 重新启动。不要改用 8081 掩盖重复进程。若提示
`Package 'luxi_web_control' not found`，重新执行
`source /home/nvidia/Desktop/lunar_-slam/install/setup.bash`。

网页 launch 默认先执行一次 `ros2 daemon stop`，再启动网页节点。该操作只清理
`ros2cli` 的图发现缓存，不会停止相机、建图或底盘 ROS 节点；仅在确认 daemon
健康且不希望重启时传入 `reset_ros_daemon:=false`。

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

### 已连接的 D1 机器人（192.168.123.49）

控制目标使用 Fast DDS、Domain 42 和子网发现。代码不会根据 DDS 图自动选择机器人：
同一网络可能发现多台 D1，自动选择存在误控风险。用 `robot_namespace` 显式指定唯一目标。
网页与导航先输出标准
`geometry_msgs/msg/Twist`，再由 `slam_d1_bridge` 转换为厂家接口
`/d15041873/command/user_command`。启动网页和速度仲裁：

```bash
ros2 launch luxi_web_control lekiwi_web_control.launch.py \
  robot_namespace:=d15041873
```

一键入口还可同时指定该机器人的有线 IP：

```bash
./scripts/start_d1_web_control.sh \
  --robot-ns d15041873 --robot-ip 192.168.123.49
```

第二台 D1 使用相同有线 IP，按 namespace 区分：

```bash
./scripts/start_d1_web_control.sh \
  --robot-ns d15042176 --robot-ip 192.168.123.49
```

`robot_namespace` 会统一派生 FSM 反馈、控制器状态服务、SDK 参数服务、桥输出话题、
PID 文件和日志。若 8080 端口已有网页实例指向另一台机器人，脚本会拒绝复用，需先安全
停止原实例再切换。

该启动文件名因兼容已有部署仍保留 `lekiwi`，当前配置面向 D1：它通过 C++ 适配器把
网页 `/d1/cmd_vel_standard` 与导航 `/navigation/cmd_vel` 仲裁后转发到 `/cmd_vel`。
导航只有在路径跟随器发布 `/navigation/active=true` 时才能接管；非零手动指令、急停、停止、
定位失效或导航指令超过 0.3 秒未更新都会取消接管并输出零速度。厂家 SDK 定义
`angular.z > 0` 为左转，因此 D1 链路不再反转 `angular.z`。

页面中的“机器人控制”开关管理 D1 权限和姿态流程。开启完成前以及关闭过程中，服务端
拒绝所有非零运动命令；关闭会先停车，再让机器人趴下并释放 SDK。出于安全考虑，刷新
页面不会自动站立，切换时浏览器会再次要求现场确认。

实车导航操作顺序为：加载地图、自动定位、选择目标点、等待“规划完成”，再点击“出发”。
“停止行驶”保留定位和当前路径；“停止定位”和软件急停都会先停止行驶。当前速度上限为
0.10 m/s，控制链只使用保存的静态地图，尚未实现实时局部障碍物融合和动态绕行。

它会自动设置所需 DDS 环境变量。若使用通用 launch，则应在本机设置：

```bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export ROS_LOCALHOST_ONLY=0
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

D1 端 `d1_bringup.service` 必须使用相同设置。建议在它加载的 ROS 环境中设置：

```ini
Environment=ROS_DOMAIN_ID=42
Environment=RMW_IMPLEMENTATION=rmw_fastrtps_cpp
Environment=ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
Environment=ROS_LOCALHOST_ONLY=0
Environment=FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

修改环境后仅在机器人安全趴下且有人持急停时重启 D1 bringup。控制机地址为
`192.168.123.51/24`，机器人地址为
`192.168.123.49/24`；两端应允许 Fast DDS 的 UDP 发现和数据流量。

`/cmd_vel` 是本工程内部接口，D1 不直接订阅它。必须另启本工程中的厂家桥；完整构建、
SDK 模式、站立、停止和测试步骤见
[`docs/d1_robot_control.md`](../../docs/d1_robot_control.md)。

确认厂家命令订阅者：

```bash
ros2 topic info /d15041873/command/user_command --verbose
```

启动桥之前应为 `Publisher count: 0`、`Subscription count: 1`；`http_ros_gateway` 与
`slam_d1_bridge` 不能同时发布。可以观察本工程送入桥的标准速度：

```bash
ros2 topic echo /cmd_vel geometry_msgs/msg/Twist
```

桥启动后，网页顶部 `/cmd_vel` 的“ROS 订阅者”应大于 0。为 0 时网页服务仍可访问，
但命令没有进入 D1 桥，需要检查桥进程和 ROS 环境。

## 网页控制 RTAB-Map 建图（当前 NX USB 双目配置）

网页“深度链路”可选择 CREStereo 稳定档、CREStereo 极致档或 VPI OFA/PVA/VIC，
三者均连接 Luxi 和 RTAB-Map；默认使用稳定 CREStereo，极致档以 960×540 RGB-D
冲刺 10 Hz，VPI 是备用项。不要再单独启动相机 profile。算法、性能与复测方法见
[USB 双目相机 README](../../device/USBCameraSDK/ros2_ws/README.md)。

稳定档使用 640×360 两级 CREStereo，并以低分辨率反向推理做左右一致性校验；在
反光地面上会主动舍弃无法双向验证的点，避免把镜面亮斑和遮挡边缘融合成障碍。

服务端只接受 `mode=crestereo`、`mode=crestereo_max` 或 `mode=vpi`，不会恢复已移除
的 CUDA SGM/经典前端。VPI 链路默认传入 `use_imu:=true`；IMU 六轴或四元数健康门禁
失败时建图不会启动。两个 CREStereo 档均使用同时间戳的学习前端里程计直接驱动
RTAB-Map。网页默认设置
`mapping_crestereo_use_imu:=true`；H30 无数据或轴/四元数无效时健康门禁会拒绝启动，
避免不可信姿态进入地图。

CREStereo 建图采用分层深度：0.4--4 m 全量构建可靠近场结构，4--6 m 固定保留
4x4 像素网格，6--10 m 保留 8x8 像素网格形成稀疏环境轮廓。远距层不参与超过 4 m
的里程计特征求解，因此不会用低视差深度拉动相机位姿；VPI profile 的既有范围和
资源配置不变。

D1 上车安装使用实测近似值：双目中点位于机身旋转中心前方 0.20 m、上方 0.20 m，
且两者中心线重合。网页实际传入左目光心 TF，因此结合 89.963 mm 双目基线设置为
`camera_x=0.20`、`camera_y=0.044982`、`camera_z=0.20` m；相机水平朝前。

打开网页后按以下顺序操作：

1. 确认 USB 双目和 H30 已连接，机器人位于水平面且完全静止。
2. 点击“安装角度校准”。网页会短暂独占 H30，采集 200 个重力样本后自动释放串口，
   并将结果保存到 `maps/calibration/<robot_namespace>_camera_mount.json`。下一次建图和
   定位会自动覆盖 `camera_qx/qy/qz/qw`，不需要手工抄写角度。
3. 选择深度链路后点击“开始建图”。状态栏会显示实际使用的算法，并以无 RViz模式启动。
4. 使用虚拟摇杆缓慢运动并采集环境。
5. 点击“停止建图”。网页只向顶层 launch 发送一次 `SIGINT`，由 launch 按顺序关闭
   子节点；等待 RTAB-Map 打印保存完成后，状态才返回“未启动”。

安装角度校准日志位于 `log/luxi_web_control_imu_calibration.log`。校准期间不能同时建图
或定位；启动失败、H30 无数据、机器人持续晃动及结果保存失败都会直接显示在网页上。

网页下方会同时显示两块只读预览：USB 左相机 RGB 图像，以及来自
`/rtabmap/cloud_map` 的彩色点云。RGB 在相机驱动运行后即可显示；点云需要建图
成功启动并收到 RTAB-Map 地图数据后才会出现。当前 `max_cloud_points=0`，网页不再对
实时点云抽样；其显示采用固定等轴视角，适合确认重建是否持续更新。大地图若导致网页
延迟升高，可将该参数恢复为正数以限制单帧预览点数，这不会改变地图数据库或导出结果。

每一次新建图默认写入 `/home/nvidia/Desktop/lunar_slam/maps/rtab_maps/mapNNN.db`。如果启动失败，
网页会显示失败状态；详细日志位于
`/home/nvidia/Desktop/lunar_-slam/log/luxi_web_control_rtabmap.log`。常见原因是相机
被旧进程占用、USB 热插拔后仍在恢复，或没有同步双目数据。

网页只停止它自己启动的 RTAB-Map 进程，不会停止手工终端中已经运行的建图任务。
启动前还会检查 ROS 图中的 `/luxi_visual_frontend` 和 `/rtabmap/rtabmap`。如果终端已
经启动建图，网页会返回冲突并拒绝创建第二套学习前端和 RTAB-Map；此时应继续使用终端
中的任务，或者先正常结束它，再从网页启动。该保护用于防止 GPU 瞬时满载、CPU 翻倍、
重复 TF 发布和额外内存占用。

网页还会列出 `maps/rtab_maps/mapNNN.db`。选择地图后，网页会自动调用
`tools/export_rtabmap_octomap.sh` 补齐彩色 PLY 和 `.bt`，并在缺失时调用
`luxi_hloc` 导出器和 CUDA 模型构建器生成 HLoc 索引，随后立即加载显示；这一步不会
启动相机定位。“生成过滤地图”会保留原始导出，生成独立的过滤 PLY 和 `.bt`；完成后
同一按键用于在原始版和过滤版之间切换。自动定位和路径规划始终使用网页当前显示的
版本，避免混用点云与 OctoMap。保存地图的彩色 PLY 默认读取并显示全部有效顶点，不使用实时预览的
1800 点抽样上限。地图画布支持拖动旋转视角和滚轮缩放。默认视角遵循 ROS REP-103：
`+X`（机器人前方）朝屏幕上方，`+Y`（机器人左方）朝屏幕左侧，画布左下角同时显示
方向标记。首次打开时默认选择编号最大的可用地图；定时刷新列表不会改变用户已经选择
的地图。水平拖动采用轨道视角语义：向右拖动时观察视角向右环绕，地图内容向左旋转；
该手势只改变观察角度，不修改地图坐标。

地图加载时还会调用 `luxi_3d_navigation/terrain_map_to_points`，使用与规划器相同的
C++ `TerrainModel` 生成网页地形层。它读取与当前地图版本匹配的 PLY，按 0.05 m
体素降采样并以 0.30 m
局部邻域法向和高度连续性拆分平面，只保留最大的连续主地面；只有高于附近地面至少 0.15 m、且确有点云
支撑的栅格才显示为红色障碍，稀疏或没有观测的区域保持 unknown。青绿色表示可通行
表面，黄色到红色表示逐渐靠近地图边界或不可通行区域；原始 OctoMap 体素可用独立
开关显示。当前按机器人半径 0.10 m 做碰撞检查，并在 0.60 m 边缘带内生成代价。
A* 使用同一代价且默认权重为 8.0，在存在宽通道时会选择低代价的中间路线。

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
| `reset_ros_daemon` | `true` | launch 启动前清理旧 Fast DDS 图发现状态，不影响运行中的 ROS 节点 |
| `auto_stop_existing_web_control` | `true` | 自动替换同用户、同包的旧网页控制实例 |
| `publish_rate` | `20.0` | Twist 发布频率（Hz） |
| `command_timeout` | `0.6` | 运动命令失效时间（秒） |
| `max_linear_x` | `0.25` | 前后最大速度（m/s） |
| `max_linear_y` | `0.0` | 横移最大速度，差速小车保持 0 |
| `max_angular_z` | `0.8` | 最大角速度（rad/s） |
| `enable_output` | `true` | 设为 false 时仅发布零速度 |
| `web_root` | 安装目录 | 自定义网页资源目录，主要用于开发测试 |
| `enable_mapping_control` | `true` | 是否显示并允许 RTAB-Map 建图开关 |
| `mapping_launch_package` | `lunar_usb_rtabmap_bringup` | 与 D435i 同层级的 USB 专属建图入口包 |
| `mapping_launch_file` | `usb_rtabmap.launch.py` | 兼容项；服务端根据网页模式选择稳定 CRE、CRE MAX 或 VPI 入口 |
| `mapping_launch_arguments` | `new_map/无界面/H30 IMU/自由 6DoF` | 传给建图入口的参数列表；IMU 健康门禁失败时不会启动里程计 |
| `mapping_crestereo_use_imu` | `true` | 两个 CREStereo 档默认使用 H30；仅无 IMU 诊断时关闭 |
| `mapping_sensor_setup` | USB 工作区 `install/setup.bash` | 在通用算法环境之上加载 USB 设备包 |
| `auto_build_hloc_index` | `true` | 加载地图时是否自动构建缺失的 GPU HLoc 索引 |
| `hloc_index_build_timeout` | `900.0` | HLoc 导出和单个模型构建步骤的超时秒数 |
| `enable_preview` | `true` | 是否订阅并提供 RGB、实时点云预览 |
| `rgb_preview_topic` | `/left_camera/image/compressed` | USB 左相机压缩 RGB 话题 |
| `cloud_preview_topic` | `/rtabmap/cloud_map` | RTAB-Map 彩色点云话题 |
| `max_cloud_points` | `0` | 单次浏览器点云预览的最大抽样点数；`0` 表示不抽样 |
| `max_saved_cloud_points` | `30000` | 浏览器 Canvas 的保存点云显示上限；只限制预览，不改变 PLY、OctoMap 或定位精度 |
| `max_terrain_points` | `12000` | 每类 C++ 地形图层的浏览器抽样上限，不改变规划地图 |
| `navigation_robot_radius` | `0.10` | 网页离线地形预览使用的机器人半径（m），应与规划器一致 |
| `navigation_costmap_margin` | `0.60` | 网页离线地形预览的边缘代价宽度（m） |
| `navigation_ground_normal_radius` | `0.30` | PLY 局部法向拟合邻域半径（m） |
| `navigation_ground_max_slope_degrees` | `35.0` | 地面分割允许的法向倾角（度），不是底盘最终爬坡角 |
| `navigation_obstacle_min_height` | `0.15` | 点云高于附近地面后进入障碍层的最小高度（m） |
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
- `POST /api/navigation/load_map`：传入 `map_id` 以及布尔字段 `filtered`，转换并加载
  原始或过滤地图显示图层，不启动定位。
- `POST /api/navigation/localize`：以所选地图启动 GPU HLoc 粗定位和 ICP 精定位。
- `POST /api/navigation/stop`：停止定位、规划相关进程。
- `GET /api/navigation/terrain`：当前地图的可通行点、归一化边缘代价和障碍物点。
- `GET /api/preview/rgb`：最新压缩 RGB 图像，未收到相机数据时返回 404。
- `GET /api/preview/cloud`：抽样后的 XYZRGB 点云 JSON，用于网页 Canvas 预览。
- `GET /api/semantic/annotations?map_id=mapNNN`：检查 OctoMap 并加载独立标注。
- `POST /api/semantic/save`：校验并原子保存所选地图的语义标注 JSON。

即使外部程序直接调用 API，服务端限幅、急停和超时看门狗仍然生效。
