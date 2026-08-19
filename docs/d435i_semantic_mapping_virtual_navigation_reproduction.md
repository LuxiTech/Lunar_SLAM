# D435i 语义建图与无底盘导航复现方案

## 1. 文档目的

本文说明如何在当前工程中，仅使用一台 Intel RealSense D435i，复现
`demo/luxi_-lunar` 参考程序的主要能力，并为以后接入真实小车保留清晰接口。

本文覆盖：

- D435i RGB-D 视觉里程计与 RTAB-Map 建图；
- 回环检测、位姿图优化、地图保存和加载定位；
- RGB 语义分割、RGB-D 语义点云和持久语义地图；
- 深度障碍物与二维障碍栅格；
- 无底盘时的虚拟目标、全局规划和局部轨迹可视化；
- 所需 ROS 2 节点、话题、TF、包边界和推荐文件位置；
- 分阶段实现顺序和验收标准。

本文是一份复现与实施设计。其中 `luxi_rtab_map` 的 RGB-D 建图链路已经存在；
语义建图、IMU 接入和虚拟导航部分是建议新增的模块，不能把本文列出的计划节点
误认为当前仓库已经全部实现。

## 2. 复现目标与范围

### 2.1 可以复现的能力

没有小车时，可以手持 D435i 移动并实现：

1. RGB-D 视觉里程计；
2. 六自由度相机轨迹；
3. RTAB-Map 回环检测与图优化；
4. 彩色点云和二维占据栅格建图；
5. RTAB-Map 数据库保存、加载和重定位；
6. 基于 RGB 图像的语义分割；
7. 语义掩码与对齐深度融合生成三维语义点；
8. 在 `map` 坐标系中累计语义地图、岩石/障碍物栅格和高程栅格；
9. 在 RViz 中点击目标并计算全局路径；
10. 使用当前深度障碍物生成虚拟局部轨迹，但不驱动物理底盘。

### 2.2 当前不能完整复现的能力

没有底盘时不能验证以下闭环：

```text
/cmd_vel -> 底盘执行 -> 轮速里程计 -> 机器人真实运动 -> 重新感知与规划
```

手持 D435i 的运动也不满足差速小车的运动学约束。因此，无底盘阶段的导航结果
只能用于验证地图、TF、目标管理、路径规划和避障算法接口，不能用于评价真实车辆的
跟踪误差、制动距离、打滑或控制稳定性。

## 3. 与参考程序的对应关系

参考程序使用自研 LightGlue 双目 VO、GTSAM 后端、稀疏语义点和圆弧规划器。
D435i 复现方案不逐行移植，而是保留功能含义并替换为更适合实机的实现。

| 参考程序能力 | D435i 复现实现 | 原因 |
| --- | --- | --- |
| 灰度双目输入 | RGB + 对齐深度 | D435i 已直接提供深度 |
| SuperPoint + LightGlue VO | RTAB-Map RGB-D odometry | 当前工程已有，实机集成更成熟 |
| GTSAM 自研 SLAM 后端 | RTAB-Map 回环与图优化 | 支持数据库、图优化和重定位 |
| 稀疏双目语义三角化 | 语义掩码 + aligned depth 稠密反投影 | 不需要再次估计视差 |
| `/lac/slam/semantic_map` | `/semantic_mapping/semantic_map` | 由独立语义融合节点发布 |
| 双目语义岩石圆 | 深度语义障碍物或局部障碍点云 | 直接利用深度和语义类别 |
| Arc Planner | 无底盘阶段只发布虚拟轨迹 | 禁止误发真实控制命令 |
| Foxglove click-to-go | RViz `2D Goal Pose` 或 Foxglove goal | 统一使用 `PoseStamped` |

本方案的核心原则是：

```text
RTAB-Map 负责几何 SLAM 和定位；
语义包负责图像推理和语义地图；
导航包只消费标准地图、TF、里程计和目标，不反向侵入 SLAM。
```

## 4. 当前工程边界

当前源码根目录：

```text
/home/lunar/project/lunar_slam
```

工程分层保持如下：

```text
lunar_slam/
├── device/D435i/ros2_ws/src/
│   ├── lunar_realsense_bringup/    # D435i 驱动启动和硬件检查
│   └── realsense-ros/              # 官方驱动源码
├── 3parts/
│   ├── rtabmap/                    # RTAB-Map core 第三方源码
│   └── rtabmap_ros/                # RTAB-Map ROS 2 wrapper
└── project/
    ├── luxi_RTAB_Map/              # 当前 RGB-D SLAM 算法包
    ├── luxi_semantic_mapping/      # 计划新增：语义感知和地图
    └── luxi_virtual_navigation/    # 计划新增：无底盘规划验证
```

约束如下：

- 不在 `realsense-ros` 中加入业务算法；
- 不修改 `3parts/rtabmap*` 来实现项目功能；
- D435i 参数和硬件检查保留在 `lunar_realsense_bringup`；
- SLAM 启动、建图/定位模式保留在 `luxi_rtab_map`；
- 模型推理和语义融合放入 `luxi_semantic_mapping`；
- 规划、目标管理和安全输出放入 `luxi_virtual_navigation`。

## 5. 总体节点架构

### 5.1 完整数据流

```text
realsense2_camera_node
  ├─ /camera/camera/color/image_raw -----------------------┐
  ├─ /camera/camera/aligned_depth_to_color/image_raw ------┼────────────┐
  ├─ /camera/camera/color/camera_info ---------------------┼───────┐    │
  ├─ /camera/camera/gyro/sample ---------------------------┐│       │    │
  ├─ /camera/camera/accel/sample --------------------------┤│       │    │
  └─ /tf, /tf_static                                      ││       │    │
                                                            ││       │    │
                                              imu_filter（可选）       │    │
                                                            ││       │    │
rgbd_sync -> rgbd_odometry -> rtabmap ----------------------┘│       │    │
  ├─ /rtabmap/odom                                           │       │    │
  ├─ /rtabmap/map                                             │       │    │
  ├─ /rtabmap/mapData                                         │       │    │
  └─ TF: map -> odom -> base_link                              │       │    │
                                                               │       │    │
semantic_inference_node <---------------------------------------┘       │    │
  ├─ /semantic_mapping/mask                                           │    │
  ├─ /semantic_mapping/color                                          │    │
  └─ /semantic_mapping/confidence                                     │    │
                                                                        │    │
semantic_projection_node <----------------------------------------------┘    │
  <--------------------------------------------------------------------------┘
  + CameraInfo + TF(map <- camera)
  └─ /semantic_mapping/frame_points

semantic_mapper_node
  ├─ /semantic_mapping/semantic_map
  ├─ /semantic_mapping/obstacle_grid
  ├─ /semantic_mapping/elevation_grid
  └─ /semantic_mapping/status

RViz/Foxglove goal -> goal_manager_node -> /virtual_nav/current_goal
                                            │
/rtabmap/map + TF + current_goal ------------┴-> planner_server / path_planner_node
                                                  └─ /virtual_nav/global_path

local_obstacle_node -> /virtual_nav/local_obstacles
global_path + local_obstacles + TF -> trajectory_sampler_node
  ├─ /virtual_nav/selected_trajectory
  └─ /virtual_nav/cmd_vel_preview（仅预览，禁止连接底盘）
```

### 5.2 推荐 TF 树

无底盘阶段仍使用标准机器人 TF 命名：

```text
map
└── odom
    └── base_link
        └── camera_link
            ├── camera_color_frame
            │   └── camera_color_optical_frame
            ├── camera_depth_frame
            │   └── camera_depth_optical_frame
            └── camera_imu_optical_frame
```

职责分配：

- `map -> odom`：RTAB-Map 发布，表达回环后的全局校正；
- `odom -> base_link`：RGB-D odometry 发布；
- `base_link -> camera_link`：无底盘时可设单位变换，当前 launch 已支持；
- `camera_link` 以下：RealSense 驱动发布。

不能同时启动两个节点发布同一条 TF。接入真实底盘后，应由 URDF 或
`robot_state_publisher` 发布真实 `base_link -> camera_link` 外参，并关闭当前临时静态 TF。

## 6. 节点清单与接口

## 6.1 已有硬件节点：`realsense2_camera_node`

所属包：`realsense2_camera`

启动入口：

```text
device/D435i/ros2_ws/src/lunar_realsense_bringup/launch/d435i.launch.py
```

必须输出：

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` | VO 和语义推理 |
| `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | RGB-D VO 和语义反投影 |
| `/camera/camera/color/camera_info` | `sensor_msgs/CameraInfo` | RGB 内参 |
| `/camera/camera/depth/color/points` | `sensor_msgs/PointCloud2` | 调试和局部障碍物备选输入 |
| `/tf`、`/tf_static` | `tf2_msgs/TFMessage` | 相机内部外参 |

计划接入 IMU 时还要确认实际话题：

```text
/camera/camera/gyro/sample
/camera/camera/accel/sample
```

实际名称以 `ros2 topic list` 为准，不应在算法节点中假设未验证的话题存在。

## 6.2 已有同步节点：`rgbd_sync`

所属包：`rtabmap_sync`

由当前 `rgbd_mapping.launch.py` 间接启动。它对 RGB、对齐深度和 CameraInfo
进行近似时间同步，发布：

```text
/rtabmap/rgbd_image
```

当前同步窗口：

```text
approx_sync_max_interval = 0.05 s
```

语义投影节点也必须实施时间同步，但不要复用 `/rtabmap/rgbd_image` 中的压缩内部表示；
直接订阅标准 Image 和 CameraInfo，保持语义包与 RTAB-Map 解耦。

## 6.3 已有里程计节点：`rgbd_odometry`

所属包：`rtabmap_odom`

输入：

```text
/rtabmap/rgbd_image
TF(base_link <- camera_link)
```

主要输出：

```text
/rtabmap/odom             nav_msgs/msg/Odometry
/rtabmap/odom_info        rtabmap_msgs/msg/OdomInfo
TF: odom -> base_link
```

该节点代替参考程序的 `lightglue_vo_node`。D435i 的深度用于建立帧间 3D 几何，
不需要自行用左右图视差恢复深度。

## 6.4 已有 SLAM 节点：`rtabmap`

所属包：`rtabmap_slam`

输入：

```text
/rtabmap/rgbd_image
/rtabmap/odom
TF
```

主要输出：

```text
/rtabmap/info
/rtabmap/map
/rtabmap/mapData
/rtabmap/mapGraph
TF: map -> odom
```

职责：

- RGB-D 关键帧管理；
- 外观回环检测；
- 位姿图优化；
- 二维占据栅格生成；
- 数据库持久化；
- 加载数据库后的定位。

它代替参考程序的 `live_slam_backend_node` 和大部分 GTSAM 管理代码。

## 6.5 计划节点：`semantic_inference_node`

所属新包：`luxi_semantic_mapping`

推荐实现语言：Python。深度学习框架可以使用 PyTorch；若 Jetson 性能不足，再增加
TensorRT 后端，不应在第一版同时维护两套推理实现。

订阅：

```text
/camera/camera/color/image_raw       sensor_msgs/msg/Image
```

发布：

```text
/semantic_mapping/mask              sensor_msgs/msg/Image, mono8
/semantic_mapping/color             sensor_msgs/msg/Image, rgb8
/semantic_mapping/confidence        sensor_msgs/msg/Image, mono8，可选
/semantic_mapping/inference_status  diagnostic_msgs/msg/DiagnosticArray 或 std_msgs/msg/String
```

参数：

```yaml
image_topic: /camera/camera/color/image_raw
checkpoint_path: <模型路径>
device: cuda
input_width: 640
input_height: 480
min_period_sec: 0.1
publish_confidence: true
class_names: [background, rock, ground, obstacle]
```

要求：

- 输出 mask 必须使用项目内部稳定类别 ID，而不是数据集原始颜色；
- 输出 header 的时间戳必须复制输入 RGB 时间戳；
- 模型路径不能硬编码到 Python 源码，应由 launch 参数传入；
- 模型加载失败时节点应明确报错退出，不能持续发布空 mask；
- 第一阶段可以使用简单通用分割模型验证接口，最终效果需要实景数据训练。

## 6.6 计划节点：`semantic_projection_node`

所属新包：`luxi_semantic_mapping`

推荐实现语言：C++，便于高效处理深度图和 PointCloud2；也可以先用 Python 原型验证。

订阅并同步：

```text
/semantic_mapping/mask
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/color/camera_info
```

查询 TF：

```text
map <- camera_color_optical_frame
```

发布：

```text
/semantic_mapping/frame_points      sensor_msgs/msg/PointCloud2
```

建议 PointCloud2 字段：

```text
x float32
y float32
z float32
rgb float32
label uint16
confidence float32
```

投影公式：

```text
Z = depth(u, v) * depth_scale
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
p_map = T_map_camera * [X, Y, Z, 1]^T
```

必须过滤：

- 深度为 0 或非有限值；
- 小于 `min_depth_m` 或大于 `max_depth_m`；
- 低置信度语义像素；
- 不需要建图的 `background` 类别；
- TF 查询失败或时间差超过阈值的帧。

建议第一版参数：

```yaml
stride: 4
min_depth_m: 0.2
max_depth_m: 8.0
min_confidence: 0.6
target_frame: map
sync_slop_sec: 0.08
```

`stride` 表示隔多少像素采样一次。先使用 4 或 8，避免 640x480 每帧全部投影造成
Jetson CPU 和 ROS 传输压力。

## 6.7 计划节点：`semantic_mapper_node`

所属新包：`luxi_semantic_mapping`

订阅：

```text
/semantic_mapping/frame_points
/semantic_mapping/reset             std_msgs/msg/Empty
```

发布：

```text
/semantic_mapping/semantic_map      sensor_msgs/msg/PointCloud2
/semantic_mapping/obstacle_grid     nav_msgs/msg/OccupancyGrid
/semantic_mapping/elevation_grid    nav_msgs/msg/OccupancyGrid
/semantic_mapping/status            std_msgs/msg/String
```

第一版采用体素哈希：

```text
voxel_key = floor([x, y, z] / voxel_size)
```

每个体素保存：

```text
观测次数
XYZ 累计值或在线均值
各类别计数
置信度累计值
最后观测时间
```

输出类别取体素中的多数类别，建议默认体素尺寸 `0.10~0.15 m`。

二维障碍栅格由指定障碍类别投影到 XY 平面。建议将以下项目做成参数：

```yaml
obstacle_labels: [rock, obstacle]
ground_labels: [ground]
grid_resolution_m: 0.10
grid_extent_m: 20.0
min_voxel_observations: 2
obstacle_count_threshold: 2
```

### 回环一致性说明

如果 `semantic_projection_node` 直接把点转换到 `map` 后永久体素化，RTAB-Map 之后发生
回环校正时，已经融合的历史点不会自动移动，可能产生重影。

复现应分两步：

1. **MVP 版本**：直接 `map` 坐标体素累计，先验证完整接口；
2. **回环一致版本**：语义观测关联到 RTAB-Map 节点 ID/关键帧局部坐标；收到优化后的
   `mapData` 或图更新后，用新关键帧位姿重建语义地图。

第二步更接近参考程序把语义点绑定 SLAM 关键帧的做法，但实现复杂度明显更高，不应阻塞
第一版语义投影和地图可视化。

## 6.8 计划节点：`local_obstacle_node`

所属新包：`luxi_virtual_navigation`，也可在后续稳定后移入语义包。

第一版输入建议使用 D435i 点云：

```text
/camera/camera/depth/color/points
```

语义版输入可以替换为：

```text
/semantic_mapping/frame_points
```

输出：

```text
/virtual_nav/local_obstacles        sensor_msgs/msg/PointCloud2
```

处理步骤：

1. TF 转到 `base_link`；
2. 限制前方和左右感兴趣区域；
3. 去除地面平面或只保留语义障碍类别；
4. 体素降采样；
5. 可选聚类并估计每个障碍物半径。

无底盘阶段 `base_link` 等价于相机载体坐标。接底盘后必须使用真实外参。

## 6.9 计划节点：`goal_manager_node`

所属新包：`luxi_virtual_navigation`

订阅：

```text
/goal_pose 或 /move_base_simple/goal    geometry_msgs/msg/PoseStamped
/virtual_nav/cancel                     std_msgs/msg/Empty
TF: map <- base_link
```

发布：

```text
/virtual_nav/current_goal               geometry_msgs/msg/PoseStamped
/virtual_nav/goal_status                std_msgs/msg/String
```

要求拒绝以下目标：

- frame 为空；
- 无法转换到 `map`；
- 坐标包含 NaN/Inf；
- 目标落在占据栅格障碍单元内。

无底盘阶段不应根据手持相机偶然接近目标就宣称控制成功；状态中应区分
`path_available`、`camera_near_goal` 和未来的 `robot_goal_reached`。

## 6.10 全局规划：优先复用 Nav2 Planner Server

不建议自行实现完整 A* 节点。无底盘阶段可以只启动 Nav2 的：

```text
planner_server
global_costmap
map/costmap 输入适配
```

不启动：

```text
controller_server
bt_navigator
velocity_smoother
真实 cmd_vel 输出
```

全局地图可选择：

1. RTAB-Map 几何地图 `/rtabmap/map`；
2. 语义障碍地图 `/semantic_mapping/obstacle_grid`；
3. 后续通过 costmap layer 同时融合二者。

第一版建议把 `/rtabmap/map` remap 到规划器所需的 `/map`，验证路径生成；第二版再将
语义栅格加入代价地图。

如果当前 Nav2 安装或配置不完整，可临时实现 `path_planner_node`：订阅 OccupancyGrid、
读取 TF 中的起点、接收目标并运行 A*，发布 `nav_msgs/Path`。该节点只能作为接口原型，
后续仍建议替换为 Nav2。

## 6.11 计划节点：`trajectory_sampler_node`

这是参考程序 Arc Planner 的无底盘版本。

订阅：

```text
/virtual_nav/global_path
/virtual_nav/local_obstacles
TF: map/odom/base_link
```

发布：

```text
/virtual_nav/candidate_trajectories   visualization_msgs/msg/MarkerArray
/virtual_nav/selected_trajectory      nav_msgs/msg/Path
/virtual_nav/cmd_vel_preview          geometry_msgs/msg/TwistStamped
```

必须使用 `cmd_vel_preview`，而不是 `/cmd_vel`。在没有底盘和安全系统时，禁止让该节点默认
发布真实控制话题。

候选轨迹可以沿用参考程序的恒速圆弧：

```text
v = constant
omega in [-omega_max, omega_max]
```

代价建议扩展为：

```text
cost =
  path_distance_weight * 到全局路径距离
  + goal_distance_weight * 到局部目标距离
  + heading_weight * 航向误差
  + clearance_weight * 障碍物距离代价
  + curvature_weight * 曲率变化代价
```

无底盘阶段只检查所选轨迹是否绕开当前点云障碍物，不评价车辆是否真的能跟踪。

## 7. 推荐目录和文件位置

## 7.1 保留并修改 `luxi_rtab_map`

现有包：

```text
/home/lunar/project/lunar_slam/project/luxi_RTAB_Map/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── rgbd_mapping.launch.py             # 已有，保留建图入口
│   └── rgbd_mapping_test.launch.py        # 已有，保留测试入口
├── src/
│   └── luxi_rtab_map_node.cpp             # 已有 C++ API 实验
└── docs/
```

建议新增：

```text
launch/
├── rgbd_localization.launch.py            # 加载数据库，只做定位
└── rgbd_slam.launch.py                     # 可选统一入口，mode=mapping/localization
config/
├── rtabmap_mapping.yaml                    # 建图参数
└── rtabmap_localization.yaml               # 定位参数
rviz/
└── d435i_mapping.rviz                      # 项目自有显示配置
```

同时修改 `CMakeLists.txt` 的安装目录：

```cmake
install(DIRECTORY launch config rviz docs
  DESTINATION share/${PROJECT_NAME}
)
```

建图模式：

```text
localization=false
首次新建地图时允许 --delete_db_on_start
database_path=~/.ros/luxi_rtab_map.db
```

定位模式：

```text
localization=true
禁止 --delete_db_on_start
使用已有 database_path
不继续扩展地图或明确设置增量内存参数
```

## 7.2 新增 `luxi_semantic_mapping`

推荐使用 `ament_python` 起步，后续把性能热点单独改为 C++ 可执行节点：

```text
/home/lunar/project/lunar_slam/project/luxi_semantic_mapping/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── luxi_semantic_mapping
├── launch/
│   ├── semantic_mapping.launch.py
│   └── semantic_mapping_with_rtabmap.launch.py
├── config/
│   ├── semantic_classes.yaml
│   ├── semantic_inference.yaml
│   └── semantic_mapper.yaml
├── models/
│   └── README.md                    # 只说明模型来源和放置方式，不提交大权重
├── rviz/
│   └── semantic_mapping.rviz
├── luxi_semantic_mapping/
│   ├── __init__.py
│   ├── semantic_inference_node.py
│   ├── semantic_projection_node.py
│   ├── semantic_mapper_node.py
│   ├── pointcloud_utils.py
│   ├── label_schema.py
│   └── synchronization.py
├── test/
│   ├── test_label_schema.py
│   ├── test_depth_projection.py
│   ├── test_voxel_fusion.py
│   └── test_time_sync.py
└── docs/
    ├── model_training.md
    └── semantic_map_format.md
```

如果 `semantic_projection_node` 经性能测试确认 Python 不够用，再新增：

```text
src/semantic_projection_node.cpp
include/luxi_semantic_mapping/semantic_projection.hpp
```

届时可以将包迁移为 `ament_cmake_python`，但不建议在没有性能数据前增加构建复杂度。

## 7.3 新增 `luxi_virtual_navigation`

```text
/home/lunar/project/lunar_slam/project/luxi_virtual_navigation/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── luxi_virtual_navigation
├── launch/
│   ├── virtual_global_planning.launch.py
│   ├── virtual_arc_planning.launch.py
│   └── virtual_navigation_demo.launch.py
├── config/
│   ├── nav2_planner.yaml
│   ├── local_obstacles.yaml
│   └── trajectory_sampler.yaml
├── rviz/
│   └── virtual_navigation.rviz
├── luxi_virtual_navigation/
│   ├── __init__.py
│   ├── goal_manager_node.py
│   ├── local_obstacle_node.py
│   ├── trajectory_sampler_node.py
│   └── geometry.py
├── test/
│   ├── test_goal_transform.py
│   ├── test_occupancy_check.py
│   ├── test_arc_generation.py
│   └── test_collision_check.py
└── docs/
    └── no_base_safety.md
```

## 7.4 顶层统一启动包是否需要

第一版不需要额外创建 bringup 包。使用三个终端分别启动驱动、SLAM 和语义/规划，
更利于定位问题。各模块稳定后，可以新增：

```text
project/luxi_bringup/
└── launch/d435i_semantic_slam_demo.launch.py
```

统一入口只负责 include 其他包的 launch，不能复制其他包的参数实现。

## 8. 当前框架的接入方式

## 8.1 启动顺序

### 终端 1：D435i 驱动

```bash
source /opt/ros/humble/setup.bash
source /home/lunar/project/lunar_slam/device/D435i/ros2_ws/install/setup.bash
ros2 launch lunar_realsense_bringup d435i.launch.py rviz:=false
```

### 终端 2：RTAB-Map 建图

```bash
source /opt/ros/humble/setup.bash
source /home/lunar/project/lunar_slam/install/setup.bash
source /home/lunar/project/lunar_slam/device/D435i/ros2_ws/install/setup.bash

ros2 launch luxi_rtab_map rgbd_mapping.launch.py \
  database_path:=$HOME/.ros/luxi_rtab_map.db \
  rtabmap_args:="--delete_db_on_start"
```

仅在明确要新建地图时使用 `--delete_db_on_start`。保存地图的正常运行或定位模式不能使用它。

### 终端 3：语义建图（实现后）

```bash
source /opt/ros/humble/setup.bash
source /home/lunar/project/lunar_slam/install/setup.bash

ros2 launch luxi_semantic_mapping semantic_mapping.launch.py \
  image_topic:=/camera/camera/color/image_raw \
  depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  camera_info_topic:=/camera/camera/color/camera_info \
  target_frame:=map
```

### 终端 4：无底盘规划（实现后）

```bash
source /opt/ros/humble/setup.bash
source /home/lunar/project/lunar_slam/install/setup.bash

ros2 launch luxi_virtual_navigation virtual_navigation_demo.launch.py \
  map_topic:=/rtabmap/map \
  semantic_obstacle_grid_topic:=/semantic_mapping/obstacle_grid \
  publish_real_cmd_vel:=false
```

`publish_real_cmd_vel` 应设计为默认 `false`，且无底盘版本最好根本不声明 `/cmd_vel` publisher。

## 8.2 QoS 与时间同步

相机图像通常使用 Sensor Data QoS。语义节点订阅 QoS 必须与驱动兼容：

```text
reliability: best_effort
history: keep_last
depth: 5
```

需要同步的三路消息：

```text
RGB 时间戳产生的 semantic mask
aligned depth
CameraInfo
```

推荐规则：

- mask 必须保留原 RGB 时间戳；
- depth 与 mask 时间差建议不超过 `0.08 s`；
- CameraInfo 可缓存最近值；
- TF 必须按消息时间戳查询，不能总是使用 `Time(0)` 的最新 TF；
- 超时帧应丢弃并计数，而不是套用错误位姿。

## 8.3 坐标约定

图像反投影首先得到 optical frame 坐标：

```text
x 向右，y 向下，z 向前
```

不要手工交换坐标轴后又通过 TF 转换一次。正确方式是：

1. 点保持在 `camera_color_optical_frame`；
2. 使用 TF2 转换到 `map` 或 `base_link`；
3. 输出 header.frame_id 与实际数值坐标一致。

这一点应写入单元测试：中心像素的点应沿 optical frame 的正 Z 方向。

## 8.4 地图话题选择

系统会同时存在多种“地图”：

| 话题 | 含义 | 使用方 |
| --- | --- | --- |
| `/rtabmap/map` | 几何二维占据地图 | 全局规划 |
| `/rtabmap/mapData` | RTAB-Map 图和节点数据 | 高级地图重建 |
| `/semantic_mapping/semantic_map` | 三维语义点云 | RViz、语义查询 |
| `/semantic_mapping/obstacle_grid` | 语义障碍二维栅格 | 代价地图或规划器 |
| `/semantic_mapping/elevation_grid` | 地面高度栅格 | 可通行性分析 |
| `/virtual_nav/local_obstacles` | 当前视野局部障碍 | 局部轨迹采样 |

不要把所有地图都 remap 成 `/map`。`/map` 只应作为规划器选定的一张主 OccupancyGrid。

## 9. 建图与定位模式

## 9.1 新建地图

使用独立数据库路径，并显式删除旧库：

```bash
ros2 launch luxi_rtab_map rgbd_mapping.launch.py \
  database_path:=$HOME/.ros/site_a.db \
  rtabmap_args:="--delete_db_on_start"
```

手持采集策略：

- 缓慢平移，避免只在原地快速旋转；
- 保证连续帧有足够纹理和深度；
- 每隔一段距离回看已观察区域；
- 最终回到起点形成大回环；
- 避免玻璃、纯白墙、强反光和大面积无深度区域；
- USB2 当前建议保持 640x480@15 Hz，不要先追求高帧率。

停止节点前确认数据库存在且大小合理：

```bash
ls -lh $HOME/.ros/site_a.db
```

## 9.2 加载地图定位

计划新增的 `rgbd_localization.launch.py` 应：

- 使用同一个数据库；
- 设置 `localization=true`；
- 禁止删除数据库；
- 启动 RGB-D odometry；
- 让 RTAB-Map 用当前图像在历史关键帧中重定位；
- 继续发布 `map -> odom` 和 `/rtabmap/map`。

示例预期命令：

```bash
ros2 launch luxi_rtab_map rgbd_localization.launch.py \
  database_path:=$HOME/.ros/site_a.db
```

定位启动后，应先把相机朝向地图中纹理明显、曾经采集过的区域，并缓慢移动；单帧静止画面
不一定能立即建立稳定定位。

## 10. IMU 接入计划

IMU 不是第一阶段阻塞项。应先确保纯 RGB-D SLAM 稳定，再接入 D435i IMU。

计划节点可以使用已有 `imu_filter_madgwick` 或 RealSense/RTAB-Map 示例中的 IMU 合并方式，
而不是自行积分加速度求位置。

接入前必须验证：

```bash
ros2 topic list | grep -E 'gyro|accel|imu'
ros2 topic hz /camera/camera/gyro/sample
ros2 topic hz /camera/camera/accel/sample
ros2 topic echo --once <imu_topic>
```

IMU 主要用于：

- 重力方向和初始姿态；
- 快速转动时辅助姿态估计；
- 降低 roll/pitch 不稳定。

不能期待只靠消费级 IMU 双积分得到长期稳定位置。以后接小车时，合理的融合输入是：

```text
轮速里程计 + IMU + RGB-D/视觉定位
```

## 11. 无底盘阶段的导航测试方式

### 11.1 静态地图规划测试

最可靠的测试方式是先停止手持移动，在已经生成的 `/rtabmap/map` 上选择起点和目标，
只观察规划路径。

验收内容：

- 起点能从 TF `map -> base_link` 获得；
- RViz 目标能转换到 `map`；
- 占据区域内目标会被拒绝；
- 可达目标产生连续 `nav_msgs/Path`；
- 路径不会穿越膨胀后的障碍区；
- 不存在 `/cmd_vel` 输出。

### 11.2 手持局部障碍轨迹测试

将 D435i 固定在三脚架或手持静止，前方放置纸箱、椅子等深度明显的障碍物：

1. 检查 `/virtual_nav/local_obstacles` 与真实障碍方向一致；
2. 发布位于障碍后方的虚拟局部目标；
3. 检查直线路径被拒绝；
4. 检查左右绕行圆弧至少有一条被保留；
5. 移走障碍后检查直行轨迹恢复；
6. 遮挡深度相机或停止点云，检查规划器进入 stale/stop 状态。

这里的 stop 表示 `cmd_vel_preview` 为零或不再发布，不代表物理急停。

## 12. 分阶段实施计划

## 阶段 A：冻结现有 RGB-D SLAM 基线

工作内容：

- 验证驱动关键话题；
- 验证 `/rtabmap/rgbd_image`、`/rtabmap/odom`、`/rtabmap/map`；
- 记录稳定建图参数；
- 将建图参数从 launch 内联值逐步移到 YAML；
- 完成一段闭环手持建图并保存数据库。

完成标准：

```text
相机缓慢移动时 odom 连续；
闭环后地图重影明显减少；
数据库重启后可读取；
TF 树无重复发布者。
```

## 阶段 B：拆分建图与定位入口

新增：

```text
luxi_RTAB_Map/launch/rgbd_localization.launch.py
luxi_RTAB_Map/config/rtabmap_mapping.yaml
luxi_RTAB_Map/config/rtabmap_localization.yaml
```

完成标准：

- 建图模式可以显式新建数据库；
- 定位模式不会修改或删除既有数据库；
- 在旧场景中可恢复 `map` 位姿；
- 两种模式的启动日志明确显示当前 mode。

## 阶段 C：语义推理

新增 `luxi_semantic_mapping` 包和 `semantic_inference_node`。

完成标准：

- mask 与 RGB 尺寸、时间戳对应；
- RViz 可看到彩色语义图；
- 空模型、错误模型路径和 CUDA 不可用都有明确错误；
- 推理频率不会拖垮 15 Hz 相机与 RTAB-Map。

## 阶段 D：语义 RGB-D 投影

实现 `semantic_projection_node`。

完成标准：

- 平面墙体点云的几何位置与 D435i 原始点云一致；
- 同一物体的语义标签正确；
- 输出 frame 为 `map` 且 RViz 无坐标跳变；
- 深度失效像素不产生原点噪声；
- TF/同步丢帧有状态计数。

## 阶段 E：持久语义地图和障碍栅格

实现 `semantic_mapper_node`。

完成标准：

- 重复观察不会无限复制点；
- reset 话题能清空地图；
- obstacle grid 的 unknown/free/occupied 定义正确；
- 内存受 `max_voxels` 限制；
- 记录回环前后地图重影，决定是否进入关键帧关联版本。

## 阶段 F：无底盘全局规划

增加 `luxi_virtual_navigation`，优先接 Nav2 Planner Server。

完成标准：

- RViz 点击目标可生成全局 Path；
- 不启动 controller，不发布真实 `/cmd_vel`；
- 障碍内目标被拒绝；
- 地图或 TF 超时有明确状态。

## 阶段 G：虚拟局部圆弧

实现 `local_obstacle_node` 和 `trajectory_sampler_node`。

完成标准：

- 当前深度障碍能够淘汰碰撞圆弧；
- 所选轨迹趋向全局路径的局部前视点；
- 障碍消息过期时只输出停止预览；
- 所有控制输出均在 `/virtual_nav` 命名空间。

## 阶段 H：接入真实底盘后再做

未来新增：

- URDF 和真实相机外参；
- 轮速 odometry；
- `robot_localization` 融合；
- Nav2 Controller Server 和行为树；
- `/cmd_vel` 仲裁、速度限制、碰撞监测和物理急停；
- 足迹、最小转弯半径、制动距离和坡度约束。

在这些条件完成前，不能把 `cmd_vel_preview` remap 到真实 `/cmd_vel`。

## 13. 构建方式

当前项目算法工作区根目录是：

```text
/home/lunar/project/lunar_slam
```

新增包后建议按包构建：

```bash
cd /home/lunar/project/lunar_slam
source /opt/ros/humble/setup.bash
source /home/lunar/project/lunar_slam/device/D435i/ros2_ws/install/setup.bash

colcon build --symlink-install --packages-select \
  luxi_rtab_map \
  luxi_semantic_mapping \
  luxi_virtual_navigation

source install/setup.bash
```

不要把源码放进 `build/` 或 `install/`；这两个目录是构建产物。

## 14. 测试与诊断命令

### 14.1 相机输入

```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
ros2 topic echo --once /camera/camera/color/camera_info
```

### 14.2 RTAB-Map

```bash
ros2 topic hz /rtabmap/rgbd_image
ros2 topic hz /rtabmap/odom
ros2 topic echo --once /rtabmap/map
ros2 topic echo --once /rtabmap/info
```

### 14.3 TF

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo map camera_color_optical_frame
```

### 14.4 语义链路（实现后）

```bash
ros2 topic hz /semantic_mapping/mask
ros2 topic hz /semantic_mapping/frame_points
ros2 topic echo --once /semantic_mapping/status
ros2 topic echo --once /semantic_mapping/obstacle_grid
```

### 14.5 虚拟规划（实现后）

```bash
ros2 topic echo --once /virtual_nav/goal_status
ros2 topic echo --once /virtual_nav/global_path
ros2 topic echo --once /virtual_nav/selected_trajectory
ros2 topic list | grep cmd_vel
```

最后一条用于确认无底盘阶段没有意外出现真实 `/cmd_vel` 控制链路。

## 15. 关键风险与处理原则

### 15.1 视觉里程计丢失

表现：`Not enough inliers`、轨迹停止或跳变。

处理：降低移动速度，增加环境纹理，避免纯白墙和反光面；先排除 USB 和时间同步问题，
再调特征与里程计参数。

### 15.2 深度空洞和飞点

处理：限制有效深度范围、过滤 0 值、采用空间/时间滤波、体素降采样，并避免把未知深度
直接当作自由空间。

### 15.3 语义域差异

仿真模型不能直接代表真实 D435i 图像。必须采集目标环境数据，固定类别映射，并保存训练集
版本、模型版本和类别配置之间的对应关系。

### 15.4 回环后语义地图重影

MVP 体素地图不会自动响应历史位姿变化。先量化问题，再实现关键帧关联与重建；不要通过
简单增大体素来掩盖严重坐标错误。

### 15.5 手持运动与小车运动不一致

手持测试适合验证感知和规划接口，不适合调真实控制器。虚拟圆弧输出必须与真实 `/cmd_vel`
隔离。

### 15.6 地图话题和 frame 混淆

每条 PointCloud2、OccupancyGrid 和 Path 的 `header.frame_id` 必须与数据实际坐标一致。
规划使用的起点、目标、地图必须先统一到 `map`。

## 16. 最小可行复现结果

第一轮复现完成时，系统至少应达到：

```text
1. D435i 驱动稳定发布 RGB、aligned depth、CameraInfo 和 TF；
2. RTAB-Map 持续发布 odom、map，并完成至少一次有效回环；
3. 地图数据库可保存，并能在 localization 模式加载；
4. 语义节点发布与 RGB 对齐的 mask；
5. 语义 + depth + TF 生成 map 坐标语义点云；
6. 语义点被体素融合为持久地图和障碍栅格；
7. RViz 点击目标后能在占据地图上生成全局路径；
8. 局部深度障碍能影响虚拟轨迹选择；
9. 整个系统不发布或执行真实底盘 /cmd_vel。
```

达到以上结果后，已经复现了参考程序在无小车条件下最有价值的部分：视觉定位、回环建图、
语义地图、障碍物感知和规划决策。后续接入小车时，主要新增的是底盘状态估计、真实运动学
约束、安全控制和速度执行，而不需要推翻已有感知与地图架构。
