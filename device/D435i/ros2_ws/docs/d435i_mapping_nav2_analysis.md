# D435i 建图 + Nav2 导航需求与实现分析

本文档分析 `/home/lunar/project/lunar_slam/demo/luxi_-lunar` 参考项目的算法性质，并给出在当前 D435i 实机基础上实现“建图 + Nav2 导航”的需求拆解、系统架构、实现步骤和 RTAB-Map 可运行性判断。

## 1. 结论摘要

目标可以实现，但需要明确边界：

- 只使用 D435i 可以实现 RGB-D 建图。
- 只使用 D435i 不能单独完成完整 Nav2 闭环导航，因为 Nav2 还需要机器人底盘或仿真底盘提供 `/odom`、`base_link`、`/cmd_vel` 执行能力。
- 在“不考虑小车实体”的阶段，可以先实现：
  - D435i 数据接入；
  - RTAB-Map RGB-D 建图；
  - 地图保存和加载；
  - Nav2 软件栈启动；
  - RViz 中显示地图、costmap、路径规划结果；
  - `/cmd_vel` 输出验证。
- 后续接入实体小车或仿真底盘后，才能验证完整闭环导航。

推荐技术路线：

```text
D435i RGB + aligned depth
  -> RTAB-Map RGB-D Odometry / SLAM
  -> /map, map->odom, odom->base_link
  -> Nav2 map_server / planner / controller / costmaps
  -> /cmd_vel
  -> 实体底盘或仿真底盘执行
```

不建议直接复用 `luxi_-lunar` 的语义 SLAM 主线来做标准 Nav2。该项目更像 LAC 任务研究原型，和标准 ROS2 Nav2 架构不是同一套系统。

## 2. 参考项目 luxi_-lunar 到底做了什么

参考项目路径：

```text
/home/lunar/project/lunar_slam/demo/luxi_-lunar
```

其核心 ROS2 包：

```text
ros2_ws/src/lac_omnilrs_adapter
```

README 中说明的主节点包括：

```text
lightglue_vo_node
odom_frame_aligner_node
live_slam_backend_node
semantic_point_cloud_node
persistent_semantic_mapper_node
stereo_rock_obstacle_node
arc_planner_node
goal_manager_node
unetpp_perception_node
```

### 2.1 它使用的相机输入

该项目不是直接面向本机 D435i 实机，而是面向 OmniLRS 模拟器中的 Husky + D455 双目相机：

```text
/OmniLRS/Robots/husky/d455/left/image_raw
/OmniLRS/Robots/husky/d455/right/image_raw
/OmniLRS/Robots/husky/d455/left/camera_info
/OmniLRS/Robots/husky/d455/right/camera_info
/OmniLRS/Robots/husky/d455/left/semantic_segmentation
/OmniLRS/Robots/husky/d455/right/semantic_segmentation
```

它依赖的是双目图像、相机内参、语义分割结果。主线不是 RealSense SDK 深度图。

### 2.2 它的建图使用了什么

`luxi_-lunar` 的建图不是传统 2D occupancy grid SLAM，也不是 Nav2 默认使用的 `map_server` 地图流程。

它主要使用：

- SuperPoint + LightGlue 双目视觉里程计；
- 双目三角化生成稀疏 3D landmark；
- 语义 mask 给 3D 点赋语义标签；
- `live_slam_backend_node` 累积 keyframe 和语义观测；
- 可选 GTSAM 后端；
- loop closure hooks；
- 输出语义点云地图。

主要输出：

```text
/lac/vo/odom
/lac/vo/path
/lac/semantic_points
/lac/slam/odom
/lac/slam/path
/lac/slam/semantic_map
```

因此，参考项目中的“建图”更准确地说是：

```text
自研视觉语义 SLAM / 语义点云地图
```

而不是：

```text
标准 2D 栅格地图 SLAM
```

### 2.3 它的导航使用了什么

参考项目也不是 Nav2。

它提供两类控制：

- `click_to_go_node`
  - 输入 `/move_base_simple/goal` 和 `/odom`
  - 直接输出 `/cmd_vel`
  - 属于简单点击目标控制器。

- `arc_planner_node`
  - 输入 goal、odom、`/lac/rock_obstacles`
  - 采样多条弧线轨迹；
  - 根据目标距离和岩石碰撞风险打分；
  - 选择一条局部弧线；
  - 输出 `/cmd_vel`。

主要话题：

```text
/move_base_simple/goal
/lac/current_goal
/lac/rock_obstacles
/lac/arc_planner/selected_arc
/cmd_vel
```

所以参考项目有导航能力，但它不是 Nav2，而是 LAC 风格局部弧线规划器。

## 3. 本项目目标定义

本阶段目标：

```text
使用 D435i 实机相机实现建图 + Nav2 导航链路。
```

不做：

- 语义分割；
- 语义点云地图；
- 岩石类别识别；
- U-Net++；
- LightGlue 自研 VO 复现；
- 实体小车闭环控制。

需要做：

- D435i RGB-D 数据接入；
- D435i 到机器人坐标系的 TF；
- RGB-D SLAM 建图；
- 生成 Nav2 可用 2D 地图；
- 启动 Nav2；
- 在 RViz 中加载地图并规划路径；
- 后续预留 `/cmd_vel` 给实体小车或仿真底盘。

## 4. 功能需求

### 4.1 D435i 数据接入

输入来自当前已适配的 D435i bringup：

```text
/camera/camera/color/image_raw
/camera/camera/color/camera_info
/camera/camera/depth/image_rect_raw
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/depth/color/points
/tf
/tf_static
```

已验证的能力：

- RGB 可订阅；
- depth 可订阅；
- point cloud 可订阅；
- TF 可订阅；
- 点云约 15Hz；
- 当前相机枚举为 USB 2.1，建议保守使用 `640x480@15fps`。

### 4.2 建图需求

建图需要输出：

```text
/map
map -> odom
odom -> base_link
/rtabmap/cloud_map 或等价 3D 地图
```

地图类型：

- Nav2 使用 2D occupancy grid；
- 调试和可视化可以同时保留 3D 点云地图；
- RTAB-Map 可以从 RGB-D 数据生成 2D grid map 和 3D map。

### 4.3 Nav2 需求

Nav2 需要以下基础输入：

```text
map -> odom -> base_link
/map
/tf
/tf_static
robot footprint
costmap obstacle source
```

Nav2 输出：

```text
/plan
/cmd_vel
/global_costmap/*
/local_costmap/*
```

完整闭环还需要底盘执行：

```text
/cmd_vel -> base controller -> wheel motion
wheel/visual odom -> /odom
```

当前“不考虑小车实体”时，Nav2 可验证到：

- 地图加载；
- costmap 生成；
- 目标点接收；
- 全局路径规划；
- 局部规划器输出 `/cmd_vel`。

但不能验证：

- 机器人真实移动；
- 路径跟踪闭环；
- 到点停止；
- 动态避障闭环。

## 5. 推荐架构：D435i + RTAB-Map + Nav2

### 5.1 总体架构

```text
D435i driver
  ├── RGB image
  ├── aligned depth image
  ├── camera_info
  └── TF camera frames

static TF
  └── base_link -> camera_link

RTAB-Map RGB-D SLAM
  ├── RGB-D odometry
  ├── map -> odom
  ├── /map
  ├── /rtabmap/cloud_map
  └── /rtabmap/grid_map

Nav2
  ├── map_server
  ├── planner_server
  ├── controller_server
  ├── behavior_server
  ├── bt_navigator
  ├── global_costmap
  ├── local_costmap
  └── /cmd_vel
```

### 5.2 为什么使用 RTAB-Map

RTAB-Map 适合当前需求：

- 支持 RGB-D SLAM；
- 支持 RealSense 类深度相机；
- 可以输出 2D occupancy map；
- 可以输出 3D 点云地图；
- 支持回环检测；
- ROS2 Humble 有预编译包；
- 可以直接对接 Nav2 的 map/costmap 生态。

相比直接复现 `luxi_-lunar`：

- 不需要语义 mask；
- 不依赖 OmniLRS 话题；
- 不需要 LightGlue / SuperPoint / GTSAM 自研链路；
- 更接近标准 ROS2 建图导航流程。

## 6. RTAB-Map 在本设备上的可运行性

当前设备信息：

```text
Architecture: aarch64
Kernel: Linux tegra
CPU: 12 x Cortex-A78AE
Memory: 61GiB
Swap: 30GiB
ROS: Humble
```

当前包状态：

```text
Nav2: 已安装
slam_toolbox: 已安装
RTAB-Map ROS2: 未安装
```

apt 源中可用的 RTAB-Map Humble arm64 包：

```text
ros-humble-rtabmap-ros      0.23.7
ros-humble-rtabmap-slam     0.23.7
ros-humble-rtabmap-odom     0.23.7
```

判断：

```text
可以运行。
```

理由：

- ROS2 Humble arm64 包可用；
- 当前设备 CPU 和内存明显足够跑 RTAB-Map；
- D435i 数据已经验证可用；
- 推荐从低负载配置开始：`640x480@15fps`；
- 如果接入 USB3，可再尝试 `640x480@30fps`。

风险点：

- 当前 D435i 枚举为 USB 2.1，带宽受限；
- RTAB-Map RGB-D odometry 对图像质量、光照、纹理比较敏感；
- 纯相机 VO 长时间运行会有漂移，回环检测能缓解但不能完全替代轮速计/IMU；
- 如果 D435i 固定不动，无法完成建图；需要移动相机或后续接入机器人/仿真底盘。

建议安装：

```bash
sudo apt-get update
sudo apt-get install -y \
  ros-humble-rtabmap-ros \
  ros-humble-rtabmap-slam \
  ros-humble-rtabmap-odom \
  ros-humble-robot-localization \
  ros-humble-pointcloud-to-laserscan
```

## 7. TF 设计

Nav2 和 RTAB-Map 都依赖稳定 TF。

推荐 TF 树：

```text
map
└── odom
    └── base_link
        └── camera_link
            ├── camera_color_optical_frame
            └── camera_depth_optical_frame
```

其中：

- `camera_link -> camera_*_optical_frame` 由 RealSense 驱动发布；
- `base_link -> camera_link` 由本项目静态 TF 发布；
- `odom -> base_link` 可由 RTAB-Map RGB-D odometry 发布；
- `map -> odom` 可由 RTAB-Map SLAM 发布。

如果先不考虑小车实体，可以假设：

```text
base_link 与 camera_link 重合，或只给一个固定外参。
```

后续接入真实小车后，需要改成真实相机安装外参。

## 8. 建图实现思路

### 8.1 D435i 启动

使用现有 bringup：

```bash
source /opt/ros/humble/setup.bash
source /home/lunar/project/lunar_slam/device/D435i/ros2_ws/install/setup.bash
ros2 launch lunar_realsense_bringup d435i.launch.py rviz:=false
```

需要保证：

```text
/camera/camera/color/image_raw
/camera/camera/color/camera_info
/camera/camera/aligned_depth_to_color/image_raw
/tf
/tf_static
```

### 8.2 启动静态 TF

如果相机坐标和机器人基座先简化为重合，可以发布：

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link \
  --child-frame-id camera_link
```

实际部署时应替换为真实外参。

### 8.3 RTAB-Map RGB-D 建图

RTAB-Map 输入建议使用：

```text
rgb_topic   = /camera/camera/color/image_raw
depth_topic = /camera/camera/aligned_depth_to_color/image_raw
camera_info = /camera/camera/color/camera_info
frame_id    = base_link
```

关键点：

- 使用 aligned depth，而不是原始 depth；
- `frame_id` 使用 `base_link`，便于后续接 Nav2；
- 需要存在 `base_link -> camera_link -> optical_frame` TF；
- 初始阶段使用 approximate sync。

预期输出：

```text
/odom
/map
/rtabmap/cloud_map
/rtabmap/grid_map
/tf
```

### 8.4 地图保存

RTAB-Map 运行时会保存自己的数据库，通常是：

```text
~/.ros/rtabmap.db
```

Nav2 使用的 2D 地图可以用 map saver 保存：

```bash
ros2 run nav2_map_server map_saver_cli -f /path/to/map_name
```

输出：

```text
map_name.yaml
map_name.pgm
```

## 9. Nav2 实现思路

### 9.1 建图阶段和导航阶段分开

推荐分两个阶段：

#### 阶段 A：建图模式

```text
D435i + RTAB-Map SLAM
```

目标：

- 生成 `/map`；
- 生成 `rtabmap.db`；
- 保存 `map.yaml/map.pgm`；
- 验证 TF。

#### 阶段 B：导航模式

```text
D435i + localization + Nav2
```

目标：

- 加载已保存地图；
- 启动 Nav2；
- D435i 提供局部避障数据；
- 底盘或仿真底盘执行 `/cmd_vel`。

如果没有底盘，只能做路径规划和 `/cmd_vel` 输出验证。

### 9.2 Nav2 地图输入

使用 `nav2_map_server` 加载：

```text
map.yaml
map.pgm
```

Nav2 参数中：

```yaml
global_frame: map
robot_base_frame: base_link
odom_frame: odom
```

### 9.3 局部避障数据

D435i 可以给 Nav2 costmap 提供障碍信息。两种方案：

#### 方案 1：PointCloud2 直接给 voxel layer

输入：

```text
/camera/camera/depth/color/points
```

优点：

- 保留 3D 信息；
- 适合 RGB-D 相机。

缺点：

- Nav2 costmap 参数更复杂；
- 需要合理设置高度过滤，否则地面点和噪声会影响 costmap。

#### 方案 2：pointcloud_to_laserscan

输入：

```text
/camera/camera/depth/color/points
```

输出：

```text
/scan
```

Nav2 obstacle layer 使用 `/scan`。

优点：

- 接近传统 2D 激光雷达导航流程；
- 调参更简单。

缺点：

- 损失 3D 信息；
- 视场角和障碍检测距离受 D435i 限制。

建议初期使用方案 2，先把 Nav2 跑通。

### 9.4 Nav2 控制输出

Nav2 输出：

```text
/cmd_vel
```

如果没有实体小车，应准备一个 `/cmd_vel` 消费者用于测试：

- Gazebo/Isaac Sim 差速底盘；
- 简单 fake base simulator；
- 或仅用 `ros2 topic echo /cmd_vel` 验证输出。

没有 `/cmd_vel` 执行者时，Nav2 不能真正导航到目标点。

## 10. 推荐实施阶段

### 阶段 1：RTAB-Map 安装和最小建图

目标：

- 安装 RTAB-Map；
- 用 D435i 启动 RGB-D SLAM；
- RViz 显示 `/map`、点云、TF；
- 保存地图。

验收：

```bash
ros2 topic echo /map --once
ros2 topic hz /rtabmap/cloud_map
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

### 阶段 2：Nav2 加载静态地图

目标：

- 加载保存的 `map.yaml`；
- 启动 Nav2；
- RViz 中设置 goal；
- 看到 `/plan`。

验收：

```bash
ros2 topic echo /plan --once
ros2 topic echo /cmd_vel
ros2 topic list | grep costmap
```

### 阶段 3：D435i 局部避障接入

目标：

- 将 D435i 点云转 `/scan`；
- Nav2 local costmap 使用 `/scan`；
- RViz 中看到局部障碍物。

验收：

```bash
ros2 topic hz /scan
ros2 topic echo /local_costmap/costmap --once
```

### 阶段 4：接入仿真底盘

目标：

- 提供 `/odom`；
- 消费 `/cmd_vel`；
- 发布 `odom -> base_link`；
- 完整验证导航闭环。

可选实现：

- Gazebo differential drive；
- Isaac Sim；
- 自写简化 differential drive simulator。

### 阶段 5：接入真实小车

目标：

- 使用真实底盘驱动；
- 使用轮速计/IMU 融合；
- 调整 Nav2 footprint、速度、加速度、costmap。

## 11. 推荐新增包结构

建议在当前 D435i 工作区新增一个 bringup 包，例如：

```text
ros2_ws/src/lunar_d435i_nav2_bringup
├── package.xml
├── setup.py
├── launch/
│   ├── d435i_rtabmap_mapping.launch.py
│   ├── d435i_nav2.launch.py
│   └── pointcloud_to_scan.launch.py
├── config/
│   ├── rtabmap.yaml
│   ├── nav2_params.yaml
│   └── pointcloud_to_laserscan.yaml
├── maps/
│   └── README.md
└── rviz/
    └── d435i_nav2.rviz
```

职责划分：

- `lunar_realsense_bringup`
  - 只负责 D435i 驱动、RViz 显示、基础 topic 检查。

- `lunar_d435i_nav2_bringup`
  - 负责 RTAB-Map 建图；
  - 负责 Nav2；
  - 负责点云转 scan；
  - 负责地图保存/加载配置。

这样不会把相机驱动和导航应用耦合在一起。

## 12. 与 luxi_-lunar 的关系

可以借鉴：

- 目标管理思想；
- 局部障碍物可视化；
- Foxglove/RViz 调试方式；
- 语义地图作为后续扩展方向。

不建议直接复用：

- `lightglue_vo_node` 作为主建图前端；
- `live_slam_backend_node` 作为 Nav2 地图源；
- `arc_planner_node` 替代 Nav2；
- U-Net++ 语义链路。

原因：

- 参考项目面向 OmniLRS D455 双目模拟环境；
- 强依赖语义 mask；
- 输出地图不是 Nav2 标准 2D map 主线；
- 导航不是 Nav2。

本项目优先实现：

```text
D435i + RTAB-Map + Nav2
```

后续如果需要月面语义能力，再考虑：

```text
D435i + learned segmentation + semantic costmap layer
```

## 13. 关键风险和解决思路

### 13.1 只有 D435i，没有底盘

问题：

```text
Nav2 无法闭环移动。
```

解决：

- 阶段性只验证建图和路径规划；
- 后续加仿真底盘；
- 最后接实体小车。

### 13.2 相机当前是 USB2

问题：

```text
带宽低，RGB-D 帧率和分辨率受限。
```

解决：

- 初始使用 `640x480@15fps`；
- 尽量接 USB3；
- RTAB-Map 降低特征数量和点云密度。

### 13.3 RGB-D VO 漂移

问题：

```text
纯视觉里程计容易漂移。
```

解决：

- 启用 RTAB-Map 回环；
- 后续接轮速计/IMU；
- 用 `robot_localization` 融合多源 odom。

### 13.4 D435i 视场有限

问题：

```text
只靠前向深度相机做避障，侧向和后方障碍不可见。
```

解决：

- 限制测试速度；
- 加大安全距离；
- 后续增加 2D lidar 或多相机；
- Nav2 local costmap 设置合理 obstacle persistence。

## 14. 推荐验收标准

### 建图验收

必须满足：

```text
/camera/camera/color/image_raw 正常
/camera/camera/aligned_depth_to_color/image_raw 正常
/map 正常
map -> odom TF 正常
odom -> base_link TF 正常
地图可保存为 map.yaml/map.pgm
```

### Nav2 软件栈验收

必须满足：

```text
nav2 lifecycle active
/map 可用
/global_costmap/costmap 可用
/local_costmap/costmap 可用
RViz goal 后 /plan 可用
/cmd_vel 有输出
```

### 完整导航验收

需要实体或仿真底盘后才能验收：

```text
机器人根据 /cmd_vel 移动
odom 随运动更新
Nav2 能到达目标点
障碍物进入 local costmap
机器人能绕开障碍物
```

## 15. 下一步建议

建议下一步实施顺序：

1. 安装 RTAB-Map 和 pointcloud_to_laserscan；
2. 新增 `lunar_d435i_nav2_bringup` 包；
3. 写 `d435i_rtabmap_mapping.launch.py`；
4. 验证 `/map`、`map->odom`、`odom->base_link`；
5. 保存 2D map；
6. 写 `nav2_params.yaml` 和 `d435i_nav2.launch.py`；
7. 在无底盘情况下验证 `/plan` 和 `/cmd_vel`；
8. 再决定使用仿真底盘还是接真实小车。

