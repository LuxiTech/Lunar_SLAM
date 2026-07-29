# luxi_-lunar 参考代码建图实现分析

本文档说明 `/home/lunar/project/lunar_slam/demo/luxi_-lunar` 参考项目是如何实现视觉里程计、语义点云建图、SLAM 后端和导航控制的，并分析它与当前目标“D435i + 建图 + Nav2 导航”的区别。

## 1. 总体结论

`luxi_-lunar` 不是标准 RTAB-Map / Nav2 项目。

它实现的是：

```text
OmniLRS D455 双目相机
  -> SuperPoint + LightGlue 双目视觉里程计
  -> 稀疏语义点云
  -> keyframe SLAM backend
  -> voxelized semantic point cloud map
  -> rock obstacle extraction
  -> click-to-go / arc planner
```

它的“建图”不是传统 2D occupancy grid SLAM，而是自研的视觉语义 SLAM / 语义点云地图。

它的“导航”也不是 Nav2，而是自研的：

- `click_to_go_node`
- `arc_planner_node`

因此，该项目可以作为视觉 SLAM、语义建图和局部规划算法参考，但不适合作为 D435i + Nav2 标准建图导航的直接实现基础。

## 2. 输入数据

参考项目默认使用 OmniLRS 模拟器中的 Husky + D455 双目相机话题：

```text
/OmniLRS/Robots/husky/d455/left/image_raw
/OmniLRS/Robots/husky/d455/right/image_raw
/OmniLRS/Robots/husky/d455/left/camera_info
/OmniLRS/Robots/husky/d455/right/camera_info
/OmniLRS/Robots/husky/d455/left/semantic_segmentation
/OmniLRS/Robots/husky/d455/right/semantic_segmentation
```

这些话题来自模拟器，不是当前实机 D435i 的默认话题。

当前 D435i 实机话题是：

```text
/camera/camera/color/image_raw
/camera/camera/color/camera_info
/camera/camera/depth/image_rect_raw
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/depth/color/points
/tf
/tf_static
```

两者输入模型不同：

- `luxi_-lunar` 使用双目图像和语义 mask。
- D435i 标准建图更适合使用 RGB + aligned depth。

## 3. 整体算法链路

参考代码主线可以概括为：

```text
D455 left/right RGB
  -> grayscale_converter_node
  -> left/right mono images
  -> lightglue_vo_node
  -> /lac/vo/odom
  -> odom_frame_aligner_node
  -> /lac/vo_aligned/odom

D455 left/right mono + semantic mask + odom
  -> semantic_point_cloud_node
  -> /lac/semantic_points

/lac/vo/odom 或 /lac/vo_aligned/odom
/lac/semantic_points
  -> live_slam_backend_node
  -> /lac/slam/odom
  -> /lac/slam/path
  -> /lac/slam/semantic_map

left/right semantic mask
  -> stereo_rock_obstacle_node
  -> /lac/rock_obstacles

goal + odom + rock obstacles
  -> arc_planner_node
  -> /cmd_vel
```

## 4. 视觉里程计实现：lightglue_vo_node

源码文件：

```text
ros2_ws/src/lac_omnilrs_adapter/lac_omnilrs_adapter/lightglue_vo_node.py
```

节点：

```text
lightglue_vo_node
```

输入：

```text
/OmniLRS/Robots/husky/d455/left/image_mono
/OmniLRS/Robots/husky/d455/right/image_mono
/OmniLRS/Robots/husky/d455/left/camera_info
/OmniLRS/Robots/husky/d455/right/camera_info
```

输出：

```text
/lac/vo/odom
/lac/vo/path
```

### 4.1 灰度图输入

`lightglue_vo_node` 不直接使用 RGB 图，而是使用左右灰度图：

```text
left/image_raw  -> left/image_mono
right/image_raw -> right/image_mono
```

转换由 `grayscale_converter_node` 完成。

### 4.2 SuperPoint 特征提取

节点初始化时创建：

```python
self.extractor = SuperPoint(max_num_keypoints=...)
self.matcher = LightGlue(features="superpoint")
```

对左右图像分别提取特征点和描述子。

### 4.3 LightGlue 双目匹配

函数 `_process_stereo()` 中对左图和右图做特征匹配：

```python
matches = match_feats(self.matcher, feats_left, feats_right, ...)
```

得到左右图中对应的 keypoint。

### 4.4 双目三角化

代码使用左右 keypoint 的 x 坐标差计算 disparity：

```python
disparity = left_pt[0] - right_pt[0]
```

再根据双目模型计算 3D 点：

```python
z = fx * baseline_m / disparity
x = (left_pt[0] - cx) * z / fx
y = (left_pt[1] - cy) * z / fy
```

其中：

- `fx, fy, cx, cy` 来自 `CameraInfo`；
- `baseline_m` 默认是 `0.0957 m`；
- `z` 是深度；
- `x, y, z` 是相机坐标系下的 3D 点。

这一步说明参考项目的主线深度来源是双目 disparity，不是 RealSense depth image。

### 4.5 时间相邻帧运动估计

当前帧和上一帧之间继续用 LightGlue 匹配左图特征。

然后使用：

```text
上一帧 3D 点
当前帧 2D keypoint
```

形成 3D-2D 对应关系。

### 4.6 PnP 求解相机运动

函数 `_estimate_motion()` 中使用 OpenCV：

```python
cv2.solvePnPRansac(...)
```

作用：

```text
上一帧 3D 点 + 当前帧 2D 点
  -> PnP
  -> 当前帧相对上一帧的相机运动
```

### 4.7 累积位姿并发布 odom

估计出的帧间运动被累积到：

```python
self.world_t_camera
```

然后发布：

```text
/lac/vo/odom
/lac/vo/path
```

这部分实现的是双目视觉里程计 VO。

## 5. 语义点云实现：semantic_point_cloud_node

源码文件：

```text
ros2_ws/src/lac_omnilrs_adapter/lac_omnilrs_adapter/semantic_point_cloud_node.py
```

节点：

```text
semantic_point_cloud_node
```

输入：

```text
/OmniLRS/Robots/husky/d455/left/image_mono
/OmniLRS/Robots/husky/d455/right/image_mono
/lac/perception/unetpp/semantic_mask
/lac/vo/odom
/OmniLRS/Robots/husky/d455/left/camera_info
/OmniLRS/Robots/husky/d455/right/camera_info
```

输出：

```text
/lac/semantic_points
```

### 5.1 语义 mask 来源

语义 mask 可以来自：

1. U-Net++ 推理：

```text
/lac/perception/unetpp/semantic_mask
```

2. OmniLRS 模拟器 ground-truth semantic segmentation：

```text
/OmniLRS/Robots/husky/d455/left/semantic_segmentation
```

代码支持的 mask 编码包括：

```text
mono8
8UC1
32SC1
```

默认语义类别包括：

```text
1 rock
2 lander_or_fiducial
3 ground
4 background
```

默认保留：

```text
keep_labels = 1,2,3
```

也就是保留岩石、着陆器/标志物、地面，过滤背景。

### 5.2 双目三角化生成 3D 点

`semantic_point_cloud_node` 内部也使用 SuperPoint + LightGlue 做左右图匹配。

然后和 VO 节点类似，根据 disparity 计算 3D 点：

```python
z = fx * baseline_m / disparity
x = (u - cx) * z / fx
y = (v - cy) * z / fy
```

### 5.3 语义标签赋值

每个左图 keypoint 有像素坐标 `(u, v)`。

节点在语义 mask 上采样对应像素的类别：

```text
keypoint pixel
  -> semantic mask
  -> label
```

然后生成：

```python
SemanticPoint(x, y, z, label)
```

### 5.4 转换到 odom/map 坐标系

相机坐标系点先转换到导航坐标约定：

```text
camera optical: x-right, y-down, z-forward
nav/base:       x-forward, y-left, z-up
```

再使用 odom 位姿转到输出 frame：

```text
camera point
  -> nav local point
  -> odom/map point
```

### 5.5 输出 PointCloud2

最终发布：

```text
/lac/semantic_points
```

字段包括：

```text
x, y, z, rgb, label
```

这是稀疏语义点云，不是 2D occupancy map。

## 6. SLAM 后端实现：live_slam_backend_node

源码文件：

```text
ros2_ws/src/lac_omnilrs_adapter/lac_omnilrs_adapter/live_slam_backend_node.py
```

节点：

```text
live_slam_backend_node
```

输入：

```text
/lac/vo/odom
/lac/semantic_points
/OmniLRS/Robots/husky/d455/left/image_mono
/OmniLRS/Robots/husky/d455/right/image_mono
/OmniLRS/Robots/husky/d455/left/camera_info
/OmniLRS/Robots/husky/d455/right/camera_info
```

输出：

```text
/lac/slam/odom
/lac/slam/path
/lac/slam/semantic_map
/lac/slam/status
```

这是参考代码中最接近“SLAM 建图”的模块。

### 6.1 使用 VO 作为初始轨迹

节点订阅：

```text
/lac/vo/odom
```

或在推荐 launch 中使用对齐后的：

```text
/lac/vo_aligned/odom
```

每收到 VO，就更新当前位姿，并发布 SLAM 位姿：

```text
/lac/slam/odom
/lac/slam/path
```

如果没有 GTSAM，则退化为 odometry-chain，也就是直接沿用 VO 轨迹链。

### 6.2 Keyframe 机制

代码使用关键帧机制。

默认参数：

```text
keyframe_min_translation_m = 0.25
keyframe_min_yaw_deg = 5.0
```

含义：

```text
移动超过 0.25 m 或转角超过 5 度
  -> 新增 keyframe
```

keyframe 保存：

```python
Keyframe(index, stamp_sec, vo_pose)
```

### 6.3 语义观测绑定到 keyframe

节点订阅：

```text
/lac/semantic_points
```

将语义点保存为 semantic observations：

```python
SemanticObservation(
    keyframe_index,
    local_point,
    label
)
```

这样做的意义是：

- 地图点和关键帧绑定；
- 如果关键帧位姿后续被优化，地图点可以跟随优化后的 keyframe pose 重新投到全局坐标。

### 6.4 可选 GTSAM 因子图优化

代码会尝试导入：

```python
import gtsam
```

如果 GTSAM 可用，则使用因子图优化。

图中包含：

- prior factor；
- VO relative pose between factors；
- loop closure between factors。

核心结构：

```text
Keyframe poses
  -> GTSAM Values

VO constraints
  -> BetweenFactorPose3

Loop closure constraints
  -> BetweenFactorPose3

Optimizer
  -> optimized poses
```

代码中使用：

```python
gtsam.NonlinearFactorGraph()
gtsam.Values()
gtsam.PriorFactorPose3(...)
gtsam.BetweenFactorPose3(...)
gtsam.LevenbergMarquardtOptimizer(...)
```

如果 GTSAM 不可用，则使用 odometry-chain fallback。

### 6.5 回环检测

回环检测也基于 LightGlue。

节点会保存 visual keyframe：

```python
VisualKeyframe(
    keyframe_index,
    stamp_sec,
    pose,
    features
)
```

新 keyframe 到来时，会从旧 keyframe 中找候选：

```text
keyframe 间隔足够大
距离小于 loop_distance_threshold_m
yaw 差小于 loop_yaw_threshold_deg
```

候选 keyframe 之间做：

```text
LightGlue feature matching
  -> PnP 验证
  -> inlier 检查
  -> 接受或拒绝 loop edge
```

接受的 loop edge 会加入 GTSAM 优化图。

### 6.6 语义地图发布

函数 `_publish_semantic_map()` 会将所有 semantic observations 根据当前 keyframe pose 投到全局坐标。

然后使用 voxel 聚合：

```python
VoxelAccumulator
```

默认参数：

```text
semantic_map_voxel_size_m = 0.15
semantic_map_min_observations = 1
```

聚合流程：

```text
semantic observations
  -> transform by optimized keyframe pose
  -> voxel bucket
  -> centroid
  -> label majority vote
  -> PointCloud2
```

最终发布：

```text
/lac/slam/semantic_map
```

地图类型：

```text
PointCloud2 with semantic label
```

不是 Nav2 标准的：

```text
nav_msgs/OccupancyGrid /map
```

## 7. 持久语义地图：persistent_semantic_mapper_node

源码文件：

```text
ros2_ws/src/lac_omnilrs_adapter/lac_omnilrs_adapter/persistent_semantic_mapper_node.py
```

节点：

```text
persistent_semantic_mapper_node
```

输入：

```text
/lac/semantic_points
```

输出：

```text
/lac/semantic_map
/lac/rock_grid
/lac/elevation_grid
```

### 7.1 Voxel 体素累积

节点把语义点按体素分桶：

```text
voxel_size_m = 0.15
```

内部结构：

```python
self.voxels: dict[tuple[int, int, int], VoxelStats]
```

每个 voxel 统计：

- 点数量；
- label 分布；
- 中心点；
- 高度信息。

### 7.2 发布语义点云地图

输出：

```text
/lac/semantic_map
```

类型：

```text
sensor_msgs/msg/PointCloud2
```

字段同样包含：

```text
x, y, z, rgb, label
```

### 7.3 发布 rock_grid 和 elevation_grid

它还会生成两个 `nav_msgs/msg/OccupancyGrid`：

```text
/lac/rock_grid
/lac/elevation_grid
```

用途：

- `rock_grid` 表示岩石风险/占据；
- `elevation_grid` 表示高度分布。

注意：

这两个 grid 是语义辅助栅格，不是完整 Nav2 静态地图流程。代码没有实现 Nav2 map_server 所需的标准地图保存、加载和定位闭环。

## 8. 岩石障碍物提取：stereo_rock_obstacle_node

源码文件：

```text
ros2_ws/src/lac_omnilrs_adapter/lac_omnilrs_adapter/stereo_rock_obstacle_node.py
```

输入：

```text
left semantic mask
right semantic mask
left camera_info
right camera_info
```

输出：

```text
/lac/rock_obstacles
```

实现思路：

```text
left/right semantic mask
  -> 找 rock 连通域
  -> 左右 rock 区域中心匹配
  -> 计算 disparity
  -> 使用 baseline 估计深度
  -> 输出机器人局部坐标下的岩石圆形障碍物
```

输出 PointCloud2 字段：

```text
x, y, z, radius, disparity, area
```

该结果给 `arc_planner_node` 做避障。

## 9. 导航控制实现：arc_planner_node

源码文件：

```text
ros2_ws/src/lac_omnilrs_adapter/lac_omnilrs_adapter/arc_planner_node.py
```

节点：

```text
arc_planner_node
```

输入：

```text
odom
goal
/lac/rock_obstacles
```

输出：

```text
/cmd_vel
/lac/arc_planner/selected_arc
```

### 9.1 轨迹采样

代码生成多条固定速度、不同角速度的弧线轨迹：

```python
dubins_traj(v, omega, duration, dt)
```

默认类似：

```text
target_speed_mps = 0.15
max_omega_radps = 0.8
omega_samples = 41
arc_duration_sec = 6.0
```

### 9.2 目标转换到局部坐标

全局 goal 根据当前 odom 转换到机器人局部坐标：

```python
goal_to_local(...)
```

### 9.3 弧线打分与碰撞过滤

每条候选弧线会被评估：

- 是否靠近目标；
- heading 是否合理；
- 是否碰撞岩石障碍物；
- 是否满足安全半径。

碰撞障碍来自：

```text
/lac/rock_obstacles
```

### 9.4 发布速度

选择最优弧线后发布：

```text
/cmd_vel
```

并发布可视化路径：

```text
/lac/arc_planner/selected_arc
```

这不是 Nav2 的 planner/controller/costmap 架构，而是独立局部规划器。

## 10. 参考项目地图类型总结

参考项目输出的“地图”主要有：

| 话题 | 类型 | 含义 | 是否 Nav2 标准地图 |
| --- | --- | --- | --- |
| `/lac/semantic_points` | `PointCloud2` | 稀疏语义点云 | 否 |
| `/lac/slam/semantic_map` | `PointCloud2` | SLAM 后端优化后的语义点云地图 | 否 |
| `/lac/semantic_map` | `PointCloud2` | 持久语义点云地图 | 否 |
| `/lac/rock_grid` | `OccupancyGrid` | 岩石占据/风险辅助栅格 | 不是完整 Nav2 地图 |
| `/lac/elevation_grid` | `OccupancyGrid` | 高度辅助栅格 | 不是完整 Nav2 地图 |

Nav2 标准导航通常需要：

```text
/map nav_msgs/msg/OccupancyGrid
map.yaml
map.pgm
map -> odom -> base_link
```

参考项目没有完整走这条标准路线。

## 11. 与 D435i + Nav2 目标的区别

| 对比项 | luxi_-lunar | D435i + RTAB-Map + Nav2 |
| --- | --- | --- |
| 相机来源 | OmniLRS D455 双目模拟 | D435i 实机 |
| 深度来源 | 双目 disparity | RealSense aligned depth |
| 里程计 | SuperPoint + LightGlue + PnP | RTAB-Map RGB-D odometry |
| 建图 | 语义 PointCloud2 / voxel map | 2D occupancy grid + 3D map |
| 回环 | LightGlue + GTSAM hooks | RTAB-Map 内置回环 |
| 导航 | click_to_go / arc planner | Nav2 |
| 地图格式 | 非 Nav2 标准主地图 | Nav2 可用 `/map` |
| 语义依赖 | 强依赖 | 初期不需要 |

## 12. 对当前项目的实现建议

如果目标是：

```text
D435i 实机 + 建图 + Nav2 导航
```

建议不要直接复用 `luxi_-lunar` 的建图主线，而应采用：

```text
D435i RGB + aligned depth
  -> RTAB-Map
  -> /map
  -> Nav2
```

原因：

- 当前 D435i 已经能稳定输出 RGB、depth、PointCloud、TF；
- RTAB-Map 更适合 RealSense RGB-D；
- RTAB-Map 可以输出 Nav2 可用的 2D map；
- Nav2 需要标准 `/map` 和 TF，而不是语义 PointCloud2；
- 去掉语义部分后，`luxi_-lunar` 的核心优势会被削弱。

参考项目可借鉴的部分：

- 系统分层方式；
- RViz/Foxglove 调试习惯；
- keyframe 和状态发布思路；
- 后续语义扩展方向；
- 岩石障碍物提取和局部规划思想。

不建议直接迁移的部分：

- `lightglue_vo_node` 作为主里程计；
- `live_slam_backend_node` 作为 Nav2 地图源；
- `arc_planner_node` 替代 Nav2；
- U-Net++ 语义链路。

## 13. 一句话总结

`luxi_-lunar` 的建图是：

```text
双目图像 + SuperPoint/LightGlue + PnP VO + 语义 mask + keyframe/GTSAM 后端
  -> voxelized semantic PointCloud2 map
```

不是：

```text
D435i depth image + standard 2D occupancy grid SLAM + Nav2
```

因此，当前 D435i 项目如果要实现建图 + Nav2，推荐基于 RTAB-Map 重新搭建标准 RGB-D SLAM 和 Nav2 链路，而不是直接改造参考项目的语义 SLAM。

