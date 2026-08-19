# SuperPoint + LightGlue 视觉前端与 RTAB-Map 后端集成设计

## 1. 文档目的

本文档设计一套独立的学习型 RGB-D 视觉里程计前端，使用
SuperPoint + LightGlue 替代当前 RTAB RGB-D odometry 中的 ORB 特征检测和匹配，
同时保留 RTAB-Map 作为关键帧、回环、位姿图优化和数据库管理后端。

目标运行场景是大面积草地。这类场景存在：

- ORB 可重复检测角点数量少；
- 草地纹理大量重复，容易产生错误匹配；
- 草叶随风摆动，不符合完全静态世界假设；
- 地面局部几何接近平面，位姿估计容易退化；
- 室外阳光可能降低 D435i 主动红外深度的有效距离和稳定性；
- 建图时既需要连续里程计，也需要为后续 HLoc 保留足够的 RGB 关键帧。

设计不假设 SuperPoint + LightGlue 必然解决草地的所有问题。它们需要与深度几何验证、
IMU/轮速先验、跟踪丢失检测和 RTAB 回环后端组合使用。

### 1.1 先区分三种“信息量”

ORB 关键点少和点云地图稀疏有关联，但不是同一个量：

| 信息层 | 来源 | 主要用途 | 数量由什么决定 |
|---|---|---|---|
| 稀疏视觉特征 | ORB 或 SuperPoint | 帧间运动、视觉回环 | 图像纹理、检测器、阈值、关键点上限 |
| 稠密几何点云 | D435i 深度图 | 障碍物、表面地图、ICP | 深度有效像素、距离、阳光、降采样和体素大小 |
| RGB 关键帧 | 彩色图像和相机位姿 | HLoc、纹理、人工检查 | RTAB 关键帧策略和图像保存策略 |

因此，换成 SuperPoint + LightGlue 的直接收益是增加和改善**可用于位姿估计的约束**，降低里程计因
ORB 点不足而丢失的概率；它不会凭空增加 D435i 的深度像素，也不会直接让每帧点云变密。

要保存尽可能多的建图信息，需要同时处理两条链路：

```text
视觉约束链：SuperPoint/LightGlue -> odometry -> RTAB位姿图和回环
稠密地图链：有效Depth -> 点云投影 -> 降采样/融合 -> PLY/OctoMap/障碍层
```

点云密度需要另外检查 RTAB 的深度 decimation、最大深度、噪声过滤、voxel size 和地图关键帧采样。
但不能简单把所有参数调到“最密”：无效深度和重复观测也会增加内存、重影和 ICP 误匹配。

## 2. 当前系统的实际状态

当前建图 launch 启动 `rtabmap_launch/rtabmap.launch.py`，并设置：

```text
visual_odometry=true
icp_odometry=false
Vis/FeatureType=2       # ORB
Vis/MaxFeatures=1000
Odom/ImageDecimation=2
```

因此当前 RTAB 同时承担了两类职责：

```text
RTAB RGB-D odometry
├── ORB 检测与匹配
├── RGB-D 几何位姿估计
└── odom -> base_link

RTAB-Map SLAM
├── 关键帧与 Signature
├── 短期/长期记忆
├── 回环检测
├── 位姿图优化
├── map -> odom
└── mapNNN.db
```

当前 HLoc 已经独立使用：

```text
NetVLAD + SuperPoint + LightGlue + OpenCV PnP + Depth Verification
```

但 HLoc 是低频全局重定位器，不是高频视觉里程计。它不能直接代替 15‑30 Hz
的 `odom -> base_link` 连续运动估计。

## 3. 为什么新建独立视觉前端

### 3.1 不建议直接修改 RTAB 源码

当前 RTAB 二进制版本没有启用 SuperPoint Torch 和 Python 特征接口。LightGlue 也不是
`Vis/FeatureType` 可以直接选择的标准匹配器。如果直接修改 RTAB，需要同时维护：

- RTAB 自定义分支；
- LibTorch/CUDA/aarch64 ABI；
- RTAB 特征数据结构与 Torch tensor 转换；
- LightGlue 匹配结果到 RTAB VisualRegistration 的适配；
- RTAB 版本升级后的重复合并；
- HLoc 和里程计两套模型运行时。

这会把前端算法和后端数据库强绑定，不利于单独测试、性能优化和回退。

### 3.2 独立前端的优点

新建 `luxi_visual_frontend` 后，系统可以使用稳定 ROS 接口解耦：

```text
luxi_adapter RGB-D/IMU
          │
          ▼
luxi_visual_frontend
├── SuperPoint
├── LightGlue
├── PnP/RANSAC
├── 深度验证
└── /luxi_visual_frontend/odom
          │
          ▼
RTAB-Map backend
├── 关键帧
├── 回环
├── 图优化
└── mapNNN.db
```

这种方式可以：

- 使用同一 rosbag 对比 ORB 和学习型前端；
- 不修改 RTAB 核心源码；
- 前端失败时通过 config 回退到 ORB；
- 单独记录关键点数、匹配数、PnP 内点和 GPU 延迟；
- 先用 PyTorch 验证，后续再转 TensorRT/C++；
- 未来可以替换成 DISK、ALIKED 或其他特征，不影响 RTAB 后端。

## 4. 前端与后端的职责边界

| 模块 | 职责 | 不负责 |
|---|---|---|
| `luxi_adapter` | 统一 RGB、深度、CameraInfo、IMU、TF、硬件时间戳 | 里程计、回环 |
| `luxi_visual_frontend` | 相邻帧/关键帧匹配、米制运动估计、跟踪质量、`odom -> base_link` | 全局回环、长期地图优化 |
| RTAB-Map backend | 地图关键帧、回环约束、位姿图优化、`map -> odom`、`.db` | 高频局部帧间匹配 |
| HLoc | 对已有地图进行低频全局粗定位 | 连续里程计、建图后端 |
| `luxi_location` | 已有地图上的点云 ICP 精配准 | 新地图回环优化 |
| 导航层 | 局部避障、全局路径、跟随和安全准入 | 视觉特征匹配 |

后端不应再启动第二个 RGB-D odometry，否则会出现两套 `odom -> base_link` 和重复算力。

## 5. 学习型前端内部流程

### 5.1 输入

```text
/sensors/rgbd/color/image_raw
/sensors/rgbd/depth/image_raw
/sensors/rgbd/color/camera_info
/sensors/imu/data
可选：底盘轮速里程计
TF: base_link -> camera_color_optical_frame
```

要求 RGB 和深度时间差不超过 50 ms，深度已与 RGB 像素对齐。如果更换硬件后使用原始深度，
必须改用 depth CameraInfo 并显式处理彩色-深度外参。

### 5.2 特征提取与匹配

```text
当前 RGB
   │
   ▼
SuperPoint
├── keypoints: N x 2
├── descriptors: 256 x N
└── scores: N
   │
   ▼
LightGlue(当前帧, 参考关键帧/局部地图)
   │
   ▼
匹配对 (u_ref,v_ref) <-> (u_cur,v_cur)
```

不建议只与上一原始帧匹配。生产前端应保留一个局部关键帧，并在运动较小时跟踪同一参考。
这样可避免帧间距离太小时几何基线不足，也可避免长时间不更新导致视角差过大。

### 5.3 RGB-D 米制位姿估计

对参考关键帧匹配点读取深度 `z_ref`：

```text
x_ref = (u_ref-cx)*z_ref/fx
y_ref = (v_ref-cy)*z_ref/fy
p_ref = [x_ref,y_ref,z_ref]
```

建立：

```text
参考帧 3D p_ref <-> 当前帧 2D (u_cur,v_cur)
```

通过 PnP/RANSAC 求得：

```text
T_current_camera_reference_camera
```

再取逆得到参考相机到当前相机的运动：

```text
T_reference_camera_current_camera
    = inverse(T_current_camera_reference_camera)
```

位姿累积：

```text
T_odom_camera_current
    = T_odom_camera_reference * T_reference_camera_current_camera
```

通过已知相机外参转成：

```text
T_odom_base
    = T_odom_camera * T_camera_base
```

可选使用当前帧深度建立 3D-3D 匹配，但草地和室外无效深度较多时，3D-2D PnP 通常可保留更多对应。
当前帧深度应用于结果验证，不应为了凑内点而无限放宽几何阈值。

### 5.4 准入和丢失检测

每帧需要统计：

```text
SuperPoint 关键点数
LightGlue 匹配数
具有有效参考深度的匹配数
RANSAC 内点数和比例
重投影 RMSE
内点在图像中的空间覆盖率
估计运动与 IMU/轮速预测的差值
处理墙钟时间和 CPU 时间
```

不能只以“匹配数多”判定成功。草地重复纹理可能产生大量错误匹配，而摆动草叶上的点不应主导相机运动。

建议的状态：

```text
INITIALIZING
TRACKING
LOW_PARALLAX
DEGRADED
LOST
RELOCALIZING
```

`LOST` 时不得继续积累未验证位姿，也不得突然重置 `odom` 原点。短时丢失可使用 IMU/轮速预测；
长时丢失需要 RTAB 回环或 HLoc 针对已有稳定地图重定位。

### 5.5 关键帧策略

前端关键帧与 RTAB 地图关键帧是两个层次：

- 前端关键帧：用于局部匹配和里程计；
- RTAB 关键帧：用于回环、长期图优化和数据库。

前端可在以下任一条件满足时更换参考关键帧：

- 平移或旋转超过阈值；
- 匹配内点下降；
- 图像覆盖率下降；
- 时间间隔超过上限；
- 新帧清晰度和深度有效率明显更好。

不建议每一帧都运行完整 LightGlue 并立即设为新关键帧。草地需要较大视差来提供几何约束，
但视差过大又会降低特征匹配。

## 6. ROS 接口合同

### 6.1 前端输出

```text
/luxi_visual_frontend/odom
  type: nav_msgs/msg/Odometry
  header.frame_id: odom
  child_frame_id: base_link

TF: odom -> base_link

/luxi_visual_frontend/status
  type: std_msgs/msg/String

/luxi_visual_frontend/diagnostics
  type: diagnostic_msgs/msg/DiagnosticArray

/luxi_visual_frontend/debug_matches
  type: sensor_msgs/msg/Image
  默认关闭
```

Odometry 时间戳必须对应当前 RGB-D 帧，不能使用处理结束的墙钟时间。协方差需要随内点、重投影误差和退化程度变化，
不能长期固定写零。

### 6.2 TF 所有权

```text
map                         # RTAB backend
 └── odom                    # RTAB backend发布 map -> odom
      └── base_link              # luxi_visual_frontend发布 odom -> base_link
           └── camera_link       # luxi_adapter发布静态/传感器TF
                └── camera_color_optical_frame
```

同一时刻只允许一个节点发布每条 TF：

- 关闭 RTAB 自带 `rgbd_odometry`；
- 前端是唯一 `odom -> base_link` 发布者；
- RTAB 后端是唯一 `map -> odom` 发布者；
- `luxi_adapter` 是传感器外参 TF 所有者；
- HLoc 只发布位姿消息，不直接抢占连续 TF。

## 7. RTAB 后端接入方法

### 7.1 launch 改造原则

当前 `rgbd_mapping.launch.py` 包含：

```text
visual_odometry=true
odom_topic=odom
```

新增一个独立 launch，不直接破坏现有 ORB 回退链路：

```text
project/luxi_RTAB_Map/launch/rgbd_mapping_learned_odom.launch.py
```

它包含：

```text
luxi_visual_frontend/visual_odometry.launch.py
rtabmap_launch/rtabmap.launch.py
```

RTAB launch 关键参数：

```text
visual_odometry=false
icp_odometry=false
odom_frame_id=""                         # 使用Odometry话题
odom_topic=/luxi_visual_frontend/odom
frame_id=base_link
map_frame_id=map
rgbd_sync=true
rgb_topic=/sensors/rgbd/color/image_raw
depth_topic=/sensors/rgbd/depth/image_raw
camera_info_topic=/sensors/rgbd/color/camera_info
```

RTAB 在这种模式下仍然订阅 RGB-D，用于：

- 创建地图 Signature；
- 保存 RGB 和深度；
- 检测回环；
- 生成地图点云；
- 为后续 HLoc 导出带位姿的 RGB-D 关键帧。

但它不再自己计算高频帧间 ORB odometry，而是使用外部 `nav_msgs/Odometry`。

### 7.2 前后端配合过程

```text
t0 RGB-D
  │
  ├── frontend: 初始化 odom
  └── RTAB: 创建第一地图节点

t1 RGB-D
  │
  ├── frontend: SuperPoint + LightGlue + PnP
  │              └── 发布 T_odom_base(t1)
  └── RTAB: 使用该odom决定是否创建新地图节点

tN RGB-D
  │
  ├── frontend: 继续局部里程计，允许缓慢漂移
  └── RTAB: 发现回环并优化整张位姿图
                    └── 更新 map -> odom 吸收全局修正
```

前端不需要因为 RTAB 回环而重写自己的 `odom` 轨迹。RTAB 通过 `map -> odom` 将全局修正与连续局部里程计分离。

### 7.3 两级集成：外部里程计不等于替换全部 RTAB 特征

必须明确：`nav_msgs/Odometry` 只携带位姿、速度和协方差，不携带 SuperPoint 关键点和描述子。
所以阶段3的外部 odometry 方案会替换 RTAB 的高频 RGB-D ORB 里程计，但 RTAB 后端在创建 Signature
和做视觉回环时，仍可能使用自身配置的局部特征/词袋。不能声称仅通过 `/luxi_visual_frontend/odom`
就已经替换了 RTAB 内的全部 ORB。

推荐分两级实施：

| 级别 | 前端给 RTAB 的内容 | RTAB 内部仍做什么 | 适用阶段 |
|---|---|---|---|
| A：外部里程计 | `Odometry` + 原始 RGB-D | Signature、内部视觉/几何回环、图优化 | 首选，先验证草地里程计 |
| B：外部特征 | Odometry、RGB-D、SuperPoint关键点/3D点/描述子 | 使用外部特征建节点和回环、图优化 | A通过后，若后端ORB回环仍不足 |

本机安装的 `rtabmap_msgs/msg/RGBDImage` 和 `SensorData` 消息已经包含：

```text
KeyPoint[] key_points
Point3f[] points
uint8[] descriptors       # 压缩的OpenCV descriptor matrix
GlobalDescriptor          # RGBDImage为单个，SensorData为数组
```

这给级别B提供了消息入口，但并不代表现有 Python 节点把浮点 SuperPoint 描述子填进去就一定可用。
实施前必须用当前 RTAB 版本验证描述子矩阵的类型、维度、压缩格式、字典策略和回环注册路径；必要时增加一个
`rtabmap_feature_bridge`，或维护很小的 `rtabmap_ros` 适配层。这个工作与高频 odometry 解耦，不能阻塞级别A。

当前建议是：

1. 先以级别A解决高频跟踪和建图轨迹连续性；
2. 同时保留 RGB-D，让 RTAB 继续生成稠密点云和数据库；
3. 用 HLoc 的 NetVLAD + SuperPoint + LightGlue 承担已有地图的全局粗定位；
4. 若实测表明 RTAB 内部视觉回环仍是瓶颈，再进入级别B，不在第一版修改 RTAB 核心。

## 8. 模型和第三方依赖放置

### 8.1 第一阶段复用已下载模型

不建议再下载一套重复权重。当前已有：

```text
SuperPoint:
3parts/hloc/third_party/SuperGluePretrainedNetwork/models/weights/
└── superpoint_v1.pth

LightGlue for SuperPoint:
3parts/hloc_models/hub/checkpoints/
└── superpoint_lightglue_v0-1_arxiv.pth

NetVLAD（仅HLoc使用）:
3parts/hloc_models/hub/netvlad/
└── VGG16-NetVLAD-Pitts30K.mat
```

前端需要的是 SuperPoint 和 LightGlue，不需要每帧运行 NetVLAD。NetVLAD 继续只用于 HLoc 低频全局检索。

### 8.2 Python/GPU 运行时

第一阶段复用：

```text
3parts/hloc_gpu_python/    # Jetson PyTorch 2.8.0 / CUDA 12.6
3parts/hloc_python/        # NumPy/HDF5/Kornia及CPU回退
3parts/hloc/               # HLoc SuperPoint wrapper
3parts/lightglue/          # LightGlue源码
3parts/hloc_models/        # TORCH_HOME和模型缓存
```

如果后续生成 FP16/ONNX/TensorRT 文件，也统一放到共享模型目录，而不是 ROS 包中：

```text
3parts/hloc_models/export/
├── superpoint/
│   ├── superpoint_fp16.onnx
│   └── superpoint_orin_fp16.engine
└── lightglue/
    ├── lightglue_superpoint_fp16.onnx
    └── lightglue_superpoint_orin_fp16.engine
```

`.engine` 与 JetPack、TensorRT、CUDA 和 GPU 架构相关，不能视为跨主机通用模型；配置文件应引用绝对路径或
由工作区根目录解析的路径，并在启动时校验权重哈希、TensorRT版本和输入尺寸。

使用与 `luxi_hloc` 相同的加载顺序，保证默认使用 Orin CUDA。模型不应放在：

- `project/luxi_visual_frontend` 源码包内；
- ROS `install` 目录；
- 用户主目录下的不可控 Torch cache；
- 每张地图的 `maps` 目录。

模型是全局共享的算法资产；`maps/hloc_maps/mapNNN` 是场景索引，两者不应混合。

### 8.3 生产化路线

第一阶段：

```text
Python + PyTorch CUDA + OpenCV PnP
```

优点是能最小代价复用已经测试的 HLoc 模型后端。先验证草地数据上是否真的优于 ORB，再做性能重写。

第二阶段：

```text
FP16 -> ONNX -> TensorRT
C++ ROS节点 + TensorRT engine + OpenCV/Eigen几何
```

迁移前必须对比：

- 关键点位置和分数；
- LightGlue 匹配数；
- PnP 内点和位姿；
- ORB/PyTorch/TensorRT 轨迹的 ATE/RPE；
- FP32 与 FP16 的失败帧差异。

不应在算法准确性尚未通过草地 rosbag 测试时先进行 TensorRT 重写。

## 9. 建议项目结构

```text
project/luxi_visual_frontend/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── superpoint_lightglue_odometry.yaml
│   └── orb_baseline.yaml
├── launch/
│   ├── visual_odometry.launch.py
│   └── offline_replay.launch.py
├── luxi_visual_frontend/
│   ├── __init__.py
│   ├── runtime.py
│   ├── feature_backend.py
│   ├── keyframe_tracker.py
│   ├── pose_estimator.py
│   └── odometry_node.py
├── include/luxi_visual_frontend/
│   ├── motion_filter.hpp
│   └── pose_covariance.hpp
├── src/
│   ├── motion_filter.cpp
│   └── pose_covariance.cpp
├── scripts/
│   ├── check_environment.py
│   └── evaluate_rosbag.py
└── test/
    ├── test_geometry.py
    ├── test_keyframe_policy.py
    ├── test_pose_estimator.py
    └── test_runtime.py
```

第一阶段不要为每一个内部步骤创建 ROS 话题。图像、描述子和匹配数组在单节点进程内传递，避免大量序列化和内存复制。

推荐初期将 GPU 推理、关键帧管理和 PnP 都放在一个 Python 节点中，快速完成 A/B 验证。
只有确认算法有效后，再把模型和几何求解迁移到 C++/TensorRT。

## 10. 建议配置项

```yaml
luxi_visual_frontend:
  ros__parameters:
    device: cuda
    runtime: pytorch

    color_topic: /sensors/rgbd/color/image_raw
    depth_topic: /sensors/rgbd/depth/image_raw
    camera_info_topic: /sensors/rgbd/color/camera_info
    imu_topic: /sensors/imu/data

    odom_topic: /luxi_visual_frontend/odom
    odom_frame: odom
    base_frame: base_link

    resize_max: 800
    max_keypoints: 2048
    lightglue_depth_confidence: 0.90
    lightglue_width_confidence: 0.95

    minimum_matches: 50
    minimum_depth_matches: 30
    minimum_inliers: 25
    minimum_inlier_ratio: 0.25
    maximum_reprojection_rmse: 3.0
    ransac_reprojection_error: 4.0

    keyframe_min_translation: 0.10
    keyframe_min_rotation_deg: 8.0
    keyframe_max_age: 1.0
    keyframe_min_inlier_ratio: 0.40

    target_rate: 15.0
    publish_tf: true
    publish_debug_image: false
```

这些数值是初始工程值，不是最终结论。必须使用草地数据集调整，并将每次参数修改与定量结果一起记录。

## 11. 草地场景的特殊处理

### 11.1 多特征不等于多有效信息

SuperPoint 可能在草地上检测大量点，但草叶摆动和重复纹理可能使其几何价值很低。前端应增加：

- 特征在图像网格中的均匀性限制；
- 深度有效性和深度边缘检查；
- 重投影误差和双向几何一致性；
- IMU/轮速运动门控；
- 对大面积同方向流动特征的动态检查；
- 地面平面退化检测。

### 11.2 不应仅依赖纯视觉

对长时间草地运行，推荐的优先级是：

```text
轮速/底盘里程计 + IMU 预测
                  │
                  ▼
SuperPoint + LightGlue 视觉校正
                  │
                  ▼
RTAB 回环全局优化
```

如果 D435i 在目标草地的阳光下深度有效率持续过低，需要考虑增加激光雷达、更适合室外的双目或其他里程计源。
更换特征模型无法恢复本身已经无效的深度。

## 12. GPU 调度与算力

HLoc 现在约 1 Hz 运行，视觉里程计则需要 10‑30 Hz。不能直接假设现有 HLoc 单次 0.35‑0.55 s 的推理链可以原样在
30 Hz 运行。

建议第一阶段：

```text
相机：30 Hz
学习型前端目标：10‑15 Hz
LightGlue：当前帧与参考关键帧匹配
HLoc：仅未定位/跟踪丢失时运行
ICP：需要全局修正时低频运行
```

优化顺序：

1. 减小输入尺寸和关键点上限；
2. 使用 LightGlue 提前停止；
3. 定位稳定时停止 NetVLAD/HLoc；
4. 验证 PyTorch FP16；
5. 导出 TensorRT engine；
6. 如仍超载，再考虑关键帧使用 SuperPoint/LightGlue，中间帧使用光流。

## 13. 边建图边导航的配合方式

### 13.1 可行，但不能将所有地图当作静态文件

边建图边导航需要同时维护：

```text
连续 odom -> base_link
RTAB 增量位姿图
可随回环改变的 map -> odom
实时局部障碍层
可版本化的全局地图
已发布路径的失效和重规划
```

推荐分层：

```text
当前 RGB-D/点云
        │
        ├──> 实时局部障碍层 -> 局部避障
        │
        └──> RTAB 增量地图
                     │
                     ├── 小修改：更新全局层
                     └── 回环大修改：发布新地图版本并重规划
```

导航不应直接读取正在写入的 RTAB `.db`，也不应在每帧后完整重建 PLY、OctoMap 和 HLoc 索引。

### 13.2 HLoc 索引在增量建图中的位置

当前 HLoc 索引是静态地图快照：

```text
maps/hloc_maps/mapNNN
```

在正在建图的新区域，连续位姿依靠前端 odometry + RTAB backend，不依赖 HLoc。HLoc 主要用于：

- 机器人启动时进入已有地图；
- 长时跟踪丢失后重定位；
- 返回已建区域时提供全局候选。

当新地图积累到一定量后，在后台建立新索引：

```text
mapNNN_hloc_v1   # 当前在线版本
mapNNN_hloc_v2.tmp
        │
        └── 完整导出、特征提取、校验
                    │
                    ▼
              原子切换为 v2
```

不能让在线 HLoc 读取正在写入的 HDF5 文件。

## 14. 项目总体目录安排

```text
lunar_slam/
├── 3parts/
│   ├── hloc/
│   ├── lightglue/
│   ├── hloc_models/
│   ├── hloc_gpu_python/
│   └── hloc_python/
├── project/
│   ├── luxi_adapter/             # 硬件适配
│   ├── luxi_visual_frontend/     # 新建：学习型视觉里程计
│   ├── luxi_RTAB_Map/            # RTAB后端和建图launch
│   ├── luxi_hloc/                # 静态地图全局粗定位
│   ├── luxi_location/            # ICP精定位
│   ├── luxi_voxel_navigation/    # 体素导航
│   └── luxi-web-control/         # 网页控制
├── maps/
│   ├── rtab_maps/                # RTAB数据库
│   ├── hloc_maps/                # HLoc场景索引
│   └── octo_maps/                # PLY/OctoMap地图快照
└── docs/
    └── superpoint_lightglue_rtabmap_frontend_backend_design.md
```

## 15. 测试与验收方法

### 15.1 不允许仅用建图点数评价

必须使用同一组草地 rosbag 对比：

```text
方案A：当前 RTAB ORB odometry
方案B：SuperPoint + LightGlue odometry
```

每个方案输出：

| 类别 | 指标 |
|---|---|
| 特征 | 检测数、匹配数、深度有效数、内点覆盖率 |
| 里程计 | 跟踪成功率、丢失次数、RPE、ATE、短时跳变 |
| 地图 | 有效关键帧、墙/地面重影、点云覆盖、回环数 |
| 定位 | HLoc Recall@5、PnP内点、ICP fitness/RMSE |
| 性能 | 平均/P95延迟、CPU、GPU、内存、功耗、温度 |

如果没有外部真值，至少应用：

- 轮速/IMU 融合轨迹作独立对照；
- 往返回到起点后的闭合误差；
- 地图中已知距离标志点；
- 独立留出路线的定位成功率。

### 15.2 进入实时建图的准入条件

SuperPoint + LightGlue 前端在接管正式建图前，至少应满足：

1. 同一 rosbag 可重现运行，没有非确定随机跳变；
2. 跟踪丢失次数显著少于 ORB 基线；
3. 位姿误差不因错误草地匹配而增大；
4. P95 处理时间小于前端目标周期；
5. 跟踪丢失时不发布未验证运动；
6. `odom -> base_link` 连续且只有一个发布者；
7. RTAB 回环后 `map -> odom` 修正不影响 odom 连续性；
8. 启动、停止、相机断流和节点重启均有明确状态。

## 16. 分阶段实施计划

### 阶段0：固化 ORB 基线

- 录制草地 RGB-D + IMU + 轮速 rosbag；
- 保存当前 ORB 关键点、内点、轨迹和资源数据；
- 将当前 launch 参数作为不可变基线。

验收：同一 rosbag 重复运行可得到可对比结果。

### 阶段1：离线 SuperPoint + LightGlue 运动估计

- 复用已下载模型；
- 实现关键帧、LightGlue、PnP/RANSAC 和深度验证；
- 输出 TUM/KITTI 轨迹和每帧诊断 JSON；
- 不接管 ROS TF。

验收：离线指标确认优于 ORB 基线。

### 阶段2：独立 ROS odometry 节点

- 订阅 `luxi_adapter` 统一话题；
- 发布 `/luxi_visual_frontend/odom`、TF 和诊断；
- 验证时间戳、协方差和丢失状态；
- 仍不启动 RTAB 建图。

验收：连续长时运行、无TF竞争、无队列累积。

### 阶段3：RTAB 外部里程计后端

- 新增 `rgbd_mapping_learned_odom.launch.py`；
- 设置 `visual_odometry=false`；
- 将 `odom_topic` 切到学习型前端；
- 验证地图关键帧、回环、位姿图和 `.db`；
- 保留一键回退 ORB launch。

验收：使用相同路线构建的地图质量优于 ORB 基线。

### 阶段4：性能优化

- 限制关键点和图像尺寸；
- 验证 FP16；
- 根据数据决定是否 TensorRT；
- 与 HLoc/ICP 进行 GPU 错峰调度；
- 使用 `tegrastats` 做长时温度和功耗测试。

验收：前端达到目标频率，不拖慢相机、RTAB、导航和控制回路。

### 阶段5：边建图边导航

- 实时局部障碍层与全局地图分离；
- RTAB 地图使用版本/快照发布；
- 回环大修正触发路径重规划；
- HLoc 索引后台构建并原子切换；
- 导航状态机区分 `MAPPING`、`NAVIGATING`、`RELOCALIZING`和`DEGRADED`。

验收：建图、回环和导航同时运行时无TF竞争、无半成品地图读取、路径能在地图大修正后失效并重规划。

## 17. 实施前需要确认的决策

本文档只完成设计，没有开始新前端代码编写。按照当前工程规范，实施前需要明确：

1. 是否有可重复的草地 rosbag，或允许先录制基线数据；
2. 是否有轮速里程计可作运动先验；
3. 第一阶段目标频率选择 10 Hz 还是 15 Hz；
4. 第一版是否只做 RGB-D PnP，不同时引入 IMU 滤波器重写；
5. 是否同意保留原 ORB launch 作为显式回退方案；
6. 边建图边导航是否放在前端稳定后的独立阶段。

推荐默认答案是：

```text
先录制草地基线 rosbag；
第一版目标 10 Hz；
使用 RGB-D PnP + 已有 IMU/轮速预测，不重写整套融合器；
保留 ORB 回退 launch；
先完成前端与 RTAB 后端验收，再开始边建图边导航。
```
