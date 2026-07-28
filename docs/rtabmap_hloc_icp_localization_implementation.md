# RTAB-Map + HLoc + ICP 全局定位实现方案

## 1. 文档目的

本文给出 `lunar_slam` 使用 RTAB-Map 建图、HLoc 全局粗定位、Open3D ICP 精配准和
RGB-D 里程计连续跟踪的具体实现方案。

目标是在不要求用户在网页地图上点击初始位置的情况下，实现：

1. 机器人从已建地图中的未知位置启动；
2. 机器人缓慢移动或转动后自动找到地图位置；
3. 粗定位通过几何验证后才交给 ICP；
4. ICP 输出可信位姿后才允许发送导航目标；
5. 定位成功后由高频 RGB-D 里程计保持连续运动估计；
6. 定位丢失时自动重新运行 HLoc，而不是继续使用过期位姿。

本文同时记录设计和当前实现。`project/luxi_hloc` 已于 2026-07-28 完成第一阶段集成与
Jetson GPU 实机测试。最终代码使用 HLoc 的 NetVLAD、SuperPoint、LightGlue，并以
OpenCV `solvePnPRansac` 代替 `pycolmap` 求解绝对位姿；RGB-D 地图点直接由 RTAB
优化位姿和关键帧深度生成，因此不需要另建 COLMAP 稀疏重建。

实现、启动指令和实测数据分别见：

- `project/luxi_hloc/README.md`
- `project/luxi_hloc/docs/test_report.md`

## 2. 方案结论

推荐职责划分如下：

| 组件 | 职责 | 运行频率 |
|---|---|---:|
| `luxi_adapter` | 提供与硬件型号无关的 RGB、深度、内参、IMU 和 TF | 传感器原始频率 |
| RTAB RGB-D odometry | 估计 `odom -> base_link` 连续相对运动 | 约 30 Hz |
| RTAB-Map mapping | 创建 `.db`、优化关键帧位姿、导出点云 | 仅建图时 |
| HLoc retrieval | 从全地图关键帧检索候选位置 | 未定位时约 1 Hz |
| SuperPoint + LightGlue | 验证 Top-K 候选图像 | 按候选触发 |
| PnP/RANSAC | 计算 `map -> camera` 六自由度粗位姿 | 按候选触发 |
| 深度几何验证 | 拒绝视觉相似但几何错误的候选 | 按候选触发 |
| `luxi_location` | 使用保存的 PLY 和当前深度点云做 ICP 精配准 | 约 2 Hz |
| 定位管理器 | 状态机、可信度准入、`map -> odom` 唯一发布 | 20～50 Hz |
| 网页 | 显示阶段、候选、位姿、错误原因并控制开始/停止 | 约 2～5 Hz |

HLoc 不替代 RTAB-Map 建图，也不替代实时里程计。它只替代当前
`/rtabmap/localization_pose` 所承担的全局粗定位职责。

## 3. 当前系统基线

当前统一传感器接口为：

```text
/sensors/rgbd/color/image_raw
/sensors/rgbd/depth/image_raw
/sensors/rgbd/color/camera_info
/sensors/imu/data
```

当前定位相关输出为：

```text
/rtabmap/odom
/rtabmap/localization_pose
/luxi_location/pose
/luxi_location/fitness
/luxi_location/status
```

当前地图由三类文件组成：

```text
maps/rtab_maps/mapNNN.db
maps/octo_maps/mapNNN_octomap/*_cloud.ply
maps/octo_maps/mapNNN_octomap/mapNNN.bt
```

现有流程使用 RTAB-Map 视觉词袋寻找地点候选。`map012.db` 只有 57 个关键帧，实际测试中
候选分数长期低于当前 `Rtabmap/LoopThr=0.2`，导致
`/rtabmap/localization_pose` 协方差保持 `9999`，ICP 因此拒绝粗位姿。

HLoc 的意义不是简单增加关键点数量，而是增加一个专门的全局图像检索阶段，再使用局部特征
和几何求解确认候选。

## 4. 总体数据流

### 4.1 建图与离线处理

```text
D435i RGB-D + IMU
        │
        v
luxi_adapter 统一话题
        │
        v
RTAB RGB-D odometry + RTAB-Map mapping
        │
        ├─> mapNNN.db
        ├─> 彩色 PLY
        └─> OctoMap .bt
                 │
                 v
       rtab_hloc_exporter
                 │
                 ├─> 参考 RGB 图像
                 ├─> 对应深度图
                 ├─> 相机内参
                 ├─> T_map_camera
                 └─> node_id / timestamp
                            │
                            v
                   hloc_map_builder
                            │
                            ├─> 全局描述子索引
                            ├─> SuperPoint 特征
                            └─> RGB-D 米制 3D 特征点
```

### 4.2 在线定位

```text
当前 RGB ──> 全局描述子 ──> Top-K 地图关键帧
                                  │
                                  v
                     SuperPoint + LightGlue
                                  │
                                  v
                    当前 2D 点 <-> 地图 3D 点
                                  │
                                  v
                          PnP + RANSAC
                                  │
                                  v
                        T_map_camera 候选
                                  │
当前深度 ─────────────────────────┤
                                  v
                         深度几何验证
                                  │
                                  v
                         T_map_base 粗位姿
                                  │
                                  v
                         Open3D ICP 精配准
                                  │
                                  v
                     定位管理器发布 map -> odom
                                  │
                                  v
                            导航与网页
```

## 5. 为什么采用独立 HLoc 包

不建议第一阶段直接修改 RTAB-Map 内部特征提取器，原因如下：

1. 当前 ROS 二进制 RTAB-Map 没有编译 Torch、Python 或 SuperPoint 支持；
2. HLoc 特征推理使用 Python、PyTorch 和 HDF5，独立部署更容易；
3. HLoc 可以单独离线回放和评估，不影响现有建图；
4. HLoc 出现异常时可以立即回退到原 RTAB 粗定位；
5. 全局检索、局部匹配和几何验证的中间结果可以分别显示和记录；
6. 避免为引入一个粗定位模型而重新维护整套 RTAB-Map 分支。

建议新增：

```text
project/luxi_hloc/
├── config/
│   └── hloc_localization.yaml
├── launch/
│   ├── hloc_localization.launch.py
│   └── hloc_icp_localization.launch.py
├── luxi_hloc/
│   ├── inference.py
│   ├── map_io.py
│   ├── node.py
│   ├── pose_estimator.py
│   └── geometry.py
├── scripts/
│   ├── build_reference_model.py
│   ├── check_hloc_environment.py
│   └── offline_localization_test.py
├── src/
│   └── rtab_hloc_exporter.cpp
├── test/
├── package.xml
├── CMakeLists.txt
└── README.md
```

数据库导出器使用 C++：

```text
project/luxi_hloc/src/rtab_hloc_exporter.cpp
```

第三方依赖建议放在：

```text
3parts/hloc/
3parts/lightglue/
3parts/hloc_models/
```

HLoc 本身适合 Python 实现，TF、状态机和安全准入适合 C++ 实现。第一阶段不要求为了统一语言
而重写 HLoc 推理代码。

## 6. HLoc 参考地图产物

每张 RTAB 地图必须生成一份同编号的 HLoc 地图：

```text
maps/hloc_maps/map012/
├── metadata.yaml
├── reference_images/
│   ├── node_000001.jpg
│   └── ...
├── reference_depth/
│   ├── node_000001.png
│   └── ...
├── global-feats-netvlad.h5
├── feats-superpoint-n1024.h5
├── landmarks-superpoint-n1024.h5
├── frames.csv
└── build_report.json
```

`metadata.yaml` 至少保存：

```yaml
schema_version: 1
map_id: map012
source_database: /home/lunar/project/lunar_slam/maps/rtab_maps/map012.db
source_database_sha256: ""
map_frame: map
camera_frame: camera_color_optical_frame
base_frame: base_link
image_count: 57
reference_model: netvlad
local_feature_model: superpoint
matcher_model: lightglue
created_at_utc: ""
```

启动定位前必须校验：

- `map_id` 与网页选择一致；
- `.db` 的摘要与构建 HLoc 地图时一致；
- HLoc 模型文件存在；
- NetVLAD、SuperPoint 和 RGB-D landmark HDF5 非空；
- 图像、位姿、内参和 node ID 数量一致；
- 地图坐标系为 `map`；
- 相机模型与当前硬件 profile 输出一致。

如果 `.db` 已改变但 HLoc 地图未重新生成，应拒绝启动，而不是继续使用过期索引。

## 7. 从 RTAB 数据库导出参考帧

### 7.1 导出器职责

`rtab_hloc_exporter` 应通过 RTAB-Map C++ API 以只读方式打开 `.db`，不要直接解析 SQLite
中的压缩图像字段。

建议使用：

```cpp
rtabmap::DBDriver::create()
driver->openConnection(path, false, true)  // readOnly=true
driver->getAllNodeIds(...)
driver->getNodeData(...)
driver->getNodeInfo(...)
data.uncompressData()
```

优化后的地图位姿优先来自：

```cpp
driver->loadOptimizedPoses(...)
```

如果数据库没有保存完整优化位姿，应读取节点和边并使用 RTAB-Map 优化器重建图优化结果。不能把
未经回环优化的原始里程计位姿误当成最终 `map` 位姿。

### 7.2 每个参考帧需要导出

| 字段 | 用途 |
|---|---|
| `node_id` | HLoc 结果与 RTAB 节点关联 |
| RGB 图像 | 全局检索和局部特征 |
| 深度图 | 建立米制 3D 点、在线几何验证 |
| `fx/fy/cx/cy` | 反投影和 PnP |
| 图像宽高 | 相机模型校验 |
| 深度单位 | 米/毫米转换 |
| `T_map_camera` | 建立地图坐标系 3D 特征 |
| 时间戳 | 调试与回放 |
| 原始 `map_id` | 过滤多 session 数据 |

### 7.3 位姿定义

本文统一定义：

```text
T_A_B：把 B 坐标系中的点转换到 A 坐标系。
```

因此：

```text
p_map = T_map_camera * p_camera
```

RTAB 数据库中的节点位姿通常对应机器人基准坐标系。若得到的是 `T_map_base`，必须使用建图时的
静态 TF 转换：

```text
T_map_camera = T_map_base * T_base_camera
```

导出器必须把矩阵含义写入 `metadata.yaml`，不能只保存一个没有方向定义的 4×4 数组。

### 7.4 关键帧筛选

HLoc 不要求保存每一帧传感器数据，但要求地图覆盖足够的观察方向。

建议第一版保留 RTAB 所有有效关键帧，并仅过滤：

- 图像或标定为空的节点；
- 被 RTAB 标记为坏签名的节点；
- 深度有效像素比例过低的节点；
- 严重模糊或曝光异常的节点；
- 几乎完全重复且位姿差异极小的节点。

后续地图较大时可以使用以下起步筛选阈值：

```text
相邻参考帧平移 >= 0.20～0.30 m
或相邻参考帧 yaw 差 >= 10～15°
```

但原地旋转形成的多方向关键帧不能被平移阈值误删。

### 7.5 建图采集要求

HLoc 不能推断地图中从未出现过的视角。建图时应：

1. 主要路线正向和反向各采集一次；
2. 关键路口原地缓慢旋转一圈；
3. 同一地点保存多个 yaw 方向；
4. 避免快速运动导致模糊；
5. 使用与实际运行接近的相机高度和俯仰角；
6. 对昼夜或灯光变化明显的场地采集多种光照；
7. 确保动态人员没有长期遮挡核心背景。

当前只有 57 个关键帧的 `map012` 可用于打通流程，但不应直接作为最终可靠性结论。

## 8. 构建 HLoc 参考模型

### 8.1 推荐模型组合

第一版已实现：

```text
全局检索：NetVLAD
局部特征：SuperPoint，最多 1024 个关键点
局部匹配：LightGlue
位姿求解：OpenCV solvePnPRansac
```

理由：

- NetVLAD 是 HLoc 中成熟的基线；
- SuperPoint 与 LightGlue 有直接兼容配置；
- 1024 个关键点适合作为 Jetson 第一阶段性能基线；
- OpenCV 提供成熟且无需额外 aarch64 wheel 的 PnP/RANSAC；
- 后续可以单独替换检索模型，不影响 ROS 接口。

MegaLoc、CosPlace 或其他全局描述子应在基线跑通后通过相同测试集比较，不应同时引入多个模型。

### 8.2 全局描述子索引

对每张参考 RGB 图像计算一个全局描述子并写入 HDF5：

```text
node_000001.jpg -> D 维归一化向量
node_000002.jpg -> D 维归一化向量
...
```

在线查询使用余弦相似度或模型对应距离，从全部参考帧中取 Top-K。

全局检索只回答“可能是哪几张参考图”，不直接输出机器人位姿。

### 8.3 SuperPoint 特征

对每张参考图像离线提取：

```text
keypoints: N x 2
descriptors: D x N
scores: N
image_size: 2
```

关键点坐标必须对应存入 HDF5 的实际图像分辨率。若在线推理缩放图像，必须按同一约定把坐标还原。

### 8.4 地图帧配对

建立参考模型时不需要对所有图像做全连接匹配。优先使用：

- RTAB 位姿邻近关系；
- RTAB 图中的共视/相邻边；
- 全局检索得到的相似图；
- 同一位置不同方向的补充配对。

每张参考图选择约 10～20 个合理邻居作为第一版起点，避免 `N²` 匹配。

### 8.5 使用已知 RTAB 位姿构建米制模型

纯单目 SfM 会产生尺度不确定性。当前系统已有 RGB-D 和 RTAB 优化位姿，因此推荐：

当前实现直接利用每个参考帧的已知 `T_map_camera` 和深度，把参考 SuperPoint
关键点反投影为 `map` 坐标系米制 3D 点。在线匹配建立
`query 2D keypoint <-> map 3D landmark` 后，OpenCV PnP 返回 world-to-camera 位姿：

```text
T_camera_map
```

ROS 需要的是：

```text
T_map_camera = inverse(T_camera_map)
```

此处是最容易产生镜像、旋转 180° 或位置跳变的实现错误，必须写单元测试验证。

### 8.6 深度辅助建立 3D 点

对于关键点像素 `(u, v)` 和米制深度 `z`：

```text
x = (u - cx) * z / fx
y = (v - cy) * z / fy
p_camera = [x, y, z, 1]^T
p_map = T_map_camera * p_camera
```

深度必须满足：

```text
minimum_depth <= z <= maximum_depth
```

并应排除：

- `0`、NaN 和无穷值；
- 深度边缘跳变；
- 反光区域；
- 图像与深度未对齐的像素；
- 相机有效范围外的点。

来自多个参考帧的同一 3D 轨迹，可以使用中值或鲁棒平均融合深度。

## 9. 在线 HLoc 粗定位

### 9.1 输入同步

HLoc 节点订阅：

```text
/sensors/rgbd/color/image_raw
/sensors/rgbd/depth/image_raw
/sensors/rgbd/color/camera_info
```

粗检索只需要 RGB，但深度验证要求 RGB、深度和内参属于同一时间附近。建议使用近似同步，最大时间差
第一版不超过 50 ms。

节点只保留最新一组完整数据，处理落后时丢弃旧帧，禁止形成无界队列。

### 9.2 查询调度

建议：

```text
未定位：1 Hz
已有候选但未通过：1～2 Hz
已定位：0.2～0.5 Hz 健康检查
定位质量下降：恢复 1 Hz
```

HLoc 不需要 30 Hz 运行。高频运动由 RTAB RGB-D odometry 负责。

### 9.3 Top-K 检索

在线流程：

1. 对当前 RGB 图像计算全局描述子；
2. 与参考描述子矩阵计算相似度；
3. 选择 Top-5 作为第一版默认值；
4. 合并空间相邻但图像方向不同的参考帧；
5. 依次交给 LightGlue；
6. 任何候选通过严格验证后停止处理剩余候选。

调试信息应发布：

```text
候选 node ID
全局相似度
排序
候选参考图路径
```

### 9.4 SuperPoint + LightGlue

对查询图只提取一次 SuperPoint 特征，然后与每个候选的离线特征匹配。

第一版建议保存：

```text
原始关键点数
LightGlue 匹配数
置信匹配数
匹配置信度均值
匹配点图像覆盖率
```

不能只以匹配数量判定成功。大量匹配集中在一个门框、屏幕或海报上仍可能得到错误位姿。

### 9.5 PnP/RANSAC

通过参考模型将候选图像上的匹配关键点关联到地图 3D 点：

```text
query 2D keypoint <-> map 3D landmark
```

调用 OpenCV `solvePnPRansac` 绝对位姿估计，得到：

```text
T_camera_map
RANSAC inliers
reprojection errors
```

再转换为：

```text
T_map_camera
```

第一阶段建议的粗定位准入起点：

| 指标 | 起步值 |
|---|---:|
| 2D-3D 对应数 | `>= 40` |
| PnP/RANSAC 内点数 | `>= 30` |
| 内点比例 | `>= 0.20` |
| 中值重投影误差 | `<= 3 px` |
| 图像内点覆盖率 | `>= 0.15` |
| 连续一致候选次数 | `>= 2` |

这些是初始工程值，必须用本项目数据集调整，不能作为模型固有参数。

### 9.6 深度几何验证

PnP 成功后使用当前深度做第二次验证：

1. 将查询匹配点反投影为当前相机 3D 点；
2. 使用 `T_map_camera` 转换到地图坐标；
3. 与参考 3D 点或候选局部子地图比较；
4. 统计 3D 残差、有效深度比例和空间覆盖；
5. 拒绝深度尺度、朝向或位置不一致的候选。

起步准入建议：

```text
有效深度匹配点 >= 20
3D 残差中值 <= 0.20 m
3D 残差 P90 <= 0.50 m
```

这一步可以显著降低重复走廊、相似门窗、显示屏和海报造成的视觉误定位。

### 9.7 转换到机器人位姿

HLoc 得到：

```text
T_map_camera
```

使用适配层静态 TF：

```text
T_camera_base
```

计算：

```text
T_map_base = T_map_camera * T_camera_base
```

输出消息：

```text
geometry_msgs/msg/PoseWithCovarianceStamped
topic: /luxi_hloc/coarse_pose
frame_id: map
```

协方差不能固定写零。应依据：

- PnP 内点数；
- 重投影误差；
- 深度 3D 残差；
- 连续候选稳定度；
- 候选第一名与第二名的相似度差；

生成保守协方差。无法可靠估计时应发布状态但不发布位姿。

## 10. ICP 精配准

现有 `luxi_location` 已经支持通过配置切换初始位姿话题。HLoc 接入后：

```text
initial_pose_topic: /luxi_hloc/coarse_pose
```

ICP 保持使用：

```text
地图：maps/octo_maps/mapNNN_octomap/*_cloud.ply
当前扫描：/sensors/rgbd/depth/image_raw
内参：/sensors/rgbd/color/camera_info
```

链路变为：

```text
/luxi_hloc/coarse_pose
        │
        v
luxi_location Open3D ICP
        │
        ├─> /luxi_location/pose
        ├─> /luxi_location/fitness
        └─> /luxi_location/status
```

HLoc 粗位姿未通过时，ICP 必须继续等待，不允许使用相似度最高但不可信的候选。

第一阶段继续使用现有 ICP 阈值；待 HLoc 数据集测试完成后再根据粗位姿误差分布调整
`initial_maximum_translation_correction` 和 `initial_maximum_yaw_correction_deg`。

## 11. 连续里程计与 TF

### 11.1 定位时保留 RGB-D odometry

HLoc 只在地图中确定绝对位置，不能提供平滑的 30 Hz 轨迹。定位阶段仍需启动
RTAB `rgbd_odometry`，输出：

```text
odom -> base_link
```

定位管理器使用 ICP 精位姿计算：

```text
T_map_odom = T_map_base * inverse(T_odom_base)
```

并发布：

```text
map -> odom
```

完整 TF：

```text
map
 └── odom
      └── base_link
           └── camera_link
                └── camera_color_optical_frame
```

### 11.2 唯一 TF 发布者

同一时刻只能有一个节点发布 `map -> odom`。

HLoc 模式下：

- RTAB-Map localization 不发布 `map -> odom`；
- `luxi_location` 的 `publish_tf` 关闭；
- `localization_manager_node` 是唯一 `map -> odom` 发布者。

否则会出现机器人在网页/RViz 中跳变、TF 时间回退或规划坐标不一致。

### 11.3 位姿更新策略

首次定位：

```text
HLoc 粗位姿 -> ICP -> 立即初始化 map -> odom
```

后续健康检查：

- 小修正使用低通或有限时间平滑；
- 大修正先进入待确认状态；
- 连续两次 HLoc + ICP 一致后才重置；
- 任何单帧异常不得直接造成导航位姿跳变。

机器人正在执行路径时，应使用比静止状态更严格的大修正准入条件。

## 12. ROS 接口设计

### 12.1 HLoc 节点订阅

| 话题 | 类型 | 说明 |
|---|---|---|
| `/sensors/rgbd/color/image_raw` | `sensor_msgs/Image` | 查询 RGB |
| `/sensors/rgbd/depth/image_raw` | `sensor_msgs/Image` | 深度验证 |
| `/sensors/rgbd/color/camera_info` | `sensor_msgs/CameraInfo` | 当前内参 |
| `/rtabmap/odom` | `nav_msgs/Odometry` | 连续运动和时间一致性 |

### 12.2 HLoc 节点发布

| 话题 | 类型 | 说明 |
|---|---|---|
| `/luxi_hloc/coarse_pose` | `PoseWithCovarianceStamped` | 通过完整验证的粗位姿 |
| `/luxi_hloc/status` | `std_msgs/String` | 当前阶段和失败原因 |
| `/luxi_hloc/retrieval_score` | `std_msgs/Float32` | 第一候选全局分数 |
| `/luxi_hloc/inlier_count` | `std_msgs/Int32` | PnP 内点数 |
| `/luxi_hloc/reprojection_error` | `std_msgs/Float32` | 中值重投影误差 |
| `/luxi_hloc/depth_residual` | `std_msgs/Float32` | 深度几何残差 |
| `/luxi_hloc/debug_matches` | `sensor_msgs/Image` | 可选匹配调试图 |

调试图默认关闭，避免持续占用 CPU、内存和网络。

### 12.3 建议服务

```text
/luxi_hloc/start
/luxi_hloc/stop
/luxi_hloc/relocalize
/luxi_hloc/load_map
```

`load_map` 必须是原子操作：模型、索引、重建和元数据全部加载成功后才切换当前地图。

## 13. 状态机

建议状态：

```text
STOPPED
  │ start
  v
LOADING_MAP
  │ success
  v
SEARCHING
  │ retrieval
  v
MATCHING
  │ local matches
  v
ESTIMATING
  │ PnP
  v
VERIFYING_DEPTH
  │ accepted
  v
WAITING_ICP
  │ ICP accepted
  v
LOCALIZED
  │ quality lost
  v
DEGRADED
  │ timeout
  v
SEARCHING
```

任何阶段失败都应返回 `SEARCHING` 并保存明确原因，不应停留在没有解释的“定位中”。

建议失败原因枚举：

```text
NO_SENSOR_DATA
MAP_NOT_LOADED
RETRIEVAL_SCORE_LOW
NO_CANDIDATE_MATCH
PNP_CORRESPONDENCES_LOW
PNP_INLIERS_LOW
REPROJECTION_ERROR_HIGH
DEPTH_VALID_POINTS_LOW
DEPTH_RESIDUAL_HIGH
POSE_INCONSISTENT
ICP_REJECTED
ODOMETRY_LOST
MODEL_RUNTIME_ERROR
```

## 14. 配置文件草案

`project/luxi_hloc/config/hloc_localization.yaml` 可采用：

```yaml
luxi_hloc:
  ros__parameters:
    map_id: map012
    map_directory: /home/lunar/project/lunar_slam/maps/hloc_maps/map012

    rgb_topic: /sensors/rgbd/color/image_raw
    depth_topic: /sensors/rgbd/depth/image_raw
    camera_info_topic: /sensors/rgbd/color/camera_info
    odometry_topic: /rtabmap/odom

    map_frame: map
    base_frame: base_link
    camera_frame: camera_color_optical_frame

    retrieval_model: netvlad
    local_feature_model: superpoint
    matcher_model: lightglue

    unlocalized_query_rate: 1.0
    localized_health_check_rate: 0.2
    retrieval_top_k: 5
    max_keypoints: 1024
    image_max_edge: 800

    minimum_pnp_correspondences: 40
    minimum_pnp_inliers: 30
    minimum_pnp_inlier_ratio: 0.20
    maximum_reprojection_error_px: 3.0
    minimum_inlier_distribution: 0.15

    minimum_depth_matches: 20
    maximum_depth_median_residual: 0.20
    maximum_depth_p90_residual: 0.50

    required_consistent_poses: 2
    consistency_translation: 0.50
    consistency_yaw_deg: 15.0

    use_cuda: true
    use_mixed_precision: true
    publish_debug_image: false
```

所有阈值必须在数据集评估报告中记录来源和最终值。

## 15. 模型部署与微调策略

### 15.1 第一阶段直接使用预训练模型

第一阶段不微调：

```text
NetVLAD/CosPlace/MegaLoc：预训练权重
SuperPoint：预训练权重
LightGlue：预训练权重
```

每张地图计算自己的参考描述子和 3D 视觉模型，这叫“构建场景索引”，不是模型训练。

### 15.2 当前 Jetson 环境限制

当前系统检查结果：

```text
Jetson Orin
CUDA 12.6
RTAB-Map 0.23.7
RTAB With SuperPoint Torch: false
RTAB With Python3: false
系统 Python 不安装 torch 和 lightglue；工程隔离目录已安装 CPU/GPU 运行时
```

因此 HLoc 应使用独立 Python 环境，不应直接向 ROS 系统 Python 中无约束安装依赖。

安装 PyTorch 时必须选择与 JetPack/CUDA/aarch64 匹配的 NVIDIA 版本。不能直接假设通用 PyPI
CUDA wheel 可在 Jetson 上工作。

### 15.3 何时考虑微调

先建立独立测试集并定位失败层：

| 失败现象 | 首先处理 | 何时微调 |
|---|---|---|
| 正确关键帧不在 Top-K | 增加地图视角、检查光照与检索模型 | 地图覆盖充分但 Recall@5 仍低时微调全局描述子 |
| Top-K 正确但匹配少 | 调整分辨率、关键点数和地图角度 | 大量正确候选仍无法匹配时再考虑局部模型 |
| PnP 失败 | 检查 2D-3D 轨迹和坐标约定 | 通常不通过微调解决 |
| 深度验证失败 | 检查对齐、深度单位、动态点 | 不通过微调解决 |
| ICP 失败 | 检查 PLY、TF 和粗位姿误差 | 不通过微调解决 |

如果需要微调全局检索模型，训练数据应包括：

- 同地点不同 yaw、光照和时间的正样本；
- 相似走廊、门、墙面作为困难负样本；
- 与部署相同的 D435i 图像；
- 独立划分训练、验证和测试路线。

训练建议在 x86 GPU 主机完成，只把推理权重部署到 Jetson。

### 15.4 TensorRT 优化时机

正确顺序：

1. PyTorch FP32 验证精度；
2. PyTorch FP16 验证精度和速度；
3. 固定输入尺寸与动态范围；
4. 导出 ONNX；
5. 构建 TensorRT engine；
6. 对比 FP32/FP16 输出、召回率和位姿成功率；
7. 通过后再替换线上推理。

不应在算法尚未通过准确性测试时先进行 TensorRT 重写。

## 16. 与现有 launch 和网页的集成

### 16.1 建图流程

建图保持：

```text
luxi_adapter sensor_bringup
  -> luxi_rtab_map rgbd_mapping
```

停止并保存 RTAB 数据库后：

```text
export_rtabmap_octomap.sh
rtab_hloc_exporter
build_reference_model.py
```

只有 PLY、`.bt` 和 HLoc 地图都构建成功，网页才把地图标记为“可自动定位”。

### 16.2 定位流程

`saved_map_navigation.launch.py` 最终应启动：

```text
RTAB rgbd_odometry
luxi_hloc/hloc_localizer_node
luxi_location/icp_localization_node
luxi_hloc/localization_manager_node
octomap_file_loader
octomap_astar_planner
path_follower
```

不再启动 RTAB-Map `.db` 视觉词袋 localization 作为主要粗定位器。

第一阶段可以保留一个配置开关：

```yaml
coarse_localization_backend: hloc
```

允许：

```text
hloc
rtabmap
manual
```

但同一时刻只能启用一个粗定位后端向 ICP 发送初始位姿。

### 16.3 网页状态

网页应显示：

```text
加载 HLoc 地图
全局检索中
候选匹配中
PnP 求解中
深度验证中
ICP 精配准中
已定位
定位退化
```

建议显示：

- Top-1 地图 node ID；
- retrieval score；
- LightGlue 有效匹配数；
- PnP 内点数；
- 重投影误差；
- 深度残差；
- ICP fitness/RMSE；
- 最终 `x/y/yaw`；
- 最近一次成功时间。

只有最终 `LOCALIZED` 状态才启用网页“选择目标点”。

## 17. 测试计划

### 17.1 单元测试

必须覆盖：

- `T_camera_map` 与 `T_map_camera` 互逆；
- `T_map_camera * T_camera_base` 计算；
- `T_map_base * inverse(T_odom_base)` 计算；
- 深度毫米到米转换；
- 像素反投影；
- 无效深度过滤；
- Top-K 排序；
- PnP 准入条件；
- 协方差生成；
- 地图摘要不一致时拒绝加载；
- 状态机所有失败回退。

### 17.2 离线已知变换测试

从参考图像生成一个已知扰动：

```text
平移：0.5 m、1.0 m
yaw：15°、30°、60°
亮度：降低/提高
局部遮挡：10%、30%
```

验证 retrieval、LightGlue、PnP 和深度验证是否恢复正确候选。

### 17.3 留出路线测试

建图路线不能同时作为唯一测试数据。至少录制：

- 相同路线、相同方向；
- 相同路线、反向；
- 原地不同 yaw；
- 不同光照；
- 部分动态遮挡；
- 地图外区域；
- 重复走廊或相似门窗。

地图外区域必须保持未定位，不能为了提高“成功率”输出错误位置。

### 17.4 实机测试

每个测试点执行：

1. 停止并重新启动所有定位节点；
2. 不给初始位姿；
3. 静止等待 5 秒；
4. 缓慢旋转或移动；
5. 记录首次粗定位时间；
6. 记录 ICP 接受时间；
7. 验证网页机器人箭头；
8. 运动一段距离验证连续跟踪；
9. 遮挡相机后恢复；
10. 点击停止，确认无残留进程。

### 17.5 性能测试

使用 `tegrastats` 记录：

- CPU 总占用和各进程占用；
- GPU 利用率；
- 内存；
- 温度；
- 功耗；
- 是否热降频；
- HLoc 单次检索、SuperPoint、LightGlue、PnP 总耗时；
- RGB/Depth/odometry 是否维持原频率。

## 18. 验收指标

第一阶段建议：

| 指标 | 目标 |
|---|---:|
| Retrieval Recall@5 | `>= 95%` |
| 已知地图内粗定位成功率 | `>= 95%` |
| 地图外错误定位 | `0` |
| 粗定位平移误差 | `<= 1.0 m` |
| 粗定位 yaw 误差 | `<= 30°` |
| ICP 后平移误差 | `<= 0.20 m` |
| ICP 后 yaw 误差 | `<= 5°` |
| 未定位状态查询 P95 | `<= 1.0 s` |
| RGB-D odometry | 不因 HLoc 降至不可用频率 |
| 停止后残留 HLoc/ICP 进程 | `0` |

定位系统优先保证零错误定位。真实位置不确定时应继续搜索，不能以降低阈值换取表面成功。

## 19. 故障诊断

| 状态 | 可能原因 | 检查项 |
|---|---|---|
| `NO_SENSOR_DATA` | 硬件 profile 未启动 | RGB、Depth、CameraInfo 频率 |
| `MAP_NOT_LOADED` | HLoc 地图不完整 | `metadata.yaml`、HDF5、`frames.csv` |
| `RETRIEVAL_SCORE_LOW` | 地图缺少视角、光照变化 | Top-K 图像、地图覆盖 |
| `NO_CANDIDATE_MATCH` | SuperPoint/LightGlue 不匹配 | 分辨率、关键点数、模型组合 |
| `PNP_INLIERS_LOW` | 错误候选或 3D 轨迹不足 | 2D-3D 对应、内点分布 |
| `REPROJECTION_ERROR_HIGH` | 坐标/内参错误 | 位姿方向、图像缩放、相机模型 |
| `DEPTH_VALID_POINTS_LOW` | 深度无效或未对齐 | 深度单位、时间同步、有效范围 |
| `DEPTH_RESIDUAL_HIGH` | 候选错误、动态物体 | 匹配图、3D 残差分布 |
| `ICP_REJECTED` | 粗位姿超范围、PLY 不一致 | fitness、RMSE、修正量 |
| `ODOMETRY_LOST` | 模糊、低纹理、计算过载 | `/rtabmap/odom_info`、帧率 |
| 位姿跳变 | 多 TF 发布者 | `ros2 run tf2_tools view_frames` |

## 20. 分阶段实施

### 阶段 0：数据可用性验证

- 确认 `.db` 中 RGB、深度、标定和优化位姿完整；
- 导出 `map012` 的 57 个关键帧；
- 生成人工报告检查图像方向、深度和彩色点云一致。

验收：每个有效节点都能恢复 RGB、深度、内参和 `T_map_camera`。

### 阶段 1：离线 HLoc 基线

- 安装 Jetson 兼容 PyTorch/HLoc；
- 构建 NetVLAD + SuperPoint + LightGlue 参考模型；
- 使用留出图像计算 Recall@1/5/10；
- 运行 PnP 并输出离线位姿误差。

验收：Recall@5 和位姿误差达到第一阶段标准。

### 阶段 2：ROS HLoc 粗定位节点

- 订阅统一 RGB-D；
- 实现 Top-K、LightGlue、PnP、深度验证；
- 发布 `/luxi_hloc/coarse_pose` 和诊断；
- 不接入 ICP 和导航。

验收：实机未知初始位置可稳定发布可信粗位姿，地图外不发布。

### 阶段 3：ICP 与 TF 集成

- ICP 输入切到 `/luxi_hloc/coarse_pose`；
- 实现定位管理器；
- 保证唯一 `map -> odom`；
- 完成定位丢失与恢复。

验收：HLoc 粗位姿经过 ICP 后形成连续导航 TF。

### 阶段 4：网页集成

- 网页显示 HLoc 各阶段；
- 地图转换同时生成 HLoc 参考地图；
- 自动定位启动 HLoc + ICP；
- 只有最终可信状态开放目标点。

验收：浏览器可以完成地图选择、自动定位、显示机器人位置和停止清理。

### 阶段 5：性能优化

- FP16；
- 模型输入尺寸和 Top-K 优化；
- 定位成功后降频；
- 必要时 ONNX/TensorRT；
- 只有数据证明需要时才微调模型。

## 21. 回退策略

新实现必须保留配置回退：

```yaml
coarse_localization_backend: rtabmap
```

回退时：

- 使用原 `rgbd_localization.launch.py`；
- ICP 输入恢复 `/rtabmap/localization_pose`；
- 停止 HLoc 节点；
- TF 发布权恢复为 RTAB-Map；
- 网页仍使用统一的定位状态接口。

地图文件和原 RTAB 建图链路不应因 HLoc 接入而发生不可逆修改。

## 22. 参考资料

- HLoc 官方仓库与总体流程：
  <https://github.com/cvg/Hierarchical-Localization>
- LightGlue 官方仓库：
  <https://github.com/cvg/LightGlue>
- RTAB-Map 参数与全局描述子扩展接口：
  <https://github.com/introlab/rtabmap/blob/master/corelib/include/rtabmap/core/Parameters.h>
- Open3D 全局与局部点云配准：
  <https://www.open3d.org/docs/release/tutorial/pipelines/global_registration.html>
- 当前 ICP 工程：
  `project/luxi_location`
- 当前 RTAB 建图工程：
  `project/luxi_RTAB_Map`
- 当前硬件适配层：
  `project/luxi_adapter`
- 当前网页控制：
  `project/luxi-web-control`
