# 定位方式与 EVK4 低开销接入说明

本文说明 `lunar_slam` 当前已经具备的定位能力、可选择的后续定位方式，以及在不显著增加
Jetson CPU 开销的前提下，如何使用 Prophesee EVK4（IMX636）改善高速运动时的局部定位稳定性。

本文中的“定位”分为两类：

- **局部里程计**：连续估计短时间的相对运动，输出 `odom -> base_link`。它响应快，但会累计漂移。
- **全局重定位**：将机器人匹配回已保存地图，输出 `map -> odom` 或 `map -> base_link`。它能消除长期漂移，
  但依赖重新观察到地图中的有效场景。

## 当前系统的定位链路

当前系统以 D435i 的 RGB-D、相机内参和 IMU 为输入，通过 RTAB-Map 完成视觉里程计和已建地图重定位：

```text
D435i RGB + Depth + CameraInfo ─┐
D435i IMU ──────────────────────┼─> luxi_adapter
                                │   /sensors/rgbd/*
                                │   /sensors/imu/data
                                v
                    RTAB-Map RGB-D visual odometry
                                │
                                ├─> /rtabmap/odom（局部运动）
                                │
已保存 maps/rtab_maps/mapNNN.db ─┼─> RTAB-Map localization
                                │       map -> odom -> base_link
                                v
                      /rtabmap/localization_pose
                                │
已保存 maps/octo_maps/*.bt ─────┼─> OctoMap + A* 全局路径规划
                                v
                          /navigation/planned_path
```

对应工程文件：

- `project/luxi_adapter`：D435i 硬件话题统一为 `/sensors/rgbd/*` 和 `/sensors/imu/data`。
- `project/luxi_RTAB_Map/launch/rgbd_mapping.launch.py`：RGB-D 建图和视觉里程计。
- `project/luxi_RTAB_Map/launch/rgbd_localization.launch.py`：以 `localization:=true` 加载保存的
  `mapNNN.db`，不再向数据库写入新的关键帧。
- `project/luxi_voxel_navigation/launch/saved_map_navigation.launch.py`：将 RTAB-Map 重定位与已保存的
  `.bt` OctoMap 路径规划组合启动。
- `project/luxi-web-control/luxi_web_control/web_control_node.py`：订阅
  `/rtabmap/localization_pose`，检查协方差；定位可信前不允许网页发送导航目标。

### 当前已经实现与尚未实现的内容

| 能力 | 当前状态 | 说明 |
|---|---|---|
| D435i RGB-D 视觉里程计 | 已实现 | RTAB-Map 的局部 `odom` 来源。 |
| 使用已保存 `.db` 的重定位 | 已实现 | 网页加载地图后启动 `rgbd_localization.launch.py`。 |
| 已保存 OctoMap 上的 A* 路径规划 | 已实现 | `.bt` 用于障碍物/规划和网页显示，不直接计算位姿。 |
| 轮速计融合 | 未实现 | 推荐作为低成本、低算力的下一增强项。 |
| 点云 ICP/GICP/NDT 重定位 | 未实现 | 不应与 RGB-D 重定位混为一谈。 |
| EVK4 驱动、事件预处理、Event-VIO | 未实现 | 需要以新的 `luxi_adapter` 硬件 profile 接入。 |

## 可选择的定位方式

| 方式 | 传感器 | 适合场景 | 优点 | 主要限制 | 与当前工程的关系 |
|---|---|---|---|---|---|
| RTAB-Map RGB-D 重定位 | D435i RGB、深度、IMU | 正常室内、可重访已有场景 | 已经实现；有尺度和深度；可直接使用 `.db` | 快速运动会模糊；暗光、低纹理下易失效 | 当前全局定位主方案。 |
| 视觉/惯性里程计（VIO） | 相机 + IMU | 连续局部跟踪 | 延迟低、可提供平滑姿态 | 长时间会漂移 | 当前 RTAB-Map 视觉里程计已承担部分作用。 |
| 轮速计 + IMU EKF | 编码器 + IMU | 平坦地面、正常轮胎附着 | CPU 开销极低；补足视觉静止或低纹理段 | 打滑、悬空会漂移 | 推荐优先增加。 |
| 点云 ICP/GICP | 实时 3D 点云 + 全局点云 | 几何结构明显、已有较好初值 | 对颜色和光照不敏感 | 需要高质量点云和初值；CPU/GPU 较高 | 后续可用于激光雷达或深度点云。 |
| NDT/Scan Context + ICP | 3D 激光雷达 | 大空间、丢失后的全局检索 | 对激光环境重定位成熟 | 硬件和计算成本较高 | 适合后续 3D LiDAR 方案。 |
| Event-VIO | EVK4 + IMU | 高速、振动、HDR、暗光 | 没有帧曝光模糊；高时间分辨率 | 静止时事件少；单目无深度；需要同步/外参 | 建议作为 D435i 的局部里程计增强。 |
| AprilTag/UWB/GNSS | 标记/基站/卫星 | 有基础设施或室外 | 可提供绝对约束和初始化 | 部署条件限制 | 可作为重定位失败后的辅助。 |

推荐的总体策略不是在这些方式中只选一个，而是分层使用：

1. D435i + RTAB-Map 保存地图并做全局重定位；
2. EVK4 或轮速计提供高速、短时的局部里程计；
3. IMU 提供高频姿态预测；
4. 融合器输出稳定的局部位姿；
5. RTAB-Map 匹配成功时再纠正局部漂移。

## EVK4 的能力与接入边界

EVK4 HD 使用 Sony IMX636 事件传感器，分辨率为 1280×720，使用 USB 3.0 Type-C 输出，最大相机
带宽为 1.6 Gbps，典型相机功耗约 0.5 W，带 Trigger In、Sync In/Out 和 C/CS 镜头接口。EVK4 本体的
功耗较低，但高速运动时的事件吞吐和主机处理会成为主要负担。

来源：[Prophesee EVK4 HD 产品简介](https://www.prophesee.ai/wp-content/uploads/2026/03/EVK4-HD-Prophesee-Evaluation-Kit-Brief-2026.pdf)。

EVK4 输出的是异步亮度变化事件 `(x, y, timestamp, polarity)`，不是 RGB 图像，也不提供深度。因此：

- 不能将原始事件直接接到当前 RTAB-Map RGB-D 输入；
- 不能替代 D435i 的深度建图和 `.db` 全局重定位；
- 不应把原始事件转换为高帧率 RGB 视频后再运行第二套 RTAB-Map，这会产生不必要的 CPU/GPU 和内存压力；
- 最适合的职责是输出高频的相对位姿，再与 D435i/IMU/轮速计融合。

### 为什么 EVK4 有助于高速定位

普通帧相机需要曝光，快速转弯或振动时容易产生运动模糊；事件相机逐像素、异步报告亮度变化，适合快速
运动、强动态范围和低照度条件。事件/惯性里程计已有针对上述情况的位姿估计方法；其难点在于异步数据处理
和优化计算，而不是传感器采集本身。

参考：[NASA Event-Based Visual-Inertial Odometry](https://software.nasa.gov/software/NPO-52048-1)。

## 低 CPU 开销的推荐架构

推荐在 `project/luxi_adapter` 中新增 EVK4 profile 和一个 C++ 事件处理组件，但**不要**将每个事件转换成
ROS 消息后在多个节点间复制。事件应在同一个 C++ 进程内批处理，只有低频状态和诊断信息跨 ROS 发布。

```text
EVK4 USB 3.0
    │  Metavision C++ SDK
    v
luxi_adapter / evk4_event_frontend（单一 C++ 进程）
    ├─ 硬件事件滤波（优先）
    ├─ 热像素 / 事件尾迹 / 背景活动滤波
    ├─ 前方 ROI 裁剪
    ├─ 固定时间窗口聚合（建议 5~10 ms）
    ├─ 最大事件率保护与抽样
    └─ Event-VIO
             │
             ├─> /sensors/evs/event_rate       （诊断，1~5 Hz）
             ├─> /sensors/evs/tracking_status  （诊断，10 Hz）
             └─> /odometry/event                （位姿，建议 100 Hz）
                                      │
D435i IMU + wheel odometry ──────────┼─> 融合器（EKF/ESKF）
                                      v
                               /odometry/filtered
                                      │
                               RTAB-Map / navigation
```

### 关键限流和降载策略

1. **先做硬件侧滤波。** IMX636/EVK4 可使用事件信号处理（ESP）中的 Event Trail Filter；优先在相机侧
   去除短暂、重复的事件，避免无意义数据进入 USB 和 CPU。参考
   [Metavision EVK3/EVK4 FAQ](https://docs.prophesee.ai/stable/faq.html)。
2. **使用 ROI。** 只处理车前方、地平线附近和稳定特征较多的区域；不要在 VIO 中处理整幅 1280×720 图像。
3. **使用时间窗口，不使用“来一个处理一个”。** 将 5~10 ms 内的事件组成时间表面或事件帧；窗口保持固定，
   可使调度和内存上限可预测。
4. **设置事件率水位线。** 低于阈值正常处理；超过软阈值时提高抽样步长或缩小 ROI；超过硬阈值时丢弃最旧
   事件而非积压队列。驱动应发布 `events_per_sec` 和丢弃计数。
5. **只输出状态，不转发原始事件。** ROS 中发布 `nav_msgs/Odometry`、诊断和少量调试时间表面；仅在录包时
   记录原始数据，且写盘与 VIO 解耦。
6. **避免深度网络和稠密事件建图。** 第一阶段不做事件重建 RGB、神经网络深度估计或稠密 Event-SLAM；它们会
   与 D435i/RTAB-Map 争用 GPU，并显著增加功耗和延迟。
7. **按质量自适应启用。** 当 D435i RGB-D 里程计质量正常时，Event-VIO 可低频或仅维护状态；当检测到运动
   模糊、视觉匹配内点不足或 D435i 里程计协方差升高时，再提高 Event-VIO 的权重。

这样设计后，EVK4 的 CPU 开销由“事件率 × 有效 ROI × 处理复杂度”控制，而不是被峰值事件率拖垮。

## USB、同步、镜头与安装要求

### USB 与供电

- EVK4 应接 Jetson 的 SuperSpeed USB 3 独立端口；不要与 D435i 共用 USB 2 Hub，也应避免两者共用一个
  带宽有限的 USB 3 Hub。
- 上电后先验证 USB 链路为 SuperSpeed，再测试高运动事件率下 D435i RGB、深度和 IMU 是否仍保持既定频率。
- 0.5 W 是相机典型功耗，不包含 Jetson 对事件流进行 VIO 的 CPU/GPU 功耗和散热预算。

### 时间同步

时间同步是融合定位的必需条件，不是可选优化。EVK4 提供同步接口；帧相机可通过 VSync/Trigger 产生事件流
中的外部触发事件，供软件对齐时间轴。若 D435i 硬件同步线无法接入，应先使用统一时钟和软件时间偏移估计，
但高速融合的精度会弱于硬件同步。

参考：[Metavision 相机同步文档](https://docs.prophesee.ai/stable/hw/manuals/synchronization.html)。

### 外参和镜头

- EVK4 与 D435i 必须刚性固定在同一支架；高速振动时不能有相对位移。
- 虽然当前 D435i-only adapter 不要求额外外参，但 EVK4 与 D435i/IMU 融合时必须标定
  `base_link -> evk4_optical_frame` 外参，否则两者位姿不能正确融合。
- EVK4 默认参考镜头的对角 FOV 约为 47.7°；选择镜头时应让 EVK4 与 D435i 色彩相机有足够重叠视场，优先
  考虑前方特征区域，而不是只追求更宽视角。

## 分阶段实施计划

### 阶段 0：硬件与负载基线

目标：不改 SLAM，仅确认 EVK4 连接、镜头、USB 和事件率可控。

1. 安装与验证 Metavision SDK 的 C++ 采集链路；EVK4 的 SDK/硬件文档见
   [Metavision SDK](https://docs.prophesee.ai/stable/index.html)。
2. 同时运行 D435i profile，记录静止、低速、高速转动三种情况下的：`events_per_sec`、USB 状态、D435i
   RGB/Depth/IMU 频率、Jetson 温度、CPU/GPU/内存和功耗。
3. 使用 `tegrastats` 连续记录负载，确认不发生热降频和持续 ROS 队列增长。

验收：D435i 保持 RGB/Depth 约 30 Hz、IMU 约 200 Hz；EVK4 高事件率时没有 USB 反复重连。

### 阶段 1：低开销事件前端

目标：实现 `luxi_adapter` 的 `evk4` profile，但不引入 VIO。

1. 新增 `evk4_event_frontend` C++ 节点或组件；直接使用 SDK 回调批处理事件。
2. 实现硬件滤波、ROI、时间窗口、事件率水位线和丢弃策略。
3. 发布仅包含诊断的事件率、丢弃数、窗口延迟和处理耗时；不发布全量原始事件。

验收：高速场景下处理耗时有固定上限，超过阈值时主动降采样而不积压。

### 阶段 2：Event-VIO 与局部位姿输出

目标：以 EVK4 + IMU 输出高频 `/odometry/event`。

1. 选择可在目标 Jetson 上实时运行的轻量 Event-VIO；优先使用稀疏特征或时间表面跟踪，避免稠密建图。
2. 完成 EVK4-IMU 时间偏移与外参标定。
3. 将输出限制为 100 Hz 左右，若单周期预算为 10 ms，则 P95 处理时间目标小于 8 ms。
4. 仅在 D435i 视觉里程计质量下降时提高事件里程计在融合器中的权重。

验收：快速旋转时 `/odometry/event` 连续，且停止、低纹理时不会被错误事件驱动产生明显漂移。

### 阶段 3：与现有定位融合

目标：不改变 RTAB-Map 的全局地图职责，仅提升其局部输入质量。

1. 加入 EKF/ESKF，融合 Event-VIO、D435i IMU 和轮速计；输出 `/odometry/filtered`。
2. 将 RTAB-Map 的局部里程计输入改为该融合位姿，或使用其作为里程计先验；保留 RTAB-Map 的
   `.db` 重定位和 `map -> odom` 全局纠正。
3. 网页增加事件率、Event-VIO 状态、融合状态和定位协方差，而不是原始事件显示。

验收：快速运动后重新观察已保存地图时，`/rtabmap/localization_pose` 能恢复可信协方差；地图与路径规划
不因局部高频里程计而跳变。

## 运行时监控与停止条件

推荐监控以下量，并将其作为是否启用 Event-VIO 的依据：

| 指标 | 正常行为 | 过载处理 |
|---|---|---|
| `events_per_sec` | 随运动变化 | 超过阈值则缩小 ROI、提高抽样。 |
| 事件窗口队列长度 | 保持有界 | 丢弃最旧窗口，禁止无限缓存。 |
| Event-VIO 处理 P95 | 小于输出周期 | 降低输出率或停止 VIO，只保留 D435i。 |
| D435i RGB/Depth/IMU 频率 | 不受 EVK4 干扰 | 频率下降时先检查 USB 拓扑，再降低事件负载。 |
| Jetson 温度/频率 | 无热降频 | 降低事件处理率，改善散热。 |
| VIO 协方差/跟踪状态 | 有效且连续 | 失效时降低融合权重，等待 RTAB-Map 重定位。 |

## 结论

EVK4 应定位为“高速局部里程计增强器”：在快速转动、振动、HDR 和暗光场景帮助保持连续运动估计；D435i 与
RTAB-Map 继续承担深度建图、历史地图重定位和全局漂移纠正。低 CPU 的关键是：C++ 单进程批处理、硬件滤波、
ROI、固定时间窗口、事件率水位线、只发布位姿/诊断，以及不做稠密事件建图。

在开始阶段 2 前，必须先完成阶段 0 的真实 USB/事件率/`tegrastats` 测量；EVK4 的峰值负载与场景、镜头、
偏置和车速强相关，不能仅依据相机标称带宽估计。
