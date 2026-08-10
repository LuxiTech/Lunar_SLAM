# USB 双目相机：ROS 2 Humble / Jetson Orin NX

锐尔 USB 全局快门双目相机的采集、深度和 RTAB-Map 建图工作区。当前配置为 Ubuntu
22.04、ROS 2 Humble、左右目共同 10 Hz 外触发，并已接入 921600 baud 的 H30 IMU。
当前 H30 已完成零偏恢复，并通过独立 200 Hz 六轴、10 Hz 外触发同步及动态建图验收；
生产链路默认启用融合，健康门禁会在任一 IMU 轴无效时拒绝启动里程计。

> 默认选择 `vpi_learned`：本机复测的优化闭环、地图倾斜和 GPU 峰值持续率更好。
> `stable` 作为低内存、快速深度的显式回退。两条链路不能同时运行。

## 目录结构

```text
ros2_ws/src/
├── usb_camera_driver/             # V4L2 采集、双目深度和标定工具
├── usb_camera_bringup/            # 相机/RGB-D/IMU 采集入口
├── lunar_usb_rtabmap_bringup/     # 唯一完整建图入口
└── third_party/                    # 可选 H30 串口与消息依赖
```

该分层与 D435i 的“设备 bringup + 设备 RTAB-Map bringup”一致。通用适配、视觉前端和
RTAB-Map 算法仍位于 `project/`，设备工作区只保存 USB 专属驱动与组合参数。

## 快速开始

### 1. 构建与环境

```bash
cd /home/nvidia/Desktop/lunar_-slam
export LUXI_WORKSPACE_ROOT="$(pwd)"
./scripts/bootstrap_humble_orin_nx.sh --build usb

source /opt/ros/humble/setup.bash
source install/setup.bash
source device/USBCameraSDK/ros2_ws/install/setup.bash
```

只加载 Humble、项目工作区和 USB 工作区。不要 source 另一套 RTAB-Map overlay。

### 2. 选择并启动一条链路

默认 VPI + Luxi 链：

```bash
ros2 launch lunar_usb_rtabmap_bringup usb_rtabmap.launch.py \
  use_imu:=true planar_mode:=false rviz:=true new_map:=true
```

稳定 CUDA 回退链：

```bash
ros2 launch lunar_usb_rtabmap_bringup usb_rtabmap.launch.py \
  mode:=stable use_imu:=true planar_mode:=false rviz:=true new_map:=true
```

### 3. H30 固定设备名与健康检查

当前设备固定为 `/dev/imu-H30`，只匹配 `1a86:55d4` 且序列号为
`5A6C092260` 的串口，不会占用其他 USB 串口。规则同时阻止 ModemManager 探测：

```bash
ros2 run usb_camera_bringup install_h30_udev_rule.sh
ls -l /dev/imu-H30
```

单独检查 IMU：

```bash
ros2 launch usb_camera_bringup stereo_rgbd.launch.py \
  start_imu:=true depth_backend:=vpi_ofa_pva_vic
ros2 topic hz /imu/data
ros2 topic echo /imu/data --once
```

只有当 `orientation_covariance[0]`、`angular_velocity_covariance[0]` 和
`linear_acceleration_covariance[0]` 都不小于 `0`，静止三轴角速度接近 `0 rad/s`，
且加速度模长接近 `9.81 m/s^2` 时，才启用融合：

```bash
ros2 launch lunar_usb_rtabmap_bringup usb_rtabmap.launch.py \
  mode:=vpi_learned use_imu:=true rviz:=true new_map:=true
```

启动前检查会拒绝无效四元数、任一无效陀螺轴或加速度轴，失败时 RTAB-Map 不会启动。

2026-08-10 对当前 `YIS106-AQMI V01.02.04` 的寄存器查询确认：输出频率码为
`0x09`（`200 Hz`），波特率为 `921600`。故障时用户零偏 X/Y 接近 `int32` 边界，导致
加速度/陀螺通道输出 `INT32_MIN/MAX`。关闭相机和 `SYNC`、完全冷启动后，先把异常用户
零偏归零，再执行静止自估计；最终保存值为 `(-0.056603, 0.029869, 0.139274) dps`。
再次断电启动后连续 2000 帧六轴全部有效，频率 `199.986 Hz`，静止陀螺均值为
`(-0.002273, 0.010074, -0.004751) dps`。

持续零偏估计已关闭，避免重新接入外触发后把复位帧纳入估计。驱动仍保留无效哨兵检测，
启动健康门禁也会拒绝任何无效轴。若再次出现 `±2147.483648`，同时断开 USB 与 `SYNC`
至少 10 秒，在无外触发且完全静止的条件下恢复；不要在外触发工作时改写零偏。

### 4. 正常停止

在启动终端按一次 `Ctrl-C`，等待：

```text
Saving database/long-term memory...done!
```

确认保存完成后再切换链路或拔插相机。不要直接强杀 RTAB-Map。

## 关键文件与话题

| 项目 | 路径或话题 |
| --- | --- |
| 相机配置 | [stereo_camera.yaml](src/usb_camera_driver/config/stereo_camera.yaml) |
| 深度配置 | [stereo_depth.yaml](src/usb_camera_driver/config/stereo_depth.yaml) |
| 双目标定 | [stereo_opencv.yaml](calibration/stereo_opencv.yaml) |
| USB 适配配置 | [usb_adapter.yaml](src/lunar_usb_rtabmap_bringup/config/usb_adapter.yaml) |
| 完整建图入口 | [usb_rtabmap.launch.py](src/lunar_usb_rtabmap_bringup/launch/usb_rtabmap.launch.py) |
| 网页配置 | [web_control.yaml](../../../project/luxi-web-control/config/web_control.yaml) |
| 原子 RGB-D | `/usb_stereo/rgbd_image` |
| 深度 / 实时点云 | `/usb_stereo/depth`、`/usb_stereo/points` |
| RViz 彩色 / 深度预览 | `/usb_stereo/left/image_preview`、`/usb_stereo/depth_preview` |
| 稳定链里程计 | `/rtabmap/odom` |
| 两条链路的生产里程计 | `/rtabmap/odom` |

同一对左右图像使用相同时间戳；原子 RGB-D 内的彩色、深度和标定内参也使用同一时间戳。

## 两条完整建图链路总览

USB 双目当前保留两条可独立启动的完整链路。两者共用相机、标定、话题命名和
`luxi_adapter`，但深度后端、里程计前端和各自调优参数不同。同一时刻只能运行一条：

| 项目 | 稳定 CUDA 链路 | VPI + Luxi 链路 |
| --- | --- | --- |
| 网页模式 ID | `stable` | `vpi_learned` |
| 启动参数 | `usb_rtabmap.launch.py mode:=stable` | `usb_rtabmap.launch.py mode:=vpi_learned` |
| 深度算法 | OpenCV CUDA StereoSGM | VPI OFA eSGM + PVA/VIC |
| 视觉里程计 | RTAB RGB-D GFTT/ORB + 5 帧局部 BA | RTAB RGB-D GFTT/ORB + 5 帧局部 BA |
| 建图特征 | RTAB 经典特征 | TensorRT FP16 SuperPoint + LightGlue 外部描述子 |
| IMU | 支持 H30，健康检查通过后启用 | 支持 H30，健康检查通过后启用 |
| 主要优点 | 深度计算更快、内存约少 1 GiB | 闭环更多、图姿态更平、GPU 高峰持续率更低 |
| 当前限制 | 本轮快速转动时重置一次，优化后倾斜较大 | RAM 多约 1 GiB；本轮快速转动时也重置一次 |
| 推荐用途 | 低内存回退、单独检查 CUDA 深度 | 默认生产建图、并行感知和低倾斜地图 |
| 复测数据库 | `usb_default_ab_20260810/cuda_*.db` | `usb_default_ab_20260810/vpi_*.db` |

默认已切换为 `vpi_learned`。最终 VPI 链达到毫米到厘米级闭环和低倾斜的生产可建图
状态，并为其他 GPU 任务保留更稳定的调度余量。人工手持闭环不是动捕真值；没有外触发
IMU 时，只能判定工程表现达到 D435i 量级，不能宣称绝对 ATE 优于 D435i。

### 参数隔离范围

| 范围 | 说明 |
| --- | --- |
| 两链共享 | 1920×1080 双目采集、`stereo_opencv.yaml` 标定、960×540 匹配尺度、480×270 RGB-D 输出、0.4–3.0 m 建图区间、校正有效区和远处薄片抑制 |
| 仅稳定链生效 | `cuda_sgm_*`、CUDA 反向 BM/远处左右一致性、`mode:=stable` 中的 GFTT/ORB、局部 BA 和经典 RGB-D 里程计参数 |
| 仅 VPI 链生效 | `vpi_*`、发布端 5×5 有界中值、两线程 OpenCV 后处理、Luxi 外部描述子、24° 空间回环视角门限和对应 RTAB 参数 |

虽然两种后端的参数都记录在
[stereo_depth.yaml](src/usb_camera_driver/config/stereo_depth.yaml)，节点会按
`depth_backend` 分支读取和执行。VPI 参数不会改变 CUDA 计算，Luxi 的鲁棒深度和
`PNP_DEPTH` 开关默认关闭，也不会影响 HIK、D435i 或稳定 USB 链路。

### 网页选择

网页“建图方案”对应服务端白名单，不允许浏览器注入任意 launch 参数：

- `VPI OFA/PVA/VIC + Luxi 学习前端` → `vpi_learned`，默认选项；
- `稳定 CUDA + 经典前端` → `stable`，显式回退。

网页启动时两条方案均会自行占用 USB 相机并以 `rviz:=false` 运行，因此不要预先手工启动
`stereo_rgbd.launch.py` 或另一条建图链。点击“停止建图”后应等待 RTAB-Map 打印
`Saving database...done!`，再切换方案或拔插相机。网页配置位于
`project/luxi-web-control/config/web_control.yaml`。

## 独立 RGB-D 调试

以下命令只启动相机和深度，不启动适配层、里程计或 RTAB-Map：

```bash
cd ${LUXI_WORKSPACE_ROOT}/device/USBCameraSDK/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch usb_camera_bringup stereo_rgbd.launch.py start_imu:=false
```

独立 RGB-D 入口仍默认 CUDA；追加 `depth_backend:=vpi_ofa_pva_vic` 可单独检查 VPI。
完整建图默认值由 `usb_rtabmap.launch.py` 管理。只检查 RGB-D 时保持
`start_imu:=false`。`/usb_stereo/depth_preview` 仅裁剪显示黑边；SLAM 必须使用带正确
内参的 `/usb_stereo/depth` 或原子 `/usb_stereo/rgbd_image`。

## 链路细节

### 稳定 CUDA + 经典前端

```text
USB 双目 → CUDA StereoSGM → luxi_adapter → RTAB RGB-D 里程计 → RTAB-Map
```

- 使用 GFTT/ORB、局部特征图和 5 帧有界 BA；
- 远处左右一致性与薄片过滤抑制倾斜伪点云；
- 手持使用 `planar_mode:=false`；仅地面机器人可设为 `true`；
- 关键节点：`/usb_stereo_depth_node`、`/rtabmap/rgbd_odometry`、
  `/usb_planar_odometry`、`/rtabmap/rtabmap`。

### 稳定链验收：map048

以下结果在 Jetson Orin NX `MAXN_SUPER`、双相机共同 10 Hz 外触发、无 IMU 条件下
取得。最终参数位于
[stereo_depth.yaml](src/usb_camera_driver/config/stereo_depth.yaml) 和
[usb_rtabmap.launch.py](src/lunar_usb_rtabmap_bringup/launch/usb_rtabmap.launch.py)。

| 检查项 | map048 实测结果 |
| --- | --- |
| 测试路径 | 2.46 m，30 个有效关键帧 |
| 动态跟踪 | 0 次丢失，0 次注册失败 |
| 闭环 | 11 条；连续里程计首尾 3.57 cm，图优化后 8 mm |
| 回到起点后静止 | 15 s 首尾 2.15 mm，无伪关键帧 |
| 视觉里程计 | 5.17 Hz，内点中位数 302、P10 为 290 |
| 深度有效率 | 静止约 40-43%，快速转动最低约 28.5% |
| 实时点云 | 最大实测 Z 为 2.992 m，5 Hz、单帧显示 |
| 累积地图 | 约 1.84 万点，优化后首尾一致 |

该配置适用于室内 0.4–3.0 m 建图。被动双目在弱纹理、暗光、反光或远距离场景中的
填充率低于主动红外深度相机，因此验收重点是跟踪、闭环和累计地图，而不是单帧深度
填充率。

### VPI OFA/PVA/VIC + Luxi 学习前端

```text
USB 双目 → VPI OFA eSGM + PVA/VIC → 原子 RGB-D ─┬→ RTAB F2M + 5 帧 BA → /rtabmap/odom
                                                 └→ Luxi SuperPoint/LightGlue → RTAB-Map
```

RTAB F2M 直接读取高频原子 RGB-D 并独占里程计 TF；Luxi 以 1 Hz 向 RTAB-Map 提供
SuperPoint 浮点描述子，不再用独立双帧 PnP 累积生产位姿。这样保留学习特征的回环能力，
同时用局部地图 BA 抑制快速转动时的姿态漂移。参数与稳定链、HIK 和 D435i 链路隔离。

USB 的 480×270 输入需要在当前 NX 上生成一次 TensorRT FP16 SuperPoint 引擎：

```bash
ros2 run luxi_visual_frontend build_superpoint_tensorrt.py \
  --height 270 --width 480 --precision fp16
```

最终配置使用单次 OFA、绝对置信度 `64875`、5×5 有界中值和 2 个 OpenCV 后处理线程。
两次 OFA 会把处理延迟增至约 113 ms，单 OpenCV 线程也有相同吞吐损失，均已实测否决。
RTAB-Map 使用 `RGBD/ProximityAngle=24` 和 `RGBD/OptimizeMaxError=3`，拒绝会拉斜地图的
大视角局部空间回环。日志应出现 `Initialized USB VPI OFA+PVA+VIC SGM`；前端诊断应为
`superpoint_backend=tensorrt_fp16`。回退 CPU SGBM 的测试不计入 VPI 性能结果。

## 实测对比

以下静止资源数据于 2026-08-10 在同一台 NX、同一场景、无 IMU、关闭 RViz 条件下，
分别预热后连续采样 45 秒。动态精度为同一场景、尽量复现的手持往返路线；没有动捕，
因此是闭环重复性和地图倾斜比较，不是绝对 ATE。

| 指标 | VPI + Luxi | 稳定 CUDA |
| --- | ---: | ---: |
| 里程计平均/中位频率 | 5.10 / 7.26 Hz | 5.54 / 9.36 Hz |
| 静止末端平移漂移 | 11.83 mm | 2.90 mm |
| 静止最大平移/旋转漂移 | 30.75 mm / 1.069° | 17.90 mm / 0.652° |
| VPI 前端接受跟踪 | 161 / 161 | 不适用（经典前端） |
| VPI 平均内点/内点率 | 257.7 / 77.00% | 不适用（经典前端） |
| GR3D 平均 / P90 / 峰值 | 58.3% / 84% / 98% | 61.4% / 97% / 99% |
| GR3D ≥90% 采样占比 | 5.7% | 20.0% |
| 平均功耗 | 11.66 W | 12.60 W |
| 最大单进程 CPU | 0.848 核（深度） | 0.813 核（里程计） |
| 系统 RAM | 7.44 GiB | 6.38 GiB |
| 动态有效优化轨迹 | 1.60 m | 2.21 m |
| 快速转动跟踪重置 | 1 次 | 1 次 |
| 全局 / 空间闭环 | 4 / 23 | 1 / 10 |
| 图优化后首尾误差 | **6.63 mm** | 13.18 mm |
| 图优化后 roll/pitch 合成倾斜 | **1.18°** | 2.82° |

CUDA 的静止漂移和瞬时帧率更好；VPI+Luxi 的动态闭环数量、优化后首尾误差、地图倾斜
及 GPU 高负载持续率更好，所以最终选择 VPI+Luxi 为默认。两次手持路径长度不同，表中
闭环精度只用于工程选型。测试库保存在
`maps/benchmarks/usb_default_ab_20260810/`，不会显示在正式地图列表中。

### 当前默认 VPI + Luxi + H30 全链路验收（2026-08-10）

测试条件为 Orin NX、Ubuntu 22.04、ROS 2 Humble、USB 双目 10 Hz 外触发、H30
外触发与 IMU 融合、VPI + Luxi 默认参数，并打开 RViz 累积点云和栅格。连续运行约
7.4 分钟后正常停止，RTAB-Map 已完整保存 `/tmp/usb_imu_acceptance.db`（164 MiB）。
以下“精度”是轨迹重复性、重力一致性和图约束的工程评价；本次没有动捕或标定尺真值，
不能换算成绝对 ATE，也不能据此宣称绝对深度精度超过 D435i。

| 建图与跟踪指标 | 实测结果 | 判定 |
| --- | ---: | --- |
| 深度输入 / 立体匹配 / SLAM 输出 | 1920×1080 / 960×540 / 480×270 | 保留全分辨率匹配，降低发布与建图负载 |
| 深度有效像素，预热后 | 平均 39.2%，中位 40.7%，P10 36.1% | 室内纹理场景稳定；弱纹理仍是被动双目短板 |
| 深度处理，预热后 | 平均 99.8 ms，中位 99.0 ms，P95 105.5 ms | 约 7.1 Hz 实际输出，无持续积压 |
| RGB-D 里程计，10 秒窗口 | 44 帧，丢失 0 帧 | 移动期间连续跟踪 |
| 视觉内点 | 平均 254，最低 237，局部地图 700 | 余量充足，没有贴近失锁阈值 |
| 活跃优化图 | 30 节点、29 邻接、13 局部空间闭环、30 重力约束 | 往返路径存在有效几何约束 |
| 图优化路径 / 里程计累计路程 | 1.706 m / 5.492 m | 中间节点被合并，不应按数据库节点数计算路径 |
| 回停后 10 秒漂移 | 2.2 mm / 0.082° | 优良 |
| IMU 重力 roll / pitch 平均误差 | 0.122° / 0.058° | 未观察到持续地图倾斜 |
| 累积点云 / 栅格 | 16381 点；100×157，5 cm/格，3031 个已知格 | 适合室内结构与障碍物建图 |
| 保存库 | 322 数据节点、86290 个特征，平均 268 个/节点 | 数据完整，特征中位数 316 |

综合判定为“优良的室内实时建图状态”：移动中没有里程计丢失，内点和局部空间闭环
充足，IMU 重力约束消除了此前明显的倾斜趋势，回停漂移很小；地图的实际可辨颗粒由
`Grid/CellSize=0.05` 限定为 5 cm。它仍不是 D435i 主动红外深度的等价替代：暗光、
白墙、反光面和 3 m 以外区域的单帧密度会较低。需要比较厘米级绝对精度时，应再用
同一路线真值或固定尺寸标靶做 ATE、尺度误差和平面残差测试。

资源数据取自同一次完整链路（包含 RViz）的 10 秒进程采样。CPU 数值以一个核心为
100%，占比是下列链路进程合计 `235.44%`（约 2.35 核）中的比例；RSS 合计约
2753 MiB（2.69 GiB），会重复计入共享库和共享缓冲区，不等于关闭链路后必然释放的
系统内存。

| 模块 | CPU | 链路 CPU 占比 | RSS | 链路 RSS 占比 |
| --- | ---: | ---: | ---: | ---: |
| VPI 深度 | 78.11% | 33.18% | 338.9 MiB | 12.31% |
| Luxi SuperPoint/LightGlue | 59.93% | 25.45% | 1550.8 MiB | 56.33% |
| RGB-D 里程计 | 40.75% | 17.31% | 181.5 MiB | 6.59% |
| RTAB-Map | 21.68% | 9.21% | 329.0 MiB | 11.95% |
| RViz | 12.99% | 5.52% | 168.3 MiB | 6.11% |
| H30 驱动 | 12.19% | 5.18% | 23.2 MiB | 0.84% |
| RGB-D/IMU 适配器 | 7.29% | 3.10% | 67.5 MiB | 2.45% |
| USB 双目采集 | 1.40% | 0.59% | 61.5 MiB | 2.23% |
| 点云稳定器 | 1.10% | 0.47% | 32.3 MiB | 1.17% |
| **合计** | **235.44%** | **100%** | **2752.9 MiB** | **100%** |

`tegrastats` 以 500 ms 间隔采样 31 次：整机 GR3D 平均 `37.2%`、P95 `97%`、峰值
`98%`，其中 4/31 次不低于 95%，没有一次达到 99%；八个 CPU 核采样峰值均不超过
`86%`。Jetson 的 `tegrastats` 不提供可信的逐进程 GPU 百分比，因此不虚构模块 GPU
占比：Luxi TensorRT/LightGlue 是主要 CUDA 负载，深度节点主要使用 OFA/PVA/VIC 并
有少量 GPU 后处理，RViz 使用 GPU 渲染，里程计、RTAB-Map、H30 和适配器以 CPU 为主。
如需逐模块 GPU 时间，应使用 Nsight Systems 做隔离 A/B profiling。

## 复测与选型

固定相机和场景、关闭 RViz，并保持相同 Jetson 功耗模式：

```bash
python3 device/USBCameraSDK/ros2_ws/scripts/benchmark_vpi_depth.py
python3 device/USBCameraSDK/ros2_ws/scripts/benchmark_usb_slam.py
```

`benchmark_vpi_depth.py` 需要已经运行 USB 相机但不能同时运行另一个深度节点；
`benchmark_usb_slam.py` 需要完整 VPI 建图链已进入 `TRACKING`。

- 默认建图、需要更多闭环、更平的地图或同时运行其他 GPU 网络：选择 `vpi_learned`；
- 只重视静止漂移、瞬时帧率或低内存：显式选择 `stable`；
- 需要切换链路：先正常停止并确认数据库保存，再启动另一条，不能热切换深度后端；
- H30 外触发动态验收已经完成；修改相机/IMU 外参、触发接线、分辨率或安装刚性后，
  应重新做动态 A/B，不能直接沿用本次结论。

## 仅启动相机

先编辑 [stereo_camera.yaml](src/usb_camera_driver/config/stereo_camera.yaml)，再启动：

```bash
cd ${LUXI_WORKSPACE_ROOT}/device/USBCameraSDK/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch usb_camera_bringup stereo_camera.launch.py
```

该命令默认会同时打开 RViz 左右目视图。仅启动相机节点时使用：

```bash
ros2 launch usb_camera_bringup stereo_camera.launch.py use_rviz:=false
```

使用临时配置文件，不修改默认配置：

```bash
ros2 launch usb_camera_bringup stereo_camera.launch.py \
  params_file:=/absolute/path/to/stereo_camera.yaml
```

复现历史双目与 H30 主机时间戳链路，并应用 `run03_left` 空间外参和时间补偿：

```bash
cd ${LUXI_WORKSPACE_ROOT}/device/USBCameraSDK/ros2_ws
./scripts/start_stereo_imu.sh
```

该启动方式只用于复核历史数据，不是当前默认建图入口。它会发布
`left_camera_optical_frame -> imu_link` 静态 TF，生效参数保存在
[kalibr_cam_imu.yaml](src/usb_camera_bringup/config/kalibr_cam_imu.yaml)，图像时间戳使用
历史 `-84.344 ms` 补偿。IMU 接入新的外部触发后不要沿用这个时间补偿，应按后文重新
录制并联合标定；标定采集必须保持原始时间戳，避免重复补偿。

## 图像可视化

### 左目低延迟调焦

调整左目定焦镜头时不要使用 RViz，使用专用 OpenCV 调焦窗口：

```bash
cd ${LUXI_WORKSPACE_ROOT}/device/USBCameraSDK/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch usb_camera_bringup left_focus.launch.py
```

窗口左侧是左目完整画面和中央绿色 ROI，右侧是 ROI 像素放大图：

| 指标 | 说明 |
| --- | --- |
| `FOCUS` | 当前 ROI 的平滑清晰度分数 |
| `PEAK` | 本轮调整出现过的最高分数 |
| `FPS` | 实际显示帧率 |
| `LATENCY ms` | 相机发布到显示处理完成的延迟 |

把棋盘格或印刷清晰的文字放在计划使用的工作距离，并置于绿色框内。目标和相机保持不动，按 `r` 清除旧峰值，然后缓慢旋转左目镜头；每次小幅调整后等待约半秒，以 `FOCUS` 和 `PEAK` 的最大值作为最佳位置。清晰度分数是相对值，只能在同一目标、距离和光照下比较，没有通用合格阈值。按 `q` 或 `Esc` 退出。

镜头调整完成后必须重新执行双目标定，因为焦距变化会改变相机内参。调焦参数在 [opencv_focus_viewer.yaml](src/usb_camera_driver/config/opencv_focus_viewer.yaml) 中配置。

### RViz 预览

先在第一个终端启动相机节点，在第二个终端执行：

```bash
source /opt/ros/humble/setup.bash
source ${LUXI_WORKSPACE_ROOT}/device/USBCameraSDK/ros2_ws/install/setup.bash
rviz2
```

在 RViz 中添加两个 `Image` 显示项，分别选择 `/left_camera/image` 和 `/right_camera/image`。

快速查看单路图像：

```bash
rqt_image_view /left_camera/image
rqt_image_view /right_camera/image
```

检查左目实际发布帧率：

```bash
ros2 topic hz --qos-reliability best_effort /left_camera/image
```

## OpenCV 双目标定

工作区使用独立的 C++ OpenCV 标定器，不再调用 Python `camera_calibration`。相机保持 `1920x1080`、外触发 `10 Hz`，ROS 只传输相机原生 JPEG；预览只消费最新同步帧，角点检测和参数求解都在后台执行。

### 1. 检查标定板

当前配置对应：

- 棋盘格内角点：横向 `11`、纵向 `8`
- 单个方格边长：`0.010 m`，即 `10 mm`
- 相机模型：OpenCV 针孔模型和常规畸变模型

这里填写的是内角点数量，不是黑白方格数量。方格尺寸要用卡尺确认，填写错误会直接造成双目基线尺度错误。

### 2. 启动标定程序

```bash
cd ${LUXI_WORKSPACE_ROOT}/device/USBCameraSDK/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch usb_camera_bringup stereo_calibration.launch.py
```

临时修改棋盘参数：

```bash
ros2 launch usb_camera_bringup stereo_calibration.launch.py \
  board_cols:=11 board_rows:=8 square_size:=0.010
```

### 3. 采集标定样本

窗口顶部出现 `corners OK` 时，说明左右相机都完整识别到了棋盘。保持标定板静止，按一次空格保存当前左右图像的角点。

至少采集 `20` 组，建议采集 `25-35` 组，并覆盖以下姿态：

- 图像中央、四角和四条边缘
- 近、中、远三种距离，棋盘不能小到难以识别
- 绕水平轴、垂直轴分别向两侧倾斜
- 棋盘始终完整出现在左右两幅图中，避免反光和运动模糊

不要在同一个位置连续按空格。每次移动后停稳，确认 `corners OK` 再采样。

### 4. 求解并检查结果

采满后按 `c` 开始 OpenCV 求解。求解在后台运行，预览不会停止。完成后程序自动保存结果并切换到校正预览，绿色水平线应该穿过左右图像中的同一物体高度。

输出文件：

```text
${LUXI_WORKSPACE_ROOT}/device/USBCameraSDK/ros2_ws/calibration/stereo_opencv.yaml
```

终端会打印左目 RMS、右目 RMS 和双目 RMS。建议双目 RMS 小于 `0.5 px`；`0.5-1.0 px` 可以使用但建议重新优化采样；大于 `1.0 px` 应重新标定。还要检查 YAML 中 `translation` 的模长是否接近实际测量的左右相机基线。

按键：

| 按键 | 功能 |
| --- | --- |
| `Space` | 保存当前有效角点对 |
| `c` | 使用已保存样本开始标定 |
| `r` | 清除样本和当前标定结果 |
| `q` 或 `Esc` | 退出程序 |

标定参数集中在 [opencv_stereo_calibrator.yaml](src/usb_camera_driver/config/opencv_stereo_calibrator.yaml)。终端每 3 秒打印实际预览帧率和端到端延迟。标定时不要同时启动 RViz、`rqt_image_view` 或额外的 `ros2 topic hz`，避免增加 DDS 订阅负载。

本机双相机实测预览稳定在 `9.35-10.05 Hz`，稳定阶段平均延迟约 `16-29 ms`。在线后台求解 24 组样本耗时约 `0.18 s`，求解所在统计窗口仍为 `9.96 Hz / 21.9 ms`，没有产生后续帧积压。

### 5. 独立验证标定结果

重新打开终端，加载已保存的 YAML 并启动实时校正验证：

```bash
cd ${LUXI_WORKSPACE_ROOT}/device/USBCameraSDK/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch usb_camera_bringup stereo_validation.launch.py
```

验证程序首先检查 YAML 矩阵、左右 RMS、双目 RMS 和基线。然后使用未参与标定的新姿态，把棋盘分别放在中央、四角、近处和远处。左右都检测到棋盘后，终端会自动打印：

```text
rectification check: mean vertical error ... px, max ... px [PASS/FAIL]
```

全部姿态同时满足以下条件才建议用于双目深度：

| 检查项 | 通过标准 |
| --- | --- |
| 左目 RMS | `< 0.5 px` |
| 右目 RMS | `< 0.5 px` |
| 双目 RMS | `< 0.5 px` |
| 校正后平均垂直误差 | `< 0.5 px` |
| 校正后最大垂直误差 | `< 1.0 px` |
| 基线长度 | 接近卡尺测量值，建议误差不超过 `1-2 mm` |

绿色水平线用于肉眼复核：同一物体特征在左右画面中应处于同一水平线。只看一组中央姿态不够，必须使用多个未参与标定的姿态测试。验证文件路径在 [opencv_stereo_validator.yaml](src/usb_camera_driver/config/opencv_stereo_validator.yaml) 中配置。

## 相机调参

所有参数统一在 [stereo_camera.yaml](src/usb_camera_driver/config/stereo_camera.yaml) 中配置。

| 参数 | 说明 | 当前值 |
| --- | --- | --- |
| `left_device` | 左目稳定设备路径 | USB 端口 `0:8` |
| `right_device` | 右目稳定设备路径 | USB 端口 `0:7` |
| `pixel_format` | 图像格式，可选 `MJPG`、`YUYV` | `MJPG` |
| `output_encoding` | ROS 输出，可选 `bgr8`、`mono8`、标定专用 `jpeg` | 正常启动为 `bgr8` |
| `width`、`height` | 采集分辨率 | `1920 x 1080` |
| `frame_rate_hz` | 请求的 UVC 流帧率 | `10.0` |
| `trigger_mode` | `hardware` 外触发，`video` 连续视频 | `hardware` |
| `auto_exposure` | 自动曝光开关 | `true` |
| `exposure_absolute` | 手动 V4L2 曝光值，仅关闭自动曝光后生效 | `-1` |
| `gain` | 手动增益，`-1` 表示不修改相机当前值 | `-1` |
| `left_frame_id`、`right_frame_id` | TF 光学坐标系名称 | 左右光学坐标系 |

### 曝光和增益

自动曝光配置：

```yaml
auto_exposure: true
exposure_absolute: -1
gain: -1
```

手动曝光和增益时，左右相机必须使用相同数值，避免双目亮度不一致：

```yaml
auto_exposure: false
exposure_absolute: 100
gain: 20
```

`exposure_absolute` 是 V4L2 控制值，不保证等于微秒。建议从较小值开始，在 RViz 中确认亮度后逐步增大。外触发模式下，曝光时间与传感器帧时间之和必须小于触发周期。

## 外部触发

`trigger_mode: hardware` 会设置 `Backlight Compensation=2`，即该相机的硬件外触发模式。

两台相机必须接收同一触发信号，并与同步模块共地。10 Hz 单帧外触发建议：

```text
触发电平：1.8 V 基准
触发周期：100 ms
高电平脉宽：3-5 ms
触发边沿：上升沿
```

不要把 3.3 V PWM 直接接入 `SYNC/Trigger`。应使用 3.3 V 转 1.8 V 电平转换器，并连接公共 GND。高电平过长会使相机进入自动连续触发，10 Hz 方波可能表现为约 20 fps。

## 双目与 H30 IMU 联合标定

当前 map048 建图没有使用 IMU。历史 `run03_left` 是在 H30 未接入相机同源外部触发、
两端都使用主机接收时间时得到的结果。只要相机、镜头和 IMU 的刚性安装没有变化，
其空间外参可作为新标定初值；一旦把 IMU 接入新的外部触发，必须重新估计相机—IMU
时间偏移并重新验收联合外参，不能直接复用 `-84.344 ms`。如果镜头、焦距或相机与
IMU 的相对位置发生变化，则空间外参也必须从头标定。

联合标定使用当前已经验证通过的 `1920x1080` 双目内外参。默认只让 Kalibr 求解左目到 IMU 的外参与一个时间偏移，右目外参再由固定双目基线派生：

```text
T_right_imu = T_right_left * T_left_imu
```

不要让 Kalibr 同时为硬同步左右目分别估计时间偏移。实测这种双目联合优化会把主机时间戳误差耦合进平移外参，导致两次标定的平移相差 `21.38 mm`；改为左目与 IMU 单独求解后，同一批两组数据的平移差降至 `1.14 mm`，旋转差为 `0.086 deg`。Kalibr 配置集中在 [calibration/kalibr](calibration/kalibr)，采集时使用 `mono8`，保持全部像素信息，同时把录包体积降为 BGR 图像的约三分之一。

固定条件：

- 相机和 IMU 必须刚性固定，录制后不能再调整镜头或相对位置。
- 左右相机保持共同 `10 Hz` 上升沿外触发。
- H30 标准话题为 `/imu/data`，设备内部配置为 `200 Hz`。
- 标定板为 `11x8` 个内角点，方格边长 `10 mm`。
- 棋盘固定不动，移动整个相机与 IMU 支架。

### 1. 编译相机与 H30 驱动

H30 驱动已经迁入当前工作区的 `src/third_party/yesense_ros2`，不需要再编译或 source 海康相机工作区：

```bash
cd ${LUXI_WORKSPACE_ROOT}/device/USBCameraSDK/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

### 2. 启动双目和 IMU

```bash
cd ${LUXI_WORKSPACE_ROOT}/device/USBCameraSDK/ros2_ws
./scripts/kalibr/start_stereo_imu.sh
```

该启动方式不运行 RViz 或深度节点。另开终端检查：

```bash
source /opt/ros/humble/setup.bash
source ${LUXI_WORKSPACE_ROOT}/device/USBCameraSDK/ros2_ws/install/setup.bash
export FASTDDS_DEFAULT_PROFILES_FILE=${LUXI_WORKSPACE_ROOT}/device/USBCameraSDK/ros2_ws/install/usb_camera_bringup/share/usb_camera_bringup/config/fastdds_large_image.xml
ros2 topic hz /left_camera/image
ros2 topic hz /right_camera/image
ros2 topic hz /imu/data
ros2 topic echo /imu/data --once
```

左右目应约为 `10 Hz`；IMU 的 `frame_id` 应为 `imu_link`，静止时加速度模长应接近
`9.81 m/s^2`。当前连接的 H30 使用 `921600 baud`，寄存器确认其内部输出间隔为
`5 ms`（`200 Hz`）。设备输出 `0x51 sample_timestamp`，并在每次 10 Hz 外触发时复位；
每个触发周期存在约一个样本的空档，串口线上实测约 `184.6-189.5 Hz`。驱动用复位帧
锚定时间，并丢弃该帧中偶发的半更新数据，因此 ROS 话题实测约 `176 Hz`。这不是把
设备降频到了 176 Hz。相机到最近 IMU 的 15 秒实测中位差为 `-0.64 ms`，无需增加
视觉前端时间偏移。

独立 IMU 恢复验收中，rosbag 在 `9.810 s` 内录到 `1963` 条消息，即 `200.10 Hz`；
健康门禁统计 `imu_valid=400`、`imu_invalid=0`。接回 10 Hz `SYNC` 后，10 秒检测到
99 次时间戳复位，非复位帧六轴无效数为 0，相机—最近 IMU 时间差中位数 `-1.10 ms`、
95% 小于 `2.15 ms`。动态建图得到 30 个优化节点、13 条局部空间闭环和 30 条重力约束，
视觉内点平均 254、最低 237；回停 10 秒漂移为 `2.2 mm / 0.082 deg`。因此正式入口
与网页控制均默认使用 `use_imu:=true`，无 IMU 诊断时再显式关闭。

同次 VPI + Luxi + H30 + RTAB-Map + RViz 负载采样中，GPU 平均 `37.2%`、P95
`97%`、最大 `98%`，31 次采样没有一次达到 `99%`；八个 CPU 核最高值均不超过
`86%`。主要进程的单核占用为 VPI 深度 `78.1%`、Luxi 前端 `59.9%`、RGB-D 里程计
`40.8%`、RTAB-Map `21.7%`、H30 驱动 `12.2%`，满足单核不超过 100% 且 GPU 不频繁
99% 的约束。

当前相机固件虽然给 V4L2 缓冲区标记了 `MONOTONIC/SOE`，但实测其缓冲区时间戳不会逐帧递增，不能直接使用。相机驱动在 `VIDIOC_DQBUF` 返回时记录主机时间，并取左右目出队时刻中点作为共同时间戳，从而排除 MJPEG 解码和 ROS 发布耗时；固定的 USB 传输延迟由 Kalibr 的 `timeshift_cam_imu` 求解。可用下面的命令检查时间戳链路，延迟应稳定且不能随时间持续增长：

```bash
ros2 topic delay /left_camera/image
```

### 3. 录制动态数据

在第二个终端执行：

```bash
cd ${LUXI_WORKSPACE_ROOT}/device/USBCameraSDK/ros2_ws
./scripts/kalibr/record_stereo_imu.sh run01
```

看到 `Recording` 后开始移动，建议录制 `3-5 min`：先静止 5 秒，然后依次进行左右、上下、前后平移，再绕 X、Y、Z 三轴正反旋转，最后组合平移与旋转并静止 5 秒。棋盘应持续完整出现在左右画面中，动作要连续但不能产生明显运动模糊。完成后按一次 `Ctrl+C`。

数据默认保存到：

```text
calibration/data/run01_ros2
```

检查录包：

```bash
ros2 bag info calibration/data/run01_ros2
```

持续时间应为 `180-300 s`；左右图像数量分别约为 `Duration x 10`。当前外触发与丢弃
复位帧策略下，IMU 数量约为 `Duration x 176`；设备寄存器仍应保持 `200 Hz`。

### 4. 转换并运行 Kalibr

```bash
cd ${LUXI_WORKSPACE_ROOT}/device/USBCameraSDK/ros2_ws
./scripts/kalibr/convert_bag.sh calibration/data/run01_ros2
./scripts/kalibr/run_calibration.sh calibration/data/run01.bag
```

该命令默认使用保留的 `camchain_left.yaml`，只求解左目与 IMU 外参。

如果录制中某一段发生串口断流、标定板丢失或剧烈运动模糊，可仅使用干净时间段求解。例如使用第 `40-180 s`：

```bash
./scripts/kalibr/run_calibration.sh calibration/data/run01.bag 40 180
```

本机使用 `luxi-kalibr:rosbags` Docker 镜像。结果保存在 ROS 1 bag 同目录：

当前 H30 已接入 10 Hz 外触发并输出采样时间戳。重新联合标定时应保留原始硬件时间，
不要重复施加历史 `-84.344 ms` 补偿；只有在三轴陀螺恢复正常后，标定结果才可用于融合。

```text
run01-camchain-imucam.yaml
run01-imu.yaml
run01-results-imucam.txt
run01-report-imucam.pdf
```

验收时要求优化正常结束、左目有充足角点、重投影误差通常小于 `1 px`，并检查 `T_cam0_imu` 平移是否符合设备实际尺寸。至少独立录制三组数据；三次平移差建议小于 `5 mm`、旋转差小于 `0.5 deg`。本机两组历史数据使用该流程得到：平移差 `1.14 mm`、旋转差 `0.086 deg`、重投影均值分别为 `0.640 px` 和 `0.707 px`。

两组历史数据的时间偏移分别为 `-88.626 ms` 和 `-84.344 ms`，差值 `4.282 ms`。
空间外参重复性良好，但历史时间重复性没有达到 `<2 ms`，因此当前融合只复用空间
外参，不加载该历史时间补偿。现在同步模块触发沿已同时接入 H30，驱动按设备触发复位
时间戳锚定数据；相机—最近 IMU 实测中位差 `-1.10 ms`、P95 `2.15 ms`，已通过建图
健康门禁。若需要标定级绝对精度，仍应按当前触发接线重新录制至少三组数据，完成新的
时间偏移与空间外参重复性验收。

当前仅保留 `calibration/data/run03_ros2` 原始数据和 `run03_left-*` 最终结果。ROS1 bag 属于可重新生成的中间文件，完成求解后无需长期保存。

## 常见问题

| 现象 | 排查方式 |
| --- | --- |
| 硬触发模式没有图像 | 检查 1.8 V 电平、GND 共地、上升沿与 3-5 ms 脉宽。 |
| 10 Hz 同步模块却得到约 20 fps | 高电平脉宽过长，改为短脉冲，不要使用 50% 占空比方波。 |
| 一路图像马赛克 | 保持 `MJPG` 与 `frame_rate_hz: 10.0`，两台相机共用 USB 2.0 控制器。 |
| 双目深度异常 | 完成双目标定，并在接入深度或 SLAM 前提供有效内参和外参。 |
| `/dev/video*` 变化 | 执行 `ls -l /dev/v4l/by-path/`，更新左右设备路径。 |
