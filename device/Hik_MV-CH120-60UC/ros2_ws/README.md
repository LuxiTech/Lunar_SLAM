# 海康双目 + H30 IMU + RTAB-Map

面向 **Jetson Orin NX、Ubuntu 22.04、ROS 2 Humble、ARM64** 的实时双目建图工作区。

系统使用两台 MV-CH120-60UC 硬件触发相机、H30 IMU、NVIDIA VPI 立体匹配和 RTAB-Map。当前标定与运行基准为 **1024×750、10 Hz**。

> 运行时相机保留完整视场：相机先输出 2048×1500，再以 `INTER_AREA` 二次下采样为 1024×750；不裁剪、不缩放视角。双目标定文件同样是 1024×750，因此图像、深度和 `CameraInfo` 始终一致。

## 1. 系统与算法

```text
两台 MV-CH120-60UC（外部硬件触发，2048×1500）
                     │
                     ▼
stereo_node：Bayer→BGR，全视场 2:1 下采样，发布 1024×750 双目与 CameraInfo
                     │
                     ▼
stereo_depth_node：去畸变 / 极线校正 / 右图垂直 2 px 补偿
                     │
                     ├─ VPI OFA eSGM（单遍）→ VIC 格式转换
                     │     └─ 输出 /stereo/depth、/stereo/points、预览图
                     │
                     └─ RGB-D 打包（2.5 Hz）
                              │
H30 AHRS ── /imu/data ──► rgbd_odometry ──► /odom ──► RTAB-Map
                              │                         │
                              └─ 相机-IMU 标定 TF        └─ /map、/cloud_map、/mapPath
```

| 模块 | 当前实现 | 目的 |
| --- | --- | --- |
| 相机同步 | 外部硬件触发、SDK 公共主机时间戳 | 降低左右帧时间差与视觉延迟 |
| 立体校正 | 双目标定 + `right_rectification_y_offset_px=2` | 使极线对齐后再匹配 |
| 默认深度 | VPI `vpi_ofa_pva_vic`、单遍 eSGM | 当前 JetPack 实际由 OFA + VIC 执行，1024×750 接近 10 Hz |
| 深度回退 | `vpi_cuda`、OpenCV `sgbm` | VPI 排障或画质对比 |
| 视觉里程计 | RTAB-Map RGB-D F2M + IMU 姿态初始化 | 输出平滑的 `/odom` |
| 建图 | RTAB-Map 位姿图、回环、2D 占据栅格 | 控制在线计算量，稳定累积地图 |

### 性能边界

- 单遍 OFA 深度目标为 **10 Hz**；完整建图、实时点云和本机 RViz 同时运行时，实测处理约 **78–93 ms/帧**。
- RTAB-Map 的 RGB-D 输入与地图插入固定为 **2.5 Hz**。这是有意限频：地图不是视频流，强行提升到 10 Hz 会造成 CPU 积压和更大的显示延迟。
- `16UC1` 深度单位为毫米，有效范围默认 0.45–4.5 m。暗处、纯白墙或无纹理区域出现黑色无深度是双目匹配的正常表现。

### 完整链路资源基线（2026-08-03）

测试配置：Orin NX 处于 `MAXN`；启动 `rtabmap_stereo_imu.launch.py use_imu:=true` 和本机 `rtabmap_rviz.launch.py`；1024×750、单遍 OFA、状态监视和实时 `/stereo/points` 显示开启、冗余调试累积点云关闭。负载连续采样 10–15 秒，帧率分别单独采样，避免多个 ROS CLI 订阅者干扰图像传输。以下是不快速移动相机时的稳态基线，大地图回环时会出现短时峰值。

| 指标 | 实测值 | 说明 |
| --- | --- | --- |
| 相机输入 | 9.81 Hz | 外部硬件触发的 ROS 图像发布频率 |
| 实时深度预览 | 9.15 Hz | 完整建图与本机 RViz 同时运行；底层深度仍为 1024×750 |
| 实时几何点云 | 约 9.2 Hz | `/stereo/points`，由当前帧真实深度生成；RGB 字段保存校正后的左目灰度 |
| 视觉里程计 `/odom` | 2.13 Hz | 与 RTAB-Map 的 2.5 Hz RGB-D 输入限频相匹配 |
| RTAB-Map 插图 | 2.5 Hz | 日志中的 `Rate=0.40s`；地图更新不是视频流 |
| 板级输入功耗 `VDD_IN` | 平均 11.95 W，峰值 12.37 W | 包含整块 Jetson 载板与外设，不是单一节点功耗 |
| GPU `GR3D` | 平均 10.6%，峰值 27% | 主要来自 RViz 的实时点云 OpenGL 绘制；无 RViz 时曾实测均值约 2% |
| OFA / VIC | 平均 44% / 6%，峰值 77% / 24% | 深度计算确实运行在专用视觉硬件上 |
| PVA0 | 0% | 当前 VPI 立体路径没有可落到 PVA 的算子；强制搬运到 PVA 反而会增加拷贝 |
| 内存 | 平均约 5.12 GiB / 15.28 GiB | 未使用 Swap |
| 芯片结温 `tj` | 约 61.8 °C，峰值 62.2 °C | 本次测试未发生热降频 |
| CPU 总负载 | 平均 174% | 全部建图、IMU、监视和 RViz 进程合计，约占 8 核总能力的 22% |

15 秒稳态采样的主要进程资源如下（`CPU` 按单核 100% 计）：

| 进程 | 平均 CPU | RSS 内存 |
| --- | ---: | ---: |
| `rtabmap` | 46.8% | 约 419 MiB |
| `stereo_depth_node` | 41.4% | 约 302 MiB |
| `stereo_node` | 32.5% | 约 239 MiB |
| `rgbd_odometry` | 21.8% | 约 268 MiB |
| `rviz2` | 14.3% | 约 230 MiB |
| `yesense_node_publisher` | 8.8% | 约 26 MiB |

恢复实时真实点云显示后，与优化前同机测试相比，GR3D 均值仍由 21.8% 降至 10.6%，CPU 由约 356% 降至 174%，实时深度由 7.25 Hz 提升至 9.15 Hz。关键是按订阅延迟生成颜色/JPEG/点云、先校正单通道打包图再扩展 BGR、将 RViz 刷新率限制为 10 Hz，以及默认关闭重复的 `/luxi/cloud_map_accumulated`。这些修改没有改变深度分辨率、OFA 参数或 RTAB-Map 精度参数。

## 2. 目录

| 路径 | 内容 |
| --- | --- |
| `src/hikrobot_camera_driver` | 海康 MVS 双相机驱动、触发与下采样 |
| `src/stereo_depth` | 校正、VPI/SGBM 深度、点云、RGB-D 打包 |
| `src/hik_bringup` | 启动文件、参数、标定、IMU 与 RViz 配置 |
| `src/third_party` | 上游参考源码；常规 Humble 构建使用 `/opt/ros/humble` 的二进制包 |
| `scripts/bootstrap_humble.sh` | 安装依赖并按 Humble 策略构建 |
| `build`、`install`、`log` | colcon 生成目录；不手动编辑、不提交 |

顶层 README 是唯一项目说明入口。第三方目录中的上游文档不属于本项目维护范围，保留原样以便追溯。

## 3. 安装与构建

### 前提

- Ubuntu 22.04 + ROS 2 Humble；不要混入旧 Lyrical 覆盖层。
- ARM64 海康 MVS 安装在 `/opt/MVS`，并包含 `include/MvCameraControl.h`。
- 两台相机已写入正确的用户集、外部触发和曝光参数。

```bash
cd ~/lunar_slam/device/Hik_MV-CH120-60UC/ros2_ws
test -f /opt/MVS/include/MvCameraControl.h
bash scripts/bootstrap_humble.sh
```

MVS 不在默认路径时：

```bash
MVS_INCLUDE_DIR=/实际/MVS/include bash scripts/bootstrap_humble.sh
```

每个新终端都先加载：

```bash
source /opt/ros/humble/setup.bash
source ~/lunar_slam/device/Hik_MV-CH120-60UC/ros2_ws/install/setup.bash
```

下文用 `HIK_WS` 表示工作区：

```bash
export HIK_WS=~/lunar_slam/device/Hik_MV-CH120-60UC/ros2_ws
```

## 4. 三条常用流程

### A. 验证相机

```bash
ros2 launch hik_bringup camera_only.launch.py
```

成功标志：两台相机均出现 `Opened camera`，且有 `/left_camera/image`、`/right_camera/image`。

### B. 双目深度与本机 RViz

```bash
ros2 launch hik_bringup stereo_camera_bringup.launch.py use_rviz:=true use_imu:=false
```

深度节点会在相机后约 5 秒启动，避免 ARM64 MVS 初始化竞争。检查频率：

```bash
ros2 topic hz /left_camera/image
ros2 topic hz /stereo/depth
```

### C. IMU 融合建图并保存地图

```bash
ros2 launch hik_bringup rtabmap_stereo_imu.launch.py \
  use_imu:=true \
  database_path:=~/.ros/luxi_stereo_rtabmap.db \
  delete_db_on_start:=true \
  clear_db_on_exit:=false
```

另开终端打开 RViz：

```bash
source /opt/ros/humble/setup.bash
source "$HIK_WS/install/setup.bash"
ros2 launch hik_bringup rtabmap_rviz.launch.py
```

`Ctrl+C` 后数据库保存在 `database_path`。默认 `clear_db_on_exit:=true` 仅适合临时测试；要保留地图必须设为 `false`。开始新图会删除同名数据库，重要地图请换路径或先复制。

## 5. 启动入口

同一时刻只能启动一套会打开相机的启动文件。

| 命令 | 用途 |
| --- | --- |
| `camera_only.launch.py` | 仅相机与标定加载检查 |
| `camera_view.launch.py` | 已有相机节点时，仅打开左右图 RViz |
| `visualization.launch.py` | 已有深度节点时，显示 RGB、深度和点云 |
| `stereo_camera_bringup.launch.py` | 相机 + 深度 + 可选 IMU/RViz，日常深度测试 |
| `h30_imu.launch.py` | 单独检查 H30 串口和 IMU 输出 |
| `stereo_imu_calibrated.launch.py` | 相机 + H30 + 相机到 IMU 静态 TF |
| `rtabmap_stereo_imu.launch.py` | 完整建图链路 |
| `rtabmap_rviz.launch.py` | 已有建图链路时打开地图 RViz |

常用参数：

```bash
# 无显示器/远程显示时，只运行计算链路
ros2 launch hik_bringup stereo_camera_bringup.launch.py use_rviz:=false

# 只临时建图；Ctrl+C 后删除临时数据库
ros2 launch hik_bringup rtabmap_stereo_imu.launch.py use_imu:=true

# 打开 RTAB-Map 自带 Qt GUI（额外占用 CPU，不建议与 RViz 同时使用）
ros2 launch hik_bringup rtabmap_stereo_imu.launch.py use_imu:=true use_rtabmap_viz:=true

# 调试时额外开启 LuXi 累积点云（会重复 RTAB-Map 的点云工作）
ros2 launch hik_bringup rtabmap_stereo_imu.launch.py use_imu:=true use_debug_cloud:=true
```

## 6. 可视化、话题与测距

### 本机与远程 RViz

本机直接使用上面的 RViz 启动文件。建图 RViz 固定坐标系为 `map`，默认显示栅格、实时 RGB、深度和 `/stereo/points` 当前帧真实几何点云；重复的 LuXi 累积点云仍默认关闭。

远程主机应只查看 JPEG 预览，不要通过 Wi-Fi 订阅原始图像：

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 topic hz /stereo/preview/left_color/compressed
```

在远程 RViz 添加 `Image`，选择 `/stereo/preview/left_color` 或 `/stereo/preview/depth_visual`，传输选 `compressed`，QoS 选 `Best Effort`。

### 核心话题

| 话题 | 说明 |
| --- | --- |
| `/left_camera/image`、`/right_camera/image` | 1024×750 原始 ROS 图像 |
| `/stereo/depth` | `16UC1` 深度图，毫米 |
| `/stereo/points` | 当前帧实时几何点云，RGB 字段为左目灰度；建图 RViz 默认启用 |
| `/stereo/preview/*/compressed` | 网络友好的 RGB/深度 JPEG 预览 |
| `/stereo/rgbd_image` | 提供给 RGB-D 里程计和 RTAB-Map |
| `/imu/data` | H30 标准 IMU 消息 |
| `/odom` | RGB-D 视觉里程计 |
| `/map`、`/cloud_map`、`/mapPath` | RTAB-Map 2D 地图、全局点云、轨迹 |

测距：先启动深度，再运行：

```bash
ros2 run hik_bringup sgbm_depth_probe.py --ros-args -p duration_sec:=15.0
```

或在 RViz 临时启用 `Debug live stereo point cloud`，另开终端运行：

```bash
ros2 run hik_bringup rviz_depth_click_probe.py
```

选择 **Publish Point** 后点击纹理丰富区域，终端会输出相机光轴深度与直线距离。

## 7. H30 IMU 与标定

H30 固定别名为 `/dev/H30-imu`。首次配置或更换 USB 串口模块后，从源码目录安装规则：

```bash
cd "$HIK_WS"
bash src/hik_bringup/scripts/install_h30_udev_rule.sh
ls -l /dev/H30-imu
```

检查 IMU：

```bash
ros2 launch hik_bringup h30_imu.launch.py
ros2 topic hz /imu/data
ros2 topic echo --once /imu/data
```

当前相机-IMU 外参与原始 Kalibr 结果分别位于：

```text
src/hik_bringup/config/kalibr_cam_imu.yaml
src/hik_bringup/config/kalibr_dynamic_05_camchain.yaml
```

机械结构、相机方向、IMU 方向或分辨率变化后，必须重新双目标定和相机-IMU 联合标定。不要仅修改 `right_rectification_y_offset_px`、基线或 TF 来补偿机械变化。

## 8. 配置修改原则

| 文件 | 修改范围 |
| --- | --- |
| `config/camera_params.yaml` | 采集下采样与相机参数 |
| `config/stereo_proc.yaml` | 日常深度后端、范围、过滤与预览 |
| `config/rtabmap_stereo_imu.yaml` | 建图深度、里程计、栅格、RTAB-Map 参数 |
| `hikrobot_camera_driver/config/stereo_left.yaml`、`stereo_right.yaml` | 双目标定；仅标定后更新 |
| `config/kalibr_cam_imu.yaml` | 相机-IMU 外参记录 |
| `config/h30_imu.yaml` | H30 串口与波特率 |

修改深度算法时优先只改 `depth_backend`：`vpi_ofa_pva_vic` 是默认实时模式，`vpi_cuda` 用于 CUDA-SGM 对比，`sgbm` 仅用于回退诊断。不要将 `processing_scale` 改离 1.0，否则会脱离当前 1024×750 标定基准。

## 9. 常见问题

| 现象 | 处理 |
| --- | --- |
| `camera is already opened by another process` | 已有相机节点在运行。执行 `ros2 node list`，正常结束旧的相机/建图启动终端。 |
| `XOpenDisplay Fail` | 无显示器或 SSH 的 MVS 提示；采集正常时可忽略。无显示器使用 `use_rviz:=false`。 |
| 没有深度 | 等待相机后约 5 秒；确认两个 `camera_info` 与 `Processed frame` 日志。 |
| RViz 卡顿 | 不要把 RTAB-Map `/cloud_map` 当视频流；实时检查用 RGB/深度预览或临时启用一个实时点云显示。 |
| VPI 初始化失败 | 检查 JetPack 6 / VPI 3；可临时切换 `vpi_cuda` 或 `sgbm` 排查。 |
| `/dev/H30-imu` 不存在 | 重装 udev 规则、重新插拔设备，并确认 VID/PID/序列号。 |
| `ros2 topic echo` 报 `!rclpy.ok()` | 执行 `ros2 daemon stop && ros2 daemon start` 后重试。 |

## 10. 重新构建

修改 C++、CMake、launch 或参数后，使用构建脚本保持第三方包忽略策略一致：

```bash
cd "$HIK_WS"
bash scripts/bootstrap_humble.sh
source install/setup.bash
```

纯 YAML 与 RViz 文件在 `--symlink-install` 下通常立即生效；重新启动对应节点即可。若不确定，仍执行上述构建命令。
