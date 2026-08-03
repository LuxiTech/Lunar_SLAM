# 海康 MV-CH120-60UC 双目与 RTAB-Map 工作区

面向 **Jetson Orin NX / Ubuntu 22.04 / ROS 2 Humble / ARM64** 的海康双目相机工作区。它提供硬件同步的双目采集、CUDA-SGM 深度、H30 IMU 数据、RViz 可视化和 RTAB-Map 建图。

标定基准分辨率为 **1024×750、10 Hz**。实时建图模式下，相机以 2048×1500 采集并完整视场缩放到 **1024×750**；驱动会同步缩放 `CameraInfo`，GPU 深度匹配使用 **512×375**，比此前 384×281 多 78% 的深度像素。

## 目录与能力

| 路径 | 职责 |
| --- | --- |
| `src/hikrobot_camera_driver` | 海康 MVS 双相机驱动、硬件触发与下采样 |
| `src/stereo_depth` | 校正、VPI CUDA-SGM / SGBM、深度、点云、RGBD 打包 |
| `src/hik_bringup` | 参数、标定、IMU、RViz、RTAB-Map 启动入口 |
| `scripts/bootstrap_humble.sh` | 安装 Humble 依赖并构建本工作区 |
| `build/`、`install/`、`log/` | colcon 生成目录，不提交、不手动编辑 |

> 仅使用 Humble 的 `install/` 覆盖层。不要混入旧的 Lyrical 环境，也不要恢复 `build_humble/`、`install_humble/` 或 `log_humble/`。

## 算法与数据流

```text
两台 MV-CH120-60UC（外部硬件触发）
                 │  /left_camera/*、/right_camera/*
                 ▼
           stereo_node
  Bayer 转 BGR + 2048×1500 → 1024×750 + 缩放后的 CameraInfo
                 │
                 ▼
        stereo_depth_node
  去畸变 / 极线校正 → 右图 2 px 补偿
  → VPI CUDA-SGM（默认）→ 深度置信度与边缘过滤
  → /stereo/depth、/stereo/points、/stereo/rgbd_image
                 │                         │
                 │                         ├── rgbd_odometry
                 │                         │   视觉 F2M 里程计 → /odom
                 │                         ▼
                 └── RViz              RTAB-Map
                                         位姿图、回环、2D 占据栅格
                                         → /map、/cloud_map、/mapPath
```

### 深度设计

- 默认后端是 NVIDIA VPI CUDA-SGM，充分利用 Orin NX GPU；VPI 初始化或运行失败时可自动回退到保留的 OpenCV SGBM。匹配在 512×375 进行，深度内参同步缩放，因此不需要为此实时档重新标定。
- 当前机械结构的标定使有效视差方向为右减左，因此匹配顺序配置为 `right_left`；节点会把结果重新映射回左相机坐标系。
- 右图在校正后下移 2 px，以消除当前装配的垂直残差。**机械结构变化后必须重新双目标定，不要盲目修改该数值。**
- 深度图编码为 `32FC1`，单位为米；默认有效范围为 0.45–4.5 m。

### 建图设计

- `rgbd_odometry` 使用视觉 RGB-D F2M 里程计，启用卡尔曼平滑、最近帧策略和 20 内点下限，降低静止漂移及过期帧延迟。
- RTAB-Map 接收 `/stereo/rgbd_image` 与 `/odom`，以约 2.5 Hz 处理新鲜 RGB-D 帧。默认输出 **2D 占据栅格**（`/map`）；同时可生成全局彩色点云 `/cloud_map`。RViz 的实时点云应使用独立的 `/luxi/cloud_map_accumulated`，其目标频率为 10 Hz。
- H30 AHRS 输出 `/imu/data`、`/imu/data_extend` 和 `left_camera_optical_frame → imu_link` 静态外参。建图配置会以 AHRS 姿态初始化 RGB-D 视觉里程计，并把同一 IMU 流交给 RTAB-Map 保留重力方向。

## 1. 首次安装与构建

### 前提

- Ubuntu 22.04（Jammy）与 ROS 2 Humble。
- ARM64 版海康 MVS 开发包。默认位置 `/opt/MVS`，必须包含头文件：

  ```bash
  test -f /opt/MVS/include/MvCameraControl.h
  ```

- 两台相机已在 MVS 中配置正确的用户集、外部触发和曝光。

### 一键安装

```bash
cd ~/lunar_slam/device/Hik_MV-CH120-60UC/ros2_ws
bash scripts/bootstrap_humble.sh
```

脚本会安装 ROS 图像、RTAB-Map、RViz 等依赖，并只构建本项目需要的包。仓库中的 `src/third_party` 不参与 Humble 构建。

若 MVS 头文件不在 `/opt/MVS/include`：

```bash
MVS_INCLUDE_DIR=/实际/MVS/include bash scripts/bootstrap_humble.sh
```

### 每个新终端都要加载环境

```bash
source /opt/ros/humble/setup.bash
source ~/lunar_slam/device/Hik_MV-CH120-60UC/ros2_ws/install/setup.bash
```

下文假设工作区已加载。为简洁起见，先定义：

```bash
export HIK_WS=~/lunar_slam/device/Hik_MV-CH120-60UC/ros2_ws
export HIK_SHARE="$(ros2 pkg prefix hik_bringup)/share/hik_bringup"
```

## 2. 快速开始

### 只验证相机

```bash
ros2 launch hik_bringup camera_only.launch.py
```

成功标志：日志显示两台相机 `Opened camera`，并能看到以下话题：

```bash
ros2 topic list | rg 'left_camera|right_camera'
```

### 启动双目深度与本机 RViz

```bash
ros2 launch hik_bringup stereo_camera_bringup.launch.py use_rviz:=true use_imu:=false
```

相机启动后，深度节点会延迟约 5 秒启动；这是为了避免 ARM64 MVS 在相机打开阶段与深度节点并发初始化。

### 正式建图并保存数据库

```bash
ros2 launch hik_bringup rtabmap_stereo_imu.launch.py \
  use_imu:=true \
  database_path:=~/.ros/luxi_stereo_rtabmap.db \
  delete_db_on_start:=true \
  clear_db_on_exit:=false
```

另开终端打开建图 RViz：

```bash
source /opt/ros/humble/setup.bash
source "$HIK_WS/install/setup.bash"
ros2 launch hik_bringup rtabmap_rviz.launch.py
```

按 `Ctrl+C` 结束建图后，数据库保存在 `~/.ros/luxi_stereo_rtabmap.db`。开始新图时使用 `delete_db_on_start:=true`；不希望覆盖已有地图时，请先复制或改用新的 `database_path`。

## 3. 启动入口参考

同一时刻只能有一套启动文件打开相机。若看到 `camera is already opened by another process`，先结束已有的 `stereo_node` / 建图启动终端。

| 命令 | 启动节点或用途 | 适用场景 |
| --- | --- | --- |
| `ros2 launch hik_bringup camera_only.launch.py` | `stereo_node` | 检查相机、触发与标定文件是否加载 |
| `ros2 launch hik_bringup camera_view.launch.py` | RViz，`camera_view.rviz` | 已有相机节点时，仅看左右相机图像 |
| `ros2 launch hik_bringup visualization.launch.py` | RViz，`stereo_view.rviz` | 已有深度节点时，查看 RGB、彩色深度和点云 |
| `ros2 launch hik_bringup stereo_camera_bringup.launch.py` | 相机 + 深度 + 可选 IMU/RViz | 日常双目深度测试 |
| `ros2 launch hik_bringup h30_imu.launch.py` | `yesense_pub` | 单独检查 H30 串口和 IMU 数据 |
| `ros2 launch hik_bringup stereo_imu_calibrated.launch.py use_imu:=true` | 相机 + H30 + 相机到 IMU 静态 TF | 检查相机/IMU 外参与数据链路 |
| `ros2 launch hik_bringup rtabmap_stereo_imu.launch.py` | 相机、深度、视觉里程计、RTAB-Map | 建图或定位开发 |
| `ros2 launch hik_bringup rtabmap_rviz.launch.py` | 建图 RViz | 已有 RTAB-Map 节点时显示地图 |

### `stereo_camera_bringup.launch.py` 参数

```bash
# 不打开本机 RViz，适合远程 RViz 或无显示器运行
ros2 launch hik_bringup stereo_camera_bringup.launch.py use_rviz:=false

# 同时启动 H30 驱动
ros2 launch hik_bringup stereo_camera_bringup.launch.py use_imu:=true

# 使用自定义相机或深度参数文件
ros2 launch hik_bringup stereo_camera_bringup.launch.py \
  camera_params:="$HIK_SHARE/config/camera_params.yaml" \
  stereo_proc_params:="$HIK_SHARE/config/stereo_proc.yaml"
```

### `rtabmap_stereo_imu.launch.py` 参数

```bash
# 临时测试：Ctrl+C 后自动删除测试数据库（默认行为）
ros2 launch hik_bringup rtabmap_stereo_imu.launch.py

# 启动 H30 AHRS 融合建图（正常使用此命令）
ros2 launch hik_bringup rtabmap_stereo_imu.launch.py use_imu:=true

# 启动 RTAB-Map 自带 Qt GUI（会额外占用一个 CPU 核）
ros2 launch hik_bringup rtabmap_stereo_imu.launch.py use_rtabmap_viz:=true

# 默认发布低延迟累积点云；RViz 请订阅 /luxi/cloud_map_accumulated
ros2 launch hik_bringup rtabmap_stereo_imu.launch.py use_debug_cloud:=true

# 保留数据库，且使用指定路径
ros2 launch hik_bringup rtabmap_stereo_imu.launch.py \
  database_path:=~/.ros/room_01.db \
  delete_db_on_start:=true \
  clear_db_on_exit:=false
```

该启动文件会隔离 MVS 自带的旧 `libusb`，避免它与 PCL/RTAB-Map 冲突。因此不要在另一个终端直接手动执行 `rtabmap_viz`；应使用上面的 `use_rtabmap_viz:=true` 或 `rtabmap_rviz.launch.py`。

## 4. H30 IMU 使用

H30 的 USB 串口固定为 `/dev/H30-imu`。首次插入设备后，从**源码路径**安装 udev 规则：

```bash
cd "$HIK_WS"
bash src/hik_bringup/scripts/install_h30_udev_rule.sh
ls -l /dev/H30-imu
```

若未生成别名，重新插拔 H30 后再次检查。该规则匹配当前设备的 VID、PID 与序列号；更换 USB 转串口模块后需要更新 `config/99-h30-imu.rules`。

启动与检查：

```bash
ros2 launch hik_bringup h30_imu.launch.py
ros2 topic hz /imu/data
ros2 topic echo --once /imu/data
```

### 融合状态与启用条件

当前 RTAB-Map 已融合 H30 AHRS：`rgbd_odometry.wait_imu_to_init` 为 `true`，
`rtabmap.subscribe_imu` 为 `true`，两个节点都显式订阅 `/imu/data`。

2026-08-03 已完成设备端验证：H30 以 921600 baud 稳定发布 **200 Hz**，原始帧含
采样时间戳 `0x51`、欧拉角 `0x40` 与四元数 `0x41`；四元数范数约为 1，
`base_link → imu_link` 联合标定 TF 可查询。实际启动时，视觉里程计已用 IMU 姿态初始化，
RTAB-Map 已成功累积节点和点云。

每次修改 H30 输出配置后，先运行下面的检查：

```bash
ros2 launch hik_bringup h30_imu.launch.py
ros2 topic hz /imu/data
ros2 topic echo --once /imu/data
```

只有同时满足以下条件才能保持融合模式：

1. `/imu/data` 持续发布，频率建议不低于 100 Hz；
2. `orientation` 四元数非全零、范数约为 1，角速度与加速度均为有限值；
3. 时间戳单调递增，且不存在明显晚于 RGB-D 帧的延迟；
4. `imu_link` 与 `base_link` 的 TF 可查询，机械结构未在联合标定后改变。

满足后再将 `rgbd_odometry.subscribe_imu` 设为 `true`，启用 `wait_imu_to_init`，并在实际移动建图中核对
里程计没有 IMU 等待、丢 RGB-D 帧或姿态跳变。不能只因联合标定完成就跳过该验证。

相机到 IMU 外参及时间偏移记录于：

```text
src/hik_bringup/config/kalibr_cam_imu.yaml
src/hik_bringup/config/kalibr_dynamic_05_camchain.yaml
```

如果机械结构、相机或 IMU 方向改变，先重新完成双目标定和相机-IMU 联合标定，再更新这两个文件与 `stereo_imu_calibrated.launch.py` 中的静态 TF。

## 5. 可视化与常用话题

### 核心话题

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/left_camera/image`、`/right_camera/image` | `sensor_msgs/Image` | 1024×750 彩色输入；CameraInfo 由同分辨率标定加载 |
| `/left_camera/camera_info`、`/right_camera/camera_info` | `sensor_msgs/CameraInfo` | 原始输入标定 |
| `/stereo/left/image_rect_color` | `sensor_msgs/Image` | 左目校正彩色图 |
| `/stereo/depth` | `sensor_msgs/Image` | `32FC1` 深度图，单位米 |
| `/stereo/disparity` | `sensor_msgs/Image` | 归一化视差预览 |
| `/stereo/points` | `sensor_msgs/PointCloud2` | 512×375 VPI-SGM 生成的实时彩色点云 |
| `/luxi/cloud_map_accumulated` | `sensor_msgs/PointCloud2` | 低延迟、固定在 `odom` 的 RViz 实时累积点云（约 10 Hz） |
| `/stereo/rgbd_image` | `rtabmap_msgs/RGBDImage` | 提供给视觉里程计与 RTAB-Map |
| `/stereo/preview/left_color/compressed` | `sensor_msgs/CompressedImage` | 10 Hz 左图 JPEG 预览 |
| `/stereo/preview/right_color/compressed` | `sensor_msgs/CompressedImage` | 10 Hz 右图 JPEG 预览 |
| `/stereo/preview/depth_visual/compressed` | `sensor_msgs/CompressedImage` | 彩色深度预览，约 2.5–3 Hz |
| `/imu/data` | `sensor_msgs/Imu` | H30 标准 IMU 数据 |
| `/odom` | `nav_msgs/Odometry` | RGB-D 视觉里程计 |
| `/map`、`/cloud_map`、`/mapPath` | RTAB-Map 输出 | 2D 栅格、全局点云、轨迹 |

### 本机 RViz

双目深度启动时传入 `use_rviz:=true`，或单独执行：

```bash
ros2 launch hik_bringup visualization.launch.py
```

建图时另开终端执行：

```bash
ros2 launch hik_bringup rtabmap_rviz.launch.py
```

建图 RViz 固定坐标系为 `map`，默认关闭旧时间戳的轨迹与里程计装饰显示，以避免 RViz 消息过滤队列积压。栅格地图仍正常显示。

### 宿主机远程 RViz

设备端和宿主机需在同一网络与相同 ROS 域：

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

设备端仅启动计算，不启动本机 RViz：

```bash
ros2 launch hik_bringup stereo_camera_bringup.launch.py use_rviz:=false use_imu:=false
```

宿主机只订阅 JPEG 预览，避免通过网络传输原始双目图像：

```bash
ros2 topic hz /stereo/preview/left_color/compressed
```

在宿主机 RViz 新建 `Image` 显示项时，选择 `/stereo/preview/left_color`、`/stereo/preview/right_color` 或 `/stereo/preview/depth_visual`，传输方式选 `compressed`，QoS 选 `Best Effort`。不要用 Wi-Fi 直接显示 `/left_camera/image`、`/right_camera/image`。

## 6. 深度质量与测距

### 深度统计

先运行双目深度，再另开终端：

```bash
ros2 run hik_bringup sgbm_depth_probe.py --ros-args -p duration_sec:=15.0
```

脚本输出分辨率、有效像素比例、深度中位数及发布频率。脚本名称为历史名称；它测量的是当前 `/stereo/depth`，无论后端是 VPI CUDA-SGM 还是 SGBM 回退。

### 使用 RViz 点击点云测距

1. 启动双目深度和 RViz。
2. 在 RViz 的 `Debug live stereo point cloud` 显示项中勾选启用。
3. 新终端运行：

   ```bash
   ros2 run hik_bringup rviz_depth_click_probe.py
   ```

4. 在 RViz 工具栏选择 **Publish Point**，点击点云上的纹理丰富位置。
5. 终端输出 `Z_depth`（相机光轴深度）及 `range`（相机到目标直线距离）。

## 7. 建图操作流程

1. 用“正式建图并保存数据库”命令启动建图。
2. 等待 `rtabmap_status` 输出 `wm_nodes=1`。静止时节点数保持不变是正常现象。
3. 缓慢平移相机 0.5–1 m，再缓慢转动约 30°；`wm_nodes` 应随真实运动增长。
4. 在 RViz 中检查 `/map` 栅格和相机覆盖区域。快速甩动、纯白墙面或无纹理画面会使视觉里程计变差。
5. `Ctrl+C` 正常停止。只有设置 `clear_db_on_exit:=false` 时，数据库才会保留。

检查状态：

```bash
ros2 node list
ros2 topic echo --once /info --qos-reliability best_effort
ros2 topic info /stereo/rgbd_image -v
```

## 8. 参数位置与安全修改原则

| 文件 | 修改内容 |
| --- | --- |
| `config/camera_params.yaml` | 采集下采样比例、RViz 预览比例 |
| `config/stereo_proc.yaml` | 日常深度后端、范围与过滤参数 |
| `config/rtabmap_stereo_imu.yaml` | 建图专用深度、里程计、栅格和 RTAB-Map 参数 |
| `config/stereo_left.yaml`、`stereo_right.yaml` | 双目标定内参和外参 |
| `config/kalibr_cam_imu.yaml` | 相机-IMU 外参与时间偏移记录 |
| `config/h30_imu.yaml` | H30 串口、波特率与话题 |

修改标定、`matcher_input_order`、`right_rectification_y_offset_px`、基线或相机分辨率前，先备份配置并重新验证深度。不要用未匹配分辨率的标定文件运行。

## 9. 排障

| 现象 | 检查与处理 |
| --- | --- |
| `camera is already opened by another process` | 已有相机节点占用设备。执行 `ros2 node list`，结束旧的 `camera_only`、深度或 RTAB-Map 启动终端后再试。 |
| `XOpenDisplay Fail` | 无显示器或远程 shell 的常见 MVS 提示。采集正常时可忽略；无显示器运行时使用 `use_rviz:=false`。 |
| 没有深度图 | 确认两个 `camera_info` 已发布、深度节点已在相机启动约 5 秒后出现，并查看 `Processed frame` 日志。 |
| VPI 初始化失败 | 检查 JetPack/VPI/CUDA 环境；节点会在允许时回退 SGBM。可临时在参数文件设置 `depth_backend: sgbm` 进行对比。 |
| RViz 延迟或卡顿 | 实时查看应显示 `/luxi/cloud_map_accumulated`，不要把 RTAB-Map 的全局 `/cloud_map` 当作视频流。图像预览按订阅者惰性生成；未查看图像时不会占用 JPEG 编码资源。 |
| RViz 出现 `queue is full` | 使用 `rtabmap_rviz.launch.py` 提供的配置，不要把固定坐标系改成 `odom`，不要默认启用 `mapPath` / Visual odometry。 |
| `libusb_set_option` 或 PCL 符号错误 | MVS 的旧版 `libusb` 与 PCL 冲突。通过 `rtabmap_stereo_imu.launch.py` 启动，不要手动从带 MVS `LD_LIBRARY_PATH` 的终端运行 `rtabmap_viz`。 |
| `/dev/H30-imu` 不存在 | 运行 udev 安装脚本，重新插拔 H30，并确认规则中的序列号与当前设备一致。 |
| `ros2 topic echo` 报 `!rclpy.ok()` | ROS 2 守护进程失效：执行 `ros2 daemon stop && ros2 daemon start`，再重试话题命令。 |
| H30 驱动已打开但 `/imu/data` 没有消息或姿态不可用 | 先停止驱动，直接读取串口确认是否有字节流；若无字节流，物理重插 H30 后复测。确认 921600 baud，并在 H30 中启用数据 ID `0x51` 和四元数 `0x41`；`orientation_covariance[0]` 必须不为 `-1`。 |

## 10. 重新构建

修改 C++ 源码、CMake 或 Python 安装入口后：

```bash
cd "$HIK_WS"
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select hikrobot_camera_driver stereo_depth hik_bringup
source install/setup.bash
```

纯 YAML、RViz 和 launch 文件在 `--symlink-install` 构建后通常会直接反映；若不确定，执行上面的构建命令。完整依赖重建使用：

```bash
bash scripts/bootstrap_humble.sh
```
