# RTAB-Map 源码集成与 D435i 系统适配说明

## 1. 源码位置

RTAB-Map 源码已移动到项目根目录的 `3parts` 目录：

```text
/home/lunar/project/lunar_slam/3parts/
├── rtabmap      # RTAB-Map core，humble-devel，06ffb60
└── rtabmap_ros  # ROS2 wrapper，humble-devel，c25a091
```

`device/D435i/ros2_ws/install` 是 D435i 运行安装空间。使用
`--symlink-install` 构建后，安装空间中的启动文件会通过软链接指向源码。
因此移动源码后必须重建 RTAB-Map ROS 包，否则会出现旧路径不存在的错误。

这种结构的目的：

- 项目根目录 `3parts` 保存第三方源码原貌；
- `device/D435i/ros2_ws/src` 保存 D435i 驱动和本地适配包；
- 本项目的适配逻辑放在独立包中，不污染官方 RTAB-Map 代码。

## 2. 已完成的依赖和构建

### 2.1 源码移动后的修复构建

若启动时报错如下，说明安装空间仍指向已移动前的旧源码目录：

```text
No such file or directory: .../ros2_ws/3parts/rtabmap_ros/
rtabmap_launch/launch/rtabmap.launch.py
```

执行下面命令重新生成 D435i 工作区的 RTAB-Map ROS 安装文件：

```bash
source /opt/ros/humble/setup.bash
source /home/lunar/project/lunar_slam/install/setup.bash
cd /home/lunar/project/lunar_slam/device/D435i/ros2_ws
export MAKEFLAGS="-j4"
colcon build --symlink-install --cmake-clean-cache \
  --base-paths /home/lunar/project/lunar_slam/3parts/rtabmap_ros \
  --packages-select \
    rtabmap_msgs rtabmap_conversions rtabmap_odom rtabmap_slam \
    rtabmap_sync rtabmap_util rtabmap_rviz_plugins rtabmap_viz \
    rtabmap_launch rtabmap_costmap_plugins
```

`/home/lunar/project/lunar_slam/install` 提供已构建的 RTAB-Map core；
本命令只重建其 ROS2 封装和 D435i 运行所需的可执行节点。

已使用 `rosdep` 安装 RTAB-Map ROS2 Humble 所需依赖，包括 g2o、GTSAM、PCL ROS、OctoMap、grid-map、IMU filter、AprilTag/Aruco 消息等。

后续因源码移动、CMake 缓存失效或运行文件缺失而需要重建时，统一使用第 2.1 节命令。

源码构建结果：

- `rtabmap` core 构建成功；
- `rtabmap_conversions` 构建成功；
- `rtabmap_sync` 构建成功；
- `rtabmap_odom` 构建成功；
- `rtabmap_slam` 构建成功；
- `rtabmap_launch` 构建成功；
- `rtabmap_util`、`rtabmap_viz`、`rtabmap_rviz_plugins` 构建成功。

构建日志中出现过以下 warning，但不影响当前 D435i RGB-D SLAM 使用：

- `PCL_ROOT` / `CMP0074`：CMake dev warning；
- `aruco_markers_msgs`、`ros2_aruco_interfaces` 未找到：只影响部分 ArUco/Fiducial 兼容输入；
- `pcap/png disabled`：RTAB-Map 编译时未启用这些可选 IO 能力。

## 3. 新增适配包

新增包：

```text
src/lunar_d435i_rtabmap_bringup/
├── launch/d435i_rtabmap.launch.py
├── rviz/d435i_rtabmap.rviz
├── README.md
├── package.xml
├── setup.py
└── setup.cfg
```

该包的职责：

1. 启动已有的 `lunar_realsense_bringup` D435i 驱动；
2. 发布 `base_link -> camera_link` 零位姿静态 TF，用于无小车实体验证；
3. 启动官方 `rtabmap_launch/rtabmap.launch.py`；
4. 将 D435i 的 RGB、对齐深度和 CameraInfo 接入 RTAB-Map；
5. 提供 RViz 配置，显示 RGB、对齐深度、点云、TF 和 RTAB-Map 栅格地图。

构建命令：

```bash
source /opt/ros/humble/setup.bash
source /home/lunar/project/lunar_slam/device/D435i/ros2_ws/install/setup.bash
cd /home/lunar/project/lunar_slam/device/D435i/ros2_ws
colcon build --symlink-install --packages-select lunar_d435i_rtabmap_bringup
```

## 4. D435i 到 RTAB-Map 的话题适配

适配 launch 默认使用：

```text
RGB            /camera/camera/color/image_raw
Aligned depth  /camera/camera/aligned_depth_to_color/image_raw
CameraInfo     /camera/camera/color/camera_info
PointCloud2    /camera/camera/depth/color/points
TF             /tf, /tf_static
```

RTAB-Map 输出：

```text
/rtabmap/rgbd_image
/rtabmap/odom
/rtabmap/odom_info
/rtabmap/info
/rtabmap/map
/rtabmap/mapData
/rtabmap/mapGraph
/tf
/tf_static
```

TF 关系：

```text
map -> odom -> base_link -> camera_link -> camera_*_optical_frame
```

当前没有底盘实体，因此 `base_link -> camera_link` 默认为零位姿。后续接入真实小车时，需要把以下参数改成相机相对底盘的实测外参：

```bash
camera_x:=...
camera_y:=...
camera_z:=...
camera_roll:=...
camera_pitch:=...
camera_yaw:=...
```

## 5. 可直接运行的 RGB-D 建图命令

当前 ROS2 节点运行在 Jetson 的 Docker 容器中，RViz 通过容器挂载的 X11
套接字显示在 Jetson 桌面。容器中可见的桌面显示为 `:0`，因此先设置：

```bash
export DISPLAY=:0
export XAUTHORITY=/tmp/.docker.xauth
export QT_X11_NO_MITSHM=1
```

在完成上一节的构建后，在同一个容器终端执行以下命令即可同时启动 D435i、
RTAB-Map 和 Jetson 桌面上的 RViz：

```bash
source /opt/ros/humble/setup.bash
source /home/lunar/project/lunar_slam/device/D435i/ros2_ws/install/setup.bash
ros2 launch lunar_d435i_rtabmap_bringup d435i_rtabmap.launch.py \
  rtabmap_args:="--delete_db_on_start"
```

该命令使用 RGB、对齐深度和 RGB CameraInfo：

```text
/camera/camera/color/image_raw
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/color/camera_info
```

RViz 会显示 RGB、深度图、点云、TF 与 RTAB-Map 栅格地图。相机需要缓慢移动并持续观察有纹理的场景，视觉里程计才能持续增加地图节点。

### 5.1 D435i 最小 RGB-D 配置

`lunar_realsense_bringup` 已显式关闭 `enable_infra1` 与
`enable_infra2`。RGB-D 建图只使用彩色、对齐深度、点云和 TF，关闭两路
原始红外流可减少 Jetson USB 链路负载。在 D435i 重新插拔后若看到
`Frames didn't arrived within 5 seconds`，先停止旧的相机节点，再用本节
命令重新启动。

本地烟测应能收到：

```text
/camera/camera/color/image_raw
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/depth/color/points
/tf_static
/rtabmap/rgbd_image
/rtabmap/odom
/rtabmap/map
```

无 RViz / 远程终端运行：

```bash
ros2 launch lunar_d435i_rtabmap_bringup d435i_rtabmap.launch.py \
  rviz:=false \
  rtabmap_viz:=false
```

重新建图并删除旧数据库：

```bash
ros2 launch lunar_d435i_rtabmap_bringup d435i_rtabmap.launch.py \
  rtabmap_args:="--delete_db_on_start"
```

默认数据库：

```text
~/.ros/lunar_d435i_rtabmap.db
```

## 6. 验证结果

已完成短时间烟测：

- D435i 被识别为 Intel RealSense D435I；
- RealSense ROS 版本为 `4.58.2`；
- librealsense 版本为 `2.58.2`;
- 相机当前接在 USB 2.1，因此默认使用 `640x480@15Hz`；
- `rgbd_sync` 已订阅：

```text
/camera/camera/color/image_raw
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/color/camera_info
```

关键 topic 已出现：

```text
/camera/camera/color/image_raw
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/depth/color/points
/rtabmap/rgbd_image
/rtabmap/odom
/rtabmap/info
/rtabmap/map
/tf
/tf_static
```

`/rtabmap/rgbd_image` 频率约为 15Hz，与当前 USB 2.1 下的相机配置一致。

烟测时视觉里程计出现 `Not enough inliers` 和 `no odometry is provided` 日志。这说明相机在测试期间画面特征不足或几乎静止，视觉里程计没有稳定估计位姿；这不是话题或启动链路错误。实际建图时需要手持相机缓慢移动，并保证画面有足够纹理、深度和视差。

## 7. 后续 Nav2 对接要点

RTAB-Map 已发布 Nav2 可用的核心数据：

- 地图：`/rtabmap/map`
- TF：`map -> odom -> base_link`
- 机器人基座：`base_link`

后续接 Nav2 时需要注意：

1. Nav2 global costmap 的 static layer 应订阅 `/rtabmap/map`，或将 RTAB-Map map topic remap 到 `/map`；
2. 无实体小车时只能验证地图、TF、路径规划链路，不能验证真实控制闭环；
3. 接入真实底盘后，必须提供可靠的 `cmd_vel` 执行、底盘里程计或融合里程计，并校准 `base_link -> camera_link` 外参；
4. 仅使用单个 D435i 做视觉里程计时，低纹理、快速转动、强反光、纯白墙面都会导致 odom 丢失，必要时应融合 IMU/轮速计。
