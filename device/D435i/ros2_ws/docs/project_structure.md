# D435i ROS2 工作区结构说明

本文档说明 `/home/lunar/project/lunar_slam/device/D435i` 下 RealSense D435i 驱动适配项目的目录结构、包职责、构建产物和常用运行入口。

## 顶层目录

```text
/home/lunar/project/lunar_slam/device/D435i
└── ros2_ws
    ├── src/        # ROS2 源码包目录，主要维护对象
    ├── build/      # colcon 构建中间产物，自动生成
    ├── install/    # colcon 安装空间，source 后供 ROS2 发现包
    ├── log/        # colcon 构建/测试日志，自动生成
    └── docs/       # 项目说明文档
```

维护代码时优先关注 `ros2_ws/src` 和 `ros2_ws/docs`。`build`、`install`、`log` 属于构建产物，通常不手工修改。

## ROS2 工作区

工作区根目录：

```bash
/home/lunar/project/lunar_slam/device/D435i/ros2_ws
```

构建命令：

```bash
cd /home/lunar/project/lunar_slam/device/D435i/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

当前工作区包含 4 个主要 ROS2 包：

```text
src/
├── lunar_realsense_bringup/       # 本项目 D435i 启动、RViz、验证适配包
└── realsense-ros/                 # Intel RealSense 官方 ROS2 wrapper 源码
    ├── realsense2_camera/         # 官方相机驱动节点
    ├── realsense2_camera_msgs/    # 官方消息、服务、action 定义
    ├── realsense2_description/    # 官方 URDF、mesh、相机描述
    ├── realsense2_rgbd_plugin/    # 官方 RViz RGBD 插件源码，当前未作为核心包构建
    └── realsense2_ros_mqtt_bridge/# 官方 MQTT bridge，当前未作为核心包构建
```

## 本地适配包：lunar_realsense_bringup

路径：

```text
ros2_ws/src/lunar_realsense_bringup
```

职责：

- 封装 D435i 的默认启动参数。
- 默认启用 RGB、Depth、PointCloud、TF。
- 提供 RViz 配置，直接显示 RGB、深度图、点云和 TF。
- 提供 headless smoke test，用于检查关键 ROS2 话题是否正常发布。
- 处理 arm64/NEON 环境下 RealSense 点云滤波器参数名差异：`pointcloud__neon_`。

目录结构：

```text
lunar_realsense_bringup/
├── package.xml
├── setup.py
├── setup.cfg
├── README.md
├── launch/
│   └── d435i.launch.py
├── rviz/
│   └── d435i.rviz
├── lunar_realsense_bringup/
│   ├── __init__.py
│   └── check_realsense_topics.py
└── resource/
    └── lunar_realsense_bringup
```

关键文件说明：

- `launch/d435i.launch.py`
  - 主启动文件。
  - 启动 `realsense2_camera_node`。
  - 当前 D435i 已确认枚举为 USB 3.2，默认使用 `640x480@30fps`。
  - `rviz:=true` 时同时启动 RViz。

- `rviz/d435i.rviz`
  - RViz 显示配置。
  - 已配置：
    - RGB image
    - Depth image
    - PointCloud2
    - TF

- `lunar_realsense_bringup/check_realsense_topics.py`
  - 自动检查以下关键话题是否有消息：
    - `/camera/camera/color/image_raw`
    - `/camera/camera/depth/image_rect_raw`
    - `/camera/camera/depth/color/points`
    - `/tf_static`

## 官方驱动源码：realsense-ros

路径：

```text
ros2_ws/src/realsense-ros
```

来源：

- 官方仓库：`IntelRealSense/realsense-ros`
- 当前版本：`4.58.2`
- 对应底层库：`ros-humble-librealsense2 2.58.2`

核心包：

- `realsense2_camera`
  - D435i ROS2 驱动节点。
  - 提供图像、深度、点云、TF、metadata 等 ROS2 输出。

- `realsense2_camera_msgs`
  - RealSense 自定义消息、服务和 action。
  - 例如 `Metadata.msg`、`Extrinsics.msg`、`DeviceInfo.srv`。

- `realsense2_description`
  - 相机模型描述。
  - 包含 D435i URDF xacro、mesh、模型 RViz 配置。

非核心包：

- `realsense2_rgbd_plugin`
  - 官方 RViz RGBD 插件源码。
  - 当前项目已使用 RViz 标准 Image、PointCloud2、TF 插件满足显示需求。

- `realsense2_ros_mqtt_bridge`
  - 官方 MQTT bridge。
  - 当前 D435i 本地驱动适配不依赖该包。

## 构建产物目录

### build/

路径：

```text
ros2_ws/build
```

作用：

- 保存 colcon/CMake/setuptools 构建中间文件。
- 包括 CMake cache、编译出的临时目标、Python egg-info 等。
- 不应作为源码维护目录。

### install/

路径：

```text
ros2_ws/install
```

作用：

- colcon 安装空间。
- 执行 `source install/setup.bash` 后，ROS2 可以发现本工作区内的包。
- 当前已安装包包括：
  - `lunar_realsense_bringup`
  - `realsense2_camera`
  - `realsense2_camera_msgs`
  - `realsense2_description`

### log/

路径：

```text
ros2_ws/log
```

作用：

- 保存 colcon build/test/list 的日志。
- 排查构建失败时优先查看这里。

## 常用运行方式

### 启动 D435i 和 RViz

```bash
source /opt/ros/humble/setup.bash
source /home/lunar/project/lunar_slam/device/D435i/ros2_ws/install/setup.bash
ros2 launch lunar_realsense_bringup d435i.launch.py
```

### 只启动 D435i，不启动 RViz

```bash
source /opt/ros/humble/setup.bash
source /home/lunar/project/lunar_slam/device/D435i/ros2_ws/install/setup.bash
ros2 launch lunar_realsense_bringup d435i.launch.py rviz:=false
```

### 检查 RGB、深度、点云、TF 是否正常

在相机驱动运行时执行：

```bash
source /opt/ros/humble/setup.bash
source /home/lunar/project/lunar_slam/device/D435i/ros2_ws/install/setup.bash
ros2 run lunar_realsense_bringup check_realsense_topics
```

通过时会显示 RGB、Depth、PointCloud、TF 均收到消息。

### 查看点云频率

```bash
ros2 topic hz /camera/camera/depth/color/points
```

当前默认配置预期约为 `30Hz`。

## 主要输出话题

默认 namespace 和 node name 均为 `camera`，因此话题前缀为 `/camera/camera`。

| 功能 | 话题 | 类型 |
| --- | --- | --- |
| RGB 图像 | `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` |
| RGB 相机内参 | `/camera/camera/color/camera_info` | `sensor_msgs/msg/CameraInfo` |
| 深度图 | `/camera/camera/depth/image_rect_raw` | `sensor_msgs/msg/Image` |
| 深度相机内参 | `/camera/camera/depth/camera_info` | `sensor_msgs/msg/CameraInfo` |
| 彩色对齐深度图 | `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/msg/Image` |
| 点云 | `/camera/camera/depth/color/points` | `sensor_msgs/msg/PointCloud2` |
| 动态 TF | `/tf` | `tf2_msgs/msg/TFMessage` |
| 静态 TF | `/tf_static` | `tf2_msgs/msg/TFMessage` |

## 当前默认参数

`d435i.launch.py` 当前默认配置：

```text
color_profile = 640,480,30
depth_profile = 640,480,30
align_depth.enable = true
pointcloud = enabled
publish_tf = true
tf_publish_rate = 30.0
```

当前设备已经通过 `lsusb -t` 和 RealSense 启动日志确认工作在 USB `3.2`。如果临时
接到 USB2 端口，应显式降回 15 Hz：

```bash
ros2 launch lunar_realsense_bringup d435i.launch.py \
  color_profile:=640,480,15 \
  depth_profile:=640,480,15
```

## 注意事项

- `build/`、`install/`、`log/` 是自动生成目录，必要时可以删除后重新 `colcon build`。
- 本项目主要维护入口是 `src/lunar_realsense_bringup`。
- 官方 `realsense-ros` 源码建议保持与当前 tag 对齐，除非明确需要修改驱动内部逻辑。
- arm64 平台下，RealSense 点云滤波器参数实际使用 `pointcloud__neon_` 前缀，已在本地 launch 文件中处理。

## 移动工作区后的修复方式

本工作区使用 `colcon build --symlink-install` 构建。该模式会在 `install/` 中生成指向 `src/` 和 `build/` 的符号链接，部分环境文件也会记录构建时的绝对路径。

如果在构建后移动了工作区目录，例如从：

```text
/home/lunar/project/lunar_slam/device/ros2_ws
```

移动到：

```text
/home/lunar/project/lunar_slam/device/D435i/ros2_ws
```

旧的 `build/`、`install/`、`log/` 会继续引用原路径，常见现象是：

```text
not found: ".../install/realsense2_camera/share/realsense2_camera/local_setup.bash"
Package 'lunar_realsense_bringup' not found
```

修复方式是在新路径下清理构建产物并重新构建：

```bash
cd /home/lunar/project/lunar_slam/device/D435i/ros2_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

验证包索引：

```bash
ros2 pkg prefix lunar_realsense_bringup
ros2 pkg prefix realsense2_camera
```

验证订阅输出：

```bash
ros2 launch lunar_realsense_bringup d435i.launch.py rviz:=false
ros2 run lunar_realsense_bringup check_realsense_topics
```
