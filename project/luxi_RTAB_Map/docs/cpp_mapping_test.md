# luxi_rtab_map C++ 建图测试

本包验证直接调用 RTAB-Map C++ core API，而不是只调用 `rtabmap_launch`。

## 运行前提

`luxi_rtab_map_node` 不会自动启动 D435i 相机驱动。它只订阅相机已经发布出来的 RGB、对齐深度和 CameraInfo topic，然后调用 RTAB-Map C++ API 处理数据。

因此必须先启动相机驱动，否则节点会一直等待输入，看起来像“卡住”。

先确认工作区已经编译：

```bash
source /opt/ros/humble/setup.bash
cd /home/lunar/project/lunar_slam
colcon build --symlink-install --packages-select luxi_rtab_map --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## RGB-D 建图测试

### 1. 启动 D435i 相机

打开第一个终端：

```bash
source /opt/ros/humble/setup.bash
source /home/lunar/project/lunar_slam/device/D435i/ros2_ws/install/setup.bash
ros2 launch lunar_realsense_bringup d435i.launch.py rviz:=false
```

确认以下 topic 存在：

```bash
ros2 topic list | grep camera/camera
```

至少应看到：

```text
/camera/camera/color/image_raw
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/color/camera_info
```

### 2. 启动 C++ RGB-D 建图节点

打开第二个终端：

```bash
source /opt/ros/humble/setup.bash
source /home/lunar/project/lunar_slam/install/setup.bash
ros2 launch luxi_rtab_map rgbd_mapping_test.launch.py mode:=rgbd
```

注意：这个 launch 不会启动 RViz。它只启动 `luxi_rtab_map_node`，用于验证“直接调用 RTAB-Map C++ API 能否接收 RGB-D 并生成数据库”。

如果只是做快速验证，让节点处理 30 帧后自动退出：

```bash
ros2 launch luxi_rtab_map rgbd_mapping_test.launch.py mode:=rgbd max_frames:=30
```

默认输入：

```text
/camera/camera/color/image_raw
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/color/camera_info
```

默认数据库：

```text
/tmp/luxi_rtab_map_test.db
```

重新测试并删除旧数据库：

```bash
ros2 launch luxi_rtab_map rgbd_mapping_test.launch.py \
  mode:=rgbd \
  delete_db_on_start:=true \
  database_path:=/tmp/luxi_rtab_map_test.db
```

### 3. 判断是否正常运行

正常运行时会周期性打印状态：

```text
Status: mode=rgbd received=... mapped=... lost=... database=/tmp/luxi_rtab_map_test.db
```

字段含义：

- `received`：收到并同步成功的输入帧数；
- `mapped`：成功加入 RTAB-Map 地图的帧数；
- `lost`：视觉里程计丢失次数。

如果看到：

```text
No synchronized input received yet
```

说明节点还没有收到相机数据，应检查 D435i 驱动是否启动、topic 名称是否匹配。

如果看到类似：

```text
Status: mode=rgbd received=731 mapped=1 lost=730 database=/tmp/luxi_rtab_map_test.db
Odometry is lost in rgbd mode. received=759 mapped=1 lost=758
```

说明节点不是卡住，而是已经收到 RGB-D 数据，但视觉里程计失败：

- `received=731`：已经收到 731 帧同步后的 RGB-D 数据；
- `mapped=1`：只有 1 帧成功加入 RTAB-Map 地图；
- `lost=730`：后续 730 帧 odometry 丢失，RTAB-Map 无法继续扩展地图。

常见原因：

- 相机静止不动或运动幅度太小；
- 画面纹理太少，例如白墙、地面、暗光环境；
- 手持移动太快，导致图像模糊；
- 当前 D435i 接在 USB 2.1，带宽较低，只适合低帧率测试；
- 本节点是最小 C++ API 验证程序，没有官方 `rtabmap_ros` 的完整同步、TF、诊断和 RViz 可视化链路。

测试时应手持相机缓慢移动，对准有纹理和空间结构的区域，例如桌面、墙角、物体边缘，不要只拍纯白墙或纯地面。

### 4. 查看建图效果

本节点主要用于验证 C++ API 和生成 RTAB-Map 数据库，不负责完整 RViz 可视化。因此运行下面命令不会弹出 RViz 窗口：

```bash
ros2 launch luxi_rtab_map rgbd_mapping_test.launch.py mode:=rgbd
```

如果要在 RViz 中直接看 RGB、深度、点云、TF 和 `/rtabmap/map`，推荐运行完整 bringup：

```bash
source /opt/ros/humble/setup.bash
source /home/lunar/project/lunar_slam/device/D435i/ros2_ws/install/setup.bash
ros2 launch lunar_d435i_rtabmap_bringup d435i_rtabmap.launch.py
```

两种入口的用途不同：

```text
luxi_rtab_map
  目标：验证 C++ API、生成 RTAB-Map 数据库
  特点：不自动启动 RViz，不发布完整可视化链路

lunar_d435i_rtabmap_bringup
  目标：实际查看 RGB-D 建图效果
  特点：启动 D435i、RTAB-Map ROS 节点、RViz、TF、地图可视化
```

如果只检查 C++ 节点生成的数据库，可查看：

```bash
ls -lh /tmp/luxi_rtab_map_test.db
```

## 只有深度的可行性测试

同样需要先启动 D435i 相机驱动，然后在第二个终端运行：

```bash
source /opt/ros/humble/setup.bash
source /home/lunar/project/lunar_slam/install/setup.bash
ros2 launch luxi_rtab_map rgbd_mapping_test.launch.py mode:=depth
```

快速验证：

```bash
ros2 launch luxi_rtab_map rgbd_mapping_test.launch.py mode:=depth max_frames:=30
```

实测结论：只有深度图可以被转换为 RTAB-Map `SensorData` 并进入 C++ API 流程，但在当前 D435i 输入下 odometry 丢失，不能形成可靠地图。

本次 smoke test 输出：

```text
mode=depth
received=1
mapped=0
lost=1
```

原因是 RTAB-Map 的 RGB-D/视觉建图需要可用于特征匹配的图像或可靠外部里程计。单独深度图缺少 RGB/灰度纹理特征；直接构造伪灰度图只能验证接口可运行，不能提供稳定位姿估计。

稳定 depth-only 建图通常需要：

- 深度图转 `PointCloud2`；
- 使用 RTAB-Map ICP odometry；
- 或提供外部 odom / IMU / 轮速计；
- 再把 odom + 深度/点云输入 RTAB-Map 建图。

因此本节点的 depth 模式是 feasibility test，不建议作为最终 Nav2 建图输入。

## 已验证结果

2026-07-14 在当前设备上完成验证：

```bash
colcon build --symlink-install --packages-select luxi_rtab_map --cmake-args -DCMAKE_BUILD_TYPE=Release
```

RGB-D 模式：

```text
received=1
mapped=1
lost=0
```

Depth-only 模式：

```text
received=1
mapped=0
lost=1
```
