# Lunar Client API

## 包与入口

| ROS 2 包 | 公开能力 | 常用入口 |
| --- | --- | --- |
| `luxi_adapter` | Hik/D435i/D455 统一传感器接口 | `sensor_bringup.launch.py` |
| `luxi_visual_frontend` | 视觉里程计与特征 RGB-D | `visual_odometry.launch.py` |
| `luxi_rtab_map` | 建图、定位、数据库管理 | `rgbd_mapping_learned.launch.py` |
| `luxi_hloc` | HLoc 全局粗定位 | `hloc_localization.launch.py` |
| `luxi_location` | ICP 精定位 | `icp_localization.launch.py` |
| `luxi_voxel_navigation` | OctoMap/点云转换与体素导航 | `saved_map_navigation.launch.py` |
| `luxi_3d_navigation` | 三维规划、跟踪与安全门禁 | `saved_map_navigation.launch.py` |
| `luxi_navigation` | 键盘控制与语义标注 | `manual_map_annotation.launch.py` |
| `luxi_web_control` | Web 控制入口 | `web_control.launch.py` |
| `slam_d1_bridge` | 标准速度到 D1 厂家命令 | `slam_d1_bridge.launch.py` |

可用参数以运行时接口为准：

```bash
ros2 launch <package> <file>.launch.py --show-args
ros2 param list /<node>
ros2 param describe /<node> <parameter>
```

## 主要 ROS 话题

| 话题 | 消息类型 | 方向/用途 |
| --- | --- | --- |
| `/sensors/rgbd/rgbd_image` | `rtabmap_msgs/msg/RGBDImage` | 统一原子 RGB-D 输入 |
| `/sensors/rgbd/color/image_raw` | `sensor_msgs/msg/Image` | 兼容彩色图像输出 |
| `/sensors/rgbd/depth/image_raw` | `sensor_msgs/msg/Image` | 兼容注册深度输出 |
| `/sensors/rgbd/color/camera_info` | `sensor_msgs/msg/CameraInfo` | 相机内参 |
| `/sensors/imu/data_raw` | `sensor_msgs/msg/Imu` | 原始 IMU |
| `/sensors/imu/data` | `sensor_msgs/msg/Imu` | 建图 IMU |
| `/luxi_visual_frontend/odom` | `nav_msgs/msg/Odometry` | 视觉里程计 |
| `/luxi_visual_frontend/rgbd_image` | `rtabmap_msgs/msg/RGBDImage` | 带外部特征的 RTAB 输入 |
| `/rtabmap/cloud_map` | `sensor_msgs/msg/PointCloud2` | 累积三维点云 |
| `/rtabmap/map` | `nav_msgs/msg/OccupancyGrid` | 二维占据栅格 |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 标准运动控制输入 |
| `/d15041873/command/user_command` | `ddt_msgs/msg/UserCommand` | D1 厂家命令输出 |
| `/d15041873/status/battery1` | `sensor_msgs/msg/BatteryState` | 电池 1 状态 |
| `/d15041873/status/battery2` | `sensor_msgs/msg/BatteryState` | 电池 2 状态 |

默认 TF 链为 `map -> odom -> base_link -> camera optical frame -> imu_link`。

## 服务与控制约束

- IMU 水平标定：`std_srvs/srv/Trigger`，服务名由 `calibration_service` 参数配置。
- 语义标注节点提供 `~/start_polygon`、`~/finish_polygon`、`~/undo_vertex`、
  `~/cancel_polygon`、`~/delete_obstacle`、`~/save`、`~/reload`、`~/list`。
- D1 控制默认使用 `ROS_DOMAIN_ID=42` 和有线 `192.168.123.0/24`；运动前必须保留
  物理急停并确认没有其他控制发布者。

## C++ API

制品中的 `install/include/` 是公开 C++ 头文件，链接库位于 `install/lib/`。使用方先
加载 `activate.bash`，再通过对应包的 `find_package(...)` 和导出 target 链接。ABI 仅
保证与本发行版标注的 ARM64/ROS Humble/GCC 版本兼容。
