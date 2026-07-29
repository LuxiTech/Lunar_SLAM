# LuXi StereoCamera workspace source layout

这个 `src` 目录只把项目自研功能包放在第一层，第三方/供应商源码统一放在 `third_party/` 下。

## Project packages

- `hikrobot_camera_driver/`：海康相机 SDK 封装、双目相机节点、相机配置读取。
- `stereo_depth/`：双目深度、点云、RGBD 数据和累积 RGB 点云地图节点。
- `hik_bringup/`：整机启动、相机+IMU+RTAB-Map launch、RViz 配置和运行参数。

## Third-party packages

- `third_party/diagnostics/`：ROS diagnostics 相关包。
- `third_party/octomap_msgs/`：OctoMap ROS 消息。
- `third_party/perception_pcl/`：PCL ROS 封装。
- `third_party/rtabmap/`：RTAB-Map 核心库。
- `third_party/rtabmap_ros/`：RTAB-Map ROS2 封装。
- `third_party/serial_ros2/`：串口库。
- `third_party/yesense_ros2/`：轮趣/也仁 H30 IMU 官方 ROS2 驱动。

`colcon` 会递归扫描 `src/third_party`，因此移动后仍然可以直接在工作空间根目录执行：

```bash
colcon build --symlink-install
```

## Optional packages

`third_party/rtabmap_ros/rtabmap_costmap_plugins` 依赖 Nav2 costmap 开发包。当前 LuXi 双目视觉 SLAM 流程不依赖它，所以已经放置 `COLCON_IGNORE`，避免普通构建被 Nav2 可选依赖阻塞。

