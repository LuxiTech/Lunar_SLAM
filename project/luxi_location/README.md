# luxi_location

`luxi_location` 使用 Open3D ICP 将 D435i 当前深度帧生成的局部点云配准到保存的 RTAB-Map
彩色 PLY。它通过统一的 `/sensors/rgbd/*` 话题读取数据，不依赖 D435i 厂商话题。

ICP 是局部精配准算法，启动后必须提供一次位于地图附近的初始位姿。单独运行时默认为
`/initialpose`；网页自动定位流程会改为接收 RTAB-Map 的
`/rtabmap/localization_pose`，并拒绝协方差不可信的粗定位结果。节点约束输出为地面
三自由度 `(x, y, yaw)`，发布：

- `/luxi_location/pose`：`geometry_msgs/PoseWithCovarianceStamped`
- `/luxi_location/map_cloud`：下采样后的全局地图
- `/luxi_location/aligned_cloud`：配准后的当前局部点云
- `/luxi_location/fitness`：ICP fitness
- `/luxi_location/status`：接受或拒绝原因
- `map -> base_link`：可通过 `publish_tf` 关闭

## 构建

Open3D 0.18 安装在工作区 `3parts/open3d/install`。构建 ROS 包：

```bash
cd /home/lunar/project/lunar_slam
source /opt/ros/humble/setup.bash
colcon build --packages-select luxi_location --symlink-install
source install/setup.bash
```

## 启动

先启动硬件适配层：

```bash
ros2 launch luxi_adapter sensor_bringup.launch.py
```

再启动 ICP；不指定地图时选择编号最大的已导出 `mapNNN` 彩色 PLY：

```bash
ros2 launch luxi_location icp_localization.launch.py
```

提供地图中的粗初始位姿，例如建图起点：

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

观察结果：

```bash
ros2 topic echo /luxi_location/status
ros2 topic echo /luxi_location/pose
ros2 topic hz /luxi_location/pose
```

初始位姿必须足够接近真实位置。默认首次允许 1.5 m、45° 的修正，后续每次允许 0.5 m、20°；
fitness、RMSE 或修正量不满足阈值时保留上一可信位姿。

## RTAB-Map 自动粗定位 + ICP 精定位

网页和导航包使用以下链路，无需在地图上手工点击初始位姿：

```text
统一 RGB-D/IMU -> RTAB-Map 保存数据库全局粗定位
               -> 可信 /rtabmap/localization_pose
               -> luxi_location Open3D ICP
               -> /luxi_location/pose
```

可脱离网页直接启动整条链路：

```bash
ros2 launch luxi_voxel_navigation saved_map_navigation.launch.py \
  database_path:=/path/to/mapNNN.db \
  octomap_path:=/path/to/mapNNN.bt \
  cloud_path:=/path/to/exported_cloud.ply
```

此模式由 RTAB-Map 发布 `map -> odom`，因此 ICP 的 `publish_tf` 被关闭，只发布精配准
位姿供网页和规划准入使用，避免两个节点竞争同一 TF。

## 离线验证

```bash
ros2 run luxi_location offline_registration_test \
  /home/lunar/project/lunar_slam/maps/octo_maps/map012_octomap/luxi_rtab_map_20260727_173205_cloud.ply
```
