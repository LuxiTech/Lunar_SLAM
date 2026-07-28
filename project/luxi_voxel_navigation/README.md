# Luxi Voxel Navigation

该包将 RTAB-Map 的累计稀疏点云转换为 `octomap_msgs/Octomap`，在保存地图上运行
RGB-D 定位，并以二维 A* 生成带机器人半径膨胀的体素路径。路径跟随器默认发布到
`/navigation/cmd_vel`，且必须收到 `/navigation/start` 的 `true` 才会运动。

## 构建

OctoMap 源码与安装前缀固定在项目的 `3parts/octomap`，不写入系统目录：

```bash
cd /home/lunar/project/lunar_slam
source /opt/ros/humble/setup.bash
source device/D435i/ros2_ws/install/setup.bash
export CMAKE_PREFIX_PATH=$PWD/3parts/octomap/install:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=$PWD/3parts/octomap/install/lib:$LD_LIBRARY_PATH
colcon build --packages-select luxi_rtab_map luxi_voxel_navigation --symlink-install
source install/setup.bash
```

## 离线转换

停止建图后，以下工具自动选择最新 `mapNNN.db`，先导出 PLY，再生成 `.bt`：

```bash
tools/export_rtabmap_octomap.sh
```

也可指定数据库和输出目录：

```bash
tools/export_rtabmap_octomap.sh maps/rtab_maps/map011.db maps/octo_maps/map011_octomap
```

## 定位与实时体素转换

先启动 D435i 驱动，再运行：

```bash
ros2 launch luxi_voxel_navigation localization_voxel_navigation.launch.py \
  database_path:=/home/lunar/project/lunar_slam/maps/rtab_maps/map011.db
```

该 launch 使用已有数据库定位（不会扩展数据库），把 `/rtabmap/cloud_map` 发布为
`/navigation/octomap`，并监听 `/navigation/goal_pose`。路径发布到
`/navigation/planned_path`。确认路径与实时定位正确后，才显式开始或停止跟随：

```bash
ros2 topic pub --once /navigation/start std_msgs/msg/Bool '{data: true}'
ros2 topic pub --once /navigation/stop std_msgs/msg/Bool '{data: true}'
```

首次实车联调应保持 `cmd_vel_topic:=/navigation/cmd_vel`，验证无误后才显式改为
`cmd_vel_topic:=/cmd_vel`。稀疏点云只能表达已观察到的障碍，不能替代实体急停或近距
离碰撞传感器。

## 已保存 OctoMap 定位与规划

网页加载地图使用下面的静态体素 launch：

```bash
ros2 launch luxi_voxel_navigation saved_map_navigation.launch.py \
  database_path:=/home/lunar/project/lunar_slam/maps/rtab_maps/map011.db \
  octomap_path:=/home/lunar/project/lunar_slam/maps/octo_maps/map011_octomap/map011.bt \
  cloud_path:=/home/lunar/project/lunar_slam/maps/octo_maps/map011_octomap/map011_cloud.ply \
  hloc_map_directory:=/home/lunar/project/lunar_slam/maps/hloc_maps/map011
```

它使用 GPU HLoc 进行全局粗定位、Open3D ICP 精配准，加载 `.bt` 并发布
`/navigation/planned_path`；路径跟随仍需单独向
`/navigation/start` 发送 `true`，默认不会控制底盘。
