# lunar_d435i_rtabmap_bringup

本包是 D435i 与 RTAB-Map 的系统适配层，不修改官方 `rtabmap` / `rtabmap_ros` 源码。

## 输入

- RGB：`/camera/camera/color/image_raw`
- 对齐深度：`/camera/camera/aligned_depth_to_color/image_raw`
- CameraInfo：`/camera/camera/color/camera_info`
- 相机点云：`/camera/camera/depth/color/points`
- 相机 TF：`camera_link -> camera_*`

## 输出

- TF：`map -> odom -> base_link -> camera_link`
- 栅格地图：`/rtabmap/map`
- 视觉里程计：`/rtabmap/odom`
- RTAB-Map 数据库：默认 `~/.ros/lunar_d435i_rtabmap.db`

## 运行

```bash
source /opt/ros/humble/setup.bash
source /home/lunar/project/lunar_slam/device/D435i/ros2_ws/install/setup.bash
ros2 launch lunar_d435i_rtabmap_bringup d435i_rtabmap.launch.py
```

重新建图时删除旧数据库：

```bash
ros2 launch lunar_d435i_rtabmap_bringup d435i_rtabmap.launch.py \
  rtabmap_args:="--delete_db_on_start"
```

当前没有小车实体时，默认发布 `base_link -> camera_link` 零位姿静态 TF。后续接入真实底盘后，应把 `camera_x/y/z/roll/pitch/yaw` 改成相机相对底盘的实测外参。
