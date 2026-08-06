# Lunar SLAM 算法项目

`project/` 存放与具体相机解耦的 ROS 2 算法包。当前主链路支持海康双目相机与
Intel RealSense D435i 替换接入；两类设备经过 `luxi_adapter` 后使用相同的 RGB-D、
IMU 和 TF 接口。正式建图使用 `luxi_visual_frontend` 和项目维护的
`luxi_rtab_map`，不使用海康工程附带的 RTAB-Map 前端或建图配置。

## 当前状态

- 平台：Jetson Orin NX、ROS 2 Humble、CUDA、NVIDIA VPI。
- Hik 输出：保持原视场，将 2048×1500 采集图像缩放为标定对应的 1024×750。
- Hik 深度：VPI OFA + PVA + VIC SGM；VPI 运行失败时才回退 OpenCV SGBM。
- 视觉前端：CUDA SuperPoint + LightGlue，输出视觉里程计和 RTAB-Map 外部特征。
- 建图后端：系统 ROS 2 `rtabmap_slam`，由 `luxi_rtab_map` 统一配置。
- 可视化：RViz 同时显示 RGB、彩色深度、里程计、点云和占据栅格。
- 已验证：相关包编译通过，工作空间现有 97 项测试全部通过。

## 系统链路

```text
Hik 双目 + H30                         D435i
       │                                 │
       └────────── luxi_adapter ─────────┘
                         │
             /sensors/rgbd/rgbd_image
             /sensors/imu/data
             /tf、/tf_static
                         │
             luxi_visual_frontend
             SuperPoint + LightGlue
                         │
        ┌────────────────┴────────────────┐
        │                                 │
/luxi_visual_frontend/odom   /luxi_visual_frontend/rgbd_image
        └────────────────┬────────────────┘
                         │ exact sync
                   luxi_rtab_map
                         │
              map / cloud_map / database
```

算法节点使用原子 `RGBDImage` 消息，不再重新同步拆分的 RGB、Depth 和 CameraInfo。
这样可以保证颜色、深度、内参属于同一相机帧，并避免高负载时错配相邻帧。拆分图像
话题仍保留给兼容节点和预览使用。

在 RTAB-Map 启动前，`luxi_rtab_map` 的 C++ 同步门禁会检查：

- RGBD 外层、RGB、Depth 和两个 CameraInfo 时间戳一致；
- RGBD 时间戳单调递增；
- 相机帧附近存在有效 IMU 样本；
- 连续多帧通过，而不是单帧偶然通过。

同步失败时建图后端不会启动。

## 目录与职责

| 包 | 职责 |
| --- | --- |
| `luxi_adapter` | 选择 Hik/D435i 硬件 profile，发布统一 RGB-D、IMU 和 TF |
| `luxi_visual_frontend` | SuperPoint + LightGlue RGB-D 视觉里程计与 RTAB 外部特征 |
| `luxi_RTAB_Map` | 同步门禁、RTAB-Map 建图/定位、数据库保护和 RViz |
| `luxi_hloc` | NetVLAD/HLoc 全局检索与粗定位，不参与实时建图前端 |
| `luxi_location` | 基于统一 RGB-D 接口的 Open3D ICP 定位 |
| `luxi_voxel_navigation` | RTAB 数据库、点云、OctoMap 转换和体素导航保护 |
| `luxi_3d_navigation` | 保存 OctoMap 上的地面支撑型三维 A* 规划 |
| `luxi_navigation` | 键盘遥控和手工语义地图标注 |
| `luxi-web-control` | 网页遥控，发布标准 `geometry_msgs/Twist` |
| `luxi_semantic_annotation` | 保存地图的离线语义标注验证 |

## 构建

在工作空间根目录执行：

```bash
cd /home/nvidia/Desktop/lunar_slam
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select \
  hikrobot_camera_driver hik_bringup stereo_depth yesense_std_ros2 \
  luxi_adapter luxi_visual_frontend luxi_rtab_map

source install/setup.bash
```

海康目录中附带的 `third_party/rtabmap` 和 `third_party/rtabmap_ros` 已通过
`COLCON_IGNORE` 隔离，不应删除该隔离文件或将这些包加入当前 overlay。

## 统一硬件选择与建图

正式链路只在 `luxi_adapter` 启动时选择硬件。Hik 与 D435i 后面的视觉前端、RTAB、
RViz、定位和导航命令完全一致。终端一启动所选硬件：

```bash
cd /home/nvidia/Desktop/lunar_slam
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch luxi_adapter sensor_bringup.launch.py hardware:=hik
# 或：hardware:=d435i
```

终端二启动通用学习前端、RTAB 后端和 RViz：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/Desktop/lunar_slam/install/setup.bash
ros2 launch luxi_rtab_map rgbd_mapping_learned.launch.py \
  new_map:=true load_saved_map:=false
```

链路按顺序完成：

1. profile 启动所选相机、IMU、标定 TF 和 Adapter；
2. Hik profile 额外启动 VPI 双目深度；
3. 检查原子 RGB-D 与 IMU 同步；
4. 加载 CUDA SuperPoint/LightGlue；
5. 启动项目 RTAB-Map 后端；
6. RViz 从统一 `/sensors/rgbd/*` 显示 RGB、深度和 RTAB 地图。

无图形桌面时：

```bash
ros2 launch luxi_rtab_map rgbd_mapping_learned.launch.py \
  rviz:=false new_map:=true load_saved_map:=false
```

正常启动应看到以下关键日志：

```text
TriggerMode=On, TriggerSource=Line0
Hik stereo cameras are ready
Depth backend: vpi_ofa_pva_vic
SYNC PASS: ... valid_max_rgbd_skew=0.000 ms
Learned frontend inference device: cuda
rtabmap subscribed to (exact sync)
```

`external_trigger:=false` 只用于排查无硬件触发时能否取流，不作为正式建图配置。

同一时刻只能启动一个硬件 profile，不能让 Hik 和 D435i 同时发布统一传感器话题。
以后增加型号时，新增 `luxi_adapter/launch/<profile>.launch.py` 和
`luxi_adapter/config/<profile>_sensor_bringup.yaml`；后续算法包不增加硬件分支。

## 地图数据库

### 创建新地图

Hik 一键入口默认 `new_map:=true`。未指定路径时会原子选择下一个编号：

```text
maps/rtab_maps/map001.db
maps/rtab_maps/map002.db
...
```

结束建图时在 launch 终端按一次 `Ctrl-C`，等待出现：

```text
Saving database/long-term memory...done!
```

不要直接杀死 `rtabmap` 进程或在数据库保存过程中断电。

### 继续已有地图

```bash
ros2 launch luxi_rtab_map hik_mapping.launch.py \
  database_path:=/home/nvidia/Desktop/lunar_slam/maps/rtab_maps/map001.db \
  new_map:=false load_saved_map:=true
```

### 显式覆盖数据库

已有数据库在 `new_map:=true` 时默认受保护。确实要删除并重建同一路径时，必须显式传入：

```bash
ros2 launch luxi_rtab_map hik_mapping.launch.py \
  database_path:=/absolute/path/map.db \
  new_map:=true overwrite_existing_database:=true
```

该选项会覆盖目标数据库，使用前应确认路径。

地图查看、PLY 导出和 OctoMap 转换见 [maps/README.md](../maps/README.md) 与
[tools/README.md](../tools/README.md)。

## 主要话题和 TF

| 接口 | 类型/用途 |
| --- | --- |
| `/sensors/rgbd/rgbd_image` | 统一原子 RGB-D 算法输入 |
| `/sensors/rgbd/color/image_raw` | 兼容用彩色图像 |
| `/sensors/rgbd/depth/image_raw` | 兼容用注册深度图 |
| `/sensors/imu/data_raw` | 统一原始 IMU |
| `/sensors/imu/data` | 建图使用的 IMU |
| `/luxi_visual_frontend/odom` | `odom -> base_link` 视觉里程计 |
| `/luxi_visual_frontend/rgbd_image` | 带 SuperPoint 特征的 RTAB 输入 |
| `/stereo/preview/left_color` | RViz 低延迟 RGB 预览 |
| `/stereo/preview/depth_visual` | RViz 彩色深度预览 |
| `/rtabmap/cloud_map` | 累积三维点云 |
| `/rtabmap/map` | 二维占据栅格 |

预期 TF 链：

```text
map -> odom -> base_link -> left_camera_optical_frame -> imu_link
```

Hik 的相机内参和双目外参来自 `hikrobot_camera_driver/config/stereo_left.yaml` 与
`stereo_right.yaml`。相机到 IMU 外参在
[hik_sensor_bringup.yaml](luxi_adapter/config/hik_sensor_bringup.yaml) 中配置。
当前 `base_link` 到相机的平移由安装配置决定；如果机器人基座原点不在左相机中心，
必须填入实测安装外参，不能凭估计修改。

## 已测性能

以下数据来自当前 Orin NX、1024×750、Hik 10 Hz 硬件触发配置。场景、温度、供电和
RViz 订阅都会影响结果。

| 项目 | 实测结果 |
| --- | --- |
| 相机输入 | 10 Hz |
| 独立 RGB-D 深度链路 | 通常 9–10 Hz |
| 单帧深度处理 | 通常 90–100 ms |
| VPI 立体匹配阶段 | 通常 63–67 ms |
| 满链路 + RViz 深度输出 | 通常约 7–9.5 Hz |
| `luxi_visual_frontend` | 约 3.2–3.3 Hz |
| 视觉里程计端到端延迟 | 平均约 0.41 s |
| RViz RGB 预览延迟 | 平均约 0.03 s |
| RViz 深度预览延迟 | 平均约 0.12 s |
| RTAB 检测率 | 配置为 1 Hz |

一次满链路资源采样约为：传感器组件容器 1 个 CPU 核、视觉前端 0.7 个 CPU 核和
约 1.6 GB RSS、相机驱动 0.35 个 CPU 核、RTAB 约 0.3 个 CPU 核和 0.4 GB RSS；
GPU 在前端推理期间可达到 99%。这些数值用于判断回归，不是固定资源上限。

### Jetson 供电限制

当前设备在 MAXN 满载时曾出现：

```text
System throttled due to Over-current
```

过流降频会使满链路深度帧率短时下降到约 6–8 Hz。代码已经保持原分辨率、视场、
128 disparity 和深度过滤精度；如果要求持续稳定 10 Hz，应先检查 Jetson 电源适配器、
供电线、载板和同一路电源上的 USB 负载，而不是降低图像或深度参数。

## 常见日志

### `Graph has changed! The whole cloud is regenerated.`

这是 RTAB-Map 位姿图新增节点、回环或优化后重新生成全局点云的正常提示，不是深度
错误。偶尔出现无需处理；只有持续高频出现并伴随地图跳动时，才需要检查回环误匹配。

### `Stereo is NOT SUPPORTED`

这是 RViz/OpenGL 的立体显示模式提示，与双目相机算法无关。普通单窗口 RViz 可以继续
正常显示 RGB、深度和地图。

### 启动后没有 `SYNC PASS`

依次检查：

```bash
ros2 topic list | grep -E 'sensors/rgbd/rgbd_image|sensors/imu/data'
ros2 topic info /sensors/rgbd/rgbd_image --verbose
ros2 topic info /sensors/imu/data --verbose
```

Hik 还应确认两台相机均显示 `TriggerMode=On`、`TriggerSource=Line0`，并且 Line0
诊断存在电平跳变。同步未通过前不要绕过门禁启动 RTAB-Map。

### RGB 比深度更慢

正式 RViz 配置必须订阅 `/stereo/preview/left_color` 和
`/stereo/preview/depth_visual`，不要直接订阅全分辨率拆分图像。预览 QoS 为
Best Effort、Keep Last、Depth 1，可避免可视化消费者积压旧帧。

## 回归测试

```bash
cd /home/nvidia/Desktop/lunar_slam
source /opt/ros/humble/setup.bash
source install/setup.bash

colcon test --packages-select \
  hikrobot_camera_driver hik_bringup \
  luxi_adapter luxi_visual_frontend luxi_rtab_map
colcon test-result --all --verbose
```

硬件回归至少应确认：

1. 两台 Hik 相机帧号偏移初始化后保持一致；
2. `SYNC PASS` 的 `valid_max_rgbd_skew` 为 0 ms；
3. 前端日志显示 `cuda`；
4. RTAB 使用 exact sync 订阅前端 odom 和 RGBD 特征；
5. RViz 的 RGB、DepthPreview、点云和占据栅格均更新；
6. `Ctrl-C` 后数据库保存完成且下次能够加载。

## 子包文档

- [统一硬件接口](luxi_adapter/docs/README.md)
- [RGB-D 建图与定位](luxi_RTAB_Map/docs/rgbd_mapping.md)
- [学习视觉前端](luxi_visual_frontend/README.md)
- [HLoc 全局定位](luxi_hloc/README.md)
- [ICP 定位](luxi_location/README.md)
- [体素导航](luxi_voxel_navigation/README.md)
- [三维 A* 导航](luxi_3d_navigation/README.md)
- [网页控制](luxi-web-control/README.md)

修改硬件驱动时，应只调整 `device/` 和对应 adapter profile；修改建图、定位或导航算法
时，应只调整 `project/`。任何新设备都必须先满足统一原子 RGB-D/IMU/TF 契约，再接入
上层算法。
