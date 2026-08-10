# Lunar SLAM 算法项目

`project/` 存放与具体相机解耦的 ROS 2 算法包。当前主链路支持海康双目、USB 双目
与 Intel RealSense D435i 替换接入；三类设备经过 `luxi_adapter` 后使用相同的 RGB-D、
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
Hik 双目 + H30       USB 双目 + H30       D435i
       │                    │               │
       └──────────── luxi_adapter ──────────┘
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
| `luxi_adapter` | 提供统一 RGB-D/IMU 适配节点，并管理 Hik/D435i 通用 profile |
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

Hik 与 D435i 可通过 `luxi_adapter` 选择硬件；USB 按 D435i 的设备包结构由
`lunar_usb_rtabmap_bringup` 直接组合采集、Adapter 和 RTAB-Map。Hik/D435i 的通用
硬件入口为：

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

Hik 本机一键建图可直接使用项目入口。RViz 已包含 RGB、深度和地图显示，不要再同时
打开 RTAB-Map 自带 Qt 可视化：

```bash
ros2 launch luxi_rtab_map hik_mapping.launch.py \
  rviz:=true rtabmap_viz:=false \
  database_path:=/tmp/hik_mapping_test.db
```

USB 双目与 H30 一键建图使用独立入口。该入口直接消费 USB 深度前端发布的原子
RGB-D，避免再次拆分同步，并针对 480×270 图像关闭里程计二次降采样：

```bash
source device/USBCameraSDK/ros2_ws/install/setup.bash
ros2 launch lunar_usb_rtabmap_bringup usb_rtabmap.launch.py \
  rviz:=true rtabmap_viz:=false new_map:=true use_imu:=true
```

该入口默认使用 `vpi_learned`（VPI OFA/PVA/VIC 深度、RTAB F2M 里程计与 Luxi
SuperPoint/LightGlue 建图特征）；需要低内存 CUDA 回退链时显式追加 `mode:=stable`。
当前固定为 `/dev/imu-H30` 的设备已完成零偏恢复，并通过 921600 baud / 200 Hz 独立
六轴验收及 10 Hz 外触发动态建图验收。USB 网页与终端链路默认使用
`use_imu:=true`；驱动和启动健康门禁会拒绝任何无效轴，无 IMU 诊断时可显式传入
`use_imu:=false`。

当前 NX 完整链路（VPI + Luxi + H30 + RTAB-Map + RViz）实测约占 2.35 个 CPU 核，
最高单进程 0.78 核，所列进程 RSS 合计约 2.69 GiB；GR3D 平均 37.2%、峰值 98%，
31 次采样没有达到 99%。动态回停 10 秒漂移为 2.2 mm / 0.082°，详细的精细度、
闭环、深度填充率和逐模块资源占比见
[USB 相机工作区 README](../device/USBCameraSDK/ros2_ws/README.md#当前默认-vpi--luxi--h30-全链路验收2026-08-10)。

历史 x86 主机、ROS 2 Lyrical 实测：USB 采集与实时彩色点云约 10 Hz，单帧约 2.9 万
有效点，深度处理约 11 ms；连续原子 RGB-D/IMU 同步检查通过。针对该相机固定数据调参
后，有效深度由 13.1/17.8/13.2% 提升到 14.5/19.2/14.1%，实机常见约 20–31%。
GFTT+ORB 网格特征下里程计质量通常为 120–150；78 秒实机测试保存了 58 个数据库
节点，优化图包含 3 个有效位姿，并生成 133×100（5 cm/格）占据栅格。正式评价仍应
在纹理充足、0.4–5 m 范围内缓慢走一圈。

终端手工建图和网页“开始建图”二选一。网页控制节点会在启动前检查
`/luxi_visual_frontend` 和 `/rtabmap/rtabmap`；发现外部建图链路时拒绝再次启动，避免
重复运行两套 SuperPoint、LightGlue 和 RTAB-Map。

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
| `/usb_stereo/points` | USB 当前帧实时彩色点云（约 10 Hz） |
| `/rtabmap/odom` | 标准 RTAB RGB-D 视觉里程计 |
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
| `luxi_visual_frontend` | 当前约 4.2 Hz |
| 视觉里程计端到端延迟 | 平均约 0.41 s |
| RViz RGB 预览延迟 | 平均约 0.03 s |
| RViz 深度预览延迟 | 平均约 0.12 s |
| RTAB 检测率 | 配置为 1 Hz |

一次满链路资源采样约为：传感器组件容器 1 个 CPU 核、视觉前端 0.5 个 CPU 核和
约 2.0 GB RSS、相机驱动 0.35 个 CPU 核、RTAB 约 0.3 个 CPU 核和 0.5 GB RSS；
GPU 在前端推理期间可达到 99%。这些数值用于判断回归，不是固定资源上限。

### GPU 峰值诊断基线（2026-08-06）

测试条件：Orin NX `MAXN`、Hik 1024×750/10 Hz、VPI 深度、TensorRT FP16
SuperPoint、AMP FP16 LightGlue、单套 RTAB-Map、一个 RViz，关闭 `rtabmap_viz`。
`GR3D_FREQ` 使用 250 ms 周期采样，场景和相机运动会引起波动。

| 指标 | 实测结果 |
| --- | ---: |
| GPU 平均占用 | 56.7% |
| GPU 瞬时峰值 | 99% |
| 大于等于 90% 的采样占比 | 22.8% |
| 暂停学习视觉前端后的 GPU 平均占用 | 45.3% |
| 暂停 RViz 后的 GPU 平均占用 | 52.5% |
| 视觉里程计发布率 | 约 4.23 Hz |
| SuperPoint 提取 | 约 55.1 ms |
| LightGlue 匹配 | 约 32.9 ms |
| 单帧学习前端总耗时 | 约 94.7 ms |
| 示例关键点/匹配/内点 | 422 / 373 / 220 |
| 示例内点率 | 91.3% |

“暂停学习视觉前端”和“暂停 RViz”是分别执行的短时隔离实验，用于判断负载来源；因
场景、显示订阅和 GPU 动态频率不同，不能将表中差值直接相加作为单个节点的固定占用。

瞬时 98–99% 不表示建图异常，也不能直接解释为“当前纹理非常丰富”。SuperPoint 的
密集卷积扫描整张 `800×586` 网络输入，基础计算量主要由输入尺寸决定；CUDA kernel
执行时会短暂占满计算单元。纹理丰富会产生更多关键点和 LightGlue 候选，使高负载持续
时间变长，但通常不会显著改变已经接近 100% 的瞬时峰值。应优先判断 30 秒以上的平均
占用、推理延迟、里程计频率、温度和掉帧，而不是单个峰值。

### GPU 优化方案与优先级

1. **禁止重复链路。** 同一时刻只允许一个 `luxi_visual_frontend` 和一个
   `/rtabmap/rtabmap`。不要在终端建图运行时再次点击网页“开始建图”。这是最高优先级，
   因为重复链路会同时增加 GPU、CPU 和约数 GiB 内存。
2. **只保留一种本机建图可视化。** 正式配置使用 `rviz:=true`、
   `rtabmap_viz:=false`。`rtabmap_viz` 与 RViz 同时运行会重复保存和绘制地图，本次现场
   单个 `rtabmap_viz` 约占 2 GiB RSS 和 60% 以上单核 CPU。
3. **保持已验证的 FP16 推理配置。** 生产配置继续使用 `resize_max=800`、
   `max_keypoints=1024`、`nms_radius=4`、TensorRT FP16 SuperPoint 和 AMP FP16
   LightGlue，并启用 CUDA Graph。诊断话题必须显示
   `tensorrt_fp16_cudagraph` 和 `pytorch_amp_fp16_cudagraph_512x3`。
4. **仅在持续超载时降低工作量。** 如果 30 秒平均 GPU 长期超过 70%、温度持续超过
   80°C 或里程计开始掉帧，再依次评估视觉前端 5 Hz 降至 4 Hz、输入长边 800 降至
   720，以及限制匹配点数。每一步都必须重新测试快速转动、弱纹理墙面和回环，不允许
   只根据 GPU 数字上线。
5. **暂不启用 720/640 或 INT8。** 已有质量门禁中，720 输入正确匹配减少 16.7%，
   640 减少 29.8%；INT8 内点率为 0.815，低于 FP16 的 0.949。它们可用于独立实验，
   不能替换当前生产配置。

当前验收目标是：单套节点、Hik RGB-D 约 9–10 Hz、视觉里程计不低于 4 Hz、30 秒
平均 GPU 位于约 30–60%、温度低于 80°C、内点率通常不低于 80%，并且没有持续积压
或跟踪丢失。满足这些条件时，偶发 98–99% 峰值属于正常的高效 GPU 调度，不应通过
限制 GPU 时钟来掩盖。

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
