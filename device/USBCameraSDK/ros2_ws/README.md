# USB 双目建图

该工作区负责 USB 双目采集、深度估计、Luxi 视觉里程计和 RTAB-Map 建图。部署平台为
Jetson Orin NX + ROS 2 Humble。

## 链路选择

| 链路 | 定位 | 特点 |
|---|---|---|
| CREStereo | 默认主链路 | 深度覆盖完整，适合 0.4–4 m 主地图和 4–10 m 稀疏轮廓 |
| VPI | 网页备用项 | 资源较低，近场更稳定，但有效深度更稀疏 |

已移除 CUDA SGM 和经典前端选项。网页与终端均只保留 CREStereo、VPI 两条链路。

## 启动

从仓库根目录加载统一安装空间：

```bash
cd /home/nvidia/Desktop/lunar_-slam
source /opt/ros/humble/setup.bash
source install/setup.bash
```

CREStereo 主链路并打开 RViz：

```bash
ros2 launch lunar_usb_rtabmap_bringup usb_crestereo_rtabmap.launch.py \
  rviz:=true rtabmap_viz:=false use_imu:=true new_map:=true
```

VPI 备用链路：

```bash
ros2 launch lunar_usb_rtabmap_bringup usb_rtabmap.launch.py \
  rviz:=true rtabmap_viz:=false use_imu:=true new_map:=true
```

网页控制：

```bash
ros2 launch luxi_web_control lekiwi_web_control.launch.py \
  bind_address:=0.0.0.0 http_port:=8080
```

网页默认选择 CREStereo。停止时必须先点击“停止建图”，等待 RTAB-Map 保存完成。

## 建图策略

| 距离 | 用途 | 保留方式 |
|---|---|---|
| 0.4–4 m | 视觉里程计和主体地图 | 全量稠密 |
| 4–6 m | 中距离结构 | 每 4×4 像素保留一点 |
| 6–10 m | 远距环境轮廓 | 每 8×8 像素保留一点 |
| 超过 10 m | 不使用 | 丢弃 |

视觉里程计只使用 0.4–4 m 深度。4–10 m 数据仅进入 RTAB-Map 稀疏地图，因此不会让
低视差噪声主导位姿，也不会显著增加实时计算量。

当前静止场景深度重复性如下。该数据没有激光真值，只反映帧间稳定性：

| 距离 | 深度 MAD p50 / p90 |
|---|---:|
| 0.4–3 m | 34 / 129 mm |
| 3–6 m | 108 / 340 mm |
| 6–10 m | 239 / 418 mm |

10 m 是输出上限，不代表 10 m 范围内都具有 D435i 级绝对精度。无纹理、反光、过曝和
遮挡区域仍会缺失或波动；月面类场景应保留地表纹理，并避免只拍摄大面积纯色区域。

## NX 实测资源

最终 CREStereo 主配置使用 180×320 FP16 模型、CUDA Graph 和 6 Hz 最新帧调度：

| 指标 | 结果 |
|---|---:|
| 原始深度 | 5.94–6.02 Hz |
| 视觉里程计 | 平均 4.90 Hz，中位数 5.91 Hz |
| 平均匹配内点 | 91.2 |
| GPU | 平均 49.1%，中位数 41.5%，p90 99% |
| 整机输入功耗 | 平均 12.70 W |
| 系统内存 | 平均 6.57 GiB |
| 深度节点 | 0.518 CPU 核 / 1.103 GiB RSS |
| 视觉前端 | 0.394 CPU 核 / 1.527 GiB RSS |
| RTAB-Map | 0.121 CPU 核 / 0.298 GiB RSS |

GPU 峰值来自 CREStereo 和 LightGlue 的短时突发；中位数明显较低，说明帧间仍有调度
余量。解除 6 Hz 限速可提高深度帧率，但会增加功耗并挤占视觉前端资源，不建议作为
默认配置。

## 关键话题

| 话题 | 内容 |
|---|---|
| `/sensors/rgbd/rgbd_image` | 统一原子 RGB-D |
| `/luxi_visual_frontend/odom` | CREStereo 链路视觉里程计 |
| `/luxi_visual_frontend/rgbd_image` | 已筛选、与里程计同时间戳的 RGB-D |
| `/rtabmap/cloud_map` | RTAB-Map 实时彩色点云 |
| `/rtabmap/map` | 二维占据栅格 |

快速检查：

```bash
ros2 topic hz /sensors/rgbd/rgbd_image
ros2 topic hz /luxi_visual_frontend/odom
ros2 topic hz /rtabmap/cloud_map
```

若建图为空，先检查相机是否被其他进程占用，再确认 RGB-D 与里程计持续发布。若出现
重影或镜像房间，应检查里程计丢失/重启日志，不要通过翻转图像掩盖位姿跳变。

H30 默认为开启状态。驱动会在 2 秒无串口字节时报告 `H30 NO DATA`，持续静默 5 秒会
自动重开串口；建图健康门禁在 8 秒仍无 IMU 时停止启动。当前设备若出现该告警，应先
检查 H30 本体供电/输出状态并重新插拔，不能把 `/dev/imu-H30` 存在等同于正在出流。

完整测试数据见 [CREStereo 基准记录](../../../maps/benchmarks/usb_crestereo_20260812/README.md)。
