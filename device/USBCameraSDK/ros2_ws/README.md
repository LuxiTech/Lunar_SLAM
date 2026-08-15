# USB 双目建图

该工作区负责 USB 双目采集、深度估计、Luxi 视觉里程计和 RTAB-Map 建图。部署平台为
Jetson Orin NX + ROS 2 Humble。

## 链路选择

| 链路 | 定位 | 特点 |
|---|---|---|
| CREStereo | 默认主链路 | 深度覆盖完整，适合 0.4–4 m 主地图和 4–10 m 稀疏轮廓 |
| CREStereo MAX | 可选极致档 | 960×540 RGB-D、10 Hz 深度目标，优先帧率和画质 |
| VPI | 网页备用项 | 资源较低，近场更稳定，但有效深度更稀疏 |

已移除 CUDA SGM 和经典前端。网页提供 CREStereo 稳定档、CREStereo MAX 和 VPI；
前两项使用同一深度算法，但性能调度相互独立。

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

CREStereo 极致模式：

```bash
ros2 launch lunar_usb_rtabmap_bringup \
  usb_crestereo_max_performance_rtabmap.launch.py \
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

网页默认选择稳定 CREStereo，需要时手动切换“CREStereo 极致”。停止时必须先点击
“停止建图”，等待 RTAB-Map 保存完成。

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

## NX 实测（当前高质量主链路）

主链路使用 640×360 四输入级联模型，并用 320×180 FP16 反向模型做左右一致性校验。
在当前反光走廊的同一组双目数据和实机静态链路上测得：

| 指标 | 结果 |
|---|---:|
| 深度节点单独运行 | 2.68–2.70 Hz |
| 完整 H30 + LightGlue + RTAB 链路 | 2.32–2.47 Hz |
| 有效深度中位数 | 约 46% |
| 静态深度变化 p50 / p90 | 145 / 568 mm |
| 相邻帧变化 >0.5 m | 24.2% → 12.5% |
| 视觉前端静态匹配 | 425–431 matches，53–58 inliers |
| 448 s 静止末端位移 / yaw 范围 | 1.34 cm / 1.08° |

低置信度反光点会变为未知，而不是保留为假障碍；因此点数少于旧单次模型，但融合后
墙体和地面轮廓更稳定。网页“CREStereo 极致”仍保留高帧率模式，适合优先看帧率而非
反光场景几何置信度的情况。

极致档仍采集相机原始 1920×1080/10 Hz MJPEG，CREStereo 推理为 320×180 FP16，
RGB-D 发布为 960×540。CPU 预处理与 GPU 推理流水化后，深度内部吞吐约 10 Hz；完整
视觉里程计平均 7.74 Hz、中位数 9.19 Hz。该档平均约 18.85 W，GPU 平均/中位数
66.8%/92%，测试最高 77.2 °C、无节流。它带 85 °C 告警和 92 °C 自动停止保护。
高帧率档使用 320×180 模型冲刺 10 Hz；需要真实几何细节时使用默认的原生 640×360 主链路。详见[高帧率模式基准](../../../maps/benchmarks/usb_crestereo_max_performance_20260813/README.md)。

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
