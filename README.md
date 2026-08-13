# Lunar SLAM

面向 Jetson Orin NX 的 ROS 2 双目建图与网页控制工程。当前 USB 双目默认使用
**CREStereo + Luxi 视觉前端 + RTAB-Map**，VPI 作为低资源备用链路。

## 当前配置

| 项目 | 配置 |
|---|---|
| 平台 | Jetson Orin NX、ROS 2 Humble |
| 主链路 | CREStereo 预训练模型（FP16/CUDA Graph） |
| 备用链路 | VPI OFA/PVA/VIC |
| IMU | H30，默认启用并带启动健康检查 |
| 近场地图 | 0.4–4 m，全量稠密深度 |
| 远距地图 | 4–6 m 按 4×4 采样，6–10 m 按 8×8 采样 |
| 网页默认选项 | CREStereo |

远距层用于补充房间轮廓，不参与超过 4 m 的视觉里程计约束。这样可以避免低视差
远距深度拉坏位姿，同时控制地图大小和 NX 负载。

## 本次 main 合并

当前 `nvidia_nx` 已完整合入 `origin/main` 至 `0176f7c`，主要更新如下：

- 优化运动建图稳定性：增加视觉/IMU 一致性、角速度和位姿跳变门禁，降低运动模糊、
  错误重定位及地图重影。
- 增加地图点云过滤、地面/障碍分类和可通行区域标注；OctoMap 默认分辨率调整为 5 cm。
- 跑通已建静态地图的定位与导航闭环，增加路径跟随状态、速度仲裁、停止和急停控制。
- 修复底盘运动方向相反问题，并加入登月小车 D1 SDK、控制桥和网页启停/姿态控制。
- 优化 HLoc + ICP 定位、`map→odom` 对齐、跟踪健康检查和路径跟随安全门禁。

`main` 本身不包含 NX 的 USB/CRE 实现。本次融合保留完整 USB 相机工作区，并把上述
导航、定位、D1 和网页能力接入 CREStereo 主链路；VPI 继续作为网页备用项。

## 快速启动

网页控制：

```bash
cd /home/nvidia/Desktop/lunar_-slam
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch luxi_web_control lekiwi_web_control.launch.py \
  bind_address:=0.0.0.0 http_port:=8080
```

浏览器访问 `http://<宿主机 IP>:8080`，选择 CREStereo 后点击“开始建图”。结束时先在
网页点击“停止建图”，等待数据库保存完成再关闭终端。

直接启动 CREStereo 并打开 RViz：

```bash
cd /home/nvidia/Desktop/lunar_-slam
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch lunar_usb_rtabmap_bringup usb_crestereo_rtabmap.launch.py \
  rviz:=true use_imu:=true new_map:=true
```

H30 是默认姿态源；没有 H30 时仅可在诊断或纯视觉对比中显式传入 `use_imu:=false`。
网页和终端不要同时启动建图，避免相机占用、重复 TF 和 GPU 竞争。

## 当前实测

静止办公室场景的最终主链路结果：

| 指标 | 结果 |
|---|---:|
| 深度输出 | 5.94–6.02 Hz |
| 视觉里程计 | 平均 4.90 Hz，中位数 5.91 Hz |
| 跟踪接受率 | 146 / 146 |
| GPU 利用率 | 平均 49.1%，中位数 41.5% |
| 整机输入功耗 | 平均 12.70 W |
| 内存 | 平均 6.57 GiB |

以上是当前场景的资源与重复性测试，不代表带真值的绝对精度认证。6–10 m 深度可用于
稀疏环境轮廓，可靠定位和主体结构仍以 0.4–4 m 为主。

## 文档

- [USB 相机与建图说明](device/USBCameraSDK/ros2_ws/README.md)
- [USB RTAB-Map 启动包](device/USBCameraSDK/ros2_ws/src/lunar_usb_rtabmap_bringup/README.md)
- [CREStereo 测试结果](maps/benchmarks/usb_crestereo_20260812/README.md)
- [网页控制说明](project/luxi-web-control/README.md)
