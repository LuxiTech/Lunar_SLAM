# Lunar SLAM

面向 Jetson Orin NX 的 ROS 2 双目建图与网页控制工程。当前 USB 双目默认使用
**CREStereo + Luxi 视觉前端 + RTAB-Map**，VPI 作为低资源备用链路。

## 当前配置

| 项目 | 配置 |
|---|---|
| 平台 | Jetson Orin NX、ROS 2 Humble |
| 主链路 | 640×360 CREStereo 级联 + 快速双向置信度校验 |
| 极致模式 | 1080p 双目输入、960×540 RGB-D、10 Hz 深度目标 |
| 备用链路 | VPI OFA/PVA/VIC |
| IMU | H30，默认启用并带启动健康检查 |
| 近场地图 | 0.4–4 m，全量稠密深度 |
| 远距地图 | 4–6 m 按 4×4 采样，6–10 m 按 8×8 采样 |
| 网页默认选项 | CREStereo |
| 当前 D1 | `d15042176`，有线地址 `192.168.123.49` |

远距层用于补充房间轮廓，不参与超过 4 m 的视觉里程计约束。这样可以避免低视差
远距深度拉坏位姿，同时控制地图大小和 NX 负载。

## 本次 main 合并

当前 `nvidia_nx` 已完整合入 `origin/main` 至 `a44435c`，本地合并提交为
`3de1129`。主要更新如下：

- 补全 USB/H30 安装角度校准：网页按需启动 H30、采集重力、按机器人持久化四元数，
  并在后续建图和定位时自动加载；校准完成后自动释放串口。
- 优化运动建图稳定性：增加视觉/IMU 一致性、角速度和位姿跳变门禁，降低运动模糊、
  错误重定位及地图重影。
- 增加地图点云过滤、地面/障碍分类和可通行区域标注；OctoMap 默认分辨率调整为 5 cm。
- 跑通已建静态地图的定位与导航闭环，增加路径跟随状态、速度仲裁、停止和急停控制。
- 修复底盘运动方向相反问题，并加入登月小车 D1 SDK、控制桥和网页启停/姿态控制。
- 优化 HLoc + ICP 定位、`map→odom` 对齐、跟踪健康检查和路径跟随安全门禁。

`main` 本身不包含 NX 的 USB/CRE 实现。本次融合保留完整 USB 相机工作区，并把上述
导航、定位、D1 和网页能力接入 CREStereo 主链路；VPI 继续作为网页备用项。

## 快速启动

当前第二台 D1 的推荐启动方式：

```bash
cd /home/nvidia/Desktop/lunar_-slam
source /opt/ros/humble/setup.bash
source install/setup.bash

./scripts/start_d1_web_control.sh \
  --robot-ns d15042176 \
  --robot-ip 192.168.123.49
```

确认安全提示后访问 `http://192.168.0.187:8080`。`192.168.0.187` 是当前 NX 的 Wi‑Fi
局域网地址；`192.168.123.51` 是 NX 与机器狗之间的隔离控制网口，通常不能作为浏览器
入口。Wi‑Fi 地址发生变化时，以启动脚本最后输出的 URL 为准。网页顶部提供机器人
站立/趴下、速度摇杆和急停；建图区可选择稳定 CREStereo、CREStereo 极致或 VPI。

必须显式指定机器人命名空间，程序不会在多台 D1 间自动选择。若机器人刚开机时只能
发现命名空间却没有 `/command/user_command` 订阅者，脚本会先确认远端命名空间一致且
FSM 为 `idle`，再安全重建 `d1_bringup` 的 DDS participant；整个恢复过程不发送运动
命令。维护时可用 `--no-dds-recovery` 禁用自动恢复。

仅启动网页、不启用 D1 SDK 控制时使用：

```bash
ros2 launch luxi_web_control lekiwi_web_control.launch.py \
  robot_namespace:=d15042176 bind_address:=0.0.0.0 http_port:=8080
```

结束建图时先在网页点击“停止建图”，等待数据库保存完成再关闭终端。

直接启动 CREStereo 并打开 RViz：

```bash
cd /home/nvidia/Desktop/lunar_-slam
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch lunar_usb_rtabmap_bringup usb_crestereo_rtabmap.launch.py \
  rviz:=true use_imu:=true new_map:=true
```

测试极致模式：

```bash
ros2 launch lunar_usb_rtabmap_bringup \
  usb_crestereo_max_performance_rtabmap.launch.py \
  rviz:=true use_imu:=true new_map:=true
```

网页端可直接选择“CREStereo 极致（10 Hz / 高画质）”。该档不会改变稳定主链路的
默认参数；它使用 640 特征点、SuperPoint TensorRT FP16、LightGlue CUDA Graph 和
2 Hz 地图写入，并由 85 °C 告警、92 °C 停止及硬件节流门禁保护设备。

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

极致模式实测深度内部处理稳定在约 10 Hz；完整视觉里程计平均 7.74 Hz、中位数
9.19 Hz，156/156 帧通过。整机平均约 18.85 W，GPU 平均/中位数 66.8%/92%，测试
最高温度 77.2 °C 且未触发节流。960×540 是发布分辨率，真实深度细节仍受
320×180 CREStereo 模型限制；强行发布 1920×1080 深度只有约 6.4 Hz，未作为正式档。

## 文档

- [USB 相机与建图说明](device/USBCameraSDK/ros2_ws/README.md)
- [USB RTAB-Map 启动包](device/USBCameraSDK/ros2_ws/src/lunar_usb_rtabmap_bringup/README.md)
- [CREStereo 测试结果](maps/benchmarks/usb_crestereo_20260812/README.md)
- [CREStereo 极致模式测试](maps/benchmarks/usb_crestereo_max_performance_20260813/README.md)
- [网页控制说明](project/luxi-web-control/README.md)
- [D1 机器人控制与故障检查](docs/d1_robot_control.md)
