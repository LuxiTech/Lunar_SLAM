# ZED Link Duo 配置状态

更新时间：2026-08-24（UTC）

## 已完成

- 宿主机确认为 NVIDIA Jetson AGX Orin Developer Kit。
- 系统确认为 Ubuntu 22.04、L4T 36.5.0、内核 `5.15.185-tegra`。
- 已安装 `stereolabs-zedlink-duo` 1.4.3，包目标为 L4T 36.5.0 / ARM64。
- 模块 `sl_max96712`、`sl_max9295`、`sl_zedx`、`bmi088` 和 `bmi_spsc`
  的 vermagic 均与当前内核一致。
- `/boot/extlinux/extlinux.conf` 默认项已切换为 `Stereolabs`，使用
  `tegra234-p3737-camera-zedlink-duo-sl-overlay.dtbo`。
- 原启动配置已备份为 `/boot/extlinux/extlinux.conf.sl-backup`。
- `driver_zed_loader`、`zed_x_daemon` 和 `IMU_Daemon` 已设为开机启用。
- 已为 `nvargus-daemon` 设置 `enableCamInfiniteTimeout=1`。
- 宿主用户 `luxi-jetson` 已加入 `imu` 组。
- 已安装 ZED SDK 5.4.0；C++ 库、工具和 Python `pyzed` 均通过检查。
- 已在 `lunar_slam` 容器安装 CUDA 12.6 编译工具、TensorRT 10.3.0.30、
  ZED SDK 5.4.0，并将容器用户 `lunar` 加入 `imu` 和 `zed` 组。
- ROS 2 Humble 工作区 6 个目标包全部编译通过，ZED ROS 2 Wrapper 为 5.4.1。
- 已生成并缓存 TensorRT `neural_depth_light_5` 优化模型。
- `dpkg --audit` 无异常。

## 重启后验收结果

- `driver_zed_loader` 执行成功；`zed_x_daemon`、`IMU_Daemon` 和
  `nvargus-daemon` 均正常运行。
- 内核识别到一台 ZED X，序列号 `45570700`、相机固件 `2001`；创建
  `/dev/video0`、`/dev/video1` 和 `/dev/spsc_bmi0`。
- V4L2 支持 HD1200、HD1080 和 960x600 等模式。
- ZED SDK 实际取流测试通过：HD1200@30、60 帧、实测 30.00 FPS、时间戳严格递增、
  左图 `1920x1200 BGRA8`、深度有效像素约 86%、IMU `SUCCESS`。
- ROS 2 以 `camera_model:=zedx`、HD1200@30、NEURAL LIGHT 启动成功并干净退出。
- ROS 健康状态无图像、光照、深度或 IMU 告警。
- ROS 订阅实测：RGB 约 27–28 Hz（Python CLI 对 9.2 MB BGRA 帧进行反序列化）、
  Depth 约 28.8–29.4 Hz、IMU 约 100 Hz、点云约 10 Hz。
- ROS 消息格式：RGB `1920x1200/bgra8`，Depth `1920x1200/32FC1`。

宿主机验证命令：

```bash
sudo /usr/local/sbin/verify_zedx
```

## 容器说明

当前开发容器的 Docker restart policy 为 `no`，宿主机重启后仍需手动启动
`lunar_slam`。该容器最初只映射了 `/tmp/.X11-unix`，未按 Stereolabs 的 ZED X
Docker 要求映射完整 `/tmp`；`scripts/prepare_container_runtime.sh` 会在每次启动相机前
刷新宿主 Argus/IMU socket，以兼容当前容器。以后重建容器时应直接使用 README 中的
官方 GMSL2 volume 配置。
