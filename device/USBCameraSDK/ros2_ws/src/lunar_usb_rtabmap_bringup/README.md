# Lunar USB RTAB-Map bringup

USB 双目完整建图入口，与 `lunar_d435i_rtabmap_bringup` 保持相同职责边界。相机、
深度和标定由 `usb_camera_driver` / `usb_camera_bringup` 提供；本包只组合设备适配、
视觉里程计和通用 RTAB-Map 后端。

```bash
# 默认：VPI OFA/PVA/VIC 深度 + Luxi 学习特征建图
ros2 launch lunar_usb_rtabmap_bringup usb_rtabmap.launch.py \
  use_imu:=true rviz:=true

# 显式回退：OpenCV CUDA StereoSGM + 经典特征
ros2 launch lunar_usb_rtabmap_bringup usb_rtabmap.launch.py \
  mode:=stable rviz:=true
```

两种模式都默认使用 `use_imu:=true`。启用时会先验证相机—IMU 时间同步、四元数和
全部加速度/陀螺轴；失败会在 RTAB-Map 启动前退出，避免坏 IMU 拉斜地图。序列号
`5A6C092260` 的 H30 已完成零偏恢复、断电持久性、200 Hz 独立验收和 10 Hz 外触发
动态建图验收。没有连接 H30 的诊断场景可显式传入 `use_imu:=false`。

同一时间只能运行一种模式。`vpi_learned` 是默认模式，使用 OFA/PVA/VIC
深度、RTAB F2M 局部 BA 里程计和 Luxi 外部学习描述子，并通过 24° 空间回环门限抑制
纯视觉地图倾斜。`stable` 保留为显式回退；两种模式都发布生产里程计
`/rtabmap/odom`，且都不会改变 Hik 或 D435i 的启动参数。
