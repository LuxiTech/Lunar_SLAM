# lunar_usb_rtabmap_bringup

USB 双目到 Luxi/RTAB-Map 的一键启动包。两条链路使用独立 launch，不接受旧的
`mode` 或 `depth_backend` 参数。

| Launch | 用途 |
|---|---|
| `usb_crestereo_rtabmap.launch.py` | 默认主链路：CREStereo + Luxi 直接里程计 |
| `usb_rtabmap.launch.py` | 备用链路：VPI + Luxi + RTAB F2M |

## 使用

```bash
cd /home/nvidia/Desktop/lunar_-slam
source /opt/ros/humble/setup.bash
source install/setup.bash
```

CREStereo + RViz：

```bash
ros2 launch lunar_usb_rtabmap_bringup usb_crestereo_rtabmap.launch.py \
  rviz:=true use_imu:=true new_map:=true
```

VPI + RViz：

```bash
ros2 launch lunar_usb_rtabmap_bringup usb_rtabmap.launch.py \
  rviz:=true use_imu:=true new_map:=true
```

常用参数：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `rviz` | `true` | 启动 RViz |
| `rtabmap_viz` | `false` | 启动 rtabmap_viz |
| `new_map` | `true` | 新建数据库 |
| `database_path` | 自动 | 指定 `.db` 文件 |
| `use_imu` | `true` | 默认使用 H30；无数据或无效轴会阻止里程计启动 |
| `planar_mode` | `false` | 仅地面机器人需要时启用 3DoF 约束 |

## CREStereo 配置

- 模型：`crestereo_init_iter2_180x320_fp16.onnx`
- 深度输出：约 6 Hz，始终处理最新双目帧
- 里程计：SuperPoint + LightGlue 直接发布位姿和同时间戳 RGB-D
- 0.4–4 m：稠密地图并参与位姿估计
- 4–6 m：4×4 稀疏采样
- 6–10 m：8×8 稀疏采样

模型链路不再启动第二套 RTAB F2M 视觉里程计，避免跟踪重置造成重复或镜像地图。VPI
链路保持原有 F2M 配置，适合资源紧张或近场优先的场景。

H30 驱动支持断线和静默自动重开。只有在明确进行无 IMU 诊断时才使用
`use_imu:=false`；正常建图应保持默认值，并确认六轴、时间戳和四元数健康。

测试结果见 [CREStereo 基准记录](../../../../../maps/benchmarks/usb_crestereo_20260812/README.md)。
