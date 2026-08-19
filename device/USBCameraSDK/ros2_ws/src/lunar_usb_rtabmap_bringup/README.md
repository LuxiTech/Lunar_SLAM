# lunar_usb_rtabmap_bringup

USB 双目到 Luxi/RTAB-Map 的一键启动包。各档使用独立 launch，不接受旧的
`mode` 或 `depth_backend` 参数。

| Launch | 用途 |
|---|---|
| `usb_fast_foundation_stereo_rtabmap.launch.py` | 高质量主链路：Fast-FoundationStereo TensorRT + H30/Luxi 里程计 |
| `usb_crestereo_rtabmap.launch.py` | 兼容次选：原生 640×360 CREStereo + H30/Luxi 里程计 |
| `usb_crestereo_max_performance_rtabmap.launch.py` | 可选高帧率档：320×180 模型、10 Hz、960×540 输出 |
| `usb_rtabmap.launch.py` | 备用链路：VPI + Luxi + RTAB F2M |
| `usb_fast_foundation_stereo_saved_map_navigation.launch.py` | Foundation 主深度的已建地图定位/导航 |
| `usb_crestereo_saved_map_navigation.launch.py` | CRE 兼容深度的已建地图定位/导航 |

## 使用

```bash
cd /home/nvidia/Desktop/lunar_-slam
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Fast-FoundationStereo + RViz：

```bash
python3 scripts/install_fast_foundation_stereo_engine.py \
  --engine /absolute/path/to/model.engine \
  --output-name ffs_320x192_i4_d96.engine
ros2 launch lunar_usb_rtabmap_bringup \
  usb_fast_foundation_stereo_rtabmap.launch.py \
  rviz:=true use_imu:=true new_map:=true
```

默认 engine 位于
`~/.cache/luxi/fast_foundation_stereo/ffs_320x192_i4_d96.engine`。缺失时 launch
会在占用相机前退出并给出安装命令；也可显式传入
`model_path:=/absolute/path/to/model.engine`。左右一致性使用同一份 engine，不会加载
第二份模型。

已建地图的自动定位和导航默认继续使用同一份 Foundation engine：

```bash
ros2 launch lunar_usb_rtabmap_bringup \
  usb_fast_foundation_stereo_saved_map_navigation.launch.py \
  database_path:=/absolute/path/map.db \
  octomap_path:=/absolute/path/map.bt \
  cloud_path:=/absolute/path/map_cloud.ply \
  hloc_map_directory:=/absolute/path/hloc_map
```

需要兼容旧部署时，显式改用
`usb_crestereo_saved_map_navigation.launch.py`；不会静默从 Foundation 退回 CRE。

> **许可限制：** 官方 Fast-FoundationStereo checkpoint 及由它转换的 TensorRT
> engine 在本项目中仅用于研究/评估和非商业用途。这不构成商业授权；分发、部署或
> 商用前必须单独核对并遵守 NVIDIA 上游完整许可。

CREStereo 兼容档 + RViz：

```bash
ros2 launch lunar_usb_rtabmap_bringup usb_crestereo_rtabmap.launch.py \
  rviz:=true use_imu:=true new_map:=true
```

CREStereo 极致档 + RViz：

```bash
ros2 launch lunar_usb_rtabmap_bringup \
  usb_crestereo_max_performance_rtabmap.launch.py \
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

D1 当前安装外参以机身旋转中心为 `base_link`：双目中点约向前 0.20 m、向上
0.20 m，双目中心线与机身中心轴重合。由于建图使用左目光心坐标系，按 89.963 mm
标定基线换算后为 `camera_x=0.20`、`camera_y=0.044982`、`camera_z=0.20` m。
相机水平朝前，旋转四元数保持 `(-0.5, 0.5, -0.5, 0.5)`。

## CREStereo 配置

- 主模型：`crestereo_combined_iter2_360x640.onnx`（320×180 初始预测 + 640×360 精化）
- 置信度：320×180 FP16 反向模型做左右一致性校验，拒绝反光和遮挡错误
- 深度输出：单独运行约 2.7 Hz，完整建图约 2.3–2.5 Hz，始终处理最新双目帧
- 里程计：SuperPoint + LightGlue 直接发布位姿和同时间戳 RGB-D
- 0.4–4 m：稠密地图并参与位姿估计
- 4–6 m：4×4 稀疏采样
- 6–10 m：8×8 稀疏采样

模型链路不再启动第二套 RTAB F2M 视觉里程计，避免跟踪重置造成重复或镜像地图。VPI
链路保持原有 F2M 配置，适合资源紧张或近场优先的场景。

H30 驱动支持断线和静默自动重开。只有在明确进行无 IMU 诊断时才使用
`use_imu:=false`；正常建图应保持默认值，并确认六轴、时间戳和四元数健康。

极致档保留 1080p 相机输入，发布 960×540 RGB-D，使用 640 个特征点、匹配尺寸对应
的 SuperPoint TensorRT FP16 引擎和 LightGlue CUDA Graph。地图写入提高到 2 Hz；
健康守护在 85 °C 告警、92 °C 或硬件节流时停止整条 launch。TensorRT 引擎与本机
JetPack/TensorRT 版本绑定，缺少引擎时会自动回退 PyTorch，帧率也会随之降低。

测试结果见 [CREStereo 基准记录](../../../../../maps/benchmarks/usb_crestereo_20260812/README.md)。
极致档见 [CREStereo MAX 基准](../../../../../maps/benchmarks/usb_crestereo_max_performance_20260813/README.md)。
