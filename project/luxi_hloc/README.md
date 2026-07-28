# luxi_hloc

`luxi_hloc` 使用 RTAB-Map 已保存的 RGB-D 关键帧完成无需人工点击的全局粗定位：

```text
当前 RGB
  -> NetVLAD 全局检索 Top-K
  -> SuperPoint + LightGlue 局部匹配
  -> 地图关键点的 RGB-D 米制 3D 坐标
  -> OpenCV solvePnPRansac
  -> 当前深度几何验证
  -> /luxi_hloc/coarse_pose
  -> luxi_location Open3D ICP
  -> /luxi_location/pose
```

建图仍由 RTAB-Map 负责。HLoc 只负责从未知初始位置获得全局粗位姿，ICP 负责点云精配准。

## 当前实现

- RTAB 数据库导出器：`src/rtab_hloc_exporter.cpp`
- HLoc 推理与候选检索：`luxi_hloc/inference.py`
- OpenCV PnP/RANSAC 和深度验证：`luxi_hloc/pose_estimator.py`
- ROS2 粗定位节点：`luxi_hloc/node.py`
- 单独粗定位 launch：`launch/hloc_localization.launch.py`
- HLoc + ICP 联合 launch：`launch/hloc_icp_localization.launch.py`
- 默认配置：`config/hloc_localization.yaml`

当前地图 `map012` 的 HLoc 索引位于：

```text
/home/lunar/project/lunar_slam/maps/hloc_maps/map012
```

它包含 57 个 RTAB 关键帧、28,013 个 SuperPoint 关键点和 23,412 个有效米制地图点。

## 完整启动

终端一只启动一套硬件 profile：

```bash
cd /home/lunar/project/lunar_slam
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch luxi_adapter sensor_bringup.launch.py
```

终端二启动 HLoc 粗定位和 ICP 精定位：

```bash
cd /home/lunar/project/lunar_slam
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch luxi_hloc hloc_icp_localization.launch.py
```

launch 默认自动选择最新的 `mapNNN` HLoc 索引和最新同编号点云。也可明确指定：

```bash
ros2 launch luxi_hloc hloc_icp_localization.launch.py \
  map_directory:=/home/lunar/project/lunar_slam/maps/hloc_maps/map012 \
  cloud_path:=/home/lunar/project/lunar_slam/maps/octo_maps/map012_octomap/luxi_rtab_map_20260727_173205_cloud.ply \
  device:=cuda
```

仅运行粗定位：

```bash
ros2 launch luxi_hloc hloc_localization.launch.py device:=cuda
```

## ROS2 接口

输入：

```text
/sensors/rgbd/color/image_raw
/sensors/rgbd/depth/image_raw
/sensors/rgbd/color/camera_info
TF: base_link -> 当前彩色相机 optical frame
```

HLoc 输出：

```text
/luxi_hloc/coarse_pose
/luxi_hloc/status
/luxi_hloc/diagnostics
```

服务：

```bash
ros2 service call /luxi_hloc_localizer/enable std_srvs/srv/SetBool '{data: false}'
ros2 service call /luxi_hloc_localizer/enable std_srvs/srv/SetBool '{data: true}'
ros2 service call /luxi_hloc_localizer/relocalize std_srvs/srv/Trigger '{}'
```

联合 ICP 输出：

```text
/luxi_location/pose
/luxi_location/fitness
/luxi_location/status
```

网页中的“自动定位”也已切换到同一 GPU HLoc + ICP launch。选择 `map012` 并加载
地图图层后即可点击；没有 `maps/hloc_maps/mapNNN/metadata.yaml` 的旧地图仍可显示，
但按钮会保持禁用，避免误启动旧 RTAB-Map 粗定位。

观察运行结果：

```bash
ros2 topic echo --once /luxi_hloc/status
ros2 topic echo --full-length --once /luxi_hloc/diagnostics
ros2 topic echo --once /luxi_location/status
ros2 topic echo --once /luxi_location/pose
```

## 从新 RTAB 地图构建索引

假设新地图为 `map013.db`：

```bash
cd /home/lunar/project/lunar_slam
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run luxi_hloc rtab_hloc_exporter \
  --database maps/rtab_maps/map013.db \
  --output maps/hloc_maps/map013

LUXI_HLOC_RUNTIME=gpu \
install/luxi_hloc/lib/luxi_hloc/build_reference_model.py \
  --map-directory maps/hloc_maps/map013 \
  --source-database maps/rtab_maps/map013.db \
  --resize-max 640 \
  --max-keypoints 1024
```

生成物包括参考 RGB、参考深度、相机内参、`T_map_camera`、NetVLAD 描述子、
SuperPoint 特征和每个特征对应的米制地图点。工程使用 OpenCV
`solvePnPRansac`，在线定位不依赖 `pycolmap`；HLoc 初始化时出现
“pycolmap is not installed”只是第三方包的可选功能提示。

## GPU 与 CPU 回退

正式 launch 的 `device` 默认值是 `cuda`，因此不会无提示地把网络推理退回 CPU。
依赖选择默认 `LUXI_HLOC_RUNTIME=auto`：若 `3parts/hloc_gpu_python` 存在，则优先加载
Jetson CUDA PyTorch；否则加载 `3parts/hloc_python`，但正式 launch 会明确报告 CUDA
不可用并退出。

强制 GPU：

```bash
LUXI_HLOC_RUNTIME=gpu \
install/luxi_hloc/lib/luxi_hloc/check_hloc_environment.py --require-cuda
```

强制 CPU 回退：

```bash
LUXI_HLOC_RUNTIME=cpu \
ros2 launch luxi_hloc hloc_localization.launch.py device:=cpu
```

本机 GPU 基线是 Jetson Orin、CUDA 12.6、PyTorch 2.8.0。GPU 推理仍复用 CPU
依赖目录中的 NumPy、HDF5、Kornia 等纯 Python/通用依赖；两个 Torch 目录相互隔离。

## 构建与测试

```bash
cd /home/lunar/project/lunar_slam
source /opt/ros/humble/setup.bash
colcon build --packages-select luxi_hloc --symlink-install
colcon test --packages-select luxi_hloc --event-handlers console_direct+
colcon test-result --test-result-base build/luxi_hloc --verbose
```

真实地图离线测试默认排除查询帧自身，验证邻近参考帧能否恢复位姿：

```bash
project/luxi_hloc/scripts/offline_localization_test.py \
  --map-directory maps/hloc_maps/map012 \
  --config project/luxi_hloc/config/hloc_localization.yaml \
  --limit 5
```

本机验收结果见 `docs/test_report.md`。

## 已知边界

- `map012` 只有 57 个关键帧，当前实机位置已通过，但不代表所有视角和光照都已覆盖。
- HLoc 是低频全局粗定位；连续 30 Hz 运动仍应由 RGB-D/轮式里程计维护。
- 当前 ROS 节点每秒尝试一次。定位稳定后应由上层状态机降低查询频率。
- 地图外、严重遮挡、地图未采集的反向视角应该保持未定位，不能通过无条件降低阈值换取输出。
- 当前阶段使用 PyTorch CUDA；只有长期性能数据表明仍有必要时再进行 FP16/ONNX/TensorRT。
