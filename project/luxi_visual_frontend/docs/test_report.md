# B 级视觉前端实机测试报告

测试日期：2026-08-03  
硬件：Jetson Orin、Intel RealSense D435i（序列号 `244622073011`）  
目标：以 SuperPoint + LightGlue 同时替换高频视觉里程计前端，并把学习型局部特征交给 RTAB-Map 后端。

## 集成接口

- A 级：`/luxi_visual_frontend/odom` 发布 RGB-D PnP 视觉里程计。
- B 级：`/luxi_visual_frontend/rgbd_image` 发布原始 RGB、深度、相机内参、SuperPoint
  关键点、逐关键点三维坐标和 256 维 `float32` 描述子。
- TF 所有权：视觉前端发布 `odom -> base_link`，RTAB-Map 发布 `map -> odom`。
- 全局定位：`luxi_hloc` 独立运行低频 NetVLAD 检索、SuperPoint + LightGlue 几何验证；
  HLoc 给出粗位姿后由 ICP 精配准。

## 模型与运行设备

模型复用工作区中已存在并校验的权重，不在 ROS 包中重复保存：

| 模型 | 路径 | SHA-256 |
| --- | --- | --- |
| SuperPoint | `3parts/hloc/third_party/SuperGluePretrainedNetwork/models/weights/superpoint_v1.pth` | `52b6708629640ca883673b5d5c097c4ddad37d8048b33f09c8ca0d69db12c40e` |
| LightGlue | `3parts/hloc_models/hub/checkpoints/superpoint_lightglue_v0-1_arxiv.pth` | `6ff7040d0a497fc6639337946d7538dae07428c18f77a067a0b5a960e7cc551a` |

环境检查确认 PyTorch 2.8.0、CUDA 可用，设备为 Orin。正式 launch 使用 `device=cuda`，
CUDA 不可用时会直接报错，不会静默回退到 CPU。

## 测试结果

### D435i 与学习型里程计

适配层四个输入均收到消息：RGB、对齐深度、彩色相机内参和 IMU；相机约 30 Hz。
一次代表性跟踪结果如下：

- SuperPoint 关键点：472；
- LightGlue 匹配：322；
- 有效深度匹配：220；
- PnP 内点：201，内点率 0.914；
- 图像网格覆盖率：1.0；
- 重投影 RMSE：1.22 px；
- 单次前端处理耗时：0.140 s。

### RTAB-Map B 级数据验证

使用学习型 launch 建立临时数据库后，从 RTAB-Map 节点 24 读取到：

- 关键点：253；
- 三维特征点：253；
- 描述子矩阵：`253 x 256`、`float32`。

这验证了后端数据库实际接收的是外部 SuperPoint 描述子，而不是重新提取的 ORB 描述子。
保存后的点云为有效稠密点云，临时数据库约 87 MB。

### 网页闭环

通过网页 API 完成以下实机流程：

1. 点击开始建图，成功启动 `rgbd_mapping_learned.launch.py`；
2. 点击停止建图，RTAB-Map 以退出码 0 保存 `map013.db`；
3. 选择 `map013`，网页自动生成并加载彩色 PLY 与 OctoMap；
4. 为 map013 生成 14 帧 HLoc 索引，索引构建设备为 CUDA；
5. 点击自动定位，HLoc 与 ICP 均成功，网页状态进入 `localized`；
6. 点击停止定位，所有定位与规划进程以退出码 0 结束。

HLoc 的代表性诊断：检索分数 0.555、LightGlue 匹配 249、有效地图点 138、PnP
内点 132、内点率 0.957、深度验证 130、深度中位残差 0.020 m、重投影 RMSE
1.258 px、单次耗时 0.354 s。后续 ICP fitness 为 0.996，RMSE 为 0.051 m。

旧 `map012` 在相机当前视角下没有误报：最佳参考帧只有 29 个匹配且 PnP 为 0
内点，因此保持搜索状态。模型、GPU、话题和网页进程均正常；这是当前视角与旧地图不重叠，
不是启动故障。

## 回归命令

```bash
colcon build --packages-select \
  luxi_visual_frontend luxi_rtab_map luxi_web_control --symlink-install
colcon test --packages-select luxi_visual_frontend luxi_rtab_map luxi_web_control
colcon test-result --test-result-base build --verbose
python3 -m flake8 project/luxi_visual_frontend \
  --exclude=__pycache__ --max-line-length=120
python3 -m pydocstyle project/luxi_visual_frontend/luxi_visual_frontend \
  project/luxi_visual_frontend/scripts project/luxi_visual_frontend/launch
git diff --check
```
