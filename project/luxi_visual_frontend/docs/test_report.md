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

### HIK TensorRT FP16 加速与质量门禁（2026-08-06）

HIK `1024x750` 图像保持原分辨率输入，SuperPoint 按既有 `resize_max=800`
处理为 `800x586`。TensorRT 只替换 SuperPoint 稠密卷积，网络 I/O、NMS、
描述子采样、几何验证和 RTAB 的 256 维 `float32` 描述子接口保持不变；
LightGlue 单独启用 PyTorch AMP FP16。

| 路径 | SuperPoint 耗时 | 正确匹配总量 | 内点率 | RMSE |
| --- | ---: | ---: | ---: | ---: |
| TensorRT FP32 | 101.0 ms | 1334 | 0.9474 | 1.111 px |
| TensorRT FP16 | 40.8 ms | 1332 | 0.9494 | 1.116 px |
| TensorRT INT8（拒绝） | 30.2 ms | 1221 | 0.8151 | 1.548 px |

- FP16 相对 FP32 的 SuperPoint 加速为 2.48 倍，正确匹配保留 99.85%；
- LightGlue AMP 从 43.34 ms 降至 38.38 ms，同一帧匹配集合完全一致；
- 32 帧 HIK 熵校准 INT8 的关键点坐标重合仅 9.47%、匹配集合 Jaccard 3.3%，
  即使更快也未通过质量门禁，不进入生产配置；
- 输入长边 720/640 分别损失 16.7%/29.8% 正确匹配，因此保留 800；
- NMS 半径从 3 调至 4，关键点减少 10.7%，保留 91.4% 正确匹配总量，
  内点率和 RMSE 略有改善；最大关键点限制从 2048 降至 1024；
- Nsight Systems 显示 TensorRT FP16 卷积族约占前端 GPU kernel 时间 47%，原版
  SuperPoint NMS 的五次全分辨率 `max_pool` 单项占 18.8%；LightGlue 的主要问题
  是大量小 kernel 的启动与同步，而不是 FlashAttention 算术本身；
- CUDA Graph 仅覆盖固定形状 TensorRT 和 LightGlue `<=512` 点的前三层，较大或
  困难帧自动回到原生自适应路径。8 次固定输入测试中普通 kernel launch 从
  4450 次降到 3718 次；8 张图的 SuperPoint 关键点、分数、描述子以及小点集的
  LightGlue 匹配索引与优化前逐元素一致；
- TensorRT 直接绑定 Torch CUDA 指针并复用输出缓存，稠密输出不回传 CPU；
  仅保留一次必要的灰度图 H2D 和小型关键点/分数 D2H。

10 Hz 上限压测时前端曾达到 8.53 Hz，但会把 HIK 与前端的合计 GPU 平均值推到
68.8%，且 30.2% 的 200 ms 采样窗达到 90% 以上。生产配置改为最新帧 5 Hz，
RTAB 特征 2 Hz、后端检测 1 Hz；完整 SLAM 实测 GPU 平均 57.4%、中位 49%，
前端 CPU 46.5%、RTAB CPU 27.0%。普通帧耗时约 79--91 ms，困难帧由 LightGlue
自适应运行到第 5--6 层时约 111--121 ms，仍小于 200 ms 跟踪周期。建图前同步
检查通过：RGB/Depth 时间差 0 ms，相机/IMU 最近时间差 0.866 ms。RTAB 外部
odom、RGBD 特征均为唯一发布/订阅；本次临时库正常停止后为 43 MiB，含 47 个
节点、87 条约束和 47 条数据记录。

TensorRT 引擎只与构建它的 Jetson GPU 和 TensorRT 版本兼容。`auto` 模式只在
输入尺寸、精度和引擎完全匹配时启用，否则保持原 PyTorch 路径；诊断消息会明确
报告当前 SuperPoint 与 LightGlue 后端。

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
