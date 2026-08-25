# ZED X 接入与建图实测报告

测试日期：2026-08-24。测试平台为 Jetson AGX Orin、ZED Link Duo、ZED X
S/N `45570700`，软件为 ROS 2 Humble、ZED SDK 5.4.0、ZED ROS 2 Wrapper
5.4.x。修正后的配置为 `HD1200@30` 抓取、`960x600@10 Hz` 发布、
`NEURAL` 深度，深度置信度 `80`、纹理置信度 `100`。

## 复现命令

终端一启动设备适配层：

```bash
cd /home/lunar/project/lunar_slam
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch luxi_adapter sensor_bringup.launch.py hardware:=zedx
```

终端二启动 ZED X 专用原生 VIO 与 RTAB-Map：

```bash
cd /home/lunar/project/lunar_slam
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 launch luxi_rtab_map zedx_mapping.launch.py \
  rviz:=true rtabmap_viz:=false \
  database_path:=/tmp/zedx_luxi_mapping_test.db \
  new_map:=true load_saved_map:=false camera_wait_timeout:=30.0
```

若终端不是从本机桌面打开，先执行 `export DISPLAY=:0`；否则 RViz 会因连接不到 X
显示而退出。测试性能时建议改成 `rviz:=false`，避免把桌面渲染开销计入建图算法。

独立检查统一传感器接口：

```bash
ros2 run luxi_rtab_map sensor_sync_check --ros-args -p timeout_sec:=30.0
ros2 topic echo --once /zed/zed_node/odom
ros2 topic hz /zed/zed_node/odom --window 50
ros2 topic echo --once /rtabmap/info --field ref_id
```

## 2026-08-24 通用视觉前端基线

本节保留初次接入时使用学习视觉前端的历史基线。2026-08-25 的 `map042/map043`
对比证明该前端在 ZED X 运动帧上会产生错误航向跳变；正式配置已由文末的 ZED X
专用原生双目惯性里程计取代。

- 相机通过 GMSL2 打开，SDK 识别为 ZED X S/N `45570700`；RGB 和注册深度均为
  `960x600`，RGB 编码 `bgra8`，深度编码 `32FC1`。
- `/sensors/rgbd/rgbd_image` 连续 5 包全部有效，RGB/深度/CameraInfo 最大时间偏差
  `0.000 ms`，最近 IMU 时间偏差 `2.790 ms`。
- 学习视觉前端正常状态为 `TRACKING:ACCEPTED_PNP_DEPTH`，实测里程计约
  `4.4–4.9 Hz`。
- RTAB-Map 成功处理并保存地图；最终检查时 `ref_id=112`，关闭时数据库大小约
  `226 MB`。本次证据数据库位于
  `/tmp/zedx_luxi_mapping_test_final_20260824.db`。
- ZED SDK 内部 positional tracking、位姿话题、动态 TF 与原生点云均已关闭；
  `luxi_visual_frontend` 独占 `odom -> base_link`，RTAB-Map 独占 `map -> odom`。

完整建图运行时，采样到的主要进程资源为：学习视觉前端约
`94% CPU / 2.06 GiB RSS`，ZED 节点约 `34% CPU / 0.77 GiB RSS`，RTAB-Map 约
`24% CPU / 0.60 GiB RSS`。系统 RAM 约 `8.1 GiB`；GPU 负载随推理阶段在
`0–99%` 间波动，短时高峰
频繁，`VDD_GPU_SOC` 约 `8.3–9.0 W`，GPU 温度约 `49°C`。这些是当前静态场景、
50 W 模式下的进程累计值/短时样本，不是硬件额定值。

## 2026-08-24 重影与回环修正

重影不是通过放宽 `RGBD/OptimizeMaxError` 处理的。旧数据库 `map041.db` 中实际已有
19 条 RTAB-Map 全局回环（link type 2）；终端反复显示的是图优化正确拒绝的不一致
候选。根因在候选之前的输入：

- 无磁力计的 Madgwick 航向曾覆盖视觉 PnP 航向，静止 8 秒可累计约 `0.34°`；现在
  PnP 保持普通帧的旋转权，IMU 只做重力/一致性检查和短时丢帧桥接。
- 平面或运动模糊场景中，PnP 与 IMU 旋转相差超过 `3°` 的姿态现在以
  `IMU_ROTATION_MISMATCH` 拒绝，连续坏帧由受角速度约束的 IMU 重建关键帧，错误
  姿态不会进入 RTAB-Map 图。
- `NEURAL_LIGHT` 在静止场景的相邻深度帧误差为 P95 `0.201 m`、P99 `0.748 m`；
  改为 `NEURAL` 和 confidence `80` 后，有效深度为 `71.2%`，P95 `0.0195 m`、
  P99 `0.0349 m`。后续性能隔离实验将发布频率由 `15 Hz` 降至 `10 Hz`；前端仍
  只处理 `5 Hz`、RTAB-Map 仍只入图 `1 Hz`，因此不会降低当前位姿和建图更新率。
  这消除了点云自身的厚重抖动并减少图像搬运开销。
- 修正后稳定阶段连续 25 秒的 104 个里程计样本仅变化 `3.5 mm / 0.115°`；前端
  诊断为 537 个关键点、468 个匹配、371 个 PnP/深度内点，内点率 `95.1%`。

不要在原来的 `map041.db` 上继续建图；其错误邻接约束已经写入数据库。必须用
`new_map:=true` 和新的数据库路径重新跑闭环路线。

## 原始图像质量门禁

本轮末尾 GMSL 原始 RGB 突然出现全幅椒盐噪声；原始灰度横向相邻像素差中位数为
`55`，SuperPoint 仅剩 `36` 个有效关键点。前端因此进入 `KEYPOINTS_LOW`，RTAB-Map
不再收到里程计/关键帧，避免坏帧继续叠图。重启 ROS 节点、重启 `zed_x_daemon`、
以及临时降为 HD1080 均未恢复；ZED Diagnostic 则确认 SDK 5.4.0、CUDA 12.6、
Duo 1.4.3 驱动和相机枚举正常。这种状态需整机断电后重新插紧 Fakra 两端再开机，
恢复清晰原始 RGB 后才能进行最终同路线闭环 A/B。

2026-08-24 18:39 再次冷启动 ROS 设备链路后复测，原始 RGB 仍为全幅彩色位错误：
30 帧灰度横向相邻像素差中位数为 `52`、P95 为 `144`。同批深度有效率虽然平均
为 `82.2%`，相邻帧误差却达到 P50 `0.030 m`、P95 `0.134 m`，不满足建图输入
门槛。内核日志同时记录到 `3190000.i2c` 和 `31c0000.i2c` 大量
`I2C transfer timed out`。因此本轮没有启动 RTAB-Map；强行放宽闭环或深度阈值
只会掩盖 GMSL/Fakra 链路故障并重新制造重影。测试结束后 ZED、适配器、前端、
RTAB-Map 和 RViz 进程均已退出。

## 2026-08-25 10 Hz 性能隔离实验

物理链路恢复后，40 帧原始图像检查清晰，RGB 灰度横向相邻像素差中位数为 `1`、
P95 为 `18`；深度有效率平均为 `82.9%`，相邻深度帧误差为 P50 `2.9 mm`、
P95 `43.5 mm`。ZED X 发布频率实测 `9.62 Hz`，消息到探针的延迟为 P50
`89.2 ms`、P95 `95.3 ms`。

全链路静止测试使用全新数据库 `/tmp/zedx_10hz_static_20260825.db`。前端实际处理
`4.51 Hz`，单帧总耗时 P50 `158.5 ms`、P95 `180.7 ms`，低于 5 Hz 对应的
`200 ms` 预算；连续 20 秒的位姿范围仅为 x/y/z `2.03/1.80/2.10 mm`、航向
`0.066°`，79 帧全部为 `ACCEPTED_PNP_DEPTH`，匹配内点率中位数为 `91.4%`。
进程累计占用约为前端 `95% CPU`、ZED `32% CPU`、RTAB-Map `26% CPU`；系统
总内存使用约 `8.5 GiB / 61 GiB`。因此没有发现 CPU、内存或前端队列饱和，降至
10 Hz 可以保留作为运行余量，但它不是运动建图重影的根因。运动闭环仍须用新数据库
按相同路线复测，重点检查实际安装外参和转弯后的视觉里程计回中误差。

该静止测试数据库在约 4.5 分钟内接收 232 帧并增长到 `578 MiB`，其中 231 帧被
RTAB 标记为低运动/未连接节点。正式配置设置 `Mem/NotLinkedNodesKept=false`、
`Mem/IntermediateNodeDataKept=false`，避免无地图贡献的重复帧长期占用数据库；同时
把 32FC1 深度压缩格式明确设为 `.png`，消除 RVL 不兼容回退。第二轮静止复测中，
RTAB 处理序号增长到 112，但关闭后数据库只保留 1 个有效图节点和 1 份 RGB-D 数据，
大小为 `2.98 MiB`，证明静止重复帧不再进入持久地图。

## 已知提示与上线前事项

- RTAB-Map 启动早期可能先报告 5 秒未收到数据，因为 SuperPoint/LightGlue 正在加载；
  模型加载后状态进入 `TRACKING:ACCEPTED_PNP_DEPTH` 即属正常。
- RTAB-Map 会提示 32 位浮点深度与 `.rvl` 不兼容并自动改用 `.png`，不影响本次
  建图；如需消除提示，可另行把 `Mem/DepthCompressionFormat` 固定为 `.png`。
- 当前 `base_link -> zed_camera_link` 是台架测试用单位变换。上机器人前必须实测
  `camera_x/y/z` 和 `camera_roll/pitch/yaw`，否则不能据此评价轨迹精度或导航效果。
- 该测试证明链路、同步、TF 所有权和地图落盘正常；绝对精度仍需相同路线、相同安装
  位姿下用真值或 D455 做定量 A/B 测试。

## 2026-08-25 map042/map043 运动失败分析与 ZED X 专用修复

数据库和日志的定量对比如下：

| 项目 | map042 | map043 |
|---|---:|---:|
| 有效图节点 | 9 | 37 |
| 记录时长 | 9.5 s | 53.8 s |
| 保留轨迹长度 | 0.277 m | 1.675 m |
| 非邻接 type-2 link 行 | 0 | 14（7 对双向局部空间约束） |
| 最终全局候选 | 无，路线过短 | `41 <-> 21` 被正确拒绝 |

`map043` 的拒绝信息为图角误差比 `1.049458`，最大问题边 `3 -> 21` 的绝对角误差
`4.266309°`。更早的相邻里程计已经出现节点 `22 -> 23` 在约 1.1 秒内跳转
`+61.88°`，紧接着 `23 -> 24` 在约 1.0 秒内跳转 `-115.12°`；对应原始 RGB 仅是
小幅平移和轻微倾斜，不支持这种真实旋转。结论是运动时通用学习特征/PnP 前端发生
姿态翻转，错误先写入邻接图，RTAB-Map 的回环检查随后才拒绝矛盾候选。单帧 RGB/深度
清晰和降低到 10 Hz 都不能修复已错误的位姿；也不应放宽
`RGBD/OptimizeMaxError=1.0` 来接受它。

修复只作用于 ZED X：

- ZED SDK positional tracking 使用 `AUTO`、IMU fusion、完整 6DoF 和重力对齐，发布
  `/zed/zed_node/odom` 及 `odom -> zed_camera_link`；实测约 `30 Hz`。
- ZED area memory、SDK 回环重置和 `map -> odom` 均关闭，RTAB-Map 仍是唯一全局
  回环所有者；RTAB 不再重复融合同一份 IMU。
- 网页仅在 profile 为 `zedx` 时启动 `zedx_mapping.launch.py`。D455、D435i、HIK
  仍选择 `rgbd_mapping_learned.launch.py`，对应配置和阈值未修改。
- RTAB 仍以 1 Hz 从 RGB-D 独立提取 SIFT 进行地点识别，保留
  `RGBD/OptimizeMaxError=1.0` 和最少 30 内点，不以放宽阈值掩盖坏约束。

正式工作区构建与回归为 `365 tests, 0 errors, 0 failures, 9 skipped`。实机静止验证中，
网页正确报告 `sensor_profile=zedx`、`launch_file=zedx_mapping.launch.py`，机器人控制保持
`offline`；RTAB 连续处理到 `ref_id=19`，进程中没有 `luxi_visual_frontend`。12 秒内
采集 266 个原生 odom 样本，x/y/z 范围分别为 `0.351/0.285/0.020 mm`，航向范围
`0.0105°`。正式启动时 TF 缓存预热丢弃了最初 4 帧，之后没有持续 TF、图优化或回环
告警。

相机保持静止时 RTAB 依据运动阈值只保留 1 个图节点，因此该轮静态测试不具备可导出
的运动轨迹，自动过滤按预期报告“无 odometry poses”；测试产物已移出正式地图目录。
由于“无机器人建图”模式明确禁止启动底盘，本轮不能代替同路线运动闭环验证。下一次
应手动移动相机或恢复获准的机器人移动，用全新数据库走完往返闭环；不得继续使用已含
错误邻接约束的 `map043`。

## 2026-08-25 map044 三维重影根因与 6DoF 对比修复

`map044` 共 33 个节点，记录约 39.67 秒，保留轨迹约 1.134 m，有 6 条局部空间约束但
没有全局回环。数据库中每一个节点的位姿都严格为 `z=0、qx=0、qy=0`。与此同时，保存
的 RGB 帧明显出现相机横滚、俯仰，以及桌面/天花板方向的观察变化。这说明当时不是只
允许“左右转动”，而是把整个轨迹限制为平面 x/y/yaw；真实的 z/roll/pitch 被丢弃后，
每帧深度点云仍按错误的平面姿态累加，因而单帧完好而全局点云分层、重影。

原设计适用于固定在水平轮式底盘上的相机：平面约束能抑制地面行驶时的微小高度、横滚
和俯仰漂移。当前测试却直接移动和倾斜相机，相机运动模型已不同，所以必须移除这两层
互相独立的约束：

- `zedx_sensor_bringup.yaml`：`zedx_positional_tracking_two_d_mode: false`；
- `zedx_mapping.launch.py`：默认 `planar_motion:=false`，从而令
  `Reg/Force3DoF=false`、`RGBD/ForceOdom3DoF=false`；
- `web_control.yaml`：仅新增 `mapping_zedx_planar_motion: false`；通用
  `mapping_planar_motion: true` 未变，因此 D455、D435i、HIK 不受影响。

修改后的实机静态对比结果：ZED 启动日志明确显示 `2D mode: FALSE`，实时 odom 已观察到
非零 z、四元数 x/y（样例约 `z=-0.000191 m、qx=0.000670、qy=-0.000748`）；网页启动的
进程明确包含 `planar_motion:=false`、`--Reg/Force3DoF false` 和
`--RGBD/ForceOdom3DoF false`。SDK Spatial Mapping 基线能输出约 1 Hz 的
`/zed/zed_node/mapping/fused_cloud`，静止帧约 10643 点。Luxi 静止测试只留下 1 个节点，
这是运动阈值的正常结果，不足以判断真实三维轨迹和回环质量。

回归构建与测试结果为 `365 tests, 0 errors, 0 failures, 9 skipped`。下一轮有效 A/B 必须
在相同场景中上下移动并做 roll/pitch/yaw 后返回起点，分别观察 odom 的六自由度变化、
地图墙面/桌面厚度和 RTAB 全局回环；不得用静止的单节点数据库宣称三维建图已通过。
