# luxi_hloc 测试报告

测试日期：2026-07-28  
平台：NVIDIA Jetson AGX Orin Developer Kit，aarch64，ROS2 Humble  
地图：`map012`，57 个 RTAB RGB-D 关键帧

## 结论

HLoc 粗定位、D435i 统一传感器输入和 Open3D ICP 位姿交接均已在实机通过。
默认运行时已切换为 Jetson CUDA，CPU 环境保留为显式回退。

## 构建和单元测试

```text
colcon build --packages-select luxi_hloc --symlink-install
结果：通过

colcon test --packages-select luxi_hloc
结果：7 tests，0 errors，0 failures，0 skipped
```

覆盖了坐标变换、像素/深度几何、地图导出读取和 PnP/RANSAC 准入逻辑。

## 地图构建

```text
输入：maps/rtab_maps/map012.db
数据库 SHA256：
46354e3a21e8ddca38f1217e4a5c123c244fb228d513d56edba58121300916d6

导出关键帧：57
跳过关键帧：0
SuperPoint 关键点：28,013
有效 RGB-D 米制地图点：23,412
有效比例：83.58%
```

模型版本：

```text
HLoc 1.4 / commit 80ccb7ee3bc048cb3a8ef221c5bc4d8ac25d5792
LightGlue 0.2 / commit edb2b838efb2ecfe3f88097c5fad9887d95aedad
```

## CPU 与 GPU 对比

测试方法：从地图首、中、尾均匀抽取 5 个查询帧，并排除查询帧自身，只允许匹配其他参考帧。
成功阈值为平移误差不超过 0.10 m、旋转误差不超过 10°。

| 运行时 | 成功 | 平均墙钟时间/次 | 平均进程 CPU 时间/次 |
|---|---:|---:|---:|
| CPU，PyTorch 2.4.0 | 5/5 | 4.039 s | 17.317 s |
| Orin CUDA，PyTorch 2.8.0 + CUDA 12.6 | 5/5 | 0.552 s | 0.675 s |

相对 CPU 基线：

```text
墙钟耗时下降：86.3%
进程 CPU 时间下降：96.1%
平均加速：约 7.3 倍
```

GPU 五个测试点的平移误差为 0.004～0.037 m，旋转误差为 0.27～5.73°。
首个查询含 CUDA 预热耗时 1.06 s，后续单次为 0.35～0.47 s。

环境检查：

```text
torch=2.8.0
torch CUDA build=12.6
cuda_available=true
device=Orin
```

## D435i 实机输入测试

```text
USB：8086:0b3a Intel RealSense D435i
驱动：RealSense ROS 4.58.2 / librealsense 2.58.2
连接：USB 3.2
RGB：约 30 Hz
对齐深度：约 30 Hz
CameraInfo：camera_color_optical_frame
TF：base_link -> camera_color_optical_frame 可查询
IMU：Madgwick 节点收到首个数据
```

统一适配层输入均存在：

```text
/sensors/rgbd/color/image_raw
/sensors/rgbd/depth/image_raw
/sensors/rgbd/color/camera_info
/sensors/imu/data_raw
/sensors/imu/data
```

## 在线 HLoc 测试

当前 D435i 真实画面无需点击即可完成定位：

```text
status：LOCALIZED
device：cuda
参考帧：node_000010.jpg
retrieval score：0.388
LightGlue matches：149
有效地图点：139
PnP inliers：116
inlier ratio：0.835
深度验证点：115
深度残差中位数：0.052 m
重投影 RMSE：2.093 px
单次耗时：0.508 s
```

`enable=false/true` 和 `relocalize` 服务均返回成功，`/luxi_hloc/coarse_pose`
正常发布 `map` 坐标系的 `PoseWithCovarianceStamped`。

## HLoc 到 ICP 联合测试

联合 launch 自动选择 `map012` HLoc 索引和 `map012` PLY：

```text
ICP 地图点：8,661
HLoc 粗位姿：已自动接收
ICP status：accepted
fitness：0.942～0.945
RMSE：0.0604 m
/luxi_location/pose：正常发布
```

测试结束后 HLoc、ICP、D435i 驱动和适配层进程均正常退出，无残留定位进程。

## 网页自动定位测试

网页地图 API 正确识别：

```text
map011：loadable=true，localizable=false（没有 HLoc 索引）
map012：loadable=true，localizable=true
HLoc 目录：maps/hloc_maps/map012
```

通过与页面按钮相同的 HTTP 接口完成：

```text
POST /api/navigation/load_map {"map_id":"map012"}：成功
POST /api/navigation/localize {"map_id":"map012"}：成功
状态：searching -> localized
最终 source：icp
fitness：0.9456
位姿：x=-0.151 m，y=-0.061 m，yaw=-56.55°
POST /api/navigation/stop：成功
```

网页启动的进程包括 GPU HLoc、Open3D ICP、OctoMap loader、A* planner 和 path
follower；停止后这些子进程均已清理。`map -> base_link` TF 在定位后可以查询。

## 尚未覆盖

本次验证证明软件链路和当前实机位置可用，但以下属于场景数据集验收，不应由 57 张建图关键帧
推断：

- 地图内大量独立起点的 Recall@5 和误定位率；
- 反向路线、昼夜光照、动态人群和遮挡；
- 地图外拒绝率；
- 长时间运行时的温度、功耗和降频。

正式导航前应按主文档的留出路线测试计划补齐上述数据。
