# Stereolabs ZED X — Jetson AGX Orin / ROS 2 Humble

该目录固定用于当前主机的软件组合：

- Jetson AGX Orin Developer Kit，ARM64
- Ubuntu 22.04 / JetPack 6.2.2 / L4T 36.5.0 / CUDA 12.6
- TensorRT 10.3（JetPack 6.2.2 / L4T 36.5 官方仓库版本）
- ZED SDK 5.4.0
- ZED ROS 2 Wrapper 5.4.1
- ROS 2 Humble
- ZED X 启动模型 `camera_model:=zedx`

## 已下载内容

```text
installers/
  ZED_SDK_Tegra_L4T36.5_v5.4.0.zstd.run
  stereolabs-zedlink-mono_1.4.3-SL-MAX9296-L4T36.5.0_arm64.deb
  stereolabs-zedlink-duo_1.4.3-LI-MAX96712-L4T36.5.0_arm64.deb
  stereolabs-zedlink-quad_1.4.3-SL-MAX96712-L4T36.5.0_arm64.deb
ros2_ws/src/
  zed-ros2-wrapper/       v5.4.1
  zed-ros2-examples/      v5.4.1
  zed-ros2-interfaces/    5.3.0
  zed-ros2-description/   0.1.5
```

`VERSIONS.lock` 记录了固定版本、Git commit 和官方下载地址，`SHA256SUMS`
可用于检查下载文件。ROS 2 Humble 已安装在
本机 `/opt/ros/humble`，因此没有在项目目录中重复复制系统级 ROS 安装。

## 安装与启动

先确认运行平台：

```bash
cd /home/lunar/project/lunar_slam/device/sterellab_ZEDX
./scripts/check_platform.sh
```

### 1. 安装一个 GMSL2 驱动

根据**物理 ZED Link 采集卡型号**执行且只执行其中一个：

```bash
./scripts/install_gmsl_driver.sh mono
./scripts/install_gmsl_driver.sh duo
./scripts/install_gmsl_driver.sh quad
```

当前主机不是实时内核，所以下载的是非 `-rt` 驱动。安装驱动后必须重启。不能根据
相机数量猜测采集卡型号；请读取采集卡标签或包装清单后再选择。

### 2. 安装 ZED SDK 5.4.0

交互式安装：

```bash
./scripts/install_sdk.sh
```

无人值守安装可使用官方参数：

```bash
./scripts/install_sdk.sh -- silent
```

脚本先验证平台和安装包 SHA-256，再启动官方安装器。安装器会显示 Stereolabs
许可协议。

### 3. 构建 ROS 2 工作区

若系统缺少 CUDA/JetPack 开发组件，先安装：

```bash
sudo apt update
sudo apt install nvidia-jetpack nvidia-jetpack-dev
```

使用默认的 `NEURAL LIGHT` 深度模式还需要 TensorRT 10 的运行库和未版本化开发链接：

```bash
sudo apt install tensorrt-libs libnvinfer-dev libnvinfer-plugin-dev
```

然后安装 ROS 依赖并构建：

```bash
./scripts/build_ros2.sh
```

默认构建核心 Wrapper、接口、相机描述和 ZED RViz 显示包；不会拉入可选的 Isaac ROS
示例或内部调试包。

### 4. 启动 ZED X

```bash
./scripts/launch_zedx.sh
```

脚本会检查宿主机 daemon、IMU 权限和 X11/EGL，并在当前开发容器中刷新
Argus/IMU socket。首次使用 `NEURAL LIGHT` 时会生成 TensorRT 优化模型，可能需要
数分钟；同一 ZED SDK/CUDA/TensorRT 组合后续会直接复用缓存。

该脚本等价于：

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedx
```

其他 launch 参数可以直接追加，例如：

```bash
./scripts/launch_zedx.sh camera_name:=front_zed publish_tf:=true
```

## 查看 RGB、深度图和点云

当前设备序列号为 `45570700`。以下命令已在本机的图形桌面 `DISPLAY=:0` 上实际
启动验证。相机同一时刻只能由一个进程打开；切换 SDK 工具与 ROS 2 前，先关闭正在
运行的程序（终端中按 `Ctrl+C`）。

### 方法 A：ZED SDK Depth Viewer（最快看到效果，推荐首次检查）

该工具直接显示 RGB、伪彩色深度和 3D 点云，不经过 ROS 2：

```bash
sudo -u lunar -H \
  env DISPLAY=:0 \
      XAUTHORITY=/tmp/.docker.xauth \
      XDG_RUNTIME_DIR=/tmp/zedx-runtime-1000 \
  bash -lc '
    cd /home/lunar/project/lunar_slam/device/sterellab_ZEDX
    ./scripts/prepare_container_runtime.sh
    mkdir -p /tmp/zedx-runtime-1000
    chmod 700 /tmp/zedx-runtime-1000
    exec /usr/local/zed/tools/ZED_Depth_Viewer
  '
```

窗口中可切换 `DEPTH`、`CONFIDENCE` 和 `POINT CLOUD`，并拖动深度范围。只查看
双目/RGB 原图和采集参数时，将最后一行程序换为：

```bash
exec /usr/local/zed/tools/ZED_Explorer
```

### 方法 B：ROS 2 + 独立图像窗口（不渲染点云）

终端 1 启动相机节点。为了兼容示例自带的 RViz 配置，这里保持
`camera_name:=zed`：

```bash
cd /home/lunar/project/lunar_slam/device/sterellab_ZEDX
./scripts/launch_zedx.sh \
  camera_name:=zed \
  serial_number:=45570700 \
  node_log_type:=screen
```

若只做画面预览、不检查全分辨率数据，可用以下低负载参数替换上面的启动命令，将
ROS 发布图长宽各缩小为 1/2 并限制为 10 Hz；相机内部仍以 HD1200@30 取流：

```bash
./scripts/launch_zedx.sh \
  camera_name:=zed \
  serial_number:=45570700 \
  param_overrides:="general.pub_resolution:=CUSTOM;general.pub_downscale_factor:=2.0;general.pub_frame_rate:=10.0"
```

看到 `=== zed started ===` 后，新开终端 2 查看 RGB：

```bash
cd /home/lunar/project/lunar_slam/device/sterellab_ZEDX
source /opt/ros/humble/setup.bash
source ros2_ws/install/local_setup.bash
export DISPLAY=:0
export XAUTHORITY=/tmp/.docker.xauth
export XDG_RUNTIME_DIR=/tmp/zedx-runtime-$(id -u)
mkdir -p "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR"
ros2 run rqt_image_view rqt_image_view \
  /zed/zed_node/rgb/color/rect/image
```

再开终端 3 查看与 RGB 对齐的深度图：

```bash
cd /home/lunar/project/lunar_slam/device/sterellab_ZEDX
source /opt/ros/humble/setup.bash
source ros2_ws/install/local_setup.bash
export DISPLAY=:0
export XAUTHORITY=/tmp/.docker.xauth
export XDG_RUNTIME_DIR=/tmp/zedx-runtime-$(id -u)
ros2 run rqt_image_view rqt_image_view \
  /zed/zed_node/depth/depth_registered
```

其中 RGB 话题为 `bgra8` 图像；注册深度为 `32FC1`，单位是米。`rqt_image_view`
会为显示自动拉伸深度灰度，它不是原始距离的颜色标尺；需要读取某个像素的真实距离
时，应在程序中读取浮点值。

### 方法 C：ROS 2 + RViz（同时看 RGB、深度和三维点云）

先按方法 B 的终端 1 启动 ZED 节点。然后在终端 2 运行：

```bash
cd /home/lunar/project/lunar_slam/device/sterellab_ZEDX
source /opt/ros/humble/setup.bash
source ros2_ws/install/local_setup.bash
export DISPLAY=:0
export XAUTHORITY=/tmp/.docker.xauth
export XDG_RUNTIME_DIR=/tmp/zedx-runtime-$(id -u)
mkdir -p "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR"
ros2 launch zed_display_rviz2 display_zed_cam.launch.py \
  camera_model:=zedx \
  camera_name:=zed \
  start_zed_node:=false
```

预置布局已订阅以下内容：

```text
RGB:       /zed/zed_node/rgb/color/rect/image
深度:      /zed/zed_node/depth/depth_registered
彩色点云:  /zed/zed_node/point_cloud/cloud_registered
```

本机构建没有安装可选的 `rviz_plugin_zed_od`，所以启动时会出现
`ZedOdDisplay failed to load`；该提示只影响专用里程计面板，不影响 RGB、深度、普通
`PointCloud2` 和 TF。Jetson 本机渲染全分辨率点云开销很大，仅查看图像时优先使用
方法 B，或者在 RViz 左侧取消勾选 `PointCloud2`。

### 检查话题和实际帧率

在 ZED 节点运行时执行：

```bash
cd /home/lunar/project/lunar_slam/device/sterellab_ZEDX
source /opt/ros/humble/setup.bash
source ros2_ws/install/local_setup.bash

ros2 topic list | grep '^/zed/zed_node/'
ros2 topic info /zed/zed_node/rgb/color/rect/image
ros2 topic info /zed/zed_node/depth/depth_registered
ros2 topic hz /zed/zed_node/rgb/color/rect/image
ros2 topic hz /zed/zed_node/depth/depth_registered
```

`ros2 topic hz` 本身也是订阅者；按 `Ctrl+C` 结束统计。Wrapper 5.4 的大图、深度和
点云采用按需发布，没有订阅者时不会持续搬运所有 ROS 图像数据。

## 接入 Luxi 网页建图

ZED X 已作为独立 profile 接入 `luxi_adapter` 和 `luxi-web-control`。从正式工作区启动
网页（当前测试模式不要求启动机器人）：

```bash
cd /home/lunar/project/lunar_slam
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch luxi_web_control lekiwi_web_control.launch.py \
  bind_address:=0.0.0.0 http_port:=8080
```

打开 `http://本机IP:8080`，依次选择 `ZED X + ZED Link Duo`、启动相机、开始建图。
网页只在 ZED X profile 下选择 `zedx_mapping.launch.py`，使用 SDK 原生双目惯性
`/zed/zed_node/odom`；D455、D435i 和 HIK 继续使用原来的学习视觉里程计及其参数。
ZED X 专用链路已启用完整 6DoF（x/y/z、roll/pitch/yaw），适合手持倾斜和三维路线；
其他相机仍保持原有平面建图设置。旧的平面模式只适合相机与水平移动底盘刚性连接，
不适合拿起相机观察桌面、天花板或楼层高度变化。
结束时依次点击“停止建图”和“关闭相机”，等待数据库保存完成。

绕过网页做诊断时可在相机适配层已运行后另开终端：

```bash
cd /home/lunar/project/lunar_slam
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 topic hz /zed/zed_node/odom --window 50
ros2 launch luxi_rtab_map zedx_mapping.launch.py \
  rviz:=true new_map:=true load_saved_map:=false \
  planar_motion:=false \
  database_path:=/tmp/zedx_native_mapping.db
```

确认运行参数：

```bash
ros2 param get /zed/zed_node pos_tracking.two_d_mode
ros2 topic echo /zed/zed_node/odom --once
ps -eo args | grep '[r]tabmap' | grep -E 'Force3DoF|ForceOdom3DoF'
```

预期第一条为 `False`，里程计在相机上下移动或倾斜后应出现非零 z、姿态四元数 x/y；
RTAB-Map 命令应同时包含 `--Reg/Force3DoF false` 和
`--RGBD/ForceOdom3DoF false`。

ZED SDK 自带的是 Spatial Mapping API/示例，它直接使用 ZED positional tracking 融合
网格或 fused point cloud，可作为 RTAB-Map 的基线；它不等同于带地点识别和全局回环的
RTAB-Map。ROS Wrapper 运行时可做快速基线检查：

```bash
ros2 service call /zed/zed_node/enable_mapping std_srvs/srv/SetBool '{data: true}'
ros2 topic hz /zed/zed_node/mapping/fused_cloud
ros2 topic echo /zed/zed_node/mapping/fused_cloud --once --field width
ros2 service call /zed/zed_node/enable_mapping std_srvs/srv/SetBool '{data: false}'
```

本机 Wrapper 5.4 实测启用后能以约 1 Hz 输出 fused cloud，但禁用服务曾使
`component_container_isolated` 异常退出。因此不要把该服务和 RTAB-Map 同时作为正式
建图链路；做官方基线时独立启动相机，保存性测试优先编译 SDK 的 Spatial Mapping 示例，
完成后直接关闭相机进程。

`map043` 已包含错误视觉 PnP 邻接位姿，不要在其上续建。问题分析和实测数据见
`project/luxi_adapter/docs/zedx_test_report.md`。

## 本机资源开销实测

测试日期为 2026-08-24，平台为 Jetson AGX Orin、`MODE_50W`，ZED X 使用
`HD1200 @ 30 Hz`、`NEURAL_LIGHT`。数值来自 1 秒间隔的 `tegrastats` 和 `ps`，用于
估算该主机上的量级，不代表所有场景的固定值。

| 状态 | GR3D GPU 利用率 | 主要进程 CPU | 系统 RAM | `VDD_GPU_SOC` |
|---|---:|---:|---:|---:|
| 空闲基线 | 0% | 后台负载 | 约 5595 MB | 约 3.17 W |
| ZED ROS 节点，无图像/点云订阅 | 平均约 10%，峰值 15% | ZED 约 63%（约 0.63 个逻辑核） | 约 6425 MB（比空闲多约 830 MB） | 约 4.35 W |
| ZED 节点 + 两个全分辨率 rqt 图像窗口 | 平均约 70%，峰值 83% | ZED + rqt 约 187%（约 1.87 个逻辑核） | 约 6840 MB（比空闲多约 1245 MB） | 约 6.7–7.1 W |
| ZED 节点 + 预置 RViz（RGB、深度、点云全开） | 平均约 65%，峰值 91% | ZED + RViz 约 192%（约 1.92 个逻辑核） | 约 6780 MB（比空闲多约 1185 MB） | 约 7.5–7.9 W |

补充说明：

- 仅节点时 `component_container_isolated` 的 RSS 约 1.18 GiB；RViz 全开时 ZED
  进程 RSS 约 1.33 GiB、RViz 约 0.40 GiB。RSS 包含共享映射，不能直接相加当作系统
  增量。
- `VDD_GPU_SOC` 是 GPU 与 SoC 的合并电源轨，不是纯 GPU 或纯相机功耗；表中差值
  只能作为整个平台负载变化的近似。
- 两个全分辨率 `rqt_image_view` 窗口仍会触发大图发布、格式转换与桌面渲染，不能
  当作低开销基准。只开一个窗口、在窗口内切换话题，或使用上面的 1/2 尺寸/10 Hz
  预览参数会更合适。
- RViz 全开时，本机实测 RGB 约 14–16 Hz、深度约 12 Hz，低于相机的 30 Hz。
  Stereolabs 官方也明确不建议在 Jetson 上用 RViz 同时处理全分辨率图像和点云。
- Stereolabs 的 AGX Orin 官方基准中，单台 ZED X、30 FPS、NEURAL_LIGHT 约为
  CPU 5%、GPU 11%；其测试使用 MAXN，统计口径也和 Linux `ps` 不同。本机“仅节点”
  的约 10% GPU 与该量级一致。
- 深度模式对效果/开销影响明显：`NEURAL_LIGHT` 最快，推荐障碍物检测；`NEURAL`
  细节和稳定性更好；`NEURAL_PLUS` 细节与有效范围最好、开销也最大。

实时查看系统负载：

```bash
# 本开发容器中读取宿主机 Jetson 统计
sudo nsenter -t 1 -m -u -i -n -p \
  /usr/bin/tegrastats --interval 1000

# 另一个终端查看 ZED/RViz 进程；%CPU=100 表示占满一个逻辑核
ps -eo pid,comm,%cpu,%mem,rss,args --sort=-%cpu | \
  grep -E 'zed_node_main|component_container_isolated|rviz2'
```

## 与 Intel RealSense D455 的效果和开销对比

当前 ZED X 标定文件给出的基线为 `119.612 mm`，焦距对应约 `105° × 78°` 的
HD1200 矫正视场，属于宽视角版本。下面的 D455 数据来自其官方规格；实际画质仍应在
相同安装位姿、曝光、距离和光照下做 A/B 测试。

| 项目 | 当前 ZED X（2.2 mm 宽视角） | RealSense D455 |
|---|---|---|
| RGB/双目分辨率 | 2 × 1920×1200，全局快门；当前 30 FPS | 深度最高 1280×720；RGB 1280×800@30，全局快门 |
| 视场 | 当前标定约 105°×78°；官方最大 110°×80° | 深度 87°×58°，RGB 90°×65° |
| 双目基线 | 当前标定 119.612 mm（标称约 120 mm） | 95 mm |
| 官方理想深度范围 | 0.3–12 m；最大约 20 m | 0.6–6 m；最大分辨率 Min-Z 约 0.52 m |
| 深度计算位置 | Jetson GPU 上的 ZED 神经网络/SDK | 相机内 D4 视觉处理器 |
| 主机连接 | GMSL2/FAKRA + ZED Link，适合长线、振动和机器人固定集成 | USB-C 3.1，接入与跨平台调试更简单 |
| 主机资源 | 明显使用 CUDA/GPU；模式和订阅内容决定负载 | 深度在 D4 上计算，通常几乎不需要主机 GPU，但 USB 搬运和 ROS 对齐/点云仍占 CPU/内存 |

选型结论：

- 户外导航、较远障碍物、宽视场、快速运动和工业布线优先 ZED X。它的基线更长、
  视场更宽，RGB/深度原生像素更多；NEURAL/NEURAL_PLUS 对低纹理、低光和细小物体
  通常更完整，但会占用 Jetson GPU。
- 0.6–6 m 的室内感知、原型调试或 GPU 预算紧张时，D455 更省主机算力、更容易用
  USB 部署。其主动 IR 在室内无纹理表面可能更有帮助，但强日光下不能依赖投射纹理。
- 只比较预览窗口容易把渲染器开销误认为相机开销。SLAM 实际选型应记录同一路线，
  比较深度空洞率、边缘/细杆完整度、2/4/6/10 m 误差、快速运动拖影、轨迹漂移以及
  “不启动 RViz”时的端到端资源占用。

官方资料：

- [ZED ROS 2：RGB、深度和点云显示及 Jetson 上的 RViz 警告](https://docs.stereolabs.com/docs/integrations/ros-2)
- [ZED 神经深度模式、精度和 AGX Orin 性能数据](https://docs.stereolabs.com/docs/development/zed-sdk/modules/depth-sensing/depth-modes)
- [ZED X 官方规格表](https://support.stereolabs.com/hc/article_attachments/27901419903255)
- [RealSense D455 官方产品规格](https://www.realsenseai.com/products/real-sense-depth-camera-d455f/)
- [RealSense D400 系列官方数据手册](https://dev.realsenseai.com/download/42003/)

## Docker / 开发容器

ZED X 是 GMSL2 相机。以后重建容器时，除 `--runtime nvidia --privileged` 外，应按
Stereolabs 要求使用 `--network=host --ipc=host --pid=host`，并至少映射：

```text
/tmp:/tmp
/dev:/dev
/dev/shm:/dev/shm
/var/nvidia/nvcam/settings:/var/nvidia/nvcam/settings
/etc/systemd/system/zed_x_daemon.service:/etc/systemd/system/zed_x_daemon.service
/usr/local/zed/settings:/usr/local/zed/settings
/usr/local/zed/resources:/usr/local/zed/resources
```

当前 `lunar_slam` 容器未映射完整 `/tmp`，所以 `launch_zedx.sh` 会先调用
`prepare_container_runtime.sh`，通过已共享的 X11 目录连接实时宿主 socket。若
`nvargus-daemon` 重启并更换 socket inode，再次执行启动脚本即可自动刷新。

## 安装后检查

```bash
sudo /usr/local/sbin/verify_zedx

sudo dmesg | grep -i zedx
systemctl status zed_x_daemon --no-pager
ls -l /dev/video*
/usr/local/zed/tools/ZED_Explorer
ros2 topic list | grep zed
```

无界面的 SDK 实际取流测试：

```bash
python3 ./scripts/test_sdk_stream.py
```

该测试会打开第一台 ZED X，以 HD1200@30 采集 60 帧，并验证左图、深度、时间戳、
帧率和 IMU 数据。

本机 2026-08-24 的完整验收结果记录在 `CONFIGURATION_STATUS.md`。

GMSL2 硬件接线或端口顺序变化后，需要重启主机，或在硬件状态稳定后执行：

```bash
sudo systemctl restart zed_x_daemon
```
