# 项目 RGB-D 建图节点

`luxi_rtab_map` 是 `project` 目录下的建图功能包。它只负责算法节点，
不包含 D435i 硬件驱动。

## 职责边界

```text
device/D435i
  lunar_realsense_bringup
  -> RGB、对齐深度、CameraInfo、PointCloud2、TF

project/luxi_RTAB_Map
  rgbd_mapping.launch.py
  -> 输入检查、RGB-D 同步、视觉里程计、RTAB-Map、轻量 RViz
```

后续定位、导航和语义识别也应作为 `project` 下的独立 ROS2 包接入上述
标准话题，避免把算法代码放入硬件驱动目录。

## 从开机到建图

开机后打开两个 `lunar@lunar_slam` 容器终端，按以下顺序启动。

### 启动前：检查并关闭旧进程

同一台 D435i 不能同时被两个驱动进程占用。启动相机前先检查是否残留旧的相机或
建图进程：

```bash
pgrep -af 'ros2 launch lunar_realsense_bringup|realsense2_camera_node'
pgrep -af 'ros2 launch luxi_rtab_map|rgbd_odometry|rtabmap_slam/rtabmap'
```

如果旧程序所在终端仍然存在，优先回到对应终端按 `Ctrl-C`，等待进程正常退出。
如果找不到原终端，可以向旧的 ROS 2 launch 进程发送与 `Ctrl-C` 相同的 `SIGINT`：

```bash
pkill -SIGINT -f 'ros2 launch luxi_rtab_map rgbd_mapping.launch.py'
pkill -SIGINT -f 'ros2 launch lunar_realsense_bringup d435i.launch.py'
sleep 3
```

然后确认没有残留进程：

```bash
pgrep -af 'realsense2_camera_node|rgbd_odometry|rtabmap_slam/rtabmap'
```

没有输出表示相关进程已经关闭。若仍有输出，记录显示的 PID，只对这些明确的残留
进程发送 `SIGTERM`，例如：

```bash
kill -SIGTERM <PID>
```

不要在旧相机驱动仍运行时再次启动相机，否则可能出现 `failed to set power state`
或设备忙错误。正常建图时只需各启动一个相机进程和一个建图进程。

### 终端一：启动相机

执行后保持这个终端运行，看到 `RealSense Node Is Up!` 后再使用终端二：

```bash
source /opt/ros/humble/setup.bash
source /home/lunar/project/lunar_slam/device/D435i/ros2_ws/install/setup.bash
ros2 launch lunar_realsense_bringup d435i.launch.py \
  rviz:=false enable_imu:=true unite_imu_method:=2 enable_pointcloud:=false
```

### 终端二：启动建图和 RViz

```bash
export DISPLAY=:0
export XAUTHORITY=/tmp/.docker.xauth
export QT_X11_NO_MITSHM=1

source /opt/ros/humble/setup.bash
source /home/lunar/project/lunar_slam/device/D435i/ros2_ws/install/setup.bash
source /home/lunar/project/lunar_slam/install/setup.bash
ros2 launch luxi_rtab_map rgbd_mapping.launch.py
```

每次启动会自动选择下一个未使用的数据库，例如：

```bash
/home/lunar/project/lunar_slam/maps/map001.db
/home/lunar/project/lunar_slam/maps/map002.db
```

在建图终端按 `Ctrl-C` 并等待 RTAB-Map 正常退出后，当前地图已保存。查看、导出和
继续已有地图的方法见 [maps/README.md](../../../maps/README.md)。

<!-- 以下为历史详细排障记录，默认不参与文档阅读。

以下流程适用于当前 Jetson 图形桌面和名为 `lunar_slam` 的 Docker 容器。必须先登录
Jetson 本地图形桌面；仅停留在登录界面或只通过 SSH 启动时，不保证存在可供 RViz
连接的用户图形会话。

### 1. 在 Jetson 主机确认显示服务并启动容器

打开 Jetson 本地桌面的终端，不是在容器里执行。主机提示符通常包含
`luxi-jetson`；如果提示符是 `lunar@lunar_slam`，说明已经在容器里，不能执行
`docker` 命令：

```bash
echo "$DISPLAY"
ls -l /tmp/.X11-unix/
docker start lunar_slam
```

当前机器的正常结果是 `DISPLAY=:0`，并且 X11 socket 为
`/tmp/.X11-unix/X0`。如果 `echo` 的结果和 socket 编号不同，以实际存在的 socket
编号为准。当前容器已经挂载主机的 `/tmp/.X11-unix`，并将主机 Xauthority 挂载为
容器内只读文件 `/tmp/.docker.xauth`。

可在主机确认挂载仍然存在：

```bash
docker inspect lunar_slam --format '{{range .Mounts}}{{println .Source "->" .Destination}}{{end}}' \
  | grep -E 'X11-unix|docker.xauth'
```

### 2. 进入容器并验证 RViz 显示链路

在主机终端进入容器，同时覆盖容器中可能过期的 `DISPLAY`：

```bash
docker exec -it \
  -e DISPLAY=:0 \
  -e XAUTHORITY=/tmp/.docker.xauth \
  -e QT_X11_NO_MITSHM=1 \
  lunar_slam bash
```

如果当前已经处于 `lunar@lunar_slam` 容器 shell，无需退出或再次执行
`docker exec`，直接在当前 shell 修正环境：

```bash
export DISPLAY=:0
export XAUTHORITY=/tmp/.docker.xauth
export QT_X11_NO_MITSHM=1
```

进入容器后先检查配置。不能只看 `echo $DISPLAY`，还要确认对应 socket 存在：

```bash
echo "$DISPLAY"
test -S /tmp/.X11-unix/X${DISPLAY#:} && echo "X11 socket OK"
glxinfo -B | grep -E 'direct rendering|OpenGL vendor|OpenGL renderer|OpenGL version'
```

当前机器应显示 `direct rendering: Yes`、NVIDIA Tegra 渲染器和 OpenGL 4.6。
需要单独验证窗口时可运行：

```bash
timeout 10 rviz2
```

RViz 窗口能够在 Jetson 桌面出现，且终端输出 OpenGL 版本而不是 `could not connect
to display`，说明 GUI 链路正常。`timeout` 会在 10 秒后关闭这个测试窗口。

### 3. 终端一：启动 D435i

先确认没有旧的相机驱动占用同一设备：

```bash
pgrep -af realsense2_camera_node || true
```

如果已经存在一个正常运行的 `realsense2_camera_node`，直接复用它，不要再次启动。
同一台 D435i 不能被两个驱动进程同时打开；重复启动会出现
`RS2_USB_STATUS_BUSY`、`failed to claim usb interface` 和
`failed to set power state`。

没有相机进程时，在容器终端一执行：

```bash
export DISPLAY=:0
export XAUTHORITY=/tmp/.docker.xauth
export QT_X11_NO_MITSHM=1

source /opt/ros/humble/setup.bash
source /home/lunar/project/lunar_slam/device/D435i/ros2_ws/install/setup.bash
ros2 launch lunar_realsense_bringup d435i.launch.py \
  rviz:=false enable_imu:=true unite_imu_method:=2 enable_pointcloud:=false
```

保持终端一运行，直到日志出现 `RealSense Node Is Up!`。另开容器终端检查输入：

```bash
source /opt/ros/humble/setup.bash
source /home/lunar/project/lunar_slam/device/D435i/ros2_ws/install/setup.bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
ros2 topic hz /camera/camera/imu
```

RGB 和对齐深度应接近 30 Hz，IMU 应持续输出。按 `Ctrl-C` 退出频率检查，不要停止
终端一的相机驱动。

本 Jetson 内核没有启用 `CONFIG_HID_SENSOR_HUB`，系统自带 librealsense 的
V4L2/IIO 后端只能识别 RGB 和深度，不能识别 D435i Motion Module。硬件启动包
因此定向加载项目内按 `FORCE_RSUSB_BACKEND=true` 构建的同版本 librealsense：

```text
device/D435i/ros2_ws/3parts/librealsense/install-rsusb
```

RSUSB 在用户态直接访问 USB，不依赖 Jetson 的 HID Sensor Hub 内核配置。启动日志
中必须出现 Motion Module，并且下面两个检查都应有持续数据：

```bash
ros2 topic hz /camera/camera/imu
ros2 topic hz /imu/data
```

如果日志出现 `No HID info provided, IMU is disabled`，说明启动了系统 librealsense
或项目私有 RSUSB 库不存在。此时不要启动建图；建图启动文件也会等待原始 IMU，
超时后明确退出，避免静默退回容易在大转角时丢失的纯视觉模式。

### 4. 终端二：启动建图和 RViz

从主机再执行一次第 2 步的 `docker exec`，进入容器终端二，然后执行：

```bash
export DISPLAY=:0
export XAUTHORITY=/tmp/.docker.xauth
export QT_X11_NO_MITSHM=1

source /opt/ros/humble/setup.bash
source /home/lunar/project/lunar_slam/device/D435i/ros2_ws/install/setup.bash
source /home/lunar/project/lunar_slam/install/setup.bash
ros2 launch luxi_rtab_map rgbd_mapping.launch.py
```

正常情况下，Jetson 本地桌面会出现 RViz，ROS 图中至少包含：

```text
/camera/camera
/d435i_imu_filter
/rtabmap/rgbd_sync
/rtabmap/rgbd_odometry
/rtabmap/rtabmap
/rviz
```

可在第三个容器终端验证：

```bash
source /opt/ros/humble/setup.bash
source /home/lunar/project/lunar_slam/install/setup.bash
ros2 node list | sort
ros2 topic hz /rtabmap/odom
ros2 topic hz /rtabmap/mapData
```

当前实机验证结果为相机和里程计约 30 Hz、`/rtabmap/mapData` 约 1 Hz。RViz 日志
显示 NVIDIA OpenGL 4.6，并持续输出 `Trying to create a map ...`，确认窗口、渲染和
建图数据链路都已工作。

首次创建一张全新的地图时使用：

```bash
ros2 launch luxi_rtab_map rgbd_mapping.launch.py new_map:=true
```

`new_map:=true` 会在本次启动时删除默认数据库中的旧地图。确认旧地图不再需要，
或者已经完成备份、导出后再使用该参数。

启动时会主动终止旧的 `d435i_rtabmap`、`rgbd_mapping_test`、
`luxi_rtab_map_node`、孤儿化的 RTAB-Map 子节点和其测试 RViz 进程，再启动
正式建图节点。独立运行的 `lunar_realsense_bringup` 硬件驱动不会被清理。

建图节点启动前会等待 RGB、对齐深度和 CameraInfo 都产生真实消息。默认等待
15 秒；若驱动未启动、相机掉线或话题名错误，程序会直接给出缺失话题并退出，
不会在无数据状态下持续刷警告。可按需修改超时：

```bash
ros2 launch luxi_rtab_map rgbd_mapping.launch.py camera_wait_timeout:=30.0
```

### RViz 窗口不出现时

先抓取最新 launch 日志并检查进程：

```bash
latest_log=$(ls -td ~/.ros/log/*/ | head -1)
grep -RniE 'rviz|xcb|display|qt.qpa|error|failed' "$latest_log"
pgrep -af 'rviz2|realsense2_camera_node|rgbd_odometry|rtabmap_slam/rtabmap'
```

本次故障的直接原因是容器环境为 `DISPLAY=:1`，而主机只挂载了 Xorg 的 `X0`
socket。Qt 因此报告：

```text
qt.qpa.xcb: could not connect to display :1
```

修复当前终端后，可单独重新启动项目 RViz，不需要重启相机或建图：

```bash
export DISPLAY=:0
export XAUTHORITY=/tmp/.docker.xauth
export QT_X11_NO_MITSHM=1
source /opt/ros/humble/setup.bash
source /home/lunar/project/lunar_slam/device/D435i/ros2_ws/install/setup.bash
source /home/lunar/project/lunar_slam/install/setup.bash
rviz2 -d /home/lunar/project/lunar_slam/install/luxi_rtab_map/share/luxi_rtab_map/rviz/rgbd_mapping.rviz
```

如果没有 `/tmp/.X11-unix/X0` 或 `/tmp/.docker.xauth`，说明容器创建时缺少 X11
挂载；仅设置环境变量不能补救，需要在主机修正容器挂载后重建容器。如果 socket
和授权文件存在但仍失败，先在容器运行 `glxinfo -B`，其错误通常比 RViz 更直接。

-->

## 稳定性配置

当前启动文件针对本设备做了以下处理：

- D435i 的 RGB 和深度数据由硬件包以 `640x480@30 Hz` 提供。建图启动命令关闭
  RealSense 驱动端的当前帧点云滤波器，因为 RTAB-Map 不订阅它，并会独立生成
  `/rtabmap/cloud_map` 累计三维地图。需要单独检查硬件点云接口时可改为
  `enable_pointcloud:=true`。
- 硬件驱动默认启用 RealSense 空间和时间深度平滑，再将深度对齐到 RGB。实测当前
  问题视角中，0.2 到 4 米的有效深度比例由约 39.9% 提高到 42.7%。没有启用补洞，
  因此相机移动时不会把上一帧的旧深度错误地保留到新画面。
- D435i 陀螺仪和加速度计默认开启，使用 RealSense 的线性插值合成原始 IMU，
  再由 Madgwick 滤波器发布带姿态的 `/imu/data`。RTAB-Map 启动前会等待 IMU，
  用重力方向约束滚转和俯仰，提升手持转动时的稳定性。Madgwick 增益设为 0.03，
  静止实测相邻 IMU 姿态抖动中位数由约 0.019° 降到 0.006°。
- 关闭视觉里程计的自动重置和恒速运动猜测。短时匹配失败时保留原地图并暂停
  添加关键帧，不再创建没有空间约束的新子地图；相机重新看到上一视角后可继续
  匹配。这样可以避免日志中的 `Increment map id` 导致地图突然切换或消失。
- 启用 RTAB-Map 内置 Kalman 位姿滤波（`Odom/FilteringStrategy=1`），并将测量
  噪声设为 0.05。未滤波实测
  在相机静止时，相邻里程计帧的平移抖动中位数约 5.66 cm、旋转约 0.96°，会让
  `camera_link` 坐标轴明显跳动。启用深度、IMU 和位姿滤波后，当前问题视角中
  `/rtabmap/odom` 的相邻帧平移中位数约为 1.25 mm、旋转约为 0.027°，30 Hz 输出
  保持稳定。滤波只平滑里程计输出，不会修改 RGB、深度和已保存
  的原始关键帧数据，但快速移动时会产生轻微的显示延迟。
- 视觉里程计最低内点数调整为 10。实机日志中大转角首次失败时仍有 12 到 14 个
  有效内点，旧阈值 15 会拒绝这一帧，随后相邻画面重叠继续下降并彻底丢失。
  IMU 预测附近的特征搜索窗口由 40 像素扩大到 120 像素，允许相邻帧出现更大的
  旋转位移；用于里程计和三维地图的关键点有效深度限制为 0.2 到 4.5 米。
- 里程计局部特征图限制为 1500 个点，并关闭实时局部束调整。实机日志显示局部图
  增长到 2000 点后处理频率从 30 Hz 降到约 15 Hz，且局部束调整会在低纹理帧把
  本来已经很少的有效内点继续剔除。这个调整只作用于实时视觉里程计；RTAB-Map
  全局位姿图优化、闭环检测及 1 Hz 地图检测仍然保留。
- 视觉里程计使用多尺度 ORB 检测和描述特征，替代单尺度 GFTT 检测加 ORB 描述，
  提高转动、尺度变化和低纹理画面中的可跟踪特征数量。
- 当当前帧内点比例低于 0.5 或内点数低于 200 时，提前刷新视觉里程计关键帧，
  避免相机继续转动后仍依赖已经接近视野边缘的旧特征。
- 单帧视觉特征上限为 1000，在当前 Jetson 上增加大转角时可匹配的局部特征。
  实测视觉里程计仍保持约 30 Hz；单个里程计进程约占用一个 CPU 核，因此不应再
  同时启用计算重复的实时点云转换节点。视觉里程计内部将 640x480 图像二倍
  降采样后做位姿匹配；RTAB-Map 保存和显示的 RGB-D 地图仍使用
  原始相机数据，不会把导出的三维地图分辨率直接降为 320x240。
- RTAB-Map 地图检测率为 1 Hz，平移 0.1 m 或转动 0.1 rad 后创建地图节点。
  旧配置的 2 Hz、0.05 阈值会把静止画面的毫米级视觉抖动也保存为节点，数分钟
  便生成近千个重复节点并显著增加闭环和全图优化负担。闭环接受阈值提高到 0.2，
  减少低纹理、重复场景中的错误闭环候选。
- RTAB-Map 和视觉里程计默认只输出 `warn` 及以上日志，避免 30 Hz 的逐帧
  `INFO` 输出拖慢 SSH 终端和日志写入；调试时可传入 `log_level:=info`。
- 使用项目自己的轻量 RViz 配置。`Accumulated3DCloud` 直接订阅 RTAB-Map
  服务器组装并缓存的 `/rtabmap/cloud_map`，用于稳定显示已经写入地图的彩色
  三维点，同时显示位姿图和二维栅格地图，RViz 刷新率限制为 15 Hz。重复解压
  `/rtabmap/mapData` 的 `Global3DMap` 默认关闭，实测可把 RViz CPU 占用从约 56%
  降至约 33%；需要检查单个 RGB-D 节点时仍可手动启用。
- 启动后会自动调用 `/rtabmap/rtabmap/publish_map`，向 RViz 发布数据库中的完整
  连通地图，后续关键帧继续累积。RViz 的手动下载入口也已指向实际服务
  `/rtabmap/rtabmap/get_map_data`。
- 不再启动官方示例中的实时相机点云转换节点，也不订阅 QoS 不兼容的
  `/rtabmap/odom_local_map` 和 `/voxel_cloud`，从而减少 CPU/GPU 负载和警告。

如果视觉里程计短暂丢失，先将相机转回最后一次成功定位的、有纹理且距离约
0.3 到 4.5 米的静态场景，再缓慢平移和转动。纯白墙、玻璃、强反光、过近目标、
快速甩动和遮挡镜头都会让 RGB-D 特征匹配失败。当前配置不会自动重置并创建
无约束子地图；重新匹配成功后才会继续向原地图添加关键帧。

日志中的以下消息属于同一次视觉里程计丢失过程：

- `Failed to find a transformation with the provided guess`：D435i IMU 给出了相邻帧
  的旋转预测，但图像特征不支持该预测位姿。
- `trying again without a guess`：RTAB-Map 已经自动取消 IMU 预测并进行第二次纯
  图像匹配，不需要用户修改启动参数。
- `Not enough inliers 7/10`：只有 7 个几何一致的特征，低于当前安全阈值 10；
  `matches=13` 是描述子初步匹配数，不代表这些匹配都能产生可信位姿。
- `Trial with no guess still fail`：取消预测后仍没有足够的共同视野，通常表示转动
  太快、运动模糊、画面纹理太少或镜头已经离开上一关键帧视野。
- `no odometry is provided. Image 0 is ignored`：这是下游保护行为。视觉里程计没有
  位姿时，RTAB-Map 拒绝把该 RGB-D 帧写入地图，避免污染已有地图。

本次问题日志中，首次连续失败后 `guess` 已累计到约 37° 偏航，并不是相邻两帧
真的瞬间旋转了 37°。这是因为视觉位姿停在最后一帧有效位置，而 IMU 姿态仍继续
更新。失败帧只有 5 到 24 个描述子匹配，几何内点始终为 `0/10`；取消 IMU 猜测后
仍失败，说明当前画面与最后有效局部地图已没有足够的带深度共同特征。

同一问题视角的 RGB 仍检测到约 1000 个 ORB 特征，但有效深度不足一半，且画面
下方大面积被近距离栏杆或线缆遮挡。RGB 纹理多不等于可用于 RGB-D 位姿估计的
三维特征多。需要先让彩色和深度镜头无遮挡，并使主要静态目标处于约 0.3 到
4.5 米范围内；转弯时以小弧线同时平移和旋转，让连续画面保持至少一半重叠，
不要原地快速转向白墙、玻璃或强反光表面。

D435i IMU 在这里提供旋转预测和重力方向，不能在没有视觉重叠时独立恢复六自由度
平移。软件参数可以提高容错，但无法从零几何内点推导可信位姿。若实际导航要求在
任意遮挡和大角度原地转动后仍连续定位，需要再融合轮速里程计、外部 VIO 或其他
定位传感器；仅靠 D435i RGB-D+IMU 不能作此保证。

出现一两帧后恢复不影响地图；若连续出现，应停止继续转动并将相机缓慢转回最后
成功视角。不要仅为消除警告继续降低 `Vis/MinInliers`，否则少量错误匹配可能被
接受并造成地图折叠。若缓慢移动仍持续失败，再检查 RGB、对齐深度的频率、曝光、
画面纹理和相机到目标的距离。

默认启动会创建下一个编号数据库，例如 `maps/map001.db`、`maps/map002.db`，不会
覆盖已有地图。需要继续已有地图时，显式指定其路径：

```bash
ros2 launch luxi_rtab_map rgbd_mapping.launch.py \
  database_path:=/home/lunar/project/lunar_slam/maps/map001.db
```

新地图第一次生成或优化累计点云时，单独出现一次
`Graph has changed! The whole cloud is regenerated` 是正常行为；只有该消息伴随闭环
拒绝并持续重复、同时地图频繁跳动时，才表示数据库或约束存在问题。

## RViz 查看完整三维重建

RViz 左侧 `Displays` 中默认启用以下全局显示：

- `Accumulated3DCloud`：订阅 `/rtabmap/cloud_map`，是 RTAB-Map 节点已经组装好的
  累计彩色点云。它使用可靠、Transient Local QoS，RViz 晚于建图节点启动时也能
  收到最后保存的地图。这是判断“是否已经建出地图”的首选显示。
- `Global3DMap`：从 `/rtabmap/mapData` 的 RGB-D 关键帧在 RViz 内生成更密集的
  彩色点云，适合离线检查重建细节。该显示计算量较大，默认关闭。

它们都不是 D435i 的当前帧点云。移动相机后应看到旧场景保留、新场景逐步加入，
并在 `MapGraph` 中看到连续增加的位姿节点。

`CameraCoordinate` 显示 `camera_link` 的红、绿、蓝三轴，`CameraTF` 显示完整 TF
树，两者默认启用；坐标轴长度已经增大到 0.5 m，便于在点云中识别。相机轨迹
`CameraOdometry` 保留在配置中但默认关闭，因为 RViz 插件初始化时会短暂创建
Reliable 订阅，而里程计发布端为 Best Effort，导致一次误导性的 QoS 警告。需要
轨迹时可手动启用，该插件最终配置已经使用 Best Effort QoS。视觉里程计丢失期间
没有可信的新相机位姿，`CameraCoordinate` 会停止更新，不会伪造相机坐标。

`RGBImage` 默认订阅 `/camera/camera/color/image_raw`，使用 Best Effort QoS，在同一
个 RViz 窗口中创建 RGB 渲染区域。它只负责显示，不会再复制或转换图像，因此不会
改变 RTAB-Map 的 RGB-D 同步和建图输入。可以拖动 RGB 区域边缘调整画面与三维地图
的显示比例，也可在 `Displays -> RGBImage` 中临时关闭。

RTAB-Map 不会把静止相机的重复图像不断保存为新地图。当前配置要求相对上一个
地图节点平移约 `0.1 m` 或转动约 `0.1 rad` 后才创建新关键帧。因此相机固定在
桌面时只看到一个视角是正常行为，不代表建图节点停止。手持测试时应缓慢移动，
并保持相邻画面至少约一半重叠。

如果 RViz 被关闭后重新打开，`Accumulated3DCloud` 会通过 Transient Local QoS
自动收到最后一份累计点云。需要检查原始 RGB-D 节点时，可手动启用
`Global3DMap`，再将 `Download map` 勾选一次，从
`/rtabmap/rtabmap/get_map_data` 加载完整连通地图；加载较大地图时 RViz 短暂变灰
属于点云解压和生成过程。

RTAB-Map 只能把具有连续里程计约束或闭环约束的关键帧放入同一张准确的三维地图。
如果快速甩动相机导致里程计丢失，新数据会形成独立子地图，无法仅靠 RViz 准确
拼接。采集时应缓慢平移和转动，并让相邻画面保留至少约一半重叠区域。

## 保存和导出三维地图

建图过程中所有关键帧、深度、位姿和图约束持续保存在下一个可用编号数据库：

```text
/home/lunar/project/lunar_slam/maps/mapNNN.db
```

停止建图程序后，可导出为通用的彩色 PLY 点云：

```bash
source /opt/ros/humble/setup.bash
source /home/lunar/project/lunar_slam/device/D435i/ros2_ws/install/setup.bash
source /home/lunar/project/lunar_slam/install/setup.bash
ros2 run luxi_rtab_map export_3d_map.sh
```

建议显式指定要导出的地图和输出目录：

```bash
ros2 run luxi_rtab_map export_3d_map.sh \
  /home/lunar/project/lunar_slam/maps/map001.db \
  /home/lunar/project/lunar_slam/maps/map001_export
```

导出的 `*_cloud.ply` 可以使用 CloudCompare、MeshLab 或 PCL 工具查看。导出脚本
会检查 RTAB-Map 是否仍在运行，避免读取尚未完整写入的数据库。

## 验证

2026-07-15 使用安装后的默认配置和临时新数据库连续运行 86 秒，里程计进程保持
运行，`/rtabmap/odom` 约为 29.96 Hz，`lost: false`，当前帧为 294 个匹配、293 个
内点。累计点云宽度为 2705，二维地图为 110x82 像素。除启动阶段一条 IMU 时间
插值提示和首次生成全局点云的提示外，没有连续失跟或节点退出。

```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /rtabmap/odom
ros2 topic echo --once /rtabmap/odom_info
ros2 topic echo --once /rtabmap/map
```

正常情况下，相机约为 30 Hz，里程计应持续输出且 `odom_info` 中 `lost: false`。
`/rtabmap/map` 收到消息表示二维栅格地图链路已建立；`/rtabmap/cloud_map` 的
`width` 大于零表示累计三维点云已经生成。RViz 中默认启用的
`Accumulated3DCloud` 和可选的 `Global3DMap` 都是全局地图显示，不是会随当前帧
更新或消失的相机局部点云。

可用下面的命令确认相机坐标显示链路，而不是只看 RViz 是否画出图标：

```bash
ros2 run tf2_ros tf2_echo map camera_link
ros2 topic info -v /rtabmap/odom
```

若仍有错误，保存本次日志后检查关键字：

```bash
latest_log=$(ls -td ~/.ros/log/*/ | head -1)
rg -n "ERROR|WARN|lost|QoS|Dropped|timeout" "$latest_log"
```

## 2026-07-15 实机日志结论

- 原配置在 307 MB 历史测试库上运行时，日志持续出现错误闭环拒绝，地图节点在约
  9 分钟内增加到 900 多个，RTAB-Map 和 RViz 会反复重建整张地图。这是此前地图
  跳动或消失的主要原因。
- 使用干净测试库和当前配置连续运行约 5 分钟，视觉里程计保持 `lost: false`，
  静止场景约有 296 个匹配、292 个内点，`map -> camera_link` 持续可查询，累计
  点云保持非空；没有配准失败、里程计重置或错误闭环日志。
- 归档旧库后，再使用本节给出的无附加参数命令运行超过 1 分钟，视觉里程计为
  `lost: false`，采样得到 306 个匹配、301 个内点和 3387 个累计点。测试期间相机
  基本静止，地图只有一个有效节点属于正常防抖行为，不应把它解释为建图停止。
- 启动首帧可能出现一次 `cannot interpolate imu transform`，原因是第一张排队的
  RGB-D 图像时间早于 RTAB-Map 节点本地 IMU 缓冲区；后续 IMU 持续进入图中。
  首次生成累计点云时也会出现一次 `Graph has changed! The whole cloud is
  regenerated`。这两个一次性提示与持续配准丢失不同；若它们反复出现，才应按
  上面的日志检查流程继续排查。
