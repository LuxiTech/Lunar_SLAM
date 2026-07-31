# 海康 MV-CH120-60UC 双目相机工作区

本工作区用于在 **Ubuntu 22.04（Jammy）+ ROS 2 Humble + ARM64** 环境中运行两台海康
MV-CH120-60UC 工业相机，提供双目图像、相机标定信息、深度图、点云和可选的 H30 IMU / RTAB-Map
启动文件。

> 本工作区已经按 ROS 2 Humble 适配。请不要混用 ROS 2 Lyrical 的环境或旧的 `install_humble/`
> 覆盖层。标准 colcon 目录只有 `build/`、`install/` 和 `log/`。

## 目录说明

| 目录 | 用途 |
| --- | --- |
| `src/hikrobot_camera_driver` | 海康双相机驱动与 MVS SDK 接口 |
| `src/stereo_depth` | 双目校正、视差、深度图和点云 |
| `src/hik_bringup` | 相机、深度、IMU、RViz、RTAB-Map 启动文件及配置 |
| `scripts/bootstrap_humble.sh` | Humble 依赖安装和标准构建脚本 |
| `build/`、`install/`、`log/` | colcon 自动生成目录，不提交到 Git |

## 环境要求

- Ubuntu 22.04（`jammy`）
- ROS 2 Humble
- ARM64 海康 MVS Linux 开发版
- 两台 MV-CH120-60UC 已接入 USB，并已在 MVS 中完成触发、曝光、用户集等配置

MVS 默认安装路径为 `/opt/MVS`。除运行库外，编译还必须有开发头文件：

```bash
test -f /opt/MVS/include/MvCameraControl.h
```

如果 MVS 运行库位于 `/opt/MVS`，但头文件位于另一个 SDK 检出目录，可在构建时指定头文件目录：

```bash
MVS_INCLUDE_DIR=/path/to/MVS/include bash scripts/bootstrap_humble.sh
```

## 首次安装与构建

```bash
cd /home/nvidia/lunar_slam/device/Hik_MV-CH120-60UC/ros2_ws

# 默认使用 /opt/MVS/include；脚本会安装 Humble 依赖，因此会请求 sudo 密码。
bash scripts/bootstrap_humble.sh
```

脚本会：

1. 检查 Ubuntu 22.04 与 ROS 2 Humble；
2. 安装 `cv_bridge`、`camera_info_manager`、RTAB-Map、RViz 等系统依赖；
3. 跳过仓库内不适用于 Humble 的第三方 RTAB-Map 源码，优先使用 Humble 二进制包；
4. 构建 `serial`、H30 IMU、海康驱动、双目深度和 bringup 包；
5. 输出标准 ROS 2 目录：`build/`、`install/`、`log/`。

每次新开终端都先加载环境：

```bash
source /opt/ros/humble/setup.bash
source /home/nvidia/lunar_slam/device/Hik_MV-CH120-60UC/ros2_ws/install/setup.bash
```

## 启动方式

### 仅启动双相机

```bash
ros2 launch hik_bringup camera_only.launch.py
```

用于确认两台相机均被打开，并发布左右图像和相机内参。

### 双目图像与深度

```bash
# 设备端运行。若 RViz 在另一台宿主机运行，建议关闭设备端 RViz。
ros2 launch hik_bringup stereo_camera_bringup.launch.py use_rviz:=false use_imu:=false
```

该启动会先初始化 MVS 双相机，再延迟启动深度节点，避免 ARM64 MVS 运行时的并发初始化问题。

### 相机、H30 IMU 与 RTAB-Map

```bash
ros2 launch hik_bringup rtabmap_stereo_imu.launch.py
```

使用前请确认 H30 串口设备、IMU 参数和相机到 IMU 的外参配置正确。此启动面向建图/定位，建议先完成上面的双目深度验证。

## 常用话题

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/left_camera/image` | `sensor_msgs/Image` | 左侧全分辨率图像，供设备端深度计算使用 |
| `/right_camera/image` | `sensor_msgs/Image` | 右侧全分辨率图像，供设备端深度计算使用 |
| `/left_camera/camera_info` | `sensor_msgs/CameraInfo` | 左相机标定信息 |
| `/right_camera/camera_info` | `sensor_msgs/CameraInfo` | 右相机标定信息 |
| `/stereo/depth` | `sensor_msgs/Image` | 深度图，编码为 `32FC1` |
| `/stereo/points` | `sensor_msgs/PointCloud2` | 稀疏点云 |
| `/stereo/preview/left_color/compressed` | `sensor_msgs/CompressedImage` | 左侧 JPEG 压缩预览，推荐宿主机 RViz 使用 |
| `/stereo/preview/right_color/compressed` | `sensor_msgs/CompressedImage` | 右侧 JPEG 压缩预览，推荐宿主机 RViz 使用 |

全分辨率图像约为每帧 9 MB，不适合经 Wi-Fi 直接在宿主机显示。驱动会同时发布 JPEG 预览流，实测可显著降低网络延迟。

## 宿主机 RViz 可视化

当设备与宿主机通过网络连接时，建议让设备只负责采集和深度计算，让宿主机负责 RViz 渲染。不要在 Wi-Fi 上直接显示 `/left_camera/image`、`/right_camera/image` 两路原始图像；请使用本驱动发布的 JPEG 压缩预览。

### 1. 确认网络与 DDS 配置

设备与宿主机必须能相互 ping 通，并使用相同的 ROS 域号。以下示例使用默认域 `0`：

```bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

两台机器无需使用相同 ROS 发行版：设备端使用 Humble，宿主机可以使用 Lyrical。标准消息类型（如 `sensor_msgs/Image`）可跨这两个发行版通信。

### 2. 在设备端启动相机与深度

在设备终端执行，不启动设备本机的 RViz：

```bash
cd /home/nvidia/lunar_slam/device/Hik_MV-CH120-60UC/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
ros2 launch hik_bringup stereo_camera_bringup.launch.py use_rviz:=false use_imu:=false
```

### 3. 在宿主机安装并启动 RViz

宿主机可使用 ROS 2 Lyrical，但必须安装压缩图像传输插件：

```bash
sudo apt install ros-lyrical-rviz2 ros-lyrical-compressed-image-transport
source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
```

将设备端 RViz 配置复制到宿主机工作区。以下示例中设备 IP 为 `192.168.1.31`，请按实际地址替换：

```bash
scp nvidia@192.168.1.31:/home/nvidia/lunar_slam/device/Hik_MV-CH120-60UC/ros2_ws/install/hik_bringup/share/hik_bringup/rviz/stereo_view.rviz \
  ~/LuXi_StereoCamera_ws/hik_stereo_view.rviz

rviz2 -d ~/LuXi_StereoCamera_ws/hik_stereo_view.rviz
```

配置源文件位于设备端：

```text
install/hik_bringup/share/hik_bringup/rviz/stereo_view.rviz
```

该配置已选择左右 JPEG 压缩预览，QoS 为 `Best Effort`。如果手动添加 RViz 的 `Image` 显示项，请选择 `/stereo/preview/left_color/compressed`、`/stereo/preview/right_color/compressed`，并将可靠性设置为 `Best Effort`。

如果宿主机看不到话题，可先检查：

```bash
ros2 topic list | rg 'stereo/preview'
ros2 topic hz /stereo/preview/left_color/compressed
```

## 验证与排障

查看节点和话题：

```bash
ros2 node list
ros2 topic list | rg 'left_camera|right_camera|stereo'
```

确认深度图已输出：

```bash
ros2 topic echo --once /stereo/depth sensor_msgs/msg/Image --qos-profile sensor_data
```

检查压缩预览帧率：

```bash
ros2 topic hz /stereo/preview/left_color/compressed
```

启动日志中偶尔出现 `sequence size exceeds remaining buffer`，这是 MVS SDK 的诊断输出；只要相机成功打开且深度节点持续打印 `Processed frame`，该信息不会阻断图像和深度发布。

## 重新构建

源码、CMake 或 launch 文件有改动后：

```bash
cd /home/nvidia/lunar_slam/device/Hik_MV-CH120-60UC/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select hikrobot_camera_driver stereo_depth hik_bringup
source install/setup.bash
```

如需完全重新构建，只清理标准目录 `build/`、`install/`、`log/`，然后重新运行安装脚本或 `colcon build`。不要创建或恢复 `build_humble/`、`install_humble/`、`log_humble/`。
