# Prophesee EVK4 HD 使用说明

本文档面向当前工作空间：

```bash
/home/changxin/Ultimate SLAM
```

当前系统环境：

- Ubuntu 26.04
- ROS 2 Lyrical
- Prophesee EVK4 HD，设备 ID：`04b4:00f5`
- 底层 ROS 驱动：`ros-event-camera/metavision_driver`
- OpenEB / MetaVision SDK：工作空间本地 vendor，版本 `5.2.0`

## 1. 功能包定位

`evk4_driver` 是 EVK4 HD 的封装功能包。它不改动上游 `metavision_driver` 的核心驱动代码，而是在外层统一管理：

- EVK4 HD 默认参数
- 驱动启动入口
- demo 示例入口
- USB 权限规则
- 环境加载脚本
- 中文使用文档

这样后续开发 Ultimate SLAM 时，可以把相机驱动、测试 demo、SLAM 算法节点分开维护。

## 2. 目录结构

```text
src/evk/evk4_driver
├── config
│   └── driver
│       └── evk4_hd.params.yaml          # EVK4 HD 驱动默认参数
├── launch
│   ├── driver
│   │   └── driver.launch.py             # 驱动层：只负责启动相机
│   ├── demo
│   │   ├── compare.launch.py            # demo 层：对比 time_slice 与 sharp
│   │   ├── info.launch.py               # demo 层：查看 MetaVision 平台信息
│   │   ├── record.launch.py             # demo 层：启动相机并录制 rosbag
│   │   └── view.launch.py               # demo 层：启动相机并渲染事件图像
│   ├── evk4_driver.launch.py            # 兼容入口，转发到 launch/driver
│   ├── evk4_compare.launch.py           # 兼容入口，启动双渲染对比
│   ├── evk4_record.launch.py            # 兼容入口，转发到 launch/demo
│   └── evk4_view.launch.py              # 兼容入口，启动驱动和事件渲染器
├── scripts
│   ├── install_udev_rules.sh            # 安装 EVK4 USB 权限规则
│   └── setup_environment.bash           # 加载 EVK4 运行环境
├── udev
│   └── 99-prophesee-evk4.rules          # EVK4 udev 规则
├── docs
│   └── EVK4_HD_使用说明.md
├── CMakeLists.txt
└── package.xml
```

推荐后续新增内容时继续保持分层：

- 相机驱动配置放在 `config/driver`
- 相机驱动启动文件放在 `launch/driver`
- 验证、录包、可视化等示例放在 `launch/demo`
- 与硬件环境相关的脚本放在 `scripts`

## 3. 加载环境

每次打开新终端后执行：

```bash
cd "/home/changxin/Ultimate SLAM"
source setup_evk4_driver.bash
```

根目录的 `setup_evk4_driver.bash` 只是兼容入口，实际会加载：

```bash
src/evk/evk4_driver/scripts/setup_environment.bash
```

该脚本会配置：

- ROS 2 Lyrical
- 本地 OpenEB / MetaVision SDK vendor
- `event_camera_msgs`
- `metavision_driver`
- `evk4_driver`
- `MV_HAL_PLUGIN_PATH`
- 本地解包工具路径

## 4. 安装 USB 权限规则

首次使用或更换机器后执行一次：

```bash
cd "/home/changxin/Ultimate SLAM"
bash scripts/install_evk4_udev_rules.sh
```

根目录脚本会转发到：

```bash
src/evk/evk4_driver/scripts/install_udev_rules.sh
```

执行完成后拔插 EVK4，然后检查：

```bash
lsusb | grep -i "04b4:00f5"
ls -l /dev/bus/usb/002/003
```

如果运行相机时出现 `LIBUSB_ERROR_ACCESS`，说明 USB 权限规则还没有生效。

## 5. 构建

推荐使用默认主链路构建：

```bash
cd "/home/changxin/Ultimate SLAM"
source setup_evk4_driver.bash
colcon build --symlink-install --allow-overriding event_camera_msgs \
  --cmake-args -DBUILD_TESTING=OFF
source setup_evk4_driver.bash
```

当前默认参与构建的包是：

```text
event_camera_msgs
event_camera_codecs
event_camera_renderer
metavision_driver
evk4_driver
```

`openeb_vendor` 当前由工作空间本地 vendor 提供，源码目录默认忽略，避免重复构建系统级 SDK。事件渲染相关的 `event_camera_codecs` 和 `event_camera_renderer` 已加入构建链路，可直接用于可视化 demo。

## 6. 启动驱动

推荐使用顶层入口。ROS 2 会按文件名在包的 share 目录中查找 launch 文件，因此用户命令保持简短，包内代码仍按 `driver` 和 `demo` 分层。

```bash
ros2 launch evk4_driver evk4_driver.launch.py
```

指定当前 EVK4 序列号：

```bash
ros2 launch evk4_driver evk4_driver.launch.py serial:=00052316
```

未接外部触发线时关闭触发输入：

```bash
ros2 launch evk4_driver evk4_driver.launch.py trigger_in_mode:=disabled
```

常用参数：

```text
camera_name                 默认 event_camera，决定节点名和话题命名空间
serial                      相机序列号，留空时打开第一台可用相机
frame_id                    事件消息 header.frame_id
trigger_in_mode             external / loopback / disabled
use_multithreading          是否分离 SDK 回调线程与 ROS 发布线程
statistics_print_interval   驱动统计信息打印周期
```

## 7. 查看相机信息 demo

```bash
ros2 launch evk4_driver info.launch.py
```

等价于运行：

```bash
metavision_platform_info
```

能看到 `Prophesee IMX636 HD`、`Serial`、`1280 x 720`、`EVT3` 等信息，就说明 OpenEB 能正常识别相机。

## 8. 录制事件 demo

```bash
ros2 launch evk4_driver evk4_record.launch.py output:=evk4_events
```

默认录制：

```text
/event_camera/events
```

当前 ROS 2 Lyrical 的 `ros2 bag record` 语法需要使用 `--topics` 指定话题，demo 已按该语法封装。

## 9. 验证 ROS 事件流

另开一个终端：

```bash
cd "/home/changxin/Ultimate SLAM"
source setup_evk4_driver.bash
ros2 topic list
ros2 topic hz /event_camera/events
```

驱动日志中的含义：

```text
bw in      相机输入带宽
msgs/s in  SDK 输入消息频率
out        发送给 ROS 订阅者的消息数量
```

没有订阅者时 `out: 0` 是正常现象。

## 10. 可视化事件图像 demo

启动相机和事件渲染器：

```bash
ros2 launch evk4_driver evk4_view.launch.py serial:=00052316
```

另开一个终端查看图像：

```bash
cd "/home/changxin/Ultimate SLAM"
source setup_evk4_driver.bash
ros2 run rqt_image_view rqt_image_view
```

在 `rqt_image_view` 中选择：

```text
/event_camera/image_raw
```

可用以下命令确认图像链路已工作：

```bash
ros2 topic hz /event_camera/image_raw
```

默认 `fps:=25.0` 时，输出频率应接近 25 Hz。请在相机前移动手或物体，事件图像才会出现明显纹理；静止场景下事件相机输出接近黑色属于正常现象。

也可以单独启动渲染器：

```bash
ros2 launch event_camera_renderer renderer.launch.py camera:=event_camera
```

渲染器只有检测到图像订阅者后才会订阅 `/event_camera/events`，所以打开 `rqt_image_view` 前看到 `waiting for subscribers` 是正常现象。

### 对比 time_slice 与 sharp

当前 EVK4 已针对室内背景活动将 `bias_diff_on` 和 `bias_diff_off` 设置为 `20`。实测静止输入带宽由约 `24 MB/s` 降到约 `0.6 MB/s`，移动目标时事件带宽会明显上升。

启动双渲染对比 demo：

```bash
ros2 launch evk4_driver evk4_compare.launch.py serial:=00052316 fps:=25.0
```

在 `rqt_image_view` 中分别选择：

```text
/event_camera/time_slice/image_raw
/event_camera/sharp/image_raw
```

- `time_slice`：固定聚合每帧时间内的所有事件，运动较快时可能出现较厚的边缘。
- `sharp`：自动调整保留的事件数量，通常能得到更细、更锐利的轮廓。

### 背景事件过多时的启动滤波

`trail_filter` 由相机驱动在启动阶段配置。必须作为 launch 参数传入，不能在节点运行后通过 `ros2 param set` 临时开启。

```bash
ros2 launch evk4_driver evk4_view.launch.py \
  serial:=00052316 \
  trigger_in_mode:=disabled \
  trail_filter:=true \
  trail_filter_type:=stc_cut_trail \
  trail_filter_threshold:=5000
```

该滤波用于减少时间上孤立或重复的事件，不能消除显示器刷新、PWM 灯闪烁等整幅视场的周期性亮度变化。测试时应避免让镜头正对显示器，并优先在自然光或稳定照明下使用纸质棋盘格验证。

EVK4 还支持可选的硬件防闪烁带阻滤波：

```bash
ros2 launch evk4_driver evk4_view.launch.py \
  serial:=00052316 \
  anti_flicker:=true \
  anti_flicker_low_frequency:=90 \
  anti_flicker_high_frequency:=110
```

当前测试环境中，`50-70 Hz` 和 `90-110 Hz` 带阻都没有显著降低背景事件率，因此默认保持关闭。

## 11. 常见问题

### glxinfo: not found

只影响 OpenGL 信息查询，不影响相机驱动。需要时安装：

```bash
sudo apt install mesa-utils
```

### Failed to retrieve installed Metavision packages list

当前 OpenEB 来自工作空间本地 vendor 解包，不是完整系统级 MetaVision 安装，因此该提示可以忽略。

### LIBUSB_ERROR_ACCESS

USB 权限不足。重新安装 udev 规则，拔插相机，再测试：

```bash
bash scripts/install_evk4_udev_rules.sh
```

### cannot open default camera

先确认平台工具能看到相机：

```bash
metavision_platform_info
```

如果平台工具正常，再检查是否有其他进程正在占用 EVK4。

### Package 'event_camera_renderer' not found

说明事件渲染包没有进入当前 install 空间。重新构建：

```bash
cd "/home/changxin/Ultimate SLAM"
source setup_evk4_driver.bash
colcon build --symlink-install --allow-overriding event_camera_msgs \
  --packages-select event_camera_codecs event_camera_renderer evk4_driver \
  --cmake-args -DBUILD_TESTING=OFF
source setup_evk4_driver.bash
```

验证：

```bash
ros2 pkg prefix event_camera_renderer
ros2 launch event_camera_renderer renderer.launch.py --show-args
```

## 12. 后续开发建议

后续接入 Ultimate SLAM 时，建议保持下面的依赖方向：

```text
EVK4 硬件
  -> openeb_vendor / MetaVision SDK
  -> metavision_driver
  -> evk4_driver
  -> slam 或 demo 功能包
```

`evk4_driver` 只保留相机 bringup、参数和 demo，不把 SLAM 算法逻辑混进来。这样相机层稳定后，算法层可以独立迭代。
