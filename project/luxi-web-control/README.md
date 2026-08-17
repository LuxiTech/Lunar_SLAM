# Luxi Web Control 启动说明

网页只负责遥控以及启动、停止建图；相机硬件需要在另一个终端中单独启动。

## 1. 进入正确目录

不要混用宿主机路径和容器路径。

- 在宿主机终端中，工程目录是：

  ```bash
  cd /home/lunar/project/lunar_slam
  ```

- 如果终端提示符类似 `lunar@lunar_slam:/workspace/lunar_slam$`，说明当前位于容器内，
  应使用：

  ```bash
  cd /workspace/lunar_slam
  ```

后续命令都必须在同一个工程根目录执行。先确认目录和安装文件：

```bash
pwd
test -d project -a -d scripts || echo "当前不是 lunar_slam 工程根目录"
test -f install/setup.bash || echo "当前工作区尚未构建"
```

如果出现 `install/setup.bash: No such file or directory`，就在当前目录先构建项目：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select \
  luxi_adapter luxi_visual_frontend luxi_rtab_map \
  luxi_3d_navigation luxi_web_control slam_d1_bridge
```

D455 首次使用时还必须先按照
[`device/D455/ros2_ws/src/lunar_d455_bringup/README.md`](../../device/D455/ros2_ws/src/lunar_d455_bringup/README.md)
构建 D455 官方 SDK 和驱动工作区。

## 2. 关闭所有旧进程

每次重新启动或切换相机前，在工程根目录执行：

```bash
bash scripts/stop_luxi_system.sh
```

看到下面的提示后才能继续：

```text
Luxi/D1 processes stopped, stale PID files removed, and port 8080 is available.
```

不要使用 `killall python3`，也不要通过改成 8081 来绕过旧网页进程。

## 3. 终端一：启动硬件

新开一个终端，进入与上面相同的工程根目录：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

根据实际相机只选择下面一个命令。

### D455 / D455F

```bash
ros2 launch luxi_adapter sensor_bringup.launch.py hardware:=d455
```

### D435i

```bash
ros2 launch luxi_adapter sensor_bringup.launch.py hardware:=d435i
```

### HIK 双目 + H30 IMU

```bash
ros2 launch luxi_adapter sensor_bringup.launch.py hardware:=hik
```

同一时间只能运行一个硬件 profile。不要另外手工启动 RealSense/HIK 驱动或第二套
`sensor_bringup.launch.py`。

D435i/D455 首次启动或更换支架后，将机器人放在水平面并保持静止，稍后在网页点击
“一键水平校准”。

## 4. 终端二：启动网页

硬件保持运行，再开一个终端，进入同一个工程根目录并执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch luxi_web_control lekiwi_web_control.launch.py \
  bind_address:=0.0.0.0 http_port:=8080
```

浏览器打开：

```text
http://机器人电脑IP:8080
```

查看本机 IP：

```bash
hostname -I
```

网页出现 RGB 预览后，RealSense 先点击“一键水平校准”，再点击“开始建图”。建图结束
必须点击“停止建图”，等待页面显示已经停止，以便 RTAB-Map 完整保存数据库。

只测试本机网页、不连接 D1 底盘时，可改用：

```bash
ros2 launch luxi_web_control web_control.launch.py \
  bind_address:=127.0.0.1 http_port:=8080
```

## 5. 关闭系统

正常关闭顺序：

1. 网页点击“停止行驶”；
2. 网页点击“停止建图”并等待保存完成；
3. 网页点击“停止定位”（如果已经启动定位）；
4. 终端二按 `Ctrl-C` 关闭网页；
5. 终端一按 `Ctrl-C` 关闭硬件；
6. 回到工程根目录执行一次最终清理：

   ```bash
   bash scripts/stop_luxi_system.sh
   ```

## 常见错误

### `install/setup.bash: No such file or directory`

当前目录不是已构建的工作区。先执行本文第 1 节的目录检查和构建命令。容器内应使用
`/workspace/lunar_slam`，宿主机才使用 `/home/lunar/project/lunar_slam`。

### `Package 'luxi_web_control' not found`

说明没有成功执行 `source install/setup.bash`，或者当前目录下的工作区尚未构建。不要
继续启动；返回第 1 节处理。

### 网页没有 RGB 预览或不能开始建图

检查终端一是否仍在运行，并确认只存在一套统一传感器发布者：

```bash
ros2 topic info /sensors/rgbd/rgbd_image --verbose
ros2 topic info /sensors/imu/data --verbose
```

两个话题都应显示 `Publisher count: 1`。
