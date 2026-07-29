# luxi_navigation

`luxi_navigation` 当前提供一个安全的 ROS 2 键盘遥控节点，将 WASD 按键转换为
`geometry_msgs/msg/Twist` 并发布到 `/cmd_vel`。

## 控制映射

| 按键 | `linear.x` | `angular.z` | 动作 |
| --- | ---: | ---: | --- |
| W | `+linear_speed` | 0 | 前进 |
| S | `-linear_speed` | 0 | 后退 |
| A | 0 | `+angular_speed` | 左转 |
| D | 0 | `-angular_speed` | 右转 |
| 空格 / X | 0 | 0 | 停车 |
| Q / Ctrl-C | 0 | 0 | 停车并退出 |

节点以固定频率持续发布当前速度。移动键超过 `command_timeout` 没有刷新时会自动发布
零速度；正常退出时也会重复发布零速度。键盘长按依赖终端的按键自动重复功能。

## 构建

```bash
cd /home/lunar/project/lunar_slam
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select luxi_navigation
source install/setup.bash
```

## 实车前检查

ROS 2 使用 DDS 自动发现，不需要把 `192.168.123.49` 写入节点参数。控制机与小车需要：

- 位于可互通的局域网；
- 使用相同的 `ROS_DOMAIN_ID`；
- `ROS_LOCALHOST_ONLY=0`；
- DDS 中间件及其发现配置兼容；
- 小车确实订阅 `geometry_msgs/msg/Twist` 类型的 `/cmd_vel`。

检查：

```bash
ping 192.168.123.49
export ROS_LOCALHOST_ONLY=0
# 如果小车不是默认 domain 0，在这里设置为小车的 domain：
export ROS_DOMAIN_ID=0
ros2 topic info /cmd_vel --verbose
```

必须看到小车的订阅者后再开始运动测试。首次测试应架空驱动轮或清空周围区域。

## 键盘控制

建议第一次使用更低速度：

```bash
source /opt/ros/humble/setup.bash
source /home/lunar/project/lunar_slam/install/setup.bash
ros2 run luxi_navigation keyboard_teleop --ros-args \
  -p linear_speed:=0.10 \
  -p angular_speed:=0.35 \
  -p command_timeout:=0.6
```

节点必须直接在交互式终端中运行。按住 W/S/A/D 运动，松开后最多 0.6 秒自动停止；
空格立即停车，Q 停车并退出。

可配置参数：

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `cmd_vel_topic` | `/cmd_vel` | 速度话题 |
| `linear_speed` | `0.15` | 前后速度，m/s |
| `angular_speed` | `0.5` | 转向角速度，rad/s |
| `publish_rate` | `20.0` | 发布频率，Hz |
| `command_timeout` | `0.6` | 按键失联停车时间，秒 |
| `stop_publish_count` | `3` | 退出前零速度发送次数 |

即使遥控节点包含超时停车，小车底盘控制器仍应配置自己的 `/cmd_vel` 看门狗。进程被
`SIGKILL`、主机掉电或网络中断时，发送端无法保证最后一条零速度消息能够到达。

## 本地测试

测试话题使用 `/luxi_navigation/test_cmd_vel`，不会接触实车 `/cmd_vel`：

```bash
cd /home/lunar/project/lunar_slam
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=119 colcon test --packages-select luxi_navigation
colcon test-result --verbose
```

## 手动标注地图障碍物

`manual_map_annotator` 使用 RViz 的 `Publish Point` 工具采集障碍物多边形。坐标直接保存
为 `map` 坐标系下的米制坐标，不依赖地图图片像素，因此不会引入图片 Y 轴翻转或分辨率
换算错误。

建议先停止增量建图并固化地图，再开始语义标注。若 RTAB-Map 在标注期间发生回环优化，
显示地图可能移动，已经点击的语义坐标就可能与最终地图错位。

### 输出

| 接口 | 类型 | 用途 |
| --- | --- | --- |
| `/semantic_annotation/markers` | `visualization_msgs/MarkerArray` | RViz 中显示红色已保存多边形和黄色草稿 |
| `/semantic_annotation/obstacle_mask` | `nav_msgs/OccupancyGrid` | 与输入地图完全对齐的障碍物二值掩码 |
| `semantic_obstacles.yaml` | YAML | 持久化障碍物 ID、名称和地图坐标多边形 |

掩码中障碍物为 `100`，其余位置为 `0`。它可作为后续 Nav2 keepout 数据源，但正式
接入 Nav2 Costmap Filter 时还需要增加 `CostmapFilterInfo` 发布和相应 Nav2 参数。

### 启动节点

使用当前 RTAB-Map 地图：

```bash
source /opt/ros/humble/setup.bash
source /home/lunar/project/lunar_slam/install/setup.bash

ros2 launch luxi_navigation manual_map_annotation.launch.py \
  map_id:=map004 \
  map_topic:=/rtabmap/map \
  output_path:=/home/lunar/project/lunar_slam/maps/occupancy_maps/map004_semantic.yaml
```

如果使用 Nav2 `map_server` 发布的固化 `/map`，使用：

```bash
ros2 launch luxi_navigation manual_map_annotation.launch.py \
  map_id:=map004 \
  map_topic:=/map \
  map_transient_local:=true \
  output_path:=/home/lunar/project/lunar_slam/maps/occupancy_maps/map004_semantic.yaml
```

当前项目 RTAB-Map RViz 配置已经加入 `Publish Point` 工具和
`/semantic_annotation/markers` 显示。RViz 的 Fixed Frame 必须为 `map`。

### 标注一个障碍物

先设置稳定 ID 和显示名称；不设置 ID 时会自动生成 `obstacle_001`：

```bash
ros2 param set /manual_map_annotator active_id rock_001
ros2 param set /manual_map_annotator active_label "岩石1"
ros2 service call /manual_map_annotator/start_polygon std_srvs/srv/Trigger "{}"
```

然后在 RViz 中选择 `Publish Point`，沿障碍物边界依次点击至少三个顶点。黄色线表示尚未
提交的草稿。完成后执行：

```bash
ros2 service call /manual_map_annotator/finish_polygon std_srvs/srv/Trigger "{}"
```

完成的多边形变为红色，并自动原子写入 YAML。常用编辑命令：

```bash
# 撤销最后一个顶点
ros2 service call /manual_map_annotator/undo_vertex std_srvs/srv/Trigger "{}"

# 放弃当前草稿
ros2 service call /manual_map_annotator/cancel_polygon std_srvs/srv/Trigger "{}"

# 列出已保存障碍物
ros2 service call /manual_map_annotator/list std_srvs/srv/Trigger "{}"

# 删除 active_id 指定的障碍物
ros2 param set /manual_map_annotator active_id rock_001
ros2 service call /manual_map_annotator/delete_obstacle std_srvs/srv/Trigger "{}"

# 手动保存或从磁盘重新加载
ros2 service call /manual_map_annotator/save std_srvs/srv/Trigger "{}"
ros2 service call /manual_map_annotator/reload std_srvs/srv/Trigger "{}"
```

标注完成后应同时检查 Marker 与障碍掩码是否落在正确墙体或障碍物上，并确认没有覆盖
需要通行的门口、走廊和导航目标位置。
