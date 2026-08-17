# D1 双相机 AprilTag 精确停靠方案

> 文档状态：实施前设计与可行性评审  
> 编写日期：2026-08-17  
> 当前硬件：顶部 D455 RGB-D；对地 USB 全局快门相机；D1 非全向底盘  
> 目标标记：`tagStandard41h12`、ID `0`、标称检测边长 `0.180 m`

## 1. 目标和结论

目标是在已有网页地图导航之后增加两阶段精确停靠：

1. 顶部 D455 识别 ID 0，并通过现有三维地图导航到预先验证的交接位；
2. 停止普通导航，切换到对地相机，利用 Tag 的中心、朝向和成像尺度做低速闭环；
3. 误差连续稳定达标后停车并报告 `DOCKED`；
4. Tag、图像、避障或控制心跳丢失时立即输出零速度，不沿用最后一帧命令。

该功能可行，但不能原样使用“`vx = K1*ey, vy = K2*ex`”方案，原因如下：

- 当前网页配置明确设置 `max_linear_y: 0.0`，D1 只能通过转向、前后运动或小弧线消除横向误差；
- 当前 D1 行走控制的最小有效平移速度约为 `0.10 m/s`，普通连续小增益 P 控制在终点附近可能无响应或产生过冲；
- 单纯的 `(u,v,theta)` 不能生成地图中的远距离安全路径，顶部相机阶段仍应复用现有 HLoc/ICP、三维 A*、局部重规划和深度避障；
- 对地相机 SDK、准确成像格式、内参、安装高度和外参尚未提供，所以当前可以完成接口设计、工程拆分和验收方案，不能声称真机停靠已经通过。

第一版推荐采用“固定地图交接位 + 顶部 Tag 确认 + 对地模板闭环”。完成相机标定后，再升级为“顶部 Tag 6DoF 位姿自动计算交接位”。

## 2. 已核对的本机现状

### 2.1 顶部相机和导航接口

`luxi_adapter` 已把 D455 统一成以下公共接口：

| 用途 | ROS 2 话题 |
|---|---|
| 彩色图像 | `/sensors/rgbd/color/image_raw` |
| 彩色相机内参 | `/sensors/rgbd/color/camera_info` |
| 注册深度 | `/sensors/rgbd/depth/image_raw` |
| RGB-D | `/sensors/rgbd/rgbd_image` |
| IMU | `/sensors/imu/data_raw`、`/sensors/imu/data` |

D455 当前配置为 `848x480@30`。顶部 AprilTag 检测应直接订阅上述公共话题，不得绑定 RealSense 原生 `/camera/camera/*`，这样以后更换顶部相机时控制层不需要修改。

已有保存地图导航链为：

```text
/navigation/goal_pose
        -> octomap_3d_astar_planner
        -> /navigation/global_path
        -> local_path_replanner
        -> /navigation/planned_path
        -> terrain_path_follower
        -> /navigation/cmd_vel_raw
        -> navigation_safety_gate
        -> /navigation/cmd_vel
        -> velocity_command_mux
        -> /cmd_vel
```

已有控制/状态接口包括：

| 接口 | 含义 |
|---|---|
| `/navigation/start` | 显式允许路径跟随 |
| `/navigation/stop` | 停止路径跟随 |
| `/navigation/active` | 跟随器是否拥有导航状态 |
| `/navigation/follower_state` | `plan_ready`、`active`、`goal_reached`、`localization_lost` 等 |
| `/navigation/emergency_stop` | 网页人工急停 |
| `/luxi_location/health` | 定位健康状态 |
| `/navigation/local_obstacles/state` | D455 深度近场障碍状态 |

`velocity_command_mux` 当前只有“网页手动”和“普通导航”两个输入，导航命令超时为 `0.3 s`，手动非零命令会中止导航。停靠功能不能成为第三个直接发布 `/cmd_vel` 的节点，否则两个发布者会互相覆盖且无法证明急停优先级。

### 2.2 对地 USB 相机枚举结果

本机当前识别到：

```text
USB ID: 32e4:2234
产品名: Global Shutter Camera
驱动:   uvcvideo
采集节点: /dev/video6
伴随节点: /dev/video7（当前没有 capture 能力）
稳定路径: /dev/v4l/by-id/usb-Global_Shutter_Camera_Global_Shutter_Camera_01.00.00-video-index0
USB 总线: USB 2.0
```

适配时必须优先使用 `/dev/v4l/by-id/...-video-index0`，不能把 `/dev/video6` 写死，因为重启和插拔可能改变编号。本机暂缺 `v4l2-ctl` 和 `ffmpeg`，且厂家 SDK 尚未放入 `device/`，因此以下信息仍需在 SDK 到位后确认：

- 支持的像素格式、分辨率、帧率和是否能输出硬件时间戳；
- 镜头焦距、畸变模型、曝光/增益控制范围；
- `/dev/video7` 是否承载元数据或厂家控制；
- SDK 与 UVC 是否能二选一或必须使用厂家接口；
- D455 与该 USB 2.0 相机同时工作时的带宽、掉帧率和 CPU 占用。

本机是 `aarch64`。在已 source 的 ROS 2 Humble 和本项目工作区中，当前只有 `apriltag_msgs`，没有安装 `apriltag`、`apriltag_ros` 和 `camera_calibration`。因此实施时应把经过验证的上游版本固定到源码提交，纳入依赖构建和许可证记录，不能依赖开发机上偶然存在的系统包。

## 3. AprilTag 接口事实和打印件检查

官方 AprilTag 3 推荐大多数场景使用 `tagStandard41h12`。推荐的 ROS 2 包 `christianrauch/apriltag_ros` 支持 `Standard41h12`，订阅同步的 `image_rect` 和 `camera_info`，发布 `apriltag_msgs/msg/AprilTagDetectionArray` 以及可选 `/tf`。检测消息包含 `family`、`id`、`hamming`、`decision_margin`、中心、四角点和单应矩阵。

参考：

- [AprilRobotics/apriltag](https://github.com/AprilRobotics/apriltag)
- [AprilRobotics/apriltag-imgs](https://github.com/AprilRobotics/apriltag-imgs)
- [christianrauch/apriltag_ros](https://github.com/christianrauch/apriltag_ros)

配置中家族名称应写成包装器接受的 `Standard41h12`，不是把 `tagStandard41h12` 原样填入。只配置 ID 0，建议初始 `max_hamming: 0`：

```yaml
apriltag:
  ros__parameters:
    image_transport: raw
    qos_profile: sensor_data
    family: Standard41h12
    size: 0.18
    max_hamming: 0
    pose_estimation_method: pnp
    detector:
      threads: 2
      decimate: 1.0
      blur: 0.0
      refine: 1
      sharpening: 0.25
      debug: 0
    tag:
      ids: [0]
      frames: [dock_tag_0]
      sizes: [0.18]
```

注意：官方定义的 `size` 是四个检测角点之间的物理边长，即黑白边界交界处的边长，不是 A4 纸外沿，也不一定是整块黑色图案外沿。实施前要用尺重新测量并把实测值写入配置。打印设置必须为 100% 实际尺寸，纸面应平整、哑光、无覆膜反光、四周留足空白。卷曲、污损、遮挡和重复出现 ID 0 都应视为异常。

## 4. 总体架构

```text
                         普通导航阶段
网页/任务请求 -> docking_mission_node -> /navigation/goal_pose + start/stop
                                      -> 现有定位/规划/深度避障

D455 RGB + CameraInfo -> top apriltag_node -> /docking/top/detections
                                                   |
                                             交接条件判断
                                                   v
                         对地精调阶段
对地相机 SDK -> ground_camera_adapter -> image_rect + CameraInfo
                                      -> ground apriltag_node
                                      -> /docking/ground/detections
                                                   |
                                      docking_controller
                                                   |
                                      /docking/cmd_vel_raw
                                                   |
                                      docking_safety_gate
                                                   |
                                      /docking/cmd_vel_safe
                                                   |
                              扩展 velocity_command_mux
                                                   |
                                                /cmd_vel
```

速度优先级固定为：

```text
急停 > 网页非零手动命令 > 停靠命令 > 普通导航命令 > 零速度
```

进入停靠精调前必须：发布 `/navigation/stop=true`，等待 `/navigation/active=false`，确认最终 `/cmd_vel` 为零并稳定至少 `0.5 s`，再置 `/docking/active=true`。退出停靠时先撤销停靠 active 并持续发布一小段零速度，然后才允许新导航任务。任一时刻只能有一个自动控制源获得速度权限。

## 5. 两阶段定位策略

### 5.1 第一版：固定交接位，推荐先实现

为 Tag ID 0 人工记录两个量：

- `staging_pose_map`：对地相机刚好能稳定看到完整 Tag 的地图位姿；
- `ground_target`：机器人位于最终正确停车位置时，对地图像中的目标特征。

任务开始后：

1. 检查保存地图、定位、D455、深度避障和底盘状态；
2. 可先用顶部相机连续多帧确认 ID 0 存在，但远距离找不到 Tag 时不应盲目原地旋转；
3. 向 `/navigation/goal_pose` 发布 `staging_pose_map`；
4. 等待 `/navigation/follower_state=plan_ready` 后发布 `/navigation/start=true`；
5. 由现有地图导航到交接位；
6. 顶部 Tag 的稳定观测可作为额外交接确认，但不能替代避障和地图路径；
7. 完成“停导航—零速—对地可见”握手后进入精调。

这种方式最适合首版，因为地图坐标来自人工验证，不依赖尚未标定的顶部相机外参，也不会让机器人朝图像中的 Tag 直线穿过障碍。

### 5.2 升级版：由顶部 Tag 位姿生成交接位

顶部相机完成内参和 `base_link -> top_camera_optical_frame` 外参标定后，可利用 `size=0.18 m` 的 PnP 位姿：

```text
T_map_tag = T_map_base * T_base_top_camera * T_top_camera_tag
T_map_staging = T_map_tag * T_tag_staging_offset
```

必须先对多帧位姿做异常值剔除和稳定性判定，再生成地图目标。单帧 PnP、斜视角过大或 Tag 边长小于设定像素阈值时不能更新目标。目标还需经过现有地形可行性检查和 A* 规划，绝不能直接将相机误差变成远距离 `/cmd_vel`。

## 6. 对地相机最终视觉伺服

### 6.1 目标模板

人工把机器人移动到正确停车姿态，连续采集 1～2 秒稳定检测，不采用单帧值。保存：

```yaml
targets:
  0:
    camera_serial: Global_Shutter_Camera_Global_Shutter_Camera_01.00.00
    image_width: 640                 # 示例，必须由实际驱动填写
    image_height: 480                # 示例，必须由实际驱动填写
    target_u: null                   # 多帧中位数
    target_v: null
    target_theta_rad: null
    target_side_px: null             # 四边长度的稳健均值
    target_area_px2: null
    measured_tag_size_m: null        # 重新实测检测边长
    sample_count: 30
```

不要硬编码示例中的 `640, 380, 0°`。相机不在机器人几何中心时，人工记录的模板本来就可以偏离图像中心。

运行时计算：

```text
e_u     = u - target_u
e_v     = v - target_v
e_theta = wrapToPi(theta - target_theta)
e_scale = log(side_px / target_side_px)
```

`theta` 应由同一条已定义的 Tag 边和角点顺序计算，并做 `[-pi, pi)` 环绕。仅使用 `e_v` 判断前后距离对相机俯仰和安装偏差较敏感，因此第一版也应同时保存 `side_px` 或面积；相机标定完成后优先使用 PnP 的平面相对位姿 `(x,y,yaw)`。

### 6.2 非全向底盘控制

D1 不执行 `linear.y`，控制命令中必须始终令 `linear.y=0`。建议采用分段、小步、每步重观察的控制，而非一条连续三通道 P 公式：

1. `ALIGN_YAW`：原地旋转，先把朝向误差降到进入阈值；
2. `CORRECT_LATERAL`：根据 `e_u` 执行有界的小角度转向—短距离前/后移—反向回正，即小 S 形或弧线修正；
3. `APPROACH`：朝向稳定后，依据 `e_scale` 和 `e_v` 做短距离前后脉冲；
4. `FINE_ALIGN`：减小单次脉冲时长，每个脉冲后停稳、重新检测，再决定下一步；
5. `VERIFY`：全部误差连续稳定 `0.5～1.0 s` 后进入 `DOCKED`。

底盘最小有效平移速度约为 `0.10 m/s`，因此精细程度主要由脉冲持续时间、落脚误差和图像延迟决定。初始参数建议只作为台架起点：

| 参数 | 初始上限/范围 | 说明 |
|---|---:|---|
| 平移命令幅值 | `0.10 m/s` | 不再继续降低幅值 |
| 精调平移脉冲 | `0.10～0.25 s` | 对应理想路程约 1～2.5 cm，必须真机验证 |
| 旋转上限 | `0.15 rad/s` | 低于普通导航的 `0.35 rad/s` |
| 动作后静止观察 | `0.25～0.50 s` | 避免运动模糊和机身摆动 |
| 图像超时 | `0.20 s` | 超时立即零速 |
| Tag 丢失确认 | 1～2 帧 | 只用于抑制单帧抖动，丢失期间始终零速 |

方向符号不得凭代码假设。首次轮子离地/命令重映射测试时，分别施加正 `linear.x` 和正 `angular.z`，记录 Tag 的 `u/v/theta/side` 变化，再锁定符号。

### 6.3 停车判定

像素阈值必须换算成实际毫米后确定。建议第一轮把以下值作为待标定目标，而不是保证值：

- `|e_u| <= 10 px`；
- `|e_v| <= 10 px` 或 `|e_scale| <= 0.02`；
- `|e_theta| <= 2°`；
- 连续满足 `0.8 s`，期间至少有 8 帧有效检测；
- 最终实测位置误差目标先定为 `<= 30 mm`，偏航 `<= 3°`，经重复试验后再收紧。

如果 D1 步态的最小位移大于阈值所对应的地面距离，应接受机械能力可达到的阈值，或在底盘 SDK 层增加真正的微动接口；不能仅继续减小 ROS 速度数字。

## 7. 状态机和故障行为

```text
IDLE
  -> PRECHECK
  -> COARSE_GOAL_SENT
  -> COARSE_NAV
  -> HANDOFF_STOP
  -> GROUND_ACQUIRE
  -> ALIGN_YAW
  -> CORRECT_LATERAL / APPROACH
  -> FINE_ALIGN
  -> VERIFY
  -> DOCKED

任意运动状态 -> ESTOP
精调时 Tag/图像丢失 -> LOST_HOLD -> 重新捕获或 ABORTED
导航/规划/相机/底盘故障 -> ABORTED
```

关键规则：

- `PRECHECK` 要求 D1 已站立、映射任务未运行、定位健康、顶部图像新鲜、深度避障健康、无急停；
- 第一版 `GROUND_ACQUIRE` 若看不到完整 ID 0，只原地保持零速度并超时退出，不自动盲搜；交接位应保证 Tag 已进入对地视野；
- 顶部相机在普通导航时短暂丢 Tag，可以继续前往已知固定交接位；若任务使用动态 Tag 位姿，丢失后禁止更新目标；
- 对地精调期间 Tag 丢失、检测时间戳过期、出现两个 ID 0、`hamming` 超限或质量过低时，当前控制周期立即输出零；
- 任何控制节点异常退出后，速度看门狗必须在 `<=0.3 s` 内把 `/cmd_vel` 置零；
- 网页非零手动命令和急停均抢占停靠，并将任务置为 `ABORTED/ESTOP`，不能自动恢复运动；
- `DOCKED` 状态持续输出零速度，直到明确取消或开始新任务。

## 8. ROS 2 工程代码规划

### 8.1 厂家 SDK 和对地相机适配

SDK 到位后建议目录：

```text
device/GroundCamera/
├── README.md
├── sdk/                         # 厂家原始 SDK，记录版本/校验值/许可证
└── ros2_ws/
    └── src/<vendor_driver>/     # 厂家驱动或最薄 ROS 2 包装

project/luxi_ground_camera_adapter/
├── config/ground_camera.yaml
├── launch/ground_camera.launch.py
├── src/ground_camera_adapter_node.cpp
├── test/
└── README.md
```

厂家驱动只负责访问设备；`luxi_ground_camera_adapter` 负责稳定设备选择、话题命名、时间戳/帧名、灰度转换、内参发布、诊断和可选压缩预览。它必须能与顶部 `luxi_adapter` 同时运行，不能成为 `hardware:=d455/hik/...` 中互斥的主传感器选项。

公共接口建议：

| 方向 | 话题 | 类型 |
|---|---|---|
| 输出 | `/docking/ground/image_raw` | `sensor_msgs/msg/Image`，优先 `mono8` |
| 输出 | `/docking/ground/camera_info` | `sensor_msgs/msg/CameraInfo` |
| 输出 | `/docking/ground/image_raw/compressed` | `sensor_msgs/msg/CompressedImage`，仅调试/网页 |
| 输出 | `/docking/ground/status` | `diagnostic_msgs/msg/DiagnosticArray` |
| TF | `base_link -> ground_camera_link -> ground_camera_optical_frame` | 静态 TF |

驱动必须保留采集时间戳，不能在多个转发节点中反复改成“当前时间”。`Image` 和 `CameraInfo` 的时间戳、分辨率、frame ID 必须匹配，否则 `apriltag_ros` 的相机订阅和 PnP 位姿会失效。

### 8.2 AprilTag 和停靠功能包

建议新增：

```text
project/luxi_apriltag_docking/
├── config/
│   ├── apriltag_top.yaml
│   ├── apriltag_ground.yaml
│   ├── docking_controller.yaml
│   └── docking_targets.yaml
├── include/luxi_apriltag_docking/
│   ├── tag_observation.hpp
│   ├── docking_state_machine.hpp
│   └── differential_visual_servo.hpp
├── src/
│   ├── docking_mission_node.cpp
│   ├── docking_controller_node.cpp
│   └── docking_safety_gate_node.cpp
├── launch/apriltag_docking.launch.py
├── test/
└── README.md
```

启动两个独立命名空间的 `apriltag_node`：

| 检测器 | 图像 remap | CameraInfo remap | 输出 |
|---|---|---|---|
| top | `/sensors/rgbd/color/image_raw` | `/sensors/rgbd/color/camera_info` | `/docking/top/detections` |
| ground | `/docking/ground/image_raw` | `/docking/ground/camera_info` | `/docking/ground/detections` |

两个实例都只接受 `Standard41h12/ID 0`。控制器仍要再次检查 family、ID、hamming、时间戳、四角点是否在图内及 `decision_margin`；质量阈值通过现场数据分布确定，不直接照抄网络经验值。

停靠公共接口建议：

| 方向 | 接口 | 说明 |
|---|---|---|
| 输入 | `/docking/request` | 开始 ID 0 停靠；最终建议自定义 Action |
| 输入 | `/docking/cancel` | 取消并零速 |
| 输出 | `/docking/active` | transient-local 控制权状态 |
| 输出 | `/docking/state` | 当前状态和原因 |
| 输出 | `/docking/cmd_vel_raw` | 未经安全审核的控制量 |
| 输出 | `/docking/debug_image/compressed` | 角点、目标模板和误差叠加图 |
| 服务 | `/docking/capture_target` | 仅在零速、连续稳定检测时保存模板 |

正式网页接入推荐建立 `luxi_docking_msgs`，提供 `Dock.action`、`DockingStatus.msg` 和 `CaptureTarget.srv`，避免用自由格式字符串解析任务结果。Action goal 至少包含 `tag_id` 和目标配置名；feedback 包含状态、相机来源、误差、检测新鲜度和安全门状态；result 包含成功/取消/故障原因。

### 8.3 速度仲裁和安全门改造

扩展 `project/luxi_3d_navigation` 的 `velocity_command_mux`：

- 新增 `/docking/cmd_vel_safe` 和 `/docking/active`；
- 新增约 `0.2 s` 的停靠命令看门狗；
- 手动非零命令同时取消普通导航和停靠；
- 急停清除所有自动 active；
- 状态切换时丢弃切换前缓存命令，直到新 active 后收到新命令；
- 为优先级、超时、切换、急停、析构零速增加单元测试。

不能直接复用当前 `navigation_safety_gate` 而不改逻辑，因为它要求局部规划器和全局定位状态持续健康，而最终对地伺服并不依赖规划路径。应增加停靠专用安全门，至少检查：

- 命令、对地图像和 Tag 观测新鲜度；
- 顶部 D455 的近场障碍状态仍为 fresh/clear；
- D1 姿态和急停状态；
- `linear.y` 恒为零；
- 速度和脉冲持续时间上限；
- 控制状态与命令类型一致，例如 `ALIGN_YAW` 禁止平移；
- 节点退出、超时、状态非法时输出零。

### 8.4 网页规划

首版网页只承担任务触发和可观测性，不在 JavaScript 内做运动控制：

- 增加“精确停靠 ID 0”“取消停靠”“记录当前正确停车模板”按钮；
- 显示顶部/对地检测状态、当前状态、`e_u/e_v/e_theta/e_scale`、安全门原因；
- 显示对地调试图，默认限帧率，避免占满 USB/CPU；
- 停靠 active 时禁用普通导航启动；手动控制一旦非零即显示“已人工接管并取消停靠”；
- 急停沿用 `/navigation/emergency_stop`，不创建第二套急停语义；
- README 最终补充正确工作区、构建、启动、停止旧进程和验收命令。

## 9. 相机 SDK 到位后的接入步骤

1. 把 SDK 放到 `device/GroundCamera/sdk/`，保留原压缩包校验值、版本号和许可证，不修改唯一原件；
2. 阅读 SDK 的 ARM64、Ubuntu 22.04、Jetson、ROS 2 Humble 支持情况，确认运行库和 udev 规则；
3. 先在机器人不运动时运行厂家示例，确认序列号、格式、曝光、帧率和重启恢复；
4. 若 UVC 已满足要求，优先使用标准接口；只有曝光、同步或像素格式必须依赖厂家能力时才包 SDK；
5. 实现独立 ROS 2 驱动/适配器，发布规范图像、内参和诊断；
6. 使用 `camera_calibration` 标定对地相机，保存与序列号、分辨率绑定的 YAML；
7. 测量相机相对 `base_link` 的位置与朝向，发布 optical frame 规范的静态 TF；
8. 固定曝光/增益或设置受控范围，避免 Tag 黑白边界因自动曝光周期性漂移；
9. 同时启动 D455 和对地相机，记录 10 分钟帧率、时间戳、USB 错误、CPU、内存和温度；
10. 再接 AprilTag 检测，不接底盘速度，完成静态验收后才进入控制测试。

## 10. 测试计划

### T0：算法离线测试

- 使用官方 ID 0 图片生成旋转、缩放、模糊、亮暗、透视和部分出框样本；
- 验证只接受 `Standard41h12/0`，拒绝其他 ID、过大 hamming 和重复 ID 0；
- 验证中心、角点、`theta` 环绕、尺度误差和中位数滤波；
- 用 rosbag 回放真实图像，处理速度不得长期落后于输入帧率。

### T1：相机接入验收

- 稳定路径选择正确，重启、拔插后仍绑定同一序列号；
- `Image` 与 `CameraInfo` 分辨率、时间戳和 frame ID 一致；
- 连续 10 分钟无驱动崩溃，统计实际 FPS、p95 延迟和丢帧；
- D455 与对地相机同时工作，无 USB 降速或不可接受掉帧；
- 覆盖现场照明、阴影、反光和机器人振动。

建议门槛：检测链有效输出 `>=15 Hz`、端到端 p95 延迟 `<150 ms`、控制所用帧年龄 `<200 ms`。若设备实际能力不支持，应基于制动距离重新评估，不可只放宽超时。

### T2：静态几何测试

- 在地面建立已知毫米网格和角度刻度；
- 在多个高度、前后、左右和偏航位置记录检测率与误差；
- 检查 Tag 完整进入对地视野的交接区域，形成 `staging_pose` 容差图；
- 连续记录正确停车位置并重复生成模板，评估日间、夜间和曝光变化；
- 比较像素模板与 PnP 位姿的重复性，再决定正式控制量。

### T3：无运动闭环测试

- 将最终输出 remap 到 `/docking/test_cmd_vel`，不得连接 `/cmd_vel`；
- 人工移动 Tag/机器人，检查状态迁移、速度方向、限幅和脉冲结束；
- 注入图像过期、Tag 丢失、重复 ID、节点退出、定位丢失、障碍和急停；
- 对所有故障测量从故障发生到零命令的时间，要求 `<=0.3 s`；
- 验证手动非零命令必然抢占，停靠不会自动恢复。

### T4：底盘方向和最小动作测试

- 机器人架空或置于可靠防护条件下，逐轴确认正负号；
- 地面测试只执行单个低速短脉冲，测量实际位移、偏航、响应延迟和停稳时间；
- 建立最小可重复前进、后退、旋转和 S 形修正动作；
- 根据实测结果设置脉冲时长和误差滞回，避免在阈值边缘来回摆动。

### T5：对地闭环测试

- 先从小误差、Tag 始终完整可见的位置开始；
- 每扩大一次初始误差范围前，上一范围至少连续成功 10 次；
- 人员手持急停，地面设置安全边界，控制区内不放人员或脆弱物品；
- 测量最终毫米误差、角度误差、耗时、脉冲次数和最大过冲；
- Tag 丢失时机器人必须保持停止，首版不做自主搜索。

### T6：完整任务验收

覆盖“网页请求 -> 保存地图导航 -> 交接 -> 对地精调 -> DOCKED”：

- 从至少 5 个不同地图起点运行，每个起点至少 4 次；
- 暂定通过条件为 20 次连续任务无碰撞、无控制权冲突，成功率 100%；
- 成功任务最终误差暂定 `<=30 mm`、偏航 `<=3°`，稳定 `>=0.8 s`；
- 障碍、急停、相机拔出、Tag 移走和节点杀死测试全部在 `<=0.3 s` 输出零；
- 保存 rosbag、节点日志、最终误差表和网页截图，之后才能在 README 标记“真机测试通过”。

## 11. 实施顺序和交付物

### 里程碑 A：设备接入

- `device/GroundCamera` SDK 归档和说明；
- `luxi_ground_camera_adapter`、标定 YAML、静态 TF；
- 双相机同时运行报告和 rosbag。

### 里程碑 B：只检测、不运动

- 固定提交版本的 `apriltag` 与 `apriltag_ros` 源码/依赖（当前本机尚未安装）；
- 顶部和对地两套参数、话题 remap、ID 过滤；
- 调试图、观测质量指标、离线与静态测试。

### 里程碑 C：控制与安全

- 差速视觉伺服、状态机、目标模板服务；
- 停靠安全门和三通道速度仲裁；
- 单元测试、故障注入、无运动输出验证。

### 里程碑 D：真机和网页

- 分级真机测试与参数整定；
- 网页开始/取消/状态/调试画面；
- 完整验收记录；
- 更新 `project/luxi-web-control/README.md` 的构建、启动、停止旧进程、校准和测试步骤。

## 12. 当前待补信息

后续放入相机 SDK 时，需要一并确认或提供：

1. SDK 厂家、型号、版本、ARM64 库和示例；
2. 相机安装照片，镜头朝向、距地高度和相对 `base_link` 的大致位置；
3. Tag 的 `180 mm` 是哪两条边界之间的实测尺寸；
4. 最终停车允许的毫米/角度误差，以及是否允许机器人压住或跨过 Tag；
5. D1 是否有厂家级微动/步长控制接口，后退和原地转向的最小可重复动作；
6. ID 0 的地图位置是否固定。如果固定，优先记录交接位；如果会移动，里程碑 B 就必须完成顶部 6DoF 和外参标定。

在上述信息和 SDK 未到位前，下一步最有价值的工作是准备 `luxi_ground_camera_adapter` 的空接口、锁定第三方依赖版本，并用 D455/离线图片把 `Standard41h12/ID 0` 检测链跑通；不应提前连接真实 `/cmd_vel`。
