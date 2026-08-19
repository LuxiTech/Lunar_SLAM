# RTAB-Map 手动语义标注、定位与 Nav2 导航适配规划

## 1. 文档目标

本文规划当前 D435i + RTAB-Map 工程从“完成几何建图”扩展到以下完整流程：

1. 固化 RTAB-Map 建图结果；
2. 在二维地图上进行手动语义标注；
3. 使用固化后的 RTAB-Map 数据库进行纯定位；
4. 将定位、静态地图和障碍物数据接入 Nav2；
5. 将“房间、充电桩、禁行区”等语义转换为可执行的导航任务。

本规划只描述适配边界、接口和实施顺序，不要求修改 RTAB-Map、RealSense 或 Nav2
第三方源码。新增功能应放在 `/home/lunar/project/lunar_slam/project` 下的项目自有
ROS 2 包中。

## 2. 当前基础

当前工程已经具备：

- D435i 彩色、对齐深度和 IMU 数据发布；
- `rgbd_sync` RGB-D 同步；
- `rgbd_odometry` 视觉里程计；
- RTAB-Map 增量建图和数据库保存；
- 地图数据库自动保存为 `maps/mapNNN.db`；
- `/rtabmap/map`、`/rtabmap/mapData`、`/rtabmap/odom` 等标准输出；
- RViz 地图显示和 RTAB-Map 数据库查看工具。

正式适配前应重新采集一张质量稳定、闭环正确的地图。短时间烟测生成的数据库只能
用于验证软件链路，不应直接作为导航生产地图。

## 3. 总体设计

几何地图、导航地图和语义信息分开保存，但通过同一个地图版本绑定：

```text
D435i RGB-D + IMU
        │
        ├──> rgbd_odometry ───────────────> odom -> base_link
        │
        └──> RTAB-Map localization ───────> map -> odom
                                                   │
Nav2 map_server <── mapNNN.yaml + mapNNN.pgm       │
                                                   ▼
semantic_map_server <── mapNNN_semantic.yaml ──> Nav2 NavigateToPose
              │
              ├──> RViz MarkerArray
              ├──> 语义名称/区域查询服务
              └──> Keepout/Speed Filter 掩码
```

核心原则如下：

- `mapNNN.db` 用于 RTAB-Map 视觉重定位；
- `mapNNN.yaml` 和 `mapNNN.pgm` 用于 Nav2 固定二维地图；
- `mapNNN_semantic.yaml` 保存人工标注；
- 三者必须使用同一个 `map` 坐标系和版本号；
- 导航阶段禁止 RTAB-Map继续增量建图，避免语义坐标失效；
- 同一时刻只能有一个权威静态地图发布源。

## 4. 地图版本和目录结构

建议将当前平铺的数据库升级为按版本组织的目录：

```text
/home/lunar/project/lunar_slam/maps/
├── map001/
│   ├── map001.db
│   ├── map001.pgm
│   ├── map001.yaml
│   ├── map001_semantic.yaml
│   ├── map001_keepout.pgm
│   ├── map001_keepout.yaml
│   └── preview.png
└── map002/
    └── ...
```

每个地图版本还应记录：

- 建图日期和场地名称；
- RTAB-Map 参数版本；
- 二维地图分辨率与原点；
- 相机、底盘和传感器外参版本；
- 语义标注文件版本；
- 是否通过定位和导航验收。

第一阶段可以继续兼容现有的 `maps/mapNNN.db`，待导出二维地图时再创建同名目录，
避免一次性破坏当前自动编号逻辑。

## 5. 固化 Nav2 二维地图

### 5.1 导出流程

完成建图并按 `Ctrl-C` 正常关闭 RTAB-Map 后，重新以只读方式加载目标数据库并发布
二维栅格地图，再调用 Nav2 map saver：

```bash
ros2 run nav2_map_server map_saver_cli \
  -f /home/lunar/project/lunar_slam/maps/map003/map003 \
  --ros-args -r map:=/rtabmap/map
```

输出的 `map003.pgm` 和 `map003.yaml` 是 Nav2 的权威静态地图。导出完成后检查墙体、
门口和走廊是否连通，清除明显的漂移重影和孤立噪声后才能进入标注阶段。

### 5.2 坐标约束

语义数据统一保存为 `map` 坐标系下的米制坐标，不直接以图片像素作为运行时接口。
图片标注工具需要根据地图 YAML 中的 `resolution`、`origin` 和图片高度完成像素到
地图坐标转换。常见的零旋转地图可使用：

```text
x_map = origin_x + u * resolution
y_map = origin_y + (image_height - 1 - v) * resolution
```

必须用 RViz 中的已知位置验证 Y 轴方向和一个像素的边界偏差。若地图原点带旋转，
则应使用完整二维刚体变换，不能继续使用上述简化公式。

## 6. 手动语义标注设计

### 6.1 第一阶段标注类型

- `landmark`：充电桩、门、电梯、设备、工作台等点目标；
- `region`：房间、走廊、仓库等多边形区域；
- `navigation_goal`：经过人工确认的安全导航位姿；
- `keepout`：楼梯、玻璃、危险区域等禁行多边形；
- `speed_zone`：需要降低速度的区域。

房间或物体的几何中心不一定可通行，因此每个可导航语义对象应单独设置
`navigation_goal`，包含位置和朝向。

### 6.2 建议数据格式

```yaml
schema_version: 1
map_id: map003
frame_id: map
map_yaml: map003.yaml

landmarks:
  - id: charging_station
    label: 充电桩
    type: charging_station
    pose: {x: 2.35, y: -1.20, yaw: 3.14}

regions:
  - id: room_101
    label: 房间101
    type: room
    polygon:
      - [1.0, 1.0]
      - [4.0, 1.0]
      - [4.0, 3.5]
      - [1.0, 3.5]
    navigation_goal: {x: 1.5, y: 2.0, yaw: 0.0}

navigation_zones:
  - id: glass_wall_guard
    type: keepout
    polygon:
      - [5.0, -2.0]
      - [6.0, -2.0]
      - [6.0, 0.0]
      - [5.0, 0.0]
```

加载时应校验 ID 唯一、坐标有限、多边形至少三个点、目标点处于空闲栅格，并校验
语义文件声明的 `map_id` 与当前加载地图一致。

### 6.3 标注工具

建议优先实现 RViz 标注工具，避免图片坐标转换错误：

1. 订阅固定二维地图；
2. 使用 RViz `Publish Point` 或交互式标记获取 `map` 坐标；
3. 选择标注类型并输入 ID、中文名称；
4. 单次点击生成点目标，多次点击闭合为区域；
5. 使用交互式箭头设置导航目标朝向；
6. 保存语义 YAML，并发布 MarkerArray 供人工复核。

最小可用版本可以先实现命令行节点，订阅 `/clicked_point` 并将标注写入 YAML；后续
再增加 RViz Panel。标注文件写入时应先写临时文件，再原子替换正式文件，防止异常
退出产生半个 YAML。

## 7. 项目包划分

建议增加以下项目自有包：

```text
project/
├── luxi_RTAB_Map/              # 保留现有建图能力
├── luxi_rtabmap_localization/  # RTAB-Map 纯定位启动与参数
├── luxi_semantic_map/          # 标注、加载、查询与 RViz 显示
├── luxi_nav2_bringup/          # Nav2 参数和系统总启动
└── luxi_navigation_interfaces/ # 必要时定义自有消息和服务
```

若第一阶段接口较少，可以先将自定义消息放入 `luxi_semantic_map`，但长期建议把消息
定义拆开，防止标注工具、任务层和导航层相互依赖具体实现。

## 8. 语义地图服务器接口

`semantic_map_server` 建议提供：

### 发布话题

```text
/semantic_map/markers    visualization_msgs/msg/MarkerArray
/semantic_map/keepout    nav_msgs/msg/OccupancyGrid
/semantic_map/speed      nav_msgs/msg/OccupancyGrid
```

### 服务或动作

```text
/semantic_map/list
/semantic_map/get_landmark
/semantic_map/get_region
/semantic_map/save
/semantic_navigation/navigate_to_name
```

`navigate_to_name` 的处理流程为：

1. 根据语义 ID 或名称查询目标；
2. 读取预先校验过的 `navigation_goal`；
3. 检查当前 Nav2 和定位状态；
4. 发送 `nav2_msgs/action/NavigateToPose`；
5. 返回导航反馈和最终状态。

名称允许重复时必须要求调用方提供类型或区域 ID；内部业务接口应优先使用稳定 ID，
中文名称只用于显示和自然语言交互。

## 9. RTAB-Map 纯定位适配

新增独立的 `rgbd_localization.launch.py`，不要通过建图启动文件的临时参数承担生产
定位。主要配置为：

```text
database_path=<固定的 mapNNN.db>
Mem/IncrementalMemory=false
Mem/InitWMWithAllNodes=true
```

定位节点继续使用 D435i RGB-D、IMU 和 `rgbd_odometry`，但不得使用
`--delete_db_on_start`。预期 TF 职责为：

```text
map --RTAB-Map--> odom --里程计或状态估计--> base_link
```

启动时应检查数据库存在且可读、当前地图版本匹配、相机话题就绪，并明确打印当前
定位地图路径。停止定位不应修改固化数据库；若 RTAB-Map 运行方式仍会更新数据库，
应在启动前复制只读工作副本或增加退出后哈希校验。

定位初始化要求相机位于历史地图中有足够视觉特征的位置。应发布定位健康状态，至少
包含：是否已定位、最近一次定位时间、匹配内点数和协方差。未成功定位时禁止向 Nav2
下发语义导航目标。

## 10. Nav2 适配

### 10.1 必需输入和 TF

Nav2 运行前必须具备：

```text
/map                 固定二维占据地图
/odom                连续局部里程计
/tf                  map -> odom -> base_link
/scan 或 PointCloud2 实时障碍物
/cmd_vel             底盘速度指令输出
```

如果当前平台尚无可执行 `/cmd_vel` 的移动底盘，只能验证定位、代价地图、目标转换和
全局路径，不能宣称完成自主导航闭环。

### 10.2 地图发布边界

推荐由 Nav2 `map_server` 加载固化的 `mapNNN.yaml` 并发布 `/map`。RTAB-Map 的
二维地图输出用于建图和一致性检查，在导航模式下不要同时重映射为同一个 `/map`，
否则会产生两个地图发布者。RTAB-Map 在定位阶段主要负责 `map -> odom`。

### 10.3 障碍物来源

D435i 深度可以通过以下任一方案接入局部代价地图：

- 深度图转换为 `sensor_msgs/msg/LaserScan`；
- 点云作为 Nav2 obstacle/voxel layer 的观测源。

需要限制最小、最大高度和距离，过滤机器人自身及地面噪声。全局静态地图不能替代
局部实时避障。

### 10.4 语义区域

- `keepout` 多边形栅格化为与静态地图相同尺寸、分辨率和原点的掩码，接入 Nav2
  Keepout Filter；
- `speed_zone` 生成速度掩码，接入 Speed Filter；
- 房间和目标点由任务层转换为 `NavigateToPose`，不直接修改全局占据地图；
- 掩码生成后必须叠加显示，检查是否发生翻转、平移或缩放。

## 11. 启动顺序

建议最终提供一个总启动入口，但开发阶段按以下顺序分开验证：

1. 启动 D435i 驱动；
2. 验证 RGB、对齐深度和 IMU 频率；
3. 启动 RTAB-Map 纯定位；
4. 在已知位置确认 `map -> odom` 和定位状态稳定；
5. 启动 Nav2 map server 和生命周期管理器；
6. 启动全局、局部代价地图和规划控制节点；
7. 启动语义地图服务器和 Marker 显示；
8. 先发送普通坐标目标，再发送语义名称目标；
9. 最后验证禁行区、限速区和动态避障。

总启动文件应接受一个统一参数，例如：

```text
map_id:=map003
```

由此解析对应的数据库、二维地图和语义文件，禁止分别指定三个不一致的版本。

## 12. 分阶段实施计划

### 阶段 A：地图固化和版本管理

- 完成二维地图导出脚本；
- 建立地图目录和元数据；
- 检查 RTAB-Map 数据库与二维地图坐标一致；
- 为地图文件生成校验和。

验收：重启系统后能够加载同一数据库和二维地图，原点及墙体位置不变。

### 阶段 B：手动语义标注 MVP

- 定义语义 YAML schema；
- 实现 `/clicked_point` 点标注；
- 支持导航目标和多边形；
- 发布 RViz MarkerArray；
- 加入格式和占据栅格校验。

验收：保存、重启、重新加载后，所有标记仍落在原位置，目标点位于可通行区域。

### 阶段 C：RTAB-Map 纯定位

- 新增定位参数和启动文件；
- 禁用增量建图；
- 增加数据库和话题启动检查；
- 输出定位健康状态；
- 验证遮挡、短时丢失和恢复行为。

验收：多次从已知位置启动均能重定位，数据库不被修改，TF 连续且没有多发布者。

### 阶段 D：Nav2 基础导航

- 配置 map server、planner、controller、行为树和生命周期；
- 标定机器人 footprint、速度和加速度限制；
- 接入深度障碍物；
- 验证坐标目标导航。

验收：目标路径位于空闲区，局部代价地图可检测障碍；有真实底盘时完成闭环行驶。

### 阶段 E：语义导航

- 实现语义查询和 `navigate_to_name`；
- 生成 keepout/speed mask；
- 增加任务反馈、取消和错误处理；
- 增加地图版本不匹配保护。

验收：按语义 ID 导航到正确安全位姿，禁行区不会被规划穿越，错误名称得到明确返回。

## 13. 测试矩阵

### 数据测试

- YAML 格式错误、重复 ID、非法多边形；
- 地图版本不匹配；
- 目标位于障碍物或地图外；
- 像素与地图坐标转换误差；
- 掩码尺寸、分辨率和原点不一致。

### 定位测试

- 已知位置冷启动；
- 不同朝向启动；
- 低纹理区域；
- 短时遮挡后恢复；
- 快速转动导致里程计丢失；
- 定位失败时 Nav2 是否停止接收目标。

### 导航测试

- 普通坐标目标；
- 语义点目标；
- 房间入口目标；
- 禁行区绕行；
- 动态障碍停车和恢复；
- 目标取消、超时和不可达；
- RTAB-Map 定位丢失时的安全停车。

## 14. 主要风险与对策

### 地图修改导致语义漂移

地图重新优化、裁剪或重新建图后，旧语义坐标不再可信。对策是绑定 `map_id` 和文件
校验和；地图变化后必须重新检查或迁移标注。

### 纯 D435i 里程计不够稳定

白墙、玻璃、遮挡和快速旋转会导致视觉里程计丢失。真实移动机器人应优先融合轮速
里程计与 IMU，再由 RTAB-Map提供全局校正。定位失效时必须阻止控制器继续盲走。

### 两个地图或 TF 发布者冲突

RTAB-Map 与 Nav2 map server 不应同时发布同名权威地图；`map -> odom` 也只能由一个
定位系统发布。启动检查应统计发布者数量并在冲突时失败退出。

### 语义目标不可通行

人工点击可能落在墙内或离障碍太近。保存时应结合膨胀后的代价地图校验，并允许为
语义对象设置独立的安全停靠位姿。

## 15. 建议的第一步实现范围

首个可运行版本只实现：

1. 从 `mapNNN.db` 导出固定 `pgm/yaml`；
2. 在 RViz 中点击并保存具名导航目标；
3. 单独启动 RTAB-Map 纯定位；
4. 用 Nav2 加载固定地图并接受普通 `NavigateToPose`；
5. 将一个语义名称转换为一个 `NavigateToPose` 目标。

完成这一最小闭环后，再增加多边形房间、禁行区、限速区和更复杂的自然语言任务，
可以显著降低同时调试地图、定位、语义和控制四个系统的风险。
