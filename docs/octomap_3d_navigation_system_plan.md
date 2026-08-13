# Lunar SLAM 三维 OctoMap 导航系统方案

## 1. 文档目的

本文给出当前 `lunar_slam` 工程的三维地形导航实施方案。方案以仓库中的三个参考工程为依据：

- `demo/jie_3d_nav`：静态 OctoMap、可通行层、预禁行层、风险代价层和 26 邻域 A*；
- `demo/SCAN-Planner-Ros2`：在线概率体素、障碍膨胀、滑动局部地图、重规划和 B-spline 局部轨迹；
- `demo/lucid-atlas`：IMU/点云里程计、局部点云地图 ICP、导航 dry-run 和控制安全门禁。

目标不是直接复制任一 demo，而是把三个 demo 中已经验证过的职责组合到当前工程已有的
`luxi_adapter`、`luxi_location`、`luxi_voxel_navigation` 和 `luxi_3d_navigation` 中。
核心实现继续使用 C++17；ROS 2 launch 文件沿用现有 Python 写法。

## 2. 本期范围、假设和成功标准

### 2.1 本期范围

本期实现范围包括：

1. 从保存点云和 OctoMap 生成三维地形代价地图；
2. 区分可通行平地/坡道、悬崖或坑、墙壁、普通障碍、低净空和未知区域；
3. 在 OctoMap 的地面支撑体素上执行带代价的三维 A*；
4. 使用 IMU 重力方向和点云匹配提供连续位姿，现有 HLoc 只保留为可选的全局粗定位入口；
5. 用当前点云维护局部滑窗占据层，进行停车和局部重规划；
6. 先完成离线、回放和 dry-run，再开放低速底盘控制。

### 2.2 明确不纳入本期

- 不新增相机到地面、相机到车体或车体几何中心的标定流程；
- 不训练新的视觉或地形识别模型；
- 不把三个 demo 整体迁入正式工作空间；
- 不在第一阶段直接接管真实 `/cmd_vel`；
- 不把 SCAN-Planner 面向自由空间的 B-spline 直接当成轮式底盘的地面轨迹。

临时坐标约定是把现有 `base_link` 或传感器跟踪坐标当作规划参考点，使用当前 TF；没有有效
车体外参时允许完成地图、定位、规划和速度预览，但不能据此证明车体包络已经安全避障。

### 2.3 参数假设

当前代码只有以下临时值，并非实车能力结论：

| 参数 | 当前值 | 本方案处理 |
| --- | ---: | --- |
| `robot_radius` | 0.10 m | 按当前需求暂定；实车前必须按真实车体包络复核 |
| `robot_height` | 0.35 m | 暂时沿用并增加净空余量 |
| `max_step_height` | 0.15 m | 不直接沿用；先按地形测试重新确定 |
| `max_slope_degrees` | 50° | 仅能说明 demo 可搜索，不能作为轮式底盘安全阈值 |
| OctoMap 分辨率 | 0.05 m | 当前建图、离线转换和正式地形层统一使用；0.10 m 仅保留历史兼容地图 |

本方案提供一组保守的软件起步阈值，最终值必须通过标准坡板/台阶测试确定。机器人型号、轮径、
轴距和厂商允许坡度尚未固化到仓库，因此文档不会把起步阈值描述成硬件能力。

### 2.4 成功标准

- 合成地图上的平地、不同角度坡道、墙、悬崖、坑、台阶、低净空和噪点分类全部通过；
- 同一输入地图重复生成的地形层逐体素一致；
- 路径只经过有连续支撑、足够净空且低于硬阈值的体素；
- 高风险坡道和障碍边缘有渐增代价，A* 在存在更安全路线时主动绕开；
- 定位失效、局部障碍侵入、路径失效或急停时输出零速度；
- dry-run 阶段只发布 `/navigation/cmd_vel_preview`；
- 所有新增第三方源码、模型或预编译工具放入现有 `3parts/`，不创建重复的 `3part/`。

## 3. 参考 demo 与当前模块的对应关系

| 设计内容 | 参考实现 | 当前落点 |
| --- | --- | --- |
| PCD 体素化、稀疏点过滤 | `jie_octomap/src/pcd_to_octomap_node.cpp` | 扩展 `luxi_voxel_navigation` 的离线转换 |
| 占据/预禁行/可通行/风险四层 | `octo_planner/src/jie_path_node.cpp` | 新增 `terrain_costmap` C++ 库 |
| 地图层保存和回放 | `jie_octomap/.../map_package_manager.py` | 保存为同编号 C++ 二进制层和 YAML 元数据 |
| 26 邻域 A*、起终点吸附 | `octo_planner/src/jie_path_node.cpp` | 扩展现有 `astar_3d.cpp` 和 `terrain_model.cpp` |
| 概率更新和射线清空 | `plan_env/src/grid_map.cpp` | 新增局部滑窗 OctoMap/体素层 |
| 机器人膨胀模型 | SCAN `double_cylinder_radius`、`body_height` | 地形层中的足迹、净空和扫掠体积检查 |
| 路线引导局部重规划 | SCAN `navi_mode=3`、`initial_path` | 全局地形路径作为局部规划参考线 |
| 轨迹平滑/动态可行性 | SCAN B-spline optimizer | 第二阶段局部轨迹；必须增加地面支撑约束 |
| IMU+点云连续里程计 | lucid FAST-LIO 数据链 | 当前统一 IMU + 深度点云/以后雷达点云 |
| 局部子地图 ICP、多尺度门控 | lucid `luxi-localization` | 扩展当前 `luxi_location` |
| dry-run DWB 和速度隔离 | lucid `luxi-navigation` | `cmd_vel_preview`、显式 start/stop 和安全仲裁 |

不能照搬的部分：

- `jie_3d_nav` 的预禁行规则主要依赖局部占据拓扑，不能可靠估计真实坡度和粗糙度；
- SCAN-Planner 主要检查自由空间碰撞，默认允许机身沿三维 B-spline 运动，不保证轮子始终有地面支撑；
- lucid-atlas 的 FAST-LIO 输入是 MID360，当前工程主要输入是统一 RGB-D/IMU，需复用算法职责而非话题名；
- demo 中的机器人尺寸、速度和地图分辨率不能直接作为当前底盘参数。

## 4. 当前工程基线和缺口

### 4.1 已有能力

当前正式链路已经具有：

- `luxi_adapter` 的统一 RGB-D、IMU 和 TF；
- RTAB-Map 保存数据库、PLY 和 `.bt` 地图；
- `tools/map_cloud_filter` 的 C++ 统计滤波、半径滤波和小聚类去除；
- `luxi_voxel_navigation` 的 PLY→OctoMap、`.bt` 加载和基础导航节点；
- `luxi_3d_navigation` 的地面支撑、圆柱碰撞、语义坑区、26 邻域三维 A* 和路径跟随；
- HLoc 三帧一致性门控和 Open3D ICP；
- `/navigation/start`、`/navigation/stop` 与默认隔离的速度输出。

对当前两张 map042 地图的只读检查结果为：

```text
map042_octomap/map042.bt:
  resolution=0.1, supported_cells=665, path_cells=78
  start=[-1.85, 0.15, 0.05], goal=[1.15, 7.65, 0.95]

map042_filtered_pipeline/map042.bt:
  resolution=0.1, supported_cells=604, path_cells=86
  start=[-1.85, 0.15, 0.05], goal=[1.15, 7.55, 0.95]
```

这证明当前地图至少包含一条跨高度连通表面，不证明该路线的坡度、边缘距离和车体净空安全。

### 4.2 必须修正的缺口

1. `TerrainModel::plan()` 传给 A* 的 `cell_cost` 恒为 0，当前没有实际地形代价地图；
2. 坡度由相邻体素 `atan2(|dz|, hypot(dx,dy))` 得到，0.10 m 地图中只有高度量化，不能区分
   平滑坡面、台阶和点云噪声；
3. `ply_to_octomap` 只把 PLY 端点标为 occupied，没有传感器原点射线，free/unknown 不完整；
4. 未知空间在碰撞查询中等同未占据，虽然“地面支撑”会挡住一部分未知区域，但低净空和侧向未知
   仍可能被当作安全；
5. 悬崖只通过单个候选体素下方是否有支撑间接处理，没有检查整个足迹、前方落差和制动距离；
6. 斜向转移没有检查整段扫掠体积和防止从两个障碍角之间穿越；
7. 当前跟随器忽略路径 z，只执行 x/y/yaw，也不在执行期间重新验证局部点云；
8. 当前 `luxi_location` 的 ICP 被约束为平面 x/y/yaw，IMU 重力方向尚未成为三维地形定位主约束；
9. 50° 坡度和 0.15 m 台阶是未验证默认值，不能作为轮式底盘实车门限。

## 5. 推荐总体架构

```text
离线地图链
RTAB DB / PLY
  -> C++ 点云清理
  -> 带 free/occupied/unknown 的静态 OctoMap
  -> 地面候选与局部平面拟合
  -> 静态三维地形代价层
  -> 保存 mapNNN.bt + terrain_costmap.bin + metadata.yaml

在线定位链
IMU -----------------------> 重力方向、姿态预测、异常倾斜检测
当前深度点云/雷达点云 ---> 降采样、去地面异常、scan-to-local-map ICP/GICP
可选 HLoc ---------------> 冷启动/丢失后的全局粗位姿
                              |
                              v
                     map -> odom -> base_link

在线导航链
静态地形代价层 -----------------------------+
当前点云 -> SCAN 风格局部概率滑窗/膨胀 ----+--> 路径有效性检查
语义 pit/禁行区 ----------------------------+       |
定位与目标 ---------------------------------+       v
                                            全局地形 A*
                                                   |
                                                   v
                                      局部轨迹/速度预览/安全仲裁
```

全局规划不是让机器人在任意 free 体素中“飞行”，而是在 OctoMap 中提取出的多层地面支撑图上做
三维搜索。一个 x/y 位置可以保留多个 z 表面，因而仍可表示坡道、桥下/桥上和多楼层结构；每个状态
是 `(x,y,z)`，但只允许沿连续地面转移。

## 6. 地图表示和地形分类

### 6.1 基础占据层

每个体素必须保留三态：

- `OCCUPIED`：占据概率大于阈值；
- `FREE`：射线观测明确清空；
- `UNKNOWN`：没有足够观测。

静态地图优先从 RTAB 数据库的关键帧深度和位姿逐帧射线插入；只有 PLY 时才使用当前“端点占据”
回退模式，并在元数据记录 `free_space_observed=false`。回退地图对 unknown 一律按高风险或不可通行
处理，不能假装已经观测为空闲。

分辨率采用两档验证：

- 0.05 m：当前正式默认，用于坡面、小台阶和边缘地形层；
- 0.10 m：仅用于历史 map042 兼容和性能对照。

如果原点云密度不足以支撑 0.05 m，不通过插值伪造地面；应回到建图质量或保守使用 0.10 m。

### 6.2 地面候选提取

对每个占据表面体素，在 XY 半径 `plane_fit_radius` 内收集邻居，使用带法向约束的 RANSAC 或
加权最小二乘拟合：

```text
n.x + d = 0
slope = acos(|n dot gravity_unit|)
roughness = sqrt(mean(point_to_plane_distance^2))
height_span = z_max - z_min
support_ratio = footprint 内具有连续地面回波的采样数 / 总采样数
```

IMU 给出的重力方向用于定义“水平”，而不是固定假设地图 Z 完全竖直。离线地图没有同步 IMU 时，
使用建图优化后的重力对齐地图坐标；在线局部层再用当前 IMU 做一致性检查。

候选机器人参考点位于支撑面上方，不再简单固定为“占据体素上面一个 cell”。应保存
`surface_z` 和 `body_reference_z` 两个值，避免路径 z 与 `base_link` 高度含义混淆。

### 6.3 初始分类阈值

以下是用于开始软件测试的保守值，不是实车认证值：

| 指标 | 低代价可通行 | 高代价可通行 | 硬禁行 |
| --- | ---: | ---: | ---: |
| 坡度 | `<= 10°` | `10°～18°` | `> 18°` |
| 相邻地面高度突变 | `<= 0.05 m` | `0.05～0.10 m` | `> 0.10 m` |
| 向下落差 | `<= 0.05 m` | `0.05～0.15 m` | `> 0.15 m` 或未知 |
| 平面 RMS 粗糙度 | `<= 0.015 m` | `0.015～0.030 m` | `> 0.030 m` |
| 足迹支撑率 | `>= 90%` | `70%～90%` | `< 70%` |
| 机身净空 | `>= robot_height+0.08 m` | `+0.03～0.08 m` | `< robot_height+0.03 m` |

0.10 m OctoMap 无法可靠分辨 0.05 m 台阶，因此在兼容测试中按体素分辨率向上量化阈值；正式
判断小台阶时必须使用 0.05 m 地形层或直接从原始 PLY 局部平面计算。

### 6.4 各类地形的明确定义

#### 可通行平地

同时满足低坡度、低粗糙度、足迹支撑、机身净空、已观测 free 和不与膨胀障碍重叠。平地不是
“没有占据点”，而是“下面有连续支撑、上面有已确认净空”。

#### 可通行坡道

坡道必须同时满足：

1. 局部平面坡度不超过硬阈值；
2. 沿行进方向的多个窗口法向连续，不能由离散台阶误拟合成一个大斜面；
3. 高度变化与水平距离一致，相邻高度跳变不超过台阶阈值；
4. 整个机器人足迹支撑率达到要求；
5. 坡顶、坡底不存在突然失去支撑；
6. 机身扫掠体积无碰撞，且坡上速度按代价降低。

因此，`<=10°` 初始视为正常通行，`10°～18°` 允许但增加代价和限速，`>18°` 初始禁行。
实车坡板测试通过后再放宽；不得直接恢复当前 50°。

#### 悬崖、坑和断边

对候选体素的整个圆形足迹以及沿速度方向的前视区域向下投射支撑射线：

- 在 `cliff_probe_depth` 内找不到地面，标为 `CLIFF_UNKNOWN`，硬禁行；
- 找到地面但落差大于 `max_drop_height`，标为 `CLIFF`，硬禁行；
- 支撑率低于阈值，标为 `EDGE`，硬禁行；
- 在悬崖边外扩 `robot_radius + localization_margin + braking_margin`；
- 语义 `pit` 多边形继续作为硬禁行覆盖层，并按机器人半径膨胀。

这比当前只检查中心体素下方更安全，也直接参考 `jie_3d_nav` 的 preblocked 层和风险膨胀思想。

#### 墙壁

局部表面法向接近水平、垂直连续高度覆盖机身，或任何占据体素进入机身扫掠体积时，都归为墙/硬
障碍。分类标签主要用于调试；碰撞结果不依赖“墙”是否识别成功，只要占据体进入包络就必须禁行。

#### 普通障碍物

支撑面上方 `obstacle_min_height` 到 `robot_height + margin` 范围内的占据点，包括石块、箱体、桌腿
和突起。先按机器人真实足迹膨胀，再叠加定位误差和制动余量。孤立点必须经过时间一致性或邻域
点数门限，避免单点噪声永久封路；局部在线层可随 miss 射线清除。

#### 低净空/顶部障碍

从地面到 `robot_height + clearance_margin` 的柱体内发现占据或 unknown，即判为净空不足。桥下、桌下
和斜顶必须沿转移边做扫掠检测，不能只检查路径节点。

#### 未知区域

全局静态规划默认 `unknown_is_lethal=true`。只有传感器射线明确标记为 FREE，且支撑和净空都通过，
才可通行。探索模式可以将 unknown 改为高代价，但不属于本期默认导航模式。

## 7. 三维地形代价地图

### 7.1 图层

每个可查询体素至少保存：

```text
occupancy_state   OCCUPIED / FREE / UNKNOWN
terrain_class     FLAT / RAMP / ROUGH / STEP / EDGE / CLIFF / WALL / OBSTACLE / LOW_CLEARANCE
surface_z         支撑面高度
slope_deg         局部坡度
roughness_m       平面残差
step_height_m     与候选邻居的高度突变
drop_height_m     向下探测落差；未知使用特殊值
support_ratio     足迹支撑率
clearance_m       到最近占据体的距离/顶部净空
semantic_mask     pit、人工禁行等
cost              0～255
```

参考 `jie_3d_nav`，保留四类可视化输出：占据层、硬禁行层、可通行层和渐变风险层；新增坡度、
粗糙度和支撑率字段，解决 demo 只有拓扑风险而没有真实地形指标的问题。

### 7.2 代价值约定

```text
0           已确认的最低风险地形
1..127      正常渐变代价
128..252    可通行但高风险；规划器应优先绕开
253         机器人足迹膨胀区
254         lethal：占据、墙、悬崖、台阶超限、净空不足、语义 pit
255         unknown
```

硬规则先执行。只有未触发硬禁行的体素才计算软代价：

```text
C = clamp(1, 252,
    w_slope   * normalized_slope
  + w_rough   * normalized_roughness
  + w_step    * normalized_step
  + w_edge    * edge_proximity
  + w_clear   * inverse_clearance
  + w_support * (1 - support_ratio)
  + w_unknown * observation_uncertainty)
```

初始权重优先级为：悬崖距离 > 净空/障碍距离 > 支撑率 > 坡度 > 台阶 > 粗糙度。权重先通过合成
地图单变量测试确定，不能同时手调全部参数。

障碍距离建议直接使用 `3parts/octomap/dynamicEDT3D`；它已随项目 OctoMap 源码存在，不需要新增
外部库。`jie_3d_nav` 的线性半径衰减可作为第一版回退实现。

### 7.3 可视化和持久化

ROS 输出建议为：

| 话题 | 类型 | 内容 |
| --- | --- | --- |
| `/navigation/terrain/occupied` | `MarkerArray` | 灰/黑占据体素 |
| `/navigation/terrain/lethal` | `MarkerArray` | 红色墙、崖、障碍、低净空 |
| `/navigation/terrain/traversable` | `PointCloud2` | 绿色到黄色的可通行表面 |
| `/navigation/terrain/risk` | `PointCloud2` | 字段含 cost/class/slope/roughness/support |
| `/navigation/terrain/debug_normals` | `MarkerArray` | 抽样地面法向 |
| `/navigation/local_octomap` | `octomap_msgs/Octomap` | 在线局部概率地图 |

同编号地图保存：

```text
maps/octo_maps/mapNNN_octomap/
  mapNNN.bt
  *_cloud.ply
  terrain_costmap.bin
  terrain_metadata.yaml
  terrain_build_report.json
```

元数据至少记录源 PLY/DB 摘要、分辨率、机器人包络、所有阈值、重力方向、地图边界、图层计数、
构建版本和 `free_space_observed`。参数改变后必须重建，不允许加载旧代价层配新机器人尺寸。

## 8. 路径规划方法

### 8.1 第一阶段：带代价的地面支撑三维 A*

沿用当前 `GridCell3D` 和 26 邻域，加入以下约束：

1. 禁止纯垂直移动；
2. 起点、终点必须吸附到同一可达地面连通分量；
3. 每个目标节点必须非 lethal 且有完整足迹支撑；
4. 转移边的高度差、坡度、法向变化和落差必须通过；
5. 斜向移动要检查两侧正交邻居，禁止穿角；
6. 沿边按不大于半个体素采样机身扫掠体积和地面支撑；
7. 向上和向下分别设置代价，防止在高风险下坡上选择最短路线；
8. unknown 默认不可扩展。

A* 边代价：

```text
g(next) = g(current)
        + metric_distance
        * (1 + terrain_weight * cost(next) / 252)
        + ascent_weight * max(0, delta_z)
        + descent_weight * max(0, -delta_z)
        + normal_change_weight * delta_normal
```

启发函数使用三维欧氏距离；只要额外代价非负，仍保持可采纳。第一版使用普通 A*，先保证路径正确；
只有地图规模证明性能不足时再引入 Weighted A*，并记录次优界。

### 8.2 起终点吸附

当前按 `dx²+dy²+0.25dz²` 选择最近可通行体素，容易吸附到错误楼层。改为：

- 先限制 XY 搜索半径和最大允许高度差；
- 优先与当前支撑面法向、高度和连通分量一致；
- 多楼层同 XY 时优先当前定位 z 附近层；
- 网页目标若没有 z，返回所有候选表面并选择与当前层可达且总代价最低者；
- 没有明确候选时拒绝目标，不静默吸附到远处或另一层。

### 8.3 路径后处理

不能简单对三维点做直线稀疏或无约束 B-spline，因为它可能切过悬崖边、墙角，或离开支撑面。
第一阶段只做：

1. 在地形表面上进行可见性简化；
2. 以 `<= resolution` 重新采样；
3. 每个采样点重新投影到同一连续支撑面；
4. 对整段重新执行扫掠碰撞、坡度和落差检查；
5. 给路径点附加 `z`、坡度、推荐速度和风险代价。

第二阶段可以参考 SCAN-Planner 的 B-spline smoothness/collision/feasibility 目标，但必须新增 surface
fitness 和 support 两项，使控制点不能离开可通行表面。

### 8.4 全局与局部规划分工

- 静态三维地形 A*：解决坡道、多高度层、静态墙/崖的全局可达性；
- 局部滑窗：融合当前点云，处理新出现或移动的障碍；
- 路径有效性监视器：局部 lethal 侵入路径或定位跳变时立即停车；
- 局部重规划：以全局路径前视点为目标，在 3～5 m 范围内重新搜索；
- 全局重规划：局部连续失败、目标变化或静态层变化时触发。

## 9. 在线局部地图

参考 SCAN-Planner `GridMap` 的概率更新，建立独立于静态地图的局部层：

```text
resolution:             0.05 m 起步
window:                 6 m x 6 m x 3 m 起步
p_hit / p_miss:         0.85 / 0.30
p_min / p_max / p_occ:  0.12 / 0.98 / 0.80
max_ray_length:         与有效深度范围一致，当前最多 4～5 m
```

处理流程：

1. 当前点云按时间戳转换到局部 odom/map；
2. 根据 IMU 方向和机器人附近排除区过滤自身/地面异常；
3. 对端点执行 hit，对传感器到端点射线执行 miss；
4. occupied 体素按机器人 XY 半径、车体高度和安全余量膨胀；
5. 静态 lethal 与局部 lethal 取并集；
6. 局部层只影响附近路径，不重写保存的静态地图；
7. 动态点消失后需连续 miss 才清除，避免单帧闪烁；
8. 每次规划使用同一时间快照，避免图层更新中途改变搜索结果。

局部层不负责把没看见的悬崖变成 free。深度相机对玻璃、强光和视野外区域可能无回波，失去地面
回波必须按 unknown/悬崖处理，而不是按自由空间处理。

## 10. IMU 和点云匹配定位

### 10.1 推荐职责

连续定位以 IMU 和几何点云为主：

```text
IMU 高频传播
  + 当前点云 scan-to-scan / scan-to-local-map 匹配
  + 保存地图局部子图 ICP/GICP 校正
  -> odom -> base_link 连续运动
  -> map -> odom 低频全局修正
```

参考 lucid-atlas：局部地图裁剪、多尺度 ICP、fitness/RMSE/最大修正门控、连续多帧接受和丢失重启。
当前 HLoc 可以保留为冷启动或彻底丢失后的全局粗定位，不作为连续导航唯一来源。

### 10.2 本期不加标定时的处理

- 复用 `luxi_adapter` 已有同步后的 `/sensors/imu/data` 和统一深度点云；
- 不新增相机-地面或相机-车体标定步骤；
- 临时令规划参考坐标与当前传感器跟踪坐标一致，或使用仓库已有 TF；
- IMU 只用重力方向、角速度传播和异常倾斜检测，不估计未提供的车体几何偏移；
- dry-run 和地图测试不受影响；真实车体碰撞安全仍受未确认外参限制，必须保守放大 footprint。

### 10.3 准入和失效条件

至少发布：位姿、fitness、RMSE、匹配点数、位姿协方差、最后接受时间和状态原因。以下任一发生时
导航停车：

- 位姿时间超过 `localization_timeout`；
- 连续 ICP 拒绝达到门限；
- 单帧修正量超过平移/旋转门限；
- IMU roll/pitch 超过硬阈值；
- map/odom TF 跳变使当前路径不再连续；
- 点云数量或局部地图点数不足。

不要在 ICP 失败时继续发布旧位姿并保持运动。参考当前 `LocalizationSupervisor`，进入 searching 状态后
清除运动许可，重新获取全局初值。

## 11. 控制与安全状态机

第一阶段沿用当前跟随器的显式 start/stop，但输出改为：

```text
/navigation/cmd_vel_preview
```

状态机：

```text
WAIT_MAP
  -> WAIT_LOCALIZATION
  -> READY
  -> PLAN_READY
  -> ACTIVE
  -> GOAL_REACHED

任意状态 --地图/定位/障碍/急停异常--> STOPPED
STOPPED --重新定位并重新规划--> READY
```

ACTIVE 的持续条件：

- 定位新鲜且可信；
- 路径版本与地图版本一致；
- 当前位姿未偏离路径过大；
- 前视路径扫掠体积仍可通行；
- 局部滑窗数据新鲜；
- 收到显式 `/navigation/start=true` 且没有 stop/estop；
- 速度输出看门狗工作。

局部控制可先沿用当前 lookahead 跟随，再参考 lucid 的 DWB 轨迹采样加入动态碰撞检查。坡度越高、
边缘越近、定位协方差越大，最大线速度越低。未确认底盘运动学前，不增加倒车、侧移或原地大角速度
等能力。

当前已落地的低速静态地图闭环采用显式网页“出发/停止行驶”：路径跟随器发布
`/navigation/active` 和状态，C++ 速度仲裁器只在 active 且导航速度新鲜时把
`/navigation/cmd_vel` 接到 LeKiwi `/cmd_vel`。非零手动输入、软件急停、定位 TF 超时、偏离路径、
到达目标或 0.3 秒速度看门狗超时都会停车。该闭环不等同于阶段 4：实时局部障碍物融合和动态绕行
仍未实现，因此首次实车只能在封闭、清场和具备实体急停/防跌落保护的环境中低速验证。

## 12. 建议代码组织

保持现有包边界，只增加必要文件：

```text
project/luxi_3d_navigation/
  include/luxi_3d_navigation/
    terrain_costmap.hpp
    terrain_classifier.hpp
    terrain_planner.hpp
    local_map_fusion.hpp          # 局部阶段再加入
  src/
    terrain_costmap.cpp
    terrain_classifier.cpp
    terrain_costmap_node.cpp
    terrain_planner.cpp
    terrain_path_monitor_node.cpp
  test/
    test_terrain_classifier.cpp
    test_terrain_costmap.cpp
    test_terrain_planner.cpp
    data/                         # 小型程序生成，不提交大地图

project/luxi_voxel_navigation/
  src/
    rtab_depth_to_octomap.cpp     # 从关键帧位姿射线构建三态静态图

project/luxi_location/
  src/
    cloud_imu_localizer.cpp       # 若现有 ICP 扩展后仍职责清晰，也可直接扩展现有节点
```

`astar_3d.cpp` 保留通用 A*；地形分类和代价查询放入独立库，避免继续把全部逻辑堆进 ROS node。
第一版不建立插件框架，不引入 Nav2 costmap 插件，也不复制 demo GUI。

## 13. 分阶段实施与验证

### 阶段 0：冻结基线和地图审计

工作：

1. 固定 map042 原始 PLY、滤波 PLY 和两个 `.bt` 的摘要；
2. 输出占据体素数、边界、分辨率、z 分布和 connected components；
3. 保存当前 `terrain_plan_check` 结果作为回归基线；
4. 明确地图是否有 raycast free-space，写入构建报告。

验收：相同输入重复报告完全一致；地图文件不可读、空图或坐标异常时明确失败。

### 阶段 1：先测试地形代价地图生成

这是实现优先级最高的阶段，暂不接定位和控制。

用 C++ 测试地图生成器程序化构造：

| 场景 | 预期 |
| --- | --- |
| 0° 平地 | 全部低代价可通行 |
| 5°/10° 坡 | 可通行，代价随角度非递减 |
| 12°/18° 坡 | 高代价区，仍连通 |
| 20°/25° 坡 | 按初始硬阈值禁行 |
| 平滑坡与同高度离散台阶 | 坡可行，台阶按突变阈值处理 |
| 0.05/0.10/0.15/0.20 m 台阶 | 依次验证低代价、高代价、禁行边界 |
| 断边和深坑 | 足迹接近前已经进入 edge/lethal |
| 细墙/箱体/孤立噪点 | 墙和箱体膨胀，孤立噪点按门限过滤 |
| 低顶 | 柱体净空不足即禁行 |
| 双层地面 | 同 XY 保留两个 z 层且不串层 |
| unknown 缝隙 | 默认不可通过 |

技术检查：

- 坡度应接近输入真值，误差目标 `<= 2°`；
- 代价对坡度、粗糙度和边缘距离分别单调；
- 膨胀半径误差不超过一个体素；
- 平地连续性不因 OctoMap 叶节点深度不同而断裂；
- 0.05/0.10 m 两种分辨率均无越界、NaN 和非确定输出；
- 性能报告包含总耗时、峰值内存和各图层体素数。

阶段 1 验收后，才在真实 map042 上生成四色图层并人工检查坡道、墙边和地图边界。

### 阶段 2：带代价全局规划

工作：

1. 把 `TerrainModel::plan()` 的零代价回调替换为真实地形代价；
2. 增加穿角、扫掠体积、法向变化、上/下坡代价和同层吸附；
3. 路径消息保留 z，并附带独立 debug cost cloud；
4. 失败时输出明确原因和被拒绝类别统计。

测试：

- 短而危险与长而安全两条路线并存时选择安全路线；
- 提高风险权重后路径总代价不增加危险体素数量；
- 没有可行路线时返回空路径且不保留旧路径；
- 起点/目标在墙内、悬崖上、另一楼层或 map 外时分别正确拒绝/吸附；
- 路径每条边重新检查均通过；
- map042 原始和滤波地图均完成指定起终点规划，并输出风险统计。

### 阶段 3：IMU+点云匹配定位

工作：

1. 录制统一 IMU、深度/点云、TF 和现有定位 rosbag；
2. 用 IMU 对齐重力并传播短时姿态；
3. 实现/扩展多尺度 scan-to-local-map ICP；
4. 保留 HLoc 粗位姿作为可选初始化；
5. 使用连续接受、最大修正、fitness/RMSE 和超时门控；
6. 输出唯一且连续的 `map -> odom`。

验收：

- 静止 60 s 位姿不持续漂移或跳变；
- 缓慢平移、转动、上下坡回放无 TF 时间回退；
- 短时点云不足时 IMU 只短时预测，超时后停止导航；
- 人工注入错误粗位姿不能通过 ICP 连续门控；
- 定位重启后能够恢复且旧运动许可被清除。

### 阶段 4：局部概率地图和路径监视

工作：

1. 按 SCAN 风格加入 hit/miss 和滑动窗口；
2. 融合静态地形层与当前障碍；
3. 当前路径前视段持续扫掠检查；
4. 障碍进入时发布 stop，并触发局部或全局重规划；
5. 障碍消失后需稳定清除再恢复 READY，不自动恢复 ACTIVE。

验收：静态箱体、突然放置障碍、移动人员、点云瞬时消失和玻璃无回波分别得到安全行为。

### 阶段 5：dry-run 完整导航

工作：网页下发目标，系统完成定位、吸附、全局路径、局部轨迹、状态和速度预览；速度只发
`/navigation/cmd_vel_preview`。

验收：

- RViz/Web 同时显示三维路径和风险层；
- 人工移动传感器沿路径时路径进度正确；
- 定位丢失、障碍侵入、stop/estop 时预览速度立即归零；
- 系统中没有真实底盘 `/cmd_vel` publisher；
- 连续运行 30 min 无地图反复重建、内存持续增长或 TF 冲突。

### 阶段 6：低速闭环

仅在前五阶段全部通过后，把预览速度接入安全仲裁，再由仲裁输出 `/cmd_vel`。初始速度不超过
0.10 m/s，使用实体急停和底盘自身看门狗。坡道按角度限速，首次测试使用标准坡板和防跌落保护。

由于本期排除了相机到车体标定，阶段 6 只能在现有 TF 已足够准确或 footprint 已保守覆盖安装偏移的
前提下进行；否则停留在 dry-run。这不是新增标定要求，而是对未确认几何关系的风险边界说明。

## 14. 参数确定方法

不要直接凭 demo 参数上线。按以下顺序一次只确定一组参数：

1. 地图分辨率：比较 0.05/0.10 m 的连通性、坡度误差、内存和构建时间；
2. 点云滤波：以保留墙边和坡面为前提，调统计/半径/小聚类门限；
3. 平面窗口：在坡面法向稳定与墙边不过度跨面之间折中；
4. 机器人包络：先用当前 0.20/0.35 m，再只向保守方向加 margin；
5. 坡/台阶/落差硬阈值：使用实体坡板和台阶逐级测试；
6. 软代价权重：用双路线合成地图验证选择顺序；
7. 局部地图 hit/miss：用静态障碍和移除障碍实验决定响应与清除时间；
8. 定位门控：用 rosbag 统计正常和失败分布后确定，不按单帧最佳结果设置；
9. 控制速度：最后确定，且随坡度、曲率、边缘距离和定位置信度限速。

每次参数变更保存 YAML、地图摘要、测试报告和路径代价，避免“调通后无法复现”。

## 15. 依赖和 `3parts/` 规则

第一阶段不需要新增第三方库：

- OctoMap 和 `dynamicEDT3D` 已在 `3parts/octomap`；
- Open3D 0.18 已在 `3parts/open3d`；
- PCL、Eigen、yaml-cpp、nlohmann-json 已被当前 C++ 工程使用；
- 三个参考项目保留在 `demo/`，不加入正式 colcon 构建。

若后续确实需要引入新的地图/优化库：

1. 源码作为固定提交的 Git submodule 放到 `3parts/<name>`；
2. 安装前缀使用 `3parts/<name>/install`；
3. 模型和缓存放 `3parts/<name>_models` 或明确的缓存目录；
4. 更新 `scripts/setup_3parts.sh` 和 `3parts/README.md`；
5. 不使用 `sudo make install` 写系统目录；
6. 不把构建产物提交到 Git。

当前仓库实际目录名是 `3parts/`，本文将需求中的“3part 文件夹”统一解释为该现有目录。

## 16. 推荐的首个实现批次

首个代码批次只完成“离线地形代价地图”，不同时修改定位和控制：

1. 新建 `terrain_costmap.hpp/.cpp` 和 `terrain_classifier.hpp/.cpp`；
2. 增加合成地形 GTest；
3. 从现有 `.bt` 构建占据、支撑、净空、坡度、粗糙度、边缘和风险层；
4. 发布 debug PointCloud2/MarkerArray；
5. 在 map042 原始/滤波地图生成报告并人工复核；
6. 代价地图验收后，再把它接入 A* 的 `cell_cost`。

这样每个阶段只有一个主要变量：先证明地图分类正确，再证明规划选择正确，最后加入定位、局部感知和
控制。它也符合当前工程已有模块边界，能以最小改动逐步替换现有的零代价基线。

## 17. 最终交付物

- C++ 地形代价地图库、ROS 2 节点和 GTest；
- C++ 带地形代价的三维地面支撑 A*；
- map042 的地形构建报告、图层统计和示例路径报告；
- IMU+点云定位回放报告；
- 局部地图与动态障碍测试报告；
- dry-run 启动文件、RViz 配置和话题检查脚本；
- 所有参数 YAML、地图元数据和失败原因定义；
- 低速闭环前检查清单。

上述交付物完成前，不把“找到一条三维体素路径”等同于“具备安全 3D 导航能力”。
