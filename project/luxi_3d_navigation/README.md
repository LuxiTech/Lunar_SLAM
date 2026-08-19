# luxi_3d_navigation

该包在当前 HLoc 首次粗定位、连续三帧一致后交给 ICP 的定位链之后，使用地面支撑型三维 A* 对保存的 OctoMap 规划路径。候选扩展来自 3×3×3 邻域；规划会拒绝纯垂直运动、无地面支撑、机身净空不足、超过台阶/坡度阈值以及落入语义 `pit` 多边形的体素。地形模型同时分割障碍物和可通行表面，并按到可通行边界的距离生成渐变代价；A* 把该代价计入实际路径开销，因此在有空间时优先走中间。

动态避障的实现架构、`demo/SCAN-Planner-Ros2` 方法分析、备选方案对比、首轮参数和从离线回放到 10～20 m 真机路线的分级测试门槛，见 [D1 动态避障设计与分阶段测试方案](DYNAMIC_OBSTACLE_AVOIDANCE.md)。当前实现保留保存地图全局 A*，用 D435i 建立带时间衰减的滚动 3D 障碍层，局部重规划后重新接回全局路径，并由独立速度安全门处理障碍、无路、命令超时和传感器故障。

针对 map022～map025 的地图质量、避障后失定位和安全停车问题，现行改造项、参数门槛及从本机零运动测试到 1 m 实物绕行的验收步骤，见 [D1 建图、定位、导航与动态避障系统性修改及测试文档](NAVIGATION_ROBUSTNESS_MODIFICATION_AND_TEST_PLAN.md)。

`saved_map_navigation.launch.py` 已完整接入动态避障。全局路径使用 `/navigation/global_path`，局部规划输出为 `/navigation/planned_path`；跟随器先发布 `/navigation/cmd_vel_raw`，安全门检查后才发布 `/navigation/cmd_vel`。网页 HTTP API 保持不变，但网页和其他客户端都只向独立的 `navigation_task_manager` 发布 `/navigation/task/*` 高层命令；目标替换、取消、清除旧路径、保存返航点和返航自动出发不再由某个网页私自实现。默认 `dynamic_monitor_only:=false`，深度、CameraInfo、TF 或局部规划状态不健康时导航速度为零。

定位恢复完全封装在本包的路径跟随器、恢复控制器和独立速度安全门中。ICP 异常时，只有丢失前最后一次前方障碍状态为 `clear`、当前深度仍为 `clear`、局部路径仍有效三个条件同时成立，才保留视觉惯性里程计最多 0.8 秒；安全门把命令限制为不超过 0.10 m/s 的纯前向运动，因此最远续行 8 cm。任一条件不成立都会立即停止平移。随后锁存丢失定位前最后一个路径航向的左右方向，以 0.10 rad/s 原地旋转并通过 `/luxi_location/relocalization_request` 重启 HLoc。恢复旋转没有时间上限，直到定位连续确认成功、用户取消任务或旋转包络不安全。连续确认定位后先清空旧地图对齐下积累的滚动障碍历史，并等待两个最新深度帧重新建立安全障碍层，再从修正位置规划到原目标。

2026-08-15 曾记录一次 map023 + 前方垃圾桶运行遥测：D435i 障碍层约 10 Hz，日志显示局部路径热更新、净移动约 1.417 m，最终速度为零。但现场用户明确反馈尚未完成可见、可重复的实物绕障验收，因此该记录只算“链路遥测”，**不能标记为避障测试通过**。D1 低于 0.10 m/s 基本不产生有效步态，跟随器和安全门暂保留 0.10 m/s 的硬件最小有效平移速度；下一次必须按设计文档第 8、11 节重新完成有人监护的分级实测。

地形分割直接读取与 `.bt` 配套的 PLY：先按当前 0.05 m OctoMap 分辨率降采样，再以 `ground_normal_radius` 邻域估计局部法向。近水平点按相邻高度连续性拆成平面分量，只保留面积最大的连续主地面，每个 XY 位置只保留一个表面高度。局部法向转成墙面或相邻高度突然变化超过一个当前体素时，地面传播会停止，因此水平障碍顶部也不会单独成为可通行地面。只有高于附近主地面至少 `obstacle_min_height` 的实际点云栅格进入红色障碍层；法向不足且没有地面支撑的稀疏区域保持 unknown，不会仅因 `.bt` 的粗体素展开而变成障碍。`.bt` 继续提供地图范围和 OctoMap 发布，不能代替 PLY 地形拟合。

地图收到后会发布三个 transient-local 可视化 Marker：

- `/navigation/terrain/obstacles`：红色障碍物体素；
- `/navigation/terrain/traversable`：绿色可通行表面；
- `/navigation/terrain/costmap`：黄到红的边缘代价带。

网页按下可通行地面确定目标位置，拖动箭头确定最终 yaw；规划器会在 `snap_search_radius_cells` 范围内吸附到可行走表面高度。输出的 `/navigation/planned_path` 保留每个节点的 z；轮式底盘跟随器只执行 x/y/偏航。客户端应发布 `/navigation/task/start=true`，由任务管理器检查并转发低层 `/navigation/start`，不要直接绕过高层状态机。

跟随器以 15 Hz 发布 `/navigation/cmd_vel_raw`，安全门以 20 Hz 审核后发布
`/navigation/cmd_vel`，并通过 `/navigation/active` 和
`/navigation/follower_state` 报告 `plan_ready`、`active`、`goal_reached`、`localization_lost`
等状态。定位 TF 超过 1 秒未更新或机器人距离路径超过 0.50 m 时立即停车。LeKiwi 专用网页
启动中的 C++ 速度仲裁器负责手动/导航独占、0.3 秒导航看门狗、急停以及底盘角速度方向适配，
最后才发布真实 `/cmd_vel`。

```bash
cd /home/lunar/project/lunar_slam
source /opt/ros/humble/setup.bash
colcon build --packages-select luxi_3d_navigation --symlink-install
source install/setup.bash
ros2 launch luxi_3d_navigation saved_map_navigation.launch.py \
  octomap_path:=/absolute/map.bt \
  cloud_path:=/absolute/map.ply \
  hloc_map_directory:=/absolute/hloc \
  semantic_path:=/absolute/annotations.json
```

离线验证实际 OctoMap 是否至少包含一段可连通的地面支撑路径：

```bash
ros2 run luxi_3d_navigation terrain_plan_check /absolute/map.bt /absolute/map.ply
```

在真实地图路径中部加入半径 0.20 m 的合成临时障碍，验证局部绕行或安全无路：

```bash
ros2 run luxi_3d_navigation terrain_plan_check \
  /absolute/map.bt /absolute/map.ply --dynamic-test
```

`config/navigation.yaml` 中按实测中心到轮子距离设置 `robot_radius=0.25m`，后缘为 0.15 m，最前端相机为 0.18 m；`costmap_margin=0.35m`、`costmap_weight=8.0`。代码先用 0.25 m 半径生成硬碰撞边界，再从该边界向外铺 0.35 m 渐变代价带，因此障碍物到最外层代价边界约为 0.60 m（受 0.05 m 体素离散影响）。局部法向半径为 0.30 m、地面法向最大倾角为 35°、障碍最小离地高度为 0.15 m。这里的 35° 是允许地图整体倾斜和法向噪声的分割阈值，不是底盘最终可爬坡角。没有确认路径、目标方向和定位状态前不要发布 `/navigation/start=true`。

路径跟随的常速仍为 0.10 m/s。只有局部障碍状态为 `clear`、定位为 `tracking`、
里程计新鲜，而且直行命令连续 2.5 s 只产生不足 0.04 m 位移时，才会进入
`traction_boost`，短时把速度提高到 0.15 m/s。检测到有效位移会立刻恢复常速；增力
持续 2.0 s 仍无位移则停止并报告 `stuck_no_progress`，不会在障碍物前反复加速。

定位变为 `searching` 或 `verifying` 后，先执行 0.8 s 的短时保持/推算，再锁定丢失前
最后一次路径转向，持续单向原地旋转并请求重新定位。重新进入 `tracking` 后仍需稳定
确认 1.0 s。恢复转速为 0.10 rad/s（约 5.7°/s），为图像检索和 ICP 保留更多重叠、
低运动模糊的视角；旋转会持续到定位成功或用户取消，不再因固定时长退出。确认后不会直接沿可能已经错位的旧路径
继续，而是保留原目标，从修正后的当前位置强制重新执行全局规划；新路径到达后才恢复
运动。安全路线暂时不可用时保持原目标、停车并每 5 s 重试，最长等待 120 s。机器人偏离路径超过 0.50 m 时也按定位跳变处理，先
重新定位再重规划。恢复旋转仍受近场障碍层和独立速度安全门约束：前向走廊即使为
`blocked`，也只有周身最近障碍大于 0.30 m 旋转包络时才允许纯旋转；深度数据中断或
旋转净空不足时保持停车，数据恢复安全后继续旋转，不会盲转。

网页中的路径只在新路径规划完成或导航活动期间显示。任务到达、人工停止、安全重规划
超时、避障超时或其他中断都会立即删除活动路径和旧路径缓存，不再显示灰色旧路线。
两个网页的“选择目标”和“设置返航点”都使用与 RViz 相同的按下、拖动选方向、松开
确认方式。返航点保存在当前地图导航进程的 `navigation_task_manager` 中，并通过
transient-local `/navigation/task/home_state` 对所有客户端公开；“一键返回起点”由该
核心节点停止当前任务、规划返航路线，并在局部安全路径就绪后自动出发。网页只显示
`/navigation/task/status`，不会参与等待路径或自动出发的判定。

统一高层接口如下，网页 HTTP API 和其他控制端最终都落到这些话题：

- `/navigation/task/goal_pose` (`PoseStamped`)：设置带最终方向的目标；自动停止旧任务；
- `/navigation/task/start` (`Bool`)：执行已经生成的安全路径；
- `/navigation/task/cancel` (`Bool`)：取消任务并清除全局、局部和网页旧路径；
- `/navigation/task/home_pose` (`PoseStamped`)：保存当前位置或人工选定的返航姿态；
- `/navigation/task/return_home` (`Bool`)：取消当前任务并在安全路径就绪后自动返航；
- `/navigation/task/status` (`String`) 与 `/navigation/task/home_state`：供所有界面统一显示。

当前方走廊进入 `blocked` 时，跟随器会停止全部平移并请求沿最新路径转向原地旋转。
安全门只有在深度与机器人周身净空数据均新鲜、定位仍为 `tracking`、局部路径为
`ready`，且所有实测障碍都在 0.30 m 旋转包络之外时，才放行不超过 0.20 rad/s 的
纯角速度；这相当于 0.25 m 机器人半径外再留 0.05 m 余量，并非只检查相机正前方。
前方转为 `slow/clear` 后自动接回局部绕行路径；连续 10 s 仍未清除则报告
`obstacle_recovery_timeout` 并停车。
