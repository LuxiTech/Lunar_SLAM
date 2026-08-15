# luxi_3d_navigation

该包在当前 HLoc 首次粗定位、连续三帧一致后交给 ICP 的定位链之后，使用地面支撑型三维 A* 对保存的 OctoMap 规划路径。候选扩展来自 3×3×3 邻域；规划会拒绝纯垂直运动、无地面支撑、机身净空不足、超过台阶/坡度阈值以及落入语义 `pit` 多边形的体素。地形模型同时分割障碍物和可通行表面，并按到可通行边界的距离生成渐变代价；A* 把该代价计入实际路径开销，因此在有空间时优先走中间。

动态避障的实现架构、`demo/SCAN-Planner-Ros2` 方法分析、备选方案对比、首轮参数和从离线回放到 10～20 m 真机路线的分级测试门槛，见 [D1 动态避障设计与分阶段测试方案](DYNAMIC_OBSTACLE_AVOIDANCE.md)。当前实现保留保存地图全局 A*，用 D435i 建立带时间衰减的滚动 3D 障碍层，局部重规划后重新接回全局路径，并由独立速度安全门处理障碍、无路、命令超时和传感器故障。

针对 map022～map025 的地图质量、避障后失定位和安全停车问题，现行改造项、参数门槛及从本机零运动测试到 1 m 实物绕行的验收步骤，见 [D1 建图、定位、导航与动态避障系统性修改及测试文档](NAVIGATION_ROBUSTNESS_MODIFICATION_AND_TEST_PLAN.md)。

`saved_map_navigation.launch.py` 已完整接入动态避障。全局路径改为 `/navigation/global_path`，局部规划输出仍为 `/navigation/planned_path`；跟随器先发布 `/navigation/cmd_vel_raw`，安全门检查后才发布 `/navigation/cmd_vel`。因此现有网页和 D1 速度仲裁接口不变。默认 `dynamic_monitor_only:=false`，深度、CameraInfo、TF 或局部规划状态不健康时导航速度为零；首次相机外参与实物障碍测试可显式设为 `true`，只观察而不触发动态限速。

定位恢复采用有界状态机：单帧 ICP 异常先保留视觉惯性里程计 0.8 秒，随后停止平移，锁存丢失定位前最后一个路径航向的左右方向，并以 0.20 rad/s 持续同向旋转，同时通过 `/luxi_location/relocalization_request` 明确重启 HLoc。HLoc 候选进入 ICP 验证时保持静止，连续确认后才恢复原路径；45 秒仍未恢复则结束导航。定位节点随里程计持续刷新健康心跳，避免低频 ICP 间隔被误判为失联；速度安全门只放行带恢复标志的纯旋转，滚动障碍层为 `clear` 或 `slow` 时可转动，`blocked`、深度断流或障碍层超时仍输出零速度。相关参数位于 `config/navigation.yaml` 的 `terrain_path_follower` 和 `navigation_safety_gate` 段。

2026-08-15 曾记录一次 map023 + 前方垃圾桶运行遥测：D435i 障碍层约 10 Hz，日志显示局部路径热更新、净移动约 1.417 m，最终速度为零。但现场用户明确反馈尚未完成可见、可重复的实物绕障验收，因此该记录只算“链路遥测”，**不能标记为避障测试通过**。D1 低于 0.10 m/s 基本不产生有效步态，跟随器和安全门暂保留 0.10 m/s 的硬件最小有效平移速度；下一次必须按设计文档第 8、11 节重新完成有人监护的分级实测。

地形分割直接读取与 `.bt` 配套的 PLY：先按当前 0.05 m OctoMap 分辨率降采样，再以 `ground_normal_radius` 邻域估计局部法向。近水平点按相邻高度连续性拆成平面分量，只保留面积最大的连续主地面，每个 XY 位置只保留一个表面高度。局部法向转成墙面或相邻高度突然变化超过一个当前体素时，地面传播会停止，因此水平障碍顶部也不会单独成为可通行地面。只有高于附近主地面至少 `obstacle_min_height` 的实际点云栅格进入红色障碍层；法向不足且没有地面支撑的稀疏区域保持 unknown，不会仅因 `.bt` 的粗体素展开而变成障碍。`.bt` 继续提供地图范围和 OctoMap 发布，不能代替 PLY 地形拟合。

地图收到后会发布三个 transient-local 可视化 Marker：

- `/navigation/terrain/obstacles`：红色障碍物体素；
- `/navigation/terrain/traversable`：绿色可通行表面；
- `/navigation/terrain/costmap`：黄到红的边缘代价带。

网页仍发布二维点击目标 `(x, y, 0)`，规划器会在 `snap_search_radius_cells` 范围内自动吸附到可行走表面高度。输出的 `/navigation/planned_path` 保留每个节点的 z；轮式底盘跟随器只执行 x/y/偏航，并且必须额外收到 `/navigation/start=true` 才会运动。

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

`config/navigation.yaml` 中当前暂定 `robot_radius=0.10m`、`costmap_margin=0.60m`、`costmap_weight=8.0`，局部法向半径为 0.30 m、地面法向最大倾角为 35°、障碍最小离地高度为 0.15 m。这里的 35° 是允许地图整体倾斜和法向噪声的分割阈值，不是底盘最终可爬坡角；机器人尺寸与运动坡度必须在真机运动前实测。没有确认路径和定位方向前不要发布 `/navigation/start=true`。
