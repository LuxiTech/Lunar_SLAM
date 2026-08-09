# luxi_3d_navigation

该包在当前 HLoc 首次粗定位、连续三帧一致后交给 ICP 的定位链之后，使用地面支撑型三维 A* 对保存的 OctoMap 规划路径。候选扩展来自 3×3×3 邻域；规划会拒绝纯垂直运动、无地面支撑、机身净空不足、超过台阶/坡度阈值以及落入语义 `pit` 多边形的体素。地形模型同时分割障碍物和可通行表面，并按到可通行边界的距离生成渐变代价；A* 把该代价计入实际路径开销，因此在有空间时优先走中间。

地形分割直接读取与 `.bt` 配套的 PLY：先按 OctoMap 分辨率降采样，再以 `ground_normal_radius` 邻域估计局部法向。近水平点按相邻高度连续性拆成平面分量，只保留面积最大的连续主地面，每个 XY 位置只保留一个表面高度。局部法向转成墙面或相邻高度突然变化超过一个 0.10 m 栅格时，地面传播会停止，因此水平障碍顶部也不会单独成为可通行地面。只有高于附近主地面至少 `obstacle_min_height` 的实际点云栅格进入红色障碍层；法向不足且没有地面支撑的稀疏区域保持 unknown，不会仅因 `.bt` 的粗体素展开而变成障碍。`.bt` 继续提供地图范围和 OctoMap 发布，不能代替 PLY 地形拟合。

地图收到后会发布三个 transient-local 可视化 Marker：

- `/navigation/terrain/obstacles`：红色障碍物体素；
- `/navigation/terrain/traversable`：绿色可通行表面；
- `/navigation/terrain/costmap`：黄到红的边缘代价带。

网页仍发布二维点击目标 `(x, y, 0)`，规划器会在 `snap_search_radius_cells` 范围内自动吸附到可行走表面高度。输出的 `/navigation/planned_path` 保留每个节点的 z；轮式底盘跟随器只执行 x/y/偏航，并且必须额外收到 `/navigation/start=true` 才会运动。

```bash
cd /home/nvidia/Desktop/lunar_slam
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

`config/navigation.yaml` 中当前暂定 `robot_radius=0.10m`、`costmap_margin=0.60m`、`costmap_weight=8.0`，局部法向半径为 0.30 m、地面法向最大倾角为 35°、障碍最小离地高度为 0.15 m。这里的 35° 是允许地图整体倾斜和法向噪声的分割阈值，不是底盘最终可爬坡角；机器人尺寸与运动坡度必须在真机运动前实测。没有确认路径和定位方向前不要发布 `/navigation/start=true`。
