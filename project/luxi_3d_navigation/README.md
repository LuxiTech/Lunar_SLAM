# luxi_3d_navigation

该包在当前 HLoc 首次粗定位、连续三帧一致后交给 ICP 的定位链之后，使用地面支撑型三维 A* 对保存的 OctoMap 规划路径。候选扩展来自 3×3×3 邻域；规划会拒绝纯垂直运动、无地面支撑、机身净空不足、超过台阶/坡度阈值以及落入语义 `pit` 多边形的体素。

网页仍发布二维点击目标 `(x, y, 0)`，规划器会在 `snap_search_radius_cells` 范围内自动吸附到可行走表面高度。输出的 `/navigation/planned_path` 保留每个节点的 z；轮式底盘跟随器只执行 x/y/偏航，并且必须额外收到 `/navigation/start=true` 才会运动。

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
ros2 run luxi_3d_navigation terrain_plan_check /absolute/map.bt
```

`config/navigation.yaml` 中的机器人尺寸必须在真机运动前按底盘包络实测。当前默认值沿用现有工程的 `robot_radius=0.18m`；没有确认路径和定位方向前不要发布 `/navigation/start=true`。
