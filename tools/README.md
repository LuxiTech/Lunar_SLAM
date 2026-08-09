# 地图工具

## 统一查看地图

`maps_viwer` 根据文件后缀打开对应的只读地图视图：

- `.yaml` / `.yml`：使用 Nav2 `map_server` 和 RViz 显示二维占据栅格；
- `.pgm` / `.png`：自动查找同名的 `.yaml` 或 `.yml` 元数据后显示二维占据栅格；
- `.db`：临时导出 RTAB-Map 的 OccupancyMap 后，仅在 RViz 中显示三维占据体素；
- `.bt` / `.ot`：使用项目内的 OctoMap 加载器和 RViz 显示三维占据体素。

所有视图均为只读。默认 `.db` 与 `.bt/.ot` 三维窗口只显示占据体素；
两者均不显示二维地图、轨迹、相机画面或数据库面板。关闭查看窗口后，临时体素文件和
工具启动的 ROS 节点会自动退出。`.db` 打开前会临时导出 OccupancyMap，因此会有短暂准备时间。
查看正在写入的 RTAB-Map 数据库前，应先停止建图。

对于 `.db`，不带选项时显示 OccupancyMap 体素；`--color` 显示 RTAB-Map 从 RGB-D 数据
渲染出的彩色三维点云；`--both` 在同一个 RViz 窗口中叠加显示两者。组合窗口的 Displays
面板包含两个复选按键，可分别开关彩色点云和 OccupancyMap 体素。`--both` 会优先复用
`maps/octo_maps/mapNNN_octomap/` 中的 `.bt` 与 `*_cloud.ply`，缺少任一文件时自动调用
`export_rtabmap_octomap.sh` 生成它们。

```bash
cd /home/lunar/project/lunar_slam
tools/maps_viwer maps/rtab_maps/map011.db
tools/maps_viwer --color maps/rtab_maps/map011.db
tools/maps_viwer --both maps/rtab_maps/map011.db
tools/maps_viwer maps/octo_maps/map011_octomap/map011.bt
tools/maps_viwer /path/to/map.yaml
```

二维图像文件必须具有同名元数据，例如 `office.pgm` 对应 `office.yaml`；元数据中的
`image` 项必须指向该图像。无图形环境时，可验证识别结果而不启动窗口：

```bash
tools/maps_viwer --dry-run maps/rtab_maps/map011.db
```

在 Docker 中，工具会在 `DISPLAY` 指向不存在的 X11 socket、但已挂载 `X0` 时自动回退到
`DISPLAY=:0`。若仍没有图形窗口，在容器内执行：

```bash
export DISPLAY=:0
export XAUTHORITY=/tmp/.docker.xauth
export QT_X11_NO_MITSHM=1
```

可先检查工具最终会使用的 X11 设置：

```bash
tools/maps_viwer --check-display
```

## 导出 RTAB-Map 为 OctoMap

`export_rtabmap_octomap.sh` 将已经停止的 RTAB-Map 数据库转换为导航用 OctoMap：

```bash
cd /home/lunar/project/lunar_slam
tools/export_rtabmap_octomap.sh
```

默认选择 `maps/rtab_maps/` 中编号最大的 `mapNNN.db`，输出到
`maps/octo_maps/mapNNN_octomap/`，其中包含 RTAB-Map 导出的彩色 PLY 与 `mapNNN.bt`。
可显式传入数据库和输出目录：

```bash
tools/export_rtabmap_octomap.sh maps/rtab_maps/map011.db maps/octo_maps/map011_octomap
```

该工具只读取已保存数据库；若 RTAB-Map 正在运行，导出程序会拒绝执行，避免读取未
完成写入的地图。

需要同时清理离群噪点时添加 `--filter`。该开关在生成 OctoMap 前调用下节的 C++
过滤器；不带开关时保持原有转换行为。

```bash
tools/export_rtabmap_octomap.sh --filter \
  maps/rtab_maps/map042.db maps/octo_maps/map042_filtered_octomap
```

## 过滤地图点云噪声

`map_cloud_filter` 是 C++/PCL 工具，依次执行统计离群、半径邻域和小连通簇过滤。
默认参数针对本项目导出的 3 cm voxel 点云：20 邻域、2 倍标准差、12 cm 半径内至少
4 个邻居，并删除少于 20 点的独立簇。工具不会覆盖输入文件。

```bash
cd /home/nvidia/Desktop/lunar_slam
tools/map_cloud_filter/map_cloud_filter \
  maps/octo_maps/map042_octomap/luxi_rtab_map_20260809_124141_cloud.ply \
  maps/octo_maps/map042_octomap/map042_filtered_cloud.ply
```

默认参数保存在 `tools/map_cloud_filter/config/default.yaml`。网页生成过滤地图及
`export_rtabmap_octomap.sh --filter` 都会读取该文件；修改后无需重新编译。第一次运行
会在 `build/map_cloud_filter/` 编译 C++ 工具。命令行参数优先于 YAML，可按点云密度
临时覆盖：

```bash
tools/map_cloud_filter/map_cloud_filter INPUT.ply OUTPUT.ply \
  --mean-k 20 --stddev 2.0 \
  --radius 0.12 --min-neighbors 4 \
  --cluster-tolerance 0.12 --min-cluster-size 20
```

也可以选择另一份配置文件；其中未填写的字段继承内置默认值：

```bash
tools/map_cloud_filter/map_cloud_filter INPUT.ply OUTPUT.ply \
  --config /path/to/filter.yaml
```

`--min-z` 和 `--max-z` 可按 map 坐标系裁剪高度，但默认不启用。当前地图坐标系可能
随相机初始俯仰而倾斜，未确认地面方向时不应使用高度裁剪。

合并后的 PLY 不包含每个点对应的相机位姿，过滤器无法可靠判断该点是否位于原始帧的
相机视锥内。相机可见距离和深度边缘必须在 RTAB-Map 导出阶段通过 `min_range`、
`max_range` 和 `edge_bleeding_error` 约束；本工具只删除具有明确几何离群证据的点。

如需手动生成过滤后的 OctoMap：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH="$PWD/3parts/octomap/install/lib:${LD_LIBRARY_PATH:-}"
LD_PRELOAD=/lib/aarch64-linux-gnu/libusb-1.0.so.0 \
  ros2 run luxi_voxel_navigation ply_to_octomap \
  maps/octo_maps/map042_octomap/map042_filtered_cloud.ply \
  maps/octo_maps/map042_octomap/map042_filtered.bt 0.05
ros2 run luxi_3d_navigation terrain_plan_check \
  maps/octo_maps/map042_octomap/map042_filtered.bt
```

map042 默认参数回归结果：点数从 84213 降到 81390（删除 3.35%）；12 cm 内少于
4 个邻居的点从 26 降到 3，小于 20 点的独立簇从 9 个降到 0；20 邻域局部平面残差
小于 2 cm 的比例从 78.51% 提高到 80.12%。历史过滤版 10 cm OctoMap 识别 604 个
可站立栅格，并成功生成 86 个栅格的测试路径。完整流水线输出保存在
`maps/octo_maps/map042_filtered_pipeline/`。
