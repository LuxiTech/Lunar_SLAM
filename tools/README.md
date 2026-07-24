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
