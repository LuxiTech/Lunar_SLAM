# 地图转换工具

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
