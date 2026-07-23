# 已保存地图

每次启动 `rgbd_mapping.launch.py` 都会自动创建下一个编号的 RTAB-Map 数据库：

```text
map001.db
map002.db
map003.db
```

建图时数据库会持续写入。停止建图时，先在运行建图的终端按 `Ctrl-C`，等待
RTAB-Map 正常退出；对应的 `mapNNN.db` 即为完整保存的地图。相机驱动可以随后再
停止。

## 查看数据库

在容器中，先设置图形环境并加载工作空间：

```bash
export DISPLAY=:0
export XAUTHORITY=/tmp/.docker.xauth
export QT_X11_NO_MITSHM=1

source /opt/ros/humble/setup.bash
source /home/lunar/project/lunar_slam/device/D435i/ros2_ws/install/setup.bash
source /home/lunar/project/lunar_slam/install/setup.bash
```

推荐自动打开编号最大的、已经保存完成的地图：

```bash
source /opt/ros/humble/setup.bash
source /home/lunar/project/lunar_slam/install/setup.bash
ros2 run luxi_rtab_map view_latest_map.sh
```

脚本会自动选择最新的 `mapNNN.db`，例如当前的 `map011.db`；不需要手写编号。
若需确认它将选择哪个文件但不打开图形界面：

```bash
ros2 run luxi_rtab_map view_latest_map.sh --print-path
```

也可以显式查看某一张地图，例如第一张地图：

```bash
ros2 run luxi_rtab_map view_latest_map.sh \
  /home/lunar/project/lunar_slam/maps/map001.db
```

打开后可在 RTAB-Map Database Viewer 中检查轨迹、关键帧、二维栅格地图和三维点云。
查看时不要同时运行同一个数据库对应的建图进程。

## 导出为 PLY 三维点云

停止建图后，以下命令会自动导出编号最大的地图，例如 `map001.db` 到
`map001_export/`：

```bash
ros2 run luxi_rtab_map export_3d_map.sh
```

若要指定某一张地图，例如把 `map001.db` 导出到 `map001_export/`：

```bash
ros2 run luxi_rtab_map export_3d_map.sh \
  /home/lunar/project/lunar_slam/maps/map001.db \
  /home/lunar/project/lunar_slam/maps/map001_export
```

导出的 `*_cloud.ply` 可使用 CloudCompare 或 MeshLab 打开。将命令中的 `map001`
替换为所需地图编号即可。

## 继续已有地图

默认启动会创建新编号地图。若要继续 `map001.db`，在建图终端使用：

```bash
ros2 launch luxi_rtab_map rgbd_mapping.launch.py \
  database_path:=/home/lunar/project/lunar_slam/maps/map001.db
```
