# luxi_semantic_annotation

该包为已保存的 OctoMap 提供离线语义标注校验与原子保存能力。原始 `.bt` 和彩色
PLY 保持只读；岩石、墙附着到高于地面阈值的占用体素，坑保存为地面多边形和估计深度。

从工作区根目录构建并运行测试：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select luxi_semantic_annotation luxi_web_control \
  --symlink-install
colcon test --packages-select luxi_semantic_annotation luxi_web_control
colcon test-result --verbose
```

标注入口集成在 `luxi_web_control` 的已有地图页面。输出目录为：

```text
maps/semantic_maps/mapNNN/annotations.json
```

命令行检查现有地图：

```bash
ros2 run luxi_semantic_annotation semantic_annotation_tool inspect \
  maps/octo_maps/map015_octomap/map015.bt
```

数据格式：

```json
{
  "schema_version": 1,
  "map_id": "map015",
  "frame_id": "map",
  "ground": {"z": 0.0, "minimum_height": 0.15},
  "occupied_labels": [
    {"type": "rock", "x": 1.0, "y": 2.0, "z": 0.4, "size": 0.1}
  ],
  "pits": [
    {
      "id": "pit_001",
      "type": "pit",
      "depth": 0.3,
      "polygon": [[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]]
    }
  ]
}
```
