# 算法项目目录

`project` 存放与硬件无关的 ROS2 算法包。D435i 驱动及其数据接口保留在
`device/D435i`。

当前已实现：

```text
luxi_RTAB_Map/     # RGB-D RTAB-Map 建图
```

后续新增功能应使用独立包，例如定位、导航和语义识别，并订阅驱动或上游算法
发布的标准 ROS2 话题。
