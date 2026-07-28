# EVK 事件相机模块

本目录集中管理 Prophesee EVK4 HD 事件相机相关内容。

```text
src/evk
├── event_camera_msgs       # 事件相机 ROS 消息定义
├── metavision_driver       # 基于 MetaVision/OpenEB 的底层 ROS 驱动
├── evk4_driver             # 本工作空间的 EVK4 HD 封装功能包
├── openeb_vendor           # OpenEB vendor 包，当前默认忽略源码构建
├── event_camera_codecs     # 事件消息解码工具，当前默认忽略
└── event_camera_renderer   # 事件可视化工具，当前默认忽略
```

当前主链路为：

```text
event_camera_msgs -> metavision_driver -> evk4_driver
```

`evk4_driver` 是后续开发主要入口；上游包尽量保持原样，便于以后同步更新。
