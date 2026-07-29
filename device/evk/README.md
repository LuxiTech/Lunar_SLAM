# EVK4 事件相机模块

本目录只保留本项目对 Prophesee EVK4 HD 的 ROS 2 封装代码：

```text
device/evk
└── ros2_ws
    └── evk4_driver
```

MetaVision/OpenEB SDK 以及上游 ROS 事件相机依赖不随本仓库上传，使用时按需从官方或上游仓库下载。

## SDK 与依赖下载地址

```text
MetaVision/OpenEB:
https://github.com/prophesee-ai/openeb

ROS event_camera_msgs:
https://github.com/ros-event-camera/event_camera_msgs.git

ROS metavision_driver:
https://github.com/ros-event-camera/metavision_driver.git

ROS event_camera_renderer:
https://github.com/ros-event-camera/event_camera_renderer.git

ROS event_camera_codecs:
https://github.com/ros-event-camera/event_camera_codecs.git

ROS openeb_vendor:
https://github.com/ros-event-camera/openeb_vendor.git
```

建议将这些依赖放在同一个 ROS 2 workspace 中，再与 `device/evk/ros2_ws/evk4_driver` 一起构建。
