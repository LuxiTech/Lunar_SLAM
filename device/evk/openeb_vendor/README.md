# openeb_vendor

`openeb_vendor` 是一个 ROS vendor 包，用于在 ROS 2 构建环境中下载并构建 Prophesee OpenEB，也就是 MetaVision SDK 的开源部分。

上游项目：

```text
https://github.com/prophesee-ai/openeb
```

在当前 `Ultimate SLAM` 工作空间中，OpenEB 已经通过 ROS Lyrical 的 vendor 包解压到：

```text
.local_ros/opt/ros/lyrical/opt/openeb_vendor
```

因此本源码目录默认带有 `COLCON_IGNORE`，不会参与日常构建。这样可以避免在 Ubuntu 26.04 上重复源码编译 OpenEB 时引入大量系统依赖。

如后续需要从源码完整构建 OpenEB，可先安装系统依赖，再移除本目录下的 `COLCON_IGNORE`。

## 注意

上游曾提供 SilkyEVCam 插件补丁，但该补丁会影响 Prophesee 传感器，因此当前默认不启用。

## 许可证

本软件使用 Apache License 2.0。
