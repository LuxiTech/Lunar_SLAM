# Third-party source packages

这里集中放置项目依赖的第三方或供应商 ROS2 源码包。它们不属于 LuXi 项目核心业务代码，原则上尽量少改；如果必须修改，请在提交说明里注明原因。

当前第三方来源：

- `diagnostics/`：ROS diagnostics 工具包。
- `octomap_msgs/`：OctoMap 消息定义。
- `perception_pcl/`：PCL ROS 工具包。
- `rtabmap/`：RTAB-Map 核心库。
- `rtabmap_ros/`：RTAB-Map ROS2 节点、消息和 RViz 插件。
- `serial_ros2/`：串口通信库。
- `yesense_ros2/`：H30 IMU 官方 ROS2 驱动。

注意：这些目录里有些本身是独立 Git 仓库，移动目录时已保留其 `.git` 信息。

