# Lunar Client 闭源运行包

这是 `project/` 的 ARM64 二进制发行版。算法实现以 Release ELF 或 Python 3.10
字节码提供；仓库只公开 ROS 2 话题、服务、参数、launch/config 和 C++ 头文件接口。

## 运行环境

- Jetson Linux / Ubuntu 22.04，`aarch64`
- ROS 2 Humble，Python 3.10
- 与构建机兼容的 CUDA、VPI、Open3D、PCL、OctoMap、RTAB-Map 运行库
- 仓库中的设备驱动和 `3parts` 运行依赖

从 `lunar-client` 分支仓库根目录解压制品：

```bash
tar -xzf artifacts/lunar-client-linux-aarch64-ros2-humble.tar.gz
source ./activate.bash
ros2 pkg executables | grep '^luxi_'
```

然后可通过公开 launch 启动，例如：

```bash
ros2 launch luxi_adapter sensor_bringup.launch.py hardware:=d455
ros2 launch luxi_rtab_map rgbd_mapping_learned.launch.py \
  new_map:=true load_saved_map:=false
```

完整接口见 [API.md](API.md)。二进制只兼容 ARM64/Humble/Python 3.10；升级 ROS、
Python、CUDA 或系统 ABI 后需要重新发布。

## 闭源说明

本发行包不含 `project/` 的 C++/Python 算法实现源码。launch、YAML/XML 配置、ROS 消息
生成代码和公开头文件属于调用接口，因此保留为可读形式。Python 字节码可被逆向分析，
它是交付封装而不是硬件级防提取机制。
