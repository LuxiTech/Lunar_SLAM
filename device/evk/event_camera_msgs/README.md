# 事件相机 ROS 消息包

`event_camera_msgs` 定义事件视觉传感器在 ROS / ROS 2 中使用的消息类型。事件数据以紧凑的二进制格式保存，目的是减少录包、发布、订阅过程中的序列化和反序列化开销。

在当前工作空间中，它是 EVK4 主链路的基础消息包：

```text
event_camera_msgs -> metavision_driver -> evk4_driver
```

## 相关工具

- `event_camera_renderer`：将事件消息渲染为 ROS 图像消息。
- `event_camera_codecs`：提供 C++ 编解码接口。
- `event_camera_py`：提供 Python 侧快速读取和处理事件的模块。
- `event_camera_tools`：提供 echo、性能监测和格式转换工具。

## 消息说明

### EventPacket

`EventPacket` 用一个二进制数组保存一包事件。不同相机或驱动会使用不同的 `encoding`。

常见编码：

- `evt3`：Prophesee / MetaVision SDK 输出的原始 EVT3 数据。该格式需要解码后才能恢复传感器时间戳，`time_base` 字段不使用，内容未定义。
- `libcaer_cmp`：压缩的 libcaer 格式。压缩思路类似 EVT3，但会使用 `time_base` 恢复绝对传感器时间。
- `libcaer`：未压缩 libcaer 格式，每个事件占 64 bit。通常不如 `libcaer_cmp` 高效。
- `mono`：旧版 Prophesee 事件格式，已弃用。
- `trigger`：旧版外部触发事件格式，已弃用。

`mono` 旧格式的 64 bit 布局：

| 位范围 | 含义 |
|---|---|
| 63 | 极性：ON 为 1，OFF 为 0 |
| 48-62 | y 坐标 |
| 32-48 | x 坐标 |
| 0-32 | 时间增量 dt |

`trigger` 旧格式的 64 bit 布局：

| 位范围 | 含义 |
|---|---|
| 63 | 极性：ON 为 1，OFF 为 0 |
| 33-62 | 未使用 |
| 0-32 | 时间增量 dt |

恢复旧格式传感器时间时，将 `dt` 加到消息的 `time_base`。恢复 ROS 估计时间时，将 `dt` 加到消息头的 `stamp`。

## 许可证

本软件使用 Apache License 2.0，详见 `LICENSE`。
