# event_camera_codecs

`event_camera_codecs` 提供 `event_camera_msgs` 的 C++ 解码工具。它支持 ROS 1 和 ROS 2，也可以通过 `event_camera_py` 在 Python 中间接使用。

在当前 `Ultimate SLAM` 工作空间中，本包属于可选工具，默认带有 `COLCON_IGNORE`。EVK4 驱动主链路暂时不需要构建它；后续需要离线解码、渲染、转换或与图像帧同步时，再启用本包。

## 支持平台

上游测试过的平台：

- ROS 1：Ubuntu 20.04 + Noetic
- ROS 2：Ubuntu 22.04 + Humble / Iron / Rolling

当前工作空间使用 Ubuntu 26.04 + ROS 2 Lyrical，日常 EVK4 驱动构建默认不启用本包。

## 构建

启用本包前，先移除：

```bash
src/evk/event_camera_codecs/COLCON_IGNORE
```

然后构建：

```bash
cd "/home/changxin/Ultimate SLAM"
source setup_evk4_driver.bash
colcon build --symlink-install --packages-select event_camera_codecs \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

如需构建单元测试，额外添加：

```bash
-DEVENT_CAMERA_CODECS_BUILD_TESTS=ON
```

## C++ API 示例

典型用法是继承 `event_camera_codecs::EventProcessor`，然后把 `EventPacket` 交给 decoder。

```cpp
#include <event_camera_codecs/decoder.h>
#include <event_camera_codecs/decoder_factory.h>

using event_camera_codecs::EventPacket;

class MyProcessor : public event_camera_codecs::EventProcessor
{
public:
  inline void eventCD(uint64_t, uint16_t ex, uint16_t ey, uint8_t polarity) override {
    // 在这里处理普通 CD 事件
  }

  bool eventExtTrigger(uint64_t, uint8_t, uint8_t) override {
    return true;
  }

  void finished() override {}
  void rawData(const char *, size_t) override {}
};

MyProcessor processor;
event_camera_codecs::DecoderFactory<EventPacket, MyProcessor> decoderFactory;

void eventMsg(const event_camera_codecs::EventPacketConstSharedPtr & msg) {
  auto decoder = decoderFactory.getInstance(*msg);
  if (!decoder) {
    return;
  }
  decoder->decode(*msg, &processor);
}
```

如果需要和帧相机同步，可以使用 `decodeUntil()` 解码到某个帧时间边界。不要在同一个 decoder 上混用 `decode()` 和 `decodeUntil()`。

## 支持的编码

- `evt3`：Prophesee / MetaVision 相机的原生 EVT3 格式。
- `libcaer_cmp`：Inivation Labs 相机的压缩 libcaer 格式。
- `libcaer`：未压缩 libcaer 格式，适合 CPU 慢但内存带宽高的场景。
- `mono`：旧版 Prophesee 临时格式，已弃用。
- `trigger`：旧版触发事件格式，已弃用。

## 事件时间戳

decoder 返回的事件时间戳指的是设备内部的传感器时间，而不是主机时间。时间戳单位为纳秒，但大多数设备实际精度不会高于微秒。

一些编码会使用 `time_base` 字段恢复传感器时间，例如 `libcaer` 和 `libcaer_cmp`。对于 `evt3`，必须解码消息内容才能恢复传感器时间。

EVT3 时间戳有几个需要注意的点：

- 时间戳字段只有 24 bit，因此每 `2^24` 微秒，也就是约 16.77 秒会回绕一次。
- decoder 会跟踪回绕，让上层看起来像 64 bit 时间戳，但底层原始字段并不是 64 bit。
- 某些设备可能存在时间戳位错误，可能导致回绕判断异常，使时间突然偏移约 16.77 秒。
- 多相机同步时，如果两个事件流在首次解码前后处于不同回绕区间，传感器时间可能相差 16.77 秒，即使硬件同步线已经连接。

## 工具

开发者可用 `codec_perf` 测量 EVT3 解码性能：

```bash
ros2 run event_camera_codecs codec_perf -i foo.raw
```

## 许可证

本软件使用 Apache License 2.0。
