#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

constexpr uint32_t kBufferCount = 4;

int xioctl(int fd, unsigned long request, void *argument) {
  int result;
  do {
    result = ioctl(fd, request, argument);
  } while (result == -1 && errno == EINTR);
  return result;
}

uint32_t parsePixelFormat(const std::string &format) {
  if (format == "MJPG" || format == "MJPEG") {
    return V4L2_PIX_FMT_MJPEG;
  }
  if (format == "YUYV" || format == "YUY2") {
    return V4L2_PIX_FMT_YUYV;
  }
  throw std::runtime_error("pixel_format must be MJPG or YUYV");
}

enum class OutputFormat { kBgr8, kMono8, kJpeg };

OutputFormat parseOutputFormat(const std::string &encoding) {
  if (encoding == "mono8") {
    return OutputFormat::kMono8;
  }
  if (encoding == "bgr8") {
    return OutputFormat::kBgr8;
  }
  if (encoding == "jpeg") {
    return OutputFormat::kJpeg;
  }
  throw std::runtime_error("output_encoding must be bgr8, mono8, or jpeg");
}

struct MappedBuffer {
  void *start = nullptr;
  size_t length = 0;
};

struct CaptureMetadata {
  int64_t v4l2_timestamp_ns = 0;
  int64_t host_dequeue_timestamp_ns = 0;
  uint32_t sequence = 0;
  uint32_t flags = 0;
};

int64_t realtimeNowNanoseconds() {
  timespec timestamp {};
  if (clock_gettime(CLOCK_REALTIME, &timestamp) != 0) {
    throw std::runtime_error("clock_gettime: " +
                             std::string(std::strerror(errno)));
  }
  return static_cast<int64_t>(timestamp.tv_sec) * 1000000000LL +
         timestamp.tv_nsec;
}

class V4l2Camera {
 public:
  V4l2Camera() = default;
  ~V4l2Camera() { close(); }

  V4l2Camera(const V4l2Camera &) = delete;
  V4l2Camera &operator=(const V4l2Camera &) = delete;

  void open(const std::string &device, uint32_t pixel_format, uint32_t width,
            uint32_t height, double frame_rate_hz, bool hardware_trigger,
            bool auto_exposure, int exposure_absolute, int gain,
            OutputFormat output_format) {
    fd_ = ::open(device.c_str(), O_RDWR | O_NONBLOCK);
    if (fd_ < 0) {
      throw std::runtime_error("open " + device + ": " + std::strerror(errno));
    }

    v4l2_capability capability {};
    if (xioctl(fd_, VIDIOC_QUERYCAP, &capability) < 0 ||
        !(capability.capabilities & V4L2_CAP_VIDEO_CAPTURE) ||
        !(capability.capabilities & V4L2_CAP_STREAMING)) {
      throw std::runtime_error(device + " is not a streaming capture device");
    }

    setTriggerMode(hardware_trigger);
    setImageControls(auto_exposure, exposure_absolute, gain);

    v4l2_format format {};
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = width;
    format.fmt.pix.height = height;
    format.fmt.pix.pixelformat = pixel_format;
    format.fmt.pix.field = V4L2_FIELD_ANY;
    if (xioctl(fd_, VIDIOC_S_FMT, &format) < 0) {
      throw std::runtime_error("VIDIOC_S_FMT: " + std::string(std::strerror(errno)));
    }
    width_ = format.fmt.pix.width;
    height_ = format.fmt.pix.height;
    pixel_format_ = format.fmt.pix.pixelformat;
    output_format_ = output_format;
    if (output_format_ == OutputFormat::kJpeg &&
        pixel_format_ != V4L2_PIX_FMT_MJPEG) {
      throw std::runtime_error("jpeg output requires MJPG camera input");
    }

    v4l2_streamparm parameters {};
    parameters.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parameters.parm.capture.timeperframe.numerator = 1;
    parameters.parm.capture.timeperframe.denominator =
        static_cast<uint32_t>(frame_rate_hz);
    xioctl(fd_, VIDIOC_S_PARM, &parameters);

    v4l2_requestbuffers request {};
    request.count = kBufferCount;
    request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    request.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd_, VIDIOC_REQBUFS, &request) < 0 || request.count < 2) {
      throw std::runtime_error("VIDIOC_REQBUFS: " + std::string(std::strerror(errno)));
    }

    const uint32_t buffer_count = std::min(request.count, kBufferCount);
    buffers_.resize(buffer_count);
    for (uint32_t index = 0; index < buffer_count; ++index) {
      v4l2_buffer buffer {};
      buffer.type = request.type;
      buffer.memory = request.memory;
      buffer.index = index;
      if (xioctl(fd_, VIDIOC_QUERYBUF, &buffer) < 0) {
        throw std::runtime_error("VIDIOC_QUERYBUF: " + std::string(std::strerror(errno)));
      }
      buffers_[index].length = buffer.length;
      buffers_[index].start = mmap(nullptr, buffer.length, PROT_READ | PROT_WRITE,
                                   MAP_SHARED, fd_, buffer.m.offset);
      if (buffers_[index].start == MAP_FAILED) {
        buffers_[index].start = nullptr;
        throw std::runtime_error("mmap: " + std::string(std::strerror(errno)));
      }
      if (xioctl(fd_, VIDIOC_QBUF, &buffer) < 0) {
        throw std::runtime_error("VIDIOC_QBUF: " + std::string(std::strerror(errno)));
      }
    }

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
      throw std::runtime_error("VIDIOC_STREAMON: " + std::string(std::strerror(errno)));
    }
    streaming_ = true;
  }

  bool tryGrabLatest(cv::Mat &image, std::vector<uint8_t> &jpeg,
                     CaptureMetadata &metadata) {
    v4l2_buffer latest {};
    latest.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    latest.memory = V4L2_MEMORY_MMAP;
    if (xioctl(fd_, VIDIOC_DQBUF, &latest) < 0) {
      if (errno == EAGAIN) {
        return false;
      }
      throw std::runtime_error("VIDIOC_DQBUF: " + std::string(std::strerror(errno)));
    }
    int64_t latest_host_timestamp_ns = realtimeNowNanoseconds();

    // Drain completed buffers before decoding so downstream load cannot create
    // an ever-growing display delay.
    while (true) {
      v4l2_buffer next {};
      next.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      next.memory = V4L2_MEMORY_MMAP;
      if (xioctl(fd_, VIDIOC_DQBUF, &next) < 0) {
        if (errno == EAGAIN) {
          break;
        }
        xioctl(fd_, VIDIOC_QBUF, &latest);
        throw std::runtime_error("VIDIOC_DQBUF: " +
                                 std::string(std::strerror(errno)));
      }
      const int64_t next_host_timestamp_ns = realtimeNowNanoseconds();
      if (xioctl(fd_, VIDIOC_QBUF, &latest) < 0) {
        xioctl(fd_, VIDIOC_QBUF, &next);
        throw std::runtime_error("VIDIOC_QBUF: " +
                                 std::string(std::strerror(errno)));
      }
      latest = next;
      latest_host_timestamp_ns = next_host_timestamp_ns;
    }

    try {
      if (output_format_ == OutputFormat::kJpeg) {
        const auto *begin = static_cast<const uint8_t *>(buffers_[latest.index].start);
        jpeg.assign(begin, begin + latest.bytesused);
      } else if (pixel_format_ == V4L2_PIX_FMT_MJPEG) {
        const cv::Mat encoded(1, static_cast<int>(latest.bytesused), CV_8UC1,
                              buffers_[latest.index].start);
        image = cv::imdecode(
            encoded, output_format_ == OutputFormat::kMono8
                         ? cv::IMREAD_GRAYSCALE
                         : cv::IMREAD_COLOR);
      } else {
        const cv::Mat yuyv(static_cast<int>(height_), static_cast<int>(width_),
                           CV_8UC2, buffers_[latest.index].start);
        cv::cvtColor(yuyv, image,
                     output_format_ == OutputFormat::kMono8
                         ? cv::COLOR_YUV2GRAY_YUY2
                         : cv::COLOR_YUV2BGR_YUY2);
      }
    } catch (...) {
      xioctl(fd_, VIDIOC_QBUF, &latest);
      throw;
    }
    if (xioctl(fd_, VIDIOC_QBUF, &latest) < 0) {
      throw std::runtime_error("VIDIOC_QBUF: " + std::string(std::strerror(errno)));
    }
    if (output_format_ == OutputFormat::kJpeg && jpeg.empty()) {
      throw std::runtime_error("camera returned an empty JPEG frame");
    }
    if (output_format_ != OutputFormat::kJpeg && image.empty()) {
      throw std::runtime_error("camera returned an undecodable frame");
    }
    metadata.v4l2_timestamp_ns =
        static_cast<int64_t>(latest.timestamp.tv_sec) * 1000000000LL +
        static_cast<int64_t>(latest.timestamp.tv_usec) * 1000LL;
    metadata.host_dequeue_timestamp_ns = latest_host_timestamp_ns;
    metadata.sequence = latest.sequence;
    metadata.flags = latest.flags;
    return true;
  }

  int fd() const { return fd_; }
  uint32_t width() const { return width_; }
  uint32_t height() const { return height_; }

  void close() {
    if (fd_ < 0) {
      return;
    }
    if (streaming_) {
      int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      xioctl(fd_, VIDIOC_STREAMOFF, &type);
    }
    for (const auto &buffer : buffers_) {
      if (buffer.start != nullptr) {
        munmap(buffer.start, buffer.length);
      }
    }
    buffers_.clear();
    ::close(fd_);
    fd_ = -1;
    streaming_ = false;
  }

 private:
  void setTriggerMode(bool hardware_trigger) {
    v4l2_control control {};
    control.id = V4L2_CID_BACKLIGHT_COMPENSATION;
    control.value = hardware_trigger ? 2 : 0;
    if (xioctl(fd_, VIDIOC_S_CTRL, &control) < 0) {
      throw std::runtime_error("set Backlight Compensation: " +
                               std::string(std::strerror(errno)));
    }
    if (xioctl(fd_, VIDIOC_G_CTRL, &control) < 0 ||
        control.value != (hardware_trigger ? 2 : 0)) {
      throw std::runtime_error("Backlight Compensation readback did not match trigger mode");
    }
  }

  void setImageControls(bool auto_exposure, int exposure_absolute, int gain) {
    setControl(V4L2_CID_EXPOSURE_AUTO,
               auto_exposure ? V4L2_EXPOSURE_APERTURE_PRIORITY : V4L2_EXPOSURE_MANUAL);
    if (!auto_exposure && exposure_absolute >= 1) {
      setControl(V4L2_CID_EXPOSURE_ABSOLUTE, exposure_absolute);
    }
    if (gain >= 0) {
      setControl(V4L2_CID_GAIN, gain);
    }
  }

  void setControl(uint32_t id, int value) {
    v4l2_control control {};
    control.id = id;
    control.value = value;
    if (xioctl(fd_, VIDIOC_S_CTRL, &control) < 0) {
      throw std::runtime_error("set V4L2 control: " + std::string(std::strerror(errno)));
    }
  }

  int fd_ = -1;
  bool streaming_ = false;
  uint32_t width_ = 0;
  uint32_t height_ = 0;
  uint32_t pixel_format_ = 0;
  OutputFormat output_format_ = OutputFormat::kBgr8;
  std::vector<MappedBuffer> buffers_;
};

class StereoNode : public rclcpp::Node {
 public:
  StereoNode() : Node("stereo_node") {
    const auto left_device = declare_parameter<std::string>("left_device", "/dev/video0");
    const auto right_device = declare_parameter<std::string>("right_device", "/dev/video2");
    const auto pixel_format = parsePixelFormat(declare_parameter<std::string>("pixel_format", "MJPG"));
    output_encoding_ = declare_parameter<std::string>("output_encoding", "bgr8");
    const auto output_format = parseOutputFormat(output_encoding_);
    compressed_output_ = output_format == OutputFormat::kJpeg;
    const auto width = static_cast<uint32_t>(declare_parameter<int>("width", 1920));
    const auto height = static_cast<uint32_t>(declare_parameter<int>("height", 1080));
    const auto frame_rate = declare_parameter<double>("frame_rate_hz", 10.0);
    const auto trigger_mode = declare_parameter<std::string>("trigger_mode", "hardware");
    const auto timestamp_offset = declare_parameter<double>("timestamp_offset_sec", 0.0);
    const auto auto_exposure = declare_parameter<bool>("auto_exposure", true);
    const auto exposure_absolute = declare_parameter<int>("exposure_absolute", -1);
    const auto gain = declare_parameter<int>("gain", -1);
    left_frame_id_ = declare_parameter<std::string>("left_frame_id", "left_camera_optical_frame");
    right_frame_id_ = declare_parameter<std::string>("right_frame_id", "right_camera_optical_frame");
    if (frame_rate <= 0.0 || (trigger_mode != "hardware" && trigger_mode != "video")) {
      throw std::runtime_error("frame_rate_hz must be positive and trigger_mode must be hardware or video");
    }
    max_pair_interval_ns_ =
        static_cast<int64_t>(500000000.0 / frame_rate);
    timestamp_offset_ns_ = std::chrono::duration_cast<std::chrono::nanoseconds>(
                               std::chrono::duration<double>(timestamp_offset))
                               .count();

    const auto qos = rclcpp::SensorDataQoS().keep_last(1);
    if (compressed_output_) {
      const auto compressed_qos = rclcpp::QoS(1).reliable();
      left_compressed_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
          "/left_camera/image/compressed", compressed_qos);
      right_compressed_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
          "/right_camera/image/compressed", compressed_qos);
    } else {
      left_pub_ = create_publisher<sensor_msgs::msg::Image>("/left_camera/image", qos);
      right_pub_ = create_publisher<sensor_msgs::msg::Image>("/right_camera/image", qos);
    }
    left_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("/left_camera/camera_info", qos);
    right_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("/right_camera/camera_info", qos);

    left_.open(left_device, pixel_format, width, height, frame_rate,
               trigger_mode == "hardware", auto_exposure, exposure_absolute,
               gain, output_format);
    right_.open(right_device, pixel_format, width, height, frame_rate,
                trigger_mode == "hardware", auto_exposure, exposure_absolute,
                gain, output_format);
    RCLCPP_INFO(get_logger(),
                "opened stereo UVC cameras: %s and %s, trigger_mode=%s, output=%s",
                left_device.c_str(), right_device.c_str(), trigger_mode.c_str(),
                output_encoding_.c_str());

    capture_thread_ = std::thread(&StereoNode::captureLoop, this);
  }

  ~StereoNode() override {
    running_ = false;
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
  }

 private:
  void captureLoop() {
    while (running_ && rclcpp::ok()) {
      try {
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(left_.fd(), &read_fds);
        FD_SET(right_.fd(), &read_fds);
        timeval timeout {1, 0};
        const int ready = select(std::max(left_.fd(), right_.fd()) + 1, &read_fds,
                                 nullptr, nullptr, &timeout);
        if (ready < 0) {
          if (errno != EINTR) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                                  "select failed: %s", std::strerror(errno));
          }
          continue;
        }
        if (ready == 0) {
          continue;
        }
        if (FD_ISSET(left_.fd(), &read_fds)) {
          left_.tryGrabLatest(left_pending_, left_jpeg_pending_, left_metadata_);
        }
        if (FD_ISSET(right_.fd(), &read_fds)) {
          right_.tryGrabLatest(right_pending_, right_jpeg_pending_, right_metadata_);
        }
        const bool left_ready = compressed_output_ ? !left_jpeg_pending_.empty()
                                                   : !left_pending_.empty();
        const bool right_ready = compressed_output_ ? !right_jpeg_pending_.empty()
                                                    : !right_pending_.empty();
        if (left_ready && right_ready) {
          const int64_t timestamp_delta_ns =
              left_metadata_.host_dequeue_timestamp_ns -
              right_metadata_.host_dequeue_timestamp_ns;
          if (timestamp_delta_ns < -max_pair_interval_ns_) {
            left_pending_.release();
            left_jpeg_pending_.clear();
            continue;
          }
          if (timestamp_delta_ns > max_pair_interval_ns_) {
            right_pending_.release();
            right_jpeg_pending_.clear();
            continue;
          }
          publishPair();
        }
      } catch (const std::exception &error) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                              "capture error: %s", error.what());
        left_pending_.release();
        right_pending_.release();
        left_jpeg_pending_.clear();
        right_jpeg_pending_.clear();
      }
    }
  }

  void publishPair() {
    const int64_t pair_timestamp_ns =
        left_metadata_.host_dequeue_timestamp_ns +
        (right_metadata_.host_dequeue_timestamp_ns -
         left_metadata_.host_dequeue_timestamp_ns) /
            2 +
        timestamp_offset_ns_;
    const auto stamp = rclcpp::Time(pair_timestamp_ns, RCL_SYSTEM_TIME);
    const bool left_monotonic =
        (left_metadata_.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) ==
        V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
    const bool right_monotonic =
        (right_metadata_.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) ==
        V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
    if (!timestamp_source_logged_) {
      const double delta_ms = static_cast<double>(
                                  left_metadata_.host_dequeue_timestamp_ns -
                                  right_metadata_.host_dequeue_timestamp_ns) /
                              1e6;
      RCLCPP_INFO(get_logger(),
                  "capture timestamps: left_seq=%u right_seq=%u dequeue_delta=%.3f ms "
                  "left_flags=0x%x right_flags=0x%x source=host dequeue "
                  "timestamp_offset=%.3f ms "
                  "(V4L2 monotonic flags=%s)",
                  left_metadata_.sequence, right_metadata_.sequence, delta_ms,
                  left_metadata_.flags, right_metadata_.flags,
                  static_cast<double>(timestamp_offset_ns_) / 1e6,
                  left_monotonic && right_monotonic ? "yes" : "no");
      timestamp_source_logged_ = true;
    }
    if (compressed_output_) {
      sensor_msgs::msg::CompressedImage left;
      left.header.stamp = stamp;
      left.header.frame_id = left_frame_id_;
      left.format = "jpeg";
      left.data = std::move(left_jpeg_pending_);

      sensor_msgs::msg::CompressedImage right;
      right.header.stamp = stamp;
      right.header.frame_id = right_frame_id_;
      right.format = "jpeg";
      right.data = std::move(right_jpeg_pending_);

      sensor_msgs::msg::CameraInfo left_info;
      left_info.header = left.header;
      left_info.width = left_.width();
      left_info.height = left_.height();
      sensor_msgs::msg::CameraInfo right_info;
      right_info.header = right.header;
      right_info.width = right_.width();
      right_info.height = right_.height();

      left_compressed_pub_->publish(std::move(left));
      right_compressed_pub_->publish(std::move(right));
      left_info_pub_->publish(left_info);
      right_info_pub_->publish(right_info);
      return;
    }

    auto left = sensor_msgs::msg::Image();
    left.header.stamp = stamp;
    left.header.frame_id = left_frame_id_;
    left.height = static_cast<uint32_t>(left_pending_.rows);
    left.width = static_cast<uint32_t>(left_pending_.cols);
    left.encoding = output_encoding_;
    left.is_bigendian = false;
    left.step = static_cast<uint32_t>(left_pending_.cols * left_pending_.elemSize());
    left.data.assign(left_pending_.datastart, left_pending_.dataend);

    auto right = sensor_msgs::msg::Image();
    right.header.stamp = stamp;
    right.header.frame_id = right_frame_id_;
    right.height = static_cast<uint32_t>(right_pending_.rows);
    right.width = static_cast<uint32_t>(right_pending_.cols);
    right.encoding = output_encoding_;
    right.is_bigendian = false;
    right.step = static_cast<uint32_t>(right_pending_.cols * right_pending_.elemSize());
    right.data.assign(right_pending_.datastart, right_pending_.dataend);

    sensor_msgs::msg::CameraInfo left_info;
    left_info.header = left.header;
    left_info.width = left.width;
    left_info.height = left.height;
    sensor_msgs::msg::CameraInfo right_info;
    right_info.header = right.header;
    right_info.width = right.width;
    right_info.height = right.height;

    left_pub_->publish(left);
    right_pub_->publish(right);
    left_info_pub_->publish(left_info);
    right_info_pub_->publish(right_info);
    left_pending_.release();
    right_pending_.release();
  }

  V4l2Camera left_;
  V4l2Camera right_;
  cv::Mat left_pending_;
  cv::Mat right_pending_;
  std::vector<uint8_t> left_jpeg_pending_;
  std::vector<uint8_t> right_jpeg_pending_;
  CaptureMetadata left_metadata_;
  CaptureMetadata right_metadata_;
  std::string left_frame_id_;
  std::string right_frame_id_;
  std::string output_encoding_;
  bool compressed_output_ = false;
  bool timestamp_source_logged_ = false;
  int64_t max_pair_interval_ns_ = 0;
  int64_t timestamp_offset_ns_ = 0;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr left_compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr right_compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_pub_;
  std::atomic_bool running_{true};
  std::thread capture_thread_;
};

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StereoNode>());
  rclcpp::shutdown();
  return 0;
}
