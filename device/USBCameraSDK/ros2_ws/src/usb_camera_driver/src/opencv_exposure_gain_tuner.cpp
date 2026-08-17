#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

using CompressedImage = sensor_msgs::msg::CompressedImage;

int xioctl(int fd, unsigned long request, void *argument) {
  int result = 0;
  do {
    result = ioctl(fd, request, argument);
  } while (result < 0 && errno == EINTR);
  return result;
}

class CameraControls {
 public:
  explicit CameraControls(const std::string &device) : device_(device) {
    fd_ = open(device.c_str(), O_RDWR | O_NONBLOCK);
    if (fd_ < 0) {
      throw std::runtime_error("open " + device + ": " + std::strerror(errno));
    }
  }

  ~CameraControls() {
    if (fd_ >= 0) {
      close(fd_);
    }
  }

  CameraControls(const CameraControls &) = delete;
  CameraControls &operator=(const CameraControls &) = delete;

  int get(uint32_t id) const {
    v4l2_control control{};
    control.id = id;
    if (xioctl(fd_, VIDIOC_G_CTRL, &control) < 0) {
      throw std::runtime_error("read control on " + device_ + ": " +
                               std::strerror(errno));
    }
    return control.value;
  }

  void set(uint32_t id, int value) const {
    v4l2_control control{};
    control.id = id;
    control.value = value;
    if (xioctl(fd_, VIDIOC_S_CTRL, &control) < 0) {
      throw std::runtime_error("set control on " + device_ + ": " +
                               std::strerror(errno));
    }
  }

  v4l2_queryctrl query(uint32_t id) const {
    v4l2_queryctrl query{};
    query.id = id;
    if (xioctl(fd_, VIDIOC_QUERYCTRL, &query) < 0) {
      throw std::runtime_error("query control on " + device_ + ": " +
                               std::strerror(errno));
    }
    return query;
  }

 private:
  std::string device_;
  int fd_ = -1;
};

class ExposureGainTuner : public rclcpp::Node {
 public:
  ExposureGainTuner() : Node("opencv_exposure_gain_tuner") {
    const auto topic = declare_parameter<std::string>(
        "camera_topic", "/left_camera/image/compressed");
    const auto left_device = declare_parameter<std::string>(
        "left_device",
        "/dev/v4l/by-path/platform-3610000.usb-usb-0:2.1:1.0-video-index0");
    const auto right_device = declare_parameter<std::string>(
        "right_device",
        "/dev/v4l/by-path/platform-3610000.usb-usb-0:2.3:1.0-video-index0");
    display_scale_ = declare_parameter<double>("display_scale", 0.65);
    if (display_scale_ <= 0.1 || display_scale_ > 1.0) {
      throw std::runtime_error("display_scale must be in (0.1, 1.0]");
    }

    left_ = std::make_unique<CameraControls>(left_device);
    right_ = std::make_unique<CameraControls>(right_device);
    const auto exposure_query = left_->query(V4L2_CID_EXPOSURE_ABSOLUTE);
    const auto gain_query = left_->query(V4L2_CID_GAIN);
    exposure_min_ = exposure_query.minimum;
    exposure_max_ = std::min<int>(
        exposure_query.maximum,
        static_cast<int>(declare_parameter<int>("exposure_slider_max", 1000)));
    gain_min_ = gain_query.minimum;
    gain_max_ = std::min<int>(
        gain_query.maximum,
        static_cast<int>(declare_parameter<int>("gain_slider_max", 68)));

    const int exposure_mode = left_->get(V4L2_CID_EXPOSURE_AUTO);
    auto_exposure_ = exposure_mode != V4L2_EXPOSURE_MANUAL ? 1 : 0;
    exposure_ = std::clamp(left_->get(V4L2_CID_EXPOSURE_ABSOLUTE),
                           exposure_min_, exposure_max_);
    gain_ = std::clamp(left_->get(V4L2_CID_GAIN), gain_min_, gain_max_);

    subscription_ = create_subscription<CompressedImage>(
        topic, rclcpp::QoS(1).reliable(),
        [this](CompressedImage::ConstSharedPtr message) {
          std::lock_guard<std::mutex> lock(frame_mutex_);
          latest_frame_ = std::move(message);
        });

    cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
    cv::resizeWindow(window_name_, 1280, 900);
    cv::createTrackbar("Auto exposure", window_name_, nullptr, 1,
                       &ExposureGainTuner::onAutoExposure, this);
    cv::setTrackbarPos("Auto exposure", window_name_, auto_exposure_);
    cv::createTrackbar("Exposure (100 us)", window_name_, nullptr,
                       exposure_max_, &ExposureGainTuner::onExposure, this);
    cv::setTrackbarMin("Exposure (100 us)", window_name_, exposure_min_);
    cv::setTrackbarPos("Exposure (100 us)", window_name_, exposure_);
    cv::createTrackbar("Gain", window_name_, nullptr, gain_max_,
                       &ExposureGainTuner::onGain, this);
    cv::setTrackbarMin("Gain", window_name_, gain_min_);
    cv::setTrackbarPos("Gain", window_name_, gain_);
    controls_ready_ = true;

    RCLCPP_INFO(get_logger(),
                "exposure tuner subscribed to %s; exposure range=%d..%d, "
                "gain range=%d..%d",
                topic.c_str(), exposure_min_, exposure_max_, gain_min_, gain_max_);
  }

  void run() {
    while (rclcpp::ok()) {
      auto frame = takeLatestFrame();
      if (frame) {
        render(*frame);
      }
      const int key = cv::waitKey(frame ? 1 : 10) & 0xff;
      if (key == 27 || key == 'q') {
        break;
      }
      if (key == 'p' || key == 's') {
        printConfiguration();
      }
    }
    cv::destroyWindow(window_name_);
  }

 private:
  static void onAutoExposure(int position, void *context) {
    auto *tuner = static_cast<ExposureGainTuner *>(context);
    tuner->auto_exposure_ = position;
    tuner->applyAutoExposure();
  }

  static void onExposure(int position, void *context) {
    auto *tuner = static_cast<ExposureGainTuner *>(context);
    tuner->exposure_ = position;
    tuner->applyExposure();
  }

  static void onGain(int position, void *context) {
    auto *tuner = static_cast<ExposureGainTuner *>(context);
    tuner->gain_ = position;
    tuner->applyGain();
  }

  void applyAutoExposure() {
    if (!controls_ready_) {
      return;
    }
    try {
      const int mode = auto_exposure_ ? V4L2_EXPOSURE_APERTURE_PRIORITY
                                      : V4L2_EXPOSURE_MANUAL;
      left_->set(V4L2_CID_EXPOSURE_AUTO, mode);
      right_->set(V4L2_CID_EXPOSURE_AUTO, mode);
      if (!auto_exposure_) {
        applyExposure();
      }
      RCLCPP_INFO(get_logger(), "auto exposure: %s",
                  auto_exposure_ ? "ON" : "OFF");
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "%s", error.what());
    }
  }

  void applyExposure() {
    if (!controls_ready_ || auto_exposure_) {
      return;
    }
    try {
      left_->set(V4L2_CID_EXPOSURE_ABSOLUTE, exposure_);
      right_->set(V4L2_CID_EXPOSURE_ABSOLUTE, exposure_);
      RCLCPP_INFO(get_logger(), "manual exposure: %d (%.1f ms)", exposure_,
                  exposure_ / 10.0);
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "%s", error.what());
    }
  }

  void applyGain() {
    if (!controls_ready_) {
      return;
    }
    try {
      left_->set(V4L2_CID_GAIN, gain_);
      right_->set(V4L2_CID_GAIN, gain_);
      RCLCPP_INFO(get_logger(), "gain: %d", gain_);
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "%s", error.what());
    }
  }

  CompressedImage::ConstSharedPtr takeLatestFrame() {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    auto frame = latest_frame_;
    latest_frame_.reset();
    return frame;
  }

  void render(const CompressedImage &message) {
    const cv::Mat encoded(1, static_cast<int>(message.data.size()), CV_8UC1,
                          const_cast<uint8_t *>(message.data.data()));
    const cv::Mat image = cv::imdecode(encoded, cv::IMREAD_COLOR);
    if (image.empty()) {
      return;
    }

    cv::Mat gray;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    const double mean = cv::mean(gray)[0];
    const double pixels = static_cast<double>(gray.total());
    const double underexposed = 100.0 * cv::countNonZero(gray < 5) / pixels;
    const double overexposed = 100.0 * cv::countNonZero(gray > 250) / pixels;

    cv::Mat display;
    cv::resize(image, display, cv::Size(), display_scale_, display_scale_,
               cv::INTER_AREA);
    std::ostringstream status;
    status << "Mean " << std::fixed << std::setprecision(1) << mean
           << "   Black " << underexposed << "%   Clipped " << overexposed
           << "%   [P/S] print YAML   [Q/Esc] quit";
    cv::rectangle(display, cv::Rect(0, 0, display.cols, 42),
                  cv::Scalar(20, 20, 20), cv::FILLED);
    cv::putText(display, status.str(), cv::Point(16, 29),
                cv::FONT_HERSHEY_SIMPLEX, 0.66, cv::Scalar(235, 235, 235), 2,
                cv::LINE_AA);
    cv::imshow(window_name_, display);
  }

  void printConfiguration() {
    RCLCPP_INFO(get_logger(),
                "copy to stereo_camera.yaml:\n"
                "    auto_exposure: %s\n"
                "    exposure_absolute: %d\n"
                "    gain: %d",
                auto_exposure_ ? "true" : "false", exposure_, gain_);
  }

  const std::string window_name_ = "USB Stereo Exposure / Gain Tuner";
  std::unique_ptr<CameraControls> left_;
  std::unique_ptr<CameraControls> right_;
  rclcpp::Subscription<CompressedImage>::SharedPtr subscription_;
  std::mutex frame_mutex_;
  CompressedImage::ConstSharedPtr latest_frame_;
  double display_scale_ = 0.65;
  int auto_exposure_ = 1;
  int exposure_ = 78;
  int gain_ = 0;
  int exposure_min_ = 1;
  int exposure_max_ = 10000;
  int gain_min_ = 0;
  int gain_max_ = 1023;
  bool controls_ready_ = false;
};

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<ExposureGainTuner>();
    std::thread spin_thread([node]() { rclcpp::spin(node); });
    node->run();
    rclcpp::shutdown();
    spin_thread.join();
  } catch (const std::exception &error) {
    std::cerr << "exposure tuner failed: " << error.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  return 0;
}
