#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {

using CompressedImage = sensor_msgs::msg::CompressedImage;

uint64_t stampNanoseconds(const builtin_interfaces::msg::Time &stamp) {
  return static_cast<uint64_t>(stamp.sec) * 1000000000ULL + stamp.nanosec;
}

class OpenCvFocusViewer : public rclcpp::Node {
 public:
  OpenCvFocusViewer() : Node("opencv_focus_viewer") {
    const auto topic = declare_parameter<std::string>(
        "camera_topic", "/left_camera/image/compressed");
    roi_size_ = declare_parameter<int>("roi_size", 500);
    display_scale_ = declare_parameter<double>("display_scale", 0.65);
    zoom_size_ = declare_parameter<int>("zoom_size", 380);
    if (roi_size_ < 64 || display_scale_ <= 0.1 || display_scale_ > 1.0 ||
        zoom_size_ < 128) {
      throw std::runtime_error("invalid focus viewer parameters");
    }

    subscription_ = create_subscription<CompressedImage>(
        topic, rclcpp::QoS(1).reliable(),
        [this](CompressedImage::ConstSharedPtr message) {
          std::lock_guard<std::mutex> lock(frame_mutex_);
          latest_frame_ = std::move(message);
        });

    cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
    cv::resizeWindow(window_name_, 1600, 760);
    RCLCPP_INFO(get_logger(), "low-latency focus viewer subscribed to %s",
                topic.c_str());
  }

  void run() {
    while (rclcpp::ok()) {
      auto frame = takeLatestFrame();
      if (frame) {
        render(*frame);
      }

      const int key = cv::waitKey(frame ? 1 : 5) & 0xff;
      if (key == 27 || key == 'q') {
        break;
      }
      if (key == 'r') {
        best_focus_score_ = 0.0;
        RCLCPP_INFO(get_logger(), "focus score peak reset");
      }
    }
    cv::destroyWindow(window_name_);
  }

 private:
  CompressedImage::ConstSharedPtr takeLatestFrame() {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (!latest_frame_) {
      return nullptr;
    }
    const uint64_t stamp = stampNanoseconds(latest_frame_->header.stamp);
    if (stamp == last_stamp_ns_) {
      return nullptr;
    }
    last_stamp_ns_ = stamp;
    return latest_frame_;
  }

  void render(const CompressedImage &message) {
    const cv::Mat encoded(1, static_cast<int>(message.data.size()), CV_8UC1,
                          const_cast<uint8_t *>(message.data.data()));
    cv::Mat image = cv::imdecode(encoded, cv::IMREAD_COLOR);
    if (image.empty()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                            "failed to decode left camera JPEG");
      return;
    }

    const int roi_width = std::min(roi_size_, image.cols);
    const int roi_height = std::min(roi_size_, image.rows);
    const cv::Rect roi_rect((image.cols - roi_width) / 2,
                            (image.rows - roi_height) / 2,
                            roi_width, roi_height);
    const cv::Mat roi = image(roi_rect);
    const double raw_focus_score = calculateFocusScore(roi);
    if (!focus_score_initialized_) {
      filtered_focus_score_ = raw_focus_score;
      focus_score_initialized_ = true;
    } else {
      filtered_focus_score_ = 0.8 * filtered_focus_score_ + 0.2 * raw_focus_score;
    }
    const double focus_score = filtered_focus_score_;
    best_focus_score_ = std::max(best_focus_score_, focus_score);

    cv::Mat full_display;
    cv::resize(image, full_display, cv::Size(), display_scale_, display_scale_,
               cv::INTER_AREA);
    const cv::Rect display_roi(
        static_cast<int>(std::lround(roi_rect.x * display_scale_)),
        static_cast<int>(std::lround(roi_rect.y * display_scale_)),
        static_cast<int>(std::lround(roi_rect.width * display_scale_)),
        static_cast<int>(std::lround(roi_rect.height * display_scale_)));
    cv::rectangle(full_display, display_roi, cv::Scalar(0, 255, 0), 2);

    const int panel_width = zoom_size_ + 40;
    cv::Mat panel(full_display.rows, panel_width, CV_8UC3, cv::Scalar(24, 24, 24));
    cv::Mat zoom;
    cv::resize(roi, zoom, cv::Size(zoom_size_, zoom_size_), 0.0, 0.0,
               cv::INTER_NEAREST);
    const int zoom_y = 20;
    zoom.copyTo(panel(cv::Rect(20, zoom_y, zoom.cols, zoom.rows)));
    cv::rectangle(panel, cv::Rect(19, zoom_y - 1, zoom.cols + 2, zoom.rows + 2),
                  cv::Scalar(80, 80, 80), 1);

    const bool near_peak = best_focus_score_ > 0.0 &&
                           focus_score >= best_focus_score_ * 0.95;
    const cv::Scalar score_color = near_peak ? cv::Scalar(70, 220, 70)
                                             : cv::Scalar(0, 190, 255);
    drawMetric(panel, "FOCUS", focus_score, zoom_y + zoom_size_ + 48,
               score_color);
    drawMetric(panel, "PEAK", best_focus_score_, zoom_y + zoom_size_ + 92,
               cv::Scalar(230, 230, 230));
    drawMetric(panel, "FPS", measured_fps_, zoom_y + zoom_size_ + 148,
               cv::Scalar(180, 180, 180));
    drawMetric(panel, "LATENCY ms", measured_latency_ms_,
               zoom_y + zoom_size_ + 192, cv::Scalar(180, 180, 180));

    cv::Mat display;
    cv::hconcat(full_display, panel, display);
    cv::imshow(window_name_, display);

    const double latency_ms = std::max(
        0.0, (static_cast<double>(now().nanoseconds()) -
              stampNanoseconds(message.header.stamp)) /
                 1e6);
    updateStatistics(latency_ms, focus_score);
  }

  static double calculateFocusScore(const cv::Mat &roi) {
    cv::Mat gray;
    cv::Mat smoothed;
    cv::Mat laplacian;
    cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, smoothed, cv::Size(3, 3), 0.0);
    cv::Laplacian(smoothed, laplacian, CV_64F);
    cv::Scalar mean;
    cv::Scalar deviation;
    cv::meanStdDev(laplacian, mean, deviation);
    return deviation[0] * deviation[0];
  }

  static void drawMetric(cv::Mat &panel, const std::string &label, double value,
                         int y, const cv::Scalar &color) {
    std::ostringstream text;
    text << label << "  " << std::fixed << std::setprecision(1) << value;
    cv::putText(panel, text.str(), cv::Point(22, y), cv::FONT_HERSHEY_SIMPLEX,
                0.72, color, 2, cv::LINE_AA);
  }

  void updateStatistics(double latency_ms, double focus_score) {
    ++statistics_frames_;
    statistics_latency_sum_ms_ += latency_ms;
    statistics_latency_max_ms_ = std::max(statistics_latency_max_ms_, latency_ms);
    const auto current = std::chrono::steady_clock::now();
    const double elapsed =
        std::chrono::duration<double>(current - statistics_start_).count();
    if (elapsed < 3.0) {
      return;
    }
    measured_fps_ = statistics_frames_ / elapsed;
    measured_latency_ms_ = statistics_latency_sum_ms_ / statistics_frames_;
    RCLCPP_INFO(get_logger(),
                "left focus preview: %.2f fps, latency avg %.1f ms, max %.1f ms, focus %.1f",
                measured_fps_, measured_latency_ms_, statistics_latency_max_ms_,
                focus_score);
    statistics_start_ = current;
    statistics_frames_ = 0;
    statistics_latency_sum_ms_ = 0.0;
    statistics_latency_max_ms_ = 0.0;
  }

  const std::string window_name_ = "Left Camera Focus";
  int roi_size_ = 500;
  int zoom_size_ = 380;
  double display_scale_ = 0.65;
  rclcpp::Subscription<CompressedImage>::SharedPtr subscription_;
  std::mutex frame_mutex_;
  CompressedImage::ConstSharedPtr latest_frame_;
  uint64_t last_stamp_ns_ = 0;
  double best_focus_score_ = 0.0;
  double filtered_focus_score_ = 0.0;
  bool focus_score_initialized_ = false;
  double measured_fps_ = 0.0;
  double measured_latency_ms_ = 0.0;
  std::chrono::steady_clock::time_point statistics_start_ =
      std::chrono::steady_clock::now();
  size_t statistics_frames_ = 0;
  double statistics_latency_sum_ms_ = 0.0;
  double statistics_latency_max_ms_ = 0.0;
};

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OpenCvFocusViewer>();
  std::thread spin_thread([node]() { rclcpp::spin(node); });
  node->run();
  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
