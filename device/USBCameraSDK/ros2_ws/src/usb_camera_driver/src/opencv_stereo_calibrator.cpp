#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <unistd.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <future>
#include <iomanip>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

using CompressedImage = sensor_msgs::msg::CompressedImage;
using ImagePoints = std::vector<std::vector<cv::Point2f>>;

std::string defaultCalibrationFile() {
  if (const char *configured = std::getenv("LUXI_USB_CALIBRATION_FILE");
      configured != nullptr && configured[0] != '\0') {
    return configured;
  }
  return (std::filesystem::path(ament_index_cpp::get_package_share_directory(
              "usb_camera_driver")) /
          "config" / "stereo_opencv.yaml")
      .string();
}

uint64_t stampNanoseconds(const builtin_interfaces::msg::Time &stamp) {
  return static_cast<uint64_t>(stamp.sec) * 1000000000ULL + stamp.nanosec;
}

struct FramePair {
  CompressedImage::ConstSharedPtr left;
  CompressedImage::ConstSharedPtr right;
  uint64_t stamp_ns = 0;
};

struct Detection {
  bool found = false;
  std::vector<cv::Point2f> full_corners;
  std::vector<cv::Point2f> display_corners;
};

struct DetectionPair {
  Detection left;
  Detection right;
  uint64_t stamp_ns = 0;
};

struct CalibrationResult {
  cv::Mat camera_matrix_left;
  cv::Mat distortion_left;
  cv::Mat camera_matrix_right;
  cv::Mat distortion_right;
  cv::Mat rotation;
  cv::Mat translation;
  cv::Mat essential;
  cv::Mat fundamental;
  cv::Mat rectification_left;
  cv::Mat rectification_right;
  cv::Mat projection_left;
  cv::Mat projection_right;
  cv::Mat disparity_to_depth;
  double rms_left = 0.0;
  double rms_right = 0.0;
  double rms_stereo = 0.0;
};

class OpenCvStereoCalibrator : public rclcpp::Node {
 public:
  OpenCvStereoCalibrator() : Node("opencv_stereo_calibrator") {
    board_cols_ = declare_parameter<int>("board_cols", 11);
    board_rows_ = declare_parameter<int>("board_rows", 8);
    square_size_ = declare_parameter<double>("square_size", 0.010);
    minimum_samples_ = declare_parameter<int>("minimum_samples", 20);
    detection_scale_ = declare_parameter<double>("detection_scale", 0.5);
    detection_rate_hz_ = declare_parameter<double>("detection_rate_hz", 5.0);
    output_file_ = declare_parameter<std::string>(
        "output_file",
        defaultCalibrationFile());
    input_file_ = declare_parameter<std::string>("input_file", "");
    self_test_ = declare_parameter<bool>("self_test", false);
    preview_self_test_ = declare_parameter<bool>("preview_self_test", false);

    if (board_cols_ < 2 || board_rows_ < 2 || square_size_ <= 0.0 ||
        minimum_samples_ < 3 || detection_scale_ <= 0.0 || detection_scale_ > 1.0 ||
        detection_rate_hz_ <= 0.0) {
      throw std::runtime_error("invalid OpenCV stereo calibration parameters");
    }
    board_size_ = cv::Size(board_cols_, board_rows_);

    if (self_test_) {
      RCLCPP_INFO(get_logger(), "running synthetic OpenCV calibration self-test");
      return;
    }

    if (!input_file_.empty()) {
      loadCalibration(input_file_);
      validation_mode_ = true;
    }

    const auto qos = rclcpp::QoS(1).reliable();
    left_sub_ = create_subscription<CompressedImage>(
        "/left_camera/image/compressed", qos,
        [this](CompressedImage::ConstSharedPtr message) {
          std::lock_guard<std::mutex> lock(pair_mutex_);
          pending_left_ = std::move(message);
          pairIfReady();
        });
    right_sub_ = create_subscription<CompressedImage>(
        "/right_camera/image/compressed", qos,
        [this](CompressedImage::ConstSharedPtr message) {
          std::lock_guard<std::mutex> lock(pair_mutex_);
          pending_right_ = std::move(message);
          pairIfReady();
        });

    cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
    cv::resizeWindow(window_name_, 1600, 520);
    RCLCPP_INFO(get_logger(),
                "OpenCV stereo calibration: board=%dx%d square=%.6f m output=%s",
                board_cols_, board_rows_, square_size_, output_file_.c_str());
  }

  void run() {
    if (self_test_) {
      runSelfTest();
      return;
    }
    while (rclcpp::ok()) {
      pollCalibration();
      pollDetection();
      auto pair = takeLatestPair();
      if (pair.has_value()) {
        processPair(*pair);
      }

      const int key = cv::waitKey(pair.has_value() ? 1 : 5) & 0xff;
      if (key == 27 || key == 'q') {
        break;
      }
      if (!validation_mode_ && key == ' ' && left_found_ && right_found_ &&
          !calibration_running_) {
        const double detection_age_ms =
            (static_cast<double>(now().nanoseconds()) - last_detection_stamp_ns_) / 1e6;
        if (detection_age_ms <= 500.0) {
          left_samples_.push_back(left_corners_);
          right_samples_.push_back(right_corners_);
          RCLCPP_INFO(get_logger(), "captured calibration pair %zu", left_samples_.size());
        } else {
          RCLCPP_WARN(get_logger(), "corner detection is stale; hold the board still and retry");
        }
      } else if (!validation_mode_ && key == 'c' && !calibration_running_) {
        startCalibration();
      } else if (!validation_mode_ && key == 'r' && !calibration_running_) {
        left_samples_.clear();
        right_samples_.clear();
        calibration_.reset();
        RCLCPP_INFO(get_logger(), "cleared all calibration samples");
      }
    }
    cv::destroyWindow(window_name_);
  }

 private:
  void pairIfReady() {
    if (!pending_left_ || !pending_right_) {
      return;
    }
    const uint64_t left_stamp = stampNanoseconds(pending_left_->header.stamp);
    const uint64_t right_stamp = stampNanoseconds(pending_right_->header.stamp);
    if (left_stamp == right_stamp) {
      latest_pair_ = FramePair{pending_left_, pending_right_, left_stamp};
      pending_left_.reset();
      pending_right_.reset();
    } else if (left_stamp < right_stamp) {
      pending_left_.reset();
    } else {
      pending_right_.reset();
    }
  }

  std::optional<FramePair> takeLatestPair() {
    std::lock_guard<std::mutex> lock(pair_mutex_);
    if (!latest_pair_.has_value() || latest_pair_->stamp_ns == last_pair_stamp_ns_) {
      return std::nullopt;
    }
    last_pair_stamp_ns_ = latest_pair_->stamp_ns;
    auto pair = latest_pair_;
    latest_pair_.reset();
    return pair;
  }

  static cv::Mat decodeGray(const CompressedImage &message) {
    const cv::Mat encoded(1, static_cast<int>(message.data.size()), CV_8UC1,
                          const_cast<uint8_t *>(message.data.data()));
    cv::Mat gray = cv::imdecode(encoded, cv::IMREAD_GRAYSCALE);
    if (gray.empty()) {
      throw std::runtime_error("failed to decode camera JPEG");
    }
    return gray;
  }

  Detection detectCorners(const cv::Mat &gray) const {
    Detection detection;
    cv::Mat reduced;
    cv::resize(gray, reduced, cv::Size(), detection_scale_, detection_scale_,
               cv::INTER_AREA);
    detection.found = cv::findChessboardCorners(
        reduced, board_size_, detection.display_corners,
        cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE |
            cv::CALIB_CB_FAST_CHECK);
    if (!detection.found) {
      return detection;
    }

    cv::cornerSubPix(
        reduced, detection.display_corners, cv::Size(5, 5), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.01));
    detection.full_corners = detection.display_corners;
    const float inverse_scale = static_cast<float>(1.0 / detection_scale_);
    for (auto &corner : detection.full_corners) {
      corner *= inverse_scale;
    }
    cv::cornerSubPix(
        gray, detection.full_corners, cv::Size(5, 5), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.01));
    return detection;
  }

  void processPair(const FramePair &pair) {
    try {
      cv::Mat left_gray = decodeGray(*pair.left);
      cv::Mat right_gray = decodeGray(*pair.right);
      if (left_gray.size() != right_gray.size()) {
        throw std::runtime_error("left and right image sizes differ");
      }
      if (validation_mode_ && left_gray.size() != calibration_image_size_) {
        throw std::runtime_error("camera image size does not match calibration file");
      }
      image_size_ = left_gray.size();

      const double running_seconds = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - node_start_).count();
      if (preview_self_test_ && !preview_self_test_started_ &&
          running_seconds >= 6.0) {
        auto samples = makeSyntheticSamples();
        left_samples_ = std::move(samples.first);
        right_samples_ = std::move(samples.second);
        preview_self_test_started_ = true;
        startCalibration();
      }

      cv::Mat left_display;
      cv::Mat right_display;
      if (calibration_.has_value()) {
        cv::remap(left_gray, left_display, map_left_x_, map_left_y_, cv::INTER_LINEAR);
        cv::remap(right_gray, right_display, map_right_x_, map_right_y_, cv::INTER_LINEAR);
        if (validation_mode_) {
          startDetection(left_display, right_display, pair.stamp_ns);
        }
        cv::resize(left_display, left_display, cv::Size(), detection_scale_, detection_scale_);
        cv::resize(right_display, right_display, cv::Size(), detection_scale_, detection_scale_);
      } else {
        startDetection(left_gray, right_gray, pair.stamp_ns);
        cv::resize(left_gray, left_display, cv::Size(), detection_scale_, detection_scale_);
        cv::resize(right_gray, right_display, cv::Size(), detection_scale_, detection_scale_);
      }
      cv::cvtColor(left_display, left_display, cv::COLOR_GRAY2BGR);
      cv::cvtColor(right_display, right_display, cv::COLOR_GRAY2BGR);

      if (!calibration_.has_value() || validation_mode_) {
        if (left_found_) {
          cv::drawChessboardCorners(left_display, board_size_,
                                    left_display_corners_, true);
        }
        if (right_found_) {
          cv::drawChessboardCorners(right_display, board_size_,
                                    right_display_corners_, true);
        }
      }

      cv::Mat display;
      cv::hconcat(left_display, right_display, display);
      if (calibration_.has_value()) {
        for (int y = 40; y < display.rows; y += 40) {
          cv::line(display, cv::Point(0, y), cv::Point(display.cols - 1, y),
                   cv::Scalar(0, 255, 0), 1);
        }
      }

      const double latency_ms = std::max(
          0.0, (static_cast<double>(now().nanoseconds()) - pair.stamp_ns) / 1e6);
      updateStatistics(latency_ms);
      drawStatus(display);
      cv::imshow(window_name_, display);
    } catch (const std::exception &error) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "%s", error.what());
    }
  }

  void updateStatistics(double latency_ms) {
    ++statistics_frames_;
    statistics_latency_sum_ms_ += latency_ms;
    statistics_latency_max_ms_ = std::max(statistics_latency_max_ms_, latency_ms);
    const auto current = std::chrono::steady_clock::now();
    const double seconds =
        std::chrono::duration<double>(current - statistics_start_).count();
    if (seconds < 3.0) {
      return;
    }
    measured_fps_ = statistics_frames_ / seconds;
    measured_latency_ms_ = statistics_latency_sum_ms_ / statistics_frames_;
    RCLCPP_INFO(get_logger(), "OpenCV calibration: %.2f fps, latency avg %.1f ms, max %.1f ms",
                measured_fps_, measured_latency_ms_, statistics_latency_max_ms_);
    statistics_start_ = current;
    statistics_frames_ = 0;
    statistics_latency_sum_ms_ = 0.0;
    statistics_latency_max_ms_ = 0.0;
  }

  void startDetection(const cv::Mat &left_gray, const cv::Mat &right_gray,
                      uint64_t stamp_ns) {
    if ((calibration_.has_value() && !validation_mode_) || calibration_running_ ||
        detection_running_) {
      return;
    }
    const auto current = std::chrono::steady_clock::now();
    const double seconds =
        std::chrono::duration<double>(current - last_detection_start_).count();
    if (seconds < 1.0 / detection_rate_hz_) {
      return;
    }
    detection_running_ = true;
    last_detection_start_ = current;
    detection_future_ = std::async(
        std::launch::async,
        [this, left_gray, right_gray, stamp_ns]() {
          auto left_future = std::async(std::launch::async, [this, left_gray]() {
            return detectCorners(left_gray);
          });
          auto right_future = std::async(std::launch::async, [this, right_gray]() {
            return detectCorners(right_gray);
          });
          return DetectionPair{left_future.get(), right_future.get(), stamp_ns};
        });
  }

  void pollDetection() {
    if (!detection_running_ ||
        detection_future_.wait_for(std::chrono::milliseconds(0)) !=
            std::future_status::ready) {
      return;
    }
    try {
      const DetectionPair detection = detection_future_.get();
      left_found_ = detection.left.found;
      right_found_ = detection.right.found;
      left_corners_ = detection.left.full_corners;
      right_corners_ = detection.right.full_corners;
      left_display_corners_ = detection.left.display_corners;
      right_display_corners_ = detection.right.display_corners;
      last_detection_stamp_ns_ = detection.stamp_ns;
      if (validation_mode_ && left_found_ && right_found_) {
        updateValidationError(left_corners_, right_corners_);
      }
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "corner detection failed: %s", error.what());
    }
    detection_running_ = false;
  }

  void drawStatus(cv::Mat &display) const {
    std::ostringstream status;
    status << "samples " << left_samples_.size() << "/" << minimum_samples_
           << "  corners " << (left_found_ && right_found_ ? "OK" : "--")
           << "  " << std::fixed << std::setprecision(1) << measured_fps_ << " fps  "
           << measured_latency_ms_ << " ms";
    if (calibration_running_) {
      status << "  CALIBRATING";
    } else if (validation_mode_) {
      status << "  VERIFY dy " << std::setprecision(2) << validation_mean_px_
             << "/" << validation_max_px_ << " px";
    } else if (calibration_.has_value()) {
      status << "  RECTIFIED";
    }
    cv::rectangle(display, cv::Rect(0, 0, display.cols, 34), cv::Scalar(0, 0, 0),
                  cv::FILLED);
    cv::putText(display, status.str(), cv::Point(12, 24), cv::FONT_HERSHEY_SIMPLEX,
                0.65, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
  }

  void startCalibration() {
    if (static_cast<int>(left_samples_.size()) < minimum_samples_) {
      RCLCPP_WARN(get_logger(), "need at least %d samples; currently have %zu",
                  minimum_samples_, left_samples_.size());
      return;
    }
    const ImagePoints left = left_samples_;
    const ImagePoints right = right_samples_;
    const cv::Size image_size = image_size_;
    calibration_running_ = true;
    calibration_future_ = std::async(
        std::launch::async,
        [this, left, right, image_size]() {
          ::nice(10);
          return calibrate(left, right, image_size);
        });
    RCLCPP_INFO(get_logger(), "started OpenCV calibration using %zu pairs",
                left_samples_.size());
  }

  void updateValidationError(const std::vector<cv::Point2f> &left,
                             const std::vector<cv::Point2f> &right) {
    if (left.size() != right.size() || left.empty()) {
      return;
    }
    auto compute = [&left, &right](bool reverse) {
      double sum = 0.0;
      double maximum = 0.0;
      for (size_t index = 0; index < left.size(); ++index) {
        const size_t right_index = reverse ? right.size() - 1 - index : index;
        const double error = std::abs(left[index].y - right[right_index].y);
        sum += error;
        maximum = std::max(maximum, error);
      }
      return std::pair<double, double>{sum / left.size(), maximum};
    };
    const auto direct = compute(false);
    const auto reversed = compute(true);
    const auto selected = direct.first <= reversed.first ? direct : reversed;
    validation_mean_px_ = selected.first;
    validation_max_px_ = selected.second;
    ++validation_measurements_;
    if (validation_measurements_ % 5 == 0) {
      const bool passed = validation_mean_px_ < 0.5 && validation_max_px_ < 1.0;
      RCLCPP_INFO(get_logger(),
                  "rectification check: mean vertical error %.3f px, max %.3f px [%s]",
                  validation_mean_px_, validation_max_px_, passed ? "PASS" : "FAIL");
    }
  }

  void initializeRectificationMaps(const CalibrationResult &result,
                                   const cv::Size &image_size) {
    cv::initUndistortRectifyMap(
        result.camera_matrix_left, result.distortion_left,
        result.rectification_left, result.projection_left, image_size,
        CV_16SC2, map_left_x_, map_left_y_);
    cv::initUndistortRectifyMap(
        result.camera_matrix_right, result.distortion_right,
        result.rectification_right, result.projection_right, image_size,
        CV_16SC2, map_right_x_, map_right_y_);
  }

  void loadCalibration(const std::string &path) {
    cv::FileStorage file(path, cv::FileStorage::READ);
    if (!file.isOpened()) {
      throw std::runtime_error("cannot open calibration file: " + path);
    }
    CalibrationResult result;
    int width = 0;
    int height = 0;
    file["image_width"] >> width;
    file["image_height"] >> height;
    file["rms_left"] >> result.rms_left;
    file["rms_right"] >> result.rms_right;
    file["rms_stereo"] >> result.rms_stereo;
    file["camera_matrix_left"] >> result.camera_matrix_left;
    file["distortion_left"] >> result.distortion_left;
    file["camera_matrix_right"] >> result.camera_matrix_right;
    file["distortion_right"] >> result.distortion_right;
    file["rotation"] >> result.rotation;
    file["translation"] >> result.translation;
    file["essential"] >> result.essential;
    file["fundamental"] >> result.fundamental;
    file["rectification_left"] >> result.rectification_left;
    file["rectification_right"] >> result.rectification_right;
    file["projection_left"] >> result.projection_left;
    file["projection_right"] >> result.projection_right;
    file["disparity_to_depth"] >> result.disparity_to_depth;

    const bool matrices_valid =
        result.camera_matrix_left.size() == cv::Size(3, 3) &&
        result.camera_matrix_right.size() == cv::Size(3, 3) &&
        result.rotation.size() == cv::Size(3, 3) &&
        result.translation.total() == 3 &&
        result.rectification_left.size() == cv::Size(3, 3) &&
        result.rectification_right.size() == cv::Size(3, 3) &&
        result.projection_left.size() == cv::Size(4, 3) &&
        result.projection_right.size() == cv::Size(4, 3) &&
        result.disparity_to_depth.size() == cv::Size(4, 4);
    if (width <= 0 || height <= 0 || !matrices_valid) {
      throw std::runtime_error("calibration file is incomplete or malformed");
    }

    calibration_image_size_ = cv::Size(width, height);
    calibration_ = std::move(result);
    initializeRectificationMaps(*calibration_, calibration_image_size_);
    const double baseline = cv::norm(calibration_->translation);
    const bool rms_passed = calibration_->rms_left < 0.5 &&
                            calibration_->rms_right < 0.5 &&
                            calibration_->rms_stereo < 0.5;
    RCLCPP_INFO(get_logger(),
                "loaded calibration: %s, baseline %.3f mm, RMS L/R/stereo %.3f/%.3f/%.3f px",
                path.c_str(), baseline * 1000.0, calibration_->rms_left,
                calibration_->rms_right, calibration_->rms_stereo);
    if (!rms_passed) {
      RCLCPP_WARN(get_logger(), "static calibration quality check: FAIL (target < 0.5 px)");
    } else {
      RCLCPP_INFO(get_logger(), "static calibration quality check: PASS");
    }
  }

  std::pair<ImagePoints, ImagePoints> makeSyntheticSamples() const {
    const cv::Mat camera_matrix =
        (cv::Mat_<double>(3, 3) << 1100.0, 0.0, 960.0, 0.0, 1100.0,
         540.0, 0.0, 0.0, 1.0);
    const cv::Mat distortion = cv::Mat::zeros(5, 1, CV_64F);
    const cv::Mat right_offset =
        (cv::Mat_<double>(3, 1) << -0.12, 0.0, 0.0);

    std::vector<cv::Point3f> board_points;
    for (int row = 0; row < board_rows_; ++row) {
      for (int col = 0; col < board_cols_; ++col) {
        board_points.emplace_back(static_cast<float>(col * square_size_),
                                  static_cast<float>(row * square_size_), 0.0F);
      }
    }

    ImagePoints left;
    ImagePoints right;
    for (int index = 0; index < 24; ++index) {
      const double phase = index * 0.47;
      const cv::Mat rotation =
          (cv::Mat_<double>(3, 1) << 0.20 * std::sin(phase),
           0.24 * std::cos(phase * 0.73), 0.12 * std::sin(phase * 0.41));
      cv::Mat translation =
          (cv::Mat_<double>(3, 1) << 0.13 * std::sin(phase * 0.61) - 0.05,
           0.09 * std::cos(phase * 0.83) - 0.04,
           0.65 + 0.025 * index);
      std::vector<cv::Point2f> left_points;
      std::vector<cv::Point2f> right_points;
      cv::projectPoints(board_points, rotation, translation, camera_matrix,
                        distortion, left_points);
      translation += right_offset;
      cv::projectPoints(board_points, rotation, translation, camera_matrix,
                        distortion, right_points);
      left.push_back(std::move(left_points));
      right.push_back(std::move(right_points));
    }

    return {std::move(left), std::move(right)};
  }

  void runSelfTest() {
    const cv::Size test_size(1920, 1080);
    auto samples = makeSyntheticSamples();

    const CalibrationResult result = calibrate(samples.first, samples.second, test_size);
    const double baseline = cv::norm(result.translation);
    if (!std::isfinite(result.rms_stereo) || result.rms_stereo > 0.1 ||
        std::abs(baseline - 0.12) > 0.005 ||
        !std::filesystem::exists(output_file_)) {
      throw std::runtime_error("synthetic calibration self-test failed");
    }
    RCLCPP_INFO(get_logger(),
                "self-test passed: stereo RMS %.6f px, baseline %.6f m, file=%s",
                result.rms_stereo, baseline, output_file_.c_str());
  }

  CalibrationResult calibrate(const ImagePoints &left, const ImagePoints &right,
                              const cv::Size &image_size) const {
    std::vector<cv::Point3f> board_points;
    board_points.reserve(static_cast<size_t>(board_cols_ * board_rows_));
    for (int row = 0; row < board_rows_; ++row) {
      for (int col = 0; col < board_cols_; ++col) {
        board_points.emplace_back(static_cast<float>(col * square_size_),
                                  static_cast<float>(row * square_size_), 0.0F);
      }
    }
    const std::vector<std::vector<cv::Point3f>> object_points(left.size(), board_points);

    CalibrationResult result;
    result.camera_matrix_left = cv::Mat::eye(3, 3, CV_64F);
    result.camera_matrix_right = cv::Mat::eye(3, 3, CV_64F);
    const auto criteria = cv::TermCriteria(
        cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 100, 1e-7);
    std::vector<cv::Mat> rotations;
    std::vector<cv::Mat> translations;
    result.rms_left = cv::calibrateCamera(
        object_points, left, image_size, result.camera_matrix_left,
        result.distortion_left, rotations, translations, 0, criteria);
    rotations.clear();
    translations.clear();
    result.rms_right = cv::calibrateCamera(
        object_points, right, image_size, result.camera_matrix_right,
        result.distortion_right, rotations, translations, 0, criteria);
    result.rms_stereo = cv::stereoCalibrate(
        object_points, left, right, result.camera_matrix_left,
        result.distortion_left, result.camera_matrix_right,
        result.distortion_right, image_size, result.rotation, result.translation,
        result.essential, result.fundamental, cv::CALIB_FIX_INTRINSIC, criteria);
    cv::stereoRectify(
        result.camera_matrix_left, result.distortion_left,
        result.camera_matrix_right, result.distortion_right, image_size,
        result.rotation, result.translation, result.rectification_left,
        result.rectification_right, result.projection_left,
        result.projection_right, result.disparity_to_depth,
        cv::CALIB_ZERO_DISPARITY, 0.0);
    saveCalibration(result, image_size, left.size());
    return result;
  }

  void saveCalibration(const CalibrationResult &result, const cv::Size &image_size,
                       size_t sample_count) const {
    const std::filesystem::path path(output_file_);
    if (!path.parent_path().empty()) {
      std::filesystem::create_directories(path.parent_path());
    }
    cv::FileStorage file(output_file_, cv::FileStorage::WRITE);
    if (!file.isOpened()) {
      throw std::runtime_error("cannot write calibration file: " + output_file_);
    }
    file << "image_width" << image_size.width;
    file << "image_height" << image_size.height;
    file << "board_cols" << board_cols_;
    file << "board_rows" << board_rows_;
    file << "square_size" << square_size_;
    file << "sample_count" << static_cast<int>(sample_count);
    file << "rms_left" << result.rms_left;
    file << "rms_right" << result.rms_right;
    file << "rms_stereo" << result.rms_stereo;
    file << "camera_matrix_left" << result.camera_matrix_left;
    file << "distortion_left" << result.distortion_left;
    file << "camera_matrix_right" << result.camera_matrix_right;
    file << "distortion_right" << result.distortion_right;
    file << "rotation" << result.rotation;
    file << "translation" << result.translation;
    file << "essential" << result.essential;
    file << "fundamental" << result.fundamental;
    file << "rectification_left" << result.rectification_left;
    file << "rectification_right" << result.rectification_right;
    file << "projection_left" << result.projection_left;
    file << "projection_right" << result.projection_right;
    file << "disparity_to_depth" << result.disparity_to_depth;
  }

  void pollCalibration() {
    if (!calibration_running_ ||
        calibration_future_.wait_for(std::chrono::milliseconds(0)) !=
            std::future_status::ready) {
      return;
    }
    try {
      calibration_ = calibration_future_.get();
      initializeRectificationMaps(*calibration_, image_size_);
      RCLCPP_INFO(get_logger(),
                  "calibration saved: %s (RMS left %.4f, right %.4f, stereo %.4f)",
                  output_file_.c_str(), calibration_->rms_left,
                  calibration_->rms_right, calibration_->rms_stereo);
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "calibration failed: %s", error.what());
    }
    calibration_running_ = false;
  }

  const std::string window_name_ = "OpenCV Stereo Calibration";
  int board_cols_ = 11;
  int board_rows_ = 8;
  int minimum_samples_ = 20;
  double square_size_ = 0.010;
  double detection_scale_ = 0.5;
  double detection_rate_hz_ = 5.0;
  std::string output_file_;
  std::string input_file_;
  bool self_test_ = false;
  bool preview_self_test_ = false;
  bool preview_self_test_started_ = false;
  bool validation_mode_ = false;
  cv::Size board_size_;
  cv::Size image_size_;
  cv::Size calibration_image_size_;

  rclcpp::Subscription<CompressedImage>::SharedPtr left_sub_;
  rclcpp::Subscription<CompressedImage>::SharedPtr right_sub_;
  std::mutex pair_mutex_;
  CompressedImage::ConstSharedPtr pending_left_;
  CompressedImage::ConstSharedPtr pending_right_;
  std::optional<FramePair> latest_pair_;
  uint64_t last_pair_stamp_ns_ = 0;

  bool left_found_ = false;
  bool right_found_ = false;
  std::vector<cv::Point2f> left_corners_;
  std::vector<cv::Point2f> right_corners_;
  std::vector<cv::Point2f> left_display_corners_;
  std::vector<cv::Point2f> right_display_corners_;
  ImagePoints left_samples_;
  ImagePoints right_samples_;

  bool detection_running_ = false;
  std::future<DetectionPair> detection_future_;
  std::chrono::steady_clock::time_point last_detection_start_ =
      std::chrono::steady_clock::now() - std::chrono::seconds(1);
  uint64_t last_detection_stamp_ns_ = 0;
  size_t validation_measurements_ = 0;
  double validation_mean_px_ = 0.0;
  double validation_max_px_ = 0.0;

  bool calibration_running_ = false;
  std::future<CalibrationResult> calibration_future_;
  std::optional<CalibrationResult> calibration_;
  cv::Mat map_left_x_;
  cv::Mat map_left_y_;
  cv::Mat map_right_x_;
  cv::Mat map_right_y_;

  std::chrono::steady_clock::time_point statistics_start_ =
      std::chrono::steady_clock::now();
  size_t statistics_frames_ = 0;
  double statistics_latency_sum_ms_ = 0.0;
  double statistics_latency_max_ms_ = 0.0;
  double measured_fps_ = 0.0;
  double measured_latency_ms_ = 0.0;
  std::chrono::steady_clock::time_point node_start_ =
      std::chrono::steady_clock::now();
};

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OpenCvStereoCalibrator>();
  std::thread spin_thread([node]() { rclcpp::spin(node); });
  node->run();
  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}
