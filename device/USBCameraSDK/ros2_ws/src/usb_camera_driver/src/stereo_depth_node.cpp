#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/exact_time.hpp>
#include <message_filters/synchronizer.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaarithm.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudastereo.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rtabmap_msgs/msg/rgbd_image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <vpi/OpenCVInterop.hpp>
#include <vpi/Status.h>
#include <vpi/Stream.h>
#include <vpi/algo/ConvertImageFormat.h>
#include <vpi/algo/StereoDisparity.h>

namespace {

using Image = sensor_msgs::msg::Image;
using StereoPolicy = message_filters::sync_policies::ExactTime<Image, Image>;
using CompressedImage = sensor_msgs::msg::CompressedImage;
using CompressedStereoPolicy =
    message_filters::sync_policies::ExactTime<CompressedImage, CompressedImage>;

cv::Mat requireMatrix(const cv::FileStorage &file, const std::string &name,
                      int rows, int columns) {
  cv::Mat matrix;
  file[name] >> matrix;
  if (matrix.rows != rows || matrix.cols != columns || matrix.empty()) {
    throw std::runtime_error("calibration matrix '" + name + "' must be " +
                             std::to_string(rows) + "x" +
                             std::to_string(columns));
  }
  matrix.convertTo(matrix, CV_64F);
  return matrix;
}

cv::Mat scaleIntrinsic(const cv::Mat &matrix, double scale_x,
                       double scale_y) {
  cv::Mat scaled = matrix.clone();
  scaled.at<double>(0, 0) *= scale_x;
  scaled.at<double>(0, 2) *= scale_x;
  scaled.at<double>(1, 1) *= scale_y;
  scaled.at<double>(1, 2) *= scale_y;
  return scaled;
}

cv::Mat scaleProjection(const cv::Mat &projection, double scale_x,
                        double scale_y) {
  cv::Mat scaled = projection.clone();
  for (int column = 0; column < 4; ++column) {
    scaled.at<double>(0, column) *= scale_x;
    scaled.at<double>(1, column) *= scale_y;
  }
  return scaled;
}

sensor_msgs::msg::Image imageMessage(const std_msgs::msg::Header &header,
                                     const std::string &encoding,
                                     const cv::Mat &image) {
  sensor_msgs::msg::Image message;
  message.header = header;
  message.height = static_cast<uint32_t>(image.rows);
  message.width = static_cast<uint32_t>(image.cols);
  message.encoding = encoding;
  message.is_bigendian = false;
  message.step = static_cast<uint32_t>(image.cols * image.elemSize());
  message.data.resize(static_cast<size_t>(message.step) * message.height);
  for (int row = 0; row < image.rows; ++row) {
    std::memcpy(message.data.data() + static_cast<size_t>(row) * message.step,
                image.ptr(row), message.step);
  }
  return message;
}

sensor_msgs::msg::CameraInfo rectifiedCameraInfo(
    const std_msgs::msg::Header &header, const cv::Size &size,
    const cv::Mat &projection) {
  sensor_msgs::msg::CameraInfo info;
  info.header = header;
  info.height = static_cast<uint32_t>(size.height);
  info.width = static_cast<uint32_t>(size.width);
  info.distortion_model = "plumb_bob";
  info.d.assign(5, 0.0);
  info.k = {projection.at<double>(0, 0), 0.0,
            projection.at<double>(0, 2), 0.0,
            projection.at<double>(1, 1), projection.at<double>(1, 2),
            0.0, 0.0, 1.0};
  info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 4; ++column) {
      info.p[static_cast<size_t>(row * 4 + column)] =
          projection.at<double>(row, column);
    }
  }
  return info;
}

sensor_msgs::msg::PointCloud2 pointCloudMessage(
    const std_msgs::msg::Header &header, const cv::Mat &color,
    const cv::Mat &depth_mm, const cv::Mat &projection,
    const float maximum_depth_m) {
  sensor_msgs::msg::PointCloud2 cloud;
  cloud.header = header;
  cloud.height = 1;
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = 16;
  cloud.fields.resize(4);
  cloud.fields[0].name = "x";
  cloud.fields[0].offset = 0;
  cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[0].count = 1;
  cloud.fields[1].name = "y";
  cloud.fields[1].offset = 4;
  cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[1].count = 1;
  cloud.fields[2].name = "z";
  cloud.fields[2].offset = 8;
  cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[2].count = 1;
  cloud.fields[3].name = "rgb";
  cloud.fields[3].offset = 12;
  cloud.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
  cloud.fields[3].count = 1;

  const float fx = static_cast<float>(projection.at<double>(0, 0));
  const float fy = static_cast<float>(projection.at<double>(1, 1));
  const float cx = static_cast<float>(projection.at<double>(0, 2));
  const float cy = static_cast<float>(projection.at<double>(1, 2));
  cloud.data.resize(depth_mm.total() * cloud.point_step);
  size_t point_count = 0;
  for (int row = 0; row < depth_mm.rows; ++row) {
    const auto *depth_row = depth_mm.ptr<uint16_t>(row);
    const auto *color_row = color.ptr<cv::Vec3b>(row);
    for (int column = 0; column < depth_mm.cols; ++column) {
      if (depth_row[column] == 0) {
        continue;
      }
      const float z = depth_row[column] * 0.001F;
      if (maximum_depth_m > 0.0F && z > maximum_depth_m) {
        continue;
      }
      const float x = (static_cast<float>(column) - cx) * z / fx;
      const float y = (static_cast<float>(row) - cy) * z / fy;
      const cv::Vec3b &bgr = color_row[column];
      const uint32_t rgb = (static_cast<uint32_t>(bgr[2]) << 16U) |
                           (static_cast<uint32_t>(bgr[1]) << 8U) |
                           static_cast<uint32_t>(bgr[0]);
      uint8_t *point = cloud.data.data() + point_count * cloud.point_step;
      std::memcpy(point, &x, sizeof(float));
      std::memcpy(point + 4, &y, sizeof(float));
      std::memcpy(point + 8, &z, sizeof(float));
      std::memcpy(point + 12, &rgb, sizeof(uint32_t));
      ++point_count;
    }
  }
  cloud.width = static_cast<uint32_t>(point_count);
  cloud.row_step = cloud.width * cloud.point_step;
  cloud.data.resize(cloud.row_step);
  return cloud;
}

}  // namespace

class UsbStereoDepthNode final : public rclcpp::Node {
 public:
  UsbStereoDepthNode() : Node("usb_stereo_depth_node") {
    const auto calibration_file =
        declare_parameter<std::string>("calibration_file", "");
    compressed_input_ = declare_parameter<bool>("compressed_input", true);
    const auto left_topic = declare_parameter<std::string>(
        "left_image_topic", "/left_camera/image/compressed");
    const auto right_topic = declare_parameter<std::string>(
        "right_image_topic", "/right_camera/image/compressed");
    const auto color_topic = declare_parameter<std::string>(
        "color_output_topic", "/usb_stereo/left/image_rect_color");
    const auto preview_topic = declare_parameter<std::string>(
        "preview_output_topic", "/usb_stereo/left/image_preview");
    const auto depth_topic = declare_parameter<std::string>(
        "depth_output_topic", "/usb_stereo/depth");
    const auto depth_preview_topic = declare_parameter<std::string>(
        "depth_preview_output_topic", "/usb_stereo/depth_preview");
    const auto camera_info_topic = declare_parameter<std::string>(
        "camera_info_output_topic", "/usb_stereo/left/camera_info");
    const auto rgbd_topic = declare_parameter<std::string>(
        "rgbd_output_topic", "/usb_stereo/rgbd_image");
    const auto point_cloud_topic = declare_parameter<std::string>(
        "point_cloud_output_topic", "/usb_stereo/points");
    const auto disparity_topic = declare_parameter<std::string>(
        "disparity_output_topic", "/usb_stereo/disparity");
    output_frame_id_ = declare_parameter<std::string>(
        "output_frame_id", "left_camera_optical_frame");
    processing_scale_ =
        std::clamp(declare_parameter<double>("processing_scale", 0.5), 0.25,
                   1.0);
    output_scale_ = std::clamp(
        declare_parameter<double>("output_scale", processing_scale_), 0.1,
        processing_scale_);
    const bool legacy_use_cuda_sgm =
        declare_parameter<bool>("use_cuda_sgm", false);
    depth_backend_ = declare_parameter<std::string>("depth_backend", "");
    if (depth_backend_.empty()) {
      depth_backend_ = legacy_use_cuda_sgm ? "opencv_cuda_sgm" : "cpu_sgbm";
    }
    if (depth_backend_ != "opencv_cuda_sgm" &&
        depth_backend_ != "vpi_ofa_pva_vic" &&
        depth_backend_ != "vpi_cuda" && depth_backend_ != "cpu_sgbm") {
      throw std::runtime_error(
          "depth_backend must be opencv_cuda_sgm, vpi_ofa_pva_vic, "
          "vpi_cuda or cpu_sgbm");
    }
    use_cuda_sgm_ = depth_backend_ == "opencv_cuda_sgm";
    vpi_fallback_to_sgbm_ =
        declare_parameter<bool>("vpi_fallback_to_sgbm", true);
    rectification_alpha_ = std::clamp(
        declare_parameter<double>("rectification_alpha", 1.0), 0.0, 1.0);
    min_depth_m_ = declare_parameter<double>("min_depth_m", 0.4);
    max_depth_m_ = declare_parameter<double>("max_depth_m", 6.0);
    far_artifact_depth_m_ = declare_parameter<double>(
        "far_artifact_depth_m", 3.0);
    far_artifact_support_size_ = std::clamp(
        static_cast<int>(declare_parameter<int>(
            "far_artifact_support_size", 3)) | 1,
        3, 15);
    max_processing_fps_ =
        std::max(0.0, declare_parameter<double>("max_processing_fps", 10.0));
    point_cloud_max_fps_ = std::max(
        0.0, declare_parameter<double>("point_cloud_max_fps", 0.0));
    point_cloud_max_depth_m_ = std::max(
        0.0, declare_parameter<double>("point_cloud_max_depth_m", 0.0));
    max_disparity_ = std::clamp(
        static_cast<int>(declare_parameter<int>("max_disparity", 128)), 16,
        256);
    vpi_confidence_threshold_ = std::clamp(
        static_cast<int>(declare_parameter<int>(
            "vpi_confidence_threshold", 0)),
        0, 65535);
    vpi_confidence_type_ =
        declare_parameter<std::string>("vpi_confidence_type", "inference");
    if (vpi_confidence_type_ != "inference" &&
        vpi_confidence_type_ != "absolute" &&
        vpi_confidence_type_ != "relative") {
      throw std::runtime_error(
          "vpi_confidence_type must be inference, absolute or relative");
    }
    vpi_p1_ = std::clamp(
        static_cast<int>(declare_parameter<int>("vpi_p1", 8)), 1, 255);
    vpi_p2_ = std::clamp(
        static_cast<int>(declare_parameter<int>("vpi_p2", 81)), vpi_p1_,
        255);
    vpi_uniqueness_ = static_cast<float>(
        declare_parameter<double>("vpi_uniqueness", -1.0));
    vpi_ofa_window_size_ = std::clamp(
        static_cast<int>(declare_parameter<int>(
            "vpi_ofa_window_size", 7)),
        1, 7);
    if ((vpi_ofa_window_size_ % 2) == 0) {
      --vpi_ofa_window_size_;
    }
    vpi_ofa_num_passes_ = std::clamp(
        static_cast<int>(declare_parameter<int>(
            "vpi_ofa_num_passes", 2)),
        1, 3);
    vpi_ofa_apply_cpu_postfilters_ = declare_parameter<bool>(
        "vpi_ofa_apply_cpu_postfilters", false);
    vpi_output_median_filter_size_ = std::clamp(
        static_cast<int>(declare_parameter<int>(
            "vpi_output_median_filter_size", 1)) | 1,
        1, 5);
    vpi_output_median_max_difference_m_ = std::max(
        0.0, declare_parameter<double>(
            "vpi_output_median_max_difference_m", 0.08));
    const int block_size = std::clamp(
        static_cast<int>(declare_parameter<int>("block_size", 7)) | 1, 3,
        21);
    const int uniqueness_ratio = std::max(
        0, static_cast<int>(declare_parameter<int>("uniqueness_ratio", 10)));
    const int cuda_p1 = std::max(
        1, static_cast<int>(declare_parameter<int>("cuda_sgm_p1", 10)));
    const int cuda_p2 = std::max(
        cuda_p1 + 1,
        static_cast<int>(declare_parameter<int>("cuda_sgm_p2", 120)));
    const int cuda_lr_bm_block_size = std::clamp(
        static_cast<int>(declare_parameter<int>(
            "cuda_lr_bm_block_size", 15)) | 1,
        5, 51);
    speckle_window_size_ = std::max(
        0,
        static_cast<int>(declare_parameter<int>("speckle_window_size", 80)));
    speckle_range_ = std::max(
        0, static_cast<int>(declare_parameter<int>("speckle_range", 2)));
    const int disp12_max_diff =
        static_cast<int>(declare_parameter<int>("disp12_max_diff", 1));
    disparity_median_filter_size_ = std::clamp(
        static_cast<int>(declare_parameter<int>(
            "disparity_median_filter_size", 3)) | 1,
        1, 7);
    disparity_median_max_difference_ = std::max(
        0.0, declare_parameter<double>(
            "disparity_median_max_difference", 1.5));
    lr_consistency_far_depth_m_ = std::max(
        0.0, declare_parameter<double>(
            "lr_consistency_far_depth_m", 0.0));
    lr_consistency_max_difference_ = std::max(
        0.0, declare_parameter<double>(
            "lr_consistency_max_difference", 2.0));
    const int opencv_threads = std::max(
        0, static_cast<int>(declare_parameter<int>("opencv_num_threads", 3)));

    if (calibration_file.empty()) {
      throw std::runtime_error("calibration_file must not be empty");
    }
    if (!(min_depth_m_ > 0.0 && max_depth_m_ > min_depth_m_)) {
      throw std::runtime_error(
          "min_depth_m must be positive and smaller than max_depth_m");
    }
    if (opencv_threads > 0) {
      cv::setNumThreads(opencv_threads);
    }
    loadCalibration(calibration_file);

    int disparities = ((max_disparity_ + 15) / 16) * 16;
    if (use_cuda_sgm_) {
      if (cv::cuda::getCudaEnabledDeviceCount() <= 0) {
        RCLCPP_WARN(get_logger(),
                    "CUDA SGM requested but no CUDA device is available; "
                    "falling back to CPU StereoSGBM");
        use_cuda_sgm_ = false;
        depth_backend_ = "cpu_sgbm";
      } else {
        // OpenCV CUDA StereoSGM supports exactly 64, 128 or 256 disparities.
        disparities = disparities <= 64 ? 64 : (disparities <= 128 ? 128 : 256);
        cuda_stereo_ = cv::cuda::createStereoSGM(
            0, disparities, cuda_p1, cuda_p2, uniqueness_ratio,
            cv::cuda::StereoSGM::MODE_HH4);
        if (lr_consistency_far_depth_m_ > 0.0) {
          const int reverse_required_disparities = static_cast<int>(std::ceil(
              disparities * static_cast<double>(output_size_.width) /
              processing_size_.width));
          const int reverse_disparities =
              reverse_required_disparities <= 64
                  ? 64
                  : (reverse_required_disparities <= 128 ? 128 : 256);
          cuda_reverse_bm_ = cv::cuda::createStereoBM(
              reverse_disparities, cuda_lr_bm_block_size);
          cuda_reverse_bm_->setUniquenessRatio(uniqueness_ratio);
        }
        cuda_left_map_x_.upload(left_map_x_);
        cuda_left_map_y_.upload(left_map_y_);
        cuda_right_map_x_.upload(right_map_x_);
        cuda_right_map_y_.upload(right_map_y_);
      }
    }
    if (!use_cuda_sgm_) {
      stereo_ = cv::StereoSGBM::create(0, disparities, block_size);
      stereo_->setP1(8 * block_size * block_size);
      stereo_->setP2(32 * block_size * block_size);
      stereo_->setPreFilterCap(31);
      stereo_->setUniquenessRatio(uniqueness_ratio);
      stereo_->setSpeckleWindowSize(speckle_window_size_);
      stereo_->setSpeckleRange(speckle_range_);
      stereo_->setDisp12MaxDiff(disp12_max_diff);
      stereo_->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);
    }

    const auto qos = rclcpp::SensorDataQoS().keep_last(2);
    color_publisher_ = create_publisher<Image>(color_topic, qos);
    preview_publisher_ = create_publisher<Image>(preview_topic, qos);
    depth_publisher_ = create_publisher<Image>(depth_topic, qos);
    depth_preview_publisher_ =
        create_publisher<Image>(depth_preview_topic, qos);
    camera_info_publisher_ =
        create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic, qos);
    disparity_publisher_ = create_publisher<Image>(disparity_topic, qos);
    rgbd_publisher_ = create_publisher<rtabmap_msgs::msg::RGBDImage>(
        rgbd_topic, rclcpp::QoS(2).reliable());
    point_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        point_cloud_topic, rclcpp::SensorDataQoS().keep_last(1));

    if (compressed_input_) {
      const auto compressed_qos = rclcpp::QoS(5).reliable();
      const auto compressed_rmw_qos = compressed_qos.get_rmw_qos_profile();
      left_compressed_subscriber_.subscribe(this, left_topic, compressed_rmw_qos);
      right_compressed_subscriber_.subscribe(this, right_topic, compressed_rmw_qos);
      compressed_synchronizer_ = std::make_unique<
          message_filters::Synchronizer<CompressedStereoPolicy>>(
          CompressedStereoPolicy(5), left_compressed_subscriber_,
          right_compressed_subscriber_);
      compressed_synchronizer_->registerCallback(
          std::bind(&UsbStereoDepthNode::compressedStereoCallback, this,
                    std::placeholders::_1, std::placeholders::_2));
    } else {
      const auto input_rmw_qos = rclcpp::SensorDataQoS().get_rmw_qos_profile();
      left_subscriber_.subscribe(this, left_topic, input_rmw_qos);
      right_subscriber_.subscribe(this, right_topic, input_rmw_qos);
      synchronizer_ =
          std::make_unique<message_filters::Synchronizer<StereoPolicy>>(
              StereoPolicy(5), left_subscriber_, right_subscriber_);
      synchronizer_->registerCallback(
          std::bind(&UsbStereoDepthNode::stereoCallback, this,
                    std::placeholders::_1, std::placeholders::_2));
    }

    RCLCPP_INFO(
        get_logger(),
        "USB stereo depth ready: calibration=%s input=%dx%d stereo=%dx%d "
        "output=%dx%d backend=%s far_lr_check=%.1f_m "
        "baseline=%.4f m fx=%.2f disparity=%d rectification_alpha=%.2f "
        "horizontal_fov=%.1f deg depth_preview_roi=%d,%d %dx%d "
        "cloud_max_depth=%.1f m input_mode=%s",
        calibration_file.c_str(), calibration_size_.width,
        calibration_size_.height, processing_size_.width,
        processing_size_.height, output_size_.width, output_size_.height,
        depth_backend_.c_str(),
        use_cuda_sgm_ ? lr_consistency_far_depth_m_ : 0.0, baseline_m_,
        projection_output_left_.at<double>(0, 0), disparities,
        rectification_alpha_,
        2.0 * std::atan(
                  output_size_.width /
                  (2.0 * projection_output_left_.at<double>(0, 0))) *
            180.0 / CV_PI, depth_preview_roi_.x, depth_preview_roi_.y,
        depth_preview_roi_.width, depth_preview_roi_.height,
        point_cloud_max_depth_m_,
        compressed_input_ ? "jpeg" : "raw");
  }

  ~UsbStereoDepthNode() override { destroyVpi(); }

 private:
  bool checkVpiStatus(VPIStatus status, const char *operation) {
    if (status == VPI_SUCCESS) {
      return true;
    }
    char message[VPI_MAX_STATUS_MESSAGE_LENGTH] = {};
    vpiGetLastStatusMessage(message, sizeof(message));
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "USB VPI stereo %s failed: %s (%s)", operation,
        vpiStatusGetName(status), message);
    return false;
  }

  bool isVpiOfaPvaVic() const {
    return depth_backend_ == "vpi_ofa_pva_vic";
  }

  bool isVpiBackend() const {
    return depth_backend_ == "vpi_cuda" || isVpiOfaPvaVic();
  }

  uint64_t vpiBackendFlags() const {
    return isVpiOfaPvaVic()
               ? (VPI_BACKEND_OFA | VPI_BACKEND_PVA | VPI_BACKEND_VIC)
               : VPI_BACKEND_CUDA;
  }

  void destroyVpi() {
    // Destroy the stream first so no queued hardware work references images
    // or the payload while they are released.
    if (vpi_stream_ != nullptr) {
      vpiStreamDestroy(vpi_stream_);
      vpi_stream_ = nullptr;
    }
    if (vpi_disparity_ != nullptr) {
      vpiImageDestroy(vpi_disparity_);
      vpi_disparity_ = nullptr;
    }
    if (vpi_confidence_ != nullptr) {
      vpiImageDestroy(vpi_confidence_);
      vpi_confidence_ = nullptr;
    }
    if (vpi_input_left_wrapper_ != nullptr) {
      vpiImageDestroy(vpi_input_left_wrapper_);
      vpi_input_left_wrapper_ = nullptr;
    }
    if (vpi_input_right_wrapper_ != nullptr) {
      vpiImageDestroy(vpi_input_right_wrapper_);
      vpi_input_right_wrapper_ = nullptr;
    }
    if (vpi_stereo_left_ != nullptr) {
      vpiImageDestroy(vpi_stereo_left_);
      vpi_stereo_left_ = nullptr;
    }
    if (vpi_stereo_right_ != nullptr) {
      vpiImageDestroy(vpi_stereo_right_);
      vpi_stereo_right_ = nullptr;
    }
    if (vpi_payload_ != nullptr) {
      vpiPayloadDestroy(vpi_payload_);
      vpi_payload_ = nullptr;
    }
    vpi_size_ = cv::Size();
  }

  bool ensureVpi(const cv::Size &size) {
    if (vpi_stream_ != nullptr && vpi_payload_ != nullptr &&
        vpi_disparity_ != nullptr && vpi_size_ == size) {
      return true;
    }

    destroyVpi();
    VPIStereoDisparityEstimatorCreationParams create_params{};
    if (!checkVpiStatus(
            vpiInitStereoDisparityEstimatorCreationParams(&create_params),
            "parameter initialization")) {
      return false;
    }
    create_params.maxDisparity = max_disparity_;
    const bool ofa_pva_vic = isVpiOfaPvaVic();
    const uint64_t backend_flags = vpiBackendFlags();
    const VPIImageFormat input_format =
        ofa_pva_vic ? VPI_IMAGE_FORMAT_Y8_ER_BL : VPI_IMAGE_FORMAT_U8;

    if (!checkVpiStatus(vpiStreamCreate(0, &vpi_stream_),
                        "stream creation") ||
        !checkVpiStatus(vpiCreateStereoDisparityEstimator(
                            backend_flags, size.width, size.height,
                            input_format, &create_params, &vpi_payload_),
                        "payload creation") ||
        !checkVpiStatus(vpiImageCreate(size.width, size.height,
                                       VPI_IMAGE_FORMAT_S16, 0,
                                       &vpi_disparity_),
                        "output allocation") ||
        (ofa_pva_vic &&
         !checkVpiStatus(vpiImageCreate(size.width, size.height,
                                        VPI_IMAGE_FORMAT_Y8_ER_BL, 0,
                                        &vpi_stereo_left_),
                         "left OFA input allocation")) ||
        (ofa_pva_vic &&
         !checkVpiStatus(vpiImageCreate(size.width, size.height,
                                        VPI_IMAGE_FORMAT_Y8_ER_BL, 0,
                                        &vpi_stereo_right_),
                         "right OFA input allocation")) ||
        (ofa_pva_vic &&
         !checkVpiStatus(vpiImageCreate(size.width, size.height,
                                        VPI_IMAGE_FORMAT_U16, 0,
                                        &vpi_confidence_),
                         "confidence allocation")) ||
        !checkVpiStatus(vpiInitStereoDisparityEstimatorParams(&vpi_params_),
                        "submit parameter initialization")) {
      destroyVpi();
      return false;
    }

    vpi_params_.maxDisparity = max_disparity_;
    vpi_params_.confidenceThreshold = vpi_confidence_threshold_;
    vpi_params_.p1 = vpi_p1_;
    vpi_params_.p2 =
        ofa_pva_vic ? std::min(vpi_p2_, 89 - vpi_p1_) : vpi_p2_;
    vpi_params_.uniqueness = vpi_uniqueness_;
    if (ofa_pva_vic) {
      vpi_params_.windowSize = vpi_ofa_window_size_;
      vpi_params_.numPasses = vpi_ofa_num_passes_;
      vpi_params_.confidenceType =
          vpi_confidence_type_ == "absolute"
              ? VPI_STEREO_CONFIDENCE_ABSOLUTE
              : (vpi_confidence_type_ == "relative"
                     ? VPI_STEREO_CONFIDENCE_RELATIVE
                     : VPI_STEREO_CONFIDENCE_INFERENCE);
    }
    vpi_size_ = size;
    RCLCPP_INFO(get_logger(),
                "Initialized USB VPI %s for %dx%d, max_disparity=%d",
                ofa_pva_vic ? "OFA+PVA+VIC SGM" : "CUDA-SGM",
                size.width, size.height, max_disparity_);
    return true;
  }

  bool computeVpiDisparity(const cv::Mat &left_gray,
                           const cv::Mat &right_gray,
                           cv::Mat &disparity) {
    if (left_gray.empty() || right_gray.empty() ||
        left_gray.type() != CV_8UC1 || right_gray.type() != CV_8UC1 ||
        left_gray.size() != right_gray.size() ||
        !ensureVpi(left_gray.size())) {
      return false;
    }

    const bool ofa_pva_vic = isVpiOfaPvaVic();
    const VPIImageFormat wrapper_format =
        ofa_pva_vic ? VPI_IMAGE_FORMAT_Y8_ER : VPI_IMAGE_FORMAT_U8;
    const uint64_t backend_flags = vpiBackendFlags();
    const bool wrappers_ready =
        (vpi_input_left_wrapper_ == nullptr
             ? checkVpiStatus(vpiImageCreateWrapperOpenCVMat(
                                  left_gray, wrapper_format, 0,
                                  &vpi_input_left_wrapper_),
                              "left image wrapping")
             : checkVpiStatus(vpiImageSetWrappedOpenCVMat(
                                  vpi_input_left_wrapper_, left_gray),
                              "left image wrapper update")) &&
        (vpi_input_right_wrapper_ == nullptr
             ? checkVpiStatus(vpiImageCreateWrapperOpenCVMat(
                                  right_gray, wrapper_format, 0,
                                  &vpi_input_right_wrapper_),
                              "right image wrapping")
             : checkVpiStatus(vpiImageSetWrappedOpenCVMat(
                                  vpi_input_right_wrapper_, right_gray),
                              "right image wrapper update"));
    if (!wrappers_ready) {
      return false;
    }

    bool submitted = false;
    if (ofa_pva_vic) {
      VPIConvertImageFormatParams conversion{};
      submitted =
          checkVpiStatus(vpiInitConvertImageFormatParams(&conversion),
                         "conversion parameter initialization") &&
          checkVpiStatus(vpiSubmitConvertImageFormat(
                             vpi_stream_, VPI_BACKEND_VIC,
                             vpi_input_left_wrapper_, vpi_stereo_left_,
                             &conversion),
                         "left OFA input conversion") &&
          checkVpiStatus(vpiSubmitConvertImageFormat(
                             vpi_stream_, VPI_BACKEND_VIC,
                             vpi_input_right_wrapper_, vpi_stereo_right_,
                             &conversion),
                         "right OFA input conversion") &&
          checkVpiStatus(vpiSubmitStereoDisparityEstimator(
                             vpi_stream_, backend_flags, vpi_payload_,
                             vpi_stereo_left_, vpi_stereo_right_,
                             vpi_disparity_, vpi_confidence_, &vpi_params_),
                         "OFA+PVA+VIC submission");
    } else {
      submitted = checkVpiStatus(vpiSubmitStereoDisparityEstimator(
                                     vpi_stream_, backend_flags, vpi_payload_,
                                     vpi_input_left_wrapper_,
                                     vpi_input_right_wrapper_, vpi_disparity_,
                                     nullptr, &vpi_params_),
                                 "CUDA submission");
    }
    if (!submitted ||
        !checkVpiStatus(vpiStreamSync(vpi_stream_), "synchronization")) {
      return false;
    }

    VPIImageData data{};
    if (!checkVpiStatus(vpiImageLockData(
                            vpi_disparity_, VPI_LOCK_READ,
                            VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR, &data),
                        "output lock")) {
      return false;
    }
    cv::Mat disparity_q10_5;
    const bool exported = checkVpiStatus(
        vpiImageDataExportOpenCVMat(data, &disparity_q10_5),
        "output conversion");
    if (exported) {
      disparity_q10_5.convertTo(disparity, CV_32F, 1.0 / 32.0);
    }
    const bool unlocked =
        checkVpiStatus(vpiImageUnlock(vpi_disparity_), "output unlock");
    return exported && unlocked && !disparity.empty();
  }

  void loadCalibration(const std::string &path) {
    cv::FileStorage file(path, cv::FileStorage::READ);
    if (!file.isOpened()) {
      throw std::runtime_error("cannot open calibration file: " + path);
    }
    int width = 0;
    int height = 0;
    file["image_width"] >> width;
    file["image_height"] >> height;
    if (width <= 0 || height <= 0) {
      throw std::runtime_error(
          "calibration image_width and image_height must be positive");
    }
    calibration_size_ = cv::Size(width, height);
    processing_size_ = cv::Size(
        std::max(1, static_cast<int>(std::lround(width * processing_scale_))),
        std::max(1, static_cast<int>(std::lround(height * processing_scale_))));
    output_size_ = cv::Size(
        std::max(1, static_cast<int>(std::lround(width * output_scale_))),
        std::max(1, static_cast<int>(std::lround(height * output_scale_))));
    const double scale_x = static_cast<double>(processing_size_.width) / width;
    const double scale_y = static_cast<double>(processing_size_.height) / height;

    const cv::Mat camera_left =
        requireMatrix(file, "camera_matrix_left", 3, 3);
    const cv::Mat distortion_left =
        requireMatrix(file, "distortion_left", 1, 5);
    const cv::Mat camera_right =
        requireMatrix(file, "camera_matrix_right", 3, 3);
    const cv::Mat distortion_right =
        requireMatrix(file, "distortion_right", 1, 5);
    const cv::Mat rotation = requireMatrix(file, "rotation", 3, 3);
    const cv::Mat translation = requireMatrix(file, "translation", 3, 1);

    const cv::Mat scaled_camera_left =
        scaleIntrinsic(camera_left, scale_x, scale_y);
    const cv::Mat scaled_camera_right =
        scaleIntrinsic(camera_right, scale_x, scale_y);
    cv::Mat rectification_left;
    cv::Mat rectification_right;
    cv::Mat disparity_to_depth;
    cv::stereoRectify(
        scaled_camera_left, distortion_left, scaled_camera_right,
        distortion_right, processing_size_, rotation, translation,
        rectification_left, rectification_right, projection_left_,
        projection_right_, disparity_to_depth, cv::CALIB_ZERO_DISPARITY,
        rectification_alpha_, processing_size_);
    cv::initUndistortRectifyMap(
        scaled_camera_left, distortion_left, rectification_left,
        projection_left_, processing_size_, CV_32FC1, left_map_x_,
        left_map_y_);
    cv::initUndistortRectifyMap(
        scaled_camera_right, distortion_right, rectification_right,
        projection_right_, processing_size_, CV_32FC1, right_map_x_,
        right_map_y_);

    // alpha=1 keeps the calibrated field of view, so some rectified pixels
    // intentionally map outside the source images. OpenCV fills those pixels
    // with black. Without an explicit geometry mask SGBM can assign plausible
    // disparities to the black border, producing a slanted sheet of dark
    // points in RViz. Require the complete bilinear interpolation footprint
    // to remain inside each source image.
    left_rectification_valid_ =
        (left_map_x_ >= 0.0F) &
        (left_map_x_ < static_cast<float>(processing_size_.width - 1)) &
        (left_map_y_ >= 0.0F) &
        (left_map_y_ < static_cast<float>(processing_size_.height - 1));
    right_rectification_valid_ =
        (right_map_x_ >= 0.0F) &
        (right_map_x_ < static_cast<float>(processing_size_.width - 1)) &
        (right_map_y_ >= 0.0F) &
        (right_map_y_ < static_cast<float>(processing_size_.height - 1));
    // Keep calibrated RGB-D untouched, but expose an edge-free depth image
    // for RViz. Cropping the valid rectification ROI only in that display
    // topic avoids presenting the intentional alpha=1 border as missing
    // scene depth without changing camera intrinsics used by odometry.
    depth_preview_roi_ = cv::boundingRect(left_rectification_valid_);
    if (depth_preview_roi_.empty()) {
      depth_preview_roi_ = cv::Rect(0, 0, processing_size_.width,
                                    processing_size_.height);
    }

    const double output_scale_x =
        static_cast<double>(output_size_.width) / processing_size_.width;
    const double output_scale_y =
        static_cast<double>(output_size_.height) / processing_size_.height;
    projection_output_left_ =
        scaleProjection(projection_left_, output_scale_x, output_scale_y);

    const double right_fx = projection_right_.at<double>(0, 0);
    baseline_m_ = right_fx == 0.0
                      ? 0.0
                      : std::abs(projection_right_.at<double>(0, 3) / right_fx);
    if (!(baseline_m_ > 0.0) || projection_left_.at<double>(0, 0) <= 0.0) {
      throw std::runtime_error(
          "calibration projection matrices contain an invalid baseline or focal length");
    }
  }

  void stereoCallback(const Image::ConstSharedPtr &left_message,
                      const Image::ConstSharedPtr &right_message) {
    try {
      const cv::Mat left =
          cv_bridge::toCvShare(left_message, sensor_msgs::image_encodings::BGR8)
              ->image;
      const cv::Mat right =
          cv_bridge::toCvShare(right_message, sensor_msgs::image_encodings::BGR8)
              ->image;
      processStereoPair(left, right, left_message->header);
    } catch (const std::exception &error) {
      logFrameError(error);
    }
  }

  void compressedStereoCallback(
      const CompressedImage::ConstSharedPtr &left_message,
      const CompressedImage::ConstSharedPtr &right_message) {
    try {
      const cv::Mat left_buffer(
          1, static_cast<int>(left_message->data.size()), CV_8UC1,
          const_cast<uint8_t *>(left_message->data.data()));
      const cv::Mat right_buffer(
          1, static_cast<int>(right_message->data.size()), CV_8UC1,
          const_cast<uint8_t *>(right_message->data.data()));
      const cv::Mat left = cv::imdecode(left_buffer, cv::IMREAD_COLOR);
      const cv::Mat right = cv::imdecode(right_buffer, cv::IMREAD_COLOR);
      if (left.empty() || right.empty()) {
        throw std::runtime_error("failed to decode a JPEG stereo image");
      }
      processStereoPair(left, right, left_message->header);
    } catch (const std::exception &error) {
      logFrameError(error);
    }
  }

  void processStereoPair(const cv::Mat &left, const cv::Mat &right,
                         const std_msgs::msg::Header &input_header) {
    const auto now = std::chrono::steady_clock::now();
    if (max_processing_fps_ > 0.0 &&
        last_processed_ != std::chrono::steady_clock::time_point{} &&
        std::chrono::duration<double>(now - last_processed_).count() <
            1.0 / max_processing_fps_) {
      return;
    }
    last_processed_ = now;

    if (left.size() != calibration_size_ || right.size() != calibration_size_) {
      throw std::runtime_error(
          "input image size does not match the stereo calibration");
    }
    cv::Mat left_scaled;
    cv::Mat right_scaled;
    cv::Mat left_rectified;
    cv::Mat right_rectified;
    cv::Mat left_gray;
    cv::Mat right_gray;
    if (use_cuda_sgm_) {
      cuda_left_input_.upload(left);
      cuda_right_input_.upload(right);
      cv::cuda::resize(cuda_left_input_, cuda_left_scaled_, processing_size_,
                       0.0, 0.0, cv::INTER_AREA);
      cv::cuda::resize(cuda_right_input_, cuda_right_scaled_, processing_size_,
                       0.0, 0.0, cv::INTER_AREA);
      cv::cuda::remap(cuda_left_scaled_, cuda_left_rectified_,
                      cuda_left_map_x_, cuda_left_map_y_, cv::INTER_LINEAR,
                      cv::BORDER_CONSTANT);
      cv::cuda::remap(cuda_right_scaled_, cuda_right_rectified_,
                      cuda_right_map_x_, cuda_right_map_y_, cv::INTER_LINEAR,
                      cv::BORDER_CONSTANT);
      cv::cuda::cvtColor(cuda_left_rectified_, cuda_left_,
                         cv::COLOR_BGR2GRAY);
      cv::cuda::cvtColor(cuda_right_rectified_, cuda_right_,
                         cv::COLOR_BGR2GRAY);
      cuda_left_scaled_.download(left_scaled);
      cuda_left_rectified_.download(left_rectified);
    } else {
      cv::resize(left, left_scaled, processing_size_, 0.0, 0.0,
                 cv::INTER_AREA);
      cv::resize(right, right_scaled, processing_size_, 0.0, 0.0,
                 cv::INTER_AREA);
      cv::remap(left_scaled, left_rectified, left_map_x_, left_map_y_,
                cv::INTER_LINEAR, cv::BORDER_CONSTANT);
      cv::remap(right_scaled, right_rectified, right_map_x_, right_map_y_,
                cv::INTER_LINEAR, cv::BORDER_CONSTANT);
      cv::cvtColor(left_rectified, left_gray, cv::COLOR_BGR2GRAY);
      cv::cvtColor(right_rectified, right_gray, cv::COLOR_BGR2GRAY);
    }

    cv::Mat disparity_fixed;
    cv::Mat reverse_disparity_fixed;
    cv::Mat disparity;
    bool used_vpi = false;
    if (use_cuda_sgm_) {
      cuda_stereo_->compute(cuda_left_, cuda_right_, cuda_disparity_);
      cuda_disparity_.download(disparity_fixed);
      if (cuda_reverse_bm_) {
        // A half-resolution reverse pass is sufficient for rejecting far
        // mismatches and is much cheaper than a second full-resolution SGM.
        cv::cuda::resize(cuda_left_, cuda_lr_left_, output_size_, 0.0, 0.0,
                         cv::INTER_AREA);
        cv::cuda::resize(cuda_right_, cuda_lr_right_, output_size_, 0.0, 0.0,
                         cv::INTER_AREA);
        // CUDA StereoBM publishes unsigned disparity. Flip both inputs so the
        // right-to-left disparity becomes positive, then mirror the lookup
        // coordinate below.
        cv::cuda::flip(cuda_lr_right_, cuda_lr_right_flipped_, 1);
        cv::cuda::flip(cuda_lr_left_, cuda_lr_left_flipped_, 1);
        cuda_reverse_bm_->compute(cuda_lr_right_flipped_,
                                  cuda_lr_left_flipped_,
                                  cuda_reverse_disparity_,
                                  cv::cuda::Stream::Null());
        cuda_reverse_disparity_.download(reverse_disparity_fixed);
      }
      // StereoSGM does not expose StereoSGBM's internal speckle filtering.
      // Apply the equivalent connected-component rejection on its fixed-point
      // output before converting disparity to metres.
      if (speckle_window_size_ > 0) {
        cv::filterSpeckles(disparity_fixed, -16, speckle_window_size_,
                           speckle_range_ * 16);
      }
    } else if (isVpiBackend()) {
      used_vpi = computeVpiDisparity(left_gray, right_gray, disparity);
      if (!used_vpi) {
        if (!vpi_fallback_to_sgbm_) {
          throw std::runtime_error(
              "VPI stereo failed and CPU fallback is disabled");
        }
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Using CPU SGBM fallback because USB VPI stereo did not complete");
        stereo_->compute(left_gray, right_gray, disparity_fixed);
      }
    } else {
      stereo_->compute(left_gray, right_gray, disparity_fixed);
    }
    if (!used_vpi) {
      disparity_fixed.convertTo(disparity, CV_32FC1, 1.0 / 16.0);
    }

    cv::Mat disparity_locally_consistent(
        disparity.size(), CV_8UC1, cv::Scalar(255));
    const bool apply_cpu_disparity_filter =
        !isVpiOfaPvaVic() || vpi_ofa_apply_cpu_postfilters_;
    if (disparity_median_filter_size_ >= 3 &&
        apply_cpu_disparity_filter) {
      cv::Mat median_disparity;
      cv::medianBlur(disparity, median_disparity,
                     disparity_median_filter_size_);
      cv::Mat disparity_difference;
      cv::absdiff(disparity, median_disparity, disparity_difference);
      disparity_locally_consistent =
          (median_disparity > 0.5F) &
          (disparity_difference <= disparity_median_max_difference_);
      median_disparity.copyTo(disparity, disparity_locally_consistent);
    }
    cv::Mat depth_m;
    cv::divide(projection_left_.at<double>(0, 0) * baseline_m_, disparity,
               depth_m);

    cv::Mat lr_consistent(disparity.size(), CV_8UC1, cv::Scalar(255));
    if (!reverse_disparity_fixed.empty() &&
        lr_consistency_far_depth_m_ > 0.0) {
      const cv::Mat &reverse_disparity = reverse_disparity_fixed;
      const float reverse_scale_x =
          static_cast<float>(reverse_disparity.cols) / disparity.cols;
      const float reverse_scale_y =
          static_cast<float>(reverse_disparity.rows) / disparity.rows;
      for (int row = 0; row < disparity.rows; ++row) {
        const auto *disparity_row = disparity.ptr<float>(row);
        const auto *depth_row = depth_m.ptr<float>(row);
        const int reverse_row_index = std::clamp(
            static_cast<int>(std::lround(row * reverse_scale_y)), 0,
            reverse_disparity.rows - 1);
        const auto *reverse_row =
            reverse_disparity.ptr<uint8_t>(reverse_row_index);
        auto *consistent_row = lr_consistent.ptr<uint8_t>(row);
        for (int column = 0; column < disparity.cols; ++column) {
          if (depth_row[column] < lr_consistency_far_depth_m_) {
            continue;
          }
          const int right_column = static_cast<int>(std::lround(
              (column - disparity_row[column]) * reverse_scale_x));
          const int reverse_column =
              reverse_disparity.cols - 1 - right_column;
          if (right_column < 0 || right_column >= reverse_disparity.cols ||
              reverse_row[reverse_column] == 0 ||
              std::abs(disparity_row[column] * reverse_scale_x -
                       reverse_row[reverse_column]) >
                  lr_consistency_max_difference_ * reverse_scale_x) {
            consistent_row[column] = 0;
          }
        }
      }
    }
    cv::Mat valid = (disparity > 0.5f) & (depth_m >= min_depth_m_) &
                    (depth_m <= max_depth_m_) & left_rectification_valid_ &
                    disparity_locally_consistent & lr_consistent;

    // A valid left rectification pixel is not sufficient: its matched pixel
    // in the right rectified image may still lie in that image's black border.
    // Reject those correspondences before publishing depth or a point cloud.
    for (int row = 0; row < valid.rows; ++row) {
      auto *valid_row = valid.ptr<uint8_t>(row);
      const auto *disparity_row = disparity.ptr<float>(row);
      const auto *right_valid_row = right_rectification_valid_.ptr<uint8_t>(row);
      for (int column = 0; column < valid.cols; ++column) {
        if (valid_row[column] == 0) {
          continue;
        }
        const float right_column =
            static_cast<float>(column) - disparity_row[column];
        const int right_column_low = static_cast<int>(std::floor(right_column));
        const int right_column_high = right_column_low + 1;
        if (right_column_low < 0 || right_column_high >= valid.cols ||
            right_valid_row[right_column_low] == 0 ||
            right_valid_row[right_column_high] == 0) {
          valid_row[column] = 0;
        }
      }
    }

    // Passive stereo is ambiguous on long horizontal edges: a small
    // disparity error turns a few pixels into a long, slanted sheet in 3-D.
    // Only at long range, require a compact 2-D surface around the estimate.
    // Morphological opening removes thin rays while preserving broad walls and
    // other genuinely supported distant surfaces.
    if (far_artifact_depth_m_ > 0.0 &&
        far_artifact_depth_m_ < max_depth_m_) {
      cv::Mat far_candidates =
          valid & (depth_m >= far_artifact_depth_m_);
      cv::Mat far_supported;
      const cv::Mat support_kernel = cv::getStructuringElement(
          cv::MORPH_RECT,
          cv::Size(far_artifact_support_size_, far_artifact_support_size_));
      cv::morphologyEx(far_candidates, far_supported, cv::MORPH_OPEN,
                       support_kernel);
      valid.setTo(0, far_candidates & ~far_supported);
    }
    depth_m.setTo(0.0f, ~valid);
    disparity.setTo(-1.0f, ~valid);
    cv::Mat depth_mm;
    depth_m.convertTo(depth_mm, CV_16UC1, 1000.0);

    cv::Mat color_output;
    cv::Mat preview_output;
    cv::Mat depth_output;
    cv::Mat disparity_output;
    if (output_size_ != processing_size_) {
      cv::resize(left_rectified, color_output, output_size_, 0.0, 0.0,
                 cv::INTER_AREA);
      cv::resize(left_scaled, preview_output, output_size_, 0.0, 0.0,
                 cv::INTER_AREA);
      cv::resize(depth_mm, depth_output, output_size_, 0.0, 0.0,
                 cv::INTER_NEAREST);
      cv::resize(disparity, disparity_output, output_size_, 0.0, 0.0,
                 cv::INTER_NEAREST);
      disparity_output *= static_cast<float>(output_size_.width) /
                          static_cast<float>(processing_size_.width);
      disparity_output.setTo(-1.0F, disparity_output <= 0.0F);
    } else {
      color_output = left_rectified;
      preview_output = left_scaled;
      depth_output = depth_mm;
      disparity_output = disparity;
    }

    // OFA produces dense disparity cheaply, but isolated sub-pixel changes
    // are noisier than CUDA SGM. Filter at the published 480x270 resolution
    // so the cost stays small. The bounded median rejects flying pixels and
    // smooths supported surfaces without bridging a depth discontinuity.
    if (isVpiOfaPvaVic() && vpi_output_median_filter_size_ >= 3 &&
        vpi_output_median_max_difference_m_ > 0.0) {
      cv::Mat median_depth;
      cv::medianBlur(depth_output, median_depth,
                     vpi_output_median_filter_size_);
      cv::Mat depth_difference;
      cv::absdiff(depth_output, median_depth, depth_difference);
      const cv::Mat locally_supported =
          (depth_output > 0) & (median_depth > 0) &
          (depth_difference <= vpi_output_median_max_difference_m_ * 1000.0);
      depth_output.setTo(0, ~locally_supported);
      median_depth.copyTo(depth_output, locally_supported);
    }

    auto header = input_header;
    header.frame_id = output_frame_id_;
    // left_scaled is already produced for stereo processing.  Publishing it
    // only while a viewer is connected gives RViz the full, unrectified sensor
    // field of view without another resize or JPEG decode.  The rectified
    // image below remains the sole input to RGB-D odometry and mapping.
    if (preview_publisher_->get_subscription_count() > 0) {
      preview_publisher_->publish(imageMessage(
          header, sensor_msgs::image_encodings::BGR8, preview_output));
    }
    auto color_message = imageMessage(
        header, sensor_msgs::image_encodings::BGR8, color_output);
    auto depth_message = imageMessage(
        header, sensor_msgs::image_encodings::TYPE_16UC1, depth_output);
    auto camera_info =
        rectifiedCameraInfo(header, output_size_, projection_output_left_);

    rtabmap_msgs::msg::RGBDImage rgbd_message;
    rgbd_message.header = header;
    rgbd_message.rgb = color_message;
    rgbd_message.depth = depth_message;
    rgbd_message.rgb_camera_info = camera_info;
    rgbd_message.depth_camera_info = camera_info;
    rgbd_publisher_->publish(rgbd_message);

    if (color_publisher_->get_subscription_count() > 0) {
      color_publisher_->publish(color_message);
    }
    if (depth_publisher_->get_subscription_count() > 0) {
      depth_publisher_->publish(depth_message);
    }
    if (depth_preview_publisher_->get_subscription_count() > 0) {
      cv::Mat depth_preview;
      cv::resize(depth_mm(depth_preview_roi_), depth_preview, output_size_,
                 0.0, 0.0, cv::INTER_NEAREST);
      depth_preview_publisher_->publish(imageMessage(
          header, sensor_msgs::image_encodings::TYPE_16UC1, depth_preview));
    }
    if (camera_info_publisher_->get_subscription_count() > 0) {
      camera_info_publisher_->publish(camera_info);
    }
    if (point_cloud_publisher_->get_subscription_count() > 0) {
      const bool point_cloud_due =
          point_cloud_max_fps_ <= 0.0 ||
          last_point_cloud_ == std::chrono::steady_clock::time_point{} ||
          std::chrono::duration<double>(now - last_point_cloud_).count() >=
              1.0 / point_cloud_max_fps_;
      if (point_cloud_due) {
        point_cloud_publisher_->publish(pointCloudMessage(
            header, color_output, depth_output, projection_output_left_,
            static_cast<float>(point_cloud_max_depth_m_)));
        last_point_cloud_ = now;
      }
    }
    if (disparity_publisher_->get_subscription_count() > 0) {
      disparity_publisher_->publish(imageMessage(
          header, sensor_msgs::image_encodings::TYPE_32FC1,
          disparity_output));
    }

    ++processed_frames_;
    const int valid_pixels = cv::countNonZero(valid);
    const double valid_percent =
        100.0 * valid_pixels / static_cast<double>(valid.total());
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "USB stereo depth: frames=%llu valid=%.1f%% processing=%.1f ms",
        static_cast<unsigned long long>(processed_frames_), valid_percent,
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - now)
            .count());
  }

  void logFrameError(const std::exception &error) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Skipping USB stereo pair: %s", error.what());
  }

  double processing_scale_{0.5};
  double output_scale_{0.5};
  double rectification_alpha_{1.0};
  double min_depth_m_{0.4};
  double max_depth_m_{6.0};
  double far_artifact_depth_m_{3.0};
  int far_artifact_support_size_{3};
  int speckle_window_size_{80};
  int speckle_range_{2};
  int disparity_median_filter_size_{3};
  double disparity_median_max_difference_{1.5};
  double lr_consistency_far_depth_m_{0.0};
  double lr_consistency_max_difference_{2.0};
  double max_processing_fps_{10.0};
  double point_cloud_max_fps_{0.0};
  double point_cloud_max_depth_m_{0.0};
  double baseline_m_{0.0};
  std::string output_frame_id_;
  std::string depth_backend_{"cpu_sgbm"};
  bool vpi_fallback_to_sgbm_{true};
  int max_disparity_{128};
  int vpi_confidence_threshold_{0};
  std::string vpi_confidence_type_{"inference"};
  int vpi_p1_{8};
  int vpi_p2_{81};
  float vpi_uniqueness_{-1.0F};
  int vpi_ofa_window_size_{7};
  int vpi_ofa_num_passes_{2};
  bool vpi_ofa_apply_cpu_postfilters_{false};
  int vpi_output_median_filter_size_{1};
  double vpi_output_median_max_difference_m_{0.08};
  cv::Size calibration_size_;
  cv::Size processing_size_;
  cv::Size output_size_;
  cv::Mat projection_left_;
  cv::Mat projection_right_;
  cv::Mat projection_output_left_;
  cv::Mat left_map_x_;
  cv::Mat left_map_y_;
  cv::Mat right_map_x_;
  cv::Mat right_map_y_;
  cv::Mat left_rectification_valid_;
  cv::Mat right_rectification_valid_;
  cv::Rect depth_preview_roi_;
  cv::Ptr<cv::StereoSGBM> stereo_;
  cv::Ptr<cv::cuda::StereoSGM> cuda_stereo_;
  cv::Ptr<cv::cuda::StereoBM> cuda_reverse_bm_;
  cv::cuda::GpuMat cuda_left_;
  cv::cuda::GpuMat cuda_right_;
  cv::cuda::GpuMat cuda_disparity_;
  cv::cuda::GpuMat cuda_reverse_disparity_;
  cv::cuda::GpuMat cuda_lr_left_;
  cv::cuda::GpuMat cuda_lr_right_;
  cv::cuda::GpuMat cuda_lr_left_flipped_;
  cv::cuda::GpuMat cuda_lr_right_flipped_;
  cv::cuda::GpuMat cuda_left_input_;
  cv::cuda::GpuMat cuda_right_input_;
  cv::cuda::GpuMat cuda_left_scaled_;
  cv::cuda::GpuMat cuda_right_scaled_;
  cv::cuda::GpuMat cuda_left_rectified_;
  cv::cuda::GpuMat cuda_right_rectified_;
  cv::cuda::GpuMat cuda_left_map_x_;
  cv::cuda::GpuMat cuda_left_map_y_;
  cv::cuda::GpuMat cuda_right_map_x_;
  cv::cuda::GpuMat cuda_right_map_y_;
  VPIStream vpi_stream_{nullptr};
  VPIPayload vpi_payload_{nullptr};
  VPIImage vpi_disparity_{nullptr};
  VPIImage vpi_confidence_{nullptr};
  VPIImage vpi_input_left_wrapper_{nullptr};
  VPIImage vpi_input_right_wrapper_{nullptr};
  VPIImage vpi_stereo_left_{nullptr};
  VPIImage vpi_stereo_right_{nullptr};
  VPIStereoDisparityEstimatorParams vpi_params_{};
  cv::Size vpi_size_;
  bool use_cuda_sgm_{false};
  bool compressed_input_{true};
  message_filters::Subscriber<Image> left_subscriber_;
  message_filters::Subscriber<Image> right_subscriber_;
  std::unique_ptr<message_filters::Synchronizer<StereoPolicy>> synchronizer_;
  message_filters::Subscriber<CompressedImage> left_compressed_subscriber_;
  message_filters::Subscriber<CompressedImage> right_compressed_subscriber_;
  std::unique_ptr<message_filters::Synchronizer<CompressedStereoPolicy>>
      compressed_synchronizer_;
  rclcpp::Publisher<Image>::SharedPtr color_publisher_;
  rclcpp::Publisher<Image>::SharedPtr preview_publisher_;
  rclcpp::Publisher<Image>::SharedPtr depth_publisher_;
  rclcpp::Publisher<Image>::SharedPtr depth_preview_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr
      camera_info_publisher_;
  rclcpp::Publisher<Image>::SharedPtr disparity_publisher_;
  rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr rgbd_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      point_cloud_publisher_;
  std::chrono::steady_clock::time_point last_processed_;
  std::chrono::steady_clock::time_point last_point_cloud_;
  uint64_t processed_frames_{0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UsbStereoDepthNode>());
  rclcpp::shutdown();
  return 0;
}
