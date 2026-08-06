#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

#include "opencv2/opencv.hpp"

#include "camera_info_manager/camera_info_manager.hpp"

#include "hikrobot_camera_driver/StereoCamera.hpp"
#include "hikrobot_camera_driver/CameraConfig.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdint>
#include <memory>
#include <vector>

namespace
{

sensor_msgs::msg::Image::UniquePtr imageMessage(
    const std_msgs::msg::Header &header, const std::string &encoding, const cv::Mat &image)
{
    auto message = std::make_unique<sensor_msgs::msg::Image>();
    message->header = header;
    message->height = static_cast<uint32_t>(image.rows);
    message->width = static_cast<uint32_t>(image.cols);
    message->encoding = encoding;
    message->is_bigendian = false;
    message->step = static_cast<sensor_msgs::msg::Image::_step_type>(
        image.cols * image.elemSize());
    message->data.resize(static_cast<std::size_t>(message->step) * message->height);
    for (int row = 0; row < image.rows; ++row) {
        std::memcpy(
            message->data.data() + static_cast<std::size_t>(row) * message->step,
            image.ptr(row), message->step);
    }
    return message;
}

}  // namespace

class StereoCameraNode : public rclcpp::Node
{

public:

    StereoCameraNode()
    :
    Node("stereo_node")
    {

        const std::string package_share =
            ament_index_cpp::get_package_share_directory("hikrobot_camera_driver");
        const std::string default_config = package_share + "/config/stereo_camera.xml";
        const std::string config_file = declare_parameter<std::string>("config_file", default_config);
        StereoCameraConfig config;
        std::string config_error;
        if (!loadStereoCameraConfig(config_file, config, config_error)) {
            RCLCPP_ERROR(get_logger(), "Failed to load config '%s': %s",
                         config_file.c_str(), config_error.c_str());
            return;
        }
        RCLCPP_INFO(get_logger(), "camera SDK config: %s", config_file.c_str());
        RCLCPP_INFO(get_logger(), "image config: pixel_format=%s, swap_red_blue=%s, auto_white_balance=%s",
                    config.pixel_format.c_str(),
                    config.swap_red_blue ? "true" : "false",
                    config.auto_white_balance ? "true" : "false");

        // ROS parameters override XML values for one-off experiments.
        config.external_trigger = declare_parameter<bool>("external_trigger", config.external_trigger);
        camera_config_ = config;
        reconnect_after_failures_ = static_cast<int>(std::max<int64_t>(
            1, declare_parameter<int64_t>("reconnect_after_failures", 3)));
        reconnect_interval_seconds_ = std::max(
            1.0, declare_parameter<double>("reconnect_interval_seconds", 2.0));

        const std::string left_calib_file = declare_parameter<std::string>(
            "left_camera_info_file",
            package_share + "/config/stereo_left.yaml");
        const std::string right_calib_file = declare_parameter<std::string>(
            "right_camera_info_file",
            package_share + "/config/stereo_right.yaml");

        left_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
            this,
            "mvch120_stereo/left",
            "");
        right_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
            this,
            "mvch120_stereo/right",
            "");

        if (!left_info_manager_->loadCameraInfo("file://" + left_calib_file)) {
            RCLCPP_WARN(get_logger(), "Failed to load left camera info from %s", left_calib_file.c_str());
        }
        if (!right_info_manager_->loadCameraInfo("file://" + right_calib_file)) {
            RCLCPP_WARN(get_logger(), "Failed to load right camera info from %s", right_calib_file.c_str());
        }

        auto image_qos = rclcpp::SensorDataQoS().keep_last(2);
        // Raw RViz previews must never back-pressure the camera callback.
        // Compressed image transport on Humble requests Reliable QoS, so keep
        // that compatibility only for the much smaller JPEG stream.
        auto rviz_raw_qos = rclcpp::SensorDataQoS().keep_last(1);
        auto rviz_compressed_qos = rclcpp::QoS(1).reliable().durability_volatile();
        auto info_qos = rclcpp::QoS(5).reliable().durability_volatile();
        rviz_preview_scale_ = std::clamp(
            declare_parameter<double>("rviz_preview_scale", 0.25), 0.1, 1.0);
        capture_scale_ = std::clamp(
            declare_parameter<double>("capture_scale", 1.0), 0.25, 1.0);

        left_pub_ =
        create_publisher<sensor_msgs::msg::Image>(
            "/left_camera/image",
            image_qos
        );


        right_pub_ =
        create_publisher<sensor_msgs::msg::Image>(
            "/right_camera/image",
            image_qos
        );

        left_info_pub_ =
        create_publisher<sensor_msgs::msg::CameraInfo>(
            "/left_camera/camera_info",
            info_qos
        );


        right_info_pub_ =
        create_publisher<sensor_msgs::msg::CameraInfo>(
            "/right_camera/camera_info",
            info_qos
        );

        stereo_pair_pub_ =
        create_publisher<sensor_msgs::msg::Image>(
            "/stereo_camera/image_pair_mono",
            image_qos
        );
        left_preview_pub_ =
        create_publisher<sensor_msgs::msg::Image>(
            "/stereo/preview/left_color",
            rviz_raw_qos
        );
        right_preview_pub_ =
        create_publisher<sensor_msgs::msg::Image>(
            "/stereo/preview/right_color",
            rviz_raw_qos
        );
        left_preview_compressed_pub_ =
        create_publisher<sensor_msgs::msg::CompressedImage>(
            "/stereo/preview/left_color/compressed",
            rviz_compressed_qos
        );
        right_preview_compressed_pub_ =
        create_publisher<sensor_msgs::msg::CompressedImage>(
            "/stereo/preview/right_color/compressed",
            rviz_compressed_qos
        );


        camera_ready_ = openCamera();


        timer_ =
        create_wall_timer(
            std::chrono::milliseconds(33),
            std::bind(
                &StereoCameraNode::capture,
                this
            )
        );


        RCLCPP_INFO(
            get_logger(),
            "Hikrobot stereo node started with user_set=%s, trigger_source=%s",
            camera_config_.user_set.c_str(),
            camera_config_.trigger_source.c_str()
        );

    }

private:

    bool openCamera()
    {
        last_reconnect_attempt_ = std::chrono::steady_clock::now();
        if (!camera_.open(camera_config_)) {
            RCLCPP_ERROR(get_logger(), "camera open failed; automatic retry remains active");
            return false;
        }
        if (!camera_.start()) {
            RCLCPP_ERROR(get_logger(), "camera start failed; automatic retry remains active");
            camera_.close();
            return false;
        }
        consecutive_grab_failures_ = 0;
        RCLCPP_INFO(get_logger(), "Hik stereo cameras are ready");
        return true;
    }

    void capture()
    {
        if (!camera_ready_) {
            const double since_attempt = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - last_reconnect_attempt_).count();
            if (since_attempt >= reconnect_interval_seconds_) {
                camera_ready_ = openCamera();
            }
            return;
        }

        cv::Mat left;

        cv::Mat right;


        uint64_t left_ts;

        uint64_t right_ts;

        int64_t left_host_ts;

        int64_t right_host_ts;

        uint32_t left_frame_number;

        uint32_t right_frame_number;



        if(!camera_.grab(
            left,
            right,
            left_ts,
            right_ts,
            left_host_ts,
            right_host_ts,
            left_frame_number,
            right_frame_number))
        {
            ++consecutive_grab_failures_;
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "stereo grab failed (%d/%d before reconnect)",
                consecutive_grab_failures_, reconnect_after_failures_);
            if (consecutive_grab_failures_ >= reconnect_after_failures_) {
                camera_.stop();
                camera_.close();
                camera_ready_ = false;
                last_reconnect_attempt_ = std::chrono::steady_clock::now();
                RCLCPP_ERROR(get_logger(), "stereo stream lost; scheduling automatic reconnect");
            }

            return;
        }
        consecutive_grab_failures_ = 0;

        const int raw_width = left.cols;
        const int raw_height = left.rows;

        // Hardware decimation is unsupported with the active UserSet. Scale
        // directly after capture so all ROS, depth and SLAM stages receive a
        // smaller full-field image.
        if (capture_scale_ < 0.999) {
            cv::Mat left_scaled;
            cv::Mat right_scaled;
            cv::resize(left, left_scaled, cv::Size(), capture_scale_, capture_scale_, cv::INTER_AREA);
            cv::resize(right, right_scaled, cv::Size(), capture_scale_, capture_scale_, cv::INTER_AREA);
            left = std::move(left_scaled);
            right = std::move(right_scaled);
        }
        if (!output_resolution_logged_) {
            RCLCPP_INFO(
                get_logger(),
                "ROS image stream: SDK raw=%dx%d -> published=%dx%d (capture_scale=%.2f)",
                raw_width, raw_height, left.cols, left.rows, capture_scale_);
            output_resolution_logged_ = true;
        }

        // Prefer the SDK host timestamp captured with the frame instead of
        // stamping after Bayer conversion.  Keep one common stamp for the two
        // externally-triggered images so downstream stereo nodes can still pair
        // them exactly.
        const auto stamp = makeCommonFrameStamp(left_host_ts, right_host_ts);

        logTimestampDiagnostics(stamp, left_host_ts, right_host_ts, left_ts, right_ts);


        auto left_msg = imageMessage(std_msgs::msg::Header(), "bgr8", left);
        auto right_msg = imageMessage(std_msgs::msg::Header(), "bgr8", right);

        left_msg->header.stamp = stamp;
        left_msg->header.frame_id = "left_camera_optical_frame";

        right_msg->header.stamp = stamp;
        right_msg->header.frame_id = "right_camera_optical_frame";

        sensor_msgs::msg::CameraInfo left_info = left_info_manager_->getCameraInfo();
        scaleCameraInfoToImage("left", left_info, left.cols, left.rows);
        left_info.header = left_msg->header;

        sensor_msgs::msg::CameraInfo right_info = right_info_manager_->getCameraInfo();
        scaleCameraInfoToImage("right", right_info, right.cols, right.rows);
        right_info.header = right_msg->header;



        left_pub_->publish(
            *left_msg
        );

        left_info_pub_->publish(
            left_info
        );


        right_pub_->publish(
            *right_msg
        );

        right_info_pub_->publish(
            right_info
        );

        if (stereo_pair_pub_->get_subscription_count() > 0) {
            // Generate the packed stream only when a consumer asks for it.
            // The normal RGB-D pipeline subscribes to left/right directly.
            cv::Mat left_gray;
            cv::Mat right_gray;
            cv::Mat stereo_pair;
            cv::cvtColor(left, left_gray, cv::COLOR_BGR2GRAY);
            cv::cvtColor(right, right_gray, cv::COLOR_BGR2GRAY);
            cv::hconcat(left_gray, right_gray, stereo_pair);
            auto pair_msg = imageMessage(std_msgs::msg::Header(), "mono8", stereo_pair);
            pair_msg->header.stamp = stamp;
            pair_msg->header.frame_id = "left_camera_optical_frame";
            stereo_pair_pub_->publish(*pair_msg);
        }
        publishRvizPreviews(left, right, left_msg->header);

    }

    void publishRvizPreviews(
        const cv::Mat &left, const cv::Mat &right,
        const std_msgs::msg::Header &header)
    {
        const bool publish_left_raw = left_preview_pub_->get_subscription_count() > 0;
        const bool publish_right_raw = right_preview_pub_->get_subscription_count() > 0;
        const bool publish_left_compressed =
            left_preview_compressed_pub_->get_subscription_count() > 0;
        const bool publish_right_compressed =
            right_preview_compressed_pub_->get_subscription_count() > 0;
        if (!publish_left_raw && !publish_right_raw &&
            !publish_left_compressed && !publish_right_compressed) {
            return;
        }

        // These streams are only for RViz. Keep them entirely out of the
        // camera path when RViz is displaying the point cloud only.
        const bool need_left = publish_left_raw || publish_left_compressed;
        const bool need_right = publish_right_raw || publish_right_compressed;
        cv::Mat left_preview;
        cv::Mat right_preview;
        if (need_left) {
            if (rviz_preview_scale_ < 0.999) {
                cv::resize(left, left_preview, cv::Size(), rviz_preview_scale_,
                           rviz_preview_scale_, cv::INTER_AREA);
            } else {
                left_preview = left;
            }
        }
        if (need_right) {
            if (rviz_preview_scale_ < 0.999) {
                cv::resize(right, right_preview, cv::Size(), rviz_preview_scale_,
                           rviz_preview_scale_, cv::INTER_AREA);
            } else {
                right_preview = right;
            }
        }
        if (publish_left_raw) {
            left_preview_pub_->publish(*imageMessage(header, "bgr8", left_preview));
        }
        if (publish_right_raw) {
            right_preview_pub_->publish(*imageMessage(header, "bgr8", right_preview));
        }

        if (publish_left_compressed || publish_right_compressed) {
            const std::vector<int> jpeg_parameters{cv::IMWRITE_JPEG_QUALITY, 80};
            if (publish_left_compressed) {
                std::vector<uchar> left_jpeg;
                cv::imencode(".jpg", left_preview, left_jpeg, jpeg_parameters);
                auto left_compressed = std::make_unique<sensor_msgs::msg::CompressedImage>();
                left_compressed->header = header;
                left_compressed->format = "bgr8; jpeg compressed bgr8";
                left_compressed->data = std::move(left_jpeg);
                left_preview_compressed_pub_->publish(std::move(left_compressed));
            }
            if (publish_right_compressed) {
                std::vector<uchar> right_jpeg;
                cv::imencode(".jpg", right_preview, right_jpeg, jpeg_parameters);
                auto right_compressed = std::make_unique<sensor_msgs::msg::CompressedImage>();
                right_compressed->header = header;
                right_compressed->format = "bgr8; jpeg compressed bgr8";
                right_compressed->data = std::move(right_jpeg);
                right_preview_compressed_pub_->publish(std::move(right_compressed));
            }
        }
    }


    void scaleCameraInfoToImage(
        const char* camera_name,
        sensor_msgs::msg::CameraInfo& info,
        int image_width,
        int image_height)
    {
        if (info.width == 0 || info.height == 0) {
            info.width = static_cast<uint32_t>(image_width);
            info.height = static_cast<uint32_t>(image_height);
            return;
        }

        if (info.width == static_cast<uint32_t>(image_width) &&
            info.height == static_cast<uint32_t>(image_height)) {
            return;
        }

        const uint32_t calibration_width = info.width;
        const uint32_t calibration_height = info.height;
        const double scale_x = static_cast<double>(image_width) / calibration_width;
        const double scale_y = static_cast<double>(image_height) / calibration_height;
        // Decimation preserves the optical centre and field of view.  Scale
        // the intrinsic and projection matrices so rectification/depth use
        // the same camera model at the hardware output resolution.
        info.k[0] *= scale_x;
        info.k[2] *= scale_x;
        info.k[4] *= scale_y;
        info.k[5] *= scale_y;
        info.p[0] *= scale_x;
        info.p[1] *= scale_x;
        info.p[2] *= scale_x;
        info.p[3] *= scale_x;
        info.p[4] *= scale_y;
        info.p[5] *= scale_y;
        info.p[6] *= scale_y;
        info.p[7] *= scale_y;
        info.width = static_cast<uint32_t>(image_width);
        info.height = static_cast<uint32_t>(image_height);
        info.binning_x = 0;
        info.binning_y = 0;

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "%s calibration scaled from %ux%u to %dx%d (x=%.3f y=%.3f).",
            camera_name,
            calibration_width,
            calibration_height,
            image_width,
            image_height,
            scale_x,
            scale_y);
    }

    rclcpp::Time makeCommonFrameStamp(int64_t left_host_ts, int64_t right_host_ts) const
    {
        const int64_t left_host_ns = normalizeSdkHostTimestampToNs(left_host_ts);
        const int64_t right_host_ns = normalizeSdkHostTimestampToNs(right_host_ts);

        if (isValidUnixTimestampNs(left_host_ns) && isValidUnixTimestampNs(right_host_ns)) {
            return rclcpp::Time((left_host_ns + right_host_ns) / 2, get_clock()->get_clock_type());
        }
        if (isValidUnixTimestampNs(left_host_ns)) {
            return rclcpp::Time(left_host_ns, get_clock()->get_clock_type());
        }
        if (isValidUnixTimestampNs(right_host_ns)) {
            return rclcpp::Time(right_host_ns, get_clock()->get_clock_type());
        }
        return now();
    }

    static int64_t normalizeSdkHostTimestampToNs(int64_t host_ts)
    {
        if (host_ts <= 0) {
            return 0;
        }

        // Different MVS/transport versions expose nHostTimeStamp in ns, us, or
        // ms.  Normalize by magnitude so ROS stamps stay in Unix nanoseconds.
        if (host_ts >= 1000000000000000000LL) {  // ns, e.g. 1784797053309000000
            return host_ts;
        }
        if (host_ts >= 1000000000000000LL) {     // us, e.g. 1784797053309000
            return host_ts * 1000LL;
        }
        if (host_ts >= 1000000000000LL) {        // ms, e.g. 1784797053309
            return host_ts * 1000000LL;
        }
        return 0;
    }

    static bool isValidUnixTimestampNs(int64_t host_ts_ns)
    {
        // Accept a broad sane range and fall back to ROS now if unavailable.
        constexpr int64_t min_reasonable_ns = 1000000000000000000LL;  // 2001-09-09
        constexpr int64_t max_reasonable_ns = 4102444800000000000LL;  // 2100-01-01
        return host_ts_ns >= min_reasonable_ns && host_ts_ns <= max_reasonable_ns;
    }

    void logTimestampDiagnostics(
        const rclcpp::Time& stamp,
        int64_t left_host_ts,
        int64_t right_host_ts,
        uint64_t left_dev_ts,
        uint64_t right_dev_ts)
    {
        const auto ros_now = now();
        const double stamp_latency_ms = (ros_now - stamp).seconds() * 1000.0;
        const int64_t left_host_ns = normalizeSdkHostTimestampToNs(left_host_ts);
        const int64_t right_host_ns = normalizeSdkHostTimestampToNs(right_host_ts);
        const double host_delta_ms =
            (isValidUnixTimestampNs(left_host_ns) && isValidUnixTimestampNs(right_host_ns))
            ? std::abs(static_cast<double>(left_host_ns - right_host_ns)) / 1.0e6
            : -1.0;

        RCLCPP_INFO_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "camera timestamp: source=%s latency=%.2f ms left_host=%ld right_host=%ld host_delta=%.3f ms left_dev=%lu right_dev=%lu",
            (host_delta_ms >= 0.0) ? "sdk_host_common" : "ros_now_fallback",
            stamp_latency_ms,
            static_cast<long>(left_host_ts),
            static_cast<long>(right_host_ts),
            host_delta_ms,
            static_cast<unsigned long>(left_dev_ts),
            static_cast<unsigned long>(right_dev_ts));
    }



private:

    StereoCamera camera_;

    StereoCameraConfig camera_config_;

    bool camera_ready_{false};
    int consecutive_grab_failures_{0};
    int reconnect_after_failures_{3};
    double reconnect_interval_seconds_{2.0};
    std::chrono::steady_clock::time_point last_reconnect_attempt_{};


    rclcpp::Publisher<
        sensor_msgs::msg::Image
    >::SharedPtr left_pub_;


    rclcpp::Publisher<
        sensor_msgs::msg::Image
    >::SharedPtr right_pub_;

    rclcpp::Publisher<
        sensor_msgs::msg::CameraInfo
    >::SharedPtr left_info_pub_;


    rclcpp::Publisher<
        sensor_msgs::msg::CameraInfo
    >::SharedPtr right_info_pub_;

    rclcpp::Publisher<
        sensor_msgs::msg::Image
    >::SharedPtr stereo_pair_pub_;

    rclcpp::Publisher<
        sensor_msgs::msg::Image
    >::SharedPtr left_preview_pub_;

    rclcpp::Publisher<
        sensor_msgs::msg::Image
    >::SharedPtr right_preview_pub_;

    rclcpp::Publisher<
        sensor_msgs::msg::CompressedImage
    >::SharedPtr left_preview_compressed_pub_;

    rclcpp::Publisher<
        sensor_msgs::msg::CompressedImage
    >::SharedPtr right_preview_compressed_pub_;

    double rviz_preview_scale_{0.25};
    double capture_scale_{1.0};
    bool output_resolution_logged_{false};

    std::shared_ptr<camera_info_manager::CameraInfoManager> left_info_manager_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> right_info_manager_;

    rclcpp::TimerBase::SharedPtr timer_;

};



int main(
    int argc,
    char **argv
)
{

    rclcpp::init(
        argc,
        argv
    );


    auto node =
    std::make_shared<StereoCameraNode>();


    rclcpp::spin(node);


    rclcpp::shutdown();


    return 0;
}
