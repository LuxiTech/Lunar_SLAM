#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

#include "cv_bridge/cv_bridge.h"

#include "opencv2/opencv.hpp"

#include "camera_info_manager/camera_info_manager.hpp"

#include "hikrobot_camera_driver/StereoCamera.hpp"
#include "hikrobot_camera_driver/CameraConfig.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <vector>

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
        // RViz's compressed image transport on Humble requests Reliable QoS.
        // Keep exactly one preview frame so a slow renderer always receives the
        // newest frame instead of accumulating display latency.
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
            rviz_compressed_qos
        );
        right_preview_pub_ =
        create_publisher<sensor_msgs::msg::Image>(
            "/stereo/preview/right_color",
            rviz_compressed_qos
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


        if(!camera_.open(config))
        {

            RCLCPP_ERROR(
                get_logger(),
                "camera open failed"
            );

            return;
        }


        if (!camera_.start())
        {
            RCLCPP_ERROR(
                get_logger(),
                "camera start failed; node will not publish images"
            );
            camera_.close();
            return;
        }


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
            config.user_set.c_str(),
            config.trigger_source.c_str()
        );

    }

private:

    void capture()
    {

        cv::Mat left;

        cv::Mat right;


        uint64_t left_ts;

        uint64_t right_ts;

        int64_t left_host_ts;

        int64_t right_host_ts;



        if(!camera_.grab(
            left,
            right,
            left_ts,
            right_ts,
            left_host_ts,
            right_host_ts))
        {

            RCLCPP_WARN(
                get_logger(),
                "grab failed"
            );

            return;
        }

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

        // Prefer the SDK host timestamp captured with the frame instead of
        // stamping after Bayer conversion.  Keep one common stamp for the two
        // externally-triggered images so downstream stereo nodes can still pair
        // them exactly.
        const auto stamp = makeCommonFrameStamp(left_host_ts, right_host_ts);

        logTimestampDiagnostics(stamp, left_host_ts, right_host_ts, left_ts, right_ts);


        auto left_msg =
        cv_bridge::CvImage(
            std_msgs::msg::Header(),
            "bgr8",
            left
        )
        .toImageMsg();



        auto right_msg =
        cv_bridge::CvImage(
            std_msgs::msg::Header(),
            "bgr8",
            right
        )
        .toImageMsg();

        left_msg->header.stamp = stamp;
        left_msg->header.frame_id = "left_camera_optical_frame";

        right_msg->header.stamp = stamp;
        right_msg->header.frame_id = "right_camera_optical_frame";

        // Pack the synchronized grayscale pair into one DDS sample.  The
        // depth node can split it without risking one-sided image loss, and
        // the payload is one third of two BGR images.
        cv::Mat left_gray;
        cv::Mat right_gray;
        cv::Mat stereo_pair;
        cv::cvtColor(left, left_gray, cv::COLOR_BGR2GRAY);
        cv::cvtColor(right, right_gray, cv::COLOR_BGR2GRAY);
        cv::hconcat(left_gray, right_gray, stereo_pair);
        auto pair_msg = cv_bridge::CvImage(
            std_msgs::msg::Header(), "mono8", stereo_pair).toImageMsg();
        pair_msg->header.stamp = stamp;
        pair_msg->header.frame_id = "left_camera_optical_frame";

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

        stereo_pair_pub_->publish(*pair_msg);
        publishRvizPreviews(left, right, left_msg->header);

    }

    void publishRvizPreviews(
        const cv::Mat &left, const cv::Mat &right,
        const std_msgs::msg::Header &header)
    {
        // These streams are only for remote RViz.  Keep full-resolution
        // images local for stereo depth, while sending compact previews over
        // Wi-Fi so the display does not build up multi-second DDS queues.
        cv::Mat left_preview;
        cv::Mat right_preview;
        if (rviz_preview_scale_ < 0.999) {
            cv::resize(left, left_preview, cv::Size(), rviz_preview_scale_,
                       rviz_preview_scale_, cv::INTER_AREA);
            cv::resize(right, right_preview, cv::Size(), rviz_preview_scale_,
                       rviz_preview_scale_, cv::INTER_AREA);
        } else {
            left_preview = left;
            right_preview = right;
        }
        left_preview_pub_->publish(
            *cv_bridge::CvImage(header, "bgr8", left_preview).toImageMsg());
        right_preview_pub_->publish(
            *cv_bridge::CvImage(header, "bgr8", right_preview).toImageMsg());

        const std::vector<int> jpeg_parameters{cv::IMWRITE_JPEG_QUALITY, 80};
        std::vector<uchar> left_jpeg;
        std::vector<uchar> right_jpeg;
        cv::imencode(".jpg", left_preview, left_jpeg, jpeg_parameters);
        cv::imencode(".jpg", right_preview, right_jpeg, jpeg_parameters);

        auto left_compressed = std::make_unique<sensor_msgs::msg::CompressedImage>();
        left_compressed->header = header;
        left_compressed->format = "bgr8; jpeg compressed bgr8";
        left_compressed->data = std::move(left_jpeg);
        left_preview_compressed_pub_->publish(std::move(left_compressed));

        auto right_compressed = std::make_unique<sensor_msgs::msg::CompressedImage>();
        right_compressed->header = header;
        right_compressed->format = "bgr8; jpeg compressed bgr8";
        right_compressed->data = std::move(right_jpeg);
        right_preview_compressed_pub_->publish(std::move(right_compressed));
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
