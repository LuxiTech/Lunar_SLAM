#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <atomic>
#include <array>
#include <cmath>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <thread>

class StereoDepthNode : public rclcpp::Node
{
public:
    StereoDepthNode()
    : Node("stereo_depth_node")
    {
        const auto left_topic = declare_parameter<std::string>("left_image_topic", "/left_camera/image");
        const auto right_topic = declare_parameter<std::string>("right_image_topic", "/right_camera/image");
        const auto left_info_topic = declare_parameter<std::string>("left_info_topic", "/left_camera/camera_info");
        const auto right_info_topic = declare_parameter<std::string>("right_info_topic", "/right_camera/camera_info");
        const auto stereo_pair_topic = declare_parameter<std::string>(
            "stereo_pair_topic", "/stereo_camera/image_pair_mono");
        use_stereo_pair_ = !stereo_pair_topic.empty();

        baseline_m_ = declare_parameter<double>("baseline_m", 0.0);
        max_disparity_ = declare_parameter<int>("max_disparity", 96);
        block_size_ = declare_parameter<int>("block_size", 9);
        pre_filter_size_ = declare_parameter<int>("pre_filter_size", 9);
        pre_filter_cap_ = declare_parameter<int>("pre_filter_cap", 31);
        uniqueness_ratio_ = declare_parameter<int>("uniqueness_ratio", 15);
        speckle_size_ = declare_parameter<int>("speckle_size", 100);
        speckle_range_ = declare_parameter<int>("speckle_range", 4);
        rectify_images_ = declare_parameter<bool>("rectify_images", true);
        processing_scale_ = std::clamp(
            declare_parameter<double>("processing_scale", 0.5), 0.25, 1.0);
        point_cloud_step_ = std::max(
            1, static_cast<int>(declare_parameter<int>("point_cloud_step", 4)));
        min_depth_m_ = declare_parameter<double>("min_depth_m", 0.2);
        max_depth_m_ = declare_parameter<double>("max_depth_m", 10.0);

        max_disparity_ = std::max(16, ((max_disparity_ + 15) / 16) * 16);
        block_size_ = std::max(3, block_size_ | 1);
        stereo_ = cv::StereoSGBM::create(0, max_disparity_, block_size_);
        stereo_->setP1(8 * block_size_ * block_size_);
        stereo_->setP2(32 * block_size_ * block_size_);
        stereo_->setPreFilterCap(pre_filter_cap_);
        stereo_->setUniquenessRatio(uniqueness_ratio_);
        stereo_->setSpeckleWindowSize(speckle_size_);
        stereo_->setSpeckleRange(speckle_range_);
        stereo_->setDisp12MaxDiff(1);
        stereo_->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);

        // Keep a few samples because left and right are independent DDS topics.
        // The timestamp pairing cache below selects matching frames and prevents
        // a one-element queue from overwriting only one side of a stereo pair.
        auto sensor_qos = rclcpp::SensorDataQoS().keep_last(5);

        if (use_stereo_pair_) {
            stereo_pair_sub_ = create_subscription<sensor_msgs::msg::Image>(
                stereo_pair_topic, rclcpp::QoS(2),
                std::bind(&StereoDepthNode::stereoPairCb, this, std::placeholders::_1));
        } else {
            left_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
                left_topic, sensor_qos,
                std::bind(&StereoDepthNode::leftImageCb, this, std::placeholders::_1));
            right_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
                right_topic, sensor_qos,
                std::bind(&StereoDepthNode::rightImageCb, this, std::placeholders::_1));
        }
        left_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            left_info_topic, sensor_qos,
            std::bind(&StereoDepthNode::leftInfoCb, this, std::placeholders::_1));
        right_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            right_info_topic, sensor_qos,
            std::bind(&StereoDepthNode::rightInfoCb, this, std::placeholders::_1));

        disparity_pub_ = create_publisher<sensor_msgs::msg::Image>("/stereo/disparity", 10);
        depth_pub_ = create_publisher<sensor_msgs::msg::Image>("/stereo/depth", 10);
        point_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/stereo/points", 10);
        left_rect_pub_ = create_publisher<sensor_msgs::msg::Image>("/stereo/left/image_rect", 10);
        right_rect_pub_ = create_publisher<sensor_msgs::msg::Image>("/stereo/right/image_rect", 10);

        processing_running_.store(true);
        processing_thread_ = std::thread(&StereoDepthNode::processingLoop, this);
        RCLCPP_INFO(get_logger(), "Stereo depth node started.");
        RCLCPP_INFO(get_logger(), "Subscribed topics: left=%s right=%s left_info=%s right_info=%s",
                    left_topic.c_str(), right_topic.c_str(), left_info_topic.c_str(), right_info_topic.c_str());
        if (use_stereo_pair_) {
            RCLCPP_INFO(get_logger(), "Using packed stereo pair topic: %s", stereo_pair_topic.c_str());
        }
        RCLCPP_INFO(get_logger(), "Stereo parameters: baseline=%s max_disparity=%d block_size=%d pre_filter_size=%d pre_filter_cap=%d uniqueness_ratio=%d speckle_size=%d speckle_range=%d",
                    baseline_m_ > 0.0 ? std::to_string(baseline_m_).c_str() : "auto-from-calibration",
                    max_disparity_, block_size_, pre_filter_size_, pre_filter_cap_,
                    uniqueness_ratio_, speckle_size_, speckle_range_);
        RCLCPP_INFO(get_logger(), "Performance parameters: processing_scale=%.2f point_cloud_step=%d SGBM_mode=3WAY",
                    processing_scale_, point_cloud_step_);
    }

    ~StereoDepthNode() override
    {
        processing_running_.store(false);
        work_condition_.notify_all();
        if (processing_thread_.joinable()) {
            processing_thread_.join();
        }
    }

private:
    void stereoPairCb(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            last_stereo_pair_ = msg;
            work_available_ = true;
        }
        work_condition_.notify_one();
    }

    void leftImageCb(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        bool matched = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            const int64_t stamp_ns = messageStampNanoseconds(msg->header.stamp);
            pending_left_images_[stamp_ns] = msg;
            const auto right_it = pending_right_images_.find(stamp_ns);
            if (right_it != pending_right_images_.end()) {
                last_left_image_ = msg;
                last_right_image_ = right_it->second;
                last_left_image_stamp_ = msg->header.stamp;
                last_right_image_stamp_ = right_it->second->header.stamp;
                pending_left_images_.erase(stamp_ns);
                pending_right_images_.erase(right_it);
                work_available_ = true;
                matched = true;
            }
            trimPendingImages();
        }
        if (matched) {
            work_condition_.notify_one();
        }
    }
    void rightImageCb(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        bool matched = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            const int64_t stamp_ns = messageStampNanoseconds(msg->header.stamp);
            pending_right_images_[stamp_ns] = msg;
            const auto left_it = pending_left_images_.find(stamp_ns);
            if (left_it != pending_left_images_.end()) {
                last_left_image_ = left_it->second;
                last_right_image_ = msg;
                last_left_image_stamp_ = left_it->second->header.stamp;
                last_right_image_stamp_ = msg->header.stamp;
                pending_left_images_.erase(left_it);
                pending_right_images_.erase(stamp_ns);
                work_available_ = true;
                matched = true;
            }
            trimPendingImages();
        }
        if (matched) {
            work_condition_.notify_one();
        }
    }
    void leftInfoCb(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        last_left_info_ = msg;
        work_available_ = true;
        work_condition_.notify_one();
    }
    void rightInfoCb(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        last_right_info_ = msg;
        work_available_ = true;
        work_condition_.notify_one();
    }

    void processingLoop()
    {
        while (processing_running_.load()) {
            {
                std::unique_lock<std::mutex> lock(mutex_);
                work_condition_.wait(lock, [this]() {
                    return !processing_running_.load() || work_available_;
                });
                if (!processing_running_.load()) {
                    break;
                }
                work_available_ = false;
            }
            process();
        }
    }

    void process()
    {
        sensor_msgs::msg::Image::SharedPtr left_image;
        sensor_msgs::msg::Image::SharedPtr right_image;
        sensor_msgs::msg::CameraInfo::SharedPtr left_info;
        sensor_msgs::msg::CameraInfo::SharedPtr right_info;
        sensor_msgs::msg::Image::SharedPtr stereo_pair;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            left_image = last_left_image_;
            right_image = last_right_image_;
            left_info = last_left_info_;
            right_info = last_right_info_;
            stereo_pair = last_stereo_pair_;
        }

        const bool images_ready = use_stereo_pair_
            ? static_cast<bool>(stereo_pair)
            : static_cast<bool>(left_image && right_image);
        if (!images_ready || !left_info || !right_info) {
            if (!received_any_message_) {
                received_any_message_ = true;
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for left/right image and camera info messages...");
            }
            return;
        }
        received_any_message_ = true;

        const auto left_header = use_stereo_pair_ ? stereo_pair->header : left_image->header;
        const auto right_header = use_stereo_pair_ ? stereo_pair->header : right_image->header;
        const rclcpp::Time frame_stamp(left_header.stamp);
        const int64_t frame_stamp_ns = frame_stamp.nanoseconds();
        if (frame_stamp_ns == last_processed_stamp_ns_) {
            return;
        }

        const rclcpp::Time right_stamp(right_header.stamp);
        const double stamp_delta_ms = std::abs((frame_stamp - right_stamp).seconds()) * 1000.0;
        if (stamp_delta_ms > 5.0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Skipping unsynchronized stereo pair (delta=%.3f ms)", stamp_delta_ms);
            return;
        }
        last_processed_stamp_ns_ = frame_stamp_ns;

        try {
            const auto process_started = std::chrono::steady_clock::now();
            cv::Mat left_gray_raw, right_gray_raw;
            cv::Size source_size;
            cv_bridge::CvImageConstPtr packed_cv;
            cv_bridge::CvImageConstPtr left_cv;
            cv_bridge::CvImageConstPtr right_cv;
            if (use_stereo_pair_) {
                packed_cv = cv_bridge::toCvShare(stereo_pair, "mono8");
                const cv::Mat &packed = packed_cv->image;
                if (packed.empty() || packed.cols < 2 || packed.cols % 2 != 0) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                         "Invalid packed stereo image size: %dx%d",
                                         packed.cols, packed.rows);
                    return;
                }
                const int single_width = packed.cols / 2;
                left_gray_raw = packed(cv::Rect(0, 0, single_width, packed.rows));
                right_gray_raw = packed(cv::Rect(single_width, 0, single_width, packed.rows));
                source_size = left_gray_raw.size();
            } else {
                left_cv = cv_bridge::toCvShare(left_image, "bgr8");
                right_cv = cv_bridge::toCvShare(right_image, "bgr8");
                const cv::Mat &left_raw = left_cv->image;
                const cv::Mat &right_raw = right_cv->image;
                if (left_raw.empty() || right_raw.empty() || left_raw.size() != right_raw.size()) {
                    return;
                }
                cv::cvtColor(left_raw, left_gray_raw, cv::COLOR_BGR2GRAY);
                cv::cvtColor(right_raw, right_gray_raw, cv::COLOR_BGR2GRAY);
                source_size = left_raw.size();
            }

            const cv::Size processing_size(
                std::max(1, static_cast<int>(std::lround(source_size.width * processing_scale_))),
                std::max(1, static_cast<int>(std::lround(source_size.height * processing_scale_))));
            const double scale_x = static_cast<double>(processing_size.width) / source_size.width;
            const double scale_y = static_cast<double>(processing_size.height) / source_size.height;

            cv::Mat left_gray, right_gray;
            if (rectify_images_) {
                ensureRectificationMaps(*left_info, *right_info, processing_size, scale_x, scale_y);
                cv::remap(left_gray_raw, left_gray, left_map_x_, left_map_y_, cv::INTER_LINEAR);
                cv::remap(right_gray_raw, right_gray, right_map_x_, right_map_y_, cv::INTER_LINEAR);
            } else if (processing_size != source_size) {
                cv::resize(left_gray_raw, left_gray, processing_size, 0.0, 0.0, cv::INTER_AREA);
                cv::resize(right_gray_raw, right_gray, processing_size, 0.0, 0.0, cv::INTER_AREA);
            } else {
                left_gray = left_gray_raw;
                right_gray = right_gray_raw;
            }

            cv::Mat disparity;
            stereo_->compute(left_gray, right_gray, disparity);

            cv::Mat disparity_8u;
            cv::normalize(disparity, disparity_8u, 0, 255, cv::NORM_MINMAX, CV_8U);

            cv::Mat depth;
            const double fx = (left_info->p[0] != 0.0 ? left_info->p[0] : left_info->k[0]) * scale_x;
            const double fy = (left_info->p[5] != 0.0 ? left_info->p[5] : left_info->k[4]) * scale_y;
            const double cx = (left_info->p[2] != 0.0 ? left_info->p[2] : left_info->k[2]) * scale_x;
            const double cy = (left_info->p[6] != 0.0 ? left_info->p[6] : left_info->k[5]) * scale_y;
            double baseline = baseline_m_;
            if (baseline <= 0.0 && right_info->p[0] != 0.0) {
                baseline = std::abs(right_info->p[3] / right_info->p[0]);
            }
            if (fx <= 0.0 || fy <= 0.0 || baseline <= 0.0) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "Invalid calibration: fx=%.3f fy=%.3f baseline=%.6f", fx, fy, baseline);
                return;
            }
            cv::Mat disparity_float;
            disparity.convertTo(disparity_float, CV_32F, 1.0 / 16.0);
            cv::divide(fx * baseline, disparity_float, depth);
            const cv::Mat valid_mask =
                (disparity_float > 0.5f) &
                (depth >= static_cast<float>(min_depth_m_)) &
                (depth <= static_cast<float>(max_depth_m_));
            depth.setTo(0.0f, ~valid_mask);

            const auto frame_id = left_header.frame_id.empty()
                ? std::string("left_camera_optical_frame") : left_header.frame_id;

            auto left_rect_msg = cv_bridge::CvImage(left_header, "mono8", left_gray).toImageMsg();
            auto right_rect_msg = cv_bridge::CvImage(right_header, "mono8", right_gray).toImageMsg();
            left_rect_pub_->publish(*left_rect_msg);
            right_rect_pub_->publish(*right_rect_msg);

            auto disparity_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", disparity_8u).toImageMsg();
            disparity_msg->header.stamp = left_header.stamp;
            disparity_msg->header.frame_id = frame_id;
            disparity_pub_->publish(*disparity_msg);

            auto depth_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "32FC1", depth).toImageMsg();
            depth_msg->header.stamp = left_header.stamp;
            depth_msg->header.frame_id = frame_id;
            depth_pub_->publish(*depth_msg);

            publishPointCloud(depth, left_gray, fx, fy, cx, cy, left_header.stamp, frame_id);

            const int valid_pixels = static_cast<int>(cv::countNonZero(depth > 0));
            const double process_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - process_started).count();
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "Processed frame: left=%s right=%s disparity=%dx%d valid_depth_pixels=%d processing=%.1f ms",
                                 formatTime(frame_stamp).c_str(),
                                 formatTime(right_stamp).c_str(),
                                 disparity.cols, disparity.rows, valid_pixels, process_ms);
        } catch (const std::exception &e) {
            RCLCPP_WARN(get_logger(), "Stereo depth processing failed: %s", e.what());
        }
    }

    static cv::Mat cameraMatrix(const std::array<double, 9> &values)
    {
        return cv::Mat(3, 3, CV_64F, const_cast<double *>(values.data())).clone();
    }

    static int64_t messageStampNanoseconds(const builtin_interfaces::msg::Time &stamp)
    {
        return static_cast<int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
    }

    void trimPendingImages()
    {
        constexpr std::size_t max_pending_frames = 8;
        while (pending_left_images_.size() > max_pending_frames) {
            pending_left_images_.erase(pending_left_images_.begin());
        }
        while (pending_right_images_.size() > max_pending_frames) {
            pending_right_images_.erase(pending_right_images_.begin());
        }
    }

    static cv::Mat projectionMatrix3x3(const std::array<double, 12> &p)
    {
        return (cv::Mat_<double>(3, 3) <<
            p[0], p[1], p[2],
            p[4], p[5], p[6],
            p[8], p[9], p[10]);
    }

    void ensureRectificationMaps(
        const sensor_msgs::msg::CameraInfo &left_info,
        const sensor_msgs::msg::CameraInfo &right_info,
        const cv::Size &size,
        double scale_x,
        double scale_y)
    {
        if (rectification_size_ == size && !left_map_x_.empty() && !right_map_x_.empty()) {
            return;
        }

        const cv::Mat left_k = cameraMatrix(left_info.k);
        const cv::Mat right_k = cameraMatrix(right_info.k);
        const cv::Mat left_d(left_info.d, true);
        const cv::Mat right_d(right_info.d, true);
        const cv::Mat left_r = cameraMatrix(left_info.r);
        const cv::Mat right_r = cameraMatrix(right_info.r);
        cv::Mat left_p = projectionMatrix3x3(left_info.p);
        cv::Mat right_p = projectionMatrix3x3(right_info.p);
        left_p.row(0) *= scale_x;
        left_p.row(1) *= scale_y;
        right_p.row(0) *= scale_x;
        right_p.row(1) *= scale_y;

        cv::initUndistortRectifyMap(left_k, left_d, left_r, left_p, size,
                                    CV_16SC2, left_map_x_, left_map_y_);
        cv::initUndistortRectifyMap(right_k, right_d, right_r, right_p, size,
                                    CV_16SC2, right_map_x_, right_map_y_);
        rectification_size_ = size;
        RCLCPP_INFO(get_logger(), "Initialized stereo rectification maps for %dx%d", size.width, size.height);
    }

    void publishPointCloud(
        const cv::Mat &depth, const cv::Mat &intensity_image,
        double fx, double fy, double cx, double cy,
        const builtin_interfaces::msg::Time &stamp, const std::string &frame_id)
    {
        std::size_t point_count = 0;
        for (int y = 0; y < depth.rows; y += point_cloud_step_) {
            for (int x = 0; x < depth.cols; x += point_cloud_step_) {
                if (depth.at<float>(y, x) > 0.0f) {
                    ++point_count;
                }
            }
        }

        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.stamp = stamp;
        cloud.header.frame_id = frame_id;
        cloud.height = 1;
        cloud.width = static_cast<uint32_t>(point_count);
        cloud.is_dense = true;
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2Fields(
            4,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(point_count);

        sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(cloud, "z");
        sensor_msgs::PointCloud2Iterator<float> out_intensity(cloud, "intensity");
        for (int y = 0; y < depth.rows; y += point_cloud_step_) {
            for (int x = 0; x < depth.cols; x += point_cloud_step_) {
                const float z = depth.at<float>(y, x);
                if (z <= 0.0f) {
                    continue;
                }
                *out_x = static_cast<float>((x - cx) * z / fx);
                *out_y = static_cast<float>((y - cy) * z / fy);
                *out_z = z;
                *out_intensity = static_cast<float>(intensity_image.at<uint8_t>(y, x));
                ++out_x;
                ++out_y;
                ++out_z;
                ++out_intensity;
            }
        }
        point_cloud_pub_->publish(cloud);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr stereo_pair_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr disparity_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_rect_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_rect_pub_;
    std::atomic_bool processing_running_{false};
    std::thread processing_thread_;
    std::condition_variable work_condition_;
    bool work_available_{false};
    std::mutex mutex_;
    sensor_msgs::msg::Image::SharedPtr last_left_image_;
    sensor_msgs::msg::Image::SharedPtr last_right_image_;
    sensor_msgs::msg::Image::SharedPtr last_stereo_pair_;
    std::map<int64_t, sensor_msgs::msg::Image::SharedPtr> pending_left_images_;
    std::map<int64_t, sensor_msgs::msg::Image::SharedPtr> pending_right_images_;
    sensor_msgs::msg::CameraInfo::SharedPtr last_left_info_;
    sensor_msgs::msg::CameraInfo::SharedPtr last_right_info_;
    double baseline_m_{0.0};
    int max_disparity_{96};
    int block_size_{9};
    int pre_filter_size_{9};
    int pre_filter_cap_{31};
    int uniqueness_ratio_{15};
    int speckle_size_{100};
    int speckle_range_{4};
    bool rectify_images_{true};
    bool use_stereo_pair_{false};
    double processing_scale_{0.5};
    int point_cloud_step_{4};
    double min_depth_m_{0.2};
    double max_depth_m_{10.0};
    bool received_any_message_{false};
    rclcpp::Time last_left_image_stamp_{};
    rclcpp::Time last_right_image_stamp_{};
    int64_t last_processed_stamp_ns_{-1};
    cv::Ptr<cv::StereoSGBM> stereo_;
    cv::Mat left_map_x_;
    cv::Mat left_map_y_;
    cv::Mat right_map_x_;
    cv::Mat right_map_y_;
    cv::Size rectification_size_;

    std::string formatTime(const rclcpp::Time &stamp) const
    {
        if (!stamp.nanoseconds()) {
            return "(none)";
        }
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(3) << stamp.seconds();
        return oss.str();
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StereoDepthNode>());
    rclcpp::shutdown();
    return 0;
}
