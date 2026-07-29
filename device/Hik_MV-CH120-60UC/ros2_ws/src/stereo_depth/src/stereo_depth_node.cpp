#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <rtabmap_msgs/msg/rgbd_image.hpp>
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
        disp12_max_diff_ = declare_parameter<int>("disp12_max_diff", 1);
        median_blur_size_ = declare_parameter<int>("median_blur_size", 5);
        texture_filter_min_gradient_ = declare_parameter<double>(
            "texture_filter_min_gradient", 4.0);
        depth_neighbor_filter_min_count_ = declare_parameter<int>(
            "depth_neighbor_filter_min_count", 3);
        depth_neighbor_filter_max_delta_m_ = declare_parameter<double>(
            "depth_neighbor_filter_max_delta_m", 0.08);
        rectify_images_ = declare_parameter<bool>("rectify_images", true);
        processing_scale_ = std::clamp(
            declare_parameter<double>("processing_scale", 0.5), 0.25, 1.0);
        // Preview topics are deliberately independent of the RGBD mapping
        // stream. RViz can render these small images smoothly without adding
        // a second full-resolution DDS consumer to the camera pipeline.
        preview_scale_ = std::clamp(
            declare_parameter<double>("preview_scale", 0.5), 0.1, 1.0);
        preview_publish_every_n_frames_ = std::max(
            1, static_cast<int>(declare_parameter<int>("preview_publish_every_n_frames", 2)));
        point_cloud_step_ = std::max(
            1, static_cast<int>(declare_parameter<int>("point_cloud_step", 4)));
        min_depth_m_ = declare_parameter<double>("min_depth_m", 0.2);
        max_depth_m_ = declare_parameter<double>("max_depth_m", 10.0);
        depth_edge_filter_max_delta_m_ = declare_parameter<double>(
            "depth_edge_filter_max_delta_m", 0.15);

        max_disparity_ = std::max(16, ((max_disparity_ + 15) / 16) * 16);
        block_size_ = std::max(3, block_size_ | 1);
        if (median_blur_size_ < 3) {
            median_blur_size_ = 0;
        } else {
            median_blur_size_ = std::min(9, median_blur_size_ | 1);
        }
        depth_neighbor_filter_min_count_ = std::clamp(depth_neighbor_filter_min_count_, 0, 8);
        stereo_ = cv::StereoSGBM::create(0, max_disparity_, block_size_);
        stereo_->setP1(8 * block_size_ * block_size_);
        stereo_->setP2(32 * block_size_ * block_size_);
        stereo_->setPreFilterCap(pre_filter_cap_);
        stereo_->setUniquenessRatio(uniqueness_ratio_);
        stereo_->setSpeckleWindowSize(speckle_size_);
        stereo_->setSpeckleRange(speckle_range_);
        stereo_->setDisp12MaxDiff(disp12_max_diff_);
        stereo_->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);

        // Keep a few samples because left and right are independent DDS topics.
        // The timestamp pairing cache below selects matching frames and prevents
        // a one-element queue from overwriting only one side of a stereo pair.
        auto sensor_qos = rclcpp::SensorDataQoS().keep_last(5);

        if (use_stereo_pair_) {
            stereo_pair_sub_ = create_subscription<sensor_msgs::msg::Image>(
                stereo_pair_topic, sensor_qos,
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

        auto output_sensor_qos = rclcpp::SensorDataQoS().keep_last(2);
        auto rgbd_qos = rclcpp::QoS(2).reliable().durability_volatile();
        auto camera_info_qos = rclcpp::QoS(5).reliable().durability_volatile();

        disparity_pub_ = create_publisher<sensor_msgs::msg::Image>("/stereo/disparity", output_sensor_qos);
        depth_pub_ = create_publisher<sensor_msgs::msg::Image>("/stereo/depth", output_sensor_qos);
        point_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/stereo/points", output_sensor_qos);
        left_rect_pub_ = create_publisher<sensor_msgs::msg::Image>("/stereo/left/image_rect", output_sensor_qos);
        right_rect_pub_ = create_publisher<sensor_msgs::msg::Image>("/stereo/right/image_rect", output_sensor_qos);
        left_rect_color_pub_ = create_publisher<sensor_msgs::msg::Image>(
            "/stereo/left/image_rect_color", output_sensor_qos);
        right_rect_color_pub_ = create_publisher<sensor_msgs::msg::Image>(
            "/stereo/right/image_rect_color", output_sensor_qos);
        preview_color_pub_ = create_publisher<sensor_msgs::msg::Image>(
            "/stereo/preview/left_rectified_color", output_sensor_qos);
        preview_depth_pub_ = create_publisher<sensor_msgs::msg::Image>(
            "/stereo/preview/depth_visual", output_sensor_qos);
        // These CameraInfo messages describe the rectified/scaled images above.
        // They are required by standard stereo consumers such as RTAB-Map.
        left_rect_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
            "/stereo/left/camera_info", camera_info_qos);
        right_rect_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
            "/stereo/right/camera_info", camera_info_qos);
        rgbd_pub_ = create_publisher<rtabmap_msgs::msg::RGBDImage>(
            "/stereo/rgbd_image", rgbd_qos);

        processing_running_.store(true);
        processing_thread_ = std::thread(&StereoDepthNode::processingLoop, this);
        RCLCPP_INFO(get_logger(), "Stereo depth node started.");
        RCLCPP_INFO(get_logger(), "Subscribed topics: left=%s right=%s left_info=%s right_info=%s",
                    left_topic.c_str(), right_topic.c_str(), left_info_topic.c_str(), right_info_topic.c_str());
        if (use_stereo_pair_) {
            RCLCPP_INFO(get_logger(), "Using packed stereo pair topic: %s", stereo_pair_topic.c_str());
        }
        RCLCPP_INFO(get_logger(), "Stereo parameters: baseline=%s max_disparity=%d block_size=%d pre_filter_size=%d pre_filter_cap=%d uniqueness_ratio=%d speckle_size=%d speckle_range=%d disp12_max_diff=%d",
                    baseline_m_ > 0.0 ? std::to_string(baseline_m_).c_str() : "auto-from-calibration",
                    max_disparity_, block_size_, pre_filter_size_, pre_filter_cap_,
                    uniqueness_ratio_, speckle_size_, speckle_range_, disp12_max_diff_);
        RCLCPP_INFO(get_logger(), "Depth filters: median_blur=%d texture_min_gradient=%.2f edge_delta=%.3f neighbor_count=%d neighbor_delta=%.3f",
                    median_blur_size_, texture_filter_min_gradient_,
                    depth_edge_filter_max_delta_m_, depth_neighbor_filter_min_count_,
                    depth_neighbor_filter_max_delta_m_);
        RCLCPP_INFO(get_logger(), "Performance parameters: processing_scale=%.2f point_cloud_step=%d preview=%.2f/%d SGBM_mode=3WAY",
                    processing_scale_, point_cloud_step_, preview_scale_, preview_publish_every_n_frames_);
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
            cv::Mat left_color_raw, right_color_raw;
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
                cv::cvtColor(left_gray_raw, left_color_raw, cv::COLOR_GRAY2BGR);
                cv::cvtColor(right_gray_raw, right_color_raw, cv::COLOR_GRAY2BGR);
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
                left_color_raw = left_raw;
                right_color_raw = right_raw;
                source_size = left_raw.size();
            }

            const cv::Size processing_size(
                std::max(1, static_cast<int>(std::lround(source_size.width * processing_scale_))),
                std::max(1, static_cast<int>(std::lround(source_size.height * processing_scale_))));
            const double scale_x = static_cast<double>(processing_size.width) / source_size.width;
            const double scale_y = static_cast<double>(processing_size.height) / source_size.height;

            cv::Mat left_gray, right_gray, left_color, right_color;
            if (rectify_images_) {
                ensureRectificationMaps(*left_info, *right_info, processing_size, scale_x, scale_y);
                cv::remap(left_gray_raw, left_gray, left_map_x_, left_map_y_, cv::INTER_LINEAR);
                cv::remap(right_gray_raw, right_gray, right_map_x_, right_map_y_, cv::INTER_LINEAR);
                cv::remap(left_color_raw, left_color, left_map_x_, left_map_y_, cv::INTER_LINEAR);
                cv::remap(right_color_raw, right_color, right_map_x_, right_map_y_, cv::INTER_LINEAR);
            } else if (processing_size != source_size) {
                cv::resize(left_gray_raw, left_gray, processing_size, 0.0, 0.0, cv::INTER_AREA);
                cv::resize(right_gray_raw, right_gray, processing_size, 0.0, 0.0, cv::INTER_AREA);
                cv::resize(left_color_raw, left_color, processing_size, 0.0, 0.0, cv::INTER_AREA);
                cv::resize(right_color_raw, right_color, processing_size, 0.0, 0.0, cv::INTER_AREA);
            } else {
                left_gray = left_gray_raw;
                right_gray = right_gray_raw;
                left_color = left_color_raw;
                right_color = right_color_raw;
            }

            cv::Mat disparity;
            stereo_->compute(left_gray, right_gray, disparity);
            if (median_blur_size_ >= 3) {
                cv::medianBlur(disparity, disparity, median_blur_size_);
            }

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
            filterLowTextureDepth(depth, left_gray);
            filterDepthEdges(depth);
            filterDepthNeighborhood(depth);

            const auto frame_id = left_header.frame_id.empty()
                ? std::string("left_camera_optical_frame") : left_header.frame_id;

            // The packed stereo input carries only the left header. Restore the
            // real right optical frame from CameraInfo before publishing, so TF
            // based consumers can distinguish the two optical frames.
            auto left_rect_header = left_header;
            auto right_rect_header = right_header;
            if (!left_info->header.frame_id.empty()) {
                left_rect_header.frame_id = left_info->header.frame_id;
            }
            if (!right_info->header.frame_id.empty()) {
                right_rect_header.frame_id = right_info->header.frame_id;
            }
            auto left_rect_msg = cv_bridge::CvImage(left_rect_header, "mono8", left_gray).toImageMsg();
            auto right_rect_msg = cv_bridge::CvImage(right_rect_header, "mono8", right_gray).toImageMsg();
            auto left_color_rect_msg = cv_bridge::CvImage(left_rect_header, "bgr8", left_color).toImageMsg();
            auto right_color_rect_msg = cv_bridge::CvImage(right_rect_header, "bgr8", right_color).toImageMsg();
            left_rect_pub_->publish(*left_rect_msg);
            right_rect_pub_->publish(*right_rect_msg);
            left_rect_color_pub_->publish(*left_color_rect_msg);
            right_rect_color_pub_->publish(*right_color_rect_msg);

            const auto left_rect_info = makeRectifiedCameraInfo(
                *left_info, left_rect_header, processing_size, scale_x, scale_y);
            const auto right_rect_info = makeRectifiedCameraInfo(
                *right_info, right_rect_header, processing_size, scale_x, scale_y);
            left_rect_info_pub_->publish(left_rect_info);
            right_rect_info_pub_->publish(right_rect_info);

            auto disparity_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", disparity_8u).toImageMsg();
            disparity_msg->header.stamp = left_header.stamp;
            disparity_msg->header.frame_id = frame_id;
            disparity_pub_->publish(*disparity_msg);

            auto depth_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "32FC1", depth).toImageMsg();
            depth_msg->header.stamp = left_header.stamp;
            depth_msg->header.frame_id = frame_id;
            depth_pub_->publish(*depth_msg);

            // A single RGBDImage is atomically timestamped: it avoids a
            // second message_filters synchronization stage in RTAB-Map.
            rtabmap_msgs::msg::RGBDImage rgbd_msg;
            rgbd_msg.header = left_rect_header;
            rgbd_msg.rgb_camera_info = left_rect_info;
            rgbd_msg.depth_camera_info = left_rect_info;
            rgbd_msg.rgb = *left_color_rect_msg;
            rgbd_msg.depth = *depth_msg;
            rgbd_pub_->publish(rgbd_msg);

            publishPointCloud(depth, left_color, fx, fy, cx, cy, left_header.stamp, frame_id);
            publishPreview(left_color, depth, left_rect_header);

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

    static sensor_msgs::msg::CameraInfo makeRectifiedCameraInfo(
        const sensor_msgs::msg::CameraInfo &source,
        const std_msgs::msg::Header &header,
        const cv::Size &image_size,
        double scale_x,
        double scale_y)
    {
        sensor_msgs::msg::CameraInfo info = source;
        info.header = header;
        info.width = static_cast<uint32_t>(image_size.width);
        info.height = static_cast<uint32_t>(image_size.height);

        // A rectified image has no remaining lens distortion and identity R.
        std::fill(info.d.begin(), info.d.end(), 0.0);
        info.r = {1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0};

        // P describes the rectified full-resolution image. Scale it to match
        // processing_scale, including P[3] for the right camera baseline.
        info.p[0] *= scale_x;
        info.p[1] *= scale_x;
        info.p[2] *= scale_x;
        info.p[3] *= scale_x;
        info.p[4] *= scale_y;
        info.p[5] *= scale_y;
        info.p[6] *= scale_y;
        info.p[7] *= scale_y;

        info.k = {info.p[0], info.p[1], info.p[2],
                  info.p[4], info.p[5], info.p[6],
                  info.p[8], info.p[9], info.p[10]};
        info.binning_x = 0;
        info.binning_y = 0;
        info.roi = sensor_msgs::msg::RegionOfInterest();
        return info;
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
        const cv::Mat &depth, const cv::Mat &color_image,
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
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        modifier.resize(point_count);

        sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(cloud, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> out_r(cloud, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> out_g(cloud, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> out_b(cloud, "b");
        for (int y = 0; y < depth.rows; y += point_cloud_step_) {
            for (int x = 0; x < depth.cols; x += point_cloud_step_) {
                const float z = depth.at<float>(y, x);
                if (z <= 0.0f) {
                    continue;
                }
                const cv::Vec3b bgr = color_image.at<cv::Vec3b>(y, x);
                *out_x = static_cast<float>((x - cx) * z / fx);
                *out_y = static_cast<float>((y - cy) * z / fy);
                *out_z = z;
                *out_r = bgr[2];
                *out_g = bgr[1];
                *out_b = bgr[0];
                ++out_x;
                ++out_y;
                ++out_z;
                ++out_r;
                ++out_g;
                ++out_b;
            }
        }
        point_cloud_pub_->publish(cloud);
    }

    void publishPreview(
        const cv::Mat &color, const cv::Mat &depth,
        const std_msgs::msg::Header &header)
    {
        ++preview_frame_count_;
        if ((preview_frame_count_ % preview_publish_every_n_frames_) != 0) {
            return;
        }

        cv::Mat preview_color;
        cv::Mat preview_depth;
        if (preview_scale_ < 0.999) {
            cv::resize(color, preview_color, cv::Size(), preview_scale_, preview_scale_, cv::INTER_AREA);
            cv::resize(depth, preview_depth, cv::Size(), preview_scale_, preview_scale_, cv::INTER_NEAREST);
        } else {
            preview_color = color;
            preview_depth = depth;
        }

        // Visual-only 8-bit inverse depth.  Invalid pixels remain black;
        // near valid surfaces are brighter, which is convenient in RViz.
        cv::Mat depth_visual;
        const double span = std::max(0.001, max_depth_m_ - min_depth_m_);
        preview_depth.convertTo(
            depth_visual, CV_8U, -255.0 / span, 255.0 * max_depth_m_ / span);
        depth_visual.setTo(0, preview_depth <= 0.0f);

        preview_color_pub_->publish(
            *cv_bridge::CvImage(header, "bgr8", preview_color).toImageMsg());
        preview_depth_pub_->publish(
            *cv_bridge::CvImage(header, "mono8", depth_visual).toImageMsg());
    }

    void filterDepthEdges(cv::Mat &depth) const
    {
        if (depth_edge_filter_max_delta_m_ <= 0.0 || depth.rows < 3 || depth.cols < 3) {
            return;
        }

        cv::Mat filtered = depth.clone();
        const float max_delta = static_cast<float>(depth_edge_filter_max_delta_m_);
        for (int y = 1; y < depth.rows - 1; ++y) {
            for (int x = 1; x < depth.cols - 1; ++x) {
                const float center = depth.at<float>(y, x);
                if (center <= 0.0f) {
                    continue;
                }

                int large_jumps = 0;
                const float neighbors[4] = {
                    depth.at<float>(y - 1, x),
                    depth.at<float>(y + 1, x),
                    depth.at<float>(y, x - 1),
                    depth.at<float>(y, x + 1)};
                for (const float neighbor : neighbors) {
                    if (neighbor > 0.0f && std::abs(center - neighbor) > max_delta) {
                        ++large_jumps;
                    }
                }

                if (large_jumps >= 2) {
                    filtered.at<float>(y, x) = 0.0f;
                }
            }
        }
        depth = filtered;
    }

    void filterLowTextureDepth(cv::Mat &depth, const cv::Mat &left_gray) const
    {
        if (texture_filter_min_gradient_ <= 0.0 || depth.empty() || left_gray.empty()) {
            return;
        }

        cv::Mat grad_x, grad_y, abs_grad_x, abs_grad_y, gradient;
        cv::Sobel(left_gray, grad_x, CV_16S, 1, 0, 3);
        cv::Sobel(left_gray, grad_y, CV_16S, 0, 1, 3);
        cv::convertScaleAbs(grad_x, abs_grad_x);
        cv::convertScaleAbs(grad_y, abs_grad_y);
        cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0.0, gradient);
        depth.setTo(0.0f, gradient < texture_filter_min_gradient_);
    }

    void filterDepthNeighborhood(cv::Mat &depth) const
    {
        if (depth_neighbor_filter_min_count_ <= 0 ||
            depth_neighbor_filter_max_delta_m_ <= 0.0 ||
            depth.rows < 3 || depth.cols < 3) {
            return;
        }

        cv::Mat filtered = depth.clone();
        const float max_delta = static_cast<float>(depth_neighbor_filter_max_delta_m_);
        for (int y = 1; y < depth.rows - 1; ++y) {
            for (int x = 1; x < depth.cols - 1; ++x) {
                const float center = depth.at<float>(y, x);
                if (center <= 0.0f) {
                    continue;
                }

                int supported_neighbors = 0;
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (dx == 0 && dy == 0) {
                            continue;
                        }
                        const float neighbor = depth.at<float>(y + dy, x + dx);
                        if (neighbor > 0.0f && std::abs(center - neighbor) <= max_delta) {
                            ++supported_neighbors;
                        }
                    }
                }

                if (supported_neighbors < depth_neighbor_filter_min_count_) {
                    filtered.at<float>(y, x) = 0.0f;
                }
            }
        }
        depth = filtered;
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
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_rect_color_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_rect_color_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr preview_color_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr preview_depth_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr left_rect_info_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr right_rect_info_pub_;
    rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr rgbd_pub_;
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
    int disp12_max_diff_{1};
    int median_blur_size_{5};
    double texture_filter_min_gradient_{4.0};
    int depth_neighbor_filter_min_count_{3};
    double depth_neighbor_filter_max_delta_m_{0.08};
    bool rectify_images_{true};
    bool use_stereo_pair_{false};
    double processing_scale_{0.5};
    double preview_scale_{0.5};
    int preview_publish_every_n_frames_{2};
    std::size_t preview_frame_count_{0};
    int point_cloud_step_{4};
    double min_depth_m_{0.2};
    double max_depth_m_{10.0};
    double depth_edge_filter_max_delta_m_{0.15};
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
