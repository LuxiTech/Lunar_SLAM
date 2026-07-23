#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include "cv_bridge/cv_bridge.hpp"

#include "opencv2/opencv.hpp"

#include "camera_info_manager/camera_info_manager.hpp"

#include "hikrobot_camera_driver/StereoCamera.hpp"
#include "hikrobot_camera_driver/CameraConfig.hpp"

#include <ament_index_cpp/get_package_share_path.hpp>

class StereoCameraNode : public rclcpp::Node
{

public:

    StereoCameraNode()
    :
    Node("stereo_node")
    {

        const std::string default_config = (
            ament_index_cpp::get_package_share_path("hikrobot_camera_driver") /
            "config/stereo_camera.xml").string();
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
            (ament_index_cpp::get_package_share_path("hikrobot_camera_driver") / "config/stereo_left.yaml").string());
        const std::string right_calib_file = declare_parameter<std::string>(
            "right_camera_info_file",
            (ament_index_cpp::get_package_share_path("hikrobot_camera_driver") / "config/stereo_right.yaml").string());

        left_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
            this->get_node_base_interface(),
            this->get_node_services_interface(),
            this->get_node_logging_interface(),
            "mvch120_stereo/left",
            "",
            rclcpp::SystemDefaultsQoS(),
            "");
        right_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
            this->get_node_base_interface(),
            this->get_node_services_interface(),
            this->get_node_logging_interface(),
            "mvch120_stereo/right",
            "",
            rclcpp::SystemDefaultsQoS(),
            "");

        if (!left_info_manager_->loadCameraInfo("file://" + left_calib_file)) {
            RCLCPP_WARN(get_logger(), "Failed to load left camera info from %s", left_calib_file.c_str());
        }
        if (!right_info_manager_->loadCameraInfo("file://" + right_calib_file)) {
            RCLCPP_WARN(get_logger(), "Failed to load right camera info from %s", right_calib_file.c_str());
        }

        left_pub_ =
        create_publisher<sensor_msgs::msg::Image>(
            "/left_camera/image",
            10
        );


        right_pub_ =
        create_publisher<sensor_msgs::msg::Image>(
            "/right_camera/image",
            10
        );

        left_info_pub_ =
        create_publisher<sensor_msgs::msg::CameraInfo>(
            "/left_camera/camera_info",
            10
        );


        right_info_pub_ =
        create_publisher<sensor_msgs::msg::CameraInfo>(
            "/right_camera/camera_info",
            10
        );

        stereo_pair_pub_ =
        create_publisher<sensor_msgs::msg::Image>(
            "/stereo_camera/image_pair_mono",
            2
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



        if(!camera_.grab(
            left,
            right,
            left_ts,
            right_ts))
        {

            RCLCPP_WARN(
                get_logger(),
                "grab failed"
            );

            return;
        }



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



        const auto stamp = now();

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
        warnIfCalibrationSizeMismatch("left", left_info, left.cols, left.rows);
        left_info.header = left_msg->header;
        left_info.width = static_cast<uint32_t>(left.cols);
        left_info.height = static_cast<uint32_t>(left.rows);

        sensor_msgs::msg::CameraInfo right_info = right_info_manager_->getCameraInfo();
        warnIfCalibrationSizeMismatch("right", right_info, right.cols, right.rows);
        right_info.header = right_msg->header;
        right_info.width = static_cast<uint32_t>(right.cols);
        right_info.height = static_cast<uint32_t>(right.rows);



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

    }


    void warnIfCalibrationSizeMismatch(
        const char* camera_name,
        const sensor_msgs::msg::CameraInfo& info,
        int image_width,
        int image_height)
    {
        if (info.width == 0 || info.height == 0) {
            return;
        }

        if (info.width == static_cast<uint32_t>(image_width) &&
            info.height == static_cast<uint32_t>(image_height)) {
            return;
        }

        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "%s camera calibration size is %ux%u, but image size is %dx%d. Check ROI and calibration YAML.",
            camera_name,
            info.width,
            info.height,
            image_width,
            image_height);
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
