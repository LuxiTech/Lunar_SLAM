#include <filesystem>
#include <memory>
#include <string>
#include <utility>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Transform.h>
#include <rtabmap_conversions/MsgConversion.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace luxi_rtab_map
{
namespace
{
using ImageMsg = sensor_msgs::msg::Image;
using CameraInfoMsg = sensor_msgs::msg::CameraInfo;
using ImageInfoPolicy = message_filters::sync_policies::ApproximateTime<ImageMsg, CameraInfoMsg>;
using RgbdPolicy =
  message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg, CameraInfoMsg>;

cv::Mat toColorImage(const ImageMsg::ConstSharedPtr & msg)
{
  const std::string & encoding = msg->encoding;
  if (encoding == sensor_msgs::image_encodings::RGB8 ||
      encoding == sensor_msgs::image_encodings::BGR8 ||
      encoding == sensor_msgs::image_encodings::MONO8 ||
      encoding == sensor_msgs::image_encodings::TYPE_8UC1) {
    return cv_bridge::toCvShare(msg)->image.clone();
  }

  if (encoding == sensor_msgs::image_encodings::MONO16) {
    return cv_bridge::cvtColor(cv_bridge::toCvShare(msg), sensor_msgs::image_encodings::MONO8)
      ->image.clone();
  }

  return cv_bridge::cvtColor(cv_bridge::toCvShare(msg), sensor_msgs::image_encodings::BGR8)
    ->image.clone();
}

cv::Mat toDepthImage(const ImageMsg::ConstSharedPtr & msg)
{
  const std::string & encoding = msg->encoding;
  if (encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
      encoding == sensor_msgs::image_encodings::MONO16 ||
      encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    return cv_bridge::toCvShare(msg)->image.clone();
  }

  throw std::runtime_error("depth image must use 16UC1, mono16 or 32FC1 encoding");
}

rtabmap::ParametersMap baseParameters()
{
  rtabmap::ParametersMap parameters;
  parameters.insert({rtabmap::Parameters::kMemIncrementalMemory(), "true"});
  parameters.insert({rtabmap::Parameters::kMemInitWMWithAllNodes(), "false"});
  parameters.insert({rtabmap::Parameters::kRGBDCreateOccupancyGrid(), "true"});
  parameters.insert({rtabmap::Parameters::kGridSensor(), "1"});
  parameters.insert({rtabmap::Parameters::kGridDepthDecimation(), "4"});
  parameters.insert({rtabmap::Parameters::kGridRangeMax(), "5.0"});
  parameters.insert({rtabmap::Parameters::kGridCellSize(), "0.05"});
  return parameters;
}
}  // namespace

class RtabMapCppNode final : public rclcpp::Node
{
public:
  RtabMapCppNode()
  : Node("luxi_rtab_map_node")
  {
    mode_ = declare_parameter<std::string>("mode", "rgbd");
    rgb_topic_ = declare_parameter<std::string>("rgb_topic", "/sensors/rgbd/color/image_raw");
    depth_topic_ = declare_parameter<std::string>(
      "depth_topic", "/sensors/rgbd/depth/image_raw");
    camera_info_topic_ = declare_parameter<std::string>(
      "camera_info_topic", "/sensors/rgbd/color/camera_info");
    database_path_ = declare_parameter<std::string>(
      "database_path", "/tmp/luxi_rtab_map_test.db");
    delete_db_on_start_ = declare_parameter<bool>("delete_db_on_start", false);
    max_frames_ = declare_parameter<int>("max_frames", 0);
    sync_queue_size_ = declare_parameter<int>("sync_queue_size", 10);
    log_every_n_frames_ = declare_parameter<int>("log_every_n_frames", 30);

    if (mode_ != "rgbd" && mode_ != "depth") {
      throw std::runtime_error("mode must be 'rgbd' or 'depth'");
    }

    initRtabmap();
    createSubscriptions();
    status_timer_ = create_wall_timer(
      std::chrono::seconds(5), std::bind(&RtabMapCppNode::logStatus, this));

    RCLCPP_INFO(
      get_logger(),
      "luxi_rtab_map_node started: mode=%s database=%s rgb=%s depth=%s camera_info=%s",
      mode_.c_str(), database_path_.c_str(), rgb_topic_.c_str(), depth_topic_.c_str(),
      camera_info_topic_.c_str());

    if (mode_ == "depth") {
      RCLCPP_WARN(
        get_logger(),
        "Depth-only mode is a feasibility test. Reliable RTAB-Map mapping without RGB usually "
        "requires external odometry or depth->PointCloud2 plus ICP odometry.");
    }
  }

  ~RtabMapCppNode() override
  {
    rtabmap_.close(true);
  }

private:
  void initRtabmap()
  {
    if (delete_db_on_start_ && !database_path_.empty()) {
      std::error_code error;
      std::filesystem::remove(database_path_, error);
      if (error) {
        RCLCPP_WARN(
          get_logger(), "Failed to remove database '%s': %s", database_path_.c_str(),
          error.message().c_str());
      }
    }

    rtabmap::ParametersMap parameters = baseParameters();
    if (mode_ == "depth") {
      parameters.insert({rtabmap::Parameters::kRegStrategy(), "1"});
      parameters.insert({rtabmap::Parameters::kIcpVoxelSize(), "0.05"});
      parameters.insert({rtabmap::Parameters::kIcpMaxCorrespondenceDistance(), "0.1"});
      parameters.insert({rtabmap::Parameters::kIcpCorrespondenceRatio(), "0.05"});
    } else {
      parameters.insert({rtabmap::Parameters::kRegStrategy(), "0"});
    }

    rtabmap_.init(parameters, database_path_, false);
    odometry_.reset(rtabmap::Odometry::create(parameters));
    if (!odometry_) {
      throw std::runtime_error("failed to create RTAB-Map odometry");
    }
  }

  void createSubscriptions()
  {
    if (mode_ == "rgbd") {
      rgb_sub_ = std::make_unique<message_filters::Subscriber<ImageMsg>>(
        this, rgb_topic_, rmw_qos_profile_sensor_data);
      depth_sub_ = std::make_unique<message_filters::Subscriber<ImageMsg>>(
        this, depth_topic_, rmw_qos_profile_sensor_data);
      rgb_camera_info_sub_ = std::make_unique<message_filters::Subscriber<CameraInfoMsg>>(
        this, camera_info_topic_, rmw_qos_profile_sensor_data);
      rgbd_sync_ = std::make_unique<message_filters::Synchronizer<RgbdPolicy>>(
        RgbdPolicy(sync_queue_size_), *rgb_sub_, *depth_sub_, *rgb_camera_info_sub_);
      rgbd_sync_->registerCallback(
        std::bind(
          &RtabMapCppNode::rgbdCallback, this, std::placeholders::_1, std::placeholders::_2,
          std::placeholders::_3));
      return;
    }

    depth_sub_ = std::make_unique<message_filters::Subscriber<ImageMsg>>(
      this, depth_topic_, rmw_qos_profile_sensor_data);
    depth_camera_info_sub_ = std::make_unique<message_filters::Subscriber<CameraInfoMsg>>(
      this, camera_info_topic_, rmw_qos_profile_sensor_data);
    depth_sync_ = std::make_unique<message_filters::Synchronizer<ImageInfoPolicy>>(
      ImageInfoPolicy(sync_queue_size_), *depth_sub_, *depth_camera_info_sub_);
    depth_sync_->registerCallback(
      std::bind(
        &RtabMapCppNode::depthCallback, this, std::placeholders::_1, std::placeholders::_2));
  }

  void rgbdCallback(
    const ImageMsg::ConstSharedPtr & rgb_msg,
    const ImageMsg::ConstSharedPtr & depth_msg,
    const CameraInfoMsg::ConstSharedPtr & camera_info_msg)
  {
    try {
      cv::Mat rgb = toColorImage(rgb_msg);
      cv::Mat depth = toDepthImage(depth_msg);
      processFrame(rgb, depth, *camera_info_msg, rgb_msg->header.stamp);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Skipping RGB-D frame: %s", error.what());
    }
  }

  void depthCallback(
    const ImageMsg::ConstSharedPtr & depth_msg,
    const CameraInfoMsg::ConstSharedPtr & camera_info_msg)
  {
    try {
      cv::Mat depth = toDepthImage(depth_msg);
      cv::Mat synthetic_gray(depth.rows, depth.cols, CV_8UC1, cv::Scalar(0));
      processFrame(synthetic_gray, depth, *camera_info_msg, depth_msg->header.stamp);
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Skipping depth frame: %s", error.what());
    }
  }

  void processFrame(
    const cv::Mat & rgb,
    const cv::Mat & depth,
    const CameraInfoMsg & camera_info,
    const rclcpp::Time & stamp)
  {
    ++received_count_;
    rtabmap::CameraModel model = rtabmap_conversions::cameraModelFromROS(camera_info);
    rtabmap::SensorData data(
      rgb, depth, model, received_count_, rtabmap_conversions::timestampFromROS(stamp));

    rtabmap::OdometryInfo odom_info;
    rtabmap::Transform pose = odometry_->process(data, &odom_info);
    if (pose.isNull()) {
      ++lost_odometry_count_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Odometry is lost in %s mode. received=%d mapped=%d lost=%d",
        mode_.c_str(), received_count_, mapped_count_, lost_odometry_count_);
      shutdownIfDone();
      return;
    }

    if (rtabmap_.process(data, pose)) {
      ++mapped_count_;
    }

    if (log_every_n_frames_ > 0 && mapped_count_ % log_every_n_frames_ == 0) {
      RCLCPP_INFO(
        get_logger(), "Mapping update: mode=%s received=%d mapped=%d lost=%d database=%s",
        mode_.c_str(), received_count_, mapped_count_, lost_odometry_count_, database_path_.c_str());
    }

    shutdownIfDone();
  }

  void shutdownIfDone()
  {
    if (max_frames_ > 0 && received_count_ >= max_frames_) {
      RCLCPP_INFO(
        get_logger(),
        "Reached max_frames=%d input frames, mapped=%d, lost=%d. Shutting down after saving database.",
        max_frames_, mapped_count_, lost_odometry_count_);
      rclcpp::shutdown();
    }
  }

  void logStatus()
  {
    if (received_count_ == 0) {
      RCLCPP_WARN(
        get_logger(),
        "No synchronized input received yet. Waiting for mode=%s topics: rgb='%s', depth='%s', "
        "camera_info='%s'. Start the D435i driver first.",
        mode_.c_str(), rgb_topic_.c_str(), depth_topic_.c_str(), camera_info_topic_.c_str());
      return;
    }

    RCLCPP_INFO(
      get_logger(), "Status: mode=%s received=%d mapped=%d lost=%d database=%s",
      mode_.c_str(), received_count_, mapped_count_, lost_odometry_count_, database_path_.c_str());
  }

  std::string mode_;
  std::string rgb_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string database_path_;
  bool delete_db_on_start_ = false;
  int max_frames_ = 0;
  int sync_queue_size_ = 10;
  int log_every_n_frames_ = 30;
  int received_count_ = 0;
  int mapped_count_ = 0;
  int lost_odometry_count_ = 0;

  rtabmap::Rtabmap rtabmap_;
  std::unique_ptr<rtabmap::Odometry> odometry_;

  std::unique_ptr<message_filters::Subscriber<ImageMsg>> rgb_sub_;
  std::unique_ptr<message_filters::Subscriber<ImageMsg>> depth_sub_;
  std::unique_ptr<message_filters::Subscriber<CameraInfoMsg>> rgb_camera_info_sub_;
  std::unique_ptr<message_filters::Subscriber<CameraInfoMsg>> depth_camera_info_sub_;
  std::unique_ptr<message_filters::Synchronizer<RgbdPolicy>> rgbd_sync_;
  std::unique_ptr<message_filters::Synchronizer<ImageInfoPolicy>> depth_sync_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};
}  // namespace luxi_rtab_map

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<luxi_rtab_map::RtabMapCppNode>());
  rclcpp::shutdown();
  return 0;
}
