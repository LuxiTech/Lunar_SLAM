#include "luxi_location/command_gated_odometry.hpp"
#include "luxi_location/initial_pose_fusion.hpp"
#include "luxi_location/depth_projection.hpp"
#include "luxi_location/icp_localizer.hpp"
#include "luxi_location/localization_supervisor.hpp"
#include "luxi_location/map_odom_alignment.hpp"
#include "luxi_location/tracking_pose_gate.hpp"
#include "luxi_location/yaw_motion_predictor.hpp"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <open3d/Open3D.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace luxi_location
{
namespace
{

Eigen::Matrix4d transform_matrix(const geometry_msgs::msg::Transform & transform)
{
  const Eigen::Quaterniond rotation(
    transform.rotation.w, transform.rotation.x,
    transform.rotation.y, transform.rotation.z);
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  matrix.block<3, 3>(0, 0) = rotation.normalized().toRotationMatrix();
  matrix(0, 3) = transform.translation.x;
  matrix(1, 3) = transform.translation.y;
  matrix(2, 3) = transform.translation.z;
  return matrix;
}

sensor_msgs::msg::PointCloud2 point_cloud_message(
  const std::vector<Eigen::Vector3d> & points,
  const std_msgs::msg::Header & header)
{
  sensor_msgs::msg::PointCloud2 message;
  message.header = header;
  message.height = 1;
  message.width = static_cast<std::uint32_t>(points.size());
  message.is_bigendian = false;
  message.is_dense = true;
  message.point_step = 12;
  message.row_step = message.point_step * message.width;
  message.fields.resize(3);
  const std::array<std::string, 3> names{"x", "y", "z"};
  for (std::size_t index = 0; index < names.size(); ++index) {
    message.fields[index].name = names[index];
    message.fields[index].offset = static_cast<std::uint32_t>(index * sizeof(float));
    message.fields[index].datatype = sensor_msgs::msg::PointField::FLOAT32;
    message.fields[index].count = 1;
  }
  message.data.resize(message.row_step);
  for (std::size_t index = 0; index < points.size(); ++index) {
    const std::array<float, 3> value{
      static_cast<float>(points[index].x()),
      static_cast<float>(points[index].y()),
      static_cast<float>(points[index].z())};
    std::memcpy(message.data.data() + index * message.point_step, value.data(), message.point_step);
  }
  return message;
}

sensor_msgs::msg::PointCloud2 point_cloud_message(
  const open3d::geometry::PointCloud & cloud,
  const std_msgs::msg::Header & header)
{
  return point_cloud_message(cloud.points_, header);
}

Eigen::Matrix4d pose_matrix(const geometry_msgs::msg::Pose & pose)
{
  const Eigen::Quaterniond rotation(
    pose.orientation.w, pose.orientation.x,
    pose.orientation.y, pose.orientation.z);
  const double yaw = std::atan2(
    2.0 * (rotation.w() * rotation.z() + rotation.x() * rotation.y()),
    1.0 - 2.0 * (rotation.y() * rotation.y() + rotation.z() * rotation.z()));
  return IcpLocalizer::planar_pose(pose.position.x, pose.position.y, pose.position.z, yaw);
}

geometry_msgs::msg::Pose pose_message(const Eigen::Matrix4d & pose)
{
  geometry_msgs::msg::Pose message;
  message.position.x = pose(0, 3);
  message.position.y = pose(1, 3);
  message.position.z = pose(2, 3);
  const Eigen::Quaterniond rotation(pose.block<3, 3>(0, 0));
  message.orientation.x = rotation.x();
  message.orientation.y = rotation.y();
  message.orientation.z = rotation.z();
  message.orientation.w = rotation.w();
  return message;
}

}  // namespace

class LocalizationNode : public rclcpp::Node
{
public:
  LocalizationNode()
  : Node("luxi_icp_localization"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_),
    tf_broadcaster_(*this)
  {
    declare_parameter("map_path", "");
    declare_parameter("depth_topic", "/sensors/rgbd/depth/image_raw");
    declare_parameter("camera_info_topic", "/sensors/rgbd/color/camera_info");
    declare_parameter("initial_pose_topic", "/initialpose");
    declare_parameter("initial_pose_max_variance", 0.0);
    declare_parameter("map_frame", "map");
    declare_parameter("odom_frame", "odom");
    declare_parameter("base_frame", "base_link");
    declare_parameter("odometry_topic", "/navigation/odom");
    declare_parameter("navigation_active_topic", "/navigation/active");
    declare_parameter("motion_command_topic", "/cmd_vel");
    declare_parameter("motion_command_timeout", 0.50);
    declare_parameter("maximum_odometry_age", 0.25);
    declare_parameter("map_odom_correction_gain", 0.05);
    declare_parameter("maximum_odometry_icp_translation_correction", 0.35);
    declare_parameter("maximum_odometry_icp_yaw_correction_deg", 10.0);
    declare_parameter("map_odom_minimum_correction_fitness", 0.80);
    declare_parameter("map_odom_maximum_correction_rmse", 0.08);
    declare_parameter("map_odom_minimum_correction_static_ratio", 0.80);
    declare_parameter("relocalize_on_tracking_icp_failure", true);
    declare_parameter("relocalization_minimum_static_point_ratio", 0.90);
    declare_parameter("preserve_initial_translation", true);
    declare_parameter("pose_topic", "/luxi_location/pose");
    declare_parameter("map_cloud_topic", "/luxi_location/map_cloud");
    declare_parameter("aligned_cloud_topic", "/luxi_location/aligned_cloud");
    declare_parameter("status_topic", "/luxi_location/status");
    declare_parameter("health_topic", "/luxi_location/health");
    declare_parameter("fitness_topic", "/luxi_location/fitness");
    declare_parameter("imu_topic", "/d15041873/imu_sensor_broadcaster/imu");
    declare_parameter("use_imu_yaw_prediction", true);
    declare_parameter("maximum_imu_yaw_prediction_deg", 40.0);
    declare_parameter("publish_tf", true);
    declare_parameter("processing_period", 0.5);
    declare_parameter("pixel_stride", 4);
    declare_parameter("depth_scale", 0.001);
    declare_parameter("minimum_depth", 0.25);
    declare_parameter("maximum_depth", 4.0);
    declare_parameter("map_voxel_size", 0.08);
    declare_parameter("scan_voxel_size", 0.06);
    declare_parameter("coarse_voxel_size", 0.16);
    declare_parameter("coarse_max_correspondence_distance", 0.50);
    declare_parameter("fine_max_correspondence_distance", 0.20);
    declare_parameter("coarse_iterations", 30);
    declare_parameter("fine_iterations", 30);
    declare_parameter("minimum_scan_points", 150);
    declare_parameter("minimum_fitness", 0.25);
    declare_parameter("maximum_rmse", 0.15);
    declare_parameter("maximum_translation_correction", 0.50);
    declare_parameter("maximum_yaw_correction_deg", 20.0);
    declare_parameter("initial_maximum_translation_correction", 1.50);
    declare_parameter("initial_maximum_yaw_correction_deg", 45.0);
    declare_parameter("tracking_static_filter_distance", 0.30);
    declare_parameter("tracking_minimum_static_point_ratio", 0.20);
    declare_parameter("hloc_consistent_pose_count", 3);
    declare_parameter("hloc_maximum_translation_difference", 0.50);
    declare_parameter("hloc_maximum_yaw_difference_deg", 20.0);
    declare_parameter("icp_failures_before_relocalization", 5);
    declare_parameter("tracking_maximum_linear_speed", 0.20);
    declare_parameter("tracking_maximum_angular_speed", 0.70);
    declare_parameter("tracking_translation_margin", 0.08);
    declare_parameter("tracking_yaw_margin_deg", 8.0);
    declare_parameter("tracking_maximum_interval", 2.0);
    declare_parameter("relocalization_maximum_translation", 0.40);
    declare_parameter("relocalization_maximum_yaw_deg", 30.0);
    declare_parameter("hloc_enable_service", "/luxi_hloc_localizer/enable");
    declare_parameter(
      "relocalization_request_topic", "/luxi_location/relocalization_request");

    map_frame_ = get_parameter("map_frame").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    publish_tf_ = get_parameter("publish_tf").as_bool();
    processing_period_ = get_parameter("processing_period").as_double();
    pixel_stride_ = get_parameter("pixel_stride").as_int();
    depth_scale_ = get_parameter("depth_scale").as_double();
    minimum_depth_ = get_parameter("minimum_depth").as_double();
    maximum_depth_ = get_parameter("maximum_depth").as_double();
    use_imu_yaw_prediction_ = get_parameter("use_imu_yaw_prediction").as_bool();
    relocalize_on_tracking_icp_failure_ =
      get_parameter("relocalize_on_tracking_icp_failure").as_bool();
    relocalization_minimum_static_point_ratio_ =
      get_parameter("relocalization_minimum_static_point_ratio").as_double();
    minimum_fitness_ = get_parameter("minimum_fitness").as_double();
    preserve_initial_translation_ =
      get_parameter("preserve_initial_translation").as_bool();
    maximum_odometry_age_ = get_parameter("maximum_odometry_age").as_double();
    map_odom_minimum_correction_fitness_ =
      get_parameter("map_odom_minimum_correction_fitness").as_double();
    map_odom_maximum_correction_rmse_ =
      get_parameter("map_odom_maximum_correction_rmse").as_double();
    map_odom_minimum_correction_static_ratio_ =
      get_parameter("map_odom_minimum_correction_static_ratio").as_double();
    motion_command_timeout_ = get_parameter("motion_command_timeout").as_double();
    initial_pose_max_variance_ =
      get_parameter("initial_pose_max_variance").as_double();
    if (processing_period_ <= 0.0 || pixel_stride_ <= 0 ||
      minimum_depth_ <= 0.0 || maximum_depth_ <= minimum_depth_ ||
      maximum_odometry_age_ <= 0.0 || motion_command_timeout_ <= 0.0 ||
      relocalization_minimum_static_point_ratio_ < 0.0 ||
      relocalization_minimum_static_point_ratio_ > 1.0 ||
      map_odom_minimum_correction_fitness_ < 0.0 ||
      map_odom_minimum_correction_fitness_ > 1.0 ||
      map_odom_maximum_correction_rmse_ <= 0.0 ||
      map_odom_minimum_correction_static_ratio_ < 0.0 ||
      map_odom_minimum_correction_static_ratio_ > 1.0)
    {
      throw std::invalid_argument("depth and processing parameters are invalid");
    }

    IcpParameters parameters;
    parameters.map_voxel_size = get_parameter("map_voxel_size").as_double();
    parameters.scan_voxel_size = get_parameter("scan_voxel_size").as_double();
    parameters.coarse_voxel_size = get_parameter("coarse_voxel_size").as_double();
    parameters.coarse_max_correspondence_distance =
      get_parameter("coarse_max_correspondence_distance").as_double();
    parameters.fine_max_correspondence_distance =
      get_parameter("fine_max_correspondence_distance").as_double();
    parameters.coarse_iterations = get_parameter("coarse_iterations").as_int();
    parameters.fine_iterations = get_parameter("fine_iterations").as_int();
    parameters.minimum_scan_points = get_parameter("minimum_scan_points").as_int();
    parameters.minimum_fitness = get_parameter("minimum_fitness").as_double();
    parameters.maximum_rmse = get_parameter("maximum_rmse").as_double();
    parameters.maximum_translation_correction =
      get_parameter("maximum_translation_correction").as_double();
    parameters.maximum_yaw_correction =
      get_parameter("maximum_yaw_correction_deg").as_double() * M_PI / 180.0;
    parameters.initial_maximum_translation_correction =
      get_parameter("initial_maximum_translation_correction").as_double();
    parameters.initial_maximum_yaw_correction =
      get_parameter("initial_maximum_yaw_correction_deg").as_double() * M_PI / 180.0;
    parameters.tracking_static_filter_distance =
      get_parameter("tracking_static_filter_distance").as_double();
    parameters.tracking_minimum_static_point_ratio =
      get_parameter("tracking_minimum_static_point_ratio").as_double();
    localizer_ = std::make_unique<IcpLocalizer>(parameters);

    LocalizationSupervisorParameters supervisor_parameters;
    supervisor_parameters.consistent_pose_count =
      get_parameter("hloc_consistent_pose_count").as_int();
    supervisor_parameters.maximum_translation_difference =
      get_parameter("hloc_maximum_translation_difference").as_double();
    supervisor_parameters.maximum_yaw_difference =
      get_parameter("hloc_maximum_yaw_difference_deg").as_double() * M_PI / 180.0;
    supervisor_parameters.failures_before_relocalization =
      get_parameter("icp_failures_before_relocalization").as_int();
    consistent_pose_count_required_ = supervisor_parameters.consistent_pose_count;
    failures_before_relocalization_ =
      supervisor_parameters.failures_before_relocalization;
    supervisor_ = std::make_unique<LocalizationSupervisor>(supervisor_parameters);

    TrackingPoseGateParameters tracking_parameters;
    tracking_parameters.maximum_linear_speed =
      get_parameter("tracking_maximum_linear_speed").as_double();
    tracking_parameters.maximum_angular_speed =
      get_parameter("tracking_maximum_angular_speed").as_double();
    tracking_parameters.translation_margin =
      get_parameter("tracking_translation_margin").as_double();
    tracking_parameters.yaw_margin =
      get_parameter("tracking_yaw_margin_deg").as_double() * M_PI / 180.0;
    tracking_parameters.maximum_interval =
      get_parameter("tracking_maximum_interval").as_double();
    tracking_parameters.maximum_relocalization_translation =
      get_parameter("relocalization_maximum_translation").as_double();
    tracking_parameters.maximum_relocalization_yaw =
      get_parameter("relocalization_maximum_yaw_deg").as_double() * M_PI / 180.0;
    tracking_pose_gate_ = std::make_unique<TrackingPoseGate>(tracking_parameters);
    yaw_motion_predictor_ = std::make_unique<YawMotionPredictor>(
      get_parameter("maximum_imu_yaw_prediction_deg").as_double() * M_PI / 180.0);
    map_odom_alignment_ = std::make_unique<MapOdomAlignment>(
      get_parameter("map_odom_correction_gain").as_double(),
      get_parameter("maximum_odometry_icp_translation_correction").as_double(),
      get_parameter("maximum_odometry_icp_yaw_correction_deg").as_double() * M_PI / 180.0);

    const std::string map_path = get_parameter("map_path").as_string();
    std::string error;
    if (map_path.empty() || !localizer_->load_map(map_path, error)) {
      throw std::runtime_error(map_path.empty() ? "map_path must not be empty" : error);
    }

    const auto sensor_qos = rclcpp::SensorDataQoS();
    camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      get_parameter("camera_info_topic").as_string(), sensor_qos,
      std::bind(&LocalizationNode::camera_info_callback, this, std::placeholders::_1));
    depth_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      get_parameter("depth_topic").as_string(), sensor_qos,
      std::bind(&LocalizationNode::depth_callback, this, std::placeholders::_1));
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      get_parameter("imu_topic").as_string(), sensor_qos,
      std::bind(&LocalizationNode::imu_callback, this, std::placeholders::_1));
    odometry_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("odometry_topic").as_string(), sensor_qos,
      std::bind(&LocalizationNode::odometry_callback, this, std::placeholders::_1));
    navigation_active_subscription_ = create_subscription<std_msgs::msg::Bool>(
      get_parameter("navigation_active_topic").as_string(),
      rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&LocalizationNode::navigation_active_callback, this, std::placeholders::_1));
    motion_command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      get_parameter("motion_command_topic").as_string(), 10,
      std::bind(&LocalizationNode::motion_command_callback, this, std::placeholders::_1));
    initial_pose_subscription_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      get_parameter("initial_pose_topic").as_string(), 10,
      std::bind(&LocalizationNode::initial_pose_callback, this, std::placeholders::_1));
    relocalization_request_subscription_ = create_subscription<std_msgs::msg::Bool>(
      get_parameter("relocalization_request_topic").as_string(), 10,
      std::bind(
        &LocalizationNode::relocalization_request_callback, this,
        std::placeholders::_1));
    hloc_enable_client_ = create_client<std_srvs::srv::SetBool>(
      get_parameter("hloc_enable_service").as_string());
    hloc_control_timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&LocalizationNode::synchronize_hloc_control, this));

    pose_publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      get_parameter("pose_topic").as_string(), 10);
    aligned_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      get_parameter("aligned_cloud_topic").as_string(), rclcpp::SensorDataQoS());
    status_publisher_ = create_publisher<std_msgs::msg::String>(
      get_parameter("status_topic").as_string(), 10);
    health_publisher_ = create_publisher<std_msgs::msg::String>(
      get_parameter("health_topic").as_string(),
      rclcpp::QoS(1).reliable().transient_local());
    fitness_publisher_ = create_publisher<std_msgs::msg::Float32>(
      get_parameter("fitness_topic").as_string(), 10);
    const auto map_qos = rclcpp::QoS(1).reliable().transient_local();
    map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      get_parameter("map_cloud_topic").as_string(), map_qos);

    publish_map();
    publish_status(
      "waiting for " + std::to_string(consistent_pose_count_required_) +
      " consistent HLoc poses");
    publish_health("searching");
    RCLCPP_INFO(
      get_logger(),
      "loaded %zu map points from %s; waiting for %d consistent HLoc poses",
      localizer_->map().points_.size(), map_path.c_str(), consistent_pose_count_required_);
  }

private:
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr message)
  {
    CameraIntrinsics intrinsics;
    intrinsics.width = static_cast<int>(message->width);
    intrinsics.height = static_cast<int>(message->height);
    intrinsics.fx = message->k[0];
    intrinsics.fy = message->k[4];
    intrinsics.cx = message->k[2];
    intrinsics.cy = message->k[5];
    std::lock_guard<std::mutex> lock(mutex_);
    intrinsics_ = intrinsics;
    has_intrinsics_ = true;
  }

  void initial_pose_callback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr message)
  {
    if (!message->header.frame_id.empty() && message->header.frame_id != map_frame_) {
      publish_status("rejected initial pose: frame must be " + map_frame_);
      return;
    }
    if (initial_pose_max_variance_ > 0.0) {
      const auto & covariance = message->pose.covariance;
      const std::array<double, 3> variances{
        covariance[0], covariance[7], covariance[35]};
      if (std::any_of(
          variances.begin(), variances.end(),
          [this](const double value) {
            return !std::isfinite(value) || value < 0.0 ||
                   value > initial_pose_max_variance_;
          }))
      {
        publish_status("rejected initial pose: covariance is not confident");
        return;
      }
    }
    const Eigen::Matrix4d coarse_pose = pose_matrix(message->pose.pose);
    std::optional<Eigen::Matrix4d> accepted_pose;
    int consistent_pose_count = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      accepted_pose = supervisor_->add_coarse_pose(coarse_pose);
      consistent_pose_count = supervisor_->consistent_pose_count();
      if (accepted_pose.has_value()) {
        pending_relocalization_pose_ = *accepted_pose;
        if (!recovering_with_odometry_) {
          pose_ = *accepted_pose;
        }
        has_pose_ = true;
        initial_alignment_ = true;
        yaw_motion_predictor_->reset();
      }
    }
    if (accepted_pose.has_value()) {
      publish_health("verifying");
      publish_status(
        "HLoc consistency " + std::to_string(consistent_pose_count_required_) + "/" +
        std::to_string(consistent_pose_count_required_) + "; waiting for ICP");
    } else if (consistent_pose_count > 0) {
      publish_status(
        "HLoc consistency " + std::to_string(consistent_pose_count) + "/" +
        std::to_string(consistent_pose_count_required_));
    }
  }

  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr message)
  {
    if (!use_imu_yaw_prediction_) {
      return;
    }
    const Eigen::Quaterniond orientation(
      message->orientation.w, message->orientation.x,
      message->orientation.y, message->orientation.z);
    if (!std::isfinite(orientation.norm()) || orientation.norm() < 1e-6) {
      return;
    }
    const Eigen::Matrix3d rotation = orientation.normalized().toRotationMatrix();
    const double yaw = std::atan2(rotation(1, 0), rotation(0, 0));
    std::lock_guard<std::mutex> lock(mutex_);
    yaw_motion_predictor_->update(yaw);
  }

  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    if (!message->header.frame_id.empty() && message->header.frame_id != odom_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring odometry in frame %s; expected %s",
        message->header.frame_id.c_str(), odom_frame_.c_str());
      return;
    }
    if (!message->child_frame_id.empty() && message->child_frame_id != base_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring odometry child frame %s; expected %s",
        message->child_frame_id.c_str(), base_frame_.c_str());
      return;
    }

    const Eigen::Matrix4d raw_odom_from_base = pose_matrix(message->pose.pose);
    Eigen::Matrix4d map_from_base;
    Eigen::Matrix4d map_from_odom;
    double variance = 0.01;
    bool publish = false;
    std::string recovery_health;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const bool motion_active = motion_active_locked();
      const Eigen::Matrix4d odom_from_base =
        command_gated_odometry_.update(raw_odom_from_base, motion_active);
      odom_from_base_ = odom_from_base;
      odometry_stamp_seconds_ =
        static_cast<double>(message->header.stamp.sec) +
        static_cast<double>(message->header.stamp.nanosec) * 1e-9;
      has_odometry_ = true;
      if (has_pose_ && map_odom_alignment_->initialized()) {
        map_from_base = motion_active ?
          map_odom_alignment_->predict(odom_from_base_) : stationary_map_pose_;
        map_from_odom = map_odom_alignment_->map_from_odom();
        pose_ = map_from_base;
        variance = last_pose_variance_;
        publish = true;
        if (recovering_with_odometry_) {
          recovery_health = pending_relocalization_pose_.has_value() ?
            "verifying" : "dead_reckoning";
        } else {
          // ICP is intentionally slower than the navigation health timeout.
          // Refresh the validated tracking state with every odometry pose so a
          // healthy localization is not declared stale between ICP updates.
          recovery_health = "tracking";
        }
      }
    }
    if (publish) {
      publish_pose_and_transform(
        map_from_base, map_from_odom, message->header.stamp, variance);
      if (!recovery_health.empty()) {
        publish_health(recovery_health);
      }
    }
  }

  void navigation_active_callback(const std_msgs::msg::Bool::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (navigation_active_ && !message->data && has_pose_) {
      stationary_map_pose_ = pose_;
    }
    navigation_active_ = message->data;
  }

  void relocalization_request_callback(const std_msgs::msg::Bool::SharedPtr message)
  {
    if (!message->data) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      supervisor_->force_relocalization();
      recovering_with_odometry_ = map_odom_alignment_->initialized() && has_pose_;
      if (!recovering_with_odometry_) {
        has_pose_ = false;
      }
      initial_alignment_ = true;
      pending_relocalization_pose_.reset();
      yaw_motion_predictor_->reset();
    }
    request_hloc_enabled(true);
    publish_health("searching");
    publish_status("explicit navigation recovery requested; searching with HLoc");
    RCLCPP_WARN(
      get_logger(),
      "Navigation requested global relocalization; HLoc search restarted while odometry is retained");
  }

  void motion_command_callback(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    constexpr double epsilon = 1e-4;
    const bool commanded =
      std::abs(message->linear.x) > epsilon ||
      std::abs(message->linear.y) > epsilon ||
      std::abs(message->angular.z) > epsilon;
    std::lock_guard<std::mutex> lock(mutex_);
    if (robot_motion_commanded_ && !commanded && has_pose_) {
      stationary_map_pose_ = pose_;
    }
    robot_motion_commanded_ = commanded;
    motion_command_received_at_ = std::chrono::steady_clock::now();
  }

  bool motion_active_locked() const
  {
    if (navigation_active_) {
      return true;
    }
    if (!robot_motion_commanded_ ||
      motion_command_received_at_.time_since_epoch().count() == 0)
    {
      return false;
    }
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now() - motion_command_received_at_).count() <=
      motion_command_timeout_;
  }

  void depth_callback(const sensor_msgs::msg::Image::SharedPtr message)
  {
    const auto now = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!has_pose_ || !has_intrinsics_) {
        return;
      }
      if (last_processing_time_.time_since_epoch().count() != 0 &&
        std::chrono::duration<double>(now - last_processing_time_).count() < processing_period_)
      {
        return;
      }
      last_processing_time_ = now;
    }

    geometry_msgs::msg::TransformStamped base_from_camera;
    try {
      base_from_camera = tf_buffer_.lookupTransform(
        base_frame_, message->header.frame_id, message->header.stamp,
        rclcpp::Duration::from_seconds(0.1));
    } catch (const tf2::TransformException & exception) {
      publish_status(std::string("waiting for camera transform: ") + exception.what());
      return;
    }

    CameraIntrinsics intrinsics;
    Eigen::Matrix4d initial_pose;
    Eigen::Matrix4d odom_from_base;
    bool initial_alignment = false;
    const double image_stamp_seconds =
      static_cast<double>(message->header.stamp.sec) +
      static_cast<double>(message->header.stamp.nanosec) * 1e-9;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!has_odometry_) {
        publish_status("waiting for continuous RGB-D odometry");
        return;
      }
      if (std::abs(image_stamp_seconds - odometry_stamp_seconds_) > maximum_odometry_age_) {
        publish_status("waiting for time-aligned RGB-D odometry");
        return;
      }
      intrinsics = intrinsics_;
      odom_from_base = odom_from_base_;
      initial_alignment = initial_alignment_;
      if (initial_alignment) {
        if (!pending_relocalization_pose_.has_value()) {
          publish_status(
            recovering_with_odometry_ ?
            "dead reckoning while waiting for a consistent HLoc pose" :
            "waiting for a consistent HLoc pose");
          return;
        }
        initial_pose = *pending_relocalization_pose_;
      } else if (map_odom_alignment_->initialized()) {
        initial_pose = map_odom_alignment_->predict(odom_from_base);
      } else {
        publish_status("waiting for map-to-odometry initialization");
        return;
      }
    }
    if (intrinsics.width != static_cast<int>(message->width) ||
      intrinsics.height != static_cast<int>(message->height))
    {
      publish_status("depth image and camera info dimensions differ");
      return;
    }

    std::string error;
    std::vector<Eigen::Vector3d> points;
    const Eigen::Matrix4d base_from_camera_matrix = transform_matrix(base_from_camera.transform);
    if (message->encoding == "16UC1" || message->encoding == "mono16") {
      points = project_depth_u16(
        message->data.data(), message->data.size(), message->step,
        intrinsics, base_from_camera_matrix, pixel_stride_, depth_scale_,
        minimum_depth_, maximum_depth_, error);
    } else if (message->encoding == "32FC1") {
      points = project_depth_f32(
        message->data.data(), message->data.size(), message->step,
        intrinsics, base_from_camera_matrix, pixel_stride_,
        minimum_depth_, maximum_depth_, error);
    } else {
      error = "unsupported depth encoding: " + message->encoding;
    }
    if (!error.empty()) {
      publish_status(error);
      return;
    }

    open3d::geometry::PointCloud scan;
    scan.points_ = points;
    const IcpResult result = localizer_->register_scan(scan, initial_pose, initial_alignment);
    publish_result(
      result, scan, message->header.stamp, odom_from_base, initial_alignment);
  }

  void publish_result(
    const IcpResult & result,
    const open3d::geometry::PointCloud & scan,
    const builtin_interfaces::msg::Time & stamp,
    const Eigen::Matrix4d & odom_from_base,
    const bool initial_alignment)
  {
    const double stamp_seconds =
      static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
    bool accepted = result.accepted;
    std::string reason = result.reason;
    Eigen::Matrix4d fused_pose = result.pose;
    Eigen::Matrix4d map_from_odom = Eigen::Matrix4d::Identity();
    MapOdomCorrection odometry_correction;
    HlocAction hloc_action = HlocAction::kNone;
    int consecutive_failures = 0;
    bool keep_odometry_tracking = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (accepted) {
        if (initial_alignment) {
          fused_pose = initialPoseForMapAlignment(
            pending_relocalization_pose_.value_or(pose_), result.pose,
            preserve_initial_translation_);
          const auto decision = map_odom_alignment_->initialized() ?
            tracking_pose_gate_->evaluate_relocalization(
              fused_pose, map_odom_alignment_->predict(odom_from_base), stamp_seconds) :
            tracking_pose_gate_->evaluate_relocalization(fused_pose, stamp_seconds);
          accepted = decision.accepted;
          if (!accepted) {
            reason = decision.reason;
          } else {
            map_odom_alignment_->initialize(fused_pose, odom_from_base);
            fused_pose = map_odom_alignment_->predict(odom_from_base);
            stationary_map_pose_ = fused_pose;
          }
        } else {
          const bool correction_geometry_ready =
            result.fitness >= map_odom_minimum_correction_fitness_ &&
            std::isfinite(result.rmse) &&
            result.rmse <= map_odom_maximum_correction_rmse_ &&
            result.static_point_ratio >= map_odom_minimum_correction_static_ratio_;
          odometry_correction = map_odom_alignment_->correct(
            result.pose, odom_from_base, correction_geometry_ready);
          accepted = odometry_correction.accepted;
          if (accepted) {
            fused_pose = map_odom_alignment_->predict(odom_from_base);
            if (!motion_active_locked()) {
              // Freeze raw visual odometry while stopped, but retain a
              // high-confidence scan-to-map correction. Otherwise a correction
              // accumulated while stationary appears as a jump at motion start.
              stationary_map_pose_ = fused_pose;
            }
            const auto decision = tracking_pose_gate_->evaluate(fused_pose, stamp_seconds);
            accepted = decision.accepted;
            if (!accepted) {
              reason = decision.reason;
            }
          } else {
            reason = odometry_correction.reason;
          }
        }
      }
      keep_odometry_tracking =
        should_retain_odometry_after_rejected_icp(
        initial_alignment, accepted, relocalize_on_tracking_icp_failure_,
        map_odom_alignment_->initialized(), result.fitness, minimum_fitness_,
        result.static_point_ratio, relocalization_minimum_static_point_ratio_);
      hloc_action = supervisor_->report_icp_result(
        keep_odometry_tracking ? true : accepted);
      consecutive_failures = supervisor_->consecutive_icp_failures();
      if (accepted) {
        pose_ = fused_pose;
        initial_alignment_ = false;
        map_from_odom = map_odom_alignment_->map_from_odom();
        last_pose_variance_ = std::max(result.rmse * result.rmse, 1e-4);
        yaw_motion_predictor_->anchor();
        pending_relocalization_pose_.reset();
        recovering_with_odometry_ = false;
      } else if (hloc_action == HlocAction::kEnable) {
        recovering_with_odometry_ = map_odom_alignment_->initialized() && has_pose_;
        if (!recovering_with_odometry_) {
          has_pose_ = false;
        }
        initial_alignment_ = true;
        pending_relocalization_pose_.reset();
        yaw_motion_predictor_->reset();
      }
      if (keep_odometry_tracking) {
        reason += "; retaining continuous odometry pose";
      }
    }
    std::ostringstream status;
    status << reason << " fitness=" << result.fitness << " rmse=" << result.rmse
           << " static_ratio=" << result.static_point_ratio
           << " correction=" << result.translation_correction << "m/"
           << result.yaw_correction * 180.0 / M_PI << "deg";
    if (!initial_alignment && odometry_correction.reason.size() > 0U) {
      status << "; odom_residual=" << odometry_correction.translation_residual << "m/"
             << odometry_correction.yaw_residual * 180.0 / M_PI << "deg"
             << "; map_correction="
             << (odometry_correction.applied ? "applied" : "held");
    }
    if (hloc_action == HlocAction::kDisable) {
      request_hloc_enabled(false);
      status << "; HLoc pose injection disabled";
    } else if (hloc_action == HlocAction::kEnable) {
      request_hloc_enabled(true);
      status << "; " << failures_before_relocalization_
             << " consecutive ICP failures, restarting HLoc";
    }
    // Consumers use this topic as an accepted-localization signal. Keep the
    // rejected raw overlap in the status text, but never certify it as a pose.
    std_msgs::msg::Float32 fitness;
    fitness.data = accepted ? static_cast<float>(result.fitness) : 0.0F;
    fitness_publisher_->publish(fitness);
    if (!accepted) {
      publish_health(
        keep_odometry_tracking ? "tracking" :
        (hloc_action == HlocAction::kEnable ? "searching" :
        (initial_alignment ? "verifying" : "degraded")));
      if (keep_odometry_tracking) {
        status << "; local odometry remains authoritative";
      } else if (hloc_action != HlocAction::kEnable) {
        status << "; consecutive failures=" << consecutive_failures << "/"
               << failures_before_relocalization_;
      }
      publish_status(status.str());
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Localization rejected: %s", status.str().c_str());
      return;
    }
    const double variance = std::max(result.rmse * result.rmse, 1e-4);
    publish_pose_and_transform(fused_pose, map_from_odom, stamp, variance);
    publish_health("tracking");

    open3d::geometry::PointCloud aligned = scan;
    aligned.Transform(fused_pose);
    std_msgs::msg::Header cloud_header;
    cloud_header.stamp = stamp;
    cloud_header.frame_id = map_frame_;
    aligned_cloud_publisher_->publish(point_cloud_message(aligned, cloud_header));
    publish_status(status.str());
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000, "Localization tracking: %s", status.str().c_str());
  }

  void publish_pose_and_transform(
    const Eigen::Matrix4d & map_from_base,
    const Eigen::Matrix4d & map_from_odom,
    const builtin_interfaces::msg::Time & stamp,
    const double variance)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = map_frame_;
    pose.pose.pose = pose_message(map_from_base);
    pose.pose.covariance.fill(0.0);
    pose.pose.covariance[0] = variance;
    pose.pose.covariance[7] = variance;
    pose.pose.covariance[14] = 0.01;
    pose.pose.covariance[21] = 0.01;
    pose.pose.covariance[28] = 0.01;
    pose.pose.covariance[35] = variance;
    pose_publisher_->publish(pose);

    if (!publish_tf_) {
      return;
    }
    geometry_msgs::msg::TransformStamped transform;
    transform.header = pose.header;
    transform.child_frame_id = odom_frame_;
    transform.transform.translation.x = map_from_odom(0, 3);
    transform.transform.translation.y = map_from_odom(1, 3);
    transform.transform.translation.z = map_from_odom(2, 3);
    const Eigen::Quaterniond rotation(map_from_odom.block<3, 3>(0, 0));
    transform.transform.rotation.x = rotation.x();
    transform.transform.rotation.y = rotation.y();
    transform.transform.rotation.z = rotation.z();
    transform.transform.rotation.w = rotation.w();
    tf_broadcaster_.sendTransform(transform);
  }

  void publish_map()
  {
    std_msgs::msg::Header header;
    header.stamp = now();
    header.frame_id = map_frame_;
    map_publisher_->publish(point_cloud_message(localizer_->map(), header));
  }

  void publish_status(const std::string & text)
  {
    std_msgs::msg::String message;
    message.data = text;
    status_publisher_->publish(message);
  }

  void publish_health(const std::string & state)
  {
    std_msgs::msg::String message;
    message.data = state;
    health_publisher_->publish(message);
  }

  void request_hloc_enabled(bool enabled)
  {
    desired_hloc_enabled_ = enabled;
    hloc_control_synchronized_ = false;
    synchronize_hloc_control();
  }

  void synchronize_hloc_control()
  {
    if (!desired_hloc_enabled_.has_value() || hloc_request_in_flight_ ||
      hloc_control_synchronized_ || !hloc_enable_client_->service_is_ready())
    {
      return;
    }
    const bool requested_state = *desired_hloc_enabled_;
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = requested_state;
    hloc_request_in_flight_ = true;
    hloc_enable_client_->async_send_request(
      request,
      [this, requested_state](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
        hloc_request_in_flight_ = false;
        const auto response = future.get();
        if (!response->success) {
          RCLCPP_WARN(
            get_logger(), "HLoc enable service rejected request: %s",
            response->message.c_str());
          return;
        }
        if (desired_hloc_enabled_.has_value() &&
          *desired_hloc_enabled_ == requested_state)
        {
          hloc_control_synchronized_ = true;
          RCLCPP_INFO(
            get_logger(), "HLoc inference %s", requested_state ? "enabled" : "disabled");
        }
      });
  }

  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_{true};
  double processing_period_{0.5};
  int pixel_stride_{4};
  double depth_scale_{0.001};
  double minimum_depth_{0.25};
  double maximum_depth_{4.0};
  double maximum_odometry_age_{0.25};
  double map_odom_minimum_correction_fitness_{0.80};
  double map_odom_maximum_correction_rmse_{0.08};
  double map_odom_minimum_correction_static_ratio_{0.80};
  double motion_command_timeout_{0.50};
  double initial_pose_max_variance_{0.0};
  bool use_imu_yaw_prediction_{true};
  bool relocalize_on_tracking_icp_failure_{true};
  double relocalization_minimum_static_point_ratio_{0.90};
  double minimum_fitness_{0.25};
  bool preserve_initial_translation_{true};
  int consistent_pose_count_required_{3};
  int failures_before_relocalization_{5};

  std::mutex mutex_;
  CameraIntrinsics intrinsics_;
  bool has_intrinsics_{false};
  Eigen::Matrix4d pose_{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d odom_from_base_{Eigen::Matrix4d::Identity()};
  Eigen::Matrix4d stationary_map_pose_{Eigen::Matrix4d::Identity()};
  double odometry_stamp_seconds_{0.0};
  double last_pose_variance_{0.01};
  bool has_odometry_{false};
  bool navigation_active_{false};
  bool robot_motion_commanded_{false};
  std::chrono::steady_clock::time_point motion_command_received_at_;
  CommandGatedOdometry command_gated_odometry_;
  bool has_pose_{false};
  bool initial_alignment_{true};
  bool recovering_with_odometry_{false};
  std::optional<Eigen::Matrix4d> pending_relocalization_pose_;
  std::chrono::steady_clock::time_point last_processing_time_;
  std::unique_ptr<IcpLocalizer> localizer_;
  std::unique_ptr<LocalizationSupervisor> supervisor_;
  std::unique_ptr<MapOdomAlignment> map_odom_alignment_;
  std::unique_ptr<TrackingPoseGate> tracking_pose_gate_;
  std::unique_ptr<YawMotionPredictor> yaw_motion_predictor_;
  std::optional<bool> desired_hloc_enabled_;
  bool hloc_request_in_flight_{false};
  bool hloc_control_synchronized_{false};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr navigation_active_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr motion_command_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
    relocalization_request_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_cloud_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr health_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr fitness_publisher_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr hloc_enable_client_;
  rclcpp::TimerBase::SharedPtr hloc_control_timer_;
};

}  // namespace luxi_location

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<luxi_location::LocalizationNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("luxi_icp_localization"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
