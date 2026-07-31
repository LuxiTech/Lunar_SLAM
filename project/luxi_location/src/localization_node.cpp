#include "luxi_location/depth_projection.hpp"
#include "luxi_location/icp_localizer.hpp"
#include "luxi_location/localization_supervisor.hpp"

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <open3d/Open3D.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
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
    declare_parameter("base_frame", "base_link");
    declare_parameter("pose_topic", "/luxi_location/pose");
    declare_parameter("map_cloud_topic", "/luxi_location/map_cloud");
    declare_parameter("aligned_cloud_topic", "/luxi_location/aligned_cloud");
    declare_parameter("status_topic", "/luxi_location/status");
    declare_parameter("fitness_topic", "/luxi_location/fitness");
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
    declare_parameter("hloc_consistent_pose_count", 3);
    declare_parameter("hloc_maximum_translation_difference", 0.50);
    declare_parameter("hloc_maximum_yaw_difference_deg", 20.0);
    declare_parameter("icp_failures_before_relocalization", 5);
    declare_parameter("hloc_enable_service", "/luxi_hloc_localizer/enable");

    map_frame_ = get_parameter("map_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    publish_tf_ = get_parameter("publish_tf").as_bool();
    processing_period_ = get_parameter("processing_period").as_double();
    pixel_stride_ = get_parameter("pixel_stride").as_int();
    depth_scale_ = get_parameter("depth_scale").as_double();
    minimum_depth_ = get_parameter("minimum_depth").as_double();
    maximum_depth_ = get_parameter("maximum_depth").as_double();
    initial_pose_max_variance_ =
      get_parameter("initial_pose_max_variance").as_double();
    if (processing_period_ <= 0.0 || pixel_stride_ <= 0 ||
      minimum_depth_ <= 0.0 || maximum_depth_ <= minimum_depth_)
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
    initial_pose_subscription_ =
      create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      get_parameter("initial_pose_topic").as_string(), 10,
      std::bind(&LocalizationNode::initial_pose_callback, this, std::placeholders::_1));
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
    fitness_publisher_ = create_publisher<std_msgs::msg::Float32>(
      get_parameter("fitness_topic").as_string(), 10);
    const auto map_qos = rclcpp::QoS(1).reliable().transient_local();
    map_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      get_parameter("map_cloud_topic").as_string(), map_qos);

    publish_map();
    publish_status(
      "waiting for " + std::to_string(consistent_pose_count_required_) +
      " consistent HLoc poses");
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
        pose_ = *accepted_pose;
        has_pose_ = true;
        initial_alignment_ = true;
      }
    }
    if (accepted_pose.has_value()) {
      publish_status(
        "HLoc consistency " + std::to_string(consistent_pose_count_required_) + "/" +
        std::to_string(consistent_pose_count_required_) + "; waiting for ICP");
    } else if (consistent_pose_count > 0) {
      publish_status(
        "HLoc consistency " + std::to_string(consistent_pose_count) + "/" +
        std::to_string(consistent_pose_count_required_));
    }
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
    bool initial_alignment = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      intrinsics = intrinsics_;
      initial_pose = pose_;
      initial_alignment = initial_alignment_;
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
    publish_result(result, scan, message->header.stamp);
  }

  void publish_result(
    const IcpResult & result,
    const open3d::geometry::PointCloud & scan,
    const builtin_interfaces::msg::Time & stamp)
  {
    std_msgs::msg::Float32 fitness;
    fitness.data = static_cast<float>(result.fitness);
    fitness_publisher_->publish(fitness);

    std::ostringstream status;
    status << result.reason << " fitness=" << result.fitness << " rmse=" << result.rmse;
    HlocAction hloc_action = HlocAction::kNone;
    int consecutive_failures = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      hloc_action = supervisor_->report_icp_result(result.accepted);
      consecutive_failures = supervisor_->consecutive_icp_failures();
      if (result.accepted) {
        pose_ = result.pose;
        initial_alignment_ = false;
      } else if (hloc_action == HlocAction::kEnable) {
        has_pose_ = false;
        initial_alignment_ = true;
      }
    }
    if (hloc_action == HlocAction::kDisable) {
      request_hloc_enabled(false);
      status << "; HLoc pose injection disabled";
    } else if (hloc_action == HlocAction::kEnable) {
      request_hloc_enabled(true);
      status << "; " << failures_before_relocalization_
             << " consecutive ICP failures, restarting HLoc";
    }
    if (!result.accepted) {
      if (hloc_action != HlocAction::kEnable) {
        status << "; consecutive failures=" << consecutive_failures << "/"
               << failures_before_relocalization_;
      }
      publish_status(status.str());
      return;
    }
    geometry_msgs::msg::PoseWithCovarianceStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = map_frame_;
    pose.pose.pose = pose_message(result.pose);
    pose.pose.covariance.fill(0.0);
    const double variance = std::max(result.rmse * result.rmse, 1e-4);
    pose.pose.covariance[0] = variance;
    pose.pose.covariance[7] = variance;
    pose.pose.covariance[14] = 0.01;
    pose.pose.covariance[21] = 0.01;
    pose.pose.covariance[28] = 0.01;
    pose.pose.covariance[35] = variance;
    pose_publisher_->publish(pose);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = pose.header;
      transform.child_frame_id = base_frame_;
      transform.transform.translation.x = result.pose(0, 3);
      transform.transform.translation.y = result.pose(1, 3);
      transform.transform.translation.z = result.pose(2, 3);
      const Eigen::Quaterniond rotation(result.pose.block<3, 3>(0, 0));
      transform.transform.rotation.x = rotation.x();
      transform.transform.rotation.y = rotation.y();
      transform.transform.rotation.z = rotation.z();
      transform.transform.rotation.w = rotation.w();
      tf_broadcaster_.sendTransform(transform);
    }

    open3d::geometry::PointCloud aligned = scan;
    aligned.Transform(result.pose);
    std_msgs::msg::Header cloud_header;
    cloud_header.stamp = stamp;
    cloud_header.frame_id = map_frame_;
    aligned_cloud_publisher_->publish(point_cloud_message(aligned, cloud_header));
    publish_status(status.str());
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
  std::string base_frame_;
  bool publish_tf_{true};
  double processing_period_{0.5};
  int pixel_stride_{4};
  double depth_scale_{0.001};
  double minimum_depth_{0.25};
  double maximum_depth_{4.0};
  double initial_pose_max_variance_{0.0};
  int consistent_pose_count_required_{3};
  int failures_before_relocalization_{5};

  std::mutex mutex_;
  CameraIntrinsics intrinsics_;
  bool has_intrinsics_{false};
  Eigen::Matrix4d pose_{Eigen::Matrix4d::Identity()};
  bool has_pose_{false};
  bool initial_alignment_{true};
  std::chrono::steady_clock::time_point last_processing_time_;
  std::unique_ptr<IcpLocalizer> localizer_;
  std::unique_ptr<LocalizationSupervisor> supervisor_;
  std::optional<bool> desired_hloc_enabled_;
  bool hloc_request_in_flight_{false};
  bool hloc_control_synchronized_{false};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_cloud_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
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
