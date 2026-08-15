#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "luxi_3d_navigation/rolling_voxel_grid.hpp"

namespace luxi_3d_navigation
{
namespace
{

double readDepthMeters(const sensor_msgs::msg::Image & image, int row, int column)
{
  const std::size_t bytes_per_pixel = image.encoding == sensor_msgs::image_encodings::TYPE_16UC1 ?
    sizeof(std::uint16_t) : sizeof(float);
  const std::size_t offset = static_cast<std::size_t>(row) * image.step +
    static_cast<std::size_t>(column) * bytes_per_pixel;
  if (offset + bytes_per_pixel > image.data.size()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (image.encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
    std::uint16_t value{};
    std::memcpy(&value, image.data.data() + offset, sizeof(value));
    if (image.is_bigendian) {
      value = static_cast<std::uint16_t>((value >> 8U) | (value << 8U));
    }
    return static_cast<double>(value) * 0.001;
  }
  if (image.encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    float value{};
    std::memcpy(&value, image.data.data() + offset, sizeof(value));
    return static_cast<double>(value);
  }
  return std::numeric_limits<double>::quiet_NaN();
}

tf2::Transform asTransform(const geometry_msgs::msg::TransformStamped & message)
{
  tf2::Transform transform;
  tf2::fromMsg(message.transform, transform);
  return transform;
}

Point3D point3D(const tf2::Vector3 & point)
{
  return Point3D{point.x(), point.y(), point.z()};
}

}  // namespace

class LocalObstacleMapNode : public rclcpp::Node
{
public:
  LocalObstacleMapNode()
  : Node("local_obstacle_map"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_),
    grid_(gridParameters())
  {
    depth_topic_ = declare_parameter<std::string>(
      "depth_topic", "/sensors/rgbd/depth/image_raw");
    camera_info_topic_ = declare_parameter<std::string>(
      "camera_info_topic", "/sensors/rgbd/color/camera_info");
    target_frame_ = declare_parameter<std::string>("target_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    points_topic_ = declare_parameter<std::string>(
      "points_topic", "/navigation/local_obstacles/points");
    state_topic_ = declare_parameter<std::string>(
      "state_topic", "/navigation/local_obstacles/state");
    nearest_topic_ = declare_parameter<std::string>(
      "nearest_topic", "/navigation/local_obstacles/nearest_distance");
    skip_pixel_ = std::max(
      1, static_cast<int>(declare_parameter<int>("skip_pixel", 8)));
    raycast_stride_ = std::max(
      1, static_cast<int>(declare_parameter<int>("raycast_stride", 4)));
    minimum_depth_ = declare_parameter<double>("minimum_depth", 0.25);
    maximum_depth_ = declare_parameter<double>("maximum_depth", 2.5);
    minimum_obstacle_z_ = declare_parameter<double>("minimum_obstacle_z", -0.32);
    maximum_obstacle_z_ = declare_parameter<double>("maximum_obstacle_z", 0.30);
    self_filter_x_min_ = declare_parameter<double>("self_filter_x_min", -0.25);
    self_filter_x_max_ = declare_parameter<double>("self_filter_x_max", 0.20);
    self_filter_half_width_ = declare_parameter<double>("self_filter_half_width", 0.20);
    corridor_half_width_ = declare_parameter<double>("corridor_half_width", 0.25);
    stop_distance_ = declare_parameter<double>("stop_distance", 0.45);
    slow_distance_ = declare_parameter<double>("slow_distance", 0.70);
    sensor_timeout_ = declare_parameter<double>("sensor_timeout", 0.35);
    tf_timeout_ = declare_parameter<double>("tf_timeout", 0.05);
    state_clear_hold_ = declare_parameter<double>("state_clear_hold", 0.50);
    publish_rate_ = std::max(1.0, declare_parameter<double>("publish_rate", 15.0));

    if (minimum_depth_ <= 0.0 || maximum_depth_ <= minimum_depth_ ||
      minimum_obstacle_z_ >= maximum_obstacle_z_ || stop_distance_ <= 0.0 ||
      slow_distance_ < stop_distance_ || sensor_timeout_ <= 0.0)
    {
      throw std::invalid_argument("local obstacle map parameters are invalid");
    }

    points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      points_topic_, rclcpp::SensorDataQoS());
    state_pub_ = create_publisher<std_msgs::msg::String>(
      state_topic_, rclcpp::QoS(1).reliable().transient_local());
    nearest_pub_ = create_publisher<std_msgs::msg::Float32>(nearest_topic_, 10);
    auto latest_sensor_qos = rclcpp::SensorDataQoS();
    latest_sensor_qos.keep_last(1);
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, latest_sensor_qos,
      [this](const sensor_msgs::msg::CameraInfo::SharedPtr message) {camera_info_ = message;});
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic_, latest_sensor_qos,
      [this](const sensor_msgs::msg::Image::SharedPtr message) {onDepth(*message);});
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_), [this]() {publish();});
    publishState("stale");
    RCLCPP_INFO(
      get_logger(), "Rolling obstacle map: depth=%s target=%s resolution=%.3f timeout=%.2fs",
      depth_topic_.c_str(), target_frame_.c_str(),
      get_parameter("resolution").as_double(), sensor_timeout_);
  }

private:
  RollingVoxelGridParameters gridParameters()
  {
    RollingVoxelGridParameters parameters;
    parameters.resolution = declare_parameter<double>("resolution", 0.05);
    parameters.hit_increment = declare_parameter<int>("hit_increment", 1);
    parameters.miss_decrement = declare_parameter<int>("miss_decrement", 1);
    parameters.occupied_threshold = declare_parameter<int>("occupied_threshold", 2);
    parameters.minimum_score = declare_parameter<int>("minimum_score", -3);
    parameters.maximum_score = declare_parameter<int>("maximum_score", 5);
    parameters.occupied_ttl = declare_parameter<double>("occupied_ttl", 1.5);
    parameters.stale_entry_ttl = declare_parameter<double>("stale_entry_ttl", 3.0);
    // The grid is stored in map coordinates. A symmetric 6 m square keeps a
    // 3 m robot-relative sensing horizon for every robot yaw.
    parameters.size_x = declare_parameter<double>("size_x", 6.0);
    parameters.size_y = declare_parameter<double>("size_y", 6.0);
    parameters.size_z = declare_parameter<double>("size_z", 1.5);
    return parameters;
  }

  void onDepth(const sensor_msgs::msg::Image & image)
  {
    last_depth_received_ = now();
    if (!camera_info_) {
      last_error_ = "camera_info_missing";
      return;
    }
    if (image.encoding != sensor_msgs::image_encodings::TYPE_16UC1 &&
      image.encoding != sensor_msgs::image_encodings::TYPE_32FC1)
    {
      last_error_ = "unsupported_depth_encoding";
      return;
    }
    if (camera_info_->k[0] <= 0.0 || camera_info_->k[4] <= 0.0 || image.width == 0U ||
      image.height == 0U)
    {
      last_error_ = "invalid_camera_info";
      return;
    }
    const std::string camera_frame = !image.header.frame_id.empty() ?
      image.header.frame_id : camera_info_->header.frame_id;
    if (camera_frame.empty()) {
      last_error_ = "camera_frame_missing";
      return;
    }

    try {
      const rclcpp::Time header_stamp(image.header.stamp);
      const rclcpp::Time stamp = header_stamp.nanoseconds() == 0 ? now() : header_stamp;
      const double image_age = (now() - stamp).seconds();
      if (image_age < -0.1 || image_age > sensor_timeout_) {
        last_error_ = "depth_stamp_stale";
        return;
      }
      // The camera commonly leads the 2 Hz localization transform by one or
      // two depth frames.  Use one internally consistent latest base pose
      // instead of dropping the whole frame on a small future extrapolation.
      // Image age is checked above, so this fallback remains time-bounded.
      const auto base_from_camera_msg = tf_buffer_.lookupTransform(
        base_frame_, camera_frame, tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_));
      const auto target_from_base_msg = tf_buffer_.lookupTransform(
        target_frame_, base_frame_, tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_));
      const tf2::Transform base_from_camera = asTransform(base_from_camera_msg);
      const tf2::Transform target_from_base = asTransform(target_from_base_msg);
      const tf2::Transform target_from_camera = target_from_base * base_from_camera;

      const double fx = camera_info_->k[0];
      const double fy = camera_info_->k[4];
      const double cx = camera_info_->k[2];
      const double cy = camera_info_->k[5];
      std::vector<RayObservation> observations;
      observations.reserve(
        static_cast<std::size_t>(image.width / skip_pixel_ + 1U) *
        static_cast<std::size_t>(image.height / skip_pixel_ + 1U));
      for (int row = 0; row < static_cast<int>(image.height); row += skip_pixel_) {
        for (int column = 0; column < static_cast<int>(image.width); column += skip_pixel_) {
          double depth = readDepthMeters(image, row, column);
          if (!std::isfinite(depth) || depth < minimum_depth_) {
            continue;
          }
          const bool endpoint_in_range = depth <= maximum_depth_;
          depth = std::min(depth, maximum_depth_);
          const tf2::Vector3 camera_point(
            (static_cast<double>(column) - cx) * depth / fx,
            (static_cast<double>(row) - cy) * depth / fy, depth);
          const tf2::Vector3 base_point = base_from_camera * camera_point;
          const bool self_point = base_point.x() >= self_filter_x_min_ &&
            base_point.x() <= self_filter_x_max_ &&
            std::abs(base_point.y()) <= self_filter_half_width_;
          const bool collision_height = base_point.z() >= minimum_obstacle_z_ &&
            base_point.z() <= maximum_obstacle_z_;
          observations.push_back(RayObservation{
            point3D(target_from_camera * camera_point),
            endpoint_in_range && collision_height && !self_point,
            ((row / skip_pixel_) + (column / skip_pixel_)) % raycast_stride_ == 0});
        }
      }
      grid_.integrateFrame(point3D(target_from_camera.getOrigin()), observations, stamp.seconds());
      grid_.prune(point3D(target_from_base.getOrigin()), stamp.seconds());
      last_successful_depth_ = now();
      last_error_.clear();
    } catch (const tf2::TransformException & error) {
      last_error_ = "tf_unavailable";
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Cannot integrate depth frame: %s", error.what());
    }
  }

  void publish()
  {
    const auto current_time = now();
    const bool healthy = last_successful_depth_.nanoseconds() != 0 &&
      (current_time - last_successful_depth_).seconds() <= sensor_timeout_;
    if (!healthy) {
      safety_state_ = last_error_.empty() ? "stale" : last_error_;
      less_restrictive_candidate_.clear();
      publishState(last_error_.empty() ? "stale" : last_error_);
      publishNearest(std::numeric_limits<float>::infinity());
      return;
    }

    try {
      const auto target_from_base_msg = tf_buffer_.lookupTransform(
        target_frame_, base_frame_, tf2::TimePointZero);
      const tf2::Transform target_from_base = asTransform(target_from_base_msg);
      const tf2::Transform base_from_target = target_from_base.inverse();
      grid_.prune(point3D(target_from_base.getOrigin()), current_time.seconds());
      const auto occupied = grid_.occupiedPoints(current_time.seconds());
      double nearest = std::numeric_limits<double>::infinity();
      for (const auto & point : occupied) {
        const auto in_base = base_from_target * tf2::Vector3(point.x, point.y, point.z);
        if (in_base.x() >= 0.0 && std::abs(in_base.y()) <= corridor_half_width_ &&
          in_base.z() >= minimum_obstacle_z_ && in_base.z() <= maximum_obstacle_z_)
        {
          nearest = std::min(nearest, in_base.x());
        }
      }
      const std::string raw_state = nearest <= stop_distance_ ? "blocked" :
        (nearest <= slow_distance_ ? "slow" : "clear");
      publishSafetyState(raw_state, current_time);
      publishNearest(static_cast<float>(nearest));
      publishCloud(occupied, current_time);
    } catch (const tf2::TransformException & error) {
      publishState("tf_unavailable");
      publishNearest(std::numeric_limits<float>::infinity());
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Cannot evaluate obstacle corridor: %s", error.what());
    }
  }

  void publishCloud(const std::vector<Point3D> & points, const rclcpp::Time & stamp)
  {
    if (points_pub_->get_subscription_count() == 0U) {
      return;
    }
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = target_frame_;
    cloud.height = 1U;
    cloud.width = static_cast<std::uint32_t>(points.size());
    cloud.is_dense = true;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    for (const auto & point : points) {
      *x = static_cast<float>(point.x);
      *y = static_cast<float>(point.y);
      *z = static_cast<float>(point.z);
      ++x;
      ++y;
      ++z;
    }
    points_pub_->publish(cloud);
  }

  void publishState(const std::string & state)
  {
    std_msgs::msg::String message;
    message.data = state;
    state_pub_->publish(message);
    last_published_state_ = state;
  }

  static int safetyRank(const std::string & state)
  {
    if (state == "blocked") {
      return 2;
    }
    if (state == "slow") {
      return 1;
    }
    if (state == "clear") {
      return 0;
    }
    return -1;
  }

  void publishSafetyState(const std::string & raw_state, const rclcpp::Time & stamp)
  {
    const int current_rank = safetyRank(safety_state_);
    const int raw_rank = safetyRank(raw_state);
    if (current_rank < 0 || raw_rank >= current_rank) {
      safety_state_ = raw_state;
      less_restrictive_candidate_.clear();
    } else if (less_restrictive_candidate_ != raw_state) {
      less_restrictive_candidate_ = raw_state;
      less_restrictive_since_ = stamp;
    } else if ((stamp - less_restrictive_since_).seconds() >= state_clear_hold_) {
      safety_state_ = raw_state;
      less_restrictive_candidate_.clear();
    }
    publishState(safety_state_);
  }

  void publishNearest(float distance)
  {
    std_msgs::msg::Float32 message;
    message.data = distance;
    nearest_pub_->publish(message);
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  RollingVoxelGrid grid_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string target_frame_;
  std::string base_frame_;
  std::string points_topic_;
  std::string state_topic_;
  std::string nearest_topic_;
  std::string last_error_;
  std::string last_published_state_;
  int skip_pixel_{};
  int raycast_stride_{};
  double minimum_depth_{};
  double maximum_depth_{};
  double minimum_obstacle_z_{};
  double maximum_obstacle_z_{};
  double self_filter_x_min_{};
  double self_filter_x_max_{};
  double self_filter_half_width_{};
  double corridor_half_width_{};
  double stop_distance_{};
  double slow_distance_{};
  double sensor_timeout_{};
  double tf_timeout_{};
  double state_clear_hold_{};
  double publish_rate_{};
  rclcpp::Time last_depth_received_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_successful_depth_{0, 0, RCL_ROS_TIME};
  rclcpp::Time less_restrictive_since_{0, 0, RCL_ROS_TIME};
  std::string safety_state_{"stale"};
  std::string less_restrictive_candidate_;
  sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr nearest_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace luxi_3d_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<luxi_3d_navigation::LocalObstacleMapNode>());
  rclcpp::shutdown();
  return 0;
}
