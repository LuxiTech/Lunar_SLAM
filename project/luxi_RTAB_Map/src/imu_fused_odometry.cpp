#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace
{

using Imu = sensor_msgs::msg::Imu;
using Odometry = nav_msgs::msg::Odometry;

double stamp_seconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1.0e-9;
}

double quaternion_angle(const tf2::Quaternion & quaternion)
{
  const auto normalized_w = std::clamp(std::abs(quaternion.w()), 0.0, 1.0);
  return 2.0 * std::acos(normalized_w);
}

bool finite_quaternion(const geometry_msgs::msg::Quaternion & value)
{
  return std::isfinite(value.x) && std::isfinite(value.y) &&
         std::isfinite(value.z) && std::isfinite(value.w);
}

struct OrientationSample
{
  double stamp = 0.0;
  tf2::Quaternion world_from_base;
};

class ImuFusedOdometry final : public rclcpp::Node
{
public:
  ImuFusedOdometry()
  : Node("imu_fused_odometry"), tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_), tf_broadcaster_(*this)
  {
    const auto input_topic = declare_parameter<std::string>(
      "input_topic", "/rtabmap/odom_raw");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/rtabmap/odom");
    const auto imu_topic = declare_parameter<std::string>(
      "imu_topic", "/sensors/imu/data");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    maximum_imu_time_difference_ = declare_parameter<double>(
      "maximum_imu_time_difference", 0.03);
    maximum_raw_translation_ = declare_parameter<double>(
      "maximum_raw_translation", 0.30);
    maximum_raw_rotation_ = declare_parameter<double>(
      "maximum_raw_rotation_deg", 60.0) * M_PI / 180.0;
    maximum_orientation_disagreement_ = declare_parameter<double>(
      "maximum_orientation_disagreement_deg", 15.0) * M_PI / 180.0;
    maximum_pose_variance_ = declare_parameter<double>(
      "maximum_pose_variance", 0.1);
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    if (input_topic == output_topic) {
      throw std::invalid_argument("input_topic and output_topic must be different");
    }

    const auto qos = rclcpp::SensorDataQoS().keep_last(20);
    publisher_ = create_publisher<Odometry>(output_topic, qos);
    imu_subscription_ = create_subscription<Imu>(
      imu_topic, rclcpp::SensorDataQoS().keep_last(200),
      std::bind(&ImuFusedOdometry::imu_callback, this, std::placeholders::_1));
    odom_subscription_ = create_subscription<Odometry>(
      input_topic, qos,
      std::bind(&ImuFusedOdometry::odom_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Filtering F2M pose increments with synchronized H30 rotation: %s + %s -> %s",
      input_topic.c_str(), imu_topic.c_str(), output_topic.c_str());
  }

private:
  void imu_callback(const Imu::ConstSharedPtr input)
  {
    if (input->orientation_covariance[0] < 0.0 || input->header.frame_id.empty() ||
      !finite_quaternion(input->orientation))
    {
      return;
    }
    tf2::Quaternion world_from_imu;
    tf2::fromMsg(input->orientation, world_from_imu);
    if (world_from_imu.length2() < 1.0e-12) {
      return;
    }
    world_from_imu.normalize();

    try {
      const auto transform = tf_buffer_.lookupTransform(
        base_frame_, input->header.frame_id, tf2::TimePointZero);
      tf2::Quaternion base_from_imu;
      tf2::fromMsg(transform.transform.rotation, base_from_imu);
      base_from_imu.normalize();
      auto world_from_base = world_from_imu * base_from_imu.inverse();
      world_from_base.normalize();
      orientations_.push_back({stamp_seconds(input->header.stamp), world_from_base});
      while (orientations_.size() > 400) {
        orientations_.pop_front();
      }
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for %s <- %s transform: %s", base_frame_.c_str(),
        input->header.frame_id.c_str(), error.what());
    }
  }

  bool nearest_orientation(double stamp, tf2::Quaternion & orientation) const
  {
    double nearest_error = std::numeric_limits<double>::infinity();
    for (auto iterator = orientations_.rbegin(); iterator != orientations_.rend(); ++iterator) {
      const double error = std::abs(iterator->stamp - stamp);
      if (error < nearest_error) {
        nearest_error = error;
        orientation = iterator->world_from_base;
      }
      if (iterator->stamp < stamp - maximum_imu_time_difference_ &&
        nearest_error <= maximum_imu_time_difference_)
      {
        break;
      }
    }
    return nearest_error <= maximum_imu_time_difference_;
  }

  void odom_callback(const Odometry::ConstSharedPtr input)
  {
    if (!finite_quaternion(input->pose.pose.orientation)) {
      return;
    }
    tf2::Quaternion raw_orientation;
    tf2::fromMsg(input->pose.pose.orientation, raw_orientation);
    if (raw_orientation.length2() < 1.0e-12) {
      return;
    }
    raw_orientation.normalize();

    tf2::Quaternion world_from_base;
    if (!nearest_orientation(stamp_seconds(input->header.stamp), world_from_base)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skipping F2M odometry without a synchronized H30 orientation");
      return;
    }

    const tf2::Vector3 raw_position(
      input->pose.pose.position.x, input->pose.pose.position.y,
      input->pose.pose.position.z);
    if (!initialized_) {
      if (!std::isfinite(input->pose.covariance[0]) ||
        input->pose.covariance[0] >= maximum_pose_variance_)
      {
        return;
      }
      correction_orientation_.setValue(0.0, 0.0, 0.0, 1.0);
      correction_translation_.setValue(0.0, 0.0, 0.0);
      fused_orientation_ = raw_orientation;
      fused_position_ = raw_position;
      previous_raw_orientation_ = raw_orientation;
      previous_imu_orientation_ = world_from_base;
      initialized_ = true;
    } else {
      auto raw_delta_rotation = previous_raw_orientation_.inverse() * raw_orientation;
      raw_delta_rotation.normalize();
      auto imu_delta_rotation =
        previous_imu_orientation_.inverse() * world_from_base;
      imu_delta_rotation.normalize();
      auto orientation_disagreement =
        raw_delta_rotation.inverse() * imu_delta_rotation;
      orientation_disagreement.normalize();

      auto candidate_orientation = correction_orientation_ * raw_orientation;
      candidate_orientation.normalize();
      const auto candidate_position =
        tf2::quatRotate(correction_orientation_, raw_position) +
        correction_translation_;
      const auto candidate_translation = candidate_position - fused_position_;
      const bool covariance_is_valid =
        std::isfinite(input->pose.covariance[0]) &&
        input->pose.covariance[0] < maximum_pose_variance_;
      const bool rotation_is_valid =
        quaternion_angle(raw_delta_rotation) <= maximum_raw_rotation_ &&
        quaternion_angle(orientation_disagreement) <=
        maximum_orientation_disagreement_;
      const bool translation_is_valid =
        candidate_translation.length() <= maximum_raw_translation_;
      const bool reset_detected = waiting_for_reset_recovery_ ||
        !covariance_is_valid || !rotation_is_valid || !translation_is_valid;

      if (reset_detected) {
        if (!covariance_is_valid) {
          waiting_for_reset_recovery_ = true;
          previous_raw_orientation_ = raw_orientation;
          previous_imu_orientation_ = world_from_base;
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Suppressing invalid F2M reset frame: covariance=%.3g",
            input->pose.covariance[0]);
          return;
        } else {
          auto desired_orientation = fused_orientation_ * imu_delta_rotation;
          desired_orientation.normalize();
          correction_orientation_ = desired_orientation * raw_orientation.inverse();
          correction_orientation_.normalize();
          correction_translation_ = fused_position_ -
            tf2::quatRotate(correction_orientation_, raw_position);
          fused_orientation_ = desired_orientation;
          waiting_for_reset_recovery_ = false;
          reset_corrected_ = true;
        }
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Holding F2M reset discontinuity: covariance=%.3g "
          "translation=%.3f m raw_rotation=%.1f deg disagreement=%.1f deg",
          input->pose.covariance[0], candidate_translation.length(),
          quaternion_angle(raw_delta_rotation) * 180.0 / M_PI,
          quaternion_angle(orientation_disagreement) * 180.0 / M_PI);
      } else {
        fused_orientation_ = candidate_orientation;
        fused_position_ = candidate_position;
      }
      previous_raw_orientation_ = raw_orientation;
      previous_imu_orientation_ = world_from_base;
    }

    Odometry output = *input;
    output.pose.pose.position.x = fused_position_.x();
    output.pose.pose.position.y = fused_position_.y();
    output.pose.pose.position.z = fused_position_.z();
    output.pose.pose.orientation = tf2::toMsg(fused_orientation_);
    if (reset_corrected_) {
      output.twist.twist = geometry_msgs::msg::Twist();
      reset_corrected_ = false;
    }
    publisher_->publish(output);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = output.header;
      transform.child_frame_id = output.child_frame_id;
      transform.transform.translation.x = fused_position_.x();
      transform.transform.translation.y = fused_position_.y();
      transform.transform.translation.z = fused_position_.z();
      transform.transform.rotation = output.pose.pose.orientation;
      tf_broadcaster_.sendTransform(transform);
    }
  }

  std::string base_frame_ = "base_link";
  double maximum_imu_time_difference_ = 0.03;
  double maximum_raw_translation_ = 0.30;
  double maximum_raw_rotation_ = M_PI / 3.0;
  double maximum_orientation_disagreement_ = M_PI / 12.0;
  double maximum_pose_variance_ = 0.1;
  bool publish_tf_ = true;
  bool initialized_ = false;
  bool waiting_for_reset_recovery_ = false;
  bool reset_corrected_ = false;
  std::deque<OrientationSample> orientations_;
  tf2::Quaternion correction_orientation_;
  tf2::Vector3 correction_translation_;
  tf2::Quaternion previous_raw_orientation_;
  tf2::Quaternion previous_imu_orientation_;
  tf2::Quaternion fused_orientation_;
  tf2::Vector3 fused_position_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  rclcpp::Publisher<Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_subscription_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuFusedOdometry>());
  rclcpp::shutdown();
  return 0;
}
