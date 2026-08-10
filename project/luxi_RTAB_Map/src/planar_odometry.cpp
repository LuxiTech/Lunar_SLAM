#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

namespace
{

using Odometry = nav_msgs::msg::Odometry;

bool finite_pose(const geometry_msgs::msg::Pose & pose)
{
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w);
}

class PlanarOdometry final : public rclcpp::Node
{
public:
  PlanarOdometry()
  : Node("planar_odometry"), tf_broadcaster_(*this)
  {
    const auto input_topic = declare_parameter<std::string>(
      "input_topic", "/rtabmap/odom_raw");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/rtabmap/odom");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    constrain_to_planar_ = declare_parameter<bool>("constrain_to_planar", true);
    if (input_topic == output_topic) {
      throw std::invalid_argument("input_topic and output_topic must be different");
    }

    const auto qos = rclcpp::SensorDataQoS().keep_last(10);
    publisher_ = create_publisher<Odometry>(output_topic, qos);
    subscription_ = create_subscription<Odometry>(
      input_topic, qos,
      std::bind(&PlanarOdometry::callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Odometry relay: %s -> %s (mode=%s, TF=%s)",
      input_topic.c_str(), output_topic.c_str(),
      constrain_to_planar_ ? "planar" : "full-6DoF",
      publish_tf_ ? "on" : "off");
  }

private:
  void callback(const Odometry::ConstSharedPtr input)
  {
    if (input->header.frame_id.empty() || input->child_frame_id.empty() ||
      !finite_pose(input->pose.pose))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skipping invalid raw odometry message");
      return;
    }

    tf2::Quaternion raw_orientation;
    tf2::fromMsg(input->pose.pose.orientation, raw_orientation);
    if (raw_orientation.length2() < 1.0e-12) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skipping raw odometry with a zero-length orientation");
      return;
    }
    raw_orientation.normalize();

    Odometry output = *input;
    output.pose.pose.orientation = tf2::toMsg(raw_orientation);
    if (constrain_to_planar_) {
      double roll = 0.0;
      double pitch = 0.0;
      double yaw = 0.0;
      tf2::Matrix3x3(raw_orientation).getRPY(roll, pitch, yaw);
      (void)roll;
      (void)pitch;

      tf2::Quaternion planar_orientation;
      planar_orientation.setRPY(0.0, 0.0, yaw);
      planar_orientation.normalize();

      output.pose.pose.position.z = 0.0;
      output.pose.pose.orientation = tf2::toMsg(planar_orientation);
      output.twist.twist.linear.z = 0.0;
      output.twist.twist.angular.x = 0.0;
      output.twist.twist.angular.y = 0.0;
    }
    publisher_->publish(output);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header = output.header;
      transform.child_frame_id = output.child_frame_id;
      transform.transform.translation.x = output.pose.pose.position.x;
      transform.transform.translation.y = output.pose.pose.position.y;
      transform.transform.translation.z = output.pose.pose.position.z;
      transform.transform.rotation = output.pose.pose.orientation;
      tf_broadcaster_.sendTransform(transform);
    }
  }

  bool publish_tf_ = true;
  bool constrain_to_planar_ = true;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  rclcpp::Publisher<Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<Odometry>::SharedPtr subscription_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanarOdometry>());
  rclcpp::shutdown();
  return 0;
}
