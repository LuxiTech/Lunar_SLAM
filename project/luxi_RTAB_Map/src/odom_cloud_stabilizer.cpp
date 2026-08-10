#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/exact_time.hpp>
#include <message_filters/synchronizer.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace
{

using Cloud = sensor_msgs::msg::PointCloud2;
using OccupancyGrid = nav_msgs::msg::OccupancyGrid;
using Odometry = nav_msgs::msg::Odometry;
using ExactPolicy = message_filters::sync_policies::ExactTime<Cloud, Odometry>;

bool finite_pose(const geometry_msgs::msg::Pose & pose)
{
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
         std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) &&
         std::isfinite(pose.orientation.w);
}

class OdomCloudStabilizer final : public rclcpp::Node
{
public:
  OdomCloudStabilizer()
  : Node("odom_cloud_stabilizer"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    const auto cloud_input = declare_parameter<std::string>(
      "cloud_input", "/usb_stereo/points");
    const auto odom_input = declare_parameter<std::string>(
      "odom_input", "/rtabmap/odom");
    const auto cloud_output = declare_parameter<std::string>(
      "cloud_output", "/rtabmap/usb_points_stable");
    const auto map_cloud_input = declare_parameter<std::string>(
      "map_cloud_input", "/rtabmap/cloud_map");
    const auto map_cloud_output = declare_parameter<std::string>(
      "map_cloud_output", "/rtabmap/cloud_map_visual");
    const auto occupancy_grid_input = declare_parameter<std::string>(
      "occupancy_grid_input", "/rtabmap/map");
    const auto occupancy_grid_output = declare_parameter<std::string>(
      "occupancy_grid_output", "/rtabmap/map_visual");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    history_size_ = declare_parameter<int>("history_size", 2);
    const int sync_queue_size = declare_parameter<int>("sync_queue_size", 20);
    if (history_size_ < 1 || sync_queue_size < 1) {
      throw std::invalid_argument("history_size and sync_queue_size must be positive");
    }

    publisher_ = create_publisher<Cloud>(
      cloud_output, rclcpp::SensorDataQoS().keep_last(1));
    const auto map_qos = rclcpp::QoS(1).reliable().transient_local();
    map_publisher_ = create_publisher<Cloud>(map_cloud_output, map_qos);
    map_subscription_ = create_subscription<Cloud>(
      map_cloud_input, map_qos,
      std::bind(&OdomCloudStabilizer::map_callback, this, std::placeholders::_1));
    occupancy_grid_publisher_ = create_publisher<OccupancyGrid>(
      occupancy_grid_output, map_qos);
    occupancy_grid_subscription_ = create_subscription<OccupancyGrid>(
      occupancy_grid_input, map_qos,
      std::bind(
        &OdomCloudStabilizer::occupancy_grid_callback, this, std::placeholders::_1));
    const auto input_qos = rclcpp::SensorDataQoS().keep_last(sync_queue_size);
    // Humble's message_filters API takes the underlying RMW QoS profile.
    const auto input_rmw_qos = input_qos.get_rmw_qos_profile();
    cloud_subscriber_.subscribe(this, cloud_input, input_rmw_qos);
    odom_subscriber_.subscribe(this, odom_input, input_rmw_qos);
    synchronizer_ = std::make_unique<message_filters::Synchronizer<ExactPolicy>>(
      ExactPolicy(sync_queue_size), cloud_subscriber_, odom_subscriber_);
    synchronizer_->registerCallback(
      std::bind(
        &OdomCloudStabilizer::callback, this, std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "Stable USB cloud: exact sync %s + %s -> %s, history=%d",
      cloud_input.c_str(), odom_input.c_str(), cloud_output.c_str(), history_size_);
    RCLCPP_INFO(
      get_logger(), "RViz map cloud: %s -> %s using the latest map transform",
      map_cloud_input.c_str(), map_cloud_output.c_str());
    RCLCPP_INFO(
      get_logger(), "RViz occupancy grid: %s -> %s using the latest map transform",
      occupancy_grid_input.c_str(), occupancy_grid_output.c_str());
  }

private:
  void map_callback(const Cloud::ConstSharedPtr cloud)
  {
    Cloud output = *cloud;
    output.header.stamp.sec = 0;
    output.header.stamp.nanosec = 0;
    map_publisher_->publish(output);
  }

  void occupancy_grid_callback(const OccupancyGrid::ConstSharedPtr grid)
  {
    OccupancyGrid output = *grid;
    output.header.stamp.sec = 0;
    output.header.stamp.nanosec = 0;
    occupancy_grid_publisher_->publish(output);
  }

  void callback(const Cloud::ConstSharedPtr cloud, const Odometry::ConstSharedPtr odom)
  {
    if (publisher_->get_subscription_count() == 0) {
      history_.clear();
      return;
    }
    if (cloud->data.empty() || cloud->header.frame_id.empty() ||
      odom->header.frame_id.empty() || !finite_pose(odom->pose.pose))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skipping stable cloud because its matching odometry is invalid");
      return;
    }

    geometry_msgs::msg::TransformStamped base_from_sensor;
    try {
      base_from_sensor = tf_buffer_.lookupTransform(
        base_frame_, cloud->header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for static transform %s <- %s: %s", base_frame_.c_str(),
        cloud->header.frame_id.c_str(), error.what());
      return;
    }

    tf2::Transform odom_from_base;
    tf2::Transform base_from_sensor_tf;
    tf2::fromMsg(odom->pose.pose, odom_from_base);
    tf2::fromMsg(base_from_sensor.transform, base_from_sensor_tf);
    const tf2::Transform odom_from_sensor = odom_from_base * base_from_sensor_tf;

    geometry_msgs::msg::TransformStamped odom_from_sensor_message;
    odom_from_sensor_message.header.stamp = cloud->header.stamp;
    odom_from_sensor_message.header.frame_id = odom->header.frame_id;
    odom_from_sensor_message.child_frame_id = cloud->header.frame_id;
    odom_from_sensor_message.transform = tf2::toMsg(odom_from_sensor);

    Cloud transformed;
    tf2::doTransform(*cloud, transformed, odom_from_sensor_message);
    transformed.header.stamp = cloud->header.stamp;
    transformed.header.frame_id = odom->header.frame_id;
    transformed.height = 1;
    transformed.row_step = transformed.width * transformed.point_step;

    history_.push_back(std::move(transformed));
    while (history_.size() > static_cast<std::size_t>(history_size_)) {
      history_.pop_front();
    }
    publisher_->publish(combine_history());
  }

  Cloud combine_history() const
  {
    Cloud output = history_.back();
    output.header.stamp.sec = 0;
    output.header.stamp.nanosec = 0;
    std::size_t total_bytes = 0;
    std::uint64_t total_points = 0;
    for (const auto & cloud : history_) {
      total_bytes += cloud.data.size();
      total_points += cloud.width * cloud.height;
    }

    output.height = 1;
    output.width = static_cast<std::uint32_t>(std::min<std::uint64_t>(
      total_points, std::numeric_limits<std::uint32_t>::max()));
    output.data.clear();
    output.data.reserve(total_bytes);
    for (const auto & cloud : history_) {
      output.data.insert(output.data.end(), cloud.data.begin(), cloud.data.end());
    }
    output.row_step = output.width * output.point_step;
    output.is_dense = true;
    return output;
  }

  std::string base_frame_;
  int history_size_ = 2;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<Cloud>::SharedPtr publisher_;
  rclcpp::Publisher<Cloud>::SharedPtr map_publisher_;
  rclcpp::Subscription<Cloud>::SharedPtr map_subscription_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr occupancy_grid_publisher_;
  rclcpp::Subscription<OccupancyGrid>::SharedPtr occupancy_grid_subscription_;
  message_filters::Subscriber<Cloud> cloud_subscriber_;
  message_filters::Subscriber<Odometry> odom_subscriber_;
  std::unique_ptr<message_filters::Synchronizer<ExactPolicy>> synchronizer_;
  std::deque<Cloud> history_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomCloudStabilizer>());
  rclcpp::shutdown();
  return 0;
}
