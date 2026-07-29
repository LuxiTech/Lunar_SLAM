#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/exceptions.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>

namespace
{
struct VoxelKey
{
    int x{};
    int y{};
    int z{};

    bool operator==(const VoxelKey &other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct VoxelKeyHash
{
    std::size_t operator()(const VoxelKey &key) const
    {
        // Large odd constants commonly used for spatial hashing.
        const std::size_t hx = static_cast<std::size_t>(key.x) * 73856093u;
        const std::size_t hy = static_cast<std::size_t>(key.y) * 19349663u;
        const std::size_t hz = static_cast<std::size_t>(key.z) * 83492791u;
        return hx ^ hy ^ hz;
    }
};

struct PointRGB
{
    float x{};
    float y{};
    float z{};
    std::uint8_t r{};
    std::uint8_t g{};
    std::uint8_t b{};
    std::uint16_t observations{};
};
}  // namespace

class RgbCloudAccumulatorNode : public rclcpp::Node
{
public:
    RgbCloudAccumulatorNode()
    : Node("rgb_cloud_accumulator_node"),
      tf_buffer_(std::make_unique<tf2_ros::Buffer>(get_clock())),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
    {
        input_topic_ = declare_parameter<std::string>("input_cloud_topic", "/stereo/points");
        output_topic_ = declare_parameter<std::string>("output_cloud_topic", "/luxi/cloud_map_accumulated");
        fixed_frame_ = declare_parameter<std::string>("fixed_frame", "odom");
        voxel_size_ = std::max(0.005, declare_parameter<double>("voxel_size", 0.025));
        min_sensor_range_m_ = std::max(0.0, declare_parameter<double>("min_sensor_range_m", 0.45));
        max_sensor_range_m_ = std::max(min_sensor_range_m_ + 0.1, declare_parameter<double>("max_sensor_range_m", 4.5));
        stable_min_observations_ = std::max(
            1, static_cast<int>(declare_parameter<int>("stable_min_observations", 2)));
        input_stride_ = std::max(
            1, static_cast<int>(declare_parameter<int>("input_stride", 1)));
        publish_every_n_clouds_ = std::max(
            1, static_cast<int>(declare_parameter<int>("publish_every_n_clouds", 1)));
        max_voxels_ = std::max<std::size_t>(
            1000, static_cast<std::size_t>(declare_parameter<int>("max_voxels", 700000)));

        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, rclcpp::SensorDataQoS().keep_last(3),
            std::bind(&RgbCloudAccumulatorNode::cloudCallback, this, std::placeholders::_1));
        cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            output_topic_, rclcpp::QoS(1).reliable().durability_volatile());

        RCLCPP_INFO(
            get_logger(),
            "Accumulating RGB cloud: input=%s output=%s fixed_frame=%s voxel=%.3f range=%.2f..%.2fm stable_hits=%d stride=%d max_voxels=%zu",
            input_topic_.c_str(), output_topic_.c_str(), fixed_frame_.c_str(),
            voxel_size_, min_sensor_range_m_, max_sensor_range_m_, stable_min_observations_,
            input_stride_, max_voxels_);
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (msg->header.frame_id.empty()) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Skipping cloud without frame_id");
            return;
        }

        geometry_msgs::msg::TransformStamped transform_msg;
        try {
            // Use the latest TF. For online visualization this is more robust
            // than exact timestamp lookup when camera/odom callbacks have
            // small DDS latency differences.
            transform_msg = tf_buffer_->lookupTransform(
                fixed_frame_, msg->header.frame_id, tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Cannot transform %s -> %s: %s",
                msg->header.frame_id.c_str(), fixed_frame_.c_str(), ex.what());
            return;
        }

        tf2::Transform transform;
        tf2::fromMsg(transform_msg.transform, transform);

        std::size_t accepted = 0;
        try {
            sensor_msgs::PointCloud2ConstIterator<float> in_x(*msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> in_y(*msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> in_z(*msg, "z");
            sensor_msgs::PointCloud2ConstIterator<std::uint8_t> in_r(*msg, "r");
            sensor_msgs::PointCloud2ConstIterator<std::uint8_t> in_g(*msg, "g");
            sensor_msgs::PointCloud2ConstIterator<std::uint8_t> in_b(*msg, "b");

            const std::size_t total_points =
                static_cast<std::size_t>(msg->width) * static_cast<std::size_t>(msg->height);
            for (std::size_t i = 0; i < total_points;
                 ++i, ++in_x, ++in_y, ++in_z, ++in_r, ++in_g, ++in_b) {
                if (input_stride_ > 1 && (i % static_cast<std::size_t>(input_stride_)) != 0) {
                    continue;
                }

                const float x = *in_x;
                const float y = *in_y;
                const float z = *in_z;
                if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
                    z < static_cast<float>(min_sensor_range_m_) ||
                    z > static_cast<float>(max_sensor_range_m_)) {
                    continue;
                }

                const tf2::Vector3 p = transform * tf2::Vector3(x, y, z);
                const VoxelKey key{
                    static_cast<int>(std::floor(p.x() / voxel_size_)),
                    static_cast<int>(std::floor(p.y() / voxel_size_)),
                    static_cast<int>(std::floor(p.z() / voxel_size_))};

                PointRGB point;
                point.x = static_cast<float>(p.x());
                point.y = static_cast<float>(p.y());
                point.z = static_cast<float>(p.z());
                point.r = *in_r;
                point.g = *in_g;
                point.b = *in_b;
                point.observations = 1;

                auto [it, inserted] = map_.insert({key, point});
                if (inserted) {
                    insertion_order_.push_back(key);
                    ++accepted;
                } else {
                    // Keep a light running average so repeated measurements
                    // stabilize surfaces instead of permanently keeping the
                    // first noisy stereo estimate.
                    PointRGB &existing = it->second;
                    const float old_weight = static_cast<float>(
                        std::min<int>(existing.observations, stable_min_observations_ * 4));
                    const float new_weight = 1.0f / (old_weight + 1.0f);
                    existing.x = existing.x * (1.0f - new_weight) + point.x * new_weight;
                    existing.y = existing.y * (1.0f - new_weight) + point.y * new_weight;
                    existing.z = existing.z * (1.0f - new_weight) + point.z * new_weight;
                    existing.r = static_cast<std::uint8_t>(
                        static_cast<float>(existing.r) * (1.0f - new_weight) +
                        static_cast<float>(point.r) * new_weight);
                    existing.g = static_cast<std::uint8_t>(
                        static_cast<float>(existing.g) * (1.0f - new_weight) +
                        static_cast<float>(point.g) * new_weight);
                    existing.b = static_cast<std::uint8_t>(
                        static_cast<float>(existing.b) * (1.0f - new_weight) +
                        static_cast<float>(point.b) * new_weight);
                    if (existing.observations < std::numeric_limits<std::uint16_t>::max()) {
                        ++existing.observations;
                    }
                }
            }
        } catch (const std::exception &ex) {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Cloud accumulation failed: %s", ex.what());
            return;
        }

        while (map_.size() > max_voxels_ && !insertion_order_.empty()) {
            map_.erase(insertion_order_.front());
            insertion_order_.pop_front();
        }

        ++cloud_count_;
        if ((cloud_count_ % publish_every_n_clouds_) == 0) {
            publishMap(msg->header.stamp);
        }

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Accumulated cloud: +%zu candidate voxels, stable=%zu total=%zu",
            accepted, stableVoxelCount(), map_.size());
    }

    void publishMap(const builtin_interfaces::msg::Time &stamp)
    {
        const std::size_t stable_count = stableVoxelCount();
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.stamp = stamp;
        cloud.header.frame_id = fixed_frame_;
        cloud.height = 1;
        cloud.width = static_cast<std::uint32_t>(stable_count);
        cloud.is_dense = false;

        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        modifier.resize(stable_count);

        sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> out_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> out_z(cloud, "z");
        sensor_msgs::PointCloud2Iterator<std::uint8_t> out_r(cloud, "r");
        sensor_msgs::PointCloud2Iterator<std::uint8_t> out_g(cloud, "g");
        sensor_msgs::PointCloud2Iterator<std::uint8_t> out_b(cloud, "b");

        for (const auto &[_, point] : map_) {
            if (point.observations < stable_min_observations_) {
                continue;
            }
            *out_x = point.x;
            *out_y = point.y;
            *out_z = point.z;
            *out_r = point.r;
            *out_g = point.g;
            *out_b = point.b;
            ++out_x;
            ++out_y;
            ++out_z;
            ++out_r;
            ++out_g;
            ++out_b;
        }

        cloud_pub_->publish(cloud);
    }

    std::size_t stableVoxelCount() const
    {
        std::size_t count = 0;
        for (const auto &[_, point] : map_) {
            if (point.observations >= stable_min_observations_) {
                ++count;
            }
        }
        return count;
    }

    std::string input_topic_;
    std::string output_topic_;
    std::string fixed_frame_;
    double voxel_size_{0.025};
    double min_sensor_range_m_{0.45};
    double max_sensor_range_m_{4.5};
    int stable_min_observations_{2};
    int input_stride_{1};
    int publish_every_n_clouds_{1};
    std::size_t max_voxels_{700000};
    std::size_t cloud_count_{0};

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    std::unordered_map<VoxelKey, PointRGB, VoxelKeyHash> map_;
    std::deque<VoxelKey> insertion_order_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RgbCloudAccumulatorNode>());
    rclcpp::shutdown();
    return 0;
}
