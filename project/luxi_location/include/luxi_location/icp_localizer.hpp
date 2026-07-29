#pragma once

#include <Eigen/Core>

#include <memory>
#include <string>

namespace open3d::geometry
{
class PointCloud;
}

namespace luxi_location
{

struct IcpParameters
{
  double map_voxel_size{0.08};
  double scan_voxel_size{0.06};
  double coarse_voxel_size{0.16};
  double coarse_max_correspondence_distance{0.50};
  double fine_max_correspondence_distance{0.20};
  int coarse_iterations{30};
  int fine_iterations{30};
  int minimum_scan_points{150};
  double minimum_fitness{0.25};
  double maximum_rmse{0.15};
  double maximum_translation_correction{0.50};
  double maximum_yaw_correction{0.35};
  double initial_maximum_translation_correction{1.50};
  double initial_maximum_yaw_correction{0.79};
};

struct IcpResult
{
  bool accepted{false};
  Eigen::Matrix4d pose{Eigen::Matrix4d::Identity()};
  double fitness{0.0};
  double rmse{0.0};
  double translation_correction{0.0};
  double yaw_correction{0.0};
  std::string reason;
};

class IcpLocalizer
{
public:
  explicit IcpLocalizer(IcpParameters parameters = {});

  bool load_map(const std::string & path, std::string & error);
  IcpResult register_scan(
    const open3d::geometry::PointCloud & scan,
    const Eigen::Matrix4d & initial_pose,
    bool initial_alignment) const;

  const open3d::geometry::PointCloud & map() const;
  static Eigen::Matrix4d planar_pose(double x, double y, double z, double yaw);
  static double yaw(const Eigen::Matrix4d & pose);
  static double normalize_angle(double angle);

private:
  IcpParameters parameters_;
  std::shared_ptr<open3d::geometry::PointCloud> map_;
  std::shared_ptr<open3d::geometry::PointCloud> coarse_map_;
};

}  // namespace luxi_location
