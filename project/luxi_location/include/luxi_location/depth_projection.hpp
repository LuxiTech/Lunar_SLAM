#pragma once

#include <Eigen/Core>

#include <cstdint>
#include <string>
#include <vector>

namespace luxi_location
{

struct CameraIntrinsics
{
  int width{0};
  int height{0};
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
};

std::vector<Eigen::Vector3d> project_depth_u16(
  const std::uint8_t * data,
  std::size_t data_size,
  std::size_t row_step,
  const CameraIntrinsics & intrinsics,
  const Eigen::Matrix4d & base_from_camera,
  int pixel_stride,
  double depth_scale,
  double minimum_depth,
  double maximum_depth,
  std::string & error);

std::vector<Eigen::Vector3d> project_depth_f32(
  const std::uint8_t * data,
  std::size_t data_size,
  std::size_t row_step,
  const CameraIntrinsics & intrinsics,
  const Eigen::Matrix4d & base_from_camera,
  int pixel_stride,
  double minimum_depth,
  double maximum_depth,
  std::string & error);

}  // namespace luxi_location
