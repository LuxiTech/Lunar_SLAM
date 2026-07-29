#include "luxi_location/depth_projection.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace luxi_location
{
namespace
{

bool validate(
  const std::size_t data_size,
  const std::size_t row_step,
  const CameraIntrinsics & intrinsics,
  const int pixel_stride,
  const std::size_t pixel_size,
  std::string & error)
{
  if (intrinsics.width <= 0 || intrinsics.height <= 0 ||
    intrinsics.fx <= 0.0 || intrinsics.fy <= 0.0)
  {
    error = "camera intrinsics are invalid";
    return false;
  }
  if (pixel_stride <= 0) {
    error = "pixel stride must be positive";
    return false;
  }
  if (row_step < static_cast<std::size_t>(intrinsics.width) * pixel_size ||
    data_size < row_step * static_cast<std::size_t>(intrinsics.height))
  {
    error = "depth image buffer is smaller than its declared dimensions";
    return false;
  }
  return true;
}

template<typename ReadDepth>
std::vector<Eigen::Vector3d> project(
  const std::uint8_t * data,
  const CameraIntrinsics & intrinsics,
  const Eigen::Matrix4d & base_from_camera,
  const int pixel_stride,
  const double minimum_depth,
  const double maximum_depth,
  ReadDepth read_depth)
{
  std::vector<Eigen::Vector3d> points;
  const std::size_t reserve_size =
    static_cast<std::size_t>(intrinsics.width / pixel_stride + 1) *
    static_cast<std::size_t>(intrinsics.height / pixel_stride + 1);
  points.reserve(reserve_size);
  for (int v = 0; v < intrinsics.height; v += pixel_stride) {
    for (int u = 0; u < intrinsics.width; u += pixel_stride) {
      const double depth = read_depth(data, u, v);
      if (!std::isfinite(depth) || depth < minimum_depth || depth > maximum_depth) {
        continue;
      }
      const double x = (static_cast<double>(u) - intrinsics.cx) * depth / intrinsics.fx;
      const double y = (static_cast<double>(v) - intrinsics.cy) * depth / intrinsics.fy;
      const Eigen::Vector4d camera_point(x, y, depth, 1.0);
      points.emplace_back((base_from_camera * camera_point).head<3>());
    }
  }
  return points;
}

}  // namespace

std::vector<Eigen::Vector3d> project_depth_u16(
  const std::uint8_t * data,
  const std::size_t data_size,
  const std::size_t row_step,
  const CameraIntrinsics & intrinsics,
  const Eigen::Matrix4d & base_from_camera,
  const int pixel_stride,
  const double depth_scale,
  const double minimum_depth,
  const double maximum_depth,
  std::string & error)
{
  if (!validate(data_size, row_step, intrinsics, pixel_stride, sizeof(std::uint16_t), error)) {
    return {};
  }
  if (depth_scale <= 0.0) {
    error = "depth scale must be positive";
    return {};
  }
  error.clear();
  return project(
    data, intrinsics, base_from_camera, pixel_stride, minimum_depth, maximum_depth,
    [row_step, depth_scale](const std::uint8_t * buffer, const int u, const int v) {
      std::uint16_t raw = 0;
      std::memcpy(
        &raw,
        buffer + static_cast<std::size_t>(v) * row_step +
        static_cast<std::size_t>(u) * sizeof(std::uint16_t),
        sizeof(raw));
      return static_cast<double>(raw) * depth_scale;
    });
}

std::vector<Eigen::Vector3d> project_depth_f32(
  const std::uint8_t * data,
  const std::size_t data_size,
  const std::size_t row_step,
  const CameraIntrinsics & intrinsics,
  const Eigen::Matrix4d & base_from_camera,
  const int pixel_stride,
  const double minimum_depth,
  const double maximum_depth,
  std::string & error)
{
  if (!validate(data_size, row_step, intrinsics, pixel_stride, sizeof(float), error)) {
    return {};
  }
  error.clear();
  return project(
    data, intrinsics, base_from_camera, pixel_stride, minimum_depth, maximum_depth,
    [row_step](const std::uint8_t * buffer, const int u, const int v) {
      float raw = std::numeric_limits<float>::quiet_NaN();
      std::memcpy(
        &raw,
        buffer + static_cast<std::size_t>(v) * row_step +
        static_cast<std::size_t>(u) * sizeof(float),
        sizeof(raw));
      return static_cast<double>(raw);
    });
}

}  // namespace luxi_location
