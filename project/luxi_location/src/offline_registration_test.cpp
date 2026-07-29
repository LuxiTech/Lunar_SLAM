#include "luxi_location/icp_localizer.hpp"

#include <open3d/Open3D.h>

#include <cmath>
#include <iostream>
#include <string>

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "usage: offline_registration_test MAP.ply\n";
    return 2;
  }

  luxi_location::IcpParameters parameters;
  parameters.minimum_fitness = 0.70;
  parameters.maximum_rmse = 0.10;
  luxi_location::IcpLocalizer localizer(parameters);
  std::string error;
  if (!localizer.load_map(argv[1], error)) {
    std::cerr << error << '\n';
    return 3;
  }

  const Eigen::Matrix4d expected =
    luxi_location::IcpLocalizer::planar_pose(0.30, -0.20, 0.0, 10.0 * M_PI / 180.0);
  open3d::geometry::PointCloud scan = localizer.map();
  scan.Transform(expected.inverse());
  const Eigen::Matrix4d initial =
    luxi_location::IcpLocalizer::planar_pose(0.38, -0.27, 0.0, 14.0 * M_PI / 180.0);
  const auto result = localizer.register_scan(scan, initial, true);

  const double translation_error =
    (result.pose.block<2, 1>(0, 3) - expected.block<2, 1>(0, 3)).norm();
  const double yaw_error = std::abs(luxi_location::IcpLocalizer::normalize_angle(
      luxi_location::IcpLocalizer::yaw(result.pose) -
      luxi_location::IcpLocalizer::yaw(expected)));
  std::cout << "accepted=" << std::boolalpha << result.accepted
            << " fitness=" << result.fitness
            << " rmse=" << result.rmse
            << " translation_error=" << translation_error
            << " yaw_error_deg=" << yaw_error * 180.0 / M_PI << '\n';
  return result.accepted && translation_error < 0.03 && yaw_error < 0.02 ? 0 : 4;
}
