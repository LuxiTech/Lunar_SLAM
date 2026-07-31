#include "luxi_location/depth_projection.hpp"
#include "luxi_location/icp_localizer.hpp"
#include "luxi_location/localization_supervisor.hpp"

#include <gtest/gtest.h>
#include <open3d/Open3D.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <limits>
#include <string>
#include <vector>

TEST(DepthProjection, ProjectsMillimeterDepthIntoBaseFrame)
{
  luxi_location::CameraIntrinsics intrinsics;
  intrinsics.width = 2;
  intrinsics.height = 2;
  intrinsics.fx = 1.0;
  intrinsics.fy = 1.0;
  intrinsics.cx = 0.0;
  intrinsics.cy = 0.0;
  const std::array<std::uint16_t, 4> depth{1000, 0, 2000, 5000};
  Eigen::Matrix4d base_from_camera = Eigen::Matrix4d::Identity();
  base_from_camera(0, 3) = 1.0;
  std::string error;
  const auto points = luxi_location::project_depth_u16(
    reinterpret_cast<const std::uint8_t *>(depth.data()),
    depth.size() * sizeof(std::uint16_t), 2 * sizeof(std::uint16_t),
    intrinsics, base_from_camera, 1, 0.001, 0.2, 4.0, error);

  ASSERT_TRUE(error.empty());
  ASSERT_EQ(points.size(), 2U);
  EXPECT_TRUE(points[0].isApprox(Eigen::Vector3d(1.0, 0.0, 1.0), 1e-9));
  EXPECT_TRUE(points[1].isApprox(Eigen::Vector3d(1.0, 2.0, 2.0), 1e-9));
}

TEST(DepthProjection, ProjectsFloatingPointDepth)
{
  luxi_location::CameraIntrinsics intrinsics;
  intrinsics.width = 2;
  intrinsics.height = 1;
  intrinsics.fx = 2.0;
  intrinsics.fy = 2.0;
  intrinsics.cx = 0.0;
  intrinsics.cy = 0.0;
  const std::array<float, 2> depth{
    2.0F, std::numeric_limits<float>::quiet_NaN()};
  std::string error;
  const auto points = luxi_location::project_depth_f32(
    reinterpret_cast<const std::uint8_t *>(depth.data()),
    depth.size() * sizeof(float), 2 * sizeof(float), intrinsics,
    Eigen::Matrix4d::Identity(), 1, 0.2, 4.0, error);

  ASSERT_TRUE(error.empty());
  ASSERT_EQ(points.size(), 1U);
  EXPECT_TRUE(points[0].isApprox(Eigen::Vector3d(0.0, 0.0, 2.0), 1e-9));
}

TEST(IcpLocalizer, RecoversPlanarPose)
{
  open3d::geometry::PointCloud map;
  for (int x = 0; x < 30; ++x) {
    for (int y = 0; y < 20; ++y) {
      map.points_.emplace_back(0.05 * x, 0.05 * y, 0.02 * ((x + 2 * y) % 7));
    }
  }
  const auto map_path =
    std::filesystem::temp_directory_path() / "luxi_location_icp_test.ply";
  ASSERT_TRUE(open3d::io::WritePointCloud(map_path.string(), map));

  luxi_location::IcpParameters parameters;
  parameters.minimum_scan_points = 100;
  parameters.minimum_fitness = 0.8;
  parameters.maximum_rmse = 0.08;
  luxi_location::IcpLocalizer localizer(parameters);
  std::string error;
  ASSERT_TRUE(localizer.load_map(map_path.string(), error)) << error;

  const Eigen::Matrix4d expected =
    luxi_location::IcpLocalizer::planar_pose(0.20, -0.10, 0.0, 0.12);
  open3d::geometry::PointCloud scan = map;
  scan.Transform(expected.inverse());
  const Eigen::Matrix4d initial =
    luxi_location::IcpLocalizer::planar_pose(0.24, -0.13, 0.0, 0.15);
  const auto result = localizer.register_scan(scan, initial, true);

  EXPECT_TRUE(result.accepted) << result.reason;
  EXPECT_NEAR(result.pose(0, 3), expected(0, 3), 0.03);
  EXPECT_NEAR(result.pose(1, 3), expected(1, 3), 0.03);
  EXPECT_NEAR(
    luxi_location::IcpLocalizer::yaw(result.pose),
    luxi_location::IcpLocalizer::yaw(expected), 0.03);
}

TEST(IcpLocalizer, RejectsScanWithTooFewPoints)
{
  open3d::geometry::PointCloud map;
  for (int index = 0; index < 20; ++index) {
    map.points_.emplace_back(0.1 * index, 0.03 * (index % 4), 0.02 * (index % 3));
  }
  const auto map_path =
    std::filesystem::temp_directory_path() / "luxi_location_small_scan_test.ply";
  ASSERT_TRUE(open3d::io::WritePointCloud(map_path.string(), map));

  luxi_location::IcpParameters parameters;
  parameters.map_voxel_size = 0.01;
  parameters.minimum_scan_points = 10;
  luxi_location::IcpLocalizer localizer(parameters);
  std::string error;
  ASSERT_TRUE(localizer.load_map(map_path.string(), error)) << error;

  open3d::geometry::PointCloud scan;
  scan.points_.emplace_back(0.0, 0.0, 0.0);
  const auto result =
    localizer.register_scan(scan, Eigen::Matrix4d::Identity(), true);

  EXPECT_FALSE(result.accepted);
  EXPECT_EQ(result.reason, "scan has too few points");
}

TEST(LocalizationSupervisor, RequiresThreeMutuallyConsistentHlocPoses)
{
  luxi_location::LocalizationSupervisorParameters parameters;
  parameters.consistent_pose_count = 3;
  parameters.maximum_translation_difference = 0.5;
  parameters.maximum_yaw_difference = 20.0 * M_PI / 180.0;
  luxi_location::LocalizationSupervisor supervisor(parameters);

  const auto first =
    luxi_location::IcpLocalizer::planar_pose(1.0, 2.0, 0.0, 0.10);
  const auto second =
    luxi_location::IcpLocalizer::planar_pose(1.2, 1.9, 0.0, 0.15);
  const auto third =
    luxi_location::IcpLocalizer::planar_pose(0.9, 2.1, 0.0, 0.05);

  EXPECT_FALSE(supervisor.add_coarse_pose(first).has_value());
  EXPECT_FALSE(supervisor.add_coarse_pose(second).has_value());
  const auto accepted = supervisor.add_coarse_pose(third);
  ASSERT_TRUE(accepted.has_value());
  EXPECT_TRUE(accepted->isApprox(third));
  EXPECT_EQ(supervisor.phase(), luxi_location::LocalizationPhase::kWaitingForIcp);
}

TEST(LocalizationSupervisor, InconsistentPoseRestartsConsecutiveCount)
{
  luxi_location::LocalizationSupervisor supervisor;
  const auto first =
    luxi_location::IcpLocalizer::planar_pose(0.0, 0.0, 0.0, 0.0);
  const auto inconsistent =
    luxi_location::IcpLocalizer::planar_pose(2.0, 0.0, 0.0, 0.0);

  supervisor.add_coarse_pose(first);
  supervisor.add_coarse_pose(first);
  EXPECT_EQ(supervisor.consistent_pose_count(), 2);
  EXPECT_FALSE(supervisor.add_coarse_pose(inconsistent).has_value());
  EXPECT_EQ(supervisor.consistent_pose_count(), 1);
}

TEST(LocalizationSupervisor, DisablesHlocAfterFirstIcpSuccess)
{
  luxi_location::LocalizationSupervisor supervisor;
  const auto pose =
    luxi_location::IcpLocalizer::planar_pose(0.0, 0.0, 0.0, 0.0);
  supervisor.add_coarse_pose(pose);
  supervisor.add_coarse_pose(pose);
  ASSERT_TRUE(supervisor.add_coarse_pose(pose).has_value());

  EXPECT_EQ(
    supervisor.report_icp_result(true),
    luxi_location::HlocAction::kDisable);
  EXPECT_EQ(supervisor.phase(), luxi_location::LocalizationPhase::kTracking);
  EXPECT_EQ(
    supervisor.report_icp_result(true),
    luxi_location::HlocAction::kNone);
}

TEST(LocalizationSupervisor, RestartsHlocAfterFiveConsecutiveIcpFailures)
{
  luxi_location::LocalizationSupervisor supervisor;
  const auto pose =
    luxi_location::IcpLocalizer::planar_pose(0.0, 0.0, 0.0, 0.0);
  supervisor.add_coarse_pose(pose);
  supervisor.add_coarse_pose(pose);
  ASSERT_TRUE(supervisor.add_coarse_pose(pose).has_value());
  ASSERT_EQ(
    supervisor.report_icp_result(true),
    luxi_location::HlocAction::kDisable);

  for (int failure = 1; failure < 5; ++failure) {
    EXPECT_EQ(
      supervisor.report_icp_result(false),
      luxi_location::HlocAction::kNone);
    EXPECT_EQ(supervisor.consecutive_icp_failures(), failure);
  }
  EXPECT_EQ(
    supervisor.report_icp_result(false),
    luxi_location::HlocAction::kEnable);
  EXPECT_EQ(supervisor.phase(), luxi_location::LocalizationPhase::kSearching);
  EXPECT_EQ(supervisor.consistent_pose_count(), 0);
  EXPECT_EQ(supervisor.consecutive_icp_failures(), 0);
}

TEST(LocalizationSupervisor, SuccessfulTrackingResetsFailureCount)
{
  luxi_location::LocalizationSupervisor supervisor;
  const auto pose =
    luxi_location::IcpLocalizer::planar_pose(0.0, 0.0, 0.0, 0.0);
  supervisor.add_coarse_pose(pose);
  supervisor.add_coarse_pose(pose);
  ASSERT_TRUE(supervisor.add_coarse_pose(pose).has_value());
  supervisor.report_icp_result(true);

  supervisor.report_icp_result(false);
  supervisor.report_icp_result(false);
  EXPECT_EQ(supervisor.consecutive_icp_failures(), 2);
  EXPECT_EQ(
    supervisor.report_icp_result(true),
    luxi_location::HlocAction::kNone);
  EXPECT_EQ(supervisor.consecutive_icp_failures(), 0);
}
