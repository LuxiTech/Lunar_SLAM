#include "luxi_location/depth_projection.hpp"
#include "luxi_location/icp_localizer.hpp"
#include "luxi_location/initial_pose_fusion.hpp"
#include "luxi_location/localization_supervisor.hpp"
#include "luxi_location/map_odom_alignment.hpp"
#include "luxi_location/command_gated_odometry.hpp"
#include "luxi_location/tracking_pose_gate.hpp"
#include "luxi_location/yaw_motion_predictor.hpp"

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

TEST(IcpLocalizer, FiltersTemporaryObjectsDuringTracking)
{
  open3d::geometry::PointCloud map;
  for (int x = 0; x < 30; ++x) {
    for (int y = 0; y < 20; ++y) {
      map.points_.emplace_back(
        0.05 * x, 0.05 * y, 0.015 * ((3 * x + 2 * y) % 9));
    }
  }
  const auto map_path =
    std::filesystem::temp_directory_path() / "luxi_location_dynamic_filter_test.ply";
  ASSERT_TRUE(open3d::io::WritePointCloud(map_path.string(), map));

  luxi_location::IcpParameters parameters;
  parameters.map_voxel_size = 0.03;
  parameters.scan_voxel_size = 0.03;
  parameters.coarse_voxel_size = 0.08;
  parameters.minimum_scan_points = 100;
  parameters.minimum_fitness = 0.70;
  parameters.maximum_rmse = 0.08;
  parameters.tracking_static_filter_distance = 0.10;
  parameters.tracking_minimum_static_point_ratio = 0.30;
  luxi_location::IcpLocalizer localizer(parameters);
  std::string error;
  ASSERT_TRUE(localizer.load_map(map_path.string(), error)) << error;

  open3d::geometry::PointCloud scan = map;
  for (int x = 0; x < 15; ++x) {
    for (int y = 0; y < 15; ++y) {
      scan.points_.emplace_back(4.0 + 0.03 * x, -1.0 + 0.03 * y, 0.30);
    }
  }
  const auto result = localizer.register_scan(scan, Eigen::Matrix4d::Identity(), false);

  EXPECT_TRUE(result.accepted) << result.reason;
  EXPECT_GT(result.static_point_ratio, 0.50);
  EXPECT_LT(result.static_point_ratio, 0.90);
  EXPECT_NEAR(result.pose(0, 3), 0.0, 0.02);
  EXPECT_NEAR(result.pose(1, 3), 0.0, 0.02);
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

TEST(LocalizationSupervisor, ExplicitRecoveryRestartsHlocFromTracking)
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

  supervisor.report_icp_result(false);
  supervisor.force_relocalization();

  EXPECT_EQ(supervisor.phase(), luxi_location::LocalizationPhase::kSearching);
  EXPECT_EQ(supervisor.consistent_pose_count(), 0);
  EXPECT_EQ(supervisor.consecutive_icp_failures(), 0);
  EXPECT_FALSE(supervisor.add_coarse_pose(pose).has_value());
  EXPECT_EQ(supervisor.consistent_pose_count(), 1);
}

TEST(LocalizationRecovery, KeepsOdometryOnlyWhenRejectedScanStillOverlapsMap)
{
  EXPECT_TRUE(luxi_location::should_retain_odometry_after_rejected_icp(
    false, false, false, true, 0.61, 0.25, 1.0, 0.65));
  EXPECT_FALSE(luxi_location::should_retain_odometry_after_rejected_icp(
    false, false, false, true, 0.0, 0.25, 1.0, 0.65));
  EXPECT_FALSE(luxi_location::should_retain_odometry_after_rejected_icp(
    false, false, false, true,
    std::numeric_limits<double>::quiet_NaN(), 0.25, 1.0, 0.65));
}

TEST(LocalizationRecovery, DoesNotBypassExplicitRelocalizationPolicy)
{
  EXPECT_FALSE(luxi_location::should_retain_odometry_after_rejected_icp(
    false, false, true, true, 0.80, 0.25, 1.0, 0.65));
  EXPECT_FALSE(luxi_location::should_retain_odometry_after_rejected_icp(
    true, false, false, true, 0.80, 0.25, 1.0, 0.65));
  EXPECT_FALSE(luxi_location::should_retain_odometry_after_rejected_icp(
    false, true, false, true, 0.80, 0.25, 1.0, 0.65));
}

TEST(LocalizationRecovery, DynamicOcclusionDoesNotRestartGlobalLocalization)
{
  EXPECT_TRUE(luxi_location::should_retain_odometry_after_rejected_icp(
    false, false, true, true, 1.0, 0.25, 0.44, 0.65));
  EXPECT_FALSE(luxi_location::should_retain_odometry_after_rejected_icp(
    false, false, true, true, 1.0, 0.25, 0.90, 0.65));
}

TEST(TrackingPoseGate, RejectsSinglePhysicallyImpossibleJumpAndAcceptsRecovery)
{
  luxi_location::TrackingPoseGateParameters parameters;
  parameters.maximum_linear_speed = 0.20;
  parameters.translation_margin = 0.08;
  luxi_location::TrackingPoseGate gate(parameters);
  gate.reset(luxi_location::IcpLocalizer::planar_pose(0.0, 0.0, 0.0, 0.0), 10.0);

  const auto normal = gate.evaluate(
    luxi_location::IcpLocalizer::planar_pose(0.08, 0.0, 0.0, 0.0), 10.5);
  EXPECT_TRUE(normal.accepted) << normal.reason;

  const auto runaway = gate.evaluate(
    luxi_location::IcpLocalizer::planar_pose(0.65, 0.0, 0.0, 0.0), 11.0);
  EXPECT_FALSE(runaway.accepted);
  EXPECT_EQ(runaway.reason, "translation jump exceeds physical limit");

  const auto recovered = gate.evaluate(
    luxi_location::IcpLocalizer::planar_pose(0.15, 0.0, 0.0, 0.0), 11.5);
  EXPECT_TRUE(recovered.accepted) << recovered.reason;
}

TEST(TrackingPoseGate, RejectsYawJumpAndOutOfOrderMeasurement)
{
  luxi_location::TrackingPoseGate gate;
  gate.reset(luxi_location::IcpLocalizer::planar_pose(0.0, 0.0, 0.0, 0.0), 1.0);

  const auto yaw_jump = gate.evaluate(
    luxi_location::IcpLocalizer::planar_pose(0.0, 0.0, 0.0, 1.0), 1.5);
  EXPECT_FALSE(yaw_jump.accepted);
  EXPECT_EQ(yaw_jump.reason, "yaw jump exceeds physical limit");

  const auto old = gate.evaluate(
    luxi_location::IcpLocalizer::planar_pose(0.0, 0.0, 0.0, 0.0), 0.9);
  EXPECT_FALSE(old.accepted);
  EXPECT_EQ(old.reason, "pose timestamp is not newer than the accepted pose");
}

TEST(TrackingPoseGate, RelocalizationMustReconnectToLastTrustedPose)
{
  luxi_location::TrackingPoseGateParameters parameters;
  parameters.maximum_relocalization_translation = 0.40;
  parameters.maximum_relocalization_yaw = 30.0 * M_PI / 180.0;
  luxi_location::TrackingPoseGate gate(parameters);
  gate.reset(luxi_location::IcpLocalizer::planar_pose(-2.2, -0.4, 0.0, 0.8), 1.0);

  const auto runaway = gate.evaluate_relocalization(
    luxi_location::IcpLocalizer::planar_pose(-1.62, -0.4, 0.0, 0.8), 30.0);
  EXPECT_FALSE(runaway.accepted);
  EXPECT_EQ(
    runaway.reason,
    "relocalized translation is inconsistent with the last trusted pose");

  const auto recovered = gate.evaluate_relocalization(
    luxi_location::IcpLocalizer::planar_pose(-2.05, -0.45, 0.0, 0.9), 31.0);
  EXPECT_TRUE(recovered.accepted) << recovered.reason;
}

TEST(TrackingPoseGate, AcceptsNearbyPoseAfterLongMeasurementGap)
{
  luxi_location::TrackingPoseGate gate;
  gate.reset(luxi_location::IcpLocalizer::planar_pose(0.0, 0.0, 0.0, 0.0), 1.0);

  const auto recovered = gate.evaluate(
    luxi_location::IcpLocalizer::planar_pose(0.25, 0.0, 0.0, 0.1), 6.0);
  EXPECT_TRUE(recovered.accepted) << recovered.reason;
}

TEST(TrackingPoseGate, RelocalizationUsesContinuousOdometryDuringOutage)
{
  luxi_location::TrackingPoseGateParameters parameters;
  parameters.maximum_relocalization_translation = 0.40;
  luxi_location::TrackingPoseGate gate(parameters);
  gate.reset(luxi_location::IcpLocalizer::planar_pose(0.0, 0.0, 0.0, 0.0), 1.0);

  const auto odometry_prediction =
    luxi_location::IcpLocalizer::planar_pose(1.0, 0.0, 0.0, 0.0);
  const auto global_recovery =
    luxi_location::IcpLocalizer::planar_pose(1.06, 0.0, 0.0, 0.02);
  const auto decision = gate.evaluate_relocalization(
    global_recovery, odometry_prediction, 8.0);

  EXPECT_TRUE(decision.accepted) << decision.reason;
  EXPECT_NEAR(decision.translation_delta, 0.06, 1e-9);
}

TEST(YawMotionPredictor, AppliesWrappedImuYawDeltaWithoutChangingPosition)
{
  luxi_location::YawMotionPredictor predictor(0.7);
  predictor.update(3.10);
  predictor.anchor();
  predictor.update(-3.08);
  const auto pose =
    luxi_location::IcpLocalizer::planar_pose(1.0, 2.0, 0.3, 0.20);
  const auto predicted = predictor.predict(pose);

  EXPECT_NEAR(predicted(0, 3), 1.0, 1e-9);
  EXPECT_NEAR(predicted(1, 3), 2.0, 1e-9);
  EXPECT_NEAR(predicted(2, 3), 0.3, 1e-9);
  EXPECT_NEAR(
    luxi_location::IcpLocalizer::yaw(predicted),
    0.20 + (-3.08 - 3.10 + 2.0 * M_PI), 1e-9);
}

TEST(YawMotionPredictor, IgnoresImplausiblyLargeDelta)
{
  luxi_location::YawMotionPredictor predictor(0.3);
  predictor.update(0.0);
  predictor.anchor();
  predictor.update(1.0);
  const auto pose =
    luxi_location::IcpLocalizer::planar_pose(0.0, 0.0, 0.0, 0.2);

  EXPECT_TRUE(predictor.predict(pose).isApprox(pose, 1e-9));
}

TEST(MapOdomAlignment, UsesContinuousOdometryInsteadOfRepeatedGlobalIcpMotion)
{
  luxi_location::MapOdomAlignment alignment(0.05, 0.10, 10.0 * M_PI / 180.0);
  const auto initial_map_base =
    luxi_location::IcpLocalizer::planar_pose(-1.2, -1.0, 0.0, 0.8);
  alignment.initialize(initial_map_base, Eigen::Matrix4d::Identity());

  const auto odom_after_motion =
    luxi_location::IcpLocalizer::planar_pose(0.05, 0.0, 0.0, 0.0);
  const auto predicted = alignment.predict(odom_after_motion);

  EXPECT_NEAR(predicted(0, 3), -1.2 + 0.05 * std::cos(0.8), 1e-9);
  EXPECT_NEAR(predicted(1, 3), -1.0 + 0.05 * std::sin(0.8), 1e-9);
  EXPECT_NEAR(luxi_location::IcpLocalizer::yaw(predicted), 0.8, 1e-9);
}

TEST(MapOdomAlignment, RejectsRunawayIcpWithoutMovingTrustedAlignment)
{
  luxi_location::MapOdomAlignment alignment(0.05, 0.10, 10.0 * M_PI / 180.0);
  const auto initial =
    luxi_location::IcpLocalizer::planar_pose(1.0, 2.0, 0.0, 0.2);
  alignment.initialize(initial, Eigen::Matrix4d::Identity());

  const auto runaway =
    luxi_location::IcpLocalizer::planar_pose(1.35, 2.0, 0.0, 0.2);
  const auto decision = alignment.correct(runaway, Eigen::Matrix4d::Identity());

  EXPECT_FALSE(decision.accepted);
  EXPECT_EQ(decision.reason, "ICP translation disagrees with local odometry");
  EXPECT_TRUE(alignment.predict(Eigen::Matrix4d::Identity()).isApprox(initial, 1e-9));
}

TEST(MapOdomAlignment, AppliesOnlyBoundedFractionOfAcceptedIcpCorrection)
{
  luxi_location::MapOdomAlignment alignment(0.10, 0.10, 10.0 * M_PI / 180.0);
  alignment.initialize(Eigen::Matrix4d::Identity(), Eigen::Matrix4d::Identity());
  const auto measured =
    luxi_location::IcpLocalizer::planar_pose(0.08, -0.04, 0.0, 0.05);

  const auto decision = alignment.correct(measured, Eigen::Matrix4d::Identity());
  ASSERT_TRUE(decision.accepted) << decision.reason;
  const auto corrected = alignment.predict(Eigen::Matrix4d::Identity());
  EXPECT_NEAR(corrected(0, 3), 0.008, 1e-9);
  EXPECT_NEAR(corrected(1, 3), -0.004, 1e-9);
  EXPECT_NEAR(luxi_location::IcpLocalizer::yaw(corrected), 0.005, 1e-9);
}

TEST(CommandGatedOdometry, FreezesVisualDriftWhileVelocityCommandIsZero)
{
  luxi_location::CommandGatedOdometry odometry;
  const auto origin = luxi_location::IcpLocalizer::planar_pose(0.0, 0.0, 0.0, 0.0);
  const auto drift = luxi_location::IcpLocalizer::planar_pose(0.08, -0.03, 0.0, 0.04);

  EXPECT_TRUE(odometry.update(origin, false).isApprox(origin, 1e-9));
  EXPECT_TRUE(odometry.update(drift, false).isApprox(origin, 1e-9));
}

TEST(CommandGatedOdometry, StartsFromLatestRawReferenceWithoutReleaseJump)
{
  luxi_location::CommandGatedOdometry odometry;
  odometry.update(Eigen::Matrix4d::Identity(), false);
  const auto drift = luxi_location::IcpLocalizer::planar_pose(0.08, 0.0, 0.0, 0.0);
  odometry.update(drift, false);
  const auto moved = luxi_location::IcpLocalizer::planar_pose(0.13, 0.0, 0.0, 0.0);

  const auto filtered = odometry.update(moved, true);
  EXPECT_NEAR(filtered(0, 3), 0.05, 1e-9);
  EXPECT_NEAR(filtered(1, 3), 0.0, 1e-9);
}

TEST(InitialPoseFusion, PreservesTrustedTranslationOnPlanarMap)
{
  const auto trusted =
    luxi_location::IcpLocalizer::planar_pose(-0.51, -1.35, -0.08, 1.82);
  const auto refined =
    luxi_location::IcpLocalizer::planar_pose(-0.12, -1.34, -0.19, 1.93);

  const auto fused = luxi_location::initialPoseForMapAlignment(trusted, refined, true);

  EXPECT_NEAR(fused(0, 3), trusted(0, 3), 1e-9);
  EXPECT_NEAR(fused(1, 3), trusted(1, 3), 1e-9);
  EXPECT_NEAR(fused(2, 3), trusted(2, 3), 1e-9);
  EXPECT_NEAR(
    luxi_location::IcpLocalizer::yaw(fused),
    luxi_location::IcpLocalizer::yaw(refined), 1e-9);
}

TEST(InitialPoseFusion, CanUseFullIcpTranslationWhenMapHasEnoughGeometry)
{
  const auto trusted =
    luxi_location::IcpLocalizer::planar_pose(-0.51, -1.35, -0.08, 1.82);
  const auto refined =
    luxi_location::IcpLocalizer::planar_pose(-0.48, -1.31, -0.07, 1.84);

  EXPECT_TRUE(
    luxi_location::initialPoseForMapAlignment(trusted, refined, false)
    .isApprox(refined, 1e-9));
}
