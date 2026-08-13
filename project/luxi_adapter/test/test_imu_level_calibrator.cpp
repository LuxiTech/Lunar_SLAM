#include <cmath>

#include "gtest/gtest.h"
#include "luxi_adapter/imu_level_calibrator.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace
{

constexpr double kTolerance = 1.0e-9;

tf2::Vector3 measured_up_for_mount(double roll, double pitch)
{
  tf2::Quaternion rotation;
  rotation.setRPY(roll, pitch, 0.0);
  return tf2::quatRotate(rotation.inverse(), tf2::Vector3(0.0, 0.0, 1.0));
}

TEST(ImuLevelCalibrator, LevelCameraProducesZeroCorrection)
{
  const auto mount = luxi_adapter::estimate_mount_angles(tf2::Vector3(0.0, 0.0, 1.0));
  EXPECT_NEAR(mount.roll, 0.0, kTolerance);
  EXPECT_NEAR(mount.pitch, 0.0, kTolerance);
}

TEST(ImuLevelCalibrator, RecoversCameraRollAndPitch)
{
  const double expected_roll = -2.0 * M_PI / 180.0;
  const double expected_pitch = 17.0 * M_PI / 180.0;
  const auto mount = luxi_adapter::estimate_mount_angles(
    measured_up_for_mount(expected_roll, expected_pitch));
  EXPECT_NEAR(mount.roll, expected_roll, kTolerance);
  EXPECT_NEAR(mount.pitch, expected_pitch, kTolerance);
}

TEST(ImuLevelCalibrator, RejectsMissingGravityVector)
{
  EXPECT_THROW(
    luxi_adapter::estimate_mount_angles(tf2::Vector3(0.0, 0.0, 0.0)),
    std::invalid_argument);
}

TEST(ImuLevelCalibrator, AcceptsStationaryGravity)
{
  EXPECT_TRUE(luxi_adapter::is_stationary_imu_sample(
    tf2::Vector3(0.01, -0.01, 0.0), tf2::Vector3(0.0, 0.0, 9.81),
    9.80665, 0.8, 0.05));
}

TEST(ImuLevelCalibrator, RejectsRotationAndLinearAcceleration)
{
  EXPECT_FALSE(luxi_adapter::is_stationary_imu_sample(
    tf2::Vector3(0.0, 0.0, 0.1), tf2::Vector3(0.0, 0.0, 9.81),
    9.80665, 0.8, 0.05));
  EXPECT_FALSE(luxi_adapter::is_stationary_imu_sample(
    tf2::Vector3(0.0, 0.0, 0.0), tf2::Vector3(5.0, 0.0, 9.81),
    9.80665, 0.8, 0.05));
}

}  // namespace
