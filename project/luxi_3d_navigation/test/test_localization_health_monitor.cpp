#include "luxi_3d_navigation/localization_health_monitor.hpp"
#include "luxi_3d_navigation/localization_recovery_controller.hpp"

#include <gtest/gtest.h>

TEST(LocalizationHealthMonitor, PausesTransientFaultAndAutomaticallyRecovers)
{
  luxi_3d_navigation::LocalizationHealthMonitor monitor(8.0);

  EXPECT_EQ(
    monitor.update(false, 10.0),
    luxi_3d_navigation::LocalizationHealthAction::kPause);
  EXPECT_EQ(
    monitor.update(false, 17.9),
    luxi_3d_navigation::LocalizationHealthAction::kPause);
  EXPECT_EQ(
    monitor.update(true, 18.0),
    luxi_3d_navigation::LocalizationHealthAction::kTrack);
}

TEST(LocalizationRecoveryController, DeadReckonsThenRequestsBoundedRotation)
{
  luxi_3d_navigation::LocalizationRecoveryParameters parameters;
  parameters.dead_reckoning_duration = 1.0;
  parameters.recovery_timeout = 10.0;
  luxi_3d_navigation::LocalizationRecoveryController controller(parameters);

  EXPECT_EQ(
    controller.update("degraded", 20.0),
    luxi_3d_navigation::LocalizationRecoveryAction::kDeadReckon);
  EXPECT_EQ(
    controller.update("searching", 21.0),
    luxi_3d_navigation::LocalizationRecoveryAction::kRotate);
  EXPECT_EQ(
    controller.update("searching", 30.0),
    luxi_3d_navigation::LocalizationRecoveryAction::kStop);
}

TEST(LocalizationRecoveryController, HoldsStillWhileVerifyingAndBeforeResume)
{
  luxi_3d_navigation::LocalizationRecoveryParameters parameters;
  parameters.healthy_confirmation_time = 1.0;
  luxi_3d_navigation::LocalizationRecoveryController controller(parameters);

  EXPECT_EQ(
    controller.update("searching", 10.0),
    luxi_3d_navigation::LocalizationRecoveryAction::kHold);
  EXPECT_EQ(
    controller.update("searching", 11.0),
    luxi_3d_navigation::LocalizationRecoveryAction::kRotate);
  EXPECT_EQ(
    controller.update("verifying", 11.1),
    luxi_3d_navigation::LocalizationRecoveryAction::kHold);
  EXPECT_EQ(
    controller.update("tracking", 12.0),
    luxi_3d_navigation::LocalizationRecoveryAction::kHold);
  EXPECT_EQ(
    controller.update("tracking", 13.0),
    luxi_3d_navigation::LocalizationRecoveryAction::kTrack);
}

TEST(LocalizationHealthMonitor, StopsPersistentFaultAfterGracePeriod)
{
  luxi_3d_navigation::LocalizationHealthMonitor monitor(8.0);

  EXPECT_EQ(
    monitor.update(false, 20.0),
    luxi_3d_navigation::LocalizationHealthAction::kPause);
  EXPECT_EQ(
    monitor.update(false, 28.0),
    luxi_3d_navigation::LocalizationHealthAction::kStop);
  EXPECT_EQ(
    monitor.update(true, 28.1),
    luxi_3d_navigation::LocalizationHealthAction::kStop);
  monitor.reset();
  EXPECT_EQ(
    monitor.update(true, 29.0),
    luxi_3d_navigation::LocalizationHealthAction::kTrack);
}
