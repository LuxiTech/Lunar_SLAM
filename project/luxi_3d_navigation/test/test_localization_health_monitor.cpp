#include "luxi_3d_navigation/localization_health_monitor.hpp"

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
