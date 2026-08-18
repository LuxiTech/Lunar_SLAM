#include <gtest/gtest.h>

#include "luxi_3d_navigation/navigation_safety_gate.hpp"

namespace
{

geometry_msgs::msg::Twist movingCommand()
{
  geometry_msgs::msg::Twist command;
  command.linear.x = 0.10;
  command.angular.z = 0.20;
  return command;
}

luxi_3d_navigation::NavigationSafetyGate readyGate()
{
  luxi_3d_navigation::SafetyGateParameters parameters;
  parameters.healthy_resume_hold = 0.10;
  luxi_3d_navigation::NavigationSafetyGate gate(parameters);
  gate.setNavigationActive(true, 1.0);
  gate.updateCommand(movingCommand(), 1.0);
  gate.updateObstacleState("clear", 1.0);
  gate.updatePlannerState("clear", 1.0);
  gate.updateLocalizationState("tracking", 1.0);
  return gate;
}

TEST(NavigationSafetyGate, PassesOnlyAfterHealthyHold)
{
  auto gate = readyGate();
  EXPECT_DOUBLE_EQ(gate.evaluate(1.05).command.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(gate.evaluate(1.16).command.linear.x, 0.10);
}

TEST(NavigationSafetyGate, BlocksWhileObstacleOrPlannerHasNoPath)
{
  auto gate = readyGate();
  gate.evaluate(1.0);
  gate.updateObstacleState("blocked", 1.05);
  EXPECT_EQ(gate.evaluate(1.06).state, "blocked");
  EXPECT_DOUBLE_EQ(gate.evaluate(1.06).command.linear.x, 0.0);
  gate.updateObstacleState("clear", 1.07);
  gate.updatePlannerState("no_path", 1.07);
  EXPECT_EQ(gate.evaluate(1.08).state, "no_path");
}

TEST(NavigationSafetyGate, SlowModePreservesD1MinimumEffectiveSpeed)
{
  luxi_3d_navigation::SafetyGateParameters parameters;
  parameters.slow_scale = 0.50;
  parameters.minimum_linear_speed = 0.10;
  parameters.healthy_resume_hold = 0.0;
  luxi_3d_navigation::NavigationSafetyGate gate(parameters);
  gate.setNavigationActive(true, 1.0);
  gate.updateCommand(movingCommand(), 1.0);
  gate.updateObstacleState("slow", 1.0);
  gate.updatePlannerState("ready", 1.0);
  gate.updateLocalizationState("tracking", 1.0);
  const auto result = gate.evaluate(1.01);
  EXPECT_EQ(result.state, "slow");
  EXPECT_DOUBLE_EQ(result.command.linear.x, 0.10);
  EXPECT_TRUE(result.limited);
}

TEST(NavigationSafetyGate, SensorTimeoutLatchesHardStopUntilNavigationRestarts)
{
  auto gate = readyGate();
  const auto stopped = gate.evaluate(1.40);
  EXPECT_EQ(stopped.state, "hard_stop");
  EXPECT_TRUE(stopped.emergency_stop);
  gate.updateObstacleState("clear", 1.41);
  EXPECT_EQ(gate.evaluate(1.42).state, "hard_stop");
  gate.setNavigationActive(false, 1.43);
  EXPECT_FALSE(gate.evaluate(1.44).emergency_stop);
}

TEST(NavigationSafetyGate, BriefSensorTimeoutStopsThenRecoversWithoutRestart)
{
  luxi_3d_navigation::SafetyGateParameters parameters;
  parameters.command_timeout = 5.0;
  parameters.obstacle_timeout = 0.20;
  parameters.obstacle_hard_stop_latch_delay = 1.0;
  parameters.planner_timeout = 5.0;
  parameters.localization_timeout = 5.0;
  parameters.healthy_resume_hold = 0.10;
  luxi_3d_navigation::NavigationSafetyGate gate(parameters);
  gate.setNavigationActive(true, 1.0);
  gate.updateCommand(movingCommand(), 1.0);
  gate.updateObstacleState("clear", 1.0);
  gate.updatePlannerState("clear", 1.0);
  gate.updateLocalizationState("tracking", 1.0);
  EXPECT_EQ(gate.evaluate(1.0).state, "recovery_hold");
  EXPECT_DOUBLE_EQ(gate.evaluate(1.15).command.linear.x, 0.10);

  const auto stopped = gate.evaluate(1.21);
  EXPECT_EQ(stopped.state, "sensor_stale_stop");
  EXPECT_DOUBLE_EQ(stopped.command.linear.x, 0.0);
  EXPECT_FALSE(stopped.emergency_stop);

  gate.updateObstacleState("clear", 1.80);
  EXPECT_EQ(gate.evaluate(1.81).state, "recovery_hold");
  EXPECT_DOUBLE_EQ(gate.evaluate(1.92).command.linear.x, 0.10);
}

TEST(NavigationSafetyGate, PersistentSensorTimeoutStillLatchesHardStop)
{
  luxi_3d_navigation::SafetyGateParameters parameters;
  parameters.command_timeout = 5.0;
  parameters.obstacle_timeout = 0.20;
  parameters.obstacle_hard_stop_latch_delay = 1.0;
  parameters.planner_timeout = 5.0;
  parameters.localization_timeout = 5.0;
  luxi_3d_navigation::NavigationSafetyGate gate(parameters);
  gate.setNavigationActive(true, 1.0);
  gate.updateCommand(movingCommand(), 1.0);
  gate.updateObstacleState("clear", 1.0);
  gate.updatePlannerState("clear", 1.0);
  gate.updateLocalizationState("tracking", 1.0);
  EXPECT_EQ(gate.evaluate(1.21).state, "sensor_stale_stop");
  const auto latched = gate.evaluate(2.22);
  EXPECT_EQ(latched.state, "hard_stop");
  EXPECT_TRUE(latched.emergency_stop);
  gate.updateObstacleState("clear", 2.23);
  EXPECT_EQ(gate.evaluate(2.24).state, "hard_stop");
}

TEST(NavigationSafetyGate, UnknownSensorAndPlannerStatesFailClosed)
{
  auto gate = readyGate();
  gate.updateObstacleState("depth_stamp_stale", 1.01);
  EXPECT_TRUE(gate.evaluate(1.02).emergency_stop);

  auto planner_gate = readyGate();
  planner_gate.updatePlannerState("waiting_map", 1.01);
  EXPECT_EQ(planner_gate.evaluate(1.02).state, "blocked");
  EXPECT_DOUBLE_EQ(planner_gate.evaluate(1.02).command.linear.x, 0.0);
}

TEST(NavigationSafetyGate, LocalizationLossDeadReckonsBrieflyAndRequiresHealthyHold)
{
  auto gate = readyGate();
  EXPECT_EQ(gate.evaluate(1.05).state, "recovery_hold");
  EXPECT_DOUBLE_EQ(gate.evaluate(1.16).command.linear.x, 0.10);
  gate.updateLocalizationState("degraded", 1.17);
  EXPECT_EQ(gate.evaluate(1.18).state, "localization_dead_reckoning");
  EXPECT_DOUBLE_EQ(gate.evaluate(1.18).command.linear.x, 0.05);
  gate.updateLocalizationState("tracking", 1.19);
  EXPECT_EQ(gate.evaluate(1.20).state, "recovery_hold");
  gate.updateCommand(movingCommand(), 1.30);
  gate.updateObstacleState("clear", 1.30);
  gate.updatePlannerState("clear", 1.30);
  gate.updateLocalizationState("tracking", 1.30);
  EXPECT_DOUBLE_EQ(gate.evaluate(1.31).command.linear.x, 0.10);
}

TEST(NavigationSafetyGate, StaleLocalizationStopsWithoutLatchingRestart)
{
  auto gate = readyGate();
  EXPECT_EQ(gate.evaluate(2.60).state, "hard_stop");

  luxi_3d_navigation::SafetyGateParameters parameters;
  parameters.localization_timeout = 0.2;
  parameters.obstacle_timeout = 2.0;
  parameters.planner_timeout = 2.0;
  parameters.command_timeout = 2.0;
  parameters.healthy_resume_hold = 0.0;
  luxi_3d_navigation::NavigationSafetyGate localization_gate(parameters);
  localization_gate.setNavigationActive(true, 1.0);
  localization_gate.updateCommand(movingCommand(), 1.0);
  localization_gate.updateObstacleState("clear", 1.0);
  localization_gate.updatePlannerState("clear", 1.0);
  localization_gate.updateLocalizationState("tracking", 1.0);
  EXPECT_EQ(localization_gate.evaluate(1.21).state, "localization_stale");
  localization_gate.updateLocalizationState("tracking", 1.22);
  EXPECT_DOUBLE_EQ(localization_gate.evaluate(1.23).command.linear.x, 0.10);
}

TEST(NavigationSafetyGate, MonitorOnlyReportsFaultButNeverRaisesEmergency)
{
  auto gate = readyGate();
  const auto result = gate.evaluate(1.40, true);
  EXPECT_EQ(result.state, "monitor_sensor_fault");
  EXPECT_FALSE(result.emergency_stop);
}

TEST(NavigationSafetyGate, AllowsOnlyBriefScaledDeadReckoning)
{
  luxi_3d_navigation::SafetyGateParameters parameters;
  parameters.command_timeout = 2.0;
  parameters.obstacle_timeout = 2.0;
  parameters.planner_timeout = 2.0;
  parameters.localization_timeout = 2.0;
  parameters.dead_reckoning_duration = 1.0;
  parameters.dead_reckoning_scale = 0.5;
  parameters.healthy_resume_hold = 0.0;
  luxi_3d_navigation::NavigationSafetyGate gate(parameters);
  gate.setNavigationActive(true, 1.0);
  gate.updateCommand(movingCommand(), 1.0);
  gate.updateObstacleState("clear", 1.0);
  gate.updatePlannerState("clear", 1.0);
  gate.updateLocalizationState("tracking", 1.0);
  gate.updateLocalizationState("dead_reckoning", 1.1);

  const auto continued = gate.evaluate(1.2);
  EXPECT_EQ(continued.state, "localization_dead_reckoning");
  EXPECT_DOUBLE_EQ(continued.command.linear.x, 0.05);
  EXPECT_DOUBLE_EQ(gate.evaluate(2.2).command.linear.x, 0.0);
}

TEST(NavigationSafetyGate, RecoveryRotationIsAngularOnlyAndObstacleGated)
{
  luxi_3d_navigation::SafetyGateParameters parameters;
  parameters.command_timeout = 1.0;
  parameters.obstacle_timeout = 2.0;
  parameters.planner_timeout = 2.0;
  parameters.localization_timeout = 2.0;
  parameters.maximum_recovery_angular_speed = 0.20;
  luxi_3d_navigation::NavigationSafetyGate gate(parameters);
  gate.setNavigationActive(true, 1.0);
  gate.updateCommand(movingCommand(), 1.0);
  gate.updateObstacleState("clear", 1.0);
  gate.updatePlannerState("no_path", 1.0);
  gate.updateLocalizationState("searching", 1.0);
  gate.updateRecoveryActive(true, 1.0);

  const auto rotating = gate.evaluate(1.1);
  EXPECT_EQ(rotating.state, "localization_recovery_spin");
  EXPECT_DOUBLE_EQ(rotating.command.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(rotating.command.angular.z, 0.20);
  gate.updateObstacleState("slow", 1.2);
  EXPECT_DOUBLE_EQ(gate.evaluate(1.21).command.angular.z, 0.20);
  gate.updateObstacleState("blocked", 1.3);
  EXPECT_DOUBLE_EQ(gate.evaluate(1.31).command.angular.z, 0.0);
}

TEST(NavigationSafetyGate, RecoveryRotationCanRestoreStaleLocalization)
{
  luxi_3d_navigation::SafetyGateParameters parameters;
  parameters.command_timeout = 1.0;
  parameters.obstacle_timeout = 2.0;
  parameters.localization_timeout = 0.2;
  parameters.maximum_recovery_angular_speed = 0.20;
  luxi_3d_navigation::NavigationSafetyGate gate(parameters);
  gate.setNavigationActive(true, 1.0);
  gate.updateCommand(movingCommand(), 1.3);
  gate.updateObstacleState("clear", 1.3);
  gate.updatePlannerState("no_path", 1.0);
  gate.updateLocalizationState("tracking", 1.0);
  gate.updateRecoveryActive(true, 1.3);

  const auto rotating = gate.evaluate(1.31);
  EXPECT_EQ(rotating.state, "localization_recovery_spin");
  EXPECT_DOUBLE_EQ(rotating.command.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(rotating.command.angular.z, 0.20);
}

}  // namespace
