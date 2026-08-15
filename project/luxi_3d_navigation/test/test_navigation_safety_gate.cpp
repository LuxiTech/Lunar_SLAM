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

TEST(NavigationSafetyGate, MonitorOnlyReportsFaultButNeverRaisesEmergency)
{
  auto gate = readyGate();
  const auto result = gate.evaluate(1.40, true);
  EXPECT_EQ(result.state, "monitor_sensor_fault");
  EXPECT_FALSE(result.emergency_stop);
}

}  // namespace
