#include <memory>

#include "luxi_adapter/imu_level_calibrator.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<luxi_adapter::ImuLevelCalibrator>());
  rclcpp::shutdown();
  return 0;
}
