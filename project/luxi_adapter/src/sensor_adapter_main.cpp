#include "luxi_adapter/sensor_adapter.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<luxi_adapter::SensorAdapter>());
  rclcpp::shutdown();
  return 0;
}
