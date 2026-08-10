cmake_minimum_required(VERSION 3.16)
project(usb_camera_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/install_h30_udev_rule.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
