cmake_minimum_required(VERSION 3.16)
project(evk4_driver)

find_package(ament_cmake REQUIRED)

get_filename_component(ULTIMATE_SLAM_WS "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
set(EVK4_LOCAL_ROS "${ULTIMATE_SLAM_WS}/.local_ros/opt/ros/lyrical")
set(EVK4_OPENEB_VENDOR "${EVK4_LOCAL_ROS}/opt/openeb_vendor")

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/evk4_openeb_vendor.sh"
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/evk4_openeb_vendor.dsv.in"
)

install(
  DIRECTORY config launch udev docs scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
