cmake_minimum_required(VERSION 3.16)
project(evk4_driver)

find_package(ament_cmake REQUIRED)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/evk4_openeb_vendor.sh"
)

install(
  DIRECTORY config launch udev docs scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
