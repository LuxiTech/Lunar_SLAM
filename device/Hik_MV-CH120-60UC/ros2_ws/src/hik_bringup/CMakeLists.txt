cmake_minimum_required(VERSION 3.16)
project(hik_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/odom_tf_republisher.py
  scripts/rtabmap_status.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
