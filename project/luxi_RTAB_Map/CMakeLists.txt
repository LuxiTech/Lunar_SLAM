cmake_minimum_required(VERSION 3.8)
project(luxi_rtab_map)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rtabmap_conversions REQUIRED)
find_package(RTABMap REQUIRED COMPONENTS core)
find_package(sensor_msgs REQUIRED)

add_executable(luxi_rtab_map_node src/luxi_rtab_map_node.cpp)
target_compile_features(luxi_rtab_map_node PUBLIC cxx_std_17)
target_include_directories(luxi_rtab_map_node
  PRIVATE
    include
    ${OpenCV_INCLUDE_DIRS}
)
ament_target_dependencies(luxi_rtab_map_node
  cv_bridge
  message_filters
  rclcpp
  rtabmap_conversions
  sensor_msgs
)
target_link_libraries(luxi_rtab_map_node
  rtabmap::core
  ${OpenCV_LIBRARIES}
)

install(TARGETS luxi_rtab_map_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch docs rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/export_3d_map.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
