cmake_minimum_required(VERSION 3.16)
project(stereo_depth)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rtabmap_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(stereo_depth_node src/stereo_depth_node.cpp)
add_executable(rgb_cloud_accumulator_node src/rgb_cloud_accumulator_node.cpp)

target_include_directories(stereo_depth_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(stereo_depth_node
  ${OpenCV_LIBRARIES}
  rclcpp::rclcpp
  cv_bridge::cv_bridge
  rtabmap_msgs::rtabmap_msgs__rosidl_typesupport_cpp
  pthread
)

ament_target_dependencies(stereo_depth_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  rtabmap_msgs
  ament_index_cpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

target_include_directories(rgb_cloud_accumulator_node PRIVATE
)

target_link_libraries(rgb_cloud_accumulator_node
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  pthread
)

ament_target_dependencies(rgb_cloud_accumulator_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS
  stereo_depth_node
  rgb_cloud_accumulator_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
