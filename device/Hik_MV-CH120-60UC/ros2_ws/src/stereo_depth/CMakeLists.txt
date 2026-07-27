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
  /opt/ros/lyrical/include
  /opt/ros/lyrical/include/rclcpp
  /opt/ros/lyrical/include/sensor_msgs
  /opt/ros/lyrical/include/std_msgs
  /opt/ros/lyrical/include/cv_bridge
)

target_link_libraries(stereo_depth_node
  ${OpenCV_LIBRARIES}
  rclcpp::rclcpp
  cv_bridge::cv_bridge
  rtabmap_msgs::rtabmap_msgs__rosidl_typesupport_cpp
  pthread
)

target_include_directories(rgb_cloud_accumulator_node PRIVATE
  /opt/ros/lyrical/include
  /opt/ros/lyrical/include/rclcpp
  /opt/ros/lyrical/include/sensor_msgs
  /opt/ros/lyrical/include/geometry_msgs
  /opt/ros/lyrical/include/nav_msgs
  /opt/ros/lyrical/include/tf2
  /opt/ros/lyrical/include/tf2_geometry_msgs
  /opt/ros/lyrical/include/tf2_ros
)

target_link_libraries(rgb_cloud_accumulator_node
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
  pthread
)

install(TARGETS
  stereo_depth_node
  rgb_cloud_accumulator_node
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
