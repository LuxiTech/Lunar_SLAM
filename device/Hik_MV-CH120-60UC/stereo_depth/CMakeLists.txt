cmake_minimum_required(VERSION 3.16)
project(stereo_depth)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

add_executable(stereo_depth_node src/stereo_depth_node.cpp)

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
  pthread
)

install(TARGETS stereo_depth_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
