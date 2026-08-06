cmake_minimum_required(VERSION 3.16)
project(stereo_depth)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rtabmap_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
# JetPack 6 / VPI 3 installs an exported CMake target named `vpi`.  This is
# intentionally a required dependency: the default depth backend below is the
# CUDA SGM implementation, while OpenCV SGBM remains available at runtime as a
# fallback backend.
find_package(vpi CONFIG REQUIRED
  PATHS /opt/nvidia/vpi3/lib/aarch64-linux-gnu/cmake/vpi
  NO_DEFAULT_PATH)
find_package(rosidl_default_runtime REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(stereo_depth_node src/stereo_depth_node.cpp)
add_library(stereo_depth_component SHARED src/stereo_depth_node.cpp)
target_compile_definitions(stereo_depth_component PRIVATE STEREO_DEPTH_COMPONENT_BUILD)
add_executable(rgb_cloud_accumulator_node src/rgb_cloud_accumulator_node.cpp)

foreach(stereo_depth_target stereo_depth_node stereo_depth_component)
  target_include_directories(${stereo_depth_target} PRIVATE
    ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(${stereo_depth_target}
    ${OpenCV_LIBRARIES}
    vpi
    rclcpp::rclcpp
    rtabmap_msgs::rtabmap_msgs__rosidl_typesupport_cpp
    pthread
  )
  ament_target_dependencies(${stereo_depth_target}
    rclcpp
    rclcpp_components
    sensor_msgs
    std_msgs
    rtabmap_msgs
    ament_index_cpp
    geometry_msgs
    nav_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
  )
endforeach()
rclcpp_components_register_nodes(stereo_depth_component "StereoDepthNode")

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
  stereo_depth_component
  rgb_cloud_accumulator_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
ament_package()
