cmake_minimum_required(VERSION 3.8)
project(luxi_voxel_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(OCTOMAP_ROOT "/home/lunar/project/lunar_slam/3parts/octomap/install" CACHE PATH
  "Prefix of the project-local OctoMap installation")
list(PREPEND CMAKE_PREFIX_PATH "${OCTOMAP_ROOT}")

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(octomap CONFIG REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(voxel_astar src/voxel_astar.cpp)
target_compile_features(voxel_astar PUBLIC cxx_std_17)
target_include_directories(voxel_astar PUBLIC include)

add_executable(cloud_to_octomap_node src/cloud_to_octomap_node.cpp)
target_compile_features(cloud_to_octomap_node PUBLIC cxx_std_17)
ament_target_dependencies(cloud_to_octomap_node
  octomap_msgs rclcpp sensor_msgs)
target_link_libraries(cloud_to_octomap_node octomap::octomap)

add_executable(ply_to_octomap src/ply_to_octomap.cpp)
target_compile_features(ply_to_octomap PUBLIC cxx_std_17)
target_include_directories(ply_to_octomap PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(ply_to_octomap octomap::octomap ${PCL_LIBRARIES})

add_executable(octomap_file_loader_node src/octomap_file_loader_node.cpp)
target_compile_features(octomap_file_loader_node PUBLIC cxx_std_17)
ament_target_dependencies(octomap_file_loader_node
  geometry_msgs octomap_msgs rclcpp std_msgs visualization_msgs)
target_link_libraries(octomap_file_loader_node octomap::octomap)

add_executable(octomap_to_points src/octomap_to_points.cpp)
target_compile_features(octomap_to_points PUBLIC cxx_std_17)
target_link_libraries(octomap_to_points octomap::octomap)

add_executable(octomap_astar_planner_node src/octomap_astar_planner_node.cpp)
target_compile_features(octomap_astar_planner_node PUBLIC cxx_std_17)
target_include_directories(octomap_astar_planner_node PRIVATE include)
ament_target_dependencies(octomap_astar_planner_node
  geometry_msgs nav_msgs octomap_msgs rclcpp tf2 tf2_geometry_msgs tf2_ros)
target_link_libraries(octomap_astar_planner_node voxel_astar octomap::octomap)

add_executable(path_follower_node src/path_follower_node.cpp)
target_compile_features(path_follower_node PUBLIC cxx_std_17)
ament_target_dependencies(path_follower_node
  geometry_msgs nav_msgs rclcpp std_msgs tf2 tf2_geometry_msgs tf2_ros)

install(TARGETS
  cloud_to_octomap_node
  ply_to_octomap
  octomap_file_loader_node
  octomap_to_points
  octomap_astar_planner_node
  path_follower_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_voxel_astar test/test_voxel_astar.cpp)
  target_link_libraries(test_voxel_astar voxel_astar)
  target_include_directories(test_voxel_astar PRIVATE include)
endif()

ament_package()
