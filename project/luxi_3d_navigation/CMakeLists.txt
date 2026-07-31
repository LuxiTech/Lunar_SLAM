cmake_minimum_required(VERSION 3.8)
project(luxi_3d_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(OCTOMAP_ROOT "/home/lunar/project/lunar_slam/3parts/octomap/install" CACHE PATH
  "Prefix of the project-local OctoMap installation")
list(PREPEND CMAKE_PREFIX_PATH "${OCTOMAP_ROOT}")

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(octomap CONFIG REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(terrain_planning
  src/astar_3d.cpp
  src/terrain_model.cpp)
target_compile_features(terrain_planning PUBLIC cxx_std_17)
target_include_directories(terrain_planning PUBLIC include)
target_link_libraries(terrain_planning PUBLIC octomap::octomap)

add_executable(octomap_3d_astar_planner_node src/octomap_3d_astar_planner_node.cpp)
target_compile_features(octomap_3d_astar_planner_node PUBLIC cxx_std_17)
target_include_directories(octomap_3d_astar_planner_node PRIVATE include)
ament_target_dependencies(octomap_3d_astar_planner_node
  geometry_msgs nav_msgs octomap_msgs rclcpp tf2 tf2_geometry_msgs tf2_ros)
target_link_libraries(octomap_3d_astar_planner_node terrain_planning nlohmann_json::nlohmann_json)

add_executable(terrain_path_follower_node src/terrain_path_follower_node.cpp)
target_compile_features(terrain_path_follower_node PUBLIC cxx_std_17)
ament_target_dependencies(terrain_path_follower_node
  geometry_msgs nav_msgs rclcpp std_msgs tf2 tf2_geometry_msgs tf2_ros)

add_executable(terrain_plan_check src/terrain_plan_check.cpp)
target_compile_features(terrain_plan_check PUBLIC cxx_std_17)
target_include_directories(terrain_plan_check PRIVATE include)
target_link_libraries(terrain_plan_check terrain_planning octomap::octomap)

install(TARGETS
  octomap_3d_astar_planner_node
  terrain_path_follower_node
  terrain_plan_check
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_astar_3d test/test_astar_3d.cpp)
  target_link_libraries(test_astar_3d terrain_planning)
  target_include_directories(test_astar_3d PRIVATE include)
  ament_add_gtest(test_terrain_model test/test_terrain_model.cpp)
  target_link_libraries(test_terrain_model terrain_planning octomap::octomap)
  target_include_directories(test_terrain_model PRIVATE include)
endif()

ament_package()
