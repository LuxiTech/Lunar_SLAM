cmake_minimum_required(VERSION 3.8)
project(luxi_3d_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

get_filename_component(LUXI_WORKSPACE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(OCTOMAP_ROOT "${LUXI_WORKSPACE_ROOT}/3parts/octomap/install" CACHE PATH
  "Prefix of the project-local OctoMap installation")
if(EXISTS "${OCTOMAP_ROOT}")
  list(PREPEND CMAKE_PREFIX_PATH "${OCTOMAP_ROOT}")
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(octomap CONFIG REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common features filters io search)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

if(TARGET octomap::octomap)
  set(LUXI_OCTOMAP_TARGET octomap::octomap)
elseif(TARGET octomap)
  set(LUXI_OCTOMAP_TARGET octomap)
else()
  message(FATAL_ERROR "The discovered OctoMap package does not export a supported CMake target")
endif()

add_library(terrain_planning
  src/astar_3d.cpp
  src/terrain_model.cpp)
target_compile_features(terrain_planning PUBLIC cxx_std_17)
target_include_directories(terrain_planning PUBLIC include)
target_link_libraries(terrain_planning PUBLIC ${LUXI_OCTOMAP_TARGET})

add_library(terrain_cloud_classifier src/terrain_cloud_classifier.cpp)
target_compile_features(terrain_cloud_classifier PUBLIC cxx_std_17)
target_include_directories(terrain_cloud_classifier PUBLIC include ${PCL_INCLUDE_DIRS})
target_link_directories(terrain_cloud_classifier PUBLIC ${PCL_LIBRARY_DIRS})
target_link_libraries(terrain_cloud_classifier PUBLIC terrain_planning ${PCL_LIBRARIES})
target_compile_definitions(terrain_cloud_classifier PUBLIC ${PCL_DEFINITIONS})

add_library(rolling_voxel_grid src/rolling_voxel_grid.cpp)
target_compile_features(rolling_voxel_grid PUBLIC cxx_std_17)
target_include_directories(rolling_voxel_grid PUBLIC include)

add_executable(octomap_3d_astar_planner_node src/octomap_3d_astar_planner_node.cpp)
target_compile_features(octomap_3d_astar_planner_node PUBLIC cxx_std_17)
target_include_directories(octomap_3d_astar_planner_node PRIVATE include)
ament_target_dependencies(octomap_3d_astar_planner_node
  geometry_msgs nav_msgs octomap_msgs rclcpp std_msgs tf2 tf2_geometry_msgs tf2_ros
  visualization_msgs)
target_link_libraries(
  octomap_3d_astar_planner_node terrain_cloud_classifier terrain_planning
  nlohmann_json::nlohmann_json)

add_executable(terrain_map_to_points src/terrain_map_to_points.cpp)
target_compile_features(terrain_map_to_points PUBLIC cxx_std_17)
target_include_directories(terrain_map_to_points PRIVATE include)
target_link_libraries(
  terrain_map_to_points terrain_cloud_classifier terrain_planning ${LUXI_OCTOMAP_TARGET})

# Hik MVS exposes an older libusb through LD_LIBRARY_PATH. PCL requires the
# system libusb symbol set, so PCL-backed terrain binaries must prefer /lib.
foreach(pcl_terrain_target octomap_3d_astar_planner_node terrain_map_to_points)
  set_target_properties(
    ${pcl_terrain_target} PROPERTIES
    BUILD_RPATH "/lib/aarch64-linux-gnu;${OCTOMAP_ROOT}/lib"
    INSTALL_RPATH "/lib/aarch64-linux-gnu;${OCTOMAP_ROOT}/lib")
  target_link_options(${pcl_terrain_target} PRIVATE "-Wl,--disable-new-dtags")
endforeach()

add_executable(terrain_path_follower_node src/terrain_path_follower_node.cpp)
target_compile_features(terrain_path_follower_node PUBLIC cxx_std_17)
target_include_directories(terrain_path_follower_node PRIVATE include)
ament_target_dependencies(terrain_path_follower_node
  geometry_msgs nav_msgs rclcpp std_msgs tf2 tf2_geometry_msgs tf2_ros)

add_executable(local_obstacle_map_node src/local_obstacle_map_node.cpp)
target_compile_features(local_obstacle_map_node PUBLIC cxx_std_17)
target_include_directories(local_obstacle_map_node PRIVATE include)
ament_target_dependencies(local_obstacle_map_node
  geometry_msgs rclcpp sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros)
target_link_libraries(local_obstacle_map_node rolling_voxel_grid)

add_executable(local_path_replanner_node src/local_path_replanner_node.cpp)
target_compile_features(local_path_replanner_node PUBLIC cxx_std_17)
target_include_directories(local_path_replanner_node PRIVATE include ${PCL_INCLUDE_DIRS})
ament_target_dependencies(local_path_replanner_node
  geometry_msgs nav_msgs octomap_msgs rclcpp sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros)
target_link_libraries(
  local_path_replanner_node terrain_cloud_classifier terrain_planning
  nlohmann_json::nlohmann_json ${PCL_LIBRARIES})
set_target_properties(
  local_path_replanner_node PROPERTIES
  BUILD_RPATH "/lib/aarch64-linux-gnu;${OCTOMAP_ROOT}/lib"
  INSTALL_RPATH "/lib/aarch64-linux-gnu;${OCTOMAP_ROOT}/lib")
target_link_options(local_path_replanner_node PRIVATE "-Wl,--disable-new-dtags")

add_executable(navigation_safety_gate_node src/navigation_safety_gate_node.cpp)
target_compile_features(navigation_safety_gate_node PUBLIC cxx_std_17)
target_include_directories(navigation_safety_gate_node PRIVATE include)
ament_target_dependencies(navigation_safety_gate_node geometry_msgs rclcpp std_msgs)

add_executable(twist_direction_adapter_node src/twist_direction_adapter_node.cpp)
target_compile_features(twist_direction_adapter_node PUBLIC cxx_std_17)
target_include_directories(twist_direction_adapter_node PRIVATE include)
ament_target_dependencies(twist_direction_adapter_node geometry_msgs rclcpp)

add_executable(velocity_command_mux_node src/velocity_command_mux_node.cpp)
target_compile_features(velocity_command_mux_node PUBLIC cxx_std_17)
target_include_directories(velocity_command_mux_node PRIVATE include)
ament_target_dependencies(velocity_command_mux_node geometry_msgs rclcpp std_msgs)

add_executable(terrain_plan_check src/terrain_plan_check.cpp)
target_compile_features(terrain_plan_check PUBLIC cxx_std_17)
target_include_directories(terrain_plan_check PRIVATE include)
target_link_libraries(
  terrain_plan_check terrain_cloud_classifier terrain_planning ${LUXI_OCTOMAP_TARGET})
set_target_properties(
  terrain_plan_check PROPERTIES
  BUILD_RPATH "/lib/aarch64-linux-gnu;${OCTOMAP_ROOT}/lib"
  INSTALL_RPATH "/lib/aarch64-linux-gnu;${OCTOMAP_ROOT}/lib")
target_link_options(terrain_plan_check PRIVATE "-Wl,--disable-new-dtags")

install(TARGETS
  octomap_3d_astar_planner_node
  terrain_path_follower_node
  local_obstacle_map_node
  local_path_replanner_node
  navigation_safety_gate_node
  twist_direction_adapter_node
  velocity_command_mux_node
  terrain_plan_check
  terrain_map_to_points
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_astar_3d test/test_astar_3d.cpp)
  target_link_libraries(test_astar_3d terrain_planning)
  target_include_directories(test_astar_3d PRIVATE include)
  ament_add_gtest(test_terrain_model test/test_terrain_model.cpp)
  target_link_libraries(test_terrain_model terrain_planning ${LUXI_OCTOMAP_TARGET})
  target_include_directories(test_terrain_model PRIVATE include)
  ament_add_gtest(test_terrain_cloud_classifier test/test_terrain_cloud_classifier.cpp)
  target_link_libraries(test_terrain_cloud_classifier terrain_cloud_classifier)
  target_include_directories(test_terrain_cloud_classifier PRIVATE include)
  ament_add_gtest(test_twist_direction_adapter test/test_twist_direction_adapter.cpp)
  target_include_directories(test_twist_direction_adapter PRIVATE include)
  ament_target_dependencies(test_twist_direction_adapter geometry_msgs)
  ament_add_gtest(test_velocity_command_mux test/test_velocity_command_mux.cpp)
  target_include_directories(test_velocity_command_mux PRIVATE include)
  ament_target_dependencies(test_velocity_command_mux geometry_msgs)
  ament_add_gtest(
    test_localization_health_monitor test/test_localization_health_monitor.cpp)
  target_include_directories(test_localization_health_monitor PRIVATE include)
  ament_add_gtest(test_path_follower_control test/test_path_follower_control.cpp)
  target_include_directories(test_path_follower_control PRIVATE include)
  ament_add_gtest(test_rolling_voxel_grid test/test_rolling_voxel_grid.cpp)
  target_link_libraries(test_rolling_voxel_grid rolling_voxel_grid)
  target_include_directories(test_rolling_voxel_grid PRIVATE include)
  ament_add_gtest(test_navigation_safety_gate test/test_navigation_safety_gate.cpp)
  target_include_directories(test_navigation_safety_gate PRIVATE include)
  ament_target_dependencies(test_navigation_safety_gate geometry_msgs)
  set_target_properties(
    test_terrain_cloud_classifier PROPERTIES BUILD_RPATH "/lib/aarch64-linux-gnu")
  target_link_options(test_terrain_cloud_classifier PRIVATE "-Wl,--disable-new-dtags")
endif()

ament_package()
