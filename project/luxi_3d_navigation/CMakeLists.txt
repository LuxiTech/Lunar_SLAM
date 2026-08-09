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
ament_target_dependencies(terrain_path_follower_node
  geometry_msgs nav_msgs rclcpp std_msgs tf2 tf2_geometry_msgs tf2_ros)

add_executable(twist_direction_adapter_node src/twist_direction_adapter_node.cpp)
target_compile_features(twist_direction_adapter_node PUBLIC cxx_std_17)
target_include_directories(twist_direction_adapter_node PRIVATE include)
ament_target_dependencies(twist_direction_adapter_node geometry_msgs rclcpp)

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
  twist_direction_adapter_node
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
  set_target_properties(
    test_terrain_cloud_classifier PROPERTIES BUILD_RPATH "/lib/aarch64-linux-gnu")
  target_link_options(test_terrain_cloud_classifier PRIVATE "-Wl,--disable-new-dtags")
endif()

ament_package()
