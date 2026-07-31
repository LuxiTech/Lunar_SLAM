cmake_minimum_required(VERSION 3.8)
project(luxi_location)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)

set(
  OPEN3D_INSTALL
  "${CMAKE_CURRENT_SOURCE_DIR}/../../3parts/open3d/install"
  CACHE PATH "Open3D installation prefix")
find_package(Open3D REQUIRED PATHS "${OPEN3D_INSTALL}/lib/cmake/Open3D" NO_DEFAULT_PATH)

add_library(luxi_location_core
  src/depth_projection.cpp
  src/icp_localizer.cpp
  src/localization_supervisor.cpp)
target_include_directories(luxi_location_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(luxi_location_core PUBLIC Open3D::Open3D)

add_executable(icp_localization_node src/localization_node.cpp)
target_link_libraries(icp_localization_node luxi_location_core)
ament_target_dependencies(
  icp_localization_node
  geometry_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros)

add_executable(offline_registration_test src/offline_registration_test.cpp)
target_link_libraries(offline_registration_test luxi_location_core)

set_target_properties(
  luxi_location_core icp_localization_node offline_registration_test
  PROPERTIES INSTALL_RPATH "${OPEN3D_INSTALL}/lib")

install(
  TARGETS luxi_location_core icp_localization_node offline_registration_test
  EXPORT luxi_location_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_icp_localizer test/test_icp_localizer.cpp)
  target_link_libraries(test_icp_localizer luxi_location_core)
endif()

ament_export_targets(luxi_location_targets HAS_LIBRARY_TARGET)
ament_package()
