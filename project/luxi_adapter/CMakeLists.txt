cmake_minimum_required(VERSION 3.8)
project(luxi_adapter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS imgcodecs)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rtabmap_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME}_core SHARED src/sensor_adapter.cpp)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME}_core ${OpenCV_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}_core
  cv_bridge
  rclcpp
  rclcpp_components
  rtabmap_msgs
  sensor_msgs)
rclcpp_components_register_nodes(${PROJECT_NAME}_core "luxi_adapter::SensorAdapter")

add_executable(sensor_adapter_node src/sensor_adapter_main.cpp)
target_link_libraries(sensor_adapter_node ${PROJECT_NAME}_core)

add_library(imu_level_calibrator_core SHARED src/imu_level_calibrator.cpp)
target_compile_features(imu_level_calibrator_core PUBLIC cxx_std_17)
target_include_directories(imu_level_calibrator_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  imu_level_calibrator_core geometry_msgs rclcpp sensor_msgs std_msgs std_srvs
  tf2 tf2_ros)

add_executable(imu_level_calibrator_node src/imu_level_calibrator_main.cpp)
target_link_libraries(imu_level_calibrator_node imu_level_calibrator_core)

install(TARGETS ${PROJECT_NAME}_core sensor_adapter_node
  imu_level_calibrator_core imu_level_calibrator_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch docs DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_sensor_adapter test/test_sensor_adapter.cpp)
  if(TARGET test_sensor_adapter)
    target_link_libraries(test_sensor_adapter ${PROJECT_NAME}_core)
  endif()
  ament_add_gtest(test_imu_level_calibrator test/test_imu_level_calibrator.cpp)
  if(TARGET test_imu_level_calibrator)
    target_link_libraries(test_imu_level_calibrator imu_level_calibrator_core)
    ament_target_dependencies(
      test_imu_level_calibrator geometry_msgs rclcpp sensor_msgs std_msgs std_srvs
      tf2 tf2_ros)
  endif()
  ament_add_pytest_test(test_sensor_bringup_launch test/test_sensor_bringup_launch.py)
endif()

ament_package()
