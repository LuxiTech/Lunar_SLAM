cmake_minimum_required(VERSION 3.8)
project(luxi_adapter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(${PROJECT_NAME}_core src/sensor_adapter.cpp)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}_core rclcpp sensor_msgs)

add_executable(sensor_adapter_node src/sensor_adapter_main.cpp)
target_link_libraries(sensor_adapter_node ${PROJECT_NAME}_core)

install(TARGETS ${PROJECT_NAME}_core sensor_adapter_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch docs DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_sensor_adapter test/test_sensor_adapter.cpp)
  if(TARGET test_sensor_adapter)
    target_link_libraries(test_sensor_adapter ${PROJECT_NAME}_core)
    ament_target_dependencies(test_sensor_adapter rclcpp sensor_msgs)
  endif()
endif()

ament_package()
