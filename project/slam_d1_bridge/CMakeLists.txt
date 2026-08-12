cmake_minimum_required(VERSION 3.8)
project(slam_d1_bridge)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ddt_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(slam_d1_bridge_node
    src/command_converter.cpp
    src/slam_d1_bridge_node.cpp
)
target_compile_features(slam_d1_bridge_node PUBLIC cxx_std_17)
target_include_directories(slam_d1_bridge_node PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(slam_d1_bridge_node ddt_msgs geometry_msgs rclcpp)

add_executable(slam_d1_bridge src/slam_d1_bridge_main.cpp)
target_compile_features(slam_d1_bridge PUBLIC cxx_std_17)
target_link_libraries(slam_d1_bridge slam_d1_bridge_node)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch docs DESTINATION share/${PROJECT_NAME})
install(
    PROGRAMS
        scripts/start_slam_d1_bridge.sh
        scripts/stop_slam_d1_bridge.sh
    DESTINATION lib/${PROJECT_NAME}
)
install(
    TARGETS slam_d1_bridge_node slam_d1_bridge
    EXPORT export_slam_d1_bridge_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
    INCLUDES DESTINATION include
)

ament_export_targets(export_slam_d1_bridge_node HAS_LIBRARY_TARGET)
ament_export_dependencies(ddt_msgs geometry_msgs rclcpp)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    find_package(ament_lint_auto REQUIRED)

    # The ROS default uncrustify profile enforces two-space indentation.
    # This package uses the repository's explicit four-space .clang-format policy.
    set(ament_cmake_uncrustify_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()

    ament_add_gtest(test_command_converter test/test_command_converter.cpp)
    target_link_libraries(test_command_converter slam_d1_bridge_node)
endif()

ament_package()
