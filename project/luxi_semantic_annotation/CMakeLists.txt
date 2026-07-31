cmake_minimum_required(VERSION 3.8)
project(luxi_semantic_annotation)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(OCTOMAP_ROOT "/home/lunar/project/lunar_slam/3parts/octomap/install" CACHE PATH
  "Prefix of the project-local OctoMap installation")
list(PREPEND CMAKE_PREFIX_PATH "${OCTOMAP_ROOT}")

find_package(ament_cmake REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(octomap CONFIG REQUIRED)

add_library(semantic_annotation_core src/semantic_annotation.cpp)
target_include_directories(semantic_annotation_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(semantic_annotation_core
  PUBLIC nlohmann_json::nlohmann_json octomap::octomap)

add_executable(semantic_annotation_tool src/semantic_annotation_tool.cpp)
target_link_libraries(semantic_annotation_tool semantic_annotation_core)

install(
  TARGETS semantic_annotation_core semantic_annotation_tool
  EXPORT luxi_semantic_annotation_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_semantic_annotation test/test_semantic_annotation.cpp)
  target_link_libraries(test_semantic_annotation semantic_annotation_core)
endif()

ament_export_targets(luxi_semantic_annotation_targets HAS_LIBRARY_TARGET)
ament_package()
