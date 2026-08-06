cmake_minimum_required(VERSION 3.8)
project(luxi_semantic_annotation)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
get_filename_component(LUXI_WORKSPACE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(OCTOMAP_ROOT "${LUXI_WORKSPACE_ROOT}/3parts/octomap/install" CACHE PATH
  "Prefix of the project-local OctoMap installation")
if(EXISTS "${OCTOMAP_ROOT}")
  list(PREPEND CMAKE_PREFIX_PATH "${OCTOMAP_ROOT}")
endif()

find_package(ament_cmake REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(octomap CONFIG REQUIRED)

if(TARGET octomap::octomap)
  set(LUXI_OCTOMAP_TARGET octomap::octomap)
elseif(TARGET octomap)
  set(LUXI_OCTOMAP_TARGET octomap)
else()
  message(FATAL_ERROR "The discovered OctoMap package does not export a supported CMake target")
endif()

add_library(semantic_annotation_core src/semantic_annotation.cpp)
target_include_directories(semantic_annotation_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_link_libraries(semantic_annotation_core
  PUBLIC nlohmann_json::nlohmann_json ${LUXI_OCTOMAP_TARGET})

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
