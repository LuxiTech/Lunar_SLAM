cmake_minimum_required(VERSION 3.16)
project(luxi_map_tools LANGUAGES CXX)

find_package(PCL REQUIRED COMPONENTS common io filters search segmentation)
find_package(yaml-cpp REQUIRED)

add_library(map_cloud_filter_lib map_cloud_filter.cpp)
target_compile_features(map_cloud_filter_lib PUBLIC cxx_std_17)
target_include_directories(
  map_cloud_filter_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${PCL_INCLUDE_DIRS})
target_link_directories(map_cloud_filter_lib PUBLIC ${PCL_LIBRARY_DIRS})
target_link_libraries(map_cloud_filter_lib PUBLIC ${PCL_LIBRARIES} yaml-cpp)
target_compile_definitions(map_cloud_filter_lib PUBLIC ${PCL_DEFINITIONS})
target_compile_options(map_cloud_filter_lib PRIVATE -Wall -Wextra -Wpedantic)

add_executable(map_cloud_filter map_cloud_filter_main.cpp)
target_link_libraries(map_cloud_filter PRIVATE map_cloud_filter_lib)
target_compile_options(map_cloud_filter PRIVATE -Wall -Wextra -Wpedantic)
# Hik MVS installs an older libusb in the global loader cache. PCL requires the
# system libusb symbol set, so keep the standalone map tool on the system path.
set_target_properties(
  map_cloud_filter PROPERTIES BUILD_RPATH "/lib/aarch64-linux-gnu")
target_link_options(map_cloud_filter PRIVATE "-Wl,--disable-new-dtags")

include(CTest)
set(map_cloud_filter_test_source
  ${CMAKE_CURRENT_SOURCE_DIR}/../test/map_cloud_filter_test.cpp)
if(BUILD_TESTING AND EXISTS ${map_cloud_filter_test_source})
  add_executable(map_cloud_filter_test ${map_cloud_filter_test_source})
  target_link_libraries(map_cloud_filter_test PRIVATE map_cloud_filter_lib)
  target_compile_options(map_cloud_filter_test PRIVATE -Wall -Wextra -Wpedantic)
  add_test(
    NAME map_cloud_filter_test
    COMMAND map_cloud_filter_test
      ${CMAKE_CURRENT_SOURCE_DIR}/../test/filter_override.yaml)
endif()
