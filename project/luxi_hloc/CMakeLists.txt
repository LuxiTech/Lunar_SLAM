cmake_minimum_required(VERSION 3.8)
project(luxi_hloc)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(OpenCV REQUIRED)
find_package(RTABMap REQUIRED COMPONENTS core)

add_executable(rtab_hloc_exporter src/rtab_hloc_exporter.cpp)
target_link_libraries(rtab_hloc_exporter rtabmap::core ${OpenCV_LIBRARIES})
target_include_directories(rtab_hloc_exporter PRIVATE ${OpenCV_INCLUDE_DIRS})

ament_python_install_package(${PROJECT_NAME})

install(TARGETS rtab_hloc_exporter
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
  scripts/build_reference_model.py
  scripts/hloc_localizer_node
  scripts/offline_localization_test.py
  scripts/check_hloc_environment.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})
install(FILES README.md
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_geometry test/test_geometry.py)
  ament_add_pytest_test(test_map_io test/test_map_io.py)
  ament_add_pytest_test(test_pose_estimator test/test_pose_estimator.py)
endif()

ament_package()
