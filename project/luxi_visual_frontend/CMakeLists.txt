cmake_minimum_required(VERSION 3.8)
project(luxi_visual_frontend)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  scripts/visual_odometry_node
  scripts/check_visual_frontend_environment.py
  scripts/build_superpoint_tensorrt.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config docs launch
  DESTINATION share/${PROJECT_NAME})
install(FILES README.md
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_geometry test/test_geometry.py)
  ament_add_pytest_test(test_rtab_codec test/test_rtab_codec.py)
  ament_add_pytest_test(test_rate_limit test/test_rate_limit.py)
  ament_add_pytest_test(test_tensorrt_superpoint test/test_tensorrt_superpoint.py)
  ament_add_pytest_test(test_tracker test/test_tracker.py)
  ament_add_pytest_test(test_node_math test/test_node_math.py)
endif()

ament_package()
