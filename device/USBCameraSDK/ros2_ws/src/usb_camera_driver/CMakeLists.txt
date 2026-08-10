cmake_minimum_required(VERSION 3.16)
project(usb_camera_driver)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rtabmap_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(vpi CONFIG REQUIRED
  PATHS /opt/nvidia/vpi3/lib/aarch64-linux-gnu/cmake/vpi
  NO_DEFAULT_PATH)

add_executable(stereo_node src/stereo_node.cpp)
target_compile_features(stereo_node PRIVATE cxx_std_17)
target_compile_options(stereo_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(stereo_node rclcpp sensor_msgs)
target_link_libraries(stereo_node ${OpenCV_LIBS})

add_executable(opencv_stereo_calibrator src/opencv_stereo_calibrator.cpp)
target_compile_features(opencv_stereo_calibrator PRIVATE cxx_std_17)
target_compile_options(opencv_stereo_calibrator PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(opencv_stereo_calibrator ament_index_cpp rclcpp sensor_msgs)
target_link_libraries(opencv_stereo_calibrator ${OpenCV_LIBS})

add_executable(opencv_focus_viewer src/opencv_focus_viewer.cpp)
target_compile_features(opencv_focus_viewer PRIVATE cxx_std_17)
target_compile_options(opencv_focus_viewer PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(opencv_focus_viewer rclcpp sensor_msgs)
target_link_libraries(opencv_focus_viewer ${OpenCV_LIBS})

add_executable(stereo_depth_node src/stereo_depth_node.cpp)
target_compile_features(stereo_depth_node PRIVATE cxx_std_17)
target_compile_options(stereo_depth_node PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(stereo_depth_node
  cv_bridge
  message_filters
  rclcpp
  rtabmap_msgs
  sensor_msgs
)
target_link_libraries(stereo_depth_node ${OpenCV_LIBS} vpi)

install(TARGETS
  stereo_node
  opencv_stereo_calibrator
  opencv_focus_viewer
  stereo_depth_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES ../../calibration/stereo_opencv.yaml
  DESTINATION share/${PROJECT_NAME}/config)

ament_package()
