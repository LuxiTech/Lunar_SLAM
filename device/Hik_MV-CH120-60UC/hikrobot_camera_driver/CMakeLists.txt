cmake_minimum_required(VERSION 3.16)

project(hikrobot_camera_driver)


# ==========================
# C++17
# ==========================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)



# ==========================
# ROS2
# ==========================

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tinyxml2 REQUIRED)
find_package(camera_info_manager REQUIRED)

find_package(rosidl_typesupport_cpp REQUIRED)



# ==========================
# OpenCV
# ==========================

find_package(OpenCV REQUIRED)



# ==========================
# Hikrobot MVS SDK
# ==========================

set(MVS_ROOT "/opt/MVS")


include_directories(
    ${MVS_ROOT}/include
)


link_directories(
    ${MVS_ROOT}/lib/64
)



# ==========================
# Stereo Camera Node
# ==========================

add_executable(
    stereo_node

    src/stereo_node.cpp
    src/HikCamera.cpp
    src/StereoCamera.cpp
    src/CameraConfig.cpp
)

# ==========================
# Include
# ==========================

target_include_directories(
    stereo_node
    PRIVATE

    ${CMAKE_CURRENT_SOURCE_DIR}/include

    ${OpenCV_INCLUDE_DIRS}


    # ROS2 Lyrical headers

    /opt/ros/lyrical/include

    /opt/ros/lyrical/include/rclcpp

    /opt/ros/lyrical/include/sensor_msgs

    /opt/ros/lyrical/include/std_msgs

    /opt/ros/lyrical/include/cv_bridge
)



# ==========================
# Link
# ==========================

target_link_libraries(
    stereo_node

    # Hikrobot SDK
    MvCameraControl


    # OpenCV
    ${OpenCV_LIBRARIES}


    # ROS2
    rclcpp::rclcpp

    cv_bridge::cv_bridge
    ament_index_cpp::ament_index_cpp
    tinyxml2::tinyxml2
    camera_info_manager::camera_info_manager


    pthread
)


# ==========================
# Install
# ==========================

install(
    TARGETS

    stereo_node

    DESTINATION lib/${PROJECT_NAME}
)


install(
    DIRECTORY include/
    DESTINATION include
)

install(
    DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
