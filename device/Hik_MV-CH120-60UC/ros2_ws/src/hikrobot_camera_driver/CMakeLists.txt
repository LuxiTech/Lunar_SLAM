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

find_package(ament_index_cpp REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(TINYXML2 REQUIRED IMPORTED_TARGET tinyxml2)



# ==========================
# OpenCV
# ==========================

find_package(OpenCV REQUIRED)



# ==========================
# Hikrobot MVS SDK
# ==========================

set(MVS_ROOT "/opt/MVS" CACHE PATH "Root directory of the Hikrobot MVS SDK")

# MVS installs its libraries below a directory named after the CPU
# architecture.  The previous project only searched lib/64, which is an x86
# layout and cannot work on this Jetson/aarch64 machine.
find_path(MVS_INCLUDE_DIR
    NAMES MvCameraControl.h
    HINTS "${MVS_ROOT}/include")

find_library(MVS_CAMERA_CONTROL_LIBRARY
    NAMES MvCameraControl
    HINTS
        "${MVS_ROOT}/lib/${CMAKE_SYSTEM_PROCESSOR}"
        "${MVS_ROOT}/lib/aarch64"
        "${MVS_ROOT}/lib/64")

if(NOT MVS_INCLUDE_DIR OR NOT MVS_CAMERA_CONTROL_LIBRARY)
    message(FATAL_ERROR
        "Hikrobot MVS development SDK was not found below ${MVS_ROOT}. "
        "Install the MVS SDK matching this architecture (including "
        "MvCameraControl.h), or configure -DMVS_ROOT=/path/to/MVS.")
endif()

get_filename_component(MVS_LIBRARY_DIR "${MVS_CAMERA_CONTROL_LIBRARY}" DIRECTORY)



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
    ${MVS_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
)



# ==========================
# Link
# ==========================

target_link_libraries(
    stereo_node

    # Hikrobot SDK
    ${MVS_CAMERA_CONTROL_LIBRARY}


    # OpenCV
    ${OpenCV_LIBRARIES}


    # ROS2
    rclcpp::rclcpp

    ament_index_cpp::ament_index_cpp
    PkgConfig::TINYXML2
    camera_info_manager::camera_info_manager


    pthread
)

ament_target_dependencies(stereo_node
    rclcpp
    sensor_msgs
    std_msgs
    ament_index_cpp
    camera_info_manager)

# The MVS control library dlopens companion libraries from the same SDK
# directory.  Keep this path on the executable so a normal ROS launch does
# not depend on a globally exported LD_LIBRARY_PATH.
set_target_properties(stereo_node PROPERTIES
    BUILD_RPATH "${MVS_LIBRARY_DIR}"
    INSTALL_RPATH "${MVS_LIBRARY_DIR}")


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
