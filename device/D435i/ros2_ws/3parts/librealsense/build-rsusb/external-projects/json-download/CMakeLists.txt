cmake_minimum_required(VERSION 3.6)
project(nlohmann-json-download NONE)

include(ExternalProject)
ExternalProject_Add(
    nlohmann_json
    PREFIX .
    
    # We do not use 'GIT_REPOSITORY' as it doesn't support a shallow clone of a specific commit,
    # this make the clone step very long, so we clone by ourselves (See https://gitlab.kitware.com/cmake/cmake/-/issues/17770).
    # Adding detachedHead=false to avoid warnings like: "You are in 'detached HEAD' state..."
    # 'remove_directory' is working but cound as deprecated from cmake version 3.17, 
    # Once we have a minimal version support of cmake 3.17, we can switch the command to rm -rF
    DOWNLOAD_COMMAND   "/usr/bin/cmake" -E  remove_directory  "/home/lunar/project/lunar_slam/device/D435i/ros2_ws/3parts/librealsense/build-rsusb/third-party/json"
             COMMAND   git clone -c advice.detachedHead=false --branch v3.12.0 https://github.com/nlohmann/json.git --depth 1 json
    DOWNLOAD_DIR       "/home/lunar/project/lunar_slam/device/D435i/ros2_ws/3parts/librealsense/build-rsusb/third-party/"
    
    # Override default steps with no action, we just want the clone step.
    UPDATE_COMMAND ""
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
    )
    
   
