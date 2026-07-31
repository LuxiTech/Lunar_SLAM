# Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Copy configure.ac
file(INSTALL /home/lunar/project/lunar_slam/device/D435i/ros2_ws/3parts/librealsense/build-rsusb/third-party/fastcdr/configure.ac
    DESTINATION /home/lunar/project/lunar_slam/device/D435i/ros2_ws/3parts/librealsense/build-rsusb/third-party/fastcdr-build/autotools
    )

# Copy m4 diretory
file(INSTALL /home/lunar/project/lunar_slam/device/D435i/ros2_ws/3parts/librealsense/build-rsusb/third-party/fastcdr/m4
    DESTINATION /home/lunar/project/lunar_slam/device/D435i/ros2_ws/3parts/librealsense/build-rsusb/third-party/fastcdr-build/autotools
    )

# Create include/fastcdr
file(MAKE_DIRECTORY /home/lunar/project/lunar_slam/device/D435i/ros2_ws/3parts/librealsense/build-rsusb/third-party/fastcdr-build/autotools/include/fastcdr)

# Run autoreconf
execute_process(COMMAND autoreconf -fi /home/lunar/project/lunar_slam/device/D435i/ros2_ws/3parts/librealsense/build-rsusb/third-party/fastcdr-build/autotools
    RESULT_VARIABLE EXECUTE_RESULT)

if(NOT EXECUTE_RESULT EQUAL 0)
    message(FATAL_ERROR "Failed the execution of autoreconf")
endif()

# Copy include/fastcdr/config.h.in
file(INSTALL /home/lunar/project/lunar_slam/device/D435i/ros2_ws/3parts/librealsense/build-rsusb/third-party/fastcdr/include/fastcdr/config.h.in
    DESTINATION /home/lunar/project/lunar_slam/device/D435i/ros2_ws/3parts/librealsense/build-rsusb/third-party/fastcdr-build/autotools/include/fastcdr
    )

# Copy licenses
file(INSTALL /home/lunar/project/lunar_slam/device/D435i/ros2_ws/3parts/librealsense/build-rsusb/third-party/fastcdr/LICENSE
    DESTINATION /home/lunar/project/lunar_slam/device/D435i/ros2_ws/3parts/librealsense/build-rsusb/third-party/fastcdr-build/autotools
    )
