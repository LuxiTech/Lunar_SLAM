#
# Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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


cmake_minimum_required(VERSION 3.16)
project(openeb_vendor CXX)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_vendor_package REQUIRED)

if (ament_cmake_vendor_package_VERSION VERSION_GREATER_EQUAL 2.0.0)
  ament_add_default_options()
endif()

option(
  BUILD_SHARED_LIBS
  "Global flag to cause add_library() to create shared libraries if on. \
  If set to true, this will cause all libraries to be built shared \
  unless the library was explicitly added as a static library."
  ON)

ament_vendor(openeb_vendor
  VCS_URL https://github.com/ros-event-camera/openeb.git
  VCS_VERSION 5.2.0-gcc-15.2-ros2
  GLOBAL_HOOK
#  PATCHES patches
  CMAKE_ARGS
  -DCOMPILE_PLAYER:BOOL=OFF
  -DCOMPILE_PYTHON3_BINDINGS:BOOL=OFF
  -DUDEV_RULES_SYSTEM_INSTALL:BOOL=OFF
  -DHDF5_DISABLED=ON
)
#  PATCHES patches

ament_package(
  CONFIG_EXTRAS "openeb_vendor-extras.cmake.in"
)
