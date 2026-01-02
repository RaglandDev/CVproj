# =====================================================================
 # MIT License
 # 
 # Copyright (c) 2025 Omni Instrument Inc.
 # 
 # Permission is hereby granted, free of charge, to any person obtaining a copy
 # of this software and associated documentation files (the "Software"), to deal
 # in the Software without restriction, including without limitation the rights
 # to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 # copies of the Software, and to permit persons to whom the Software is
 # furnished to do so, subject to the following conditions:
 # 
 # The above copyright notice and this permission notice shall be included in
 # all copies or substantial portions of the Software.
 # 
 # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 # IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 # FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 # AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 # LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 # OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 # THE SOFTWARE.
# ===================================================================== 

cmake_minimum_required(VERSION 3.8)
project(tsdf_saver)

# ----------------------------------------------------------
# Compiler Flags
# ----------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall -Wextra -Wpedantic
    -O3
    -march=native
    -mtune=native
    -ffast-math
  )
endif()

# ----------------------------------------------------------
# Dependencies
# ----------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)

# ----------------------------------------------------------
# Build Component Library
# ----------------------------------------------------------
add_library(exact_time_saver SHARED
  src/exact_time_saver.cpp
)

target_include_directories(exact_time_saver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(exact_time_saver
  rclcpp
  rclcpp_components
  std_srvs
  sensor_msgs
)

# ----------------------------------------------------------
# Register composable node
# ----------------------------------------------------------
rclcpp_components_register_nodes(exact_time_saver "tsdf_saver::ExactTimeSaver")

# ----------------------------------------------------------
# Install Targets
# ----------------------------------------------------------
install(
  TARGETS exact_time_saver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

# ----------------------------------------------------------
# Install Launch Files
# ----------------------------------------------------------
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
