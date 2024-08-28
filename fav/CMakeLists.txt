# Copyright (C) 2023-2024 Thies Lennart Alff
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA

cmake_minimum_required(VERSION 3.5)
project(fav)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CXX_FLAGS -Wall -Wextra -Wpedantic -Wshadow -std=c++17 -O2 -fPIC)
add_compile_options(${CXX_FLAGS})

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hippo_common REQUIRED)
find_package(hippo_control_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(scenario_msgs REQUIRED)
find_package(rviz_2d_overlay_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

################################################################################
# scenario node
################################################################################
add_executable(scenario_node src/scenario/scenario_node.cpp)
ament_target_dependencies(scenario_node
  rclcpp
  hippo_common
  scenario_msgs
  std_msgs
  std_srvs
  geometry_msgs
  rviz_2d_overlay_msgs
  visualization_msgs
  yaml-cpp
  ament_index_cpp
)
install(
  TARGETS scenario_node
  DESTINATION lib/${PROJECT_NAME}
)
################################################################################
# robot_marker node
################################################################################
add_executable(robot_marker_publisher src/robot_marker_publisher.cpp)
ament_target_dependencies(robot_marker_publisher
  rclcpp
  geometry_msgs
  visualization_msgs
)
install(
  TARGETS robot_marker_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
# ament_python_install_package(${PROJECT_NAME})

################################################################################
# install shared resources
################################################################################
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

