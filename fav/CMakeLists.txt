# Copyright (C) 2023 Thies Lennart Alff
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
find_package(hippo_msgs REQUIRED)

install(PROGRAMS
  nodes/scenario_node.py
  nodes/robot_marker_publisher.py
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

