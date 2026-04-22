cmake_minimum_required(VERSION 3.16)
project(unitree_gazebo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(go1_controller REQUIRED)

find_package(gazebo REQUIRED)
find_package(ignition-math6 REQUIRED)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${ignition-math6_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${ignition-math6_LIBRARY_DIRS}
)

add_library(unitreeFootContactPlugin SHARED
  plugin/foot_contact_plugin.cc
)
ament_target_dependencies(unitreeFootContactPlugin
  rclcpp
  std_msgs
  geometry_msgs
  gazebo_ros
  go1_controller
)
target_link_libraries(unitreeFootContactPlugin
  ${GAZEBO_LIBRARIES}
  ${ignition-math6_LIBRARIES}
  gazebo_sensors
  gazebo_common
)

add_library(unitreeDrawForcePlugin SHARED
  plugin/draw_force_plugin.cc
)
ament_target_dependencies(unitreeDrawForcePlugin
  rclcpp
  std_msgs
  geometry_msgs
  gazebo_ros
  go1_controller
)
target_link_libraries(unitreeDrawForcePlugin
  ${GAZEBO_LIBRARIES}
  ${ignition-math6_LIBRARIES}
  gazebo_rendering
  gazebo_common
)

install(TARGETS
  unitreeFootContactPlugin
  unitreeDrawForcePlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  config
  worlds
  launch
  src
  DESTINATION share/${PROJECT_NAME}
)



# 安装Python脚本
install(PROGRAMS
  src/env.py
  src/check_gazebo_ready
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
