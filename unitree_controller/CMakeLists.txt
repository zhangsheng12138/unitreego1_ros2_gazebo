cmake_minimum_required(VERSION 3.8)
project(unitree_controller)

# 兼容 C++17
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 找到 ROS2 依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(controller_manager REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(unitree_legged_msgs REQUIRED)

# 包含目录
include_directories(
  include
)

# 库文件：unitree_controller
add_library(${PROJECT_NAME}
  src/body.cpp
)

# 依赖
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
  tf2
  tf2_ros
  controller_manager
  gazebo_ros
  unitree_legged_msgs
)

# 可执行文件：unitree_servo
add_executable(unitree_servo
  src/servo.cpp
)
target_link_libraries(unitree_servo ${PROJECT_NAME})
ament_target_dependencies(unitree_servo
  rclcpp
  unitree_legged_msgs
)

# 可执行文件：unitree_external_force
add_executable(unitree_external_force
  src/external_force.cpp
)
ament_target_dependencies(unitree_external_force
  rclcpp
  geometry_msgs
  gazebo_ros
)

# 可执行文件：unitree_move_kinetic
add_executable(unitree_move_kinetic
  src/move_publisher.cpp
)
ament_target_dependencies(unitree_move_kinetic
  rclcpp
  geometry_msgs
)

# 安装
install(TARGETS
  ${PROJECT_NAME}
  unitree_servo
  unitree_external_force
  unitree_move_kinetic
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# 测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
