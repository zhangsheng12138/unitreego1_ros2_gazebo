cmake_minimum_required(VERSION 3.8)
project(go1_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

set(msg_files
  "msg/MotorCmd.msg"
  "msg/MotorState.msg"
  "msg/ImuData.msg"
  "msg/LowCmd.msg"
  "msg/LowState.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

include_directories(
  include
)

# 库目标使用不同名称以避免与rosidl_generate_interfaces冲突
add_library(${PROJECT_NAME}_lib SHARED
  src/joint_controller.cpp
  src/unitree_joint_control_tool.cpp
  src/body.cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  rclcpp
  controller_interface
  hardware_interface
  pluginlib
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  gazebo_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  urdf
  realtime_tools
  yaml_cpp_vendor
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_lib "${cpp_typesupport_target}" ${YAML_CPP_LIBRARIES})

# 可执行文件

add_executable(unitree_move_kinetic src/move_publisher.cpp)
ament_target_dependencies(unitree_move_kinetic rclcpp std_msgs geometry_msgs gazebo_msgs tf2 tf2_ros tf2_geometry_msgs)
target_link_libraries(unitree_move_kinetic ${PROJECT_NAME}_lib)

add_executable(unitree_servo src/servo.cpp)
ament_target_dependencies(unitree_servo rclcpp std_msgs geometry_msgs sensor_msgs nav_msgs)
target_link_libraries(unitree_servo ${PROJECT_NAME}_lib)

add_executable(unitree_external_force src/external_force.cpp)
ament_target_dependencies(unitree_external_force rclcpp geometry_msgs)
target_link_libraries(unitree_external_force ${PROJECT_NAME}_lib)

pluginlib_export_plugin_description_file(controller_interface unitree_controller_plugins.xml)

install(
  TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS unitree_move_kinetic unitree_servo unitree_external_force
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  FILES unitree_controller_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()