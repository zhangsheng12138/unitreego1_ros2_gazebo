cmake_minimum_required(VERSION 3.8)
project(unitree_legged_control)

# C++17 (ROS2 必需)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ======================= ROS2 依赖包 =======================
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(unitree_legged_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(control_toolbox REQUIRED)

# ======================= 头文件目录 =======================
include_directories(
  include
)

# ======================= 编译库文件 =======================
add_library(${PROJECT_NAME}
  src/joint_controller.cpp
  src/unitree_joint_control_tool.cpp
)

# ======================= 设置依赖 =======================
ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  realtime_tools
  unitree_legged_msgs
  urdf
  control_toolbox
)

# 插件导出（防止符号冲突）
pluginlib_export_plugin_description_file(
  controller_interface
  plugins/unitree_legged_control_plugins.xml
)

# ======================= 安装 =======================
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY plugins/
  DESTINATION share/${PROJECT_NAME}/plugins
)

# ======================= 测试 =======================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
