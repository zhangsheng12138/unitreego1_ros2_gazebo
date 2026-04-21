cmake_minimum_required(VERSION 3.8)
project(go1_description)

# C++ 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 核心构建依赖
find_package(ament_cmake REQUIRED)

# 安装所有资源文件（保留你的 meshes/xacro/urdf/launch/config）
install(
  DIRECTORY meshes urdf xacro launch config
  DESTINATION share/${PROJECT_NAME}
)

# 测试配置（可选）
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# ROS2 包导出
ament_package()
