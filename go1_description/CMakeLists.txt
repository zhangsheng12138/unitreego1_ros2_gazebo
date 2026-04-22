cmake_minimum_required(VERSION 3.8)
project(go1_description)

# 设置C++标准（ROS2推荐C++17）
# 【可变参数】C++标准版本
# 引用/被引用注意：1. ROS2 Humble+推荐C++17，Noetic为C++14；2. 修改后需确保所有依赖包兼容对应标准；3. 被ament_cmake编译系统引用，需与package.xml中构建依赖对齐
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# C++ 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # 【可变参数】编译警告选项
  # 引用/被引用注意：1. -Wall/-Wextra开启基础警告，-Wpedantic开启严格语法检查；2. 若第三方依赖有非标准语法，需临时注释避免编译失败；3. 被编译器（GCC/Clang）引用，需匹配编译器版本支持的选项
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS2 核心构建依赖（显式声明避免警告）
find_package(ament_cmake REQUIRED)
find_package(ros2_control REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rviz2 REQUIRED)
find_package(xacro REQUIRED)

# 安装所有资源文件（meshes/xacro/urdf/launch/config）
# 【可变参数】安装目录与文件列表
# 引用/被引用注意：1. DIRECTORY后为待安装的文件夹，需确保文件夹名称与实际路径一致；2. DESTINATION为ROS2包共享目录，必须是share/${PROJECT_NAME}（ROS2规范）；3. 被ament_cmake安装系统引用，缺失文件夹会导致运行时找不到配置/模型文件
install(
  DIRECTORY meshes urdf xacro launch config
  DESTINATION share/${PROJECT_NAME}
)

# 测试配置（可选）
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # 【可变参数】测试依赖自动查找
  # 引用/被引用注意：1. ament_lint_auto_find_test_dependencies()会自动查找test_depend依赖；2. 若需自定义测试规则，需注释此行并手动声明；3. 被CI/测试框架引用，关闭BUILD_TESTING可跳过测试编译
  ament_lint_auto_find_test_dependencies()
endif()

# 补充优化：显式导出依赖（提升鲁棒性）
ament_export_dependencies(
  ros2_control
  gazebo_ros2_control
  robot_state_publisher
  rviz2
  xacro
)

# ROS2 包导出
ament_package()

