cmake_minimum_required(VERSION 3.8)
project(unitree_gazebo)

# C++17 标准（ROS2 必需）
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ======================= ROS2 依赖包 =======================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(unitree_legged_msgs REQUIRED)

# ======================= Gazebo 依赖 =======================
find_package(gazebo REQUIRED)

# ======================= 头文件目录 =======================
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

# ======================= 编译 Gazebo 插件 =======================
# 足端接触检测插件
add_library(unitreeFootContactPlugin SHARED
  plugin/foot_contact_plugin.cc
)
ament_target_dependencies(unitreeFootContactPlugin
  rclcpp
  std_msgs
  geometry_msgs
  gazebo_ros
  unitree_legged_msgs
)
target_link_libraries(unitreeFootContactPlugin
  ${GAZEBO_LIBRARIES}
)

# 力可视化插件
add_library(unitreeDrawForcePlugin SHARED
  plugin/draw_force_plugin.cc
)
ament_target_dependencies(unitreeDrawForcePlugin
  rclcpp
  std_msgs
  geometry_msgs
  gazebo_ros
  unitree_legged_msgs
)
target_link_libraries(unitreeDrawForcePlugin
  ${GAZEBO_LIBRARIES}
)

# ======================= 安装 =======================
install(TARGETS
  unitreeFootContactPlugin
  unitreeDrawForcePlugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装 world、模型、launch 文件
install(DIRECTORY
  worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# 测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
