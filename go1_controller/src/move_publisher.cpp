// Copyright 2026 Unitree Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/msg/model_state.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <cmath>
#include <string>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace {

/**
 * @brief 坐标系枚举
 */
enum class Coord { kWorld, kRobot };

/**
 * @brief 加载YAML参数
 * 
 * @param node ROS2节点
 * @return 包含参数的YAML节点
 */
YAML::Node LoadParams(rclcpp::Node* node) {
  try {
    std::string pkg_path = ament_index_cpp::get_package_share_directory("unitree_gazebo");
    std::string yaml_path = pkg_path + "/config/go1_env_parameter.yaml";
    YAML::Node config = YAML::LoadFile(yaml_path);
    RCLCPP_INFO(node->get_logger(), "参数加载成功: %s", yaml_path.c_str());
    return config;
  } catch (const std::exception& e) {
    RCLCPP_WARN(node->get_logger(), "参数加载失败: %s，使用默认值", e.what());
    return YAML::Node();
  }
}

}  // namespace

/**
 * @brief 主函数
 * 
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return 退出码
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("move_publisher");

  // 声明并获取机器人名称参数
  node->declare_parameter<std::string>("robot_name", "go1");
  std::string robot_name;
  if (!node->get_parameter("robot_name", robot_name)) {
    RCLCPP_WARN(node->get_logger(),
                "获取robot_name失败，使用默认值: go1");
    robot_name = "go1";
  }
  RCLCPP_INFO(node->get_logger(), "robot_name: %s", robot_name.c_str());

  // 加载参数
  YAML::Node config = LoadParams(node.get());
  
  // 获取坐标系设置
  std::string frame = config["move"]["frame"].as<std::string>("world");
  Coord def_frame = (frame == "robot") ? Coord::kRobot : Coord::kWorld;

  // 创建模型状态发布器
  auto move_publisher = node->create_publisher<gazebo_msgs::msg::ModelState>(
      "/gazebo/set_model_state", 10);

  // 初始化模型状态消息
  gazebo_msgs::msg::ModelState model_state_pub;
  model_state_pub.model_name = robot_name + "_gazebo";

  if (def_frame == Coord::kWorld) {
    // 设置初始位置
    model_state_pub.pose.position.x = config["move"]["initial_position"]["x"].as<double>(0.0);
    model_state_pub.pose.position.y = config["move"]["initial_position"]["y"].as<double>(0.0);
    model_state_pub.pose.position.z = config["move"]["initial_position"]["z"].as<double>(0.5);

    // 设置初始姿态
    model_state_pub.pose.orientation.x = 0.0;
    model_state_pub.pose.orientation.y = 0.0;
    model_state_pub.pose.orientation.z = 0.0;
    model_state_pub.pose.orientation.w = 1.0;

    // 设置参考坐标系
    model_state_pub.reference_frame = "world";

    // 圆形运动参数
    double period = config["move"]["circle"]["period"].as<double>(5000.0);  // 周期（毫秒）
    double radius = config["move"]["circle"]["radius"].as<double>(1.5);    // 半径（米）
    long long time_ms = 0;
    tf2::Quaternion q;

    RCLCPP_INFO(node->get_logger(),
                "开始在世界坐标系中移动机器人: 圆形运动 (半径=%.2fm, "
                "周期=%.2fs)",
                radius, period / 1000.0);
    
    // 发布频率
    int publish_rate = config["move"]["publish_rate"].as<int>(1000);
    
    while (rclcpp::ok()) {
      // 计算圆形运动的位置
      model_state_pub.pose.position.x =
          radius * sin(2 * M_PI * static_cast<double>(time_ms) / period);
      model_state_pub.pose.position.y =
          radius * cos(2 * M_PI * static_cast<double>(time_ms) / period);

      // 计算朝向（与运动方向一致）
      q.setRPY(0, 0,
               -2 * M_PI * static_cast<double>(time_ms) / period);
      q.normalize();
      model_state_pub.pose.orientation = tf2::toMsg(q);

      // 发布模型状态
      move_publisher->publish(model_state_pub);

      // 睡眠以保持指定的发布频率
      rclcpp::Rate loop_rate(publish_rate);
      loop_rate.sleep();
      ++time_ms;

      // 每1000毫秒打印一次状态
      if (time_ms % 1000 == 0) {
        RCLCPP_DEBUG(
            node->get_logger(),
            "时间: %lldms, 位置(x=%.2f, y=%.2f), 偏航角: %.2f 弧度", time_ms,
            model_state_pub.pose.position.x,
            model_state_pub.pose.position.y,
            -2 * M_PI * static_cast<double>(time_ms) / period);
      }
    }
  } else if (def_frame == Coord::kRobot) {
    // 设置线速度
    model_state_pub.twist.linear.x = config["move"]["velocity"]["linear"]["x"].as<double>(0.02);  // x方向线速度（米/秒）
    model_state_pub.twist.linear.y = config["move"]["velocity"]["linear"]["y"].as<double>(0.0);    // y方向线速度（米/秒）
    model_state_pub.twist.linear.z = config["move"]["velocity"]["linear"]["z"].as<double>(0.08);   // z方向线速度（米/秒）

    // 设置角速度
    model_state_pub.twist.angular.x = config["move"]["velocity"]["angular"]["x"].as<double>(0.0);  // x方向角速度（弧度/秒）
    model_state_pub.twist.angular.y = config["move"]["velocity"]["angular"]["y"].as<double>(0.0);  // y方向角速度（弧度/秒）
    model_state_pub.twist.angular.z = config["move"]["velocity"]["angular"]["z"].as<double>(0.0);  // z方向角速度（弧度/秒）

    // 设置参考坐标系
    model_state_pub.reference_frame = "base_link";

    RCLCPP_INFO(node->get_logger(),
                "开始在机器人坐标系中移动机器人: 线速度(x=%.2fm/s, "
                "z=%.2fm/s)",
                model_state_pub.twist.linear.x,
                model_state_pub.twist.linear.z);
    
    // 发布频率
    int publish_rate = config["move"]["publish_rate"].as<int>(1000);
    
    while (rclcpp::ok()) {
      // 发布模型状态
      move_publisher->publish(model_state_pub);
      
      // 睡眠以保持指定的发布频率
      rclcpp::Rate loop_rate(publish_rate);
      loop_rate.sleep();
    }
  }

  RCLCPP_WARN(node->get_logger(), "节点关闭，停止移动机器人");
  rclcpp::shutdown();
  return 0;
}