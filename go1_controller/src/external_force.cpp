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
#include <geometry_msgs/msg/wrench.hpp>
#include <chrono>
#include <thread>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>

/**
 * @brief 外力控制类
 * 
 * 用于向机器人躯干施加外力
 */
class TeleForceCmd : public rclcpp::Node {
 public:
  /**
   * @brief 构造函数
   */
  explicit TeleForceCmd() : Node("external_force") {
    // 声明并获取机器人名称参数
    declare_parameter<std::string>("robot_name", "go1");
    std::string robot_name;
    get_parameter("robot_name", robot_name);
    
    // 加载参数
    std::string force_topic = LoadParams(robot_name);

    // 初始化力的分量
    fx_ = 0.0;
    fy_ = 0.0;
    fz_ = 0.0;
    
    // 创建力的发布器
    force_pub_ = create_publisher<geometry_msgs::msg::Wrench>(force_topic, 20);

    // 等待订阅者
    while (force_pub_->get_subscription_count() == 0 && rclcpp::ok()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      RCLCPP_INFO(this->get_logger(), "等待力的订阅者...");
    }

    // 发布初始力（零力）
    PublishForce(fx_, fy_, fz_);
    RCLCPP_INFO(this->get_logger(), "外力发布器初始化完成！");
  }
  
  /**
   * @brief 加载YAML参数
   * 
   * @param robot_name 机器人名称
   * @return 力的话题名称
   */
  std::string LoadParams(const std::string& robot_name) {
    try {
      std::string pkg_path = ament_index_cpp::get_package_share_directory("unitree_gazebo");
      std::string yaml_path = pkg_path + "/config/go1_env_parameter.yaml";
      YAML::Node config = YAML::LoadFile(yaml_path);
      
      // 构建默认力话题名称
      std::string force_topic = "/" + robot_name + "/apply_force/trunk";
      
      // 尝试从YAML中加载力话题配置
      if (config["topics"] && config["topics"]["force"]) {
        std::string force_topic_suffix = config["topics"]["force"].as<std::string>();
        force_topic = "/" + robot_name + "/" + force_topic_suffix;
      }
      
      RCLCPP_INFO(this->get_logger(), "外力话题: %s", force_topic.c_str());
      return force_topic;
    } catch (const std::exception& e) {
      RCLCPP_WARN(this->get_logger(), "参数加载失败: %s，使用默认值", e.what());
      return "/" + robot_name + "/apply_force/trunk";
    }
  }

  /**
   * @brief 发布力的命令
   * 
   * @param x x方向的力
   * @param y y方向的力
   * @param z z方向的力
   */
  void PublishForce(double x, double y, double z) {
    geometry_msgs::msg::Wrench force_msg;
    force_msg.force.x = x;
    force_msg.force.y = y;
    force_msg.force.z = z;
    force_pub_->publish(force_msg);
  }

 private:
  double fx_;  // x方向的力
  double fy_;  // y方向的力
  double fz_;  // z方向的力
  rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr force_pub_;  // 力的发布器
};

/**
 * @brief 主函数
 * 
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return 退出码
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TeleForceCmd>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}