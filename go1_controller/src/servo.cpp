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

#include "rclcpp/rclcpp.hpp"
#include "go1_controller/msg/low_cmd.hpp"
#include "go1_controller/msg/low_state.hpp"
#include "go1_controller/msg/motor_cmd.hpp"
#include "go1_controller/msg/motor_state.hpp"
#include "go1_controller/msg/imu_data.hpp"
#include "body.h"

#include <cmath>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <chrono>
#include <string>
#include <thread>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace {

// 全局参数
struct ServoParams {
  int queue_size = 10;      // 消息队列大小
  int motor_count = 12;     // 电机数量
  int foot_count = 4;       // 足端数量
  int max_wait_count = 500;  // 最大等待次数
  std::string imu_topic = "/trunk_imu";  // IMU话题
  std::string foot_force_topic_prefix = "/visual/";  // 足端力话题前缀
  std::string foot_force_topic_suffix = "_foot_contact/the_force";  // 足端力话题后缀
  std::string robot_name = "go1";  // 机器人名称
} g_servo_params;

/**
 * @brief 加载YAML参数
 * 
 * @param robot_name 机器人名称
 */
void LoadParams(const std::string& robot_name) {
  try {
    std::string pkg_path = ament_index_cpp::get_package_share_directory("unitree_gazebo");
    std::string yaml_path = pkg_path + "/config/go1_env_parameter.yaml";
    YAML::Node config = YAML::LoadFile(yaml_path);
    
    // 加载机器人名称
    g_servo_params.robot_name = robot_name;
    
    // 加载话题配置
    if (config["topics"] && config["topics"]["imu"]) {
      g_servo_params.imu_topic = config["topics"]["imu"].as<std::string>();
    }
    
    if (config["topics"] && config["topics"]["force"]) {
      // 可以在这里加载力相关的话题配置
    }
    
    RCLCPP_INFO(rclcpp::get_logger("servo"), "参数加载成功");
  } catch (const std::exception& e) {
    RCLCPP_WARN(rclcpp::get_logger("servo"), "参数加载失败: %s，使用默认值", e.what());
  }
}

/**
 * @brief 获取电机话题前缀
 * 
 * @param robot_name 机器人名称
 * @return 电机话题前缀
 */
const std::string& GetMotorTopicPrefix(const std::string& robot_name) {
  static const std::string prefix = "/" + robot_name + "_gazebo/";
  return prefix;
}

/**
 * @brief 获取电机命令话题
 * 
 * @param prefix 话题前缀
 * @param index 电机索引
 * @return 电机命令话题
 */
std::string GetMotorCommandTopic(const std::string& prefix, int index) {
  static const char* motor_names[] = {
      "FR_hip_controller/command", "FR_thigh_controller/command",
      "FR_calf_controller/command", "FL_hip_controller/command",
      "FL_thigh_controller/command", "FL_calf_controller/command",
      "RR_hip_controller/command", "RR_thigh_controller/command",
      "RR_calf_controller/command", "RL_hip_controller/command",
      "RL_thigh_controller/command", "RL_calf_controller/command"};
  return prefix + motor_names[index];
}

/**
 * @brief 获取电机状态话题
 * 
 * @param prefix 话题前缀
 * @param index 电机索引
 * @return 电机状态话题
 */
std::string GetMotorStateTopic(const std::string& prefix, int index) {
  static const char* motor_names[] = {
      "FR_hip_controller/state", "FR_thigh_controller/state",
      "FR_calf_controller/state", "FL_hip_controller/state",
      "FL_thigh_controller/state", "FL_calf_controller/state",
      "RR_hip_controller/state", "RR_thigh_controller/state",
      "RR_calf_controller/state", "RL_hip_controller/state",
      "RL_thigh_controller/state", "RL_calf_controller/state"};
  return prefix + motor_names[index];
}

/**
 * @brief 获取足端力话题
 * 
 * @param foot_name 足端名称
 * @return 足端力话题
 */
std::string GetFootForceTopic(const std::string& foot_name) {
  return g_servo_params.foot_force_topic_prefix + foot_name + g_servo_params.foot_force_topic_suffix;
}

}  // namespace

bool start_up = true;  // 启动标志

/**
 * @brief 多线程节点类
 */
class MultiThreadNode : public rclcpp::Node {
 public:
  /**
   * @brief 构造函数
   * 
   * @param robot_name 机器人名称
   */
  explicit MultiThreadNode(const std::string& robot_name)
      : Node("unitree_gazebo_servo") {
    // 加载参数
    LoadParams(robot_name);
    
    // 创建IMU订阅
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        g_servo_params.imu_topic, g_servo_params.queue_size,
        [this](const sensor_msgs::msg::Imu& msg) {
          this->ImuCallback(msg);
        });

    // 创建足端力订阅
    static const char* foot_names[] = {"FR", "FL", "RR", "RL"};
    for (int i = 0; i < g_servo_params.foot_count; ++i) {
      foot_force_sub_[i] = create_subscription<geometry_msgs::msg::WrenchStamped>(
          GetFootForceTopic(foot_names[i]), g_servo_params.queue_size,
          [this, i](const geometry_msgs::msg::WrenchStamped& msg) {
            switch (i) {
              case 0: this->FRFootCallback(msg); break;
              case 1: this->FLFootCallback(msg); break;
              case 2: this->RRFootCallback(msg); break;
              case 3: this->RLFootCallback(msg); break;
            }
          });
    }

    // 创建电机状态订阅
    const auto& servo_topic_prefix = GetMotorTopicPrefix(robot_name);
    for (int i = 0; i < g_servo_params.motor_count; ++i) {
      // 使用lambda函数作为回调，避免std::bind的问题
      servo_sub_[i] = create_subscription<go1_controller::msg::MotorState>(
          GetMotorStateTopic(servo_topic_prefix, i), g_servo_params.queue_size,
          [this, i](const go1_controller::msg::MotorState& msg) {
            this->MotorCallback(msg, i);
          });
    }

    RCLCPP_INFO(this->get_logger(),
                "所有订阅已初始化，机器人: %s",
                robot_name.c_str());
  }

 private:
  /**
   * @brief IMU回调函数
   * 
   * @param msg IMU消息
   */
  void ImuCallback(const sensor_msgs::msg::Imu& msg) {
    std::lock_guard<std::mutex> lock(go1_controller::low_state_mutex);
    
    // 填充四元数数据
    go1_controller::low_state.imu.quaternion[0] = msg.orientation.w;
    go1_controller::low_state.imu.quaternion[1] = msg.orientation.x;
    go1_controller::low_state.imu.quaternion[2] = msg.orientation.y;
    go1_controller::low_state.imu.quaternion[3] = msg.orientation.z;

    // 填充陀螺仪数据
    go1_controller::low_state.imu.gyroscope[0] = msg.angular_velocity.x;
    go1_controller::low_state.imu.gyroscope[1] = msg.angular_velocity.y;
    go1_controller::low_state.imu.gyroscope[2] = msg.angular_velocity.z;

    // 填充加速度计数据
    go1_controller::low_state.imu.accelerometer[0] = msg.linear_acceleration.x;
    go1_controller::low_state.imu.accelerometer[1] = msg.linear_acceleration.y;
    go1_controller::low_state.imu.accelerometer[2] = msg.linear_acceleration.z;
  }

  /**
   * @brief 电机状态回调函数
   * 
   * @param msg 电机状态消息
   * @param motor_index 电机索引
   */
  void MotorCallback(const go1_controller::msg::MotorState& msg,
                     int motor_index) {
    std::lock_guard<std::mutex> lock(go1_controller::low_state_mutex);
    std::lock_guard<std::mutex> lock_start(go1_controller::start_up_mutex);
    start_up = false;
    go1_controller::low_state.motor_state[motor_index].mode = msg.mode;
    go1_controller::low_state.motor_state[motor_index].q = msg.q;
    go1_controller::low_state.motor_state[motor_index].dq = msg.dq;
    go1_controller::low_state.motor_state[motor_index].tau_est = msg.tau_est;
  }

  /**
   * @brief 前右足端力回调函数
   * 
   * @param msg 力消息
   */
  void FRFootCallback(const geometry_msgs::msg::WrenchStamped& msg) {
    std::lock_guard<std::mutex> lock(go1_controller::low_state_mutex);
    go1_controller::low_state.ee_force[0].x = msg.wrench.force.x;
    go1_controller::low_state.ee_force[0].y = msg.wrench.force.y;
    go1_controller::low_state.ee_force[0].z = msg.wrench.force.z;
    go1_controller::low_state.foot_force[0] = msg.wrench.force.z;
  }

  /**
   * @brief 前左足端力回调函数
   * 
   * @param msg 力消息
   */
  void FLFootCallback(const geometry_msgs::msg::WrenchStamped& msg) {
    std::lock_guard<std::mutex> lock(go1_controller::low_state_mutex);
    go1_controller::low_state.ee_force[1].x = msg.wrench.force.x;
    go1_controller::low_state.ee_force[1].y = msg.wrench.force.y;
    go1_controller::low_state.ee_force[1].z = msg.wrench.force.z;
    go1_controller::low_state.foot_force[1] = msg.wrench.force.z;
  }

  /**
   * @brief 后右足端力回调函数
   * 
   * @param msg 力消息
   */
  void RRFootCallback(const geometry_msgs::msg::WrenchStamped& msg) {
    std::lock_guard<std::mutex> lock(go1_controller::low_state_mutex);
    go1_controller::low_state.ee_force[2].x = msg.wrench.force.x;
    go1_controller::low_state.ee_force[2].y = msg.wrench.force.y;
    go1_controller::low_state.ee_force[2].z = msg.wrench.force.z;
    go1_controller::low_state.foot_force[2] = msg.wrench.force.z;
  }

  /**
   * @brief 后左足端力回调函数
   * 
   * @param msg 力消息
   */
  void RLFootCallback(const geometry_msgs::msg::WrenchStamped& msg) {
    std::lock_guard<std::mutex> lock(go1_controller::low_state_mutex);
    go1_controller::low_state.ee_force[3].x = msg.wrench.force.x;
    go1_controller::low_state.ee_force[3].y = msg.wrench.force.y;
    go1_controller::low_state.ee_force[3].z = msg.wrench.force.z;
    go1_controller::low_state.foot_force[3] = msg.wrench.force.z;
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;  // IMU订阅
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
      foot_force_sub_[4];  // 足端力订阅
  rclcpp::Subscription<go1_controller::msg::MotorState>::SharedPtr
      servo_sub_[12];  // 电机状态订阅
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
  auto node = std::make_shared<rclcpp::Node>("unitree_gazebo_servo_main");

  // 声明并获取机器人名称参数
  node->declare_parameter<std::string>("robot_name", "go1");
  std::string robot_name;
  if (!node->get_parameter("robot_name", robot_name)) {
    RCLCPP_WARN(node->get_logger(),
                "获取robot_name失败，使用默认值: go1");
    robot_name = "go1";
  }
  RCLCPP_INFO(node->get_logger(), "robot_name: %s", robot_name.c_str());

  // 创建多线程节点
  auto listen_node = std::make_shared<MultiThreadNode>(robot_name);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(listen_node);
  executor.add_node(node);

  // 初始化low_state数组
  {
    std::lock_guard<std::mutex> lock(go1_controller::low_state_mutex);
    // 初始化电机状态数组
    go1_controller::low_state.motor_state.resize(g_servo_params.motor_count);
    // 初始化足端力数组
    go1_controller::low_state.ee_force.resize(g_servo_params.foot_count);
    go1_controller::low_state.foot_force.resize(g_servo_params.foot_count);
    // 初始化IMU数据
    go1_controller::low_state.imu.quaternion.resize(4);
    go1_controller::low_state.imu.gyroscope.resize(3);
    go1_controller::low_state.imu.accelerometer.resize(3);
  }

  // 等待电机状态初始化
  RCLCPP_INFO(node->get_logger(), "等待电机状态初始化...");
  int wait_count = 0;
  bool local_start_up = true;
  while (rclcpp::ok() && wait_count < g_servo_params.max_wait_count) {
    {
      std::lock_guard<std::mutex> lock(go1_controller::start_up_mutex);
      local_start_up = start_up;
    }
    if (!local_start_up) {
      break;
    }

    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    ++wait_count;
  }
  
  if (local_start_up) {
    RCLCPP_WARN(node->get_logger(),
                "未收到电机状态，使用默认零位置!");
    std::lock_guard<std::mutex> lock(go1_controller::low_cmd_mutex);
    // 确保low_cmd.motor_cmd已经初始化
    if (go1_controller::low_cmd.motor_cmd.empty()) {
      RCLCPP_WARN(node->get_logger(),
                  "low_cmd.motor_cmd 未初始化，初始化...");
      go1_controller::low_cmd.motor_cmd.resize(g_servo_params.motor_count);
    }
    for (int i = 0; i < g_servo_params.motor_count; ++i) {
      go1_controller::low_cmd.motor_cmd[i].q = 0.0;
      go1_controller::low_cmd.motor_cmd[i].dq = 0.0;
      go1_controller::low_cmd.motor_cmd[i].tau = 0.0;
      go1_controller::low_cmd.motor_cmd[i].mode = 1; // 位置控制模式
      go1_controller::low_cmd.motor_cmd[i].kp = 50;  // 比例增益
      go1_controller::low_cmd.motor_cmd[i].kd = 2;   // 微分增益
    }
  } else {
    RCLCPP_INFO(node->get_logger(),
                "电机状态初始化完成，等待次数: %d", wait_count);
  }

  // 创建发布器
  const auto& pub_topic_prefix = GetMotorTopicPrefix(robot_name);
  auto low_state_pub = node->create_publisher<go1_controller::msg::LowState>(
          pub_topic_prefix + "lowState/state", g_servo_params.queue_size);

  // 初始化电机发布器
  for (int i = 0; i < g_servo_params.motor_count; ++i) {
    go1_controller::servo_pub[i] = node->create_publisher<go1_controller::msg::MotorCmd>(
        GetMotorCommandTopic(pub_topic_prefix, i), g_servo_params.queue_size);
    RCLCPP_DEBUG(node->get_logger(), "创建servo_pub[%d]，话题: %s", i,
                 GetMotorCommandTopic(pub_topic_prefix, i).c_str());
  }

  // 初始化运动
  go1_controller::MotionInit();

  // 主循环
  rclcpp::Rate main_rate(1000);  // 1000Hz
  while (rclcpp::ok()) {
    {
      std::lock_guard<std::mutex> lock(go1_controller::low_state_mutex);
      // 确保发布的是low_state而不是low_cmd
      low_state_pub->publish(go1_controller::low_state);
    }
    go1_controller::SendServoCmd();
    executor.spin_some();
    main_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}