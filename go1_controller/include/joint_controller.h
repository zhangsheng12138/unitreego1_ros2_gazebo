// Copyright 2024 Unitree Robotics
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

#ifndef GO1_CONTROLLER_JOINT_CONTROLLER_H_
#define GO1_CONTROLLER_JOINT_CONTROLLER_H_

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <urdf/model.h>

#include "unitree_joint_control_tool.h"
#include "go1_controller/msg/motor_cmd.hpp"
#include "go1_controller/msg/motor_state.hpp"

namespace go1_controller {

// 电机模式常量
// PMSM模式
constexpr uint8_t kPmsmMode = 0x0A;
// 刹车模式
constexpr uint8_t kBrakeMode = 0x00;

/**
 * @brief Go1机器人关节控制器
 * 
 * 该类实现了Go1机器人的关节控制器，继承自ControllerInterface
 */
class JointController : public controller_interface::ControllerInterface {
 public:
  /**
   * @brief 构造函数
   */
  JointController();
  
  /**
   * @brief 析构函数
   */
  ~JointController() = default;

  /**
   * @brief 初始化控制器
   * 
   * @return 初始化结果
   */
  controller_interface::CallbackReturn on_init() override;
  
  /**
   * @brief 配置控制器
   * 
   * @param previous_state 之前的状态
   * @return 配置结果
   */
  controller_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;
  
  /**
   * @brief 激活控制器
   * 
   * @param previous_state 之前的状态
   * @return 激活结果
   */
  controller_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;
  
  /**
   * @brief 停用控制器
   * 
   * @param previous_state 之前的状态
   * @return 停用结果
   */
  controller_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;
  
  /**
   * @brief 更新控制器
   * 
   * @param time 当前时间
   * @param period 时间周期
   * @return 更新结果
   */
  controller_interface::return_type update(const rclcpp::Time& time,
                                           const rclcpp::Duration& period) override;

  /**
   * @brief 获取命令接口配置
   * 
   * @return 命令接口配置
   */
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  
  /**
   * @brief 获取状态接口配置
   * 
   * @return 状态接口配置
   */
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

 private:
  /**
   * @brief 初始化关节
   * 
   * @param joint_name 关节名称
   * @return 初始化结果
   */
  bool InitJoint(const std::string& joint_name);
  
  /**
   * @brief 命令回调函数
   * 
   * @param msg 电机命令消息
   */
  void SetCommandCallback(const go1_controller::msg::MotorCmd::SharedPtr msg);
  
  /**
   * @brief 位置限制
   * 
   * @param position 位置值
   */
  void PositionLimits(double& position);
  
  /**
   * @brief 速度限制
   * 
   * @param velocity 速度值
   */
  void VelocityLimits(double& velocity);
  
  /**
   * @brief 力矩限制
   * 
   * @param effort 力矩值
   */
  void EffortLimits(double& effort);

  // 成员变量
  std::string joint_name_;
  std::string effort_interface_name_;
  std::string position_interface_name_;
  std::string velocity_interface_name_;

  go1_controller::msg::MotorState last_state_;
  go1_controller::msg::MotorCmd last_cmd_;
  ServoCmd servo_cmd_;
  urdf::JointConstSharedPtr joint_urdf_;

  rclcpp::Subscription<go1_controller::msg::MotorCmd>::SharedPtr sub_cmd_;
  using RealtimeStatePublisher = realtime_tools::RealtimePublisher<go1_controller::msg::MotorState>;
  std::unique_ptr<RealtimeStatePublisher> state_pub_;
  realtime_tools::RealtimeBuffer<go1_controller::msg::MotorCmd> cmd_buffer_;
};

}  // namespace go1_controller

#endif  // GO1_CONTROLLER_JOINT_CONTROLLER_H_