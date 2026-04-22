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

#include "joint_controller.h"

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>

namespace go1_controller {

/**
 * @brief 构造函数
 */
JointController::JointController() {
  last_cmd_ = go1_controller::msg::MotorCmd();
  last_state_ = go1_controller::msg::MotorState();
  servo_cmd_ = ServoCmd{0, 0.0, 0.0, 0.0, 0.0, 0.0};
}

/**
 * @brief 控制器初始化
 * 
 * @return 初始化结果
 */
controller_interface::CallbackReturn JointController::on_init() {
  auto node = get_node();
  if (!node) {
    RCLCPP_ERROR(rclcpp::get_logger("JointController"),
                 "获取节点句柄失败");
    return controller_interface::CallbackReturn::FAILURE;
  }

  if (!node->has_parameter("joint")) {
    node->declare_parameter("joint", "");
  }
  node->get_parameter("joint", joint_name_);
  cmd_buffer_.initRT(go1_controller::msg::MotorCmd());
  return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 获取命令接口配置
 * 
 * @return 命令接口配置
 */
controller_interface::InterfaceConfiguration JointController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.push_back(joint_name_ + "/effort");
  return config;
}

/**
 * @brief 获取状态接口配置
 * 
 * @return 状态接口配置
 */
controller_interface::InterfaceConfiguration JointController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.push_back(joint_name_ + "/position");
  config.names.push_back(joint_name_ + "/velocity");
  config.names.push_back(joint_name_ + "/effort");
  return config;
}

/**
 * @brief 控制器配置
 * 
 * @param state 生命周期状态
 * @return 配置结果
 */
controller_interface::CallbackReturn JointController::on_configure(
    const rclcpp_lifecycle::State&) {
  auto node = get_node();
  if (!node || !node->get_parameter("joint", joint_name_) ||
      joint_name_.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("JointController"),
                 "无效的关节参数");
    return controller_interface::CallbackReturn::FAILURE;
  }

  if (!InitJoint(joint_name_)) {
    return controller_interface::CallbackReturn::FAILURE;
  }

  sub_cmd_ = node->create_subscription<go1_controller::msg::MotorCmd>(
      joint_name_ + "/command", 10,
      [this](const go1_controller::msg::MotorCmd::SharedPtr msg) {
        if (msg) {
          cmd_buffer_.writeFromNonRT(*msg);
        }
      });

  // 创建状态发布器
  auto publisher = node->create_publisher<go1_controller::msg::MotorState>(
      joint_name_ + "/state", 1);
  state_pub_ = std::make_unique<RealtimeStatePublisher>(publisher);

  RCLCPP_INFO(node->get_logger(), "控制器配置完成: %s",
              joint_name_.c_str());
  return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 初始化关节
 * 
 * @param joint_name 关节名称
 * @return 初始化结果
 */
bool JointController::InitJoint(const std::string& joint_name) {
  auto node = get_node();
  urdf::Model urdf;
  std::string robot_desc;
  if (!node->get_parameter("/robot_description", robot_desc) ||
      !urdf.initString(robot_desc)) {
    RCLCPP_ERROR(node->get_logger(), "URDF加载失败");
    return false;
  }

  joint_urdf_ = urdf.getJoint(joint_name);
  return joint_urdf_ &&
         (joint_urdf_->type == urdf::Joint::REVOLUTE ||
          joint_urdf_->type == urdf::Joint::PRISMATIC);
}

/**
 * @brief 控制器激活
 * 
 * @param state 生命周期状态
 * @return 激活结果
 */
controller_interface::CallbackReturn JointController::on_activate(
    const rclcpp_lifecycle::State&) {
  last_cmd_.mode = kBrakeMode;
  cmd_buffer_.initRT(last_cmd_);
  RCLCPP_INFO(get_node()->get_logger(), "控制器已激活");
  return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 控制器停用
 * 
 * @param state 生命周期状态
 * @return 停用结果
 */
controller_interface::CallbackReturn JointController::on_deactivate(
    const rclcpp_lifecycle::State&) {
  if (!command_interfaces_.empty()) {
    command_interfaces_[0].set_value(0.0);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 控制器更新
 * 
 * @param time 当前时间
 * @param period 更新周期
 * @return 更新结果
 */
controller_interface::return_type JointController::update(
    const rclcpp::Time&, const rclcpp::Duration& /*period*/) {
  if (state_interfaces_.size() < 3 || command_interfaces_.empty()) {
    return controller_interface::return_type::ERROR;
  }

  const auto& cmd = *cmd_buffer_.readFromRT();
  double pos = state_interfaces_[0].get_value();
  double vel = state_interfaces_[1].get_value();
  double eff = state_interfaces_[2].get_value();

  // 初始化伺服命令为刹车模式
  servo_cmd_ = {kBrakeMode, 0, 0, 0, 20, 0};
  if (cmd.mode == kPmsmMode) {
    servo_cmd_.mode = kPmsmMode;
    servo_cmd_.pos = cmd.q;
    PositionLimits(servo_cmd_.pos);
    servo_cmd_.posStiffness =
        (fabs(cmd.q - kPosStopF) < 1e-5) ? 0 : cmd.kp;

    servo_cmd_.vel = cmd.dq;
    VelocityLimits(servo_cmd_.vel);
    servo_cmd_.velStiffness =
        (fabs(cmd.dq - kVelStopF) < 1e-5) ? 0 : cmd.kd;

    servo_cmd_.torque = cmd.tau;
    EffortLimits(servo_cmd_.torque);
  }

  // 计算力矩
  double tau = ComputeTorque(pos, vel, servo_cmd_);
  EffortLimits(tau);
  command_interfaces_[0].set_value(tau);

  // 更新状态
  last_state_.q = pos;
  last_state_.dq = vel;
  last_state_.tau_est = eff;
  last_state_.mode = servo_cmd_.mode;

  // 发布状态
  if (state_pub_ && state_pub_->trylock()) {
    state_pub_->msg_ = last_state_;
    state_pub_->unlockAndPublish();
  }
  return controller_interface::return_type::OK;
}

/**
 * @brief 位置限制
 * 
 * @param p 位置值
 */
void JointController::PositionLimits(double& p) {
  if (joint_urdf_) {
    p = Clamp(p, joint_urdf_->limits->lower, joint_urdf_->limits->upper);
  }
}

/**
 * @brief 速度限制
 * 
 * @param v 速度值
 */
void JointController::VelocityLimits(double& v) {
  if (joint_urdf_) {
    v = Clamp(v, -joint_urdf_->limits->velocity,
              joint_urdf_->limits->velocity);
  }
}

/**
 * @brief 力矩限制
 * 
 * @param e 力矩值
 */
void JointController::EffortLimits(double& e) {
  if (joint_urdf_) {
    e = Clamp(e, -joint_urdf_->limits->effort, joint_urdf_->limits->effort);
  }
}

}  // namespace go1_controller

PLUGINLIB_EXPORT_CLASS(go1_controller::JointController,
                        controller_interface::ControllerInterface)