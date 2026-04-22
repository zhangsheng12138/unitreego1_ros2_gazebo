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

#ifndef GO1_CONTROLLER_BODY_H_
#define GO1_CONTROLLER_BODY_H_

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include "go1_controller/msg/low_cmd.hpp"
#include "go1_controller/msg/low_state.hpp"
#include "go1_controller/msg/motor_cmd.hpp"

namespace go1_controller {

// 电机控制常量
// 位置停止值
constexpr float kPosStopF = 2.146E+9f;
// 速度停止值
constexpr float kVelStopF = 16000.0f;

// 全局变量
extern rclcpp::Publisher<go1_controller::msg::MotorCmd>::SharedPtr servo_pub[12];
extern go1_controller::msg::LowCmd low_cmd;
extern go1_controller::msg::LowState low_state;
extern std::mutex low_cmd_mutex;
extern std::mutex low_state_mutex;
extern std::mutex start_up_mutex;

// 函数声明

/**
 * @brief 使机器人站立
 * 
 * 该函数将机器人设置为站立姿势
 */
void Stand();

/**
 * @brief 初始化运动控制
 * 
 * 该函数初始化机器人的运动控制参数
 */
void MotionInit();

/**
 * @brief 发送伺服控制命令
 * 
 * 该函数将控制命令发送给所有电机
 */
void SendServoCmd();

/**
 * @brief 移动所有关节到指定位置
 * 
 * @param joint_positions 关节位置数组
 * @param duration 运动持续时间（秒）
 */
void MoveAllPosition(double* joint_positions, double duration);

}  // namespace go1_controller

#endif  // GO1_CONTROLLER_BODY_H_