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

#ifndef GO1_CONTROLLER_UNITREE_JOINT_CONTROL_TOOL_H_
#define GO1_CONTROLLER_UNITREE_JOINT_CONTROL_TOOL_H_

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace go1_controller {

// 电机控制常量
// 位置停止值
constexpr double kPosStopF = 2.146E+9;
// 速度停止值
constexpr double kVelStopF = 16000.0;

/**
 * @brief 伺服命令结构体
 * 
 * 该结构体定义了电机的控制命令
 */
struct ServoCmd {
  uint8_t mode;           // 控制模式
  double pos;             // 目标位置
  double posStiffness;    // 位置刚度
  double vel;             // 目标速度
  double velStiffness;    // 速度刚度
  double torque;          // 目标力矩
};

/**
 * @brief 限制值在指定范围内
 * 
 * @param val 输入值
 * @param min_val 最小值
 * @param max_val 最大值
 * @return 限制后的值
 */
double Clamp(double val, double min_val, double max_val);

/**
 * @brief 计算速度
 * 
 * @param current_position 当前位置
 * @param last_position 上一位置
 * @param last_velocity 上一速度
 * @param duration 时间间隔
 * @return 计算得到的速度
 */
double ComputeVel(double current_position,
                  double last_position,
                  double last_velocity,
                  double duration);

/**
 * @brief 计算力矩
 * 
 * @param current_position 当前位置
 * @param current_velocity 当前速度
 * @param cmd 伺服命令
 * @return 计算得到的力矩
 */
double ComputeTorque(double current_position,
                     double current_velocity,
                     const ServoCmd& cmd);

}  // namespace go1_controller

#endif  // GO1_CONTROLLER_UNITREE_JOINT_CONTROL_TOOL_H_