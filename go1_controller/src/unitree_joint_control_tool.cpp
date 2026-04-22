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

#include "unitree_joint_control_tool.h"

#include <algorithm>

namespace go1_controller {

/**
 * @brief 限制值在指定范围内
 * 
 * @param val 输入值
 * @param min_val 最小值
 * @param max_val 最大值
 * @return 限制后的值
 */
double Clamp(double val, double min_val, double max_val) {
  if (min_val > max_val) {
    std::swap(min_val, max_val);
  }
  return std::max(min_val, std::min(val, max_val));
}

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
                  double duration) {
  if (duration <= 0.0) {
    return 0.0;
  }
  double raw_vel = (current_position - last_position) / duration;
  const double alpha = Clamp(0.8, 0.01, 0.99);
  return alpha * raw_vel + (1.0 - alpha) * last_velocity;
}

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
                     const ServoCmd& cmd) {
  double torque = cmd.posStiffness * (cmd.pos - current_position) +
                  cmd.velStiffness * (cmd.vel - current_velocity) +
                  cmd.torque;
  return torque;
}

}  // namespace go1_controller