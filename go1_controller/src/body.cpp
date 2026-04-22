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

#include "body.h"

#include <cmath>
#include <chrono>
#include <thread>

namespace go1_controller {

// 伺服电机发布器数组
rclcpp::Publisher<go1_controller::msg::MotorCmd>::SharedPtr servo_pub[12] = {
    nullptr};

// 低级命令和状态
go1_controller::msg::LowCmd low_cmd;
go1_controller::msg::LowState low_state;

// 互斥锁
std::mutex low_cmd_mutex;
std::mutex low_state_mutex;
std::mutex start_up_mutex;

namespace {

/**
 * @brief 初始化电机参数
 * 
 * 为所有12个电机设置初始参数，包括控制模式、位置、速度、力矩、比例增益和微分增益
 */
void ParamInit() {
  std::lock_guard<std::mutex> lock(low_cmd_mutex);
  // 确保low_cmd.motor_cmd已经初始化
  if (low_cmd.motor_cmd.empty()) {
    low_cmd.motor_cmd.resize(12);
  }
  for (int i = 0; i < 12; ++i) {
    low_cmd.motor_cmd[i].mode = 1;          // 位置控制模式
    low_cmd.motor_cmd[i].q = kPosStopF;      // 停止位置
    low_cmd.motor_cmd[i].dq = kVelStopF;     // 停止速度
    low_cmd.motor_cmd[i].tau = 0;           // 零力矩
    low_cmd.motor_cmd[i].kp = 50;           // 比例增益
    low_cmd.motor_cmd[i].kd = 2;            // 微分增益
  }
}

}  // namespace

/**
 * @brief 机器人站立动作
 * 
 * 将机器人移动到站立位置，所有关节设置为默认站立角度
 */
void Stand() {
  // 站立位置的关节角度
  // 顺序：FR_hip, FR_thigh, FR_calf, FL_hip, FL_thigh, FL_calf,
  //       RR_hip, RR_thigh, RR_calf, RL_hip, RL_thigh, RL_calf
  double stand_pos[12] = {
      0.0, -0.65, 1.3,  // 前右腿：髋关节，大腿，小腿
      0.0, -0.65, 1.3,  // 前左腿：髋关节，大腿，小腿
      0.0, -0.65, 1.3,  // 后右腿：髋关节，大腿，小腿
      0.0, -0.65, 1.3   // 后左腿：髋关节，大腿，小腿
  };
  MoveAllPosition(stand_pos, 2000);
}

/**
 * @brief 运动初始化
 * 
 * 初始化电机参数并执行站立动作
 */
void MotionInit() {
  ParamInit();
  RCLCPP_INFO(rclcpp::get_logger("go1_controller::MotionInit"),
              "电机参数初始化完成");

  Stand();
  RCLCPP_INFO(rclcpp::get_logger("go1_controller::MotionInit"),
              "站立动作完成");
}

/**
 * @brief 发送伺服电机命令
 * 
 * 将低级别命令发送到所有12个伺服电机
 */
void SendServoCmd() {
  std::lock_guard<std::mutex> lock(low_cmd_mutex);
  for (int i = 0; i < 12; ++i) {
    if (servo_pub[i] == nullptr) {
      RCLCPP_WARN(rclcpp::get_logger("go1_controller::SendServoCmd"),
                  "servo_pub[%d] 未初始化，跳过发布", i);
      continue;
    }

    go1_controller::msg::MotorCmd cmd_msg;
    cmd_msg.mode = low_cmd.motor_cmd[i].mode;
    cmd_msg.q = low_cmd.motor_cmd[i].q;
    cmd_msg.dq = low_cmd.motor_cmd[i].dq;
    cmd_msg.tau = low_cmd.motor_cmd[i].tau;
    cmd_msg.kp = low_cmd.motor_cmd[i].kp;
    cmd_msg.kd = low_cmd.motor_cmd[i].kd;

    servo_pub[i]->publish(cmd_msg);
  }
}

/**
 * @brief 移动所有关节到目标位置
 * 
 * @param target_pos 目标关节位置数组
 * @param duration 运动持续时间（毫秒）
 */
void MoveAllPosition(double* target_pos, double duration) {
  if (target_pos == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("go1_controller::MoveAllPosition"),
                 "target_pos 为空指针!");
    return;
  }
  if (duration <= 0) {
    RCLCPP_WARN(rclcpp::get_logger("go1_controller::MoveAllPosition"),
                "duration 必须大于0!");
    return;
  }

  double last_pos[12];
  {
    std::lock_guard<std::mutex> lock_state(low_state_mutex);
    // 检查motor_state是否已经初始化
    if (low_state.motor_state.empty()) {
      // 如果未初始化，使用默认值0.0
      RCLCPP_WARN(rclcpp::get_logger("go1_controller::MoveAllPosition"),
                  "low_state.motor_state 未初始化，使用默认值0.0");
      for (int j = 0; j < 12; j++) {
        last_pos[j] = 0.0;
      }
    } else {
      for (int j = 0; j < 12; j++) {
        if (std::isnan(low_state.motor_state[j].q) ||
            std::isinf(low_state.motor_state[j].q)) {
          last_pos[j] = 0.0;
          RCLCPP_WARN(rclcpp::get_logger("go1_controller::MoveAllPosition"),
                      "low_state.motor_state[%d].q 是 NaN/Inf，使用 0.0", j);
        } else {
          last_pos[j] = low_state.motor_state[j].q;
        }
      }
    }
  }

  auto start_time = std::chrono::steady_clock::now();
  rclcpp::Rate rate(1000);  // 1000Hz
  while (rclcpp::ok()) {
    auto current_time = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                       current_time - start_time)
                       .count();
    if (elapsed >= duration) {
      break;
    }

    double percent = static_cast<double>(elapsed) / duration;
    {
      std::lock_guard<std::mutex> lock_cmd(low_cmd_mutex);
      for (int j = 0; j < 12; j++) {
        // 线性插值计算当前位置
        low_cmd.motor_cmd[j].q = last_pos[j] * (1 - percent) +
                                target_pos[j] * percent;
        low_cmd.motor_cmd[j].dq = kVelStopF;
        low_cmd.motor_cmd[j].tau = 0;
      }
    }
    SendServoCmd();
    rate.sleep();
  }
  
  // 确保最终位置正确
  {
    std::lock_guard<std::mutex> lock_cmd(low_cmd_mutex);
    for (int j = 0; j < 12; j++) {
      low_cmd.motor_cmd[j].q = target_pos[j];
      low_cmd.motor_cmd[j].dq = kVelStopF;
      low_cmd.motor_cmd[j].tau = 0;
    }
  }
  SendServoCmd();
  
  RCLCPP_DEBUG(rclcpp::get_logger("go1_controller::MoveAllPosition"),
               "移动到目标位置完成，持续时间: %.2f ms",
               duration);
}

}  // namespace go1_controller