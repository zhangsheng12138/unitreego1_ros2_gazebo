/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#ifndef _UNITREE_JOINT_CONTROL_TOOL_H_
#define _UNITREE_JOINT_CONTROL_TOOL_H_

#include <stdio.h>
#include <stdint.h>
#include <algorithm>
#include <cmath>

// 与 ROS1/ROS2 无关，纯数学计算宏
#define posStopF (2.146E+9f)   // 停止位置控制
#define velStopF (16000.0f)    // 停止速度控制

// 伺服电机指令结构体
typedef struct 
{
    uint8_t mode;
    double pos;           // 目标位置
    double posStiffness;  // 位置刚度 Kp
    double vel;           // 目标速度
    double velStiffness;  // 速度刚度 Kd
    double torque;        // 前馈力矩
} ServoCmd;

// 数值限幅函数
double clamp(double& val, double min_val, double max_val);
// 速度估算（滤波）
double computeVel(double current_position, double last_position, double last_velocity, double duration);
// 力矩计算：tau = Kp*(pos-target) + Kd*(vel-target) + feedforward
double computeTorque(double current_position, double current_velocity, ServoCmd& cmd);

#endif
