/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#ifndef __BODY_H__
#define __BODY_H__

// ROS 2 头文件
#include "rclcpp/rclcpp.hpp"

// ROS 2 消息类型（替换 ROS1 消息）
#include "unitree_legged_msgs/msg/low_cmd.hpp"
#include "unitree_legged_msgs/msg/low_state.hpp"
#include "unitree_legged_msgs/msg/high_state.hpp"

#define PosStopF (2.146E+9f)
#define VelStopF (16000.f)

namespace unitree_model
{

  // 全局发布器（12路关节 + 高层状态）
  extern rclcpp::Publisher<unitree_legged_msgs::msg::LowCmd>::SharedPtr servo_pub[12];
  extern rclcpp::Publisher<unitree_legged_msgs::msg::HighState>::SharedPtr highState_pub;

  // 全局指令与状态
  extern unitree_legged_msgs::msg::LowCmd lowCmd;
  extern unitree_legged_msgs::msg::LowState lowState;

  // 函数接口（保持与原来完全一致）
  void stand();
  void motion_init();
  void sendServoCmd();
  void moveAllPosition(double *jointPositions, double duration);

} // namespace unitree_model

#endif
