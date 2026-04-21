/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "body.h"
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <thread>

namespace unitree_model {

// 全局发布器数组（ROS2版本）
std::shared_ptr<rclcpp::Publisher<unitree_legged_msgs::msg::MotorCmd>> servo_pub[12];
unitree_legged_msgs::msg::LowCmd lowCmd;
unitree_legged_msgs::msg::LowState lowState;

// 参数初始化（适配ROS2消息字段）
void paramInit()
{
    for(int i=0; i<4; i++){
        lowCmd.motor_cmd[i*3+0].mode = 0x0A;
        lowCmd.motor_cmd[i*3+0].kp = 70;
        lowCmd.motor_cmd[i*3+0].dq = 0;
        lowCmd.motor_cmd[i*3+0].kd = 3;
        lowCmd.motor_cmd[i*3+0].tau = 0;
        
        lowCmd.motor_cmd[i*3+1].mode = 0x0A;
        lowCmd.motor_cmd[i*3+1].kp = 180;
        lowCmd.motor_cmd[i*3+1].dq = 0;
        lowCmd.motor_cmd[i*3+1].kd = 8;
        lowCmd.motor_cmd[i*3+1].tau = 0;
        
        lowCmd.motor_cmd[i*3+2].mode = 0x0A;
        lowCmd.motor_cmd[i*3+2].kp = 300;
        lowCmd.motor_cmd[i*3+2].dq = 0;
        lowCmd.motor_cmd[i*3+2].kd = 15;
        lowCmd.motor_cmd[i*3+2].tau = 0;
    }
    for(int i=0; i<12; i++){
        lowCmd.motor_cmd[i].q = lowState.motor_state[i].q;
    }
}

// 站立姿态
void stand()
{   
    double pos[12] = {0.0, 0.67, -1.3, -0.0, 0.67, -1.3, 
                      0.0, 0.67, -1.3, -0.0, 0.67, -1.3};
    moveAllPosition(pos, 2*1000);
}

// 运动初始化
void motion_init()
{
    paramInit();
    stand();
}

// 发送电机指令（ROS2版本）
void sendServoCmd()
{
    for(int m=0; m<12; m++){
        if (servo_pub[m]) { // 检查发布器是否初始化
            servo_pub[m]->publish(lowCmd.motor_cmd[m]);
        }
    }
    std::this_thread::sleep_for(1ms); // 替换原usleep(1000)
}

// 所有关节位置移动（适配ROS2）
void moveAllPosition(double* targetPos, double duration)
{
    double pos[12] ,lastPos[12], percent;
    for(int j=0; j<12; j++) lastPos[j] = lowState.motor_state[j].q;
    
    rclcpp::Rate rate(1000); // 1000Hz控制频率
    for(int i=1; i<=duration; i++){
        if(!rclcpp::ok()) break;
        
        percent = (double)i/duration;
        for(int j=0; j<12; j++){
            lowCmd.motor_cmd[j].q = lastPos[j]*(1-percent) + targetPos[j]*percent; 
        }
        sendServoCmd();
        rate.sleep();
    }
}

} // namespace unitree_model
