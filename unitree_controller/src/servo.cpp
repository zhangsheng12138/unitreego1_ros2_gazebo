/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "rclcpp/rclcpp.hpp"
#include <stdio.h>
#include <stdlib.h>
#include "unitree_legged_msgs/msg/low_cmd.hpp"
#include "unitree_legged_msgs/msg/low_state.hpp"
#include "unitree_legged_msgs/msg/motor_cmd.hpp"
#include "unitree_legged_msgs/msg/motor_state.hpp"
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vector>
#include <string>
#include <math.h>
#include <nav_msgs/msg/odometry.hpp>
#include "body.h"
#include <chrono>
#include <thread>

using namespace std;
using namespace unitree_model;
using namespace std::chrono_literals;

bool start_up = true;

class MultiThreadNode : public rclcpp::Node
{
public:
    MultiThreadNode(const std::string& robot_name) : Node("unitree_gazebo_servo")
    {
        // 订阅IMU
        imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            "/trunk_imu", 1, 
            std::bind(&MultiThreadNode::imuCallback, this, std::placeholders::_1));

        // 订阅足端力
        footForce_sub[0] = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/visual/FR_foot_contact/the_force", 1,
            std::bind(&MultiThreadNode::FRfootCallback, this, std::placeholders::_1));
        footForce_sub[1] = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/visual/FL_foot_contact/the_force", 1,
            std::bind(&MultiThreadNode::FLfootCallback, this, std::placeholders::_1));
        footForce_sub[2] = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/visual/RR_foot_contact/the_force", 1,
            std::bind(&MultiThreadNode::RRfootCallback, this, std::placeholders::_1));
        footForce_sub[3] = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/visual/RL_foot_contact/the_force", 1,
            std::bind(&MultiThreadNode::RLfootCallback, this, std::placeholders::_1));

        // 订阅电机状态
        std::string servo_topic_prefix = "/" + robot_name + "_gazebo/";
        servo_sub[0] = this->create_subscription<unitree_legged_msgs::msg::MotorState>(
            servo_topic_prefix + "FR_hip_controller/state", 1,
            std::bind(&MultiThreadNode::FRhipCallback, this, std::placeholders::_1));
        servo_sub[1] = this->create_subscription<unitree_legged_msgs::msg::MotorState>(
            servo_topic_prefix + "FR_thigh_controller/state", 1,
            std::bind(&MultiThreadNode::FRthighCallback, this, std::placeholders::_1));
        servo_sub[2] = this->create_subscription<unitree_legged_msgs::msg::MotorState>(
            servo_topic_prefix + "FR_calf_controller/state", 1,
            std::bind(&MultiThreadNode::FRcalfCallback, this, std::placeholders::_1));
        servo_sub[3] = this->create_subscription<unitree_legged_msgs::msg::MotorState>(
            servo_topic_prefix + "FL_hip_controller/state", 1,
            std::bind(&MultiThreadNode::FLhipCallback, this, std::placeholders::_1));
        servo_sub[4] = this->create_subscription<unitree_legged_msgs::msg::MotorState>(
            servo_topic_prefix + "FL_thigh_controller/state", 1,
            std::bind(&MultiThreadNode::FLthighCallback, this, std::placeholders::_1));
        servo_sub[5] = this->create_subscription<unitree_legged_msgs::msg::MotorState>(
            servo_topic_prefix + "FL_calf_controller/state", 1,
            std::bind(&MultiThreadNode::FLcalfCallback, this, std::placeholders::_1));
        servo_sub[6] = this->create_subscription<unitree_legged_msgs::msg::MotorState>(
            servo_topic_prefix + "RR_hip_controller/state", 1,
            std::bind(&MultiThreadNode::RRhipCallback, this, std::placeholders::_1));
        servo_sub[7] = this->create_subscription<unitree_legged_msgs::msg::MotorState>(
            servo_topic_prefix + "RR_thigh_controller/state", 1,
            std::bind(&MultiThreadNode::RRthighCallback, this, std::placeholders::_1));
        servo_sub[8] = this->create_subscription<unitree_legged_msgs::msg::MotorState>(
            servo_topic_prefix + "RR_calf_controller/state", 1,
            std::bind(&MultiThreadNode::RRcalfCallback, this, std::placeholders::_1));
        servo_sub[9] = this->create_subscription<unitree_legged_msgs::msg::MotorState>(
            servo_topic_prefix + "RL_hip_controller/state", 1,
            std::bind(&MultiThreadNode::RLhipCallback, this, std::placeholders::_1));
        servo_sub[10] = this->create_subscription<unitree_legged_msgs::msg::MotorState>(
            servo_topic_prefix + "RL_thigh_controller/state", 1,
            std::bind(&MultiThreadNode::RLthighCallback, this, std::placeholders::_1));
        servo_sub[11] = this->create_subscription<unitree_legged_msgs::msg::MotorState>(
            servo_topic_prefix + "RL_calf_controller/state", 1,
            std::bind(&MultiThreadNode::RLcalfCallback, this, std::placeholders::_1));
    }

    void imuCallback(const sensor_msgs::msg::Imu & msg)
    { 
        lowState.imu.quaternion[0] = msg.orientation.w;
        lowState.imu.quaternion[1] = msg.orientation.x;
        lowState.imu.quaternion[2] = msg.orientation.y;
        lowState.imu.quaternion[3] = msg.orientation.z;

        lowState.imu.gyroscope[0] = msg.angular_velocity.x;
        lowState.imu.gyroscope[1] = msg.angular_velocity.y;
        lowState.imu.gyroscope[2] = msg.angular_velocity.z;
        
        lowState.imu.accelerometer[0] = msg.linear_acceleration.x;
        lowState.imu.accelerometer[1] = msg.linear_acceleration.y;
        lowState.imu.accelerometer[2] = msg.linear_acceleration.z;
    }

    void FRhipCallback(const unitree_legged_msgs::msg::MotorState& msg)
    {
        start_up = false;
        lowState.motor_state[0].mode = msg.mode;
        lowState.motor_state[0].q = msg.q;
        lowState.motor_state[0].dq = msg.dq;
        lowState.motor_state[0].tau_est = msg.tau_est;
    }

    void FRthighCallback(const unitree_legged_msgs::msg::MotorState& msg)
    {
        lowState.motor_state[1].mode = msg.mode;
        lowState.motor_state[1].q = msg.q;
        lowState.motor_state[1].dq = msg.dq;
        lowState.motor_state[1].tau_est = msg.tau_est;
    }

    void FRcalfCallback(const unitree_legged_msgs::msg::MotorState& msg)
    {
        lowState.motor_state[2].mode = msg.mode;
        lowState.motor_state[2].q = msg.q;
        lowState.motor_state[2].dq = msg.dq;
        lowState.motor_state[2].tau_est = msg.tau_est;
    }

    void FLhipCallback(const unitree_legged_msgs::msg::MotorState& msg)
    {
        start_up = false;
        lowState.motor_state[3].mode = msg.mode;
        lowState.motor_state[3].q = msg.q;
        lowState.motor_state[3].dq = msg.dq;
        lowState.motor_state[3].tau_est = msg.tau_est;
    }

    void FLthighCallback(const unitree_legged_msgs::msg::MotorState& msg)
    {
        lowState.motor_state[4].mode = msg.mode;
        lowState.motor_state[4].q = msg.q;
        lowState.motor_state[4].dq = msg.dq;
        lowState.motor_state[4].tau_est = msg.tau_est;
    }

    void FLcalfCallback(const unitree_legged_msgs::msg::MotorState& msg)
    {
        lowState.motor_state[5].mode = msg.mode;
        lowState.motor_state[5].q = msg.q;
        lowState.motor_state[5].dq = msg.dq;
        lowState.motor_state[5].tau_est = msg.tau_est;
    }

    void RRhipCallback(const unitree_legged_msgs::msg::MotorState& msg)
    {
        start_up = false;
        lowState.motor_state[6].mode = msg.mode;
        lowState.motor_state[6].q = msg.q;
        lowState.motor_state[6].dq = msg.dq;
        lowState.motor_state[6].tau_est = msg.tau_est;
    }

    void RRthighCallback(const unitree_legged_msgs::msg::MotorState& msg)
    {
        lowState.motor_state[7].mode = msg.mode;
        lowState.motor_state[7].q = msg.q;
        lowState.motor_state[7].dq = msg.dq;
        lowState.motor_state[7].tau_est = msg.tau_est;
    }

    void RRcalfCallback(const unitree_legged_msgs::msg::MotorState& msg)
    {
        lowState.motor_state[8].mode = msg.mode;
        lowState.motor_state[8].q = msg.q;
        lowState.motor_state[8].dq = msg.dq;
        lowState.motor_state[8].tau_est = msg.tau_est;
    }

    void RLhipCallback(const unitree_legged_msgs::msg::MotorState& msg)
    {
        start_up = false;
        lowState.motor_state[9].mode = msg.mode;
        lowState.motor_state[9].q = msg.q;
        lowState.motor_state[9].dq = msg.dq;
        lowState.motor_state[9].tau_est = msg.tau_est;
    }

    void RLthighCallback(const unitree_legged_msgs::msg::MotorState& msg)
    {
        lowState.motor_state[10].mode = msg.mode;
        lowState.motor_state[10].q = msg.q;
        lowState.motor_state[10].dq = msg.dq;
        lowState.motor_state[10].tau_est = msg.tau_est;
    }

    void RLcalfCallback(const unitree_legged_msgs::msg::MotorState& msg)
    {
        lowState.motor_state[11].mode = msg.mode;
        lowState.motor_state[11].q = msg.q;
        lowState.motor_state[11].dq = msg.dq;
        lowState.motor_state[11].tau_est = msg.tau_est;
    }

    void FRfootCallback(const geometry_msgs::msg::WrenchStamped& msg)
    {
        lowState.ee_force[0].x = msg.wrench.force.x;
        lowState.ee_force[0].y = msg.wrench.force.y;
        lowState.ee_force[0].z = msg.wrench.force.z;
        lowState.foot_force[0] = msg.wrench.force.z;
    }

    void FLfootCallback(const geometry_msgs::msg::WrenchStamped& msg)
    {
        lowState.ee_force[1].x = msg.wrench.force.x;
        lowState.ee_force[1].y = msg.wrench.force.y;
        lowState.ee_force[1].z = msg.wrench.force.z;
        lowState.foot_force[1] = msg.wrench.force.z;
    }

    void RRfootCallback(const geometry_msgs::msg::WrenchStamped& msg)
    {
        lowState.ee_force[2].x = msg.wrench.force.x;
        lowState.ee_force[2].y = msg.wrench.force.y;
        lowState.ee_force[2].z = msg.wrench.force.z;
        lowState.foot_force[2] = msg.wrench.force.z;
    }

    void RLfootCallback(const geometry_msgs::msg::WrenchStamped& msg)
    {
        lowState.ee_force[3].x = msg.wrench.force.x;
        lowState.ee_force[3].y = msg.wrench.force.y;
        lowState.ee_force[3].z = msg.wrench.force.z;
        lowState.foot_force[3] = msg.wrench.force.z;
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr footForce_sub[4];
    rclcpp::Subscription<unitree_legged_msgs::msg::MotorState>::SharedPtr servo_sub[12];
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("unitree_gazebo_servo_main");

    // 获取机器人名称参数
    std::string robot_name;
    if (!node->get_parameter("/robot_name", robot_name)) {
        RCLCPP_WARN(node->get_logger(), "Failed to get /robot_name, use default: unitree_go1");
        robot_name = "unitree_go1";
    }
    RCLCPP_INFO(node->get_logger(), "robot_name: %s", robot_name.c_str());

    // 创建订阅节点并启动多线程执行器
    auto listen_node = std::make_shared<MultiThreadNode>(robot_name);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(listen_node);
    executor.add_node(node);
    
    // 等待300ms获取初始状态
    std::this_thread::sleep_for(300ms);

    // 创建发布器
    std::string pub_topic_prefix = "/" + robot_name + "_gazebo/";
    auto lowState_pub = node->create_publisher<unitree_legged_msgs::msg::LowState>(
        pub_topic_prefix + "lowState/state", 1);
    
    // 电机指令发布器
    for (int i = 0; i < 12; ++i) {
        std::string motor_cmd_topic;
        if (i == 0) motor_cmd_topic = pub_topic_prefix + "FR_hip_controller/command";
        else if (i == 1) motor_cmd_topic = pub_topic_prefix + "FR_thigh_controller/command";
        else if (i == 2) motor_cmd_topic = pub_topic_prefix + "FR_calf_controller/command";
        else if (i == 3) motor_cmd_topic = pub_topic_prefix + "FL_hip_controller/command";
        else if (i == 4) motor_cmd_topic = pub_topic_prefix + "FL_thigh_controller/command";
        else if (i == 5) motor_cmd_topic = pub_topic_prefix + "FL_calf_controller/command";
        else if (i == 6) motor_cmd_topic = pub_topic_prefix + "RR_hip_controller/command";
        else if (i == 7) motor_cmd_topic = pub_topic_prefix + "RR_thigh_controller/command";
        else if (i == 8) motor_cmd_topic = pub_topic_prefix + "RR_calf_controller/command";
        else if (i == 9) motor_cmd_topic = pub_topic_prefix + "RL_hip_controller/command";
        else if (i == 10) motor_cmd_topic = pub_topic_prefix + "RL_thigh_controller/command";
        else if (i == 11) motor_cmd_topic = pub_topic_prefix + "RL_calf_controller/command";
        
        servo_pub[i] = node->create_publisher<unitree_legged_msgs::msg::MotorCmd>(motor_cmd_topic, 1);
    }

    // 初始化运动参数
    motion_init();

    // 主循环
    while (rclcpp::ok()) {
        lowState_pub->publish(lowState);
        sendServoCmd();
        executor.spin_some();
        std::this_thread::sleep_for(1ms); // 匹配原1000Hz频率
    }

    rclcpp::shutdown();
    return 0;
}
