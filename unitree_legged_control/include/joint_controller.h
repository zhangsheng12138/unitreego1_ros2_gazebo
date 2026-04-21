/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#ifndef _UNITREE_ROS_JOINT_CONTROLLER_H_
#define _UNITREE_ROS_JOINT_CONTROLLER_H_

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/publisher.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <control_toolbox/pid.hpp>
#include <urdf/model.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include "unitree_legged_msgs/msg/motor_cmd.hpp"
#include "unitree_legged_msgs/msg/motor_state.hpp"
#include "unitree_joint_control_tool.h"

#define PMSM      (0x0A)
#define BRAKE     (0x00)
#define PosStopF  (2.146E+9f)
#define VelStopF  (16000.0f)

namespace unitree_legged_control
{
    class UnitreeJointController : public controller_interface::ControllerInterface
    {
public:
        UnitreeJointController();
        ~UnitreeJointController() = default;

        controller_interface::CallbackReturn on_init() override;
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
        controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
        controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
        bool initJoint(const std::string & joint_name);
        void setCommandCB(const unitree_legged_msgs::msg::MotorCmd::SharedPtr msg);
        void setTorqueCB(const geometry_msgs::msg::WrenchStamped::SharedPtr msg);
        void positionLimits(double &position);
        void velocityLimits(double &velocity);
        void effortLimits(double &effort);

        // Joint handle
        hardware_interface::JointHandle joint_;
        std::string joint_name_;
        std::string namespace_;

        // State
        unitree_legged_msgs::msg::MotorState last_state_;
        unitree_legged_msgs::msg::MotorCmd last_cmd_;
        ServoCmd servo_cmd_;
        double sensor_torque_;

        // Joint type
        bool is_hip_, is_thigh_, is_calf_;
        urdf::JointConstSharedPtr joint_urdf_;

        // ROS2 interface
        rclcpp::Subscription<unitree_legged_msgs::msg::MotorCmd>::SharedPtr sub_cmd_;
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr sub_ft_;
        using RealtimeStatePublisher = realtime_tools::RealtimePublisher<unitree_legged_msgs::msg::MotorState>;
        std::unique_ptr<RealtimeStatePublisher> state_pub_;
        realtime_tools::RealtimeBuffer<unitree_legged_msgs::msg::MotorCmd> cmd_buffer_;

        control_toolbox::Pid pid_controller_;
    };
}

#endif
