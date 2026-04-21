/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include "joint_controller.h"
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>

namespace unitree_legged_control
{

    UnitreeJointController::UnitreeJointController()
    {
        last_cmd_ = unitree_legged_msgs::msg::MotorCmd();
        last_state_ = unitree_legged_msgs::msg::MotorState();
        servo_cmd_ = ServoCmd();
        sensor_torque_ = 0.0;
        is_hip_ = false;
        is_thigh_ = false;
        is_calf_ = false;
    }

    controller_interface::CallbackReturn UnitreeJointController::on_init()
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn UnitreeJointController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        auto node = get_node();
        if (!node->has_parameter("joint")) {
            RCLCPP_ERROR(node->get_logger(), "No joint name specified.");
            return controller_interface::CallbackReturn::FAILURE;
        }

        std::string joint_name;
        node->get_parameter("joint", joint_name);
        if (!initJoint(joint_name)) {
            return controller_interface::CallbackReturn::FAILURE;
        }

        // Subscribers
        sub_cmd_ = node->create_subscription<unitree_legged_msgs::msg::MotorCmd>(
            "command", 20, std::bind(&UnitreeJointController::setCommandCB, this, std::placeholders::_1));

        sub_ft_ = node->create_subscription<geometry_msgs::msg::WrenchStamped>(
            "joint_wrench", 1, std::bind(&UnitreeJointController::setTorqueCB, this, std::placeholders::_1));

        // State publisher
        state_pub_ = std::make_unique<RealtimeStatePublisher>(node, "state", 1);

        return controller_interface::CallbackReturn::SUCCESS;
    }

    bool UnitreeJointController::initJoint(const std::string & joint_name)
    {
        joint_name_ = joint_name;
        auto node = get_node();

        urdf::Model urdf;
        if (!urdf.initString(node->get_parameter("robot_description").as_string())) {
            RCLCPP_ERROR(node->get_logger(), "Failed to parse URDF.");
            return false;
        }

        joint_urdf_ = urdf.getJoint(joint_name_);
        if (!joint_urdf_) {
            RCLCPP_ERROR(node->get_logger(), "Joint %s not found in URDF.", joint_name_.c_str());
            return false;
        }

        // Identify joint type
        if (joint_name_.find("hip") != std::string::npos) is_hip_ = true;
        if (joint_name_.find("calf") != std::string::npos) is_calf_ = true;

        return true;
    }

    void UnitreeJointController::setCommandCB(const unitree_legged_msgs::msg::MotorCmd::SharedPtr msg)
    {
        last_cmd_ = *msg;
        cmd_buffer_.writeFromNonRT(last_cmd_);
    }

    void UnitreeJointController::setTorqueCB(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
    {
        if (is_hip_) sensor_torque_ = msg->wrench.torque.x;
        else         sensor_torque_ = msg->wrench.torque.y;
    }

    controller_interface::CallbackReturn UnitreeJointController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        // Get current position
        double pos = joint_.get_position();
        last_cmd_.q = pos;
        last_state_.q = pos;
        last_cmd_.dq = 0.0;
        last_cmd_.tau = 0.0;
        cmd_buffer_.initRT(last_cmd_);
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn UnitreeJointController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
    {
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type UnitreeJointController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
    {
        last_cmd_ = *cmd_buffer_.readFromRT();
        double current_pos = joint_.get_position();
        double current_vel = joint_.get_velocity();

        // Mode: PMSM or BRAKE
        if (last_cmd_.mode == PMSM) {
            servo_cmd_.pos = last_cmd_.q;
            positionLimits(servo_cmd_.pos);
            servo_cmd_.posStiffness = last_cmd_.Kp;
            if (fabs(last_cmd_.q - PosStopF) < 1e-5) servo_cmd_.posStiffness = 0;

            servo_cmd_.vel = last_cmd_.dq;
            velocityLimits(servo_cmd_.vel);
            servo_cmd_.velStiffness = last_cmd_.Kd;
            if (fabs(last_cmd_.dq - VelStopF) < 1e-5) servo_cmd_.velStiffness = 0;

            servo_cmd_.torque = last_cmd_.tau;
            effortLimits(servo_cmd_.torque);
        }

        if (last_cmd_.mode == BRAKE) {
            servo_cmd_.posStiffness = 0;
            servo_cmd_.vel = 0;
            servo_cmd_.velStiffness = 20;
            servo_cmd_.torque = 0;
            effortLimits(servo_cmd_.torque);
        }

        // Compute torque
        double calc_torque = computeTorque(current_pos, current_vel, servo_cmd_);
        effortLimits(calc_torque);
        joint_.set_command(calc_torque);

        // Update state
        last_state_.q = current_pos;
        last_state_.dq = current_vel;
        last_state_.tauEst = joint_.get_effort();

        // Publish
        if (state_pub_ && state_pub_->trylock()) {
            state_pub_->msg_ = last_state_;
            state_pub_->unlockAndPublish();
        }

        return controller_interface::return_type::OK;
    }

    void UnitreeJointController::positionLimits(double &pos)
    {
        if (joint_urdf_->type == urdf::Joint::REVOLUTE || joint_urdf_->type == urdf::Joint::PRISMATIC) {
            pos = std::clamp(pos, joint_urdf_->limits->lower, joint_urdf_->limits->upper);
        }
    }

    void UnitreeJointController::velocityLimits(double &vel)
    {
        if (joint_urdf_->type == urdf::Joint::REVOLUTE || joint_urdf_->type == urdf::Joint::PRISMATIC) {
            vel = std::clamp(vel, -joint_urdf_->limits->velocity, joint_urdf_->limits->velocity);
        }
    }

    void UnitreeJointController::effortLimits(double &eff)
    {
        if (joint_urdf_->type == urdf::Joint::REVOLUTE || joint_urdf_->type == urdf::Joint::PRISMATIC) {
            eff = std::clamp(eff, -joint_urdf_->limits->effort, joint_urdf_->limits->effort);
        }
    }

}  // namespace unitree_legged_control

PLUGINLIB_EXPORT_CLASS(unitree_legged_control::UnitreeJointController, controller_interface::ControllerInterface)
