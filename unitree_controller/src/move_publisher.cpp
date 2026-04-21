#include <rclcpp/rclcpp.hpp>
#include <gazebo_msgs/msg/model_state.hpp>
#include <string>
#include <stdio.h>  
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <math.h>
#include <iostream>
#include <chrono>

int main(int argc, char **argv)
{
    enum coord
    {
        WORLD,
        ROBOT
    };
    coord def_frame = coord::WORLD;

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("move_publisher");

    auto move_publisher = node->create_publisher<gazebo_msgs::msg::ModelState>("/gazebo/set_model_state", 1000);

    gazebo_msgs::msg::ModelState model_state_pub;

    // 获取机器人名称参数
    std::string robot_name;
    if (!node->get_parameter("/robot_name", robot_name)) {
        RCLCPP_WARN(node->get_logger(), "Failed to get /robot_name, use default: unitree_go1");
        robot_name = "unitree_go1";
    }
    std::cout << "robot_name: " << robot_name << std::endl;

    model_state_pub.model_name = robot_name + "_gazebo";
    rclcpp::Rate loop_rate(1000); // 1000Hz

    if(def_frame == coord::WORLD)
    {
        model_state_pub.pose.position.x = 0.0;
        model_state_pub.pose.position.y = 0.0;
        model_state_pub.pose.position.z = 0.5;
        
        model_state_pub.pose.orientation.x = 0.0;
        model_state_pub.pose.orientation.y = 0.0;
        model_state_pub.pose.orientation.z = 0.0;
        model_state_pub.pose.orientation.w = 1.0;

        model_state_pub.reference_frame = "world";

        long long time_ms = 0;  //time, ms
        const double period = 5000; //ms
        const double radius = 1.5;    //m
        tf2::Quaternion q;

        while(rclcpp::ok())
        {
            model_state_pub.pose.position.x = radius * sin(2*M_PI*(double)time_ms/period);
            model_state_pub.pose.position.y = radius * cos(2*M_PI*(double)time_ms/period);
            
            // 生成四元数
            q.setRPY(0, 0, -2*M_PI*(double)time_ms/period);
            q.normalize();
            model_state_pub.pose.orientation = tf2::toMsg(q);

            move_publisher->publish(model_state_pub);
            loop_rate.sleep();
            time_ms += 1;
        }
    }
    else if(def_frame == coord::ROBOT)
    {
        model_state_pub.twist.linear.x = 0.02; //0.02: 2cm/sec
        model_state_pub.twist.linear.y = 0.0;
        model_state_pub.twist.linear.z = 0.08;
        
        model_state_pub.twist.angular.x = 0.0;
        model_state_pub.twist.angular.y = 0.0;
        model_state_pub.twist.angular.z = 0.0;

        model_state_pub.reference_frame = "base";

        while(rclcpp::ok())
        {
            move_publisher->publish(model_state_pub);
            loop_rate.sleep();
        }
    }

    rclcpp::shutdown();
    return 0;
}
