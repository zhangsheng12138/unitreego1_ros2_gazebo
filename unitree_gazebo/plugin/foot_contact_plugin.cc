/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <string>
#include <gazebo/common/Events.hh>
#include <rclcpp/rclcpp.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <gazebo_ros/node.hpp>

namespace gazebo
{
    class UnitreeFootContactPlugin : public SensorPlugin
    {
        public:
        UnitreeFootContactPlugin() : SensorPlugin(){}
        ~UnitreeFootContactPlugin(){}

        void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
        {
            // 获取ROS2节点（由gazebo_ros提供）
            ros_node_ = gazebo_ros::Node::Get(_sdf);

            // 校验传感器类型
            this->parentSensor = std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);        
            if (!this->parentSensor){
                RCLCPP_ERROR(ros_node_->get_logger(), "UnitreeFootContactPlugin requires a ContactSensor.");
                return;
            }

            this->contact_namespace = "contact/";
            
            // 创建发布器
            std::string topic_name = "/visual/"+_sensor->Name()+"/the_force";
            force_pub_ = ros_node_->create_publisher<geometry_msgs::msg::WrenchStamped>(
                topic_name, 100);

            // 绑定传感器更新回调
            this->update_connection = this->parentSensor->ConnectUpdated(
                std::bind(&UnitreeFootContactPlugin::OnUpdate, this));
            
            // 激活传感器
            this->parentSensor->SetActive(true); 
            
            count = 0;
            Fx = 0;
            Fy = 0;
            Fz = 0;
            RCLCPP_INFO(ros_node_->get_logger(), "Load %s plugin.", _sensor->Name().c_str());
        }

        private:
        void OnUpdate()
        {
            msgs::Contacts contacts;
            contacts = this->parentSensor->Contacts();
            count = contacts.contact_size();

            for (unsigned int i = 0; i < count; ++i){
                if(contacts.contact(i).position_size() != 1){
                    RCLCPP_ERROR(ros_node_->get_logger(), "Contact count isn't correct!!!!");
                }     
                for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j){                 
                    Fx += contacts.contact(i).wrench(0).body_1_wrench().force().x();
                    Fy += contacts.contact(i).wrench(0).body_1_wrench().force().y();
                    Fz += contacts.contact(i).wrench(0).body_1_wrench().force().z();
                }
            }

            // 填充并发布力数据
            geometry_msgs::msg::WrenchStamped force_msg;
            force_msg.header.stamp = ros_node_->get_clock()->now();
            force_msg.header.frame_id = "base_link"; // 根据实际场景调整
            
            if(count != 0){           
                force_msg.wrench.force.x = Fx/double(count);
                force_msg.wrench.force.y = Fy/double(count);
                force_msg.wrench.force.z = Fz/double(count);
                count = 0;
                Fx = 0;
                Fy = 0;
                Fz = 0;
            }
            else{
                force_msg.wrench.force.x = 0;
                force_msg.wrench.force.y = 0;
                force_msg.wrench.force.z = 0;
            }
            this->force_pub_->publish(force_msg);
        }

        private:
            // ROS2节点指针
            gazebo_ros::Node::SharedPtr ros_node_;
            // 发布器
            rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr force_pub_;
            // 传感器更新回调
            event::ConnectionPtr update_connection;
            // 命名空间
            std::string contact_namespace;
            // 接触传感器指针
            sensors::ContactSensorPtr parentSensor;      
            // 计数和力缓存
            int count = 0;
            double Fx=0, Fy=0, Fz=0;
    };

    GZ_REGISTER_SENSOR_PLUGIN(UnitreeFootContactPlugin)
}
