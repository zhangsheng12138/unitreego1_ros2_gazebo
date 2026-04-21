/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#include <ignition/math/Color.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/rendering/DynamicLines.hh>
#include <gazebo/rendering/RenderTypes.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/rendering/Scene.hh>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <gazebo_ros/node.hpp>
#include <functional>

namespace gazebo
{
    class UnitreeDrawForcePlugin : public VisualPlugin
    {
        public:
        UnitreeDrawForcePlugin():line(NULL){}
        ~UnitreeDrawForcePlugin(){
            if (this->line) {
                this->visual->DeleteDynamicLine(this->line);
            }
        }

        void Load(rendering::VisualPtr _parent, sdf::ElementPtr _sdf )
        {
            this->visual = _parent;
            this->visual_namespace = "visual/";

            // 获取ROS2节点
            ros_node_ = gazebo_ros::Node::Get(_sdf);

            // 读取话题名称参数
            if (!_sdf->HasElement("topicName")){
                RCLCPP_INFO(ros_node_->get_logger(), "Force draw plugin missing <topicName>, defaults to /default_force_draw");
                this->topic_name = "/default_force_draw";
            } else{
                this->topic_name = _sdf->Get<std::string>("topicName");
            }

            // 创建动态线
            this->line = this->visual->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);
#if GAZEBO_MAJOR_VERSION >= 10
            this->line->AddPoint(ignition::math::Vector3d(0, 0, 0), ignition::math::Color(0, 1, 0, 1.0));
            this->line->AddPoint(ignition::math::Vector3d(1, 1, 1), ignition::math::Color(0, 1, 0, 1.0));
#else
            this->line->AddPoint(ignition::math::Vector3d(0, 0, 0), common::Color(0, 1, 0, 1.0));
            this->line->AddPoint(ignition::math::Vector3d(1, 1, 1), common::Color(0, 1, 0, 1.0));
#endif
            this->line->setMaterial("Gazebo/Purple");
            this->line->setVisibilityFlags(GZ_VISIBILITY_GUI);
            this->visual->SetVisible(true);

            // 创建订阅器
            std::string sub_topic = this->topic_name + "/" + "the_force";
            force_sub_ = ros_node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
                sub_topic, 30, 
                std::bind(&UnitreeDrawForcePlugin::GetForceCallback, this, std::placeholders::_1));

            // 绑定预渲染回调
            this->update_connection = event::Events::ConnectPreRender(
                std::bind(&UnitreeDrawForcePlugin::OnUpdate, this));

            RCLCPP_INFO(ros_node_->get_logger(), "Load %s Draw Force plugin.", this->topic_name.c_str());
        }

        void OnUpdate()
        {
            if (this->line) {
                this->line->SetPoint(1, ignition::math::Vector3d(Fx, Fy, Fz));
            }
        }

        void GetForceCallback(const geometry_msgs::msg::WrenchStamped & msg)
        {
            // 缩放力值用于可视化
            Fx = msg.wrench.force.x / 20.0;
            Fy = msg.wrench.force.y / 20.0;
            Fz = msg.wrench.force.z / 20.0;
        }

        private:
            // ROS2节点
            gazebo_ros::Node::SharedPtr ros_node_;
            // 订阅器
            rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_sub_;
            // 话题名称
            std::string topic_name;
            // 可视化指针
            rendering::VisualPtr visual;
            // 动态线
            rendering::DynamicLines *line;
            // 命名空间
            std::string visual_namespace;
            // 力缓存
            double Fx=0, Fy=0, Fz=0;
            // 更新回调连接
            event::ConnectionPtr update_connection;
    };

    GZ_REGISTER_VISUAL_PLUGIN(UnitreeDrawForcePlugin)
}
