#include <string>
#include <gazebo/common/Events.hh>
#include <rclcpp/rclcpp.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <gazebo_ros/node.hpp>

namespace gazebo {

/// @brief 足端接触传感器插件，用于检测机器人足端与地面的接触力
class FootContactPlugin : public SensorPlugin {
public:
  /// @brief 构造函数
  FootContactPlugin() : SensorPlugin() {}
  
  /// @brief 析构函数
  ~FootContactPlugin() {}

  /// @brief 插件加载函数
  /// @param[in] _sensor 传感器指针，必须为ContactSensor类型
  /// @param[in] _sdf SDF配置元素
  void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) {
    // 获取ROS2节点
    ros_node_ = gazebo_ros::Node::Get(_sdf);
    if (!ros_node_) {
      RCLCPP_FATAL(rclcpp::get_logger("FootContactPlugin"), "Failed to get Gazebo ROS2 node!");
      return;
    }

    // 校验传感器类型
    parent_sensor_ = std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);
    if (!parent_sensor_) {
      RCLCPP_ERROR(ros_node_->get_logger(), "Requires ContactSensor (got %s)", _sensor->Type().c_str());
      return;
    }

    // 从SDF读取参考坐标系（默认base_link）
    frame_id_ = "base_link";
    if (_sdf->HasElement("frame_id")) {
      frame_id_ = _sdf->Get<std::string>("frame_id");
      RCLCPP_INFO(ros_node_->get_logger(), "Using custom frame_id: %s", frame_id_.c_str());
    }

    // 构建标准化发布话题
    std::string sensor_name = _sensor->Name();
    std::string topic_name = "/unitree/contact/" + sensor_name + "/force";
    RCLCPP_INFO(ros_node_->get_logger(), "Publishing force data to: %s", topic_name.c_str());
    
    // 创建发布器
    force_pub_ = ros_node_->create_publisher<geometry_msgs::msg::WrenchStamped>(
        topic_name, rclcpp::SensorDataQoS());

    // 绑定传感器更新回调
    update_connection_ = parent_sensor_->ConnectUpdated(
        [this]() {
          OnUpdate();
        });
    
    // 激活传感器并检查状态
    parent_sensor_->SetActive(true);
    if (!parent_sensor_->IsActive()) {
      RCLCPP_WARN(ros_node_->get_logger(), "Sensor %s activation failed!", sensor_name.c_str());
    }
    
    RCLCPP_INFO(ros_node_->get_logger(), "Loaded for sensor: %s | Topic: %s | Frame: %s", 
                 sensor_name.c_str(), topic_name.c_str(), frame_id_.c_str());
  }

private:
  /// @brief 传感器数据更新回调函数
  void OnUpdate() {
    // 每次更新前重置数据
    unsigned int count = 0;
    double Fx = 0.0, Fy = 0.0, Fz = 0.0;

    // 获取接触数据
    msgs::Contacts contacts = parent_sensor_->Contacts();
    count = contacts.contact_size();

    // 遍历接触点累加力
    for (unsigned int i = 0; i < count; ++i) {
      const auto& contact = contacts.contact(i);
      if (contact.position_size() == 0) {
        RCLCPP_WARN(ros_node_->get_logger(), "Contact %d has no position data!", i);
        continue;
      }
      // body_1_wrench为接触点对传感器本体的力
      const auto& wrench = contact.wrench(0).body_1_wrench().force();
      Fx += wrench.x();
      Fy += wrench.y();
      Fz += wrench.z();
    }

    // 填充并发布力数据
    geometry_msgs::msg::WrenchStamped force_msg;
    force_msg.header.stamp = ros_node_->get_clock()->now();
    force_msg.header.frame_id = frame_id_;

    if (count > 0) {
      // 计算平均力
      force_msg.wrench.force.x = Fx / static_cast<double>(count);
      force_msg.wrench.force.y = Fy / static_cast<double>(count);
      force_msg.wrench.force.z = Fz / static_cast<double>(count);
    } else {
      // 无接触时力为0
      force_msg.wrench.force.x = 0.0;
      force_msg.wrench.force.y = 0.0;
      force_msg.wrench.force.z = 0.0;
    }

    force_pub_->publish(force_msg);
  }

  gazebo_ros::Node::SharedPtr ros_node_;  ///< Gazebo ROS2节点
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr force_pub_;  ///< 力数据发布器
  event::ConnectionPtr update_connection_;  ///< 更新回调连接
  sensors::ContactSensorPtr parent_sensor_;  ///< 接触传感器
  std::string frame_id_;  ///< 参考坐标系
};

GZ_REGISTER_SENSOR_PLUGIN(FootContactPlugin)

} // namespace gazebo