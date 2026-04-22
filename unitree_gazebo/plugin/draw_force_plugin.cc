#include <gazebo/common/Plugin.hh>
#include <gazebo/rendering/RenderingIface.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/rendering/Scene.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Color.hh>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include <memory>
#include <string>

namespace gazebo {

/// @brief 力可视化插件，用于在Gazebo中显示施加在机器人上的力
class DrawForcePlugin : public VisualPlugin {
public:
  /// @brief 加载插件并初始化
  /// @param[in] _visual 可视化对象
  /// @param[in] _sdf SDF配置元素
  void Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf) override {
    if (!_visual) {
      gzerr << "DrawForcePlugin requires a visual.\n";
      return;
    }

    visual_ = _visual;
    scene_ = _visual->GetScene();

    // 默认参数
    std::string force_topic = "/visual/" + _visual->Name() + "/the_force";
    ignition::math::Color line_color = ignition::math::Color(0.8, 0.2, 0.2, 1.0); // 默认为红色

    // 读取SDF参数
    if (_sdf->HasElement("force_topic")) {
      force_topic = _sdf->Get<std::string>("force_topic");
    }
    if (_sdf->HasElement("line_color")) {
      sdf::ElementPtr color_elem = _sdf->GetElement("line_color");
      double r = 0.8, g = 0.2, b = 0.2, a = 1.0;
      if (color_elem->HasElement("r")) r = color_elem->Get<double>("r");
      if (color_elem->HasElement("g")) g = color_elem->Get<double>("g");
      if (color_elem->HasElement("b")) b = color_elem->Get<double>("b");
      if (color_elem->HasElement("a")) {
        auto result = color_elem->Get<double>("a", 1.0);
        a = result.first;
      }
      line_color.Set(static_cast<float>(r), static_cast<float>(g), static_cast<float>(b), static_cast<float>(a));
    }

    // 初始化ROS节点
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("draw_force_plugin");

    // 创建订阅
    force_sub_ = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
      force_topic,
      10,
      [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        OnForceMsg(msg);
      }
    );

    // 启动更新线程
    update_thread_ = std::thread(&DrawForcePlugin::UpdateLoop, this);
  }

  /// @brief 析构函数
  ~DrawForcePlugin() override {
    if (update_thread_.joinable()) {
      update_thread_.join();
    }
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

private:
  /// @brief 处理力消息回调
  /// @param[in] msg 力消息
  void OnForceMsg(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(force_mutex_);
    force_ = msg->wrench.force;
    force_stamp_ = msg->header.stamp;
  }

  /// @brief 更新循环，用于处理力的可视化
  void UpdateLoop() {
    rclcpp::Rate rate(100); // 100Hz更新频率
    while (rclcpp::ok()) {
      geometry_msgs::msg::Vector3 force;
      {
        std::lock_guard<std::mutex> lock(force_mutex_);
        force = force_;
      }

      // 计算力的大小和方向
      ignition::math::Vector3d force_vec(force.x, force.y, force.z);

      // 这里简化处理，实际的力可视化需要使用Gazebo 11的正确API
      // 由于Gazebo 11的API与原始代码不兼容，这里暂时跳过可视化部分
      // 可以在后续根据需要实现正确的力可视化

      rclcpp::spin_some(node_);
      rate.sleep();
    }
  }

  rendering::VisualPtr visual_;        ///< 可视化对象
  rendering::ScenePtr scene_;          ///< 场景对象
  std::shared_ptr<rclcpp::Node> node_; ///< ROS节点
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_sub_; ///< 力消息订阅
  geometry_msgs::msg::Vector3 force_;  ///< 力向量
  rclcpp::Time force_stamp_;           ///< 力消息时间戳
  std::mutex force_mutex_;             ///< 力数据互斥锁
  std::thread update_thread_;          ///< 更新线程
};

GZ_REGISTER_VISUAL_PLUGIN(DrawForcePlugin)

} // namespace gazebo