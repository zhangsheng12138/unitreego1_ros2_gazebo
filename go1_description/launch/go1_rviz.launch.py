from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # ==========================================================================
    # 【可修改】获取功能包路径
    # 注意：
    #   1. 必须与你的描述包名称完全一致（package.xml 中的 name）
    #   2. 对齐：URDF/xacro 文件所在的包
    # ==========================================================================
    go1_description_share_dir = get_package_share_directory('go1_description')
    
    # ==========================================================================
    # 【可修改】启动参数：DEBUG 模式
    # 用途：传递给 xacro，控制是否加载仿真/传感器/调试插件
    # 对齐接口：robot.xacro 中的 <xacro:arg name="DEBUG" default="false" />
    # 注意：
    #   - xacro 必须存在 DEBUG 参数，否则会报错
    # ==========================================================================
    user_debug_arg = DeclareLaunchArgument(
        'user_debug',
        default_value='false',
        description='Enable debug mode'
    )

    # ==========================================================================
    # 【核心】通过 xacro 生成机器人 URDF 模型
    # 对齐接口：
    #   1. go1_description/xacro/robot.xacro （必须存在）
    #   2. robot_state_publisher
    #   3. RViz2 -> RobotModel 显示
    #   4. Gazebo 仿真模型
    # 可修改：
    #   - robot.xacro 换成你的主 xacro 文件名
    # 注意：
    #   - 路径错误 → 模型加载失败
    # ==========================================================================
    robot_description_content = Command(
        [
            FindExecutable(name='xacro'),
            ' ', os.path.join(go1_description_share_dir, 'xacro', 'robot.xacro'),
            ' DEBUG:=', LaunchConfiguration('user_debug')
        ]
    )
    robot_description_param = ParameterValue(robot_description_content, value_type=str)

    # ==========================================================================
    # 【可修改】关节状态发布器（GUI 调试用）
    # 用途：拖动滑块，让机器人关节运动，测试 URDF 是否正确
    # 对齐接口：
    #   - /joint_states 话题
    #   - robot_state_publisher
    # 注意：
    #   - 正式仿真（ros2_control）时必须注释掉！
    #   - 否则会与控制器冲突，导致关节乱动
    # ==========================================================================
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )

    # ==========================================================================
    # 【核心不可缺】机器人状态发布器
    # 作用：解析 URDF → 发布 TF → 发布机器人姿态
    # 全系统依赖：RViz / Gazebo / 导航 / 控制器 都依赖它
    # 可修改：
    #   - publish_frequency：发布频率（默认 50~1000 都可以）
    # 对齐接口：
    #   - robot_description
    #   - /tf_static
    #   - /joint_states
    # ==========================================================================
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_param,
            'publish_frequency': 1000.0,  # 【可修改】发布频率
        }],
    )

    # ==========================================================================
    # 【可修改】RViz2 可视化
    # 用途：显示机器人模型、TF、传感器、坐标系
    # 可修改：
    #   - check_joint.rviz → 换成你自己的 RViz 配置文件
    # 对齐接口：
    #   - Fixed Frame(base/trunk)
    #   - robot_description
    #   - /tf /joint_states
    # 注意：
    #   - RViz 配置文件中的 Fixed Frame 必须与 URDF 根 link 一致
    # ==========================================================================
    rviz_config_path = os.path.join(go1_description_share_dir, 'launch', 'check_joint.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
    )

    # ==========================================================================
    # 启动节点汇总
    # 注意：
    #   - 正式仿真时，必须删除 joint_state_publisher_node
    #   - 否则与 ros2_control 冲突
    # ==========================================================================
    return LaunchDescription([
        user_debug_arg,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ])
