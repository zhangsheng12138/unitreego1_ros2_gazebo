#!/usr/bin/env python3
"""
Go1机器人RViz2可视化启动文件

该文件用于启动Go1机器人的RViz2可视化环境，包括：
1. 检查核心文件是否存在
2. 通过XACRO生成机器人URDF模型
3. 可选：启动关节状态发布器（GUI调试用）
4. 启动机器人状态发布器
5. 启动RViz2可视化工具

遵循谷歌Python风格指南
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory


def check_file_exists(file_path, file_desc):
    """
    检查文件是否存在，不存在则打印错误并退出
    
    Args:
        file_path: 文件路径
        file_desc: 文件描述
    """
    if not os.path.exists(file_path):
        print(f"ERROR: {file_desc} 不存在 → {file_path}", file=sys.stderr)
        sys.exit(1)


def launch_setup(context, *args, **kwargs):
    """
    动态生成Launch组件的核心函数
    
    Args:
        context: Launch上下文对象，用于解析LaunchConfiguration
        *args: 可变位置参数
        **kwargs: 可变关键字参数
    
    Returns:
        list: 包含所有节点的列表
    """
    # 获取启动参数
    # user_debug参数值：传递给xacro的DEBUG参数，控制是否加载调试插件/传感器
    user_debug = LaunchConfiguration('user_debug').perform(context)
    # use_joint_gui参数值：true时启动joint_state_publisher_gui（调试），false时关闭（适配ros2_control）
    use_joint_gui = LaunchConfiguration('use_joint_gui').perform(context) == 'true'
    # use_sim_time参数值：是否使用仿真时间
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context) == 'true'
    
    # 获取功能包路径
    pkg_share = FindPackageShare('go1_description').find('go1_description')
    pkg_unitree_gazebo = get_package_share_directory('unitree_gazebo')
    
    # 加载环境参数
    yaml_path = os.path.join(pkg_unitree_gazebo, 'config', 'go1_env_parameter.yaml')
    env_params = {}
    try:
        with open(yaml_path, 'r') as f:
            env_params = yaml.safe_load(f)
    except Exception as e:
        print(f"加载环境参数失败: {e}")
    
    # 检查核心文件是否存在
    # Xacro模型文件路径
    xacro_path = os.path.join(pkg_share, 'xacro', 'go1.xacro')
    check_file_exists(xacro_path, "机器人Xacro模型文件")
    
    # RViz配置文件路径
    rviz_config_path = os.path.join(pkg_share, 'config', 'check_joint.yaml')
    check_file_exists(rviz_config_path, "RViz配置文件")

    # 通过xacro生成机器人URDF模型
    robot_description_content = Command(
        [
            FindExecutable(name='xacro'),
            ' ', xacro_path,
            ' DEBUG:=', user_debug
        ]
    )
    robot_description_param = ParameterValue(robot_description_content, value_type=str)

    # 节点列表初始化
    nodes = []

    # 可选：关节状态发布器（GUI调试用）
    if use_joint_gui:
        nodes.append(
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='joint_state_publisher_gui',
                output='screen',
                arguments=['--ros-args', '--log-level', 'WARN'],
                parameters=[{'use_sim_time': use_sim_time}]
            )
        )
        nodes.append(LogInfo(msg="启动 joint_state_publisher_gui（调试模式）→ 正式仿真请关闭 use_joint_gui"))
    else:
        nodes.append(LogInfo(msg="未启动 joint_state_publisher_gui → 适配 ros2_control 控制器"))

    # 核心不可缺：机器人状态发布器
    # 作用：解析URDF → 发布TF → 发布机器人姿态
    nodes.append(
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description_param,
                'publish_frequency': env_params.get('robot', {}).get('publish_frequency', 200.0),  # 发布频率：四足机器人建议100-200Hz
                'use_sim_time': use_sim_time,  # 使用仿真时间
            }],
            output='screen',
            arguments=['--ros-args', '--log-level', 'WARN']
        )
    )

    # RViz2可视化
    nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path, '--ros-args', '--log-level', 'WARN'],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]  # 对齐仿真时间
        )
    )

    return nodes


def generate_launch_description():
    """
    生成LaunchDescription的入口函数
    
    Returns:
        LaunchDescription: 包含所有启动组件的LaunchDescription对象
    """
    # 加载环境参数
    pkg_unitree_gazebo = get_package_share_directory('unitree_gazebo')
    yaml_path = os.path.join(pkg_unitree_gazebo, 'config', 'go1_env_parameter.yaml')
    env_params = {}
    try:
        with open(yaml_path, 'r') as f:
            env_params = yaml.safe_load(f)
    except Exception as e:
        print(f"加载环境参数失败: {e}")
    
    # 从环境参数中获取默认值
    launch_params = env_params.get('launch', {}).get('default_params', {})
    
    # 启动参数声明
    args = [
        DeclareLaunchArgument(
            'user_debug',
            default_value=launch_params.get('user_debug', 'false'),
            description='启用xacro调试模式（加载仿真/传感器/调试插件）'
        ),
        DeclareLaunchArgument(
            'use_joint_gui',
            default_value=launch_params.get('use_joint_gui', 'false'),
            description='启动关节调试GUI → 正式仿真必须设为false（避免与ros2_control冲突）'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=launch_params.get('use_sim_time', 'true'),
            description='是否使用仿真时间（Gazebo仿真必须设为true）'
        )
    ]

    # 启动描述汇总
    return LaunchDescription(
        args + [OpaqueFunction(function=launch_setup)]
    )

