#!/usr/bin/env python3
"""
Go1机器人Gazebo仿真启动文件

该文件用于启动Go1机器人的Gazebo仿真环境，包括：
1. 启动Gazebo仿真器
2. 处理XACRO文件生成机器人描述
3. 启动机器人状态发布器
4. 启动控制器管理器
5. 加载关节状态广播器
6. 加载关节组力矩控制器
7. 启动RViz2可视化工具

遵循谷歌Python风格指南
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import xacro


def generate_launch_description():
    """
    生成LaunchDescription的入口函数
    
    Returns:
        LaunchDescription: 包含所有启动组件的LaunchDescription对象
    """
    # 声明启动参数
    declared_args = [
        DeclareLaunchArgument(
            'world',
            default_value='empty.world',
            description='Gazebo世界文件名称'
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='go1',
            description='机器人名称'
        ),
    ]
    
    # 获取包路径
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_go1 = get_package_share_directory('go1_description')
    pkg_unitree_gazebo = get_package_share_directory('unitree_gazebo')

    # 加载环境参数
    yaml_path = os.path.join(pkg_unitree_gazebo, 'config', 'go1_env_parameter.yaml')
    env_params = {}
    try:
        with open(yaml_path, 'r') as f:
            env_params = yaml.safe_load(f)
    except Exception as e:
        print(f"加载环境参数失败: {e}")

    # 启动Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items(),
    )

    # 处理xacro文件
    xacro_file = os.path.join(pkg_go1, 'xacro', 'go1.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # 启动robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )

    # 启动controller_manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            os.path.join(pkg_go1, 'config', 'robot_control.yaml')
        ],
        remappings=[
            ('/go1/joint_states', 'joint_states'),
        ],
        output='screen'
    )

    # 加载关节状态广播器
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # 加载关节组力矩控制器
    load_go1_effort_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['go1_effort_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # 启动RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_go1, 'config', 'check_joint.yaml')],
        output='screen'
    )

    return LaunchDescription(declared_args + [
        gazebo,
        robot_state_publisher,
        controller_manager,
        load_joint_state_broadcaster,
        load_go1_effort_controller,
        rviz
    ])