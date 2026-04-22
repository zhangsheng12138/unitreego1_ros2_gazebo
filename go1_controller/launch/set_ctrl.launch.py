#!/usr/bin/env python3
"""
控制器配置启动文件

该文件用于启动Go1机器人的控制器节点，包括：
1. 核心伺服控制节点
2. 可选的轨迹发布节点
3. 可选的外力控制节点

遵循谷歌Python风格指南
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import SetParameter


def generate_launch_description():
    """
    生成LaunchDescription的入口函数
    
    Returns:
        LaunchDescription: 包含所有启动组件的LaunchDescription对象
    """
    # 声明机器人名称参数
    robot_name_arg = DeclareLaunchArgument(
        'rname',
        default_value='go1',
        description='机器人名称 (laikago/go1/aliengo 等)'
    )

    # 设置全局参数 robot_name = $(arg rname)
    set_robot_name_param = SetParameter(
        name='robot_name',
        value=LaunchConfiguration('rname')
    )

    # 核心伺服控制节点
    unitree_servo_node = Node(
        package='go1_controller',
        executable='unitree_servo',
        name='unitree_servo',
        respawn=True,
        output='screen'  # 输出日志到控制台
    )

    # 可选：启动轨迹发布节点（按需启用）
    unitree_move_node = Node(
        package='go1_controller',
        executable='unitree_move_kinetic',
        name='unitree_move_kinetic',
        output='screen'
    )

    # 可选：启动外力控制节点（按需启用）
    unitree_force_node = Node(
        package='go1_controller',
        executable='unitree_external_force',
        name='unitree_external_force',
        output='screen'
    )

    return LaunchDescription([
        robot_name_arg,
        set_robot_name_param,
        unitree_servo_node,  # 核心节点：必须启动
        # unitree_move_node,   # 可选：轨迹发布
        # unitree_force_node,  # 可选：外力控制
    ])

