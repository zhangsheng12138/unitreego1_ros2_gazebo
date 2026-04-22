#!/usr/bin/env python3
"""
Unitree Gazebo仿真启动文件

该文件用于启动Unitree机器人的Gazebo仿真环境，包括：
1. 启动Gazebo仿真器
2. 等待地图加载成功
3. 解析机器人URDF（Xacro）
4. 启动机器人状态发布器
5. 在Gazebo中生成机器人
6. 加载机器人控制器
7. 加载控制器配置
8. 运行env.py并输出传感器观测信息

遵循谷歌Python风格指南
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    RegisterEventHandler,
    ExecuteProcess
)
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    FindExecutable,
    PathJoinSubstitution
)
from launch_ros.actions import Node, SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    """
    动态生成Launch组件的核心函数
    
    Args:
        context: Launch上下文对象，用于解析LaunchConfiguration
        *args: 可变位置参数
        **kwargs: 可变关键字参数
    
    Returns:
        list: 包含所有LaunchAction的列表
    """
    # 解析启动参数
    robot_name = LaunchConfiguration("rname").perform(context)
    world_name = LaunchConfiguration("wname").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    num_robots = int(LaunchConfiguration("num_robots").perform(context))
    
    # 从环境参数文件中加载默认值
    import yaml
    
    # 尝试从源代码目录加载文件
    pkg_unitree_gazebo_src = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'unitree_gazebo')
    yaml_path = os.path.join(pkg_unitree_gazebo_src, 'config', 'go1_env_parameter.yaml')
    
    # 如果源代码目录不存在，再从安装目录加载
    if not os.path.exists(yaml_path):
        pkg_unitree_gazebo = get_package_share_directory('unitree_gazebo')
        yaml_path = os.path.join(pkg_unitree_gazebo, 'config', 'go1_env_parameter.yaml')
    
    # 从环境参数文件中加载有效的机器人模型名称
    valid_robot_names = ["go1", "a1"]
    try:
        with open(yaml_path, 'r') as f:
            env_params = yaml.safe_load(f)
            valid_robot_names = env_params.get('robot', {}).get('valid_names', valid_robot_names)
    except Exception as e:
        print(f"从环境参数文件加载有效机器人模型名称失败: {e}")
    
    # 校验机器人模型合法性：仅支持指定的机器人模型，避免无效模型导致后续错误
    if robot_name not in valid_robot_names:
        raise ValueError(f"Invalid robot name {robot_name}! Only support: {valid_robot_names}")
    
    # 核心路径配置
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    
    # 尝试从源代码目录加载文件
    pkg_unitree_gazebo_src = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'unitree_gazebo')
    world_path = os.path.join(pkg_unitree_gazebo_src, "worlds", f"{world_name}.world")
    
    # 如果源代码目录不存在，再从安装目录加载
    if not os.path.exists(world_path):
        pkg_unitree_gazebo = get_package_share_directory("unitree_gazebo")
        world_path = os.path.join(pkg_unitree_gazebo, "worlds", f"{world_name}.world")
    
    # 校验世界文件存在性
    if not os.path.exists(world_path):
        raise FileNotFoundError(f"World file not found: {world_path}")
    
    # 机器人描述包路径
    pkg_robot_desc = get_package_share_directory(f"{robot_name}_description")

    # 1. 启动Gazebo
    gazebo_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": world_path,
            "debug": LaunchConfiguration("debug"),
            "gui": LaunchConfiguration("gui"),
            "paused": LaunchConfiguration("paused"),
            "use_sim_time": use_sim_time,
            "headless": LaunchConfiguration("headless"),
        }.items(),
    )

    # 2. 检查Gazebo是否成功加载世界文件的脚本
    check_gazebo_ready = Node(
        package="unitree_gazebo",
        executable="check_gazebo_ready",
        name="check_gazebo_ready",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["--timeout", "60"]  # 60秒超时
    )

    # 3. 解析机器人URDF（Xacro）
    robot_description = Command(
        [
            FindExecutable(name="xacro"),
            " ", os.path.join(pkg_robot_desc, "xacro", f"{robot_name}.xacro"),
            " DEBUG:=", LaunchConfiguration("user_debug"),
            " use_sim_time:=", use_sim_time
        ]
    )

    # 4. 机器人状态发布器
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": use_sim_time}
        ],
        remappings=[
            ("/joint_states", f"/{robot_name}_gazebo/joint_states")
        ],
    )

    # 5. 在Gazebo中生成机器人
    spawn_args = [
        "-entity", f"{robot_name}_gazebo",
        "-topic", "robot_description",
        "-z", "0.6"
    ]
    if LaunchConfiguration("paused").perform(context) == "false":
        spawn_args.append("-unpause")  # 仅在非暂停模式下添加-unpause参数
    
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name=f"{robot_name}_spawner",
        output="screen",
        arguments=spawn_args,
    )

    # 从环境参数文件中加载控制器名称
    controller_names = [
        "joint_state_broadcaster",
        "FL_hip_controller", "FL_thigh_controller", "FL_calf_controller",
        "FR_hip_controller", "FR_thigh_controller", "FR_calf_controller",
        "RL_hip_controller", "RL_thigh_controller", "RL_calf_controller",
        "RR_hip_controller", "RR_thigh_controller", "RR_calf_controller"
    ]
    
    try:
        with open(yaml_path, 'r') as f:
            env_params = yaml.safe_load(f)
            controller_names = env_params.get('controller', {}).get('names', controller_names)
    except Exception as e:
        print(f"从环境参数文件加载控制器名称失败: {e}")
    
    # 6. 加载机器人控制器（使用--wait选项等待controller_manager可用）
    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="controller_spawner",
        output="screen",
        arguments=controller_names + [
            "--controller-manager", f"/{robot_name}_gazebo/controller_manager",
            "--wait"
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # 7. 加载控制器配置
    set_ctrl_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("go1_controller"), "launch", "set_ctrl.launch.py")
        ),
        launch_arguments={"rname": robot_name}.items(),
    )

    # 8. 运行env.py并输出传感器观测信息
    env_script = Node(
        package="unitree_gazebo",
        executable="env.py",
        name="go1_env",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"num_robots": num_robots},
            {"wait_for_robot": "true"}  # 等待机器人加载完成
        ],
        arguments=[
            f"--num-robots={num_robots}",
            "--wait-for-robot"
        ]
    )

    return [
        gazebo_launch,
        check_gazebo_ready,
        robot_state_pub,
        spawn_entity,
        controller_spawner,
        set_ctrl_launch,
        env_script
    ]


def generate_launch_description():
    """
    生成LaunchDescription的入口函数
    
    Returns:
        LaunchDescription: 包含所有启动组件的LaunchDescription对象
    """
    # 从环境参数文件中加载默认值
    import yaml
    
    # 尝试从源代码目录加载文件
    pkg_unitree_gazebo_src = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'unitree_gazebo')
    yaml_path = os.path.join(pkg_unitree_gazebo_src, 'config', 'go1_env_parameter.yaml')
    
    # 如果源代码目录不存在，再从安装目录加载
    if not os.path.exists(yaml_path):
        pkg_unitree_gazebo = get_package_share_directory('unitree_gazebo')
        yaml_path = os.path.join(pkg_unitree_gazebo, 'config', 'go1_env_parameter.yaml')
    
    default_params = {}
    try:
        with open(yaml_path, 'r') as f:
            env_params = yaml.safe_load(f)
            default_params = env_params.get('launch', {}).get('default_params', {})
    except Exception as e:
        print(f"从环境参数文件加载参数失败: {e}")
    
    # 声明启动参数
    declared_args = [
        DeclareLaunchArgument(
            "wname", 
            default_value=default_params.get("wname", "earth"), 
            description="Gazebo世界文件名称（不含.world）" 
        ),
        DeclareLaunchArgument(
            "rname", 
            default_value=default_params.get("rname", "go1"), 
            description="机器人模型名称 (go1/a1)" 
        ),
        DeclareLaunchArgument(
            "paused", 
            default_value=default_params.get("paused", "true"), 
            choices=["true", "false"], 
            description="是否暂停启动" 
        ),
        DeclareLaunchArgument(
            "use_sim_time", 
            default_value=default_params.get("use_sim_time", "true"), 
            choices=["true", "false"], 
            description="是否使用仿真时间" 
        ),
        DeclareLaunchArgument(
            "gui", 
            default_value=default_params.get("gui", "true"), 
            choices=["true", "false"], 
            description="是否打开Gazebo GUI" 
        ),
        DeclareLaunchArgument(
            "headless", 
            default_value=default_params.get("headless", "false"), 
            choices=["true", "false"], 
            description="是否启用无头模式" 
        ),
        DeclareLaunchArgument(
            "debug", 
            default_value=default_params.get("debug", "false"), 
            choices=["true", "false"], 
            description="是否启用Gazebo调试模式" 
        ),
        DeclareLaunchArgument(
            "user_debug", 
            default_value=default_params.get("user_debug", "false"), 
            choices=["true", "false"], 
            description="传递给xacro文件的DEBUG参数" 
        ),
        DeclareLaunchArgument(
            "num_robots", 
            default_value=default_params.get("num_robots", "1"), 
            description="生成的机器人数量" 
        ),
    ]

    # 设置Gazebo插件路径
    # 直接从安装目录加载插件
    unitree_gazebo_lib = os.path.join(get_package_share_directory("unitree_gazebo"), "lib")
    
    gazebo_plugin_path = SetEnvironmentVariable(
        name="GAZEBO_PLUGIN_PATH",
        value=[
            unitree_gazebo_lib,
            os.pathsep,
            LaunchConfiguration("GAZEBO_PLUGIN_PATH", default=""),
        ]
    )

    # 全局启用仿真时间
    set_sim_time = SetParameter(name="use_sim_time", value=LaunchConfiguration("use_sim_time"))

    return LaunchDescription(
        declared_args + [gazebo_plugin_path, set_sim_time, OpaqueFunction(function=launch_setup)]
    )

