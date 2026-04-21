import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction
)
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    FindExecutable,
    PathJoinSubstitution
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter
from launch.launch_description_sources import AnyLaunchFileSource

# 主启动函数
def generate_launch_description():
    # ====================== 1. 声明所有参数 ======================
    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument(
        "wname", default_value="earth", description="World name"))
    declared_arguments.append(DeclareLaunchArgument(
        "rname", default_value="go1", description="Robot name"))
    declared_arguments.append(DeclareLaunchArgument(
        "paused", default_value="true", description="Start gazebo paused"))
    declared_arguments.append(DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation time"))
    declared_arguments.append(DeclareLaunchArgument(
        "gui", default_value="true", description="Open gazebo gui"))
    declared_arguments.append(DeclareLaunchArgument(
        "headless", default_value="false", description="Start headless"))
    declared_arguments.append(DeclareLaunchArgument(
        "debug", default_value="false", description="Gazebo debug mode"))
    declared_arguments.append(DeclareLaunchArgument(
        "user_debug", default_value="false", description="User debug mode"))

    # ====================== 2. 配置变量 ======================
    robot_name = LaunchConfiguration("rname")
    world_name = LaunchConfiguration("wname")
    use_sim_time = LaunchConfiguration("use_sim_time")

    pkg_gazebo_ros = FindPackageShare(package="gazebo_ros").find("gazebo_ros")
    pkg_unitree_gazebo = FindPackageShare(package="unitree_gazebo").find("unitree_gazebo")
    pkg_robot_desc = PathJoinSubstitution(
        [FindPackageShare(package=[robot_name, "_description"]), ""]
    )

    world_path = PathJoinSubstitution([pkg_unitree_gazebo, "worlds", world_name, ".world"])

    # ====================== 3. 启动 Gazebo ======================
    gazebo_launch = IncludeLaunchDescription(
        launch_description_source=AnyLaunchFileSource(
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

    # ====================== 4. 机器人 URDF + Xacro 解析 ======================
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ", PathJoinSubstitution([pkg_robot_desc, "xacro", "robot.xacro"]),
            " DEBUG:=", LaunchConfiguration("user_debug")
        ]
    )

    # ====================== 5. 机器人状态发布器 ======================
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description_content},
            {"use_sim_time": use_sim_time}
        ],
        remappings=[
            ("/joint_states", "/go1_gazebo/joint_states")
        ]
    )

    # ====================== 6. 在 Gazebo 中生成机器人 ======================
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="urdf_spawner",
        output="screen",
        arguments=[
            "-entity", "go1_gazebo",
            "-topic", "robot_description",
            "-z", "0.6",
            "-unpause"
        ]
    )

    # ====================== 7. 加载控制器 ======================
    load_controllers = Node(
        package="controller_manager",
        executable="spawner",
        name="controller_spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "FL_hip_controller", "FL_thigh_controller", "FL_calf_controller",
            "FR_hip_controller", "FR_thigh_controller", "FR_calf_controller",
            "RL_hip_controller", "RL_thigh_controller", "RL_calf_controller",
            "RR_hip_controller", "RR_thigh_controller", "RR_calf_controller",
            "--controller-manager", "/controller_manager"
        ]
    )

    # ====================== 8. 加载 set_ctrl.launch ======================
    set_ctrl_launch = IncludeLaunchDescription(
        launch_description_source=AnyLaunchFileSource(
            PathJoinSubstitution([
                FindPackageShare(package="unitree_controller").find("unitree_controller"),
                "launch", "set_ctrl.launch.py"
            ])
        ),
        launch_arguments={"rname": robot_name}.items(),
    )

    # ====================== 9. 组合所有节点 ======================
    return LaunchDescription(declared_arguments + [
        SetParameter(name="use_sim_time", value=use_sim_time),
        gazebo_launch,
        robot_state_publisher,
        spawn_entity,
        load_controllers,
        set_ctrl_launch,
    ])
