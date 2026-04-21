from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import SetParameter

def generate_launch_description():
    # 声明机器人名称参数（默认：laikago）
    robot_name_arg = DeclareLaunchArgument(
        'rname',
        default_value='laikago',
        description='Robot name'
    )

    # 设置全局参数 robot_name = $(arg rname)
    set_robot_name_param = SetParameter(
        name='robot_name',
        value=LaunchConfiguration('rname')
    )

    # 你原来注释掉的节点（保持注释）
    # unitree_servo_node = Node(
    #     package='unitree_controller',
    #     executable='unitree_servo',
    #     name='unitree_servo',
    #     respawn=True,
    # )

    return LaunchDescription([
        robot_name_arg,
        set_robot_name_param,
        # unitree_servo_node,  # 保持注释状态
    ])
