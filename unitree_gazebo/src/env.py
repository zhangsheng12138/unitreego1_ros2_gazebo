#!/usr/bin/env python3
"""
Unitree Go1 仿真环境接口
功能：
1. 获取所有传感器的观测信息
2. 三种控制方式的接口
3. 外力施加的接口
4. 获取机器人各个部件的质心位置和重心方向
5. 检测机器人支撑形状和位置
6. 从YAML文件中读取机器人参数
7. 其他辅助功能
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
import numpy as np
import time
import os
import yaml
from ament_index_python.packages import get_package_share_directory

class Go1Robot:
    """
    单个Go1机器人实例类
    """
    
    def __init__(self, node, robot_id):
        """
        初始化单个机器人实例
        node: ROS2节点
        robot_id: 机器人ID
        """
        self.robot_id = robot_id
        self.node = node
        
        # 传感器数据存储
        self.imu_data = None
        # 深度相机数据（5个相机）
        self.camera_data = {
            'face': None, 'chin': None, 'left': None, 'right': None, 'rearDown': None
        }
        self.depth_data = {
            'face': None, 'chin': None, 'left': None, 'right': None, 'rearDown': None
        }
        self.pointcloud_data = {
            'face': None, 'chin': None, 'left': None, 'right': None, 'rearDown': None
        }
        # 超声波传感器数据（3个传感器）
        self.ultrasound_data = {
            'left': None, 'right': None, 'face': None
        }
        # 足端接触传感器数据
        self.foot_contacts = {'FR': None, 'FL': None, 'RR': None, 'RL': None}
        # 关节状态数据
        self.joint_states_data = None
        
        # 初始化时间
        self.start_time = time.time()
        
        # 从URDF文件中读取机器人参数
        self.link_masses = {}
        self.link_com_relative = {}
        self.foot_positions = {}
        self._load_urdf_parameters()
        
        # 从环境参数文件中读取参数
        self.env_params = {}
        self._load_env_parameters()
        
        # PID控制器参数
        self.pid_gains = self.env_params.get('pid', {}).get('gains', {
            'p': 10.0,  # 比例增益
            'i': 0.0,   # 积分增益
            'd': 1.0    # 微分增益
        })
        
        # 上一次的关节位置和误差
        self.last_joint_positions = [0.0] * 12
        self.joint_error_integral = [0.0] * 12
        
        # 传感器订阅
        self._create_subscribers()
        
        # 控制发布器
        self._create_publishers()
        
        print(f"Go1 机器人 {robot_id} 初始化完成")
    
    def _create_subscribers(self):
        """
        创建传感器订阅器
        """
        # 获取传感器名称和话题名称
        camera_names = self.env_params.get('sensors', {}).get('cameras', {}).get('names', ['face', 'chin', 'left', 'right', 'rearDown'])
        ultrasound_names = self.env_params.get('sensors', {}).get('ultrasound', {}).get('names', ['left', 'right', 'face'])
        foot_names = self.env_params.get('sensors', {}).get('foot_contacts', {}).get('names', ['FR', 'FL', 'RR', 'RL'])
        
        # 获取话题名称
        imu_topic = self.env_params.get('topics', {}).get('imu', 'trunk_imu')
        camera_color_topic = self.env_params.get('topics', {}).get('camera', {}).get('color', 'color/image_raw')
        camera_depth_topic = self.env_params.get('topics', {}).get('camera', {}).get('depth', 'depth/image_raw')
        camera_points_topic = self.env_params.get('topics', {}).get('camera', {}).get('points', 'depth/points')
        ultrasound_topic = self.env_params.get('topics', {}).get('ultrasound', 'data')
        foot_contact_topic = self.env_params.get('topics', {}).get('foot_contact', 'unitree/contact/{foot}_foot_contact/force')
        joint_states_topic = self.env_params.get('topics', {}).get('joint_states', 'joint_states')
        
        # IMU订阅
        self.imu_sub = self.node.create_subscription(
            Imu, f'/robot_{self.robot_id}/{imu_topic}', 
            lambda msg: self.imu_callback(msg), 10)
        
        # 深度相机
        for name in camera_names:
            # 彩色图像订阅
            self.node.create_subscription(
                Image, f'/robot_{self.robot_id}/camera_{name}/{camera_color_topic}', 
                lambda msg, name=name: self.camera_callback(msg, name), 10)
            # 深度图像订阅
            self.node.create_subscription(
                Image, f'/robot_{self.robot_id}/camera_{name}/{camera_depth_topic}', 
                lambda msg, name=name: self.depth_callback(msg, name), 10)
            # 点云订阅
            self.node.create_subscription(
                PointCloud2, f'/robot_{self.robot_id}/camera_{name}/{camera_points_topic}', 
                lambda msg, name=name: self.pointcloud_callback(msg, name), 10)
        
        # 超声波传感器
        for name in ultrasound_names:
            self.node.create_subscription(
                WrenchStamped, f'/robot_{self.robot_id}/ultrasound_{name}/{ultrasound_topic}', 
                lambda msg, name=name: self.ultrasound_callback(msg, name), 10)
        
        # 足端接触传感器
        self.foot_subs = {}
        for foot in foot_names:
            foot_topic = foot_contact_topic.format(foot=foot)
            self.foot_subs[foot] = self.node.create_subscription(
                WrenchStamped, f'/robot_{self.robot_id}/{foot_topic}', 
                lambda msg, foot=foot: self.foot_contact_callback(msg, foot), 10)
        
        # 关节状态
        self.joint_states_sub = self.node.create_subscription(
            Float64MultiArray, f'/robot_{self.robot_id}/{joint_states_topic}', 
            lambda msg: self.joint_states_callback(msg), 10)
    
    def _create_publishers(self):
        """
        创建控制发布器
        """
        # 获取话题名称
        effort_topic = self.env_params.get('topics', {}).get('controller', {}).get('effort', 'go1_effort_controller/commands')
        velocity_topic = self.env_params.get('topics', {}).get('controller', {}).get('velocity', 'cmd_vel')
        force_topic = self.env_params.get('topics', {}).get('force', 'apply_force/trunk')
        
        # 控制发布器
        self.joint_effort_pub = self.node.create_publisher(
            Float64MultiArray, f'/robot_{self.robot_id}/{effort_topic}', 10)
        self.velocity_pub = self.node.create_publisher(
            Twist, f'/robot_{self.robot_id}/{velocity_topic}', 10)
        
        # 外力施加发布器
        self.force_pub = self.node.create_publisher(
            WrenchStamped, f'/robot_{self.robot_id}/{force_topic}', 10)
    
    def _load_urdf_parameters(self):
        """
        从YAML文件中加载机器人参数
        """
        # 获取YAML文件路径
        pkg_go1 = get_package_share_directory('go1_description')
        yaml_path = os.path.join(pkg_go1, 'config', 'go1_urdf_parameter.yaml')
        
        try:
            with open(yaml_path, 'r') as f:
                yaml_content = yaml.safe_load(f)
            
            # 解析YAML文件，提取参数
            self._parse_yaml(yaml_content)
            print(f"机器人 {self.robot_id} 成功从YAML文件加载参数: {yaml_path}")
        except Exception as e:
            print(f"机器人 {self.robot_id} 从YAML文件加载参数失败: {e}")
            # 如果解析失败，使用默认值
            self._set_default_parameters()
    
    def _load_env_parameters(self):
        """
        从环境参数文件中加载参数
        """
        # 获取环境参数文件路径
        pkg_unitree_gazebo = get_package_share_directory('unitree_gazebo')
        yaml_path = os.path.join(pkg_unitree_gazebo, 'config', 'go1_env_parameter.yaml')
        
        try:
            with open(yaml_path, 'r') as f:
                self.env_params = yaml.safe_load(f)
            print(f"机器人 {self.robot_id} 成功从环境参数文件加载参数: {yaml_path}")
        except Exception as e:
            print(f"机器人 {self.robot_id} 从环境参数文件加载参数失败: {e}")
            # 如果解析失败，使用默认值
            self.env_params = {}
            print("使用默认环境参数")
    
    def _parse_yaml(self, yaml_content):
        """
        解析YAML文件内容
        """
        # 提取质量信息
        if 'masses' in yaml_content:
            self.link_masses = yaml_content['masses']
        
        # 提取质心位置信息
        if 'positions' in yaml_content and 'com' in yaml_content['positions']:
            com_data = yaml_content['positions']['com']
            # 将字符串格式的位置值转换为数字列表
            self.link_com_relative = {}
            for link, pos_str in com_data.items():
                try:
                    # 分割字符串并转换为浮点数
                    pos = list(map(float, pos_str.split()))
                    self.link_com_relative[link] = pos
                except Exception as e:
                    print(f"解析质心位置失败 for {link}: {e}")
                    # 如果解析失败，使用默认值
                    self.link_com_relative[link] = [0.0, 0.0, 0.0]
        
        # 提取足端位置信息
        # 计算足端位置（基于关节原点位置）
        if 'positions' in yaml_content and 'joint_origin' in yaml_content['positions']:
            joint_origins = yaml_content['positions']['joint_origin']
            self.foot_positions = {
                'FR': self._calculate_foot_position('FR', joint_origins),
                'FL': self._calculate_foot_position('FL', joint_origins),
                'RR': self._calculate_foot_position('RR', joint_origins),
                'RL': self._calculate_foot_position('RL', joint_origins)
            }
        
        # 保存其他参数
        self.yaml_params = yaml_content
    
    def _calculate_foot_position(self, leg_name, joint_origins):
        """
        根据关节原点位置计算足端位置
        """
        # 从关节原点位置计算足端位置
        # 这里简化处理，实际应该根据关节角度计算
        hip_pos = list(map(float, joint_origins[f'{leg_name}_hip_joint'].split()))
        thigh_pos = list(map(float, joint_origins[f'{leg_name}_thigh_joint'].split()))
        calf_pos = list(map(float, joint_origins[f'{leg_name}_calf_joint'].split()))
        foot_pos = list(map(float, joint_origins[f'{leg_name}_foot_joint'].split()))
        
        # 计算绝对位置
        x = hip_pos[0] + thigh_pos[0] + calf_pos[0] + foot_pos[0]
        y = hip_pos[1] + thigh_pos[1] + calf_pos[1] + foot_pos[1]
        z = hip_pos[2] + thigh_pos[2] + calf_pos[2] + foot_pos[2]
        
        return [x, y, z]
    
    def _parse_urdf(self, urdf_content):
        """
        解析URDF文件内容
        """
        # 简单的URDF解析，提取link的质量和质心位置
        import re
        
        # 提取link信息
        link_pattern = r'<link\s+name="([^"]+)">(.*?)</link>'
        links = re.findall(link_pattern, urdf_content, re.DOTALL)
        
        for link_name, link_content in links:
            # 提取质量
            mass_pattern = r'<mass\s+value="([^"]+)"/>'
            mass_match = re.search(mass_pattern, link_content)
            if mass_match:
                mass = float(mass_match.group(1))
                self.link_masses[link_name] = mass
            
            # 提取质心位置
            inertia_pattern = r'<inertia\s+[^>]*?origin\s+xyz="([^"]+)"[^>]*?>'
            inertia_match = re.search(inertia_pattern, link_content)
            if inertia_match:
                origin = inertia_match.group(1).split()
                if len(origin) == 3:
                    com = [float(origin[0]), float(origin[1]), float(origin[2])]
                    self.link_com_relative[link_name] = com
        
        # 提取足端位置
        # 这里需要根据URDF的实际结构进行调整
        # 简单起见，我们使用默认的足端位置
        self.foot_positions = {
            'FR': [0.1881, -0.04675, -0.4],  # 前右足位置
            'FL': [0.1881, 0.04675, -0.4],   # 前左足位置
            'RR': [-0.1881, -0.04675, -0.4],  # 后右足位置
            'RL': [-0.1881, 0.04675, -0.4]   # 后左足位置
        }
    
    def _set_default_parameters(self):
        """
        设置默认参数
        """
        # 机器人部件质量
        self.link_masses = {
            'trunk': 5.204,  # 躯干质量
            'imu_link': 0.001,  # IMU质量
            'depthCamera_link_left': 0.05,  # 相机质量
            'FR_hip': 0.05,  # 前右髋部质量
            'FR_thigh': 0.15,  # 前右大腿质量
            'FR_calf': 0.1,  # 前右小腿质量
            'FR_foot': 0.02,  # 前右足质量
            'FL_hip': 0.05,  # 前左髋部质量
            'FL_thigh': 0.15,  # 前左大腿质量
            'FL_calf': 0.1,  # 前左小腿质量
            'FL_foot': 0.02,  # 前左足质量
            'RR_hip': 0.05,  # 后右髋部质量
            'RR_thigh': 0.15,  # 后右大腿质量
            'RR_calf': 0.1,  # 后右小腿质量
            'RR_foot': 0.02,  # 后右足质量
            'RL_hip': 0.05,  # 后左髋部质量
            'RL_thigh': 0.15,  # 后左大腿质量
            'RL_calf': 0.1,  # 后左小腿质量
            'RL_foot': 0.02,  # 后左足质量
        }
        
        # 部件相对质心位置（相对于父连杆）
        self.link_com_relative = {
            'trunk': [0.0223, 0.002, -0.0005],  # 躯干质心相对位置
            'imu_link': [0.0, 0.0, 0.0],  # IMU质心相对位置
            'depthCamera_link_left': [0.0, 0.0, 0.0],  # 相机质心相对位置
            'FR_hip': [0.0, 0.0, 0.0],  # 前右髋部质心相对位置
            'FR_thigh': [0.0, 0.0, -0.1],  # 前右大腿质心相对位置
            'FR_calf': [0.0, 0.0, -0.1],  # 前右小腿质心相对位置
            'FR_foot': [0.0, 0.0, 0.0],  # 前右足质心相对位置
            'FL_hip': [0.0, 0.0, 0.0],  # 前左髋部质心相对位置
            'FL_thigh': [0.0, 0.0, -0.1],  # 前左大腿质心相对位置
            'FL_calf': [0.0, 0.0, -0.1],  # 前左小腿质心相对位置
            'FL_foot': [0.0, 0.0, 0.0],  # 前左足质心相对位置
            'RR_hip': [0.0, 0.0, 0.0],  # 后右髋部质心相对位置
            'RR_thigh': [0.0, 0.0, -0.1],  # 后右大腿质心相对位置
            'RR_calf': [0.0, 0.0, -0.1],  # 后右小腿质心相对位置
            'RR_foot': [0.0, 0.0, 0.0],  # 后右足质心相对位置
            'RL_hip': [0.0, 0.0, 0.0],  # 后左髋部质心相对位置
            'RL_thigh': [0.0, 0.0, -0.1],  # 后左大腿质心相对位置
            'RL_calf': [0.0, 0.0, -0.1],  # 后左小腿质心相对位置
            'RL_foot': [0.0, 0.0, 0.0],  # 后左足质心相对位置
        }
        
        # 足端位置（相对于躯干）
        self.foot_positions = {
            'FR': [0.1881, -0.04675, -0.4],  # 前右足位置
            'FL': [0.1881, 0.04675, -0.4],   # 前左足位置
            'RR': [-0.1881, -0.04675, -0.4],  # 后右足位置
            'RL': [-0.1881, 0.04675, -0.4]   # 后左足位置
        }
    
    def imu_callback(self, msg):
        """
        IMU 数据回调
        """
        self.imu_data = msg
    
    def camera_callback(self, msg, camera_name):
        """
        相机数据回调
        """
        self.camera_data[camera_name] = msg
    
    def depth_callback(self, msg, camera_name):
        """
        深度相机数据回调
        """
        self.depth_data[camera_name] = msg
    
    def pointcloud_callback(self, msg, camera_name):
        """
        点云数据回调
        """
        self.pointcloud_data[camera_name] = msg
    
    def ultrasound_callback(self, msg, ultrasound_name):
        """
        超声波传感器数据回调
        """
        self.ultrasound_data[ultrasound_name] = msg
    
    def foot_contact_callback(self, msg, foot):
        """
        足端接触数据回调
        """
        self.foot_contacts[foot] = msg
    
    def joint_states_callback(self, msg):
        """
        关节状态数据回调
        """
        self.joint_states_data = msg
    
    def get_observation(self):
        """
        获取所有传感器的观测信息
        返回：包含所有传感器数据的字典
        """
        # 处理ROS2回调
        rclpy.spin_once(self.node, timeout_sec=0.1)
        
        observation = {
            'imu': self.imu_data,
            'cameras': self.camera_data,
            'depth_cameras': self.depth_data,
            'pointclouds': self.pointcloud_data,
            'ultrasound': self.ultrasound_data,
            'foot_contacts': self.foot_contacts,
            'joint_states': self.joint_states_data,
            'time': time.time() - self.start_time
        }
        
        return observation
    
    def get_link_com(self, link_name):
        """
        获取指定部件的质心位置
        link_name: 部件名称
        返回：质心位置 [x, y, z]
        """
        # 这里需要根据机器人的关节状态计算各个部件的绝对位置
        # 简单起见，这里返回相对位置
        # 实际应用中需要根据关节状态和机器人模型计算绝对位置
        if link_name in self.link_com_relative:
            return self.link_com_relative[link_name]
        else:
            return [0.0, 0.0, 0.0]
    
    def get_robot_com(self):
        """
        获取机器人整体的质心位置
        返回：质心位置 [x, y, z]
        """
        total_mass = sum(self.link_masses.values())
        com_x = 0.0
        com_y = 0.0
        com_z = 0.0
        
        for link_name, mass in self.link_masses.items():
            if link_name in self.link_com_relative:
                com = self.link_com_relative[link_name]
                com_x += mass * com[0]
                com_y += mass * com[1]
                com_z += mass * com[2]
        
        if total_mass > 0:
            com_x /= total_mass
            com_y /= total_mass
            com_z /= total_mass
        
        return [com_x, com_y, com_z]
    
    def get_link_com_direction(self, link_name):
        """
        获取指定部件的重心方向
        link_name: 部件名称
        返回：重心方向 [x, y, z]
        """
        # 重心方向通常指向地心，这里返回重力方向
        return [0.0, 0.0, -1.0]
    
    def get_robot_com_direction(self):
        """
        获取机器人整体的重心方向
        返回：重心方向 [x, y, z]
        """
        # 重心方向通常指向地心，这里返回重力方向
        return [0.0, 0.0, -1.0]
    
    def get_support_polygon(self):
        """
        检测机器人支撑形状和位置
        返回：包含支撑形状信息的字典
        """
        rclpy.spin_once(self.node, timeout_sec=0.1)
        
        # 检测哪些足端与地面接触
        contact_feet = []
        contact_positions = []
        
        for foot, contact in self.foot_contacts.items():
            # 简单判断：如果有接触力数据，则认为与地面接触
            if contact is not None:
                # 检查z方向力是否大于阈值（表示与地面接触）
                if contact.wrench.force.z > 0.1:
                    contact_feet.append(foot)
                    # 获取足端位置
                    if foot in self.foot_positions:
                        contact_positions.append(self.foot_positions[foot])
        
        # 确定支撑形状
        num_contacts = len(contact_feet)
        support_shape = ""
        
        if num_contacts == 4:
            support_shape = "四边形"
        elif num_contacts == 3:
            support_shape = "三角形"
        elif num_contacts == 2:
            support_shape = "线段"
        elif num_contacts == 1:
            support_shape = "点"
        else:
            support_shape = "无支撑"
        
        # 计算支撑形状的中心点
        center_x = 0.0
        center_y = 0.0
        center_z = 0.0
        
        if num_contacts > 0:
            for pos in contact_positions:
                center_x += pos[0]
                center_y += pos[1]
                center_z += pos[2]
            center_x /= num_contacts
            center_y /= num_contacts
            center_z /= num_contacts
        
        # 计算水平投影（z=0平面）
        projection_points = []
        for pos in contact_positions:
            projection_points.append([pos[0], pos[1], 0.0])
        
        # 计算投影形状的中心点
        projection_center_x = 0.0
        projection_center_y = 0.0
        
        if num_contacts > 0:
            for point in projection_points:
                projection_center_x += point[0]
                projection_center_y += point[1]
            projection_center_x /= num_contacts
            projection_center_y /= num_contacts
        
        support_info = {
            'contact_feet': contact_feet,
            'contact_positions': contact_positions,
            'support_shape': support_shape,
            'support_center': [center_x, center_y, center_z],
            'projection_points': projection_points,
            'projection_center': [projection_center_x, projection_center_y, 0.0],
            'projection_shape': support_shape  # 投影形状与支撑形状相同
        }
        
        return support_info
    
    def set_joint_effort(self, efforts):
        """
        关节力矩控制
        efforts: 12个关节的力矩值列表
        """
        if len(efforts) != 12:
            raise ValueError("需要提供12个关节的力矩值")
        
        msg = Float64MultiArray()
        msg.data = efforts
        self.joint_effort_pub.publish(msg)
    
    def set_velocity(self, linear_x, linear_y, angular_z):
        """
        速度控制
        linear_x: 线速度x方向
        linear_y: 线速度y方向
        angular_z: 角速度z方向
        """
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.angular.z = angular_z
        self.velocity_pub.publish(msg)
    
    def apply_force(self, fx, fy, fz):
        """
        施加外力
        fx: x方向力
        fy: y方向力
        fz: z方向力
        """
        msg = WrenchStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'trunk'
        msg.wrench.force.x = fx
        msg.wrench.force.y = fy
        msg.wrench.force.z = fz
        self.force_pub.publish(msg)
    
    def set_position(self, positions):
        """
        位置控制（通过力矩控制实现）
        positions: 12个关节的目标位置列表
        """
        if len(positions) != 12:
            raise ValueError("需要提供12个关节的目标位置")
        
        # 处理ROS2回调，获取当前关节状态
        rclpy.spin_once(self.node, timeout_sec=0.1)
        
        # 获取当前关节位置
        current_positions = [0.0] * 12
        if self.joint_states_data is not None:
            current_positions = self.joint_states_data.data[:12]
        
        # 计算关节力矩
        efforts = [0.0] * 12
        for i in range(12):
            # 确定关节类型（0-3: 髋关节, 4-7: 大腿关节, 8-11: 小腿关节）
            if i < 4:
                joint_type = "hip"
            elif i < 8:
                joint_type = "thigh"
            else:
                joint_type = "calf"
            
            # 获取关节限制
            joint_limits = self.get_joint_limits(joint_type)
            
            # 限制目标位置在关节范围内
            target_pos = positions[i]
            if joint_limits['lower_limit'] is not None:
                target_pos = max(target_pos, joint_limits['lower_limit'])
            if joint_limits['upper_limit'] is not None:
                target_pos = min(target_pos, joint_limits['upper_limit'])
            
            # 计算误差
            error = target_pos - current_positions[i]
            error_dot = error - (current_positions[i] - self.last_joint_positions[i])
            
            # 更新积分误差
            self.joint_error_integral[i] += error
            
            # PID控制计算
            p_term = self.pid_gains['p'] * error
            i_term = self.pid_gains['i'] * self.joint_error_integral[i]
            d_term = self.pid_gains['d'] * error_dot
            
            # 计算力矩
            effort = p_term + i_term + d_term
            
            # 限制力矩在关节范围内
            if joint_limits['effort_limit'] is not None:
                effort = max(min(effort, joint_limits['effort_limit']), -joint_limits['effort_limit'])
            
            efforts[i] = effort
        
        # 更新上一次的关节位置
        self.last_joint_positions = current_positions.copy()
        
        # 发布力矩命令
        self.set_joint_effort(efforts)
    
    def get_yaml_parameter(self, param_path, default=None):
        """
        获取YAML文件中的参数
        param_path: 参数路径，使用点号分隔，例如 "sensors.camera.update_rate"
        default: 默认值
        """
        if not hasattr(self, 'yaml_params'):
            return default
        
        # 解析参数路径
        parts = param_path.split('.')
        value = self.yaml_params
        
        for part in parts:
            if isinstance(value, dict) and part in value:
                value = value[part]
            else:
                return default
        
        return value
    
    def get_joint_limits(self, joint_type):
        """
        获取关节限制
        joint_type: 关节类型，例如 "hip", "thigh", "calf"
        """
        return {
            'effort_limit': self.get_yaml_parameter(f"joints.{joint_type}.effort_limit"),
            'lower_limit': self.get_yaml_parameter(f"joints.{joint_type}.lower_limit"),
            'upper_limit': self.get_yaml_parameter(f"joints.{joint_type}.upper_limit"),
            'velocity_limit': self.get_yaml_parameter(f"joints.{joint_type}.velocity_limit"),
            'damping': self.get_yaml_parameter(f"joints.{joint_type}.damping"),
            'friction': self.get_yaml_parameter(f"joints.{joint_type}.friction")
        }
    
    def get_sensor_config(self, sensor_type):
        """
        获取传感器配置
        sensor_type: 传感器类型，例如 "imu", "camera", "foot_contact"
        """
        return self.get_yaml_parameter(f"sensors.{sensor_type}")
    
    def get_geometry(self, part):
        """
        获取几何尺寸
        part: 部件名称，例如 "trunk", "hip", "thigh"
        """
        return self.get_yaml_parameter(f"geometry.{part}")
    
    def get_position(self, position_type, name=None):
        """
        获取位置配置
        position_type: 位置类型，例如 "joint_origin", "fixed_joint", "com"
        name: 具体名称，例如 "FR_hip_joint"
        """
        if name:
            return self.get_yaml_parameter(f"positions.{position_type}.{name}")
        else:
            return self.get_yaml_parameter(f"positions.{position_type}")
    
    def reset(self):
        """
        重置环境
        """
        # 重置传感器数据
        self.imu_data = None
        self.camera_data = {
            'face': None, 'chin': None, 'left': None, 'right': None, 'rearDown': None
        }
        self.depth_data = {
            'face': None, 'chin': None, 'left': None, 'right': None, 'rearDown': None
        }
        self.pointcloud_data = {
            'face': None, 'chin': None, 'left': None, 'right': None, 'rearDown': None
        }
        self.ultrasound_data = {
            'left': None, 'right': None, 'face': None
        }
        self.joint_states_data = None
        for foot in self.foot_contacts:
            self.foot_contacts[foot] = None
        
        # 重置时间
        self.start_time = time.time()
        
        # 发送零力矩
        self.set_joint_effort([0.0] * 12)
        
        print(f"机器人 {self.robot_id} 环境重置完成")

class Go1Env:
    """
    Go1 机器人仿真环境类（支持多个机器人）
    """
    
    def __init__(self, node, num_robots=1):
        """
        初始化环境
        node: ROS2节点实例
        num_robots: 机器人数量
        """
        # 使用传入的Node实例，不再创建新的Node
        self.node = node
        self.num_robots = num_robots
        
        # 存储多个机器人实例
        self.robots = {}
        for i in range(num_robots):
            robot_id = i + 1  # 机器人ID从1开始
            self.robots[robot_id] = Go1Robot(self.node, robot_id)
        
        print(f"Go1 环境初始化完成，共 {num_robots} 个机器人")
    
    def get_robot(self, robot_id):
        """
        获取指定ID的机器人实例
        robot_id: 机器人ID
        返回：Go1Robot实例
        """
        if robot_id in self.robots:
            return self.robots[robot_id]
        else:
            raise ValueError(f"机器人ID {robot_id} 不存在")
    
    def get_observation(self, robot_id):
        """
        获取指定机器人的传感器观测信息
        robot_id: 机器人ID
        返回：包含所有传感器数据的字典
        """
        robot = self.get_robot(robot_id)
        
        # 检查机器人是否加载完成（关节状态是否可用）
        if robot.joint_states_data is None:
            # 机器人尚未加载完成，返回空的观测信息
            return {
                'imu': None,
                'cameras': {},
                'depth_cameras': {},
                'pointclouds': {},
                'ultrasound': {},
                'foot_contacts': {},
                'joint_states': None,
                'time': 0.0
            }
        
        return robot.get_observation()
    
    def set_joint_effort(self, robot_id, efforts):
        """
        控制指定机器人的关节力矩
        robot_id: 机器人ID
        efforts: 12个关节的力矩值列表
        """
        robot = self.get_robot(robot_id)
        robot.set_joint_effort(efforts)
    
    def set_velocity(self, robot_id, linear_x, linear_y, angular_z):
        """
        控制指定机器人的速度
        robot_id: 机器人ID
        linear_x: 线速度x方向
        linear_y: 线速度y方向
        angular_z: 角速度z方向
        """
        robot = self.get_robot(robot_id)
        robot.set_velocity(linear_x, linear_y, angular_z)
    
    def set_position(self, robot_id, positions):
        """
        控制指定机器人的关节位置
        robot_id: 机器人ID
        positions: 12个关节的目标位置列表
        """
        robot = self.get_robot(robot_id)
        robot.set_position(positions)
    
    def apply_force(self, robot_id, fx, fy, fz):
        """
        对指定机器人施加外力
        robot_id: 机器人ID
        fx: x方向力
        fy: y方向力
        fz: z方向力
        """
        robot = self.get_robot(robot_id)
        robot.apply_force(fx, fy, fz)
    
    def get_link_com(self, robot_id, link_name):
        """
        获取指定机器人的部件质心位置
        robot_id: 机器人ID
        link_name: 部件名称
        返回：质心位置 [x, y, z]
        """
        robot = self.get_robot(robot_id)
        return robot.get_link_com(link_name)
    
    def get_robot_com(self, robot_id):
        """
        获取指定机器人的整体质心位置
        robot_id: 机器人ID
        返回：质心位置 [x, y, z]
        """
        robot = self.get_robot(robot_id)
        return robot.get_robot_com()
    
    def get_link_com_direction(self, robot_id, link_name):
        """
        获取指定机器人的部件重心方向
        robot_id: 机器人ID
        link_name: 部件名称
        返回：重心方向 [x, y, z]
        """
        robot = self.get_robot(robot_id)
        return robot.get_link_com_direction(link_name)
    
    def get_robot_com_direction(self, robot_id):
        """
        获取指定机器人的整体重心方向
        robot_id: 机器人ID
        返回：重心方向 [x, y, z]
        """
        robot = self.get_robot(robot_id)
        return robot.get_robot_com_direction()
    
    def get_support_polygon(self, robot_id):
        """
        检测指定机器人的支撑形状和位置
        robot_id: 机器人ID
        返回：包含支撑形状信息的字典
        """
        robot = self.get_robot(robot_id)
        return robot.get_support_polygon()
    
    def get_yaml_parameter(self, robot_id, param_path, default=None):
        """
        获取指定机器人的YAML参数
        robot_id: 机器人ID
        param_path: 参数路径，使用点号分隔
        default: 默认值
        """
        robot = self.get_robot(robot_id)
        return robot.get_yaml_parameter(param_path, default)
    
    def get_joint_limits(self, robot_id, joint_type):
        """
        获取指定机器人的关节限制
        robot_id: 机器人ID
        joint_type: 关节类型
        """
        robot = self.get_robot(robot_id)
        return robot.get_joint_limits(joint_type)
    
    def get_sensor_config(self, robot_id, sensor_type):
        """
        获取指定机器人的传感器配置
        robot_id: 机器人ID
        sensor_type: 传感器类型
        """
        robot = self.get_robot(robot_id)
        return robot.get_sensor_config(sensor_type)
    
    def get_geometry(self, robot_id, part):
        """
        获取指定机器人的几何尺寸
        robot_id: 机器人ID
        part: 部件名称
        """
        robot = self.get_robot(robot_id)
        return robot.get_geometry(part)
    
    def get_position(self, robot_id, position_type, name=None):
        """
        获取指定机器人的位置配置
        robot_id: 机器人ID
        position_type: 位置类型
        name: 具体名称
        """
        robot = self.get_robot(robot_id)
        return robot.get_position(position_type, name)
    
    def reset(self, robot_id=None):
        """
        重置环境
        robot_id: 机器人ID，如果为None则重置所有机器人
        """
        if robot_id is None:
            # 重置所有机器人
            for robot in self.robots.values():
                robot.reset()
            print("所有机器人环境重置完成")
        else:
            # 重置指定机器人
            robot = self.get_robot(robot_id)
            robot.reset()
    
    def close(self):
        """
        关闭环境
        """
        self.node.destroy_node()
        rclpy.shutdown()
        print("环境关闭完成")

import argparse
import sys

class Go1EnvNode(Node):
    """
    Go1 机器人环境ROS2节点
    """
    
    def __init__(self):
        # 从环境参数文件中加载参数
        pkg_unitree_gazebo = get_package_share_directory('unitree_gazebo')
        yaml_path = os.path.join(pkg_unitree_gazebo, 'config', 'go1_env_parameter.yaml')
        
        env_params = {}
        try:
            with open(yaml_path, 'r') as f:
                env_params = yaml.safe_load(f)
        except Exception as e:
            # 注意：在super().__init__之前，self.get_logger()不可用
            print(f"从环境参数文件加载参数失败: {e}")
        
        # 获取节点名称
        node_name = env_params.get('env', {}).get('node_name', 'go1_env_node')
        
        super().__init__(node_name)
        
        # 移除ROS2的命令行参数
        import rclpy.utilities
        args = rclpy.utilities.remove_ros_args(sys.argv)
        
        # 解析命令行参数
        parser = argparse.ArgumentParser()
        parser.add_argument('--num-robots', type=int, default=int(env_params.get('launch', {}).get('default_params', {}).get('num_robots', '1')), help='Number of robots')
        parser.add_argument('--wait-for-robot', action='store_true', help='Wait for robot to be loaded before receiving sensor data')
        parsed_args = parser.parse_args(args[1:])  # 跳过第一个参数（程序名称）
        
        self.num_robots = parsed_args.num_robots
        self.wait_for_robot = parsed_args.wait_for_robot
        self.robot_loaded = False
        
        self.get_logger().info(f"初始化Go1环境，机器人数量: {self.num_robots}")
        
        # 初始化Go1Env，传递当前节点实例
        self.env = Go1Env(self, num_robots=self.num_robots)
        
        # 等待机器人加载完成
        if self.wait_for_robot:
            self.get_logger().info("等待机器人加载完成...")
            self.wait_for_robot_loaded()
        
        # 获取定时器间隔
        timer_interval = env_params.get('env', {}).get('timer_interval', 1.0)
        
        # 创建定时器，定期打印传感器观测信息
        self.timer = self.create_timer(timer_interval, self.timer_callback)
        self.get_logger().info("Go1环境节点初始化完成")
    
    def wait_for_robot_loaded(self):
        """
        等待机器人加载完成
        """
        import time
        start_time = time.time()
        timeout = 60  # 60秒超时
        
        while time.time() - start_time < timeout:
            # 检查关节状态是否可用
            for robot_id in range(1, self.num_robots + 1):
                robot = self.env.get_robot(robot_id)
                if robot.joint_states_data is not None:
                    self.robot_loaded = True
                    self.get_logger().info("机器人加载完成，开始接收传感器信息")
                    return
            
            self.get_logger().info("等待机器人加载...")
            time.sleep(1.0)
        
        # 超时
        self.get_logger().warn("等待机器人加载超时，开始接收传感器信息")
        self.robot_loaded = True
    
    def timer_callback(self):
        """
        定时器回调函数，定期打印传感器观测信息
        """
        for robot_id in range(1, self.num_robots + 1):
            try:
                # 获取传感器观测
                obs = self.env.get_observation(robot_id)
                
                # 打印传感器观测信息
                self.get_logger().info(f"=== 机器人 {robot_id} 传感器观测信息 ===")
                self.get_logger().info(f"IMU数据: {'可用' if obs['imu'] is not None else '不可用'}")
                
                # 打印相机数据
                cameras = obs['cameras']
                if cameras:
                    for cam_name, cam_data in cameras.items():
                        self.get_logger().info(f"{cam_name}相机: {'可用' if cam_data is not None else '不可用'}")
                
                # 打印深度相机数据
                depth_cameras = obs['depth_cameras']
                if depth_cameras:
                    for cam_name, cam_data in depth_cameras.items():
                        self.get_logger().info(f"{cam_name}深度相机: {'可用' if cam_data is not None else '不可用'}")
                
                # 打印点云数据
                pointclouds = obs['pointclouds']
                if pointclouds:
                    for cam_name, pc_data in pointclouds.items():
                        self.get_logger().info(f"{cam_name}点云: {'可用' if pc_data is not None else '不可用'}")
                
                # 打印超声波传感器数据
                ultrasound = obs['ultrasound']
                if ultrasound:
                    for us_name, us_data in ultrasound.items():
                        self.get_logger().info(f"{us_name}超声波: {'可用' if us_data is not None else '不可用'}")
                
                # 打印足端接触数据
                foot_contacts = obs['foot_contacts']
                if foot_contacts:
                    contact_feet = [foot for foot, contact in foot_contacts.items() if contact is not None and contact.wrench.force.z > 0.1]
                    self.get_logger().info(f"接触足端: {contact_feet}")
                
                # 打印关节状态
                self.get_logger().info(f"关节状态: {'可用' if obs['joint_states'] is not None else '不可用'}")
                
                # 打印支撑形状
                support_info = self.env.get_support_polygon(robot_id)
                self.get_logger().info(f"支撑形状: {support_info['support_shape']}")
                self.get_logger().info(f"支撑中心: {support_info['support_center']}")
                
                # 打印质心位置
                robot_com = self.env.get_robot_com(robot_id)
                self.get_logger().info(f"质心位置: {robot_com}")
                
            except Exception as e:
                self.get_logger().error(f"获取机器人 {robot_id} 观测信息失败: {e}")
    
    def destroy_node(self):
        """
        销毁节点时关闭环境
        """
        self.env.close()
        super().destroy_node()

if __name__ == '__main__':
    # 运行ROS2节点
    rclpy.init(args=None)
    node = Go1EnvNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
