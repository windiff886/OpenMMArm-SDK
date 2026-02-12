#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # 配置文件路径
    config_file = PathJoinSubstitution([
        FindPackageShare('openmmarm_hw'),
        'config',
        'openmmarm_hw_config.yaml'
    ])

    # Controller Manager 节点
    # remap ~/robot_description 到 /robot_description，
    # 因为 robot_state_publisher 发布到 /robot_description
    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('~/robot_description', '/robot_description'),
        ],
        output='both',
    )

    # 关节状态广播器
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    # 机械臂轨迹控制器
    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['openmmarm_arm_controller', '--controller-manager', '/controller_manager'],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        controller_manager_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
    ])
