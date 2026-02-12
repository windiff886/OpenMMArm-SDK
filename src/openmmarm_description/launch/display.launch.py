#!/usr/bin/env python3
"""
OpenMMARM 模型可视化 Launch 文件

启动 robot_state_publisher、joint_state_publisher_gui 和 RViz2
用于在 RViz 中查看和调试机械臂 URDF 模型。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 包路径
    pkg_share = get_package_share_directory('openmmarm_description')

    # URDF 文件路径
    urdf_file = os.path.join(pkg_share, 'urdf', 'openmmarm.urdf')
    
    # 直接读取 URDF 文件内容
    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()

    # Launch 参数
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='启动 joint_state_publisher_gui'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='使用仿真时间'
    )

    # robot_state_publisher 节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # joint_state_publisher_gui 节点（带滑块）
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # RViz2 节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'config', 'display.rviz')]
    )

    return LaunchDescription([
        gui_arg,
        use_sim_time_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])
