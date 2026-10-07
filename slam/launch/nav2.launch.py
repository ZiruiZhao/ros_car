#!/usr/bin/env python3
"""
nav2.launch.py
==============
Nav2 stack：规划 / 控制 / 行为 / BT，参数全部指向 config/nav2_params.yaml。

启动节点:
    1. controller_server     (MPPI，差速模式)
    2. planner_server        (NavfnPlanner A*)
    3. behavior_server       (spin / backup / wait / assisted_teleop)
    4. bt_navigator          (Behavior Tree)
    5. waypoint_follower     (多点，可选)
    6. velocity_smoother     (cmd_vel_nav -> cmd_vel_smoothed -> 底盘)
    7. lifecycle_manager_navigation

注意:
    * 不含 AMCL、不含 map_server —— /map 和 map→odom TF 都由 slam_toolbox 发布。
    * velocity_smoother 从 cmd_vel_nav 收，输出到 cmd_vel；controller / behavior
      的 cmd_vel 已 remap 到 cmd_vel_nav，最终只有一路 cmd_vel 到底盘。
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SLAM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_PARAMS = os.path.join(SLAM_DIR, 'config', 'nav2_params.yaml')


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file = LaunchConfiguration('params_file', default=DEFAULT_PARAMS)
    autostart = LaunchConfiguration('autostart', default='true')

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use Gazebo /clock if true'),
        DeclareLaunchArgument('params_file', default_value=DEFAULT_PARAMS,
                              description='Nav2 参数文件完整路径'),
        DeclareLaunchArgument('autostart', default_value='true',
                              description='自动 configure+activate 各生命周期节点'),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=[
                ('cmd_vel', 'cmd_vel_nav'),       # 输入: 来自 controller / behavior
                ('cmd_vel_smoothed', 'cmd_vel'),  # 输出: 接到底盘（roscar_bridge 收）
            ],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': lifecycle_nodes,
            }],
        ),
    ])
