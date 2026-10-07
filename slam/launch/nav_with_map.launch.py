#!/usr/bin/env python3
"""
nav_with_map.launch.py
======================
「有地图导航」：加载已建好的地图做定位（slam_toolbox localization，替代 AMCL）+ Nav2。

只起软件栈，不起硬件 —— 硬件层（底盘 + 雷达 + 模型）请在另一个终端起：
    ros2 launch roscar_bringup bringup.launch.py

本文件启动：
    1. slam_toolbox localization     → 加载已有地图定位
    2. Nav2 stack                    → 规划 / 控制 / BT / 恢复
    3. RViz（可选，默认开）

前提: 已用建图流程保存好序列化地图（.posegraph + .data）。
      默认地图: /home/orangepi/ros2_car/slam/maps/slam_toolbox_map
      覆盖方式: map:=/path/to/map_name（不带扩展名）

初始位姿:
    * 开机位置 ≈ 地图原点 (0,0,0) 时，slam_toolbox 自动在该处初始化；
    * 否则两种办法二选一：
        - 命令行给:  map_x:=1.5 map_y:=-0.3 map_yaw:=1.57
        - RViz 里用 "2D Pose Estimate" 工具点一下当前真实位置。
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

SLAM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH_DIR = os.path.join(SLAM_DIR, 'launch')
DEFAULT_MAP = os.path.join(SLAM_DIR, 'maps', 'slam_toolbox_map')


def _launch_of(filename):
    return os.path.join(LAUNCH_DIR, filename)


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    map_file = LaunchConfiguration('map', default=DEFAULT_MAP)
    map_x = LaunchConfiguration('map_x', default='0.0')
    map_y = LaunchConfiguration('map_y', default='0.0')
    map_yaw = LaunchConfiguration('map_yaw', default='0.0')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use Gazebo /clock if true'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='是否连 RViz 一起起'),
        DeclareLaunchArgument('map', default_value=DEFAULT_MAP,
                              description='序列化地图路径（不带扩展名）'),
        DeclareLaunchArgument('map_x', default_value='0.0',
                              description='初始位姿 x（map 系）'),
        DeclareLaunchArgument('map_y', default_value='0.0',
                              description='初始位姿 y（map 系）'),
        DeclareLaunchArgument('map_yaw', default_value='0.0',
                              description='初始位姿 yaw（rad，map 系）'),

        # 1) 定位层: slam_toolbox localization (替代 AMCL)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch_of('localization.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': map_file,
                'map_x': map_x,
                'map_y': map_y,
                'map_yaw': map_yaw,
            }.items(),
        ),

        # 2) Nav2 stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch_of('nav2.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # 3) RViz（可选）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(_launch_of('rviz_nav.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
            condition=IfCondition(use_rviz),
        ),
    ])
