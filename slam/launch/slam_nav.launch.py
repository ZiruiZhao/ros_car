#!/usr/bin/env python3
"""
slam_nav.launch.py
==================
「无地图导航」：一边建图一边导航，不需要预先保存的地图。

只起软件栈，不起硬件 —— 硬件层（底盘 + 雷达 + 模型）请在另一个终端起：
    ros2 launch roscar_bringup bringup.launch.py

本文件启动：
    1. slam_toolbox online_async      → 实时建图，发 /map + map→odom TF
    2. Nav2 stack                     → 规划 / 控制 / BT / 恢复
    3. RViz（可选，默认开）            → rviz_nav.launch.py

用法:
    ros2 launch /home/orangepi/ros2_car/slam/launch/slam_nav.launch.py
    ros2 launch /home/orangepi/ros2_car/slam/launch/slam_nav.launch.py use_rviz:=false
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

SLAM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH_DIR = os.path.join(SLAM_DIR, 'launch')


def _launch_of(filename):
    return os.path.join(LAUNCH_DIR, filename)


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use Gazebo /clock if true'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='是否连 RViz 一起起'),

        # 1) slam_toolbox 在线建图
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                _launch_of('slam_toolbox_online_async.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
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
