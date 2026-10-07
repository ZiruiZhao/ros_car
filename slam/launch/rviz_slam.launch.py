#!/usr/bin/env python3
"""
rviz_slam.launch.py
====================
建图模式 RViz（配置 rviz/slam_toolbox.rviz，Fixed Frame = map）。

用法:
    ros2 launch /home/orangepi/ros2_car/slam/launch/rviz_slam.launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SLAM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RVIZ_CONFIG = os.path.join(SLAM_DIR, 'rviz', 'slam_toolbox.rviz')


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation/Gazebo clock'),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', RVIZ_CONFIG],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
