#!/usr/bin/env python3
"""
slam_toolbox_online_async.launch.py
====================================
在线异步建图：只起 slam_toolbox 的 async_slam_toolbox_node，订阅 /scan，
发布 /map (OccupancyGrid) 和 map→odom TF。

硬件层（底盘 + 雷达 + 模型）不归这里管，另开一个终端起：
  ros2 launch roscar_bringup bringup.launch.py

用法（免编译目录，直接按路径启动；参数文件和 use_sim_time 都写死在本文件里）:
  ros2 launch /home/orangepi/ros2_car/slam/launch/slam_toolbox_online_async.launch.py

地图保存（建图还跑着时调，两个服务作用不同，都要）:
  # 序列化位姿图 (.posegraph/.data, 定位模式加载的就是它; filename 是纯字符串):
  ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: '/home/orangepi/ros2_car/slam/maps/room'}"
  # 标准占据栅格 (.pgm/.yaml, 供 map_server / 看图; name 是 std_msgs/String, 要 {data: ...}):
  ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
    "{name: {data: '/home/orangepi/ros2_car/slam/maps/room'}}"
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 本文件在 ~/ros2_car/slam/launch/ 下，向上一级就是 slam/ 目录本身。
SLAM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_PARAMS = os.path.join(SLAM_DIR, 'config', 'mapper_params_online_async.yaml')


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    slam_params_file = LaunchConfiguration('slam_params_file', default=DEFAULT_PARAMS)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation/Gazebo clock. 真车必须 false，否则节点死等 /clock'),
        DeclareLaunchArgument(
            'slam_params_file', default_value=DEFAULT_PARAMS,
            description='slam_toolbox 参数文件完整路径'),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params_file,
                {'use_sim_time': use_sim_time},
            ],
        ),
    ])
