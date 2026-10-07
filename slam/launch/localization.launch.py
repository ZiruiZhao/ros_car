#!/usr/bin/env python3
"""
localization.launch.py
======================
slam_toolbox 定位模式：加载已建好的序列化地图 (.posegraph/.data) 做定位，
发布 map→odom TF（AMCL 的效果）。

硬件层（底盘 + 雷达 + 模型）不归这里管，另开一个终端起：
    ros2 launch roscar_bringup bringup.launch.py

与 slam_toolbox_online_async.launch.py 互斥 —— 两者都发布 map→odom TF 和 /map，
同一时刻只能有一个 map→odom 发布者。

使用方式:
    ros2 launch /home/orangepi/ros2_car/slam/launch/localization.launch.py \
        map:=/home/orangepi/ros2_car/slam/maps/room

初始位姿:
    默认从地图原点 (0,0,0) 开始。车开机位置不在原点时，两种办法二选一：
      - 命令行给:  map_x:=1.5 map_y:=-0.3 map_yaw:=1.57
      - 启动后在 RViz 里用 "2D Pose Estimate" 工具点一下（发 /initialpose）
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SLAM_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_PARAMS = os.path.join(SLAM_DIR, 'config', 'mapper_params_localization.yaml')
DEFAULT_MAP = os.path.join(SLAM_DIR, 'maps', 'slam_toolbox_map')


def _localization_node(context, *args, **kwargs):
    """在 launch 期把 map_x/map_y/map_yaw 三个标量拼成 map_start_pose 双精度数组。

    不能直接写成 {'map_start_pose': [LaunchConfiguration(...), ...]}: Humble 的
    launch_ros 参数归一化 (_normalize_parameter_array_value) 只接受
    float/int/str/bool/Substitution 作为数组元素, 用 ParameterValue 包元素会抛
    TypeError。这里在 OpaqueFunction 里先求出真正的 Python float 列表再传进去,
    归一化时会走 Sequence 分支得到正确的 double[] 类型。
    """
    map_start_pose = [
        float(context.launch_configurations[key])
        for key in ('map_x', 'map_y', 'map_yaw')
    ]

    return [
        Node(
            package='slam_toolbox',
            executable='localization_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                LaunchConfiguration('slam_params_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time'),
                 'map_file_name': LaunchConfiguration('map'),
                 'map_start_pose': map_start_pose},
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation/Gazebo clock. 真车必须 false'),
        DeclareLaunchArgument(
            'slam_params_file', default_value=DEFAULT_PARAMS,
            description='slam_toolbox 定位参数文件完整路径'),
        DeclareLaunchArgument(
            'map', default_value=DEFAULT_MAP,
            description='序列化地图路径（不带扩展名，指向 .posegraph/.data 的公共前缀）'),
        DeclareLaunchArgument(
            'map_x', default_value='0.0',
            description='初始位姿 x（map 系，覆盖 YAML 里的 map_start_pose）'),
        DeclareLaunchArgument(
            'map_y', default_value='0.0',
            description='初始位姿 y（map 系）'),
        DeclareLaunchArgument(
            'map_yaw', default_value='0.0',
            description='初始位姿 yaw（rad，map 系）'),

        OpaqueFunction(function=_localization_node),
    ])
