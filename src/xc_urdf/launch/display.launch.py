"""起机器人模型（robot_state_publisher + joint_state_publisher）。

    ros2 launch xc_urdf display.launch.py                    # 香橙派上这么跑
    ros2 launch xc_urdf display.launch.py use_rviz:=true     # 有显示器的机器才加

**默认不起 rviz2** —— 香橙派是机器人上的那台，没接显示器，rviz 在别处跑。
要在同一台机器上连 rviz 一起起，加 `use_rviz:=true`。

rviz 在另一台机器（比如虚拟机）上跑的话，那边用 `rviz.launch.py`，
**不要用这个** —— 两边都起 robot_state_publisher 会重复发同一批话题和 TF：

    ros2 launch xc_urdf rviz.launch.py

这条命令**只起模型**，不发运动指令，车不会动。也没连串口 —— 想同时看里程计和
激光，另外开终端起：

    ros2 launch roscar_bridge bridge.launch.py
    ros2 launch ydlidar_ros2_driver x3_ydlidar_launch.py

一起跑的时候，rviz 的 Fixed Frame 是 `odom`：桥接节点一发 odom→base_link，
车模型就会跟着实车动。

TF 树（完整的样子）：

    odom ──(桥接节点)──→ base_link ─┬─(fixed)──────→ chassis
                                    ├─(continuous)─→ wheel_left / wheel_right
                                    ├─(fixed)──────→ front_wheel_left / front_wheel_right
                                    ├─(fixed)──────→ laser
                                    └─(static, 桥接节点)──→ imu_link
"""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("xc_urdf")
    xacro_file = os.path.join(pkg_share, "urdf", "xc_urdf.urdf.xacro")
    rviz_config = os.path.join(pkg_share, "rviz", "roscar.rviz")

    # 这里就把 xacro 展开成 urdf 字符串发给 robot_state_publisher，
    # 省得再起一个 xacro 进程。
    robot_description = xacro.process_file(xacro_file).toxml()

    use_rviz = LaunchConfiguration("use_rviz")
    use_joint_state_publisher = LaunchConfiguration("use_joint_state_publisher")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description=(
                    "是否在本机一起起 rviz2。默认 false —— 香橙派没接显示器，"
                    "rviz 在虚拟机上用 rviz.launch.py 起。"
                ),
            ),
            DeclareLaunchArgument(
                "use_joint_state_publisher",
                default_value="true",
                description=(
                    "是否发布 /joint_states。两个驱动轮是 continuous 关节，"
                    "没有关节角的话它们在 TF 里就没有位姿，rviz 里画不出来。"
                    "现在发的全是 0（停着的样子）—— 桥接节点不发单轮转速，"
                    "所以轮子本来也不会真转。"
                ),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="log",
                condition=IfCondition(use_joint_state_publisher),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
