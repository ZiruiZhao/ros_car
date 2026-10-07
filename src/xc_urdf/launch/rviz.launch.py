"""只起 rviz2，加载 xc_urdf 的显示配置。

给「香橙派起模型、虚拟机里跑 rviz」这种分工用：

    # 香橙派（只起模型）
    ros2 launch xc_urdf display.launch.py

    # 虚拟机（只起 rviz）
    ros2 launch xc_urdf rviz.launch.py

这里**只起 rviz2**。robot_state_publisher 和 joint_state_publisher 都在香橙派
那边跑，两边都起会重复发同一批话题和 TF。

前置条件：**虚拟机上也要有 xc_urdf 这个包**。rviz 是在本机把
`package://xc_urdf/meshes/*.STL` 解析成文件路径的，香橙派发过来的 urdf 里只有路径
字符串，所以虚拟机上没这个包就画不出车（rviz 会报加载 mesh 失败）。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("xc_urdf")
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz_config",
                default_value=os.path.join(pkg_share, "rviz", "roscar.rviz"),
                description="rviz2 的显示配置路径。",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]
    )
