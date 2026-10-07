"""启动桥接节点 + imu_link 的静态 TF。

    ros2 launch roscar_bridge bridge.launch.py
    ros2 launch roscar_bridge bridge.launch.py port:=/dev/ttyUSB0

TF 树（对接手册 4.6）：

    map ──(SLAM)──→ odom ──(桥接节点)──→ base_link ──(static)──→ imu_link

`odom → base_link` 由桥接节点发，内容就是 [ODOM] 的 x, y, th。
`map → odom` 现在没有 —— 要等有激光雷达 + SLAM（阶段三）。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("roscar_bridge")
    default_params = os.path.join(pkg_share, "config", "bridge.yaml")

    port = LaunchConfiguration("port")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "port",
                default_value="/dev/roscar",
                description="串口设备。建议用 udev 规则绑的固定名，不要用 /dev/ttyUSB0。",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="参数文件路径",
            ),
            Node(
                package="roscar_bridge",
                executable="bridge_node",
                name="roscar_bridge",
                output="screen",
                parameters=[params_file, {"port": port}],
                # 串口断了会让节点重启 —— 但注意重启会让板子再复位一次
                # （开串口拉 DTR/RTS），要再等 8 秒。节点内部本来就有重连逻辑，
                # 所以这里先关掉，交给它自己处理。
                respawn=False,
            ),
            # imu_link 相对 base_link 是零位：固件已经把 IMU 的轴摆正了
            # （x 朝车头、y 朝左、z 朝上），这里不需要任何旋转（手册 4.3）。
            #
            # 用带 -- 的旗标写法。旧的「六个裸数字 + 两个 frame 名」写法在
            # Humble 里能用但每次启动都打一串 deprecated 警告，而且分不清
            # 哪三个数是平移、哪三个是旋转。
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_link_to_imu_link",
                arguments=[
                    "--x", "0", "--y", "0", "--z", "0",
                    "--roll", "0", "--pitch", "0", "--yaw", "0",
                    "--frame-id", "base_link",
                    "--child-frame-id", "imu_link",
                ],
                output="log",
            ),
        ]
    )
