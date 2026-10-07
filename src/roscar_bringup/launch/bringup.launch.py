"""香橙派侧的一键启动（阶段三 · 任务6）。

    ros2 launch roscar_bringup bringup.launch.py

一次起三样：

    roscar_bridge        串口桥接   → /odom、/imu/data、/tf，收 /cmd_vel
    xc_urdf              机器人模型 → /robot_description + base_link 底下那棵 TF
    ydlidar_ros2_driver  激光雷达   → /scan

**默认不起 rviz2** —— 香橙派是装在车上的那台，没接显示器。rviz 在虚拟机上单独起：

    ros2 launch xc_urdf rviz.launch.py

真在同一台机器上接了显示器、想连 rviz 一起起，加 `use_rviz:=true`。

调试的时候想单跑某几样，把别的关掉：

    ros2 launch roscar_bringup bringup.launch.py use_lidar:=false
    ros2 launch roscar_bringup bringup.launch.py use_bridge:=false use_lidar:=false
    ros2 launch roscar_bringup bringup.launch.py port:=/dev/ttyUSB0

⚠ 这个包**只在香橙派上跑**。虚拟机那边只起 rviz，别在虚拟机上跑这个 —— 两边都起
robot_state_publisher / 桥接节点会重复发同一批话题和 TF。

这里只是把另外三个包自己的 launch **原样 include 进来**，没有重写任何节点定义 ——
所以单跑某一条 launch 和走这里的效果完全一样，排查问题时可以放心拆开单跑。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _launch_of(package: str, filename: str) -> str:
    """另一个包里那条 launch 的完整路径。

    注意是 `install/` 里的路径（`get_package_share_directory`），不是 `src/` ——
    这样改别人的 launch 之后得重新编译那个包才生效，跟平时一样。
    """
    return os.path.join(get_package_share_directory(package), "launch", filename)


def generate_launch_description() -> LaunchDescription:
    use_bridge = LaunchConfiguration("use_bridge")
    use_model = LaunchConfiguration("use_model")
    use_lidar = LaunchConfiguration("use_lidar")
    use_rviz = LaunchConfiguration("use_rviz")
    port = LaunchConfiguration("port")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_bridge",
                default_value="true",
                description="串口桥接节点（底盘）。车没接板子时关掉。",
            ),
            DeclareLaunchArgument(
                "use_model",
                default_value="true",
                description="机器人模型：robot_state_publisher + joint_state_publisher。",
            ),
            DeclareLaunchArgument(
                "use_lidar",
                default_value="true",
                description="YDLIDAR 驱动。没插雷达时关掉，省得它一直重连。",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="false",
                description=(
                    "是否连 rviz2 一起起。默认 false —— 香橙派没接显示器，"
                    "rviz 在虚拟机上用 xc_urdf 的 rviz.launch.py 起。"
                ),
            ),
            DeclareLaunchArgument(
                "port",
                default_value="/dev/roscar",
                description="桥接节点的串口。用 udev 规则绑的固定名，别写 /dev/ttyUSB0。",
            ),

            # ⚠ 三个 include 是**故意**各包一层 scoped GroupAction 的，别拆。
            # 雷达的 launch 和桥接的 launch 都声明了同名的 `params_file`，
            # launch 参数是「同名最外层赢」——不隔开的话桥接节点会拿到
            # ydlidar_x3.yaml，bridge.yaml 整份被静默忽略（dist_scale 标定值
            # 就白标了，节点还会把 1.0 重写给板子）。scoped=True 让各 include
            # 声明的参数只在各自组内可见，互不串台。
            # 复现/验证：ps 看 bridge_node 的 --params-file 是哪个文件；
            #           ros2 param get /roscar_bridge dist_scale 应是 1.087。
            #
            # 换雷达型号的话，下面第一条 include 的文件名跟着换 ——
            # 三角测距型号是 4ros_ydlidar_launch.py。
            GroupAction(
                scoped=True,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            _launch_of("ydlidar_ros2_driver", "x3_ydlidar_launch.py")
                        ),
                        condition=IfCondition(use_lidar),
                    ),
                ],
            ),
            GroupAction(
                scoped=True,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            _launch_of("roscar_bridge", "bridge.launch.py")
                        ),
                        launch_arguments={"port": port}.items(),
                        condition=IfCondition(use_bridge),
                    ),
                ],
            ),
            GroupAction(
                scoped=True,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            _launch_of("xc_urdf", "display.launch.py")
                        ),
                        launch_arguments={"use_rviz": use_rviz}.items(),
                        condition=IfCondition(use_model),
                    ),
                ],
            ),
        ]
    )
