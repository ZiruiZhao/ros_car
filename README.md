# ros_car

差速轮式机器人的 ROS 2 (Humble) 工作空间：底盘接入、2D 激光 SLAM、定位与 Nav2 导航，附带网页端三航点巡航控制。

## 硬件

- 上位机：Orange Pi 4 Pro（aarch64）
- 底盘：ESP32-S3 差速底盘（串口协议，见 `ros_car_esp32s3`）
- 激光雷达：YDLIDAR X3（2D，12Hz）

## 软件包

| 包 | 说明 |
|---|---|
| `roscar_bridge` | 串口桥接：odom 上行（5Hz）、cmd_vel 下行（5Hz 短帧 + 看门狗急停）、odom→base_link TF 50Hz 补发 |
| `roscar_bringup` | 硬件层 launch：底盘桥接 + 雷达驱动 + 机器人模型 |
| `xc_urdf` | URDF/xacro 模型与静态 TF |
| `ydlidar_ros2_driver` | YDLIDAR X3 ROS 2 驱动 |

## slam/

launch 文件与参数配置：

- 建图：`slam_toolbox` online_async
- 定位：`slam_toolbox` localization（加载 posegraph，替代 AMCL）
- 导航：Nav2（MPPI 差分控制器 + costmap + velocity_smoother + waypoint_follower）
- 配置：`config/`（mapper / localization / nav2 参数）
- 工具：`script/`（存图、起导航、清残留）

## 使用

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# 硬件层
ros2 launch roscar_bringup bringup.launch.py
# 建图
ros2 launch /home/orangepi/ros2_car/slam/launch/slam_toolbox_online_async.launch.py
# 存图：栅格图
ros2 run nav2_map_server map_saver_cli -f <map_name>
# 存图：位姿图
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph   "{filename: '/home/orangepi/ros2_car/slam/maps/<map_name>'}"
# 有图导航
ros2 launch /home/orangepi/ros2_car/slam/launch/nav_with_map.launch.py map:=<map_name>
```

## web/

网页巡航控制台（Python http.server + nav2_simple_commander）：

- 地图显示（PGM 前端解析渲染）
- 初始位姿标注（点击拖拽设置位置与朝向）
- 三航点巡航（follow_waypoints）、停止
- 全局/局部规划线、机器人实时位姿

```bash
python3 web/bank_web_app.py   # 端口 8000
```
