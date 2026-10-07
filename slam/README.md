# slam：建图 + 定位 + 导航（slam_toolbox + Nav2）

本车（差速底盘、YDLIDAR X3）的建图 / 定位 / 导航栈。launch 全部按路径启动、
**免编译**（本目录不是 ROS 包），改完存盘即生效。

**硬件层不归这里管**：底盘 / 雷达 / 模型由 `roscar_bringup` 起，任何阶段都是
单独一个终端先跑起来：

```bash
ros2 launch roscar_bringup bringup.launch.py
```

本目录的 launch 只起软件栈（slam / 定位 / Nav2 / RViz）。

## 目录

| 路径 | 干嘛的 |
|---|---|
| `launch/` | 全部 launch（按路径启动，参数指向 `config/`） |
| `config/mapper_params_online_async.yaml` | 建图参数（slam_toolbox online_async） |
| `config/mapper_params_localization.yaml` | 定位参数（slam_toolbox localization，替代 AMCL） |
| `config/nav2_params.yaml` | Nav2 参数（MPPI 差速 + costmap + velocity_smoother） |
| `rviz/slam_toolbox.rviz` | 建图 RViz 配置（Fixed Frame = map） |
| `rviz/nav.rviz` | 导航 RViz 配置（含 costmap / plan / goal 工具） |
| `script/` | 小工具（start_nav 起导航 / save_map 存图 / stop_all 清残留） |
| `maps/` | 存图目录（`.pgm/.yaml` + `.posegraph/.data`） |
| `sim/` | 离线测试件（不接硬件的假车 + 体检探针，见 `sim/simcar.py` 头部注释） |

## 阶段一 · 建图

```bash
# 终端 1 · 硬件层（底盘 + 雷达 + 模型）
ros2 launch roscar_bringup bringup.launch.py

# 终端 2 · slam_toolbox 在线建图（参数文件和 use_sim_time 都写在 launch 里，不用带参数）
#   上次卡死过的，先跑 script/stop_all.sh 清干净再起
ros2 launch /home/orangepi/ros2_car/slam/launch/slam_toolbox_online_async.launch.py

# 终端 3 · RViz（有显示器的机器上；看建图画面就是这条）
ros2 launch /home/orangepi/ros2_car/slam/launch/rviz_slam.launch.py

# 终端 4 · 遥控
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.15 -p turn:=1.0
```

慢走一圈，图够了**先存图再收工**（建图还跑着时存；两个服务都要调）：

```bash
# 栅格图（.pgm/.yaml，看图/存档用）
/home/orangepi/ros2_car/slam/script/save_map.sh room
# 序列化位姿图（.posegraph/.data —— 阶段二定位真正加载的就是它）
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/orangepi/ros2_car/slam/maps/room'}"
```

- **启动顺序随便**：曾经的「先起 slam、等 15 秒」说法已实测证伪并撤回。真正的
  卡死根因是 tf2 的锁序缺陷，触发条件是雷达帧到达时查不到能包住其时间戳的
  `odom→base_link` TF —— 桥接节点 50Hz 补发 TF 后触发条件已不存在。
- **建图和定位不能同时跑**：`map→odom` 同一时刻只能有一个发布者。
- 两个存图服务作用不同：`save_map` 出 `.pgm/.yaml`，`serialize_map` 出
  `.posegraph/.data`；**定位加载的是后者**，只存 pgm/yaml 定位起不来。
- ⚠ **局域网别串台**：另一台机器也在同一 ROS 域里起了同样的节点时，会出现重复
  节点 / 两份 map→odom / TF_OLD_DATA 刷屏。用 `ROS_LOCALHOST_ONLY=1` 对照排查。

## 阶段二 · 定位 + 导航（有地图）

前提：阶段一已存出 `.posegraph/.data`，且硬件层已在终端 1 跑着。

```bash
# 终端 2 · 定位 + Nav2 + RViz（脚本只是把地图路径拼好，参数原样透传给 launch）
/home/orangepi/ros2_car/slam/script/start_nav.sh room
# 等价的手敲版：
ros2 launch /home/orangepi/ros2_car/slam/launch/nav_with_map.launch.py \
  map:=/home/orangepi/ros2_car/slam/maps/room

# 车开机位置不在图原点时，直接给初始位姿（m / m / rad）：
/home/orangepi/ros2_car/slam/script/start_nav.sh room map_x:=1.5 map_y:=-0.3 map_yaw:=1.57
# 或在 RViz 里用 "2D Pose Estimate" 点一下
```

- 定位跑的是 `localization_slam_toolbox_node`，替代 AMCL；`/map` 和 `map→odom`
  都由它发，Nav2 侧不启动 AMCL / map_server。
- 给完初始位姿后 Nav2 生命周期链会在几秒内全 active；不给位姿时
  `planner_server` 会卡在等 TF。

## 阶段三 · 无地图导航（边建图边导航）

```bash
# 硬件层照旧另起；这条起 slam + Nav2 + RViz
ros2 launch /home/orangepi/ros2_car/slam/launch/slam_nav.launch.py
# 不要 RViz: 加 use_rviz:=false
```

Nav2 输出 `cmd_vel` → `velocity_smoother`（`cmd_vel_nav`→`cmd_vel`）→ `roscar_bridge`。

## 关键话题

| 话题 | 类型 | 从哪来 |
|---|---|---|
| `/scan` | LaserScan | ydlidar_ros2_driver（12Hz 左右） |
| `/odom` | Odometry | roscar_bridge（5Hz，TF 另有 50Hz 补发） |
| `/map`、`/map_updates` | OccupancyGrid | slam_toolbox |
| `/tf` | TF2 | map→odom 由 slam_toolbox；odom→base_link 由 roscar_bridge |
| `/cmd_vel` | Twist | 遥控 / Nav2（经 velocity_smoother） |
| `/slam_toolbox/graph_visualization` | MarkerArray | 图可视化 |

## 设计取舍

- **帧名没有 base_footprint**：本车 TF 里 `base_link` 就在地面高度（两驱动轮轴心
  中点），参数文件里的 `base_frame` / `robot_base_frame` 一律用 `base_link`。
- **定位不用 AMCL**：用 slam_toolbox 的 localization 模式加载 `.posegraph/.data`，
  建图和定位一套工具，省掉 map_server 那一路。
- **MPPI 差速模型**：`motion_model: DiffDrive`，`vy` 全为 0；`model_dt` 必须等于
  `1/controller_frequency`，写不一致 MPPI configure 阶段直接报错退出。
- **建图 / 定位互斥**：两者都发布 `map→odom`，同一时刻只能起一个。
