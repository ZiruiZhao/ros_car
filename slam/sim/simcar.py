#!/usr/bin/env python3
"""最小仿真车：一个房间 + 一辆假车，发 /sim_scan 和 odom→base_link TF。

存在的意义：**不动真车、不插雷达**也能验证 slam 的时序问题。
它只做三件事，全部按真实链条的时序来：
  1. /sim_scan：12Hz、339 线（X3 的指纹），**打戳 = 当前时刻 − 95ms** 再发布
     —— 复刻实测的「帧到得比戳晚约 95ms」。
  2. odom→base_link TF：按 tf_hz 发（5Hz ≈ 修复前的桥接 / 50Hz = 修复后），
     用**当前时刻**打戳、位姿取最新一拍 —— 复刻阶梯式 TF。
  3. base_link→laser 静态 TF（真车上来自 URDF）。

参数（命令行 -p 传）：
  tf_hz     TF 频率，5.0 复现问题 / 50.0 验证修复
  lag_ms    扫描戳滞后，实测中位 95
  scan_hz   扫描频率，实测 12.26
房间 4m×3m，车沿回字形慢速走。

用法见同目录 README.md。
"""
import math
import time
import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

ROOM = (0.0, 4.0, 0.0, 3.0)  # xmin xmax ymin ymax
N_RAYS = 339


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class SimCar(Node):
    def __init__(self):
        super().__init__("sim_car")
        self.tf_hz = float(self.declare_parameter("tf_hz", 50.0).value)
        self.lag_ms = float(self.declare_parameter("lag_ms", 95.0).value)
        self.scan_hz = float(self.declare_parameter("scan_hz", 12.0).value)

        q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST, depth=5)
        self.scan_pub = self.create_publisher(LaserScan, "/sim_scan", q)
        self.tfb = TransformBroadcaster(self)
        self.stfb = StaticTransformBroadcaster(self)

        # base_link→laser：真车上来自 URDF（这里用简化的 z 值就够）
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "laser"
        t.transform.translation.z = 0.1075
        self.stfb.sendTransform(t)

        self.t0 = time.time()
        self.pose = (1.0, 1.0, 0.0)      # x, y, yaw（房间内）
        self.latest = self.pose
        self.create_timer(1.0 / 20.0, self.motion_tick)
        self.create_timer(1.0 / max(self.tf_hz, 0.1), self.tf_tick)
        self.create_timer(1.0 / self.scan_hz, self.scan_tick)
        self.get_logger().info(
            f"仿真车启动：房间 {ROOM[1]-ROOM[0]:.0f}x{ROOM[3]-ROOM[2]:.0f}m，"
            f"TF {self.tf_hz:g}Hz，扫描 {self.scan_hz:g}Hz 戳滞后 {self.lag_ms:g}ms")

    # ── 运动：0.9m 方框回字形，约 0.075 m/s ──
    def motion_tick(self):
        t = time.time() - self.t0
        seg = 12.0
        phase = int(t / seg) % 4
        frac = (t % seg) / seg
        corners = [(1.0, 1.0), (1.9, 1.0), (1.9, 1.9), (1.0, 1.9)]
        a = corners[phase]
        b = corners[(phase + 1) % 4]
        x = a[0] + (b[0] - a[0]) * frac
        y = a[1] + (b[1] - a[1]) * frac
        yaw = math.atan2(b[1] - a[1], b[0] - a[0])
        self.pose = (x, y, yaw)

    def tf_tick(self):
        self.latest = self.pose
        x, y, yaw = self.latest
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = x
        t.transform.translation.y = y
        qx, qy, qz, qw = quat_from_yaw(yaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tfb.sendTransform(t)

    # ── 扫描：射线求交四面墙 ──
    def scan_tick(self):
        x, y, yaw = self.pose
        msg = LaserScan()
        now = self.get_clock().now()
        lag = (self.lag_ms + random.uniform(-15, 15)) / 1000.0
        ns = int(now.nanoseconds - lag * 1e9)
        msg.header.stamp.sec = ns // 1_000_000_000
        msg.header.stamp.nanosec = ns % 1_000_000_000
        msg.header.frame_id = "laser"
        msg.angle_min = -math.pi
        msg.angle_max = math.pi
        msg.angle_increment = 2 * math.pi / (N_RAYS - 1)
        msg.scan_time = 1.0 / self.scan_hz
        msg.time_increment = msg.scan_time / N_RAYS
        msg.range_min = 0.1
        msg.range_max = 12.0
        xmin, xmax, ymin, ymax = ROOM
        ranges = []
        for i in range(N_RAYS):
            a = yaw + msg.angle_min + i * msg.angle_increment
            dx, dy = math.cos(a), math.sin(a)
            best = float("inf")
            if dx > 1e-9:
                best = min(best, (xmax - x) / dx)
            elif dx < -1e-9:
                best = min(best, (xmin - x) / dx)
            if dy > 1e-9:
                best = min(best, (ymax - y) / dy)
            elif dy < -1e-9:
                best = min(best, (ymin - y) / dy)
            best += random.gauss(0, 0.01)
            ranges.append(best if msg.range_min < best < msg.range_max else float("inf"))
        msg.ranges = ranges
        msg.intensities = [1.0] * N_RAYS
        self.scan_pub.publish(msg)


def main():
    rclpy.init()
    n = SimCar()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


main()
