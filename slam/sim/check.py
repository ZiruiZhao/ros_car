#!/usr/bin/env python3
"""体检探针：TF 查得到吗 + slam 服务活着吗 + 地图还在更新吗。

用法：
  python3 check.py <时长秒> [扫描话题]                  # 默认 /sim_scan（仿真车）
  python3 check.py 3600 /scan                          # 真车：建图时挂在旁边

输出：
  · 每 10 秒一行：扫描数、TF 查不到率、/map 条数与尺寸、地图内容多久没变；
  · can_transform('odom','laser', 帧戳) 失败率 —— 桥接没在 50Hz 补 TF 时约
    40-50%（旧桥接/旧工作区的判据），正常应 ~0%；
  · 每 20 秒给 /slam_toolbox 发一次 describe_parameters —— 卡死（死锁）后它会
    超时，此时 Ctrl+C 也杀不掉，只能 SIGKILL。

⚠ /map 每 3 秒来一条是正常的（map_update_interval=3.0 定时重发），不代表在
  建图；判「有没有在建图」看打印的「地图 Ns 未变化」——车在动却 >20 秒不变，
  才是不建图现场。

⚠ 每行还会打印关键话题的发布者数量（scan/odom/map）：本机正常都是 1。
  出现 2 及以上 = 局域网别的机器在跑同一套栈（串台），先处理它再谈建图。
"""
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rcl_interfaces.srv import DescribeParameters

DURATION = float(sys.argv[1]) if len(sys.argv) > 1 else 300.0
TOPIC = sys.argv[2] if len(sys.argv) > 2 else "/sim_scan"


class Check(Node):
    def __init__(self):
        super().__init__("slam_check")
        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)
        q = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST, depth=20)
        self.create_subscription(LaserScan, TOPIC, self.on_scan, q)
        self.n_scan = 0
        self.n_fail = 0
        # /map 是 latched（Transient Local），起订后立刻会收到最后一帧
        qm = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL,
                        history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(OccupancyGrid, "/map", self.on_map, qm)
        self.n_map = 0
        self.map_hash = None
        self.map_changed_t = 0.0
        self.map_size = "--"
        self.cli = self.create_client(DescribeParameters, "/slam_toolbox/describe_parameters")
        self.pending = None
        self.pending_t0 = 0.0
        self.dead = False

    def on_scan(self, m):
        self.n_scan += 1
        if not self.buf.can_transform("odom", m.header.frame_id, m.header.stamp,
                                      Duration(seconds=0.0)):
            self.n_fail += 1

    def on_map(self, m):
        self.n_map += 1
        now = time.time()
        try:
            b = m.data.tobytes()
        except AttributeError:
            b = bytes(m.data)
        h = hash(b)
        self.map_size = f"{m.info.width}x{m.info.height}"
        if h != self.map_hash:      # 内容真变了才算“在建图”
            self.map_hash = h
            self.map_changed_t = now


def liveness(node, timeout=5.0):
    """给 slam 发 describe_parameters，5 秒不回 = 卡死。"""
    if not node.cli.service_is_ready():
        node.cli.wait_for_service(timeout_sec=1.0)
    if not node.cli.service_is_ready():
        return "服务不可达"
    req = DescribeParameters.Request()
    req.names = ["use_sim_time"]
    fut = node.cli.call_async(req)
    t0 = time.time()
    while not fut.done() and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.05)
    if fut.done():
        return "活着"
    return f"无响应（{timeout:.0f}s 超时）"


def main():
    rclpy.init()
    n = Check()
    t0 = time.time()
    last_report = t0
    last_live = t0 - 15.0        # 第一次存活检查提前到 t+5s（卡住时不用干等 20 秒）
    live_log = []
    print(f"体检开始，话题 {TOPIC}，时长 {DURATION:.0f}s", flush=True)
    try:
        while time.time() - t0 < DURATION:
            rclpy.spin_once(n, timeout_sec=0.05)
            now = time.time()
            if now - last_live >= 20.0:
                r = liveness(n)
                live_log.append(r)
                print(f"  [t+{now-t0:4.0f}s] slam 服务：{r}", flush=True)
                last_live = now
            if now - last_report >= 10.0:
                f = n.n_fail / n.n_scan * 100 if n.n_scan else 0.0
                if n.n_map == 0:
                    mstr = "/map 还没收到（slam 没起？）"
                else:
                    stale = now - n.map_changed_t
                    mstr = f"/map {n.n_map} 条 {n.map_size}，地图 {stale:.0f}s 未变化"
                pubs = {t: n.count_publishers(t) for t in (TOPIC, "/odom", "/map")}
                dup = any(c >= 2 for c in pubs.values())
                pubstr = " ".join(f"{t.rsplit('/', 1)[-1]}={c}" for t, c in pubs.items())
                warn = "⚠ " if dup or (n.n_map and now - n.map_changed_t >= 20.0) else ""
                print(f"  [t+{now-t0:4.0f}s] {warn}扫描 {n.n_scan} 条，TF 查不到 {f:.1f}%；"
                      f"{mstr}；发布者 {'【串台！】' if dup else ''}{pubstr}", flush=True)
                last_report = now
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C / 被 kill：不吐堆栈，照常打总结
        pass

    f = n.n_fail / n.n_scan * 100 if n.n_scan else 0.0
    ok = sum(1 for x in live_log if x == "活着")
    if n.n_map:
        last_change = f"最后一次内容变化在 t+{n.map_changed_t - t0:.0f}s"
    else:
        last_change = "没收到 /map"
    print(f"═══ 总结：扫描 {n.n_scan} 条，TF 查不到 {f:.1f}%；/map {n.n_map} 条 {n.map_size}，"
          f"{last_change}；slam 服务存活 {ok}/{len(live_log)} 次检查 ═══", flush=True)
    try:
        n.destroy_node()
        rclpy.shutdown()
    except Exception:
        # Ctrl+C 时 context 已被 rclpy 的信号处理器关过，重复关闭会抛异常，忽略
        pass


main()
