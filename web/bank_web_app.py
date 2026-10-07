#!/usr/bin/env python3
# 江苏银行营业厅 3航点巡航网页 (单线程rclpy工作队列 + http.server)
import os
os.environ.setdefault("ROS_DOMAIN_ID", "42")  # 网络隔离: 独立ROS2通信域
import json, threading, math, queue
from http.server import BaseHTTPRequestHandler, HTTPServer

import rclpy
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path
from nav2_msgs.action import FollowWaypoints
from nav2_simple_commander.robot_navigator import BasicNavigator

MAP_PGM = "/home/orangepi/ros2_car/slam/maps/room.pgm"
MAP_YAML = "/home/orangepi/ros2_car/slam/maps/room.yaml"
WAYPOINTS = [("休息区", -6.15, 12.5), ("储蓄业务区", -6.15, 2.5), ("信贷业务区", 3.1, 0.4)]

print("1/4 rclpy初始化…", flush=True)
rclpy.init()
print("2/4 创建导航客户端…", flush=True)
nav = BasicNavigator()
tf_buf = Buffer(node=nav)
tf_listener = TransformListener(tf_buf, nav)
plans = {"global": [], "local": []}
cruising = False

def cb_global_plan(m):
    plans["global"] = [[p.pose.position.x, p.pose.position.y] for p in m.poses]

def cb_local_plan(m):
    plans["local"] = [[p.pose.position.x, p.pose.position.y] for p in m.poses]

nav.create_subscription(Path, "/plan", cb_global_plan, 10)
nav.create_subscription(Path, "/local_plan", cb_local_plan, 10)

cmd_q = queue.Queue()
cancel_flag = {"v": False}

def yaw_to_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q

def make_pose(x, y, yaw=0.0):
    p = PoseStamped()
    p.header.frame_id = "map"
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.orientation = yaw_to_quat(float(yaw))
    return p

def do_cruise(poses):
    global cruising
    cruising = True
    cancel_flag["v"] = False
    goal = FollowWaypoints.Goal()
    goal.poses = poses
    f = nav.follow_waypoints_client.send_goal_async(goal)
    while rclpy.ok() and not f.done():
        rclpy.spin_once(nav, timeout_sec=0.1)
    if not f.done():
        cruising = False
        return
    gh = f.result()
    if not gh.accepted:
        print("巡航目标被拒绝", flush=True)
        cruising = False
        return
    rf = gh.get_result_async()
    while rclpy.ok():
        rclpy.spin_once(nav, timeout_sec=0.1)
        if rf.done():
            print("巡航完成", flush=True)
            break
        if cancel_flag["v"]:
            gh.cancel_goal_async()
            print("巡航已取消", flush=True)
            break
    cruising = False

def worker():
    """所有 rclpy 调用都在这一个线程里执行(节点非线程安全)。"""
    while rclpy.ok():
        try:
            cmd, arg = cmd_q.get_nowait()
        except queue.Empty:
            rclpy.spin_once(nav, timeout_sec=0.1)
            continue
        if cmd == "initial_pose":
            nav.setInitialPose(arg)
        elif cmd == "cruise":
            do_cruise(arg)
        elif cmd == "cancel":
            cancel_flag["v"] = True
            nav.cancelTask()

threading.Thread(target=worker, daemon=True).start()
print("3/4 后台ROS线程已启动", flush=True)

def robot_pose():
    try:
        t = tf_buf.lookup_transform("map", "base_link", rclpy.time.Time())
        x, y = t.transform.translation.x, t.transform.translation.y
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return {"x": round(x, 2), "y": round(y, 2), "yaw": round(math.degrees(yaw), 1)}
    except Exception:
        return None

class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def do_GET(self):
        if self.path == "/":
            self._serve(open("/home/orangepi/bank_web/index.html", "rb").read(), "text/html; charset=utf-8")
        elif self.path == "/pose":
            self._serve(json.dumps(robot_pose() or {}).encode(), "application/json")
        elif self.path == "/waypoints":
            self._serve(json.dumps(WAYPOINTS).encode(), "application/json")
        elif self.path == "/plans":
            self._serve(json.dumps(plans).encode(), "application/json")
        elif self.path == "/cruise_status":
            self._serve(json.dumps({"cruising": cruising}).encode(), "application/json")
        elif self.path == "/map.pgm":
            self._serve(open(MAP_PGM, "rb").read(), "image/x-portable-graymap")
        else:
            self.send_error(404)

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        body = json.loads(self.rfile.read(length) or b"{}")
        if self.path == "/initial_pose":
            cmd_q.put(("initial_pose", make_pose(body["x"], body["y"], body.get("yaw", 0.0))))
            self._serve(b'{"ok": true}', "application/json")
        elif self.path == "/cruise":
            poses = [make_pose(w[1], w[2]) for w in WAYPOINTS]
            cmd_q.put(("cruise", poses))
            self._serve('{"ok": true, "msg": "A-B-C 巡航开始"}'.encode("utf-8"), "application/json")
        elif self.path == "/cancel":
            cmd_q.put(("cancel", None))
            self._serve(b'{"ok": true}', "application/json")
        else:
            self.send_error(404)

    def _serve(self, data, ctype):
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

print("4/4 网页服务器启动: http://0.0.0.0:8000", flush=True)
HTTPServer(("0.0.0.0", 8000), Handler).serve_forever()
