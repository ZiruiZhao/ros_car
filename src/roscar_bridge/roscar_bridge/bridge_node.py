"""ROS2 ↔ ESP32-S3 桥接节点。

对应 `香橙派对接手册.md` 4 节。形状就是那里写的：

    /cmd_vel  ──→ 编码 0x01 帧 ──→ 串口
    串口 ──→ 切行 ──→ 前缀匹配 ──→ 发布 /odom  /imu/data  /tf

还有一条下行通道是给标定用的：参数 `dist_scale` 一变就编 `0x2A` 发给板子
（`ros2 param set /roscar_bridge dist_scale 1.083`）。做成参数而不是服务，
理由写在 `__init__` 里那个参数旁边。

两条线程：
  · **IO 线程**（本文件 `_io_loop`）—— 独占串口 fd，阻塞读、解析、塞队列。
    rclpy 默认是单线程 executor，**阻塞读放在 ROS 回调里会卡死整个节点**。
    这条是分开的，不是优化。
  · **ROS 线程**（executor）—— 三个定时器：5Hz 转发 cmd_vel、50Hz 消费解析
    队列并发布、5s 一次诊断节拍；外加参数回调。

串口 fd 只由 IO 线程碰。ROS 侧要发帧就丢进 `self._tx` 队列 —— 避免两个线程
同时操作同一个 fd。
"""

from __future__ import annotations

import collections
import math
import queue
import threading
import time

import rclpy
import serial
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

from . import protocol as proto

# 板子上电后不响应的时间：3 秒上电延时 + 约 5 秒陀螺零偏标定。
# 开串口会拉 DTR/RTS 让板子复位，所以这段时间必须等 —— 发过去的帧会被直接
# 丢掉，**不报任何错**（手册 0 节）。
#
# ⚠ 手册写「大约 8 秒」，**实测 8.47 秒**（2026-10-02）。卡 8.0 会刚好错过，
# 然后开遥测的帧全被丢，现象是「[DATA] 在刷但 /odom 一个点都不出」。
# 这里给 15 秒是**兜底值不是等待值** —— _wait_ready 看到 [SYS] 就绪会立刻返回，
# 正常情况一秒都不用等。只有板子真起不来时才会耗满它。
DEFAULT_STARTUP_WAIT_S = 15.0

# 板子打印的「就绪」标记。看到这行才算真正可用，比盲等 8 秒准。
READY_MARKER = "[SYS] 就绪"

# 值得往 ROS 日志里转的板子输出。`[CMD]` 是 0x01 的回显（5Hz 转发下每秒
# 5 行），转进来会把日志刷满；`[DATA]` 同理。其余一律默默丢掉。
#
# `[ODOM]` 要转，但**只转非 DATA 的行** —— 启动横幅里的
# 「[ODOM] 里程计就绪：距离刻度 = 1.0000」是回读刻度的最直接手段，
# 而 5Hz 的 `[ODOM] DATA` 会刷屏，单独排除（见 _log_board_line）。
FORWARD_PREFIXES = ("[SYS]", "[ERR]", "[FRM]", "[YAW]", "[ODOM]")
ODOM_DATA_PREFIX = "[ODOM] DATA"

# 距离刻度的合法范围，与固件 `ODOM_DIST_SCALE_MIN/MAX` 一致（`开发流程.md` 参数表）。
# ⚠ 固件对超范围的值是**静默夹掉**，不报错 —— 所以校验必须在发帧之前做，
# 否则现象是「回读出来和你写的不是同一个数」。
DIST_SCALE_MIN = 0.2
DIST_SCALE_MAX = 5.0


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def _cov36(diag: dict[int, float]) -> list[float]:
    """6x6 协方差按行展开成 36 元素。

    给 nav_msgs/Odometry 用（顺序 x, y, z, roll, pitch, yaw）。
    """
    cov = [0.0] * 36
    for i, v in diag.items():
        cov[i] = v
    return cov


def _cov9(diag: dict[int, float]) -> list[float]:
    """3x3 协方差按行展开成 9 元素。

    给 sensor_msgs/Imu 用 —— 它的三个协方差字段是 **9 不是 36**。
    填错了会抛 AssertionError，而异常在定时器回调里会把整个 executor 打死，
    现象是「节点还活着但什么都不发了」，很难定位。
    """
    cov = [0.0] * 9
    for i, v in diag.items():
        cov[i] = v
    return cov


class RosCarBridge(Node):
    def __init__(self) -> None:
        super().__init__("roscar_bridge")

        dp = self.declare_parameter

        # ── 串口 ──
        self.port: str = dp("port", "/dev/roscar").value
        self.baudrate: int = dp("baudrate", 115200).value
        self.startup_wait_s: float = dp("startup_wait_s", DEFAULT_STARTUP_WAIT_S).value
        # 在 open() 之前把 DTR/RTS 置低，试图压住板子复位。
        # 手册 1.2 说了：**能不能压住取决于板子的自动复位电路，得实测**。
        # 压不住也没关系 —— startup_wait_s 那一段照样等。
        self.toggle_dtr: bool = dp("toggle_dtr", False).value

        # ── 话题与坐标系 ──
        self.cmd_vel_topic: str = dp("cmd_vel_topic", "/cmd_vel").value
        self.odom_topic: str = dp("odom_topic", "/odom").value
        self.imu_topic: str = dp("imu_topic", "/imu/data").value
        self.odom_frame: str = dp("odom_frame", "odom").value
        self.base_frame: str = dp("base_frame", "base_link").value
        self.publish_tf: bool = dp("publish_tf", True).value
        self.publish_imu: bool = dp("publish_imu", True).value

        # ── odom→base_link TF 的补发频率 ──
        # 不是「照抄里程计频率」，是给 tf2 铺时间轴的。雷达帧比它自己的
        # 时间戳晚约 95ms 才到 slam（扫描周期 + USB 传输 + 驱动缓冲），
        # 而板子的里程计只有 5Hz ≈ 每 200ms 才有一拍。要是一拍里程计只发一条
        # TF，约 1/3~1/2 的雷达帧到达时找不到一对能包住它时间戳的 TF 样本，
        # tf2 就会把它挂成 pending 等待请求 —— pending 一多，slam 会踩中
        # tf2 MessageFilter/BufferCore 的已知锁序死锁（现象：进程还活着但
        # 服务不响应、地图不再更新，线程栈全部 0% CPU）。用 50Hz 把最新
        # 位姿反复广播（阶梯状 TF），任何时刻到达的旧帧都能一次性查到，
        # pending 不产生，死锁的路就断了。位姿本身还是 10Hz 的，这只是
        # 把时间覆盖铺密，不引入新信息。
        self.tf_rate_hz: float = dp("tf_rate_hz", 50.0).value

        # ── 下行：转发频率与看门狗（手册 4.4）──
        # nav2 以 20Hz 发 cmd_vel，但**不需要** 20Hz 转给板子：车最高才 30cm/s，
        # 5Hz 就是每 6cm 更新一次目标，而底盘的闭环本来就是 50ms 自己在跑。
        # 20Hz 转发还有个副作用：每秒 20 行 [CMD] 回显，日志里什么都看不见。
        self.forward_hz: float = dp("forward_hz", 5.0).value
        self.cmd_timeout_ms: float = dp("cmd_timeout_ms", 500.0).value
        self.duration_ms: int = dp("duration_ms", 500).value
        self.v_full_scale: float = dp("v_full_scale_mm_s", float(proto.V_FULL_SCALE_MM_S)).value
        self.w_full_scale: float = dp("w_full_scale_mrad_s", float(proto.W_FULL_SCALE_MRAD_S)).value

        # ── 启动时开哪些上报（手册 0.2：三路关键遥测默认是关的，开关不落盘）──
        self.enable_odom: bool = dp("enable_odom_stream", True).value
        self.enable_att: bool = dp("enable_att_stream", True).value
        self.enable_imu: bool = dp("enable_imu_stream", True).value
        self.enable_data: bool = dp("enable_data_stream", False).value

        # ── 底盘标定（阶段三 任务4）──
        # 里程计距离刻度，对应固件 `ODOM_DIST_SCALE`（0x2A）。
        # 1.0 = **未标定**，不是「正确的默认值」—— 出厂就是这个数，走 1 米可能报 90cm。
        #
        # 做成参数而不是服务，有两个原因：
        #   ① Humble 的 std_srvs 只有 Empty/SetBool/Trigger，没有能带浮点的；
        #      自定义 srv 要新开一个 ament_cmake 接口包，为这一个数不值当。
        #   ② 参数能写进 bridge.yaml，**每次板子就绪后自动重发** ——
        #      刻度在板上落不落盘就不重要了，不用赌固件的行为。
        #
        # 改它：ros2 param set /roscar_bridge dist_scale 1.083
        self.dist_scale: float = dp("dist_scale", 1.0).value

        # ── 协方差（手册 4.3：不要留全零）──
        # 全零在 ROS 里的意思是「这个测量完全精确」，滤波器会把它当真值，
        # 反而比给个诚实的大数更糟。这板子没标定过协方差，下面都是**量级估计**，
        # 不是标定值 —— 跑起来之后按实际漂移调。
        self.pose_cov_xy: float = dp("pose_cov_xy", 0.0025).value
        self.pose_cov_yaw: float = dp("pose_cov_yaw", 0.01).value
        self.twist_cov_vx: float = dp("twist_cov_vx", 0.01).value
        self.twist_cov_wz: float = dp("twist_cov_wz", 0.01).value
        # 平面机器人：z / roll / pitch 不可观测，按惯例给个大数。
        self.cov_unobservable: float = dp("cov_unobservable", 1e6).value

        self.log_board_lines: bool = dp("log_board_lines", True).value
        self.no_telemetry_warn_s: float = dp("no_telemetry_warn_s", 12.0).value

        # ── 自检：跑不通过就别启动（手册 7 节锚点一）──
        if not proto.CRC_ANCHOR_OK:
            raise RuntimeError(
                "CRC 自检锚失败：crc16_modbus(b'123456789') != 0x4B37。"
                " CRC 实现错了的话帧会被板子全部丢掉，而且不报任何错 —— 到这里停下。"
            )
        if not proto.check_reference_frame():
            raise RuntimeError(
                f"契约锚失败：编出来的样例帧是 {proto.REFERENCE_FRAME.hex(' ').upper()}，"
                f" 期望 {proto.REFERENCE_FRAME_HEX}。协议动了，先对齐手册 1.5 节。"
            )
        if not proto.check_dist_scale_frame():
            raise RuntimeError(
                f"刻度锚失败：1.083 编出来是 {proto.frame_dist_scale(1.083).hex(' ').upper()}，"
                f" 期望 {proto.DIST_SCALE_REFERENCE_HEX}。多半是 ×10000 写成了 ×1000 ——"
                f" 这个错了不会报错，只会「写进去的和回读的对不上」，别跳过。"
            )
        self.get_logger().info("CRC 自检锚 ✓　契约锚 ✓　刻度锚 ✓")

        # ── ROS 接口 ──
        self._odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self._imu_pub = self.create_publisher(Imu, self.imu_topic, 10)
        self._tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        # cmd_vel 订阅用 BEST_EFFORT：这样**同时兼容** reliable 和 best-effort
        # 两种发布端（reliable 发布 → best-effort 订阅是允许的，反过来不行）。
        # 反正我们只取最新值，丢几帧无所谓。
        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._cmd_sub = self.create_subscription(
            Twist, self.cmd_vel_topic, self._on_cmd_vel, cmd_qos
        )

        # ── 内部状态 ──
        self._ser: serial.Serial | None = None
        self._tx: queue.Queue[bytes] = queue.Queue(maxsize=256)
        self._rx: collections.deque = collections.deque(maxlen=512)
        self._rx_lock = threading.Lock()
        self._stop = threading.Event()
        self._seq = 0

        self._latest_twist: Twist | None = None
        self._latest_twist_t = 0.0
        self._estop_sent = False

        self._odom: dict[str, float] | None = None
        # 最新位姿（换算成 SI 之后的 x, y, qx, qy, qz, qw），给 _tf_tick 用。
        self._tf_pose: tuple[float, float, float, float, float, float] | None = None
        self._att: dict[str, float] | None = None
        self._imu: dict[str, float] | None = None
        # 板子自己报的距离刻度（[ODOM] DATA 的 scale=），用来确认 0x2A 真的生效了。
        # None = 还没收到过，和「收到 1.0」是两回事。
        self._board_scale: float | None = None
        # 初始化成「现在」而不是 0.0：否则第一个诊断节拍（5 秒）算出来的
        # 「距上次里程计」是个巨大的数，直接打出「启动 12s 了」的警告 ——
        # 而实际只过了 5 秒。这种时间和事实对不上的日志最误导人。
        self._last_odom_walltime = time.monotonic()
        self._warned_no_odom = False
        # 主循环里再看到 [SYS] 就绪 = 板子中途复位过，要重开一遍上报开关
        self._session_ready = False

        # 注册在 _tx 之后：回调里要往 TX 队列里塞帧，放前面会踩到还没建的属性。
        # 参数文件里给的 dist_scale **不会**走这个回调（那是声明时就生效的），
        # 由 _send_boot_frames 统一发 —— 两条路各管一段，不重叠。
        self.add_on_set_parameters_callback(self._on_set_params)

        # ── 两条线程 ──
        self._io_thread = threading.Thread(target=self._io_loop, name="serial_io", daemon=True)
        self._io_thread.start()

        self._tx_timer = self.create_timer(1.0 / max(self.forward_hz, 0.1), self._tx_tick)
        self._rx_timer = self.create_timer(0.02, self._rx_tick)
        self._tf_timer = self.create_timer(1.0 / max(self.tf_rate_hz, 1.0), self._tf_tick)
        self._diag_timer = self.create_timer(5.0, self._diag_tick)

        self.get_logger().info(
            f"桥接节点启动：{self.port} @ {self.baudrate}，"
            f"转发 {self.forward_hz:g}Hz / TF {self.tf_rate_hz:g}Hz / "
            f"看门狗 {self.cmd_timeout_ms:g}ms，"
            f"等待板子就绪（最多 {self.startup_wait_s:g}s）…"
        )

    # ═══════════════════════════ 串口 IO 线程 ═══════════════════════════

    def _io_loop(self) -> None:
        prev_msg: str | None = None
        n_same = 0
        while not self._stop.is_set():
            msg: str | None = None
            try:
                self._session()
            except (serial.SerialException, OSError) as exc:
                msg = f"串口异常：{exc}　1 秒后重连"
            except Exception as exc:  # noqa: BLE001 — IO 线程死了节点就哑了，兜住
                msg = f"IO 线程异常：{exc!r}　1 秒后重连"
            finally:
                self._close()

            if self._stop.is_set():
                break

            if msg is None:
                prev_msg, n_same = None, 0
            else:
                # 同一条错误连着来（端口被占是最常见的），首条立刻报，
                # 之后每 10 条提一次 —— 每秒一行会把真正有用的那几行冲走，
                # 而第一条本来就把原因说清了。
                n_same = n_same + 1 if msg == prev_msg else 1
                prev_msg = msg
                if n_same == 1:
                    self.get_logger().error(msg)
                elif n_same % 10 == 0:
                    self.get_logger().error(f"（同一条，已重复 {n_same} 次）{msg}")

            self._stop.wait(1.0)

    def _session(self) -> None:
        self._open()
        self._wait_ready()
        self._send_boot_frames()

        buf = bytearray()
        while not self._stop.is_set():
            self._flush_tx()

            # in_waiting 有货就直接取走，没有就等一小会儿 —— timeout=0.02
            # 保证这个循环至少每 20ms 转一圈，TX 队列不至于压太久。
            n = self._ser.in_waiting or 1
            data = self._ser.read(n)
            if not data:
                continue

            buf += data
            while True:
                idx = buf.find(b"\n")
                if idx < 0:
                    break
                raw = bytes(buf[:idx])
                del buf[: idx + 1]
                self._handle_line(raw)

            # 半截行攒太长 = 收发两端波特率不一致或者线在丢字节。
            # 留尾部、丢头部 —— 和固件「缓存满了丢最老的一个字节」同一个道理：
            # 清空会把已经收进来的帧头一起丢掉。
            if len(buf) > 4096:
                self.get_logger().warn(f"半截行攒到 {len(buf)} 字节仍无换行，丢弃头部")
                del buf[:-1024]

    def _open(self) -> None:
        ser = serial.Serial()
        ser.port = self.port
        ser.baudrate = self.baudrate
        ser.bytesize = serial.EIGHTBITS
        ser.parity = serial.PARITY_NONE
        ser.stopbits = serial.STOPBITS_ONE
        # 不设 timeout 的话 read() 永远阻塞，节点没法干净退出（手册 2.5）。
        ser.timeout = 0.02
        # 不设 write_timeout 的话，TX 缓冲满时 write() 会卡死 IO 线程。
        ser.write_timeout = 0.5
        if not self.toggle_dtr:
            # 必须在 open() 之前设，open 之后设就晚了。
            ser.dtr = False
            ser.rts = False
        if hasattr(ser, "exclusive"):
            # 防止两个进程同时开同一个口 —— 轮流读会各自读到半截行，
            # 症状是「解析偶尔失败、但重跑就好」，极难查（手册 2.5）。
            ser.exclusive = True
        try:
            ser.open()
        except serial.SerialException as exc:
            # 被独占锁挡住是个**很容易认错**的故障：上一次的节点没退干净时，
            # 新节点连不上，但旧节点还在发话题 —— 于是 `ros2 topic hz /odom`
            # 照样出数，你以为一切正常，其实看的是僵尸进程的数据。
            if "lock" in str(exc).lower():
                raise serial.SerialException(
                    f"{self.port} 被另一个进程独占着 —— 多半是上一次的 bridge_node "
                    f"没退干净（Ctrl-C 之后 launch 有时会留下子进程）。"
                    f"查是谁占着：fuser -v {self.port}　　然后 kill 掉那些 PID。"
                    f"⚠ 别只看话题出不出数 —— 旧进程还在的话，话题照样是通的。"
                ) from exc
            raise serial.SerialException(f"打不开 {self.port}：{exc}") from exc

        self._ser = ser
        self.get_logger().info(f"串口已打开：{self.port}")

    def _close(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:  # noqa: BLE001
                pass
            self._ser = None

    def _wait_ready(self) -> None:
        """等板子打完启动日志。

        手册 0 节：开串口会让板子复位，复位后 3 秒不响应（接着约 5 秒陀螺零偏
        标定）。这段时间里发过去的帧会被板子直接丢掉，**不报任何错**。

        ⚠ 手册写「大约 8 秒」，**实测是 8.47 秒**（2026-10-02，香橙派 + 板子）。
        所以这里不盲等，盯 `[SYS] 就绪` 那行 —— 看到就走，比任何固定秒数都准。
        正因为会提前返回，`startup_wait_s` 给宽一点是**免费的**：正常情况根本
        用不到它，它只在板子真的起不来时兜底。

        三种情况分开处理：
          ① 看到 `[SYS] 就绪`           → 板子起来了，走
          ② 先看到 `[SYS]` 启动横幅     → 确实复位了，继续等就绪
          ③ 先看到非 `[SYS]` 的行       → 板子本来就在跑（DTR 压住了复位），
                                        不必等满，直接走
        """
        deadline = time.monotonic() + self.startup_wait_s
        buf = bytearray()
        booting = False

        while time.monotonic() < deadline and not self._stop.is_set():
            n = self._ser.in_waiting or 1
            data = self._ser.read(n)
            if not data:
                continue
            buf += data
            while True:
                idx = buf.find(b"\n")
                if idx < 0:
                    break
                raw = bytes(buf[:idx])
                del buf[: idx + 1]
                line = raw.decode("utf-8", "replace").strip()
                if not line:
                    continue
                self._log_board_line(line)

                if line.startswith(READY_MARKER):
                    self.get_logger().info("板子已就绪（看到 [SYS] 就绪）")
                    self._session_ready = True
                    return

                if line.startswith("[SYS]"):
                    booting = True  # 在打启动横幅 → 确实复位了，继续等
                elif not booting:
                    self.get_logger().info(
                        f"板子已在运行（收到「{line[:48]}」），跳过启动等待"
                    )
                    self._session_ready = True
                    return

        if self._stop.is_set():
            return
        self.get_logger().warn(
            f"等了 {self.startup_wait_s:g}s 没看到「{READY_MARKER}」。"
            f"链路可能没问题（板子在跑、只是遥测关着），也可能是板子没烧固件/线不对。"
            f"继续开遥测 —— 如果板子其实还在启动，这几个开关帧会被丢掉，"
            f"现象就是「[DATA] 在刷但 /odom 一个点都不出」。"
        )

    def _send_boot_frames(self) -> None:
        """板子每次就绪后要重发的那批帧。

        两件事，都是「板上不落盘、必须由上位机补」的：

        ① **开遥测**。手册 0.2：这是最容易漏的一步 —— 漏了的现象是「链路明明
           通着（[DATA] 在刷），但 /odom 一个点都不出」。
        ② **写距离刻度**。不依赖板子是否保存它，重发一遍最省心。

        板子中途复位（掉电、看门狗、拔插）也会走到这里，见 `_handle_line`。
        """
        plan = [
            (proto.FC_ODOM_STREAM, self.enable_odom, "里程计上报"),
            (proto.FC_ATT_STREAM, self.enable_att, "姿态上报"),
            (proto.FC_IMU_STREAM, self.enable_imu, "IMU 上报"),
            # [DATA] 是调参用的，导航跑起来之后建议关掉省带宽（手册 1.4）。
            (proto.FC_DATA_STREAM, self.enable_data, "底盘 [DATA] 上报"),
        ]
        for func, on, label in plan:
            self._write_now(proto.frame_stream(func, on, self._next_seq()))
            self.get_logger().info(f"  → {label}：{'开' if on else '关'}")

        # 距离刻度（0x2A，×10000）。**每次就绪都重发** —— 真值存在本节点的
        # dist_scale 参数里（bridge.yaml），板子复位多少次标定都不会丢。
        #
        # ⚠ 顺序：板子启动横幅里那句「[ODOM] 里程计就绪：距离刻度 = x.xxxx」
        # 打的是**它自己存的旧值**，在我们这条之前。所以「横幅说 1.0000、
        # 这里说 1.0830」不是矛盾，以这里为准 —— 而横幅那行反过来是
        # 「板子到底落没落盘」的唯一证据（重起一次看横幅变没变）。
        self._write_now(proto.frame_dist_scale(self.dist_scale, self._next_seq()))
        self.get_logger().info(f"  → 距离刻度：{self.dist_scale:.4f}")

        self.get_logger().info("启动帧已发出（开关量是「置位」不是「切换」，重复发无害）")

    def _next_seq(self) -> int:
        self._seq = (self._seq + 1) & 0xFF
        return self._seq

    def _flush_tx(self) -> None:
        while True:
            try:
                frame = self._tx.get_nowait()
            except queue.Empty:
                return
            self._write_now(frame)

    def _write_now(self, frame: bytes) -> None:
        if self._ser is None:
            return
        self._ser.write(frame)
        # 不 flush：115200 下一帧十几字节，内核 tty 缓冲会自己发走；
        # flush 会等 TX 排空，反而把 IO 线程拖住。

    def _handle_line(self, raw: bytes) -> None:
        line = raw.decode("utf-8", "replace").strip()
        if not line:
            return

        # 启动流程走完之后又看到「就绪」= 板子中途复位了（掉电、看门狗、手动
        # 拔插）。上报开关**不落盘**，一复位就全回到「关」—— 这时候如果不重开，
        # 现象就是手册 0.2 警告的那条：链路通着、[DATA] 在刷，但 /odom 一个点都不出。
        if self._session_ready and line.startswith(READY_MARKER):
            self.get_logger().warn("板子重新就绪 —— 中途复位过，重发上报开关")
            self._send_boot_frames()

        parsed = proto.parse_line(line)
        if parsed is None:
            self._log_board_line(line)
            return

        tag, kv = parsed
        if not kv:
            # 半截行、或者被 ModemManager 打断的行 —— 丢掉，不要当错误记
            # （手册 4.5：不要写 else: log.error("未知行")）。
            return

        stamp = self.get_clock().now().to_msg()
        with self._rx_lock:
            self._rx.append((tag, kv, stamp))

    def _log_board_line(self, line: str) -> None:
        if not self.log_board_lines:
            return
        # [ODOM] 要转，但 5Hz 的 DATA 行不能转 —— 那是遥测，不是日志。
        # 正常情况下 _handle_line 里 parse_line 已经把它截走了，但 _wait_ready
        # 是直接调这里的（不过 parse_line），所以这个排除必须写在函数内部。
        if line.startswith(ODOM_DATA_PREFIX):
            return
        if not line.startswith(FORWARD_PREFIXES):
            return
        if line.startswith("[ERR]") or line.startswith("[FRM]"):
            self.get_logger().warn(f"板子：{line}")
        else:
            self.get_logger().info(f"板子：{line}")

    # ═══════════════════════════ ROS 侧 ═══════════════════════════

    def _on_cmd_vel(self, msg: Twist) -> None:
        # 回调里**只存不发** —— 发帧由 _tx_tick 按 forward_hz 统一做，
        # 这样 20Hz 的 cmd_vel 不会变成 20Hz 的串口流量（手册 4.4）。
        self._latest_twist = msg
        self._latest_twist_t = time.monotonic()

    def _tx_tick(self) -> None:
        if self._ser is None:
            return

        now = time.monotonic()
        fresh = (
            self._latest_twist is not None
            and (now - self._latest_twist_t) * 1000.0 <= self.cmd_timeout_ms
        )

        if fresh:
            tw = self._latest_twist
            v = int(round(_clamp(tw.linear.x, -1e9, 1e9) * 1000.0))
            w = int(round(_clamp(tw.angular.z, -1e9, 1e9) * 1000.0))
            # 固件侧也会夹（clamp_throttle），这里夹一次是为了让日志里的数
            # 和实际发出去的一致。注意：**固件不会为超速报任何错**，
            # 要看 [CMD] 回显里的 v = +100% 才知道被夹到满了。
            v = int(_clamp(v, -self.v_full_scale, self.v_full_scale))
            w = int(_clamp(w, -self.w_full_scale, self.w_full_scale))
            frame = proto.frame_velocity(v, w, self.duration_ms, self._next_seq())
            if self._estop_sent:
                self.get_logger().info("收到新的 cmd_vel，解除急停")
                self._estop_sent = False
        elif not self._estop_sent:
            # 看门狗：cmd_timeout_ms 内没有新的 cmd_vel → 主动急停。
            # 手册 4.4 说这是本设计里最好的一点 —— 节点崩了、香橙派断电，
            # 板子在最后一个 duration_ms 到点时**自己停**，不依赖任何一端活着。
            frame = proto.frame_estop(self._next_seq())
            self._estop_sent = True
            self.get_logger().warn(
                f"cmd_vel 超过 {self.cmd_timeout_ms:g}ms 没更新 → 发 0x02 急停"
            )
        else:
            return

        try:
            self._tx.put_nowait(frame)
        except queue.Full:
            self.get_logger().warn("TX 队列满，丢弃一帧（串口是不是卡住了？）")

    # ── 参数回调：ros2 param set /roscar_bridge dist_scale <值> ──

    def _on_set_params(self, params) -> SetParametersResult:
        """`dist_scale` 变了 → 编一条 0x2A 排队发出去。

        只**排队**不直接写串口：串口 fd 归 IO 线程独占，ROS 侧直接写会和读循环
        抢（见文件头）。`_tx_tick` 发速度帧也是这个路子。
        """
        for p in params:
            if p.name != "dist_scale":
                continue

            v = float(p.value)
            if not DIST_SCALE_MIN <= v <= DIST_SCALE_MAX:
                # 这里选择**让它失败**，而不是像 _tx_tick 那样夹一下。
                # 固件对超范围的值本来就是静默夹掉的，上位机再默默夹一次的话，
                # 现象变成「我写的 8.0，回读是 5.0」而中间没有任何提示 ——
                # 标定的时候看到这个会以为是固件坏了。宁可当场报错。
                return SetParametersResult(
                    successful=False,
                    reason=(
                        f"距离刻度 {v} 超出 {DIST_SCALE_MIN}~{DIST_SCALE_MAX}"
                        f"（固件 ODOM_DIST_SCALE_MIN/MAX），板子会静默夹掉它。"
                    ),
                )

            self.dist_scale = v
            self._queue_dist_scale()
            self.get_logger().info(
                f"距离刻度 → {v:.4f}（0x2A 已排队，回读看 [ODOM] DATA 的 scale=）"
            )

        return SetParametersResult(successful=True)

    def _queue_dist_scale(self) -> None:
        try:
            self._tx.put_nowait(proto.frame_dist_scale(self.dist_scale, self._next_seq()))
        except queue.Full:
            self.get_logger().warn("TX 队列满，0x2A 没发出去 —— 刻度**没改成功**")

    def _note_board_scale(self, scale: float | None) -> None:
        """板子报的 scale 一变就记一行。

        光看「0x2A 已排队」是不够的 —— 那只说明帧发出去了，不代表板子收下了
        （帧可能在复位窗口里被丢，也可能被 `ODOM_DIST_SCALE_MIN/MAX` 夹掉）。
        真正「存进去并生效了」的证据只有这个：`[ODOM] DATA` 的 `scale=` 跟着变。
        标定的时候靠它闭环。
        """
        if scale is None or scale == self._board_scale:
            return
        if self._board_scale is None:
            self.get_logger().info(
                f"板子报的距离刻度：{scale:.4f}"
                + ("　⚠ 还是 1.0（未标定）" if scale == 1.0 else "")
            )
        else:
            self.get_logger().info(f"板子距离刻度已生效：{self._board_scale:.4f} → {scale:.4f}")
        self._board_scale = scale

    def _rx_tick(self) -> None:
        with self._rx_lock:
            if not self._rx:
                return
            items = list(self._rx)
            self._rx.clear()

        # 只留每种数据里最新的一条 —— 队列里可能攒了好几拍。
        # 每条用它**自己到达时**的时刻打戳，不是用定时器的时刻。
        odom = att = imu = None
        odom_stamp = att_stamp = imu_stamp = None
        for tag, kv, stamp in items:
            if tag == "ODOM":
                odom, odom_stamp = kv, stamp
            elif tag == "ATT":
                att, att_stamp = kv, stamp
            elif tag == "IMU":
                imu, imu_stamp = kv, stamp

        if odom is not None:
            self._odom = odom
            self._note_board_scale(odom.get("scale"))
            self._publish_odom(odom, odom_stamp)
            self._last_odom_walltime = time.monotonic()
            self._warned_no_odom = False

        # 有新姿态、或有新 IMU 读数时重发一条 /imu/data。
        # 只来 [IMU] 而没有过 [ATT] 的话填不出 orientation —— 那种情况不发。
        if self.publish_imu and (att is not None or imu is not None):
            if att is not None:
                self._att = att
            if imu is not None:
                self._imu = imu
            if self._att is not None:
                self._publish_imu(
                    self._att, self._imu, att_stamp or imu_stamp
                )

    # ── 发布 ──

    def _publish_odom(self, d: dict[str, float], stamp) -> None:
        # 手册 4.3：板子报的既不是 SI，也不统一。照抄进 ROS2 消息不会报错 ——
        # 消息里只有数，没有单位。
        x = d.get("x", 0.0) / 100.0  # cm → m
        y = d.get("y", 0.0) / 100.0  # cm → m
        # TF 和 /odom 用 th（相对里程计归零点的角），
        # **不是** [ATT] 的 yaw（开机以来的累计角，不折回）。二者不是一回事。
        th = math.radians(d.get("th", 0.0))  # 度 → rad
        v = d.get("v", 0.0) / 100.0  # cm/s → m/s
        w = math.radians(d.get("w", 0.0))  # 度/s → rad/s

        qx, qy, qz, qw = proto.quat_from_yaw(th)

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w
        msg.pose.covariance = _cov36(
            {
                0: self.pose_cov_xy,
                7: self.pose_cov_xy,
                14: self.cov_unobservable,
                21: self.cov_unobservable,
                28: self.cov_unobservable,
                35: self.pose_cov_yaw,
            }
        )
        msg.twist.covariance = _cov36(
            {
                0: self.twist_cov_vx,
                7: self.cov_unobservable,
                14: self.cov_unobservable,
                21: self.cov_unobservable,
                28: self.cov_unobservable,
                35: self.twist_cov_wz,
            }
        )
        self._odom_pub.publish(msg)

        # TF **不**在这一拍发：交给 _tf_tick 按 tf_rate_hz 高频补发（原因见参数处）。
        # 这里只记下最新位姿；/odom 话题保持原来的节奏和到达时刻戳不变。
        self._tf_pose = (x, y, qx, qy, qz, qw)

    def _tf_tick(self) -> None:
        """按 tf_rate_hz 反复广播最新的 odom→base_link。

        打的是**当前时刻**的戳，不是里程计到达时刻：/tf 的时间轴因此永远是
        密的，晚到 ~95ms 的雷达帧回头查也总有一对样本包住它 —— 这正是
        slam 里 tf2 MessageFilter 挂 pending 请求的触发条件，铺密了就没了。
        """
        if self._tf_broadcaster is None:
            return
        pose = self._tf_pose
        if pose is None:
            return  # 还没收到过任何一拍里程计，没有位姿可发
        x, y, qx, qy, qz, qw = pose
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(t)

    def _publish_imu(self, att: dict[str, float], imu: dict[str, float] | None, stamp) -> None:
        # 朝向用 [ATT] 的 roll/pitch/yaw（累计角），不是 th。
        # 手册 4.3：做 TF 用 th，做 /imu/data 的朝向用 yaw。
        roll = math.radians(att.get("roll", 0.0))
        pitch = math.radians(att.get("pitch", 0.0))
        yaw = math.radians(att.get("yaw", 0.0))
        qx, qy, qz, qw = proto.quat_from_euler(roll, pitch, yaw)

        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self.base_frame  # 固件已经把 IMU 的轴摆正了
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        if imu is not None:
            # [IMU] 的角速度是 °/s，加速度是 g —— 都不是 SI。
            msg.angular_velocity.x = math.radians(imu.get("gx", 0.0))
            msg.angular_velocity.y = math.radians(imu.get("gy", 0.0))
            msg.angular_velocity.z = math.radians(imu.get("gz", 0.0))
            msg.linear_acceleration.x = imu.get("ax", 0.0) * proto.G_TO_MS2
            msg.linear_acceleration.y = imu.get("ay", 0.0) * proto.G_TO_MS2
            msg.linear_acceleration.z = imu.get("az", 0.0) * proto.G_TO_MS2
            av_cov = self.twist_cov_wz
            la_cov = 0.1
        else:
            av_cov = self.cov_unobservable
            la_cov = self.cov_unobservable

        # 三个都是 3x3（9 元素）：对角元 0=xx, 4=yy, 8=zz
        msg.orientation_covariance = _cov9(
            {
                0: self.pose_cov_yaw,  # roll  —— 互补滤波，roll/pitch 比 yaw 准
                4: self.pose_cov_yaw,  # pitch
                8: self.pose_cov_yaw,  # yaw   —— 陀螺纯积分，会漂
            }
        )
        msg.angular_velocity_covariance = _cov9({0: av_cov, 4: av_cov, 8: av_cov})
        msg.linear_acceleration_covariance = _cov9({0: la_cov, 4: la_cov, 8: la_cov})
        self._imu_pub.publish(msg)

    # ── 诊断 ──

    def _diag_tick(self) -> None:
        if time.monotonic() - self._last_odom_walltime < self.no_telemetry_warn_s:
            return
        if self._warned_no_odom:
            return
        self._warned_no_odom = True
        self.get_logger().warn(
            f"启动 {self.no_telemetry_warn_s:g}s 了还没收到 [ODOM] DATA。"
            f"最常见的两个原因：① 0x28 没发出去（遥测默认是关的，且开关不落盘）；"
            f"② 板子还没就绪就发了。链路本身可能是好的 —— 看有没有 [DATA] 在刷。"
        )

    # ── 关闭 ──

    def shutdown(self) -> None:
        # 主动发一条急停。**顺序很要紧**：一旦 _stop 置位，IO 线程就会退出
        # 循环、在 finally 里把串口关掉（_ser 变 None），那时再想发就晚了 ——
        # 所以是「先排队 → 等 IO 线程发出去 → 再停」。
        #
        # 不依赖它也行：duration_ms 到点板子会自己停，看门狗也兜得住。
        # 这里只是让「正常退出」比「异常掉线」干净一点。
        if self._ser is not None:
            try:
                self._tx.put_nowait(proto.frame_estop(self._next_seq()))
                time.sleep(0.05)  # 读循环 20ms 一圈，50ms 够它转两圈
            except queue.Full:
                pass
        self._stop.set()
        self._io_thread.join(timeout=2.0)
        self._close()
        if rclpy.ok():  # 上下文已经关了的话，这条日志发不出去，只会招来一行 rosout 报错
            self.get_logger().info("桥接节点已停止")


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = RosCarBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C 时 rclpy 的信号处理会先把上下文 shutdown 掉，spin() 再抛
        # ExternalShutdownException。不接住的话**每次 Ctrl-C 都吐一大段
        # traceback**，看着像崩了，其实是一次干净退出。
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
