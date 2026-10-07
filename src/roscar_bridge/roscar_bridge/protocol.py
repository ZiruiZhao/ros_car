"""串口帧编解码 —— 与 `串口通信手册.md` 1.1 / 1.2 / 1.4 节一一对应。

这个模块**不 import ROS**，是纯编解码层。理由和固件那边把 `lib/serial_frame`
和底盘分开是同一个：不认识速度、不认识里程计的这一层，才是能一行一行对着字节
看的那一层。出问题时它能单独跑测试，不用起 ROS。

跨语言契约锚（`开发流程.md` 4 节「下位机怎么验」）：

    前进 150mm/s 2000ms  →  EB 90 01 01 00 06 96 00 00 00 D0 07 4B 66

改协议时先看这串变没变，变了就是契约动了 —— 固件 `lib/serial_frame`、上位机
`test/car_upper.py` 顶部、`串口通信手册.md` 1.5 节必须一起改。
"""

from __future__ import annotations

import math

# ─────────────────────────── 帧结构 (手册 1.1) ───────────────────────────

FRAME_HEAD = b"\xEB\x90"
ADDR_LOCAL = 0x01
ADDR_BROADCAST = 0xFF

FRAME_TOTAL_MAX = 40  # 8 开销 + 32 数据域

# 功能码 (手册 1.3)。bit7 = 方向位：0 = 上位机→下位机。
FC_VEL = 0x01
FC_ESTOP = 0x02
FC_TURN = 0x03
FC_TURN_ABORT = 0x04
FC_PID_CHASSIS = 0x05

FC_IMU_ONCE = 0x20
FC_IMU_STREAM = 0x21
FC_GYRO_CALIB = 0x22
FC_I2C_SCAN = 0x23
FC_ATT_ONCE = 0x24
FC_ATT_STREAM = 0x25
FC_YAW_ZERO = 0x26
FC_ODOM_ONCE = 0x27
FC_ODOM_STREAM = 0x28
FC_ODOM_RESET = 0x29
FC_DIST_SCALE = 0x2A
FC_PID_YAW = 0x2B
FC_FRAME_STATS = 0x2C
FC_DATA_STREAM = 0x2D
FC_DIST_SCALE_READ = 0x2E

# ──────────────────────── 量化常数 (手册 1.4) ────────────────────────
#
# 这些常数在三处必须一致：固件 src/main.cpp 的 PROTO_*、test/car_upper.py 顶部、
# 串口通信手册.md 1.4 节。改一个三处都得改。

V_FULL_SCALE_MM_S = 300  # 满油门线速度 30 cm/s
W_FULL_SCALE_MRAD_S = 5000  # 满油门角速度（标称值，非标定值）
DEG_TO_MRAD = 17.4533
DEG_TO_RAD = math.pi / 180.0
G_TO_MS2 = 9.80665  # [IMU] 的加速度单位是 g，不是 m/s²


# ────────────────────────── CRC16-Modbus (手册 1.2) ──────────────────────────


def crc16_modbus(data: bytes) -> int:
    """多项式 0xA001（0x8005 反射），初值 0xFFFF，无最终异或。

    范围 = 设备地址 .. 数据域末尾 —— **不含帧头，也不含 CRC 自己**。
    这是最容易写错的一处，写错的现象是「帧全被丢掉，不报任何错」。
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def _check_crc_anchor() -> bool:
    """CRC-16/MODBUS 的标准校验值。

    这是整套里**唯一不依赖本工程任何代码的判据** —— 拿被测代码算出来的 CRC
    去验被测代码的帧是自证，锚不住。差一个多项式两边就永远对不上。
    """
    return crc16_modbus(b"123456789") == 0x4B37


CRC_ANCHOR_OK = _check_crc_anchor()


# ───────────────────────────── 编码侧 ─────────────────────────────


def i16(v: int) -> bytes:
    """int16 小端**二进制补码**。

    负数必须走补码：按无符号打包的话 −785 会变成 0xFF6F，板子解出来是个很大的
    正数，车朝反方向转（手册 1.5 的警告）。
    """
    v = int(v)
    if not -0x8000 <= v <= 0x7FFF:
        raise ValueError(f"i16 溢出：{v}")
    return v.to_bytes(2, "little", signed=True)


def u16(v: int) -> bytes:
    v = int(v)
    if not 0 <= v <= 0xFFFF:
        raise ValueError(f"u16 溢出：{v}")
    return v.to_bytes(2, "little")


def i32(v: int) -> bytes:
    v = int(v)
    if not -0x80000000 <= v <= 0x7FFFFFFF:
        raise ValueError(f"i32 溢出：{v}")
    return v.to_bytes(4, "little", signed=True)


def build_frame(func: int, payload: bytes = b"", seq: int = 0, addr: int = ADDR_LOCAL) -> bytes:
    """组装一条下行帧。

    body = 地址 + 功能码 + 序号 + 长度 + 数据域；CRC 覆盖 body 全部，小端附在尾部。
    """
    if func & 0x80:
        raise ValueError(
            f"功能码 {func:#04x} 的 bit7 是方向位，下行请求只能用 0x00~0x7F。"
            f"应答码 = 请求码 | 0x80 是上行用的。"
        )
    if len(payload) > 32:
        raise ValueError(f"数据域 {len(payload)} 字节超过上限 32（手册 0 节 FRAME_TOTAL_MAX）")
    body = bytes([addr, func, seq & 0xFF, len(payload)]) + payload
    return FRAME_HEAD + body + crc16_modbus(body).to_bytes(2, "little")


def frame_velocity(v_mm_s: int, w_mrad_s: int, duration_ms: int, seq: int = 0) -> bytes:
    """0x01 底盘速度控制：v(int16 mm/s) + w(int16 mrad/s) + 时长(uint16 ms)。

    v 和 w 是**独立的两个字段** —— 两个都非零就是走圆弧，不需要任何新功能码。
    """
    return build_frame(FC_VEL, i16(v_mm_s) + i16(w_mrad_s) + u16(duration_ms), seq)


def frame_estop(seq: int = 0) -> bytes:
    """0x02 底盘急停。看门狗超时后发这条。"""
    return build_frame(FC_ESTOP, b"", seq)


def frame_stream(func: int, on: bool, seq: int = 0) -> bytes:
    """上报开关（0x21 / 0x25 / 0x28 / 0x2D）。

    数据域里直接写**目标状态**，不是「切换」—— 发几次结果都一样，上位机不用
    记着固件现在开着没。做成切换的话不同步之后现象是「点了没反应」，最难查。
    """
    return build_frame(func, bytes([1 if on else 0]), seq)


def frame_turn_to(angle_deg: float, seq: int = 0) -> bytes:
    """0x03 定角度转向，目标角 int16 mrad，正 = 左转。"""
    return build_frame(FC_TURN, i16(round(angle_deg * DEG_TO_MRAD)), seq)


def frame_pid_chassis(kp: float, ki: float, kd: float, seq: int = 0) -> bytes:
    """0x05 轮速环 PID，增益 ×1000。"""
    return build_frame(
        FC_PID_CHASSIS,
        u16(round(kp * 1000)) + u16(round(ki * 1000)) + u16(round(kd * 1000)),
        seq,
    )


def frame_pid_yaw(kp: float, ki: float, kd: float, seq: int = 0) -> bytes:
    """0x2B yaw 环 PID，增益 **×10000**（不是 0x05 那套 ×1000，差 10 倍）。"""
    return build_frame(
        FC_PID_YAW,
        u16(round(kp * 10000)) + u16(round(ki * 10000)) + u16(round(kd * 10000)),
        seq,
    )


def frame_dist_scale(scale: float, seq: int = 0) -> bytes:
    """0x2A 距离刻度，uint16（刻度 **×10000**）。

    ⚠ 同一份手册里有三个「增益 ×N」的旋钮：`0x05` 是 ×1000、`0x2B` 是 ×10000、
    这个也是 ×10000。抄错一个量级的现象是「写进去的数和回读的对不上」，
    而车子照跑不误 —— 所以下面挂了契约锚，别凭记忆填。

    固件把值夹在 `ODOM_DIST_SCALE_MIN/MAX` = 0.2 ~ 5.0（`开发流程.md`「参数表」），
    超出会被**静默夹掉**，不会报错。范围校验放在调用方。
    """
    return build_frame(FC_DIST_SCALE, u16(round(scale * 10000)), seq)


def frame_dist_scale_read(seq: int = 0) -> bytes:
    """0x2E 回读距离刻度。

    注意：`[ODOM] DATA` 文本行里本来就有 `scale` 字段（5Hz 一直在发），
    平时回读用那个就够，不必发这条。这条是板上遥测关着时的退路。
    """
    return build_frame(FC_DIST_SCALE_READ, b"", seq)


# 契约锚：手册 1.5 节第一行的样例帧，逐字节核对过。
REFERENCE_FRAME = frame_velocity(150, 0, 2000, seq=0)
REFERENCE_FRAME_HEX = "EB 90 01 01 00 06 96 00 00 00 D0 07 4B 66"


def check_reference_frame() -> bool:
    """自检：编出来的样例帧必须和手册 1.5 节的字节串完全一致。"""
    return REFERENCE_FRAME.hex(" ").upper() == REFERENCE_FRAME_HEX


# 契约锚：手册 1.5 节「距离刻度 1.083」那行，逐字节核对过。
# 和上一条是**两种错法**：那条抓 v/w/时长 的字段顺序，这条抓刻度的量级
# （×10000 写成 ×1000 的话，这串会变成 ...00 01 0E 22 ... 之类，一眼能看出来）。
DIST_SCALE_REFERENCE_HEX = "EB 90 01 2A 00 02 4E 2A 0D B3"


def check_dist_scale_frame() -> bool:
    """自检：1.083 编出来必须正好是手册那两个字节 `4E 2A` = 10830。"""
    return frame_dist_scale(1.083).hex(" ").upper() == DIST_SCALE_REFERENCE_HEX


# ───────────────────────────── 解码侧 ─────────────────────────────
#
# 手册 1.7：**板子不回数据帧**，固件根本没有发帧的代码，它是纯接收端。
# 所有回话都是文本日志行，按 \n 切。1.8 节那套上行帧（0xA4/0xA7/0xA0/0xAC）
# 是**设计稿，尚未实现** —— 所以在那边落地之前，这一层只解析文本。

PREFIX_ODOM = "[ODOM] DATA "
PREFIX_ATT = "[ATT] DATA "
PREFIX_IMU = "[IMU] DATA "
PREFIX_DATA = "[DATA] "

PREFIXES = (PREFIX_ODOM, PREFIX_ATT, PREFIX_IMU, PREFIX_DATA)


def parse_kv(body: str) -> dict[str, float]:
    """解析 `k1=v1,k2=v2,...`。坏字段跳过，不抛异常。

    板上打印用的是 %f，正常不会出现解析不了的字段；但半截行、被 ModemManager
    打断的行都会走到这里，丢掉比崩掉好。
    """
    out: dict[str, float] = {}
    for item in body.split(","):
        key, sep, val = item.strip().partition("=")
        if not sep:
            continue
        try:
            out[key.strip()] = float(val)
        except ValueError:
            continue
    return out


def parse_line(line: str) -> tuple[str, dict[str, float]] | None:
    """把一行日志切成 (类型, 字段表)；不是遥测行就返回 None。

    只认四种带 ` DATA ` 的遥测前缀。板子还会吐 `[CMD]` 回显、`[SYS]` 启动日志、
    `[ERR]` / `[FRM]` / `[YAW]` …… 这些**一律默默丢掉**，不要当错误记。

    注意 `[DATA]` 那条的格式是 `[DATA] enc_l=...`，没有 "DATA" 关键字。
    """
    if line.startswith(PREFIX_ODOM):
        return "ODOM", parse_kv(line[len(PREFIX_ODOM) :])
    if line.startswith(PREFIX_ATT):
        return "ATT", parse_kv(line[len(PREFIX_ATT) :])
    if line.startswith(PREFIX_IMU):
        return "IMU", parse_kv(line[len(PREFIX_IMU) :])
    if line.startswith(PREFIX_DATA):
        return "DATA", parse_kv(line[len(PREFIX_DATA) :])
    return None


# ───────────────────────────── 单位换算 ─────────────────────────────
#
# 手册 4.3：板子报的既不是 SI，也不统一。照抄进 ROS2 消息不会报错 ——
# 消息里只有数，没有单位。数错了就是「看起来在工作，但融合结果完全不对」。


def quat_from_yaw(yaw_rad: float) -> tuple[float, float, float, float]:
    """只绕 Z 的旋转 → 四元数 (x, y, z, w)。里程计用这个。"""
    return 0.0, 0.0, math.sin(yaw_rad * 0.5), math.cos(yaw_rad * 0.5)


def quat_from_euler(
    roll: float, pitch: float, yaw: float
) -> tuple[float, float, float, float]:
    """ZYX 欧拉角 → 四元数 (x, y, z, w)。IMU 朝向用这个。"""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


if __name__ == "__main__":
    # 单独跑：python3 -m roscar_bridge.protocol
    print(f"CRC 自检锚 crc16_modbus(b'123456789') == 0x4B37 : {CRC_ANCHOR_OK}")
    print(f"契约锚   {REFERENCE_FRAME.hex(' ').upper()}")
    print(f"期望     {REFERENCE_FRAME_HEX}")
    print(f"一致     : {check_reference_frame()}")
    print(f"刻度锚   {frame_dist_scale(1.083).hex(' ').upper()}")
    print(f"期望     {DIST_SCALE_REFERENCE_HEX}")
    print(f"一致     : {check_dist_scale_frame()}")
    print(f"读刻度   {frame_dist_scale_read().hex(' ').upper()}  (手册 1.5: EB 90 01 2E 00 00 61 D1)")
