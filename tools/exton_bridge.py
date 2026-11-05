#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DM-EXton2 ↔ 客户双臂控制器 双向桥接（机械臂通信协议 V0.3）——异步收发版。

依据：
  《机械臂通信协议说明_V0.3》
  《机器人手眼标定平台系统信息档案》
  hr_up_7dof_fixed / robot_fk_adapter / robot_ik_retargeting

协议要点（V0.3）：
  - TCP，单连接；默认现场 IP 192.168.61.60:30485（档案）
  - 每帧：JSON 数据域 + 帧尾 '#'
  - 17 维 joint，单位「弧度」，顺序：腰 / L1~L7 / R1~R7 / 摇头 / 点头
  - -100 表示该关节保持不动
  - 遥操作透传用 movej_canfd；点到点用 movej（带 v=0~100）
  - 反馈用 get_joint_position

与 hr_up_7dof_fixed 映射：
  L1~L7 (下标 1~7)  → Joint1L~Joint7L / Link7L（左臂）
  R1~R7 (下标 8~14) → Joint1~Joint7   / Link7 （右臂）
  腰/头              → URDF 无，发 -100

架构（V0.3-async2，收发双线程）：
  IK → /target_robot/joint_commands
      → 本节点计算限幅步进 → 写「最新运动目标槽」（覆盖旧目标）
      → 发送线程精确节拍下发 movej/movej_canfd + 周期 get_joint_position
  接收线程 select→recv→解析 → 原子快照 latest_joints + 响应队列
      → /client_robot/joint_states（URDF 名，弧度）→ FK
  ROS 回调只做「写槽」和「读快照」，永不阻塞 socket。

  ★ 修复：旧版在单线程 executor 回调内 poll_recv（50ms 阻塞）×30Hz 查询
    +50Hz 下发，每秒排队 5.5s 阻塞工作，反馈实测仅 6.6 Hz（控制器本身
    往返 0.4ms / 可跑 2000Hz+），导致下发与反馈双双滞后。

部署：
  1. 关闭占用 30485 的机械臂图形界面（单连接）
  2. source /opt/ros/humble/setup.bash && source <ws>/install/setup.bash
  3. export ROS_DOMAIN_ID=42
  4. 先启 bridge 等同步（日志出现「已用 get_joint_position 同步」），再启 IK
"""

from __future__ import annotations

import collections
import json
import math
import queue
import select
import socket
import threading
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# ---------------------------------------------------------------------------
# 协议 V0.3：17 维下标
# ---------------------------------------------------------------------------
N_JOINTS = 17
IDX_WAIST = 0
IDX_LEFT_START = 1    # L1..L7 → 1..7
IDX_RIGHT_START = 8   # R1..R7 → 8..14
IDX_HEAD_SHAKE = 15
IDX_HEAD_NOD = 16
ARM_DOF = 7
HOLD = -100.0
FRAME_TAIL = '#'

# hr_up_7dof_fixed（与 ik_retargeting / fk_adapter 一致）
LEFT_URDF_JOINTS = [
    'Joint1L', 'Joint2L', 'Joint3L', 'Joint4L', 'Joint5L', 'Joint6L', 'Joint7L',
]
RIGHT_URDF_JOINTS = [
    'Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6', 'Joint7',
]
ALL_URDF_JOINTS = RIGHT_URDF_JOINTS + LEFT_URDF_JOINTS  # 发布顺序：右+左（按名字匹配）

# 档案右臂软限位（度）→ 运行时转弧度裁剪；左臂默认用 URDF ±π
ARCHIVE_RIGHT_LIMITS_DEG: List[tuple] = [
    (-179.9 + 2.0, 179.9 - 2.0),
    (-148.97 + 2.0, 154.70 - 2.0),
    (-39.53 + 2.0, 129.49 - 2.0),
    (-148.97 + 2.0, 154.70 - 2.0),
    (-89.95 + 2.0, 89.95 - 2.0),
    (-154.70 + 2.0, 154.70 - 2.0),
    (-154.70 + 2.0, 154.70 - 2.0),
]
URDF_LIMITS_RAD: List[tuple] = [(-3.14 + 0.05, 3.14 - 0.05)] * ARM_DOF

# 档案 default_arm_pose（度）→ 仅作右臂缺省参考，真正下发前会被反馈覆盖
DEFAULT_RIGHT_HOME_DEG = [-86.14, -88.6, 98.22, 18.61, 90.0, -13.45, 89.81]


def deg_to_rad(deg: float) -> float:
    return deg * math.pi / 180.0


def clip(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def limits_deg_to_rad(limits_deg: List[tuple]) -> List[tuple]:
    return [(deg_to_rad(lo), deg_to_rad(hi)) for lo, hi in limits_deg]


def extract_joint_rad(msg: dict) -> Optional[List[float]]:
    """从应答中提取 joint 数组（弧度）。"""
    candidates = []
    for key in ('joint', 'joints'):
        if key in msg:
            candidates.append(msg[key])
    for value in candidates:
        if isinstance(value, list) and len(value) >= ARM_DOF:
            try:
                return [float(x) for x in value]
            except (TypeError, ValueError):
                continue
    return None


class AsyncTcpClient:
    """协议 V0.3 异步客户端：收发各自独立线程，ROS 回调零阻塞。

    - 接收线程：select(50ms) → recv → 解析帧 → 原子快照 latest_joint_rad
      + 响应队列（供状态检查，不阻塞）。
    - 发送线程：单循环内 ①控制命令 FIFO（enable/stop，保证送达）
      ②运动目标槽（最新覆盖，防过期帧堆积）③get_joint_position 周期查询。
    - 所有 send 帧尾 '#'、TCP_NODELAY。
    """

    def __init__(self, host: str, port: int, query_hz: float = 30.0):
        self.host = host
        self.port = port
        self.query_period = 1.0 / max(query_hz, 1.0)
        self._sock: Optional[socket.socket] = None
        self._connected = False
        self._closing = False
        self._send_lock = threading.Lock()
        self._state_lock = threading.Lock()
        self._latest_joint_rad: Optional[List[float]] = None
        self._responses: collections.deque = collections.deque(maxlen=64)
        self._rx_buffer = ''
        self._rx_thread: Optional[threading.Thread] = None
        self._tx_thread: Optional[threading.Thread] = None
        self._ctrl_queue: 'queue.Queue[dict]' = queue.Queue()
        self._motion_cv = threading.Condition()
        self._motion_slot: Optional[dict] = None
        # movej 是阻塞型程序；同一 TCP 连接上任意时刻只允许一条在途。
        # rx 只把“command=movej 且带 status”的应答计为完成，
        # get_joint_position 的 joint 反馈不能解锁下一条运动指令。
        self._movej_state_lock = threading.Lock()
        self._movej_ack_seq = 0
        self._movej_sent_count = 0
        self._movej_timeout_count = 0
        self._movej_timeout_s = 0.3
        self._last_error: Optional[str] = None

    @property
    def connected(self) -> bool:
        return self._connected and self._sock is not None

    # ----------------------------------------------------------------
    # 连接管理（由节点线程调用）
    # ----------------------------------------------------------------
    def connect(self) -> None:
        self.close()
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)
        sock.connect((self.host, self.port))
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(None)
        self._sock = sock
        self._rx_buffer = ''
        self._last_error = None
        self._closing = False
        self._connected = True
        self._rx_thread = threading.Thread(
            target=self._rx_loop, name='rx', daemon=True)
        self._tx_thread = threading.Thread(
            target=self._tx_loop, name='tx', daemon=True)
        self._rx_thread.start()
        self._tx_thread.start()

    def close(self) -> None:
        self._closing = True
        self._connected = False
        with self._motion_cv:
            self._motion_slot = None
            self._motion_cv.notify_all()
        for t in (self._rx_thread, self._tx_thread):
            if t is not None and t.is_alive():
                t.join(timeout=1.0)
        self._rx_thread = None
        self._tx_thread = None
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
        self._sock = None

    def take_error(self) -> Optional[str]:
        err = self._last_error
        self._last_error = None
        return err

    # ----------------------------------------------------------------
    # 发送 API（非阻塞，线程安全）
    # ----------------------------------------------------------------
    def send_control(self, payload: dict) -> None:
        """一次性控制命令（enable/stop/get_joint_position 首查），FIFO 保证送达。"""
        self._ctrl_queue.put(dict(payload))

    def set_motion_target(self, payload: dict) -> None:
        """写入最新运动目标；旧目标直接覆盖，恢复后只追最新。"""
        with self._motion_cv:
            self._motion_slot = dict(payload)
            self._motion_cv.notify()

    def clear_motion(self) -> None:
        with self._motion_cv:
            self._motion_slot = None
            self._motion_cv.notify_all()

    def poll_responses(self) -> List[dict]:
        """非阻塞取走全部待处理应答（deque 操作本身线程安全）。"""
        out: List[dict] = []
        try:
            while True:
                out.append(self._responses.popleft())
        except IndexError:
            pass
        return out

    def get_latest_joint_rad(self) -> Optional[List[float]]:
        with self._state_lock:
            if self._latest_joint_rad is None:
                return None
            return list(self._latest_joint_rad)

    def get_movej_lock_stats(self) -> tuple:
        """返回 movej 实际发送/应答/超时计数，用于现场验证 lock-step。"""
        with self._movej_state_lock:
            return (
                self._movej_sent_count,
                self._movej_ack_seq,
                self._movej_timeout_count,
            )

    # ----------------------------------------------------------------
    # 内部：发送帧
    # ----------------------------------------------------------------
    def _send_now(self, payload: dict) -> None:
        line = json.dumps(
            payload, ensure_ascii=False, separators=(',', ':')) + FRAME_TAIL
        with self._send_lock:
            sock = self._sock
            if sock is None or not self._connected:
                raise ConnectionError('TCP 未连接')
            sock.sendall(line.encode('utf-8'))

    def _tx_loop(self) -> None:
        next_query = time.perf_counter()
        waiting_movej_seq: Optional[int] = None
        movej_deadline = 0.0
        while not self._closing:
            if self._sock is None or not self._connected:
                time.sleep(0.05)
                continue
            try:
                # movej 的 status 应答到达前不发下一条 movej。等待期间
                # 仍然处理 stop/enable 和关节查询，不让安全控制被锁住。
                if waiting_movej_seq is not None:
                    with self._movej_state_lock:
                        acked = self._movej_ack_seq > waiting_movej_seq
                    if acked:
                        waiting_movej_seq = None
                    elif time.perf_counter() >= movej_deadline:
                        with self._movej_state_lock:
                            self._movej_timeout_count += 1
                        waiting_movej_seq = None

                # ① 控制命令优先清空（enable/stop 等，保证送达）
                while True:
                    try:
                        payload = self._ctrl_queue.get_nowait()
                    except queue.Empty:
                        break
                    self._send_now(payload)

                # ② 运动目标槽（最新覆盖）
                with self._motion_cv:
                    payload = self._motion_slot
                    if payload is not None:
                        self._motion_slot = None
                if payload is not None:
                    is_movej = payload.get('command') == 'movej'
                    if is_movej and waiting_movej_seq is not None:
                        # 在途时把最新目标放回单槽；旧目标仍会被新目标覆盖。
                        with self._motion_cv:
                            if self._motion_slot is None:
                                self._motion_slot = payload
                    else:
                        if is_movej:
                            with self._movej_state_lock:
                                sent_after_seq = self._movej_ack_seq
                        self._send_now(payload)
                        if is_movej:
                            with self._movej_state_lock:
                                self._movej_sent_count += 1
                            waiting_movej_seq = sent_after_seq
                            movej_deadline = (
                                time.perf_counter() + self._movej_timeout_s
                            )

                # ③ 周期查询反馈
                now = time.perf_counter()
                if now >= next_query:
                    self._send_now({'command': 'get_joint_position'})
                    next_query = now + self.query_period
                    continue

                # 空闲等待：有新槽/控制命令立即唤醒，否则睡到下个查询点
                timeout = max(0.001, min(next_query - time.perf_counter(), 0.02))
                with self._motion_cv:
                    if self._motion_slot is None and self._ctrl_queue.empty():
                        self._motion_cv.wait(timeout)
            except (OSError, ConnectionError) as exc:
                if not self._closing:
                    self._last_error = f'发送断开: {exc}'
                    self._connected = False
                return

    def _rx_loop(self) -> None:
        while not self._closing:
            sock = self._sock
            if sock is None:
                break
            try:
                readable, _, _ = select.select([sock], [], [], 0.05)
                if not readable:
                    continue
                chunk = sock.recv(4096)
                if not chunk:
                    raise ConnectionError('控制器关闭连接')
                self._feed(chunk.decode('utf-8', errors='ignore'))
            except (OSError, ConnectionError, ValueError) as exc:
                if not self._closing:
                    self._last_error = f'接收断开: {exc}'
                    self._connected = False
                return

    def _feed(self, text: str) -> None:
        with self._state_lock:
            self._rx_buffer += text
            while FRAME_TAIL in self._rx_buffer:
                raw, self._rx_buffer = self._rx_buffer.split(FRAME_TAIL, 1)
                raw = raw.strip()
                if not raw:
                    continue
                try:
                    msg = json.loads(raw)
                except json.JSONDecodeError:
                    continue
                self._responses.append(msg)
                if msg.get('command') == 'movej' and 'status' in msg:
                    with self._movej_state_lock:
                        self._movej_ack_seq += 1
                    # 立即唤醒 tx，不必等待空闲轮询周期。
                    with self._motion_cv:
                        self._motion_cv.notify_all()
                joints = extract_joint_rad(msg)
                if joints is not None and len(joints) >= N_JOINTS:
                    self._latest_joint_rad = joints[:N_JOINTS]
                elif joints is not None and len(joints) >= IDX_RIGHT_START + ARM_DOF:
                    # 容错：不足 17 时也尽量用
                    padded = [HOLD] * N_JOINTS
                    for i, v in enumerate(joints):
                        if i < N_JOINTS:
                            padded[i] = v
                    self._latest_joint_rad = padded


class ExtonBridge(Node):
    def __init__(self) -> None:
        super().__init__('exton_bridge')

        # ---- 网络 / 协议 ----
        self.declare_parameter('controller_host', '192.168.61.60')
        self.declare_parameter('controller_port', 30485)
        self.declare_parameter('auto_reconnect', True)
        self.declare_parameter('auto_enable', False)  # True 则连接后发 enable_robot（慎用）
        # movej_canfd = 透传遥操作；movej = 带速度规划（首次真机更安全）
        self.declare_parameter('control_mode', 'movej_canfd')
        self.declare_parameter('move_speed', 15.0)   # movej 的 v：0~100
        self.declare_parameter('canfd_freq', 50)     # movej_canfd 的 freq
        self.declare_parameter('command_rate_hz', 50.0)
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('feedback_rate_hz', 30.0)
        self.declare_parameter('command_topic', '/target_robot/joint_commands')
        self.declare_parameter('state_topic', '/client_robot/joint_states')
        self.declare_parameter('enable_right', True)
        self.declare_parameter('enable_left', True)
        self.declare_parameter('use_archive_right_limits', True)
        self.declare_parameter('send_only_on_change', False)  # 遥操建议 false，否则易只下发一帧
        self.declare_parameter('command_eps_rad', 0.001)
        # 仅丢弃接近整圈/错误初值的轴；普通大偏差改为限速追（见 apply_arm）
        self.declare_parameter('max_joint_jump_rad', 3.0)
        # 每周期最大步进（rad）。跟手不够大就加大，例如 0.15~0.30
        self.declare_parameter('max_joint_step_rad', 0.20)
        # 1.0=每帧尽量跟上 IK 目标；<1 会更滞后
        self.declare_parameter('follow_ratio', 1.0)
        # 调试：打印每次下发与 TCP 应答
        self.declare_parameter('debug_tcp', False)

        host = str(self.get_parameter('controller_host').value)
        port = int(self.get_parameter('controller_port').value)
        self.auto_reconnect = bool(self.get_parameter('auto_reconnect').value)
        self.auto_enable = bool(self.get_parameter('auto_enable').value)
        self.control_mode = str(self.get_parameter('control_mode').value).lower().strip()
        self.move_speed = float(self.get_parameter('move_speed').value)
        self.canfd_freq = int(self.get_parameter('canfd_freq').value)
        self.enable_right = bool(self.get_parameter('enable_right').value)
        self.enable_left = bool(self.get_parameter('enable_left').value)
        self.send_only_on_change = bool(self.get_parameter('send_only_on_change').value)
        self.command_eps_rad = float(self.get_parameter('command_eps_rad').value)
        self.max_joint_jump_rad = float(self.get_parameter('max_joint_jump_rad').value)
        self.max_joint_step_rad = float(self.get_parameter('max_joint_step_rad').value)
        self.follow_ratio = float(self.get_parameter('follow_ratio').value)
        self.follow_ratio = max(0.05, min(1.0, self.follow_ratio))
        self.debug_tcp = bool(self.get_parameter('debug_tcp').value)
        self._send_count = 0
        self._cmd_rx_count = 0
        self._fail_count = 0
        self._fb_synced = False
        self._last_status_log = 0.0
        self._jump_warn_count = 0
        self._last_skip_reason = ''  # jump | nodelta | none
        self._skipped_last = 0

        if self.control_mode not in ('movej_canfd', 'movej'):
            raise RuntimeError("control_mode 只能是 'movej_canfd' 或 'movej'")

        self.right_limits = (
            limits_deg_to_rad(ARCHIVE_RIGHT_LIMITS_DEG)
            if bool(self.get_parameter('use_archive_right_limits').value)
            else URDF_LIMITS_RAD
        )
        self.left_limits = URDF_LIMITS_RAD

        fb_hz = float(self.get_parameter('feedback_rate_hz').value)
        self.tcp = AsyncTcpClient(host, port, query_hz=fb_hz)
        self._cmd_lock = threading.Lock()
        self._pending_cmd: Optional[JointState] = None
        self._last_sent_joint: Optional[List[float]] = None

        # 17 维模板：非控制轴 -100；右臂用档案 home 作初值（弧度）
        self._joint_cmd = [HOLD] * N_JOINTS
        for i, deg in enumerate(DEFAULT_RIGHT_HOME_DEG):
            self._joint_cmd[IDX_RIGHT_START + i] = deg_to_rad(deg)

        self.pub_state = self.create_publisher(
            JointState, self.get_parameter('state_topic').value, 10
        )
        self.create_subscription(
            JointState,
            self.get_parameter('command_topic').value,
            self._on_joint_commands,
            10,
        )

        cmd_hz = float(self.get_parameter('command_rate_hz').value)
        pub_hz = float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(1.0 / max(cmd_hz, 1.0), self._flush_commands)
        self.create_timer(1.0 / max(pub_hz, 1.0), self._publish_joint_states)
        self.create_timer(1.0, self._ensure_connected)
        self.create_timer(2.0, self._status_watchdog)

        self._connect_controller()
        self.get_logger().info(
            f'exton_bridge V0.3-async2 | {host}:{port} | mode={self.control_mode} | '
            f'v={self.move_speed} freq={self.canfd_freq} | '
            f'查询={fb_hz:.0f}Hz(发送线程) | '
            f'L→{LEFT_URDF_JOINTS[0]}.. R→{RIGHT_URDF_JOINTS[0]}.. | '
            f'debug_tcp={self.debug_tcp}'
        )
        if self.control_mode == 'movej':
            self.get_logger().info(
                'movej lock-step 已启用：仅 movej status 应答解锁，'
                '300ms 超时兜底'
            )
        self.get_logger().info(
            f'订阅指令话题: {self.get_parameter("command_topic").value} '
            f'（若长期无“已下发”，先查 IK 是否在发 /target_robot/joint_commands）'
        )

    # ------------------------------------------------------------------
    # 连接
    # ------------------------------------------------------------------
    def _connect_controller(self) -> None:
        try:
            self._fb_synced = False
            self.tcp.connect()
            self.get_logger().info(
                f'已连接控制器 {self.tcp.host}:{self.tcp.port}（协议 V0.3，帧尾 #，收发双线程）'
            )
            if self.auto_enable:
                self.tcp.send_control({'command': 'enable_robot'})
                self.get_logger().warn('已发送 enable_robot（请确认臂处于安全姿态）')
            # 首查由发送线程周期任务自动完成，随后 _maybe_sync 同步缓存
            self._maybe_sync()
        except OSError as exc:
            self.get_logger().error(f'连接控制器失败: {exc}（请确认已关闭 30485 图形界面）')

    def _ensure_connected(self) -> None:
        if self.tcp.connected or not self.auto_reconnect:
            return
        self.get_logger().warn('尝试重连控制器 ...')
        self._connect_controller()

    def _maybe_sync(self) -> None:
        """首次拿到真机反馈即同步内部缓存；同步前禁止下发/发布。"""
        if self._fb_synced or not self.tcp.connected:
            return
        joints = self.tcp.get_latest_joint_rad()
        if joints is not None and len(joints) >= N_JOINTS:
            self._joint_cmd = list(joints[:N_JOINTS])
            self._last_sent_joint = None
            with self._cmd_lock:
                self._pending_cmd = None
            self._fb_synced = True
            self.get_logger().info(
                '已用 get_joint_position 同步内部关节缓存，开始允许下发 | '
                f'R0={self._joint_cmd[IDX_RIGHT_START]:.3f} '
                f'L0={self._joint_cmd[IDX_LEFT_START]:.3f}'
            )

    # ------------------------------------------------------------------
    # 控制链：IK → 17 维 → 发送线程
    # ------------------------------------------------------------------
    def _on_joint_commands(self, msg: JointState) -> None:
        with self._cmd_lock:
            self._pending_cmd = msg
            self._cmd_rx_count += 1

    def _flush_commands(self) -> None:
        with self._cmd_lock:
            msg = self._pending_cmd
            self._pending_cmd = None
        # 未同步真机反馈前不下发，防止启动瞬间把错误姿态发给机器人
        if msg is None or not self.tcp.connected or not self._fb_synced:
            return

        fb = self.tcp.get_latest_joint_rad()
        if fb is None or len(fb) < N_JOINTS:
            return
        fb = list(fb[:N_JOINTS])

        name_to_pos = {str(n): float(p) for n, p in zip(msg.name, msg.position)}
        # 以真机反馈为底：臂关节始终发绝对值；仅腰/头用 -100。
        # （若未动轴也发 -100，部分控制器对 movej_canfd 会直接 program_run_failed）
        joints = list(fb)
        updated = False
        hit_left = 0
        hit_right = 0
        skipped = 0
        max_step_abs = 0.0
        max_err_abs = 0.0

        def apply_arm(urdf_names, limits, start_idx) -> int:
            nonlocal updated, skipped, max_step_abs, max_err_abs
            hit = 0
            for i, (uname, (lo, hi)) in enumerate(zip(urdf_names, limits)):
                if uname not in name_to_pos:
                    continue
                ik_val = clip(name_to_pos[uname], lo, hi)
                idx = start_idx + i
                err = ik_val - fb[idx]
                max_err_abs = max(max_err_abs, abs(err))
                # 仅极端错误初值(接近 ±π 量级)才丢弃；普通大偏差仍限速追
                if abs(err) > self.max_joint_jump_rad:
                    skipped += 1
                    continue
                if abs(err) <= self.command_eps_rad:
                    continue
                # 始终朝 IK 方向限速步进（反向时也能跟上，不会冻住）
                step = err * self.follow_ratio
                step = clip(step, -self.max_joint_step_rad, self.max_joint_step_rad)
                joints[idx] = fb[idx] + step
                max_step_abs = max(max_step_abs, abs(step))
                updated = True
                hit += 1
            return hit

        if self.enable_left:
            hit_left = apply_arm(LEFT_URDF_JOINTS, self.left_limits, IDX_LEFT_START)
        if self.enable_right:
            hit_right = apply_arm(RIGHT_URDF_JOINTS, self.right_limits, IDX_RIGHT_START)

        # 腰/头始终保持
        joints[IDX_WAIST] = HOLD
        joints[IDX_HEAD_SHAKE] = HOLD
        joints[IDX_HEAD_NOD] = HOLD

        if not updated or max_step_abs <= self.command_eps_rad:
            if skipped > 0:
                self._last_skip_reason = 'jump'
                self._skipped_last = skipped
                if self._jump_warn_count < 5:
                    self._jump_warn_count += 1
                    self.get_logger().warn(
                        f'IK 相对真机偏差>{self.max_joint_jump_rad}rad，本帧整帧保持（疑似错误初值）。'
                        f'请停 IK，等 bridge 同步后再启 IK。 '
                        f'fb R0={fb[IDX_RIGHT_START]:.3f} L0={fb[IDX_LEFT_START]:.3f}'
                    )
            else:
                self._last_skip_reason = 'nodelta'
                # 按住 Clutch 仍 nodelta 时，打印最大偏差便于定位
                if self.debug_tcp and self._cmd_rx_count % 50 == 0:
                    names = list(name_to_pos.keys())[:8]
                    self.get_logger().warn(
                        f'nodelta: max|IK-fb|={max_err_abs:.4f} rad '
                        f'(需按住Clutch且IK目标变化). names样例={names}'
                    )
            return

        self._last_skip_reason = 'none'

        if skipped and self.debug_tcp and self._send_count % 20 == 0:
            self.get_logger().warn(f'本帧跳过 {skipped} 个偏差过大的关节（保持真机角）')

        if self.send_only_on_change and self._last_sent_joint is not None:
            if all(
                abs(a - b) <= self.command_eps_rad or (a == HOLD and b == HOLD)
                for a, b in zip(joints, self._last_sent_joint)
            ):
                return

        self._joint_cmd = list(joints)
        payload = self._build_motion_payload(joints)
        self.tcp.set_motion_target(payload)  # 最新覆盖，发送线程按节拍发
        self._last_sent_joint = list(joints)
        self._send_count += 1
        if self.debug_tcp and (self._send_count <= 5 or self._send_count % 25 == 0):
            self.get_logger().info(
                f'匹配左{hit_left}/右{hit_right}轴后入槽（跳过{skipped}） step={max_step_abs:.4f}'
            )

    def _build_motion_payload(self, joints: List[float]) -> dict:
        if self.control_mode == 'movej_canfd':
            return {
                'command': 'movej_canfd',
                'joint': joints,
                'freq': self.canfd_freq,
            }
        return {
            'command': 'movej',
            'joint': joints,
            'v': self.move_speed,
        }

    # ------------------------------------------------------------------
    # 反馈链：接收线程快照 → /client_robot/joint_states
    # ------------------------------------------------------------------
    def _publish_joint_states(self) -> None:
        self._maybe_sync()
        # 未同步前不发 state，避免 IK 用错误初值初始化
        if not self._fb_synced:
            return
        joints = self.tcp.get_latest_joint_rad()
        if joints is None:
            # 尚无本帧反馈：用指令缓存顶住 FK（HOLD 槽用 0）
            joints = [
                0.0 if abs(v - HOLD) < 1e-9 else v
                for v in self._joint_cmd
            ]

        right = [
            0.0 if abs(joints[IDX_RIGHT_START + i] - HOLD) < 1e-9
            else float(joints[IDX_RIGHT_START + i])
            for i in range(ARM_DOF)
        ]
        left = [
            0.0 if abs(joints[IDX_LEFT_START + i] - HOLD) < 1e-9
            else float(joints[IDX_LEFT_START + i])
            for i in range(ARM_DOF)
        ]

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        # 名字必须与 tc_urdf 一致，供 robot_state_publisher / FK / IK 使用
        msg.name = list(RIGHT_URDF_JOINTS) + list(LEFT_URDF_JOINTS)
        msg.position = right + left
        self.pub_state.publish(msg)

    # ------------------------------------------------------------------
    # 看门狗：状态汇报 + 异步应答消费（失败计数/enable 失败检测）
    # ------------------------------------------------------------------
    def _status_watchdog(self) -> None:
        # 消费 TCP 应答：统计运动被拒 / enable 失败
        responses = self.tcp.poll_responses()
        for m in responses:
            if not isinstance(m, dict):
                continue
            status = str(m.get('status', ''))
            if status.endswith('failed'):
                self._fail_count += 1
                if self._fail_count <= 5 or self._fail_count % 20 == 0:
                    self.get_logger().error(
                        f'控制器拒绝运动 {m.get("command", "?")} {status} '
                        f'(累计{self._fail_count})'
                    )
            if 'enable' in str(m.get('command', '')) and 'fail' in status.lower():
                self.get_logger().error(
                    f'enable_robot 失败: {m} —— 请在示教器/wanghuo 清报警并确认远程使能'
                )

        err = self.tcp.take_error()
        if err:
            self.get_logger().warn(f'TCP 异常: {err}')

        lock_suffix = ''
        if self.control_mode == 'movej':
            movej_sent, movej_ack, movej_timeout = self.tcp.get_movej_lock_stats()
            lock_suffix = (
                f' movej_tx={movej_sent} ack={movej_ack} timeout={movej_timeout}'
            )
        self.get_logger().info(
            f'状态: connected={self.tcp.connected} '
            f'cmd_rx={self._cmd_rx_count} sent={self._send_count} '
            f'fb_synced={self._fb_synced} skip={self._last_skip_reason}'
            f'{lock_suffix}'
        )
        if self._cmd_rx_count == 0:
            self.get_logger().warn(
                '尚未收到 /target_robot/joint_commands —— 请确认 IK 已启动且 ROS_DOMAIN_ID=42'
            )
        elif self._send_count == 0 and self._last_skip_reason == 'jump':
            self.get_logger().warn(
                '已收到 joint_commands 但从未下发 —— IK 相对真机偏差过大被拦截；'
                '请确认最新 ik_node，并先 bridge 同步再启 IK'
            )
        elif self._send_count == 0 and self._last_skip_reason == 'nodelta':
            self.get_logger().info(
                'IK 与真机接近，暂无有效步进（松开 Clutch 时正常）。请按住 Clutch 再慢动手臂'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExtonBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node.tcp.connected:
                node.tcp.clear_motion()
                node.tcp.send_control({'command': 'stop_motion'})
                time.sleep(0.2)  # 给发送线程留出发送窗口
        except Exception:
            pass
        node.tcp.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
