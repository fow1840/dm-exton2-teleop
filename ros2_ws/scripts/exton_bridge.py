#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DM-EXton2 ↔ 客户双臂控制器 双向桥接（机械臂通信协议 V0.3）。

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

架构：
  IK  → /target_robot/joint_commands
      → 本节点填 17 维 → TCP movej_canfd/movej
  TCP get_joint_position
      → /client_robot/joint_states（URDF 名，弧度）
      → FK

部署：
  1. 关闭占用 30485 的机械臂图形界面（单连接）
  2. cp scripts/exton_bridge.py ~/tcvg/exton_bridge.py
  3. source /opt/ros/humble/setup.bash && source <ws>/install/setup.bash
  4. export ROS_DOMAIN_ID=42
  5. 先启 bridge 等同步，再启 IK

跟手推荐：
  python3 ~/tcvg/exton_bridge.py --ros-args \\
    -p control_mode:=movej_canfd \\
    -p canfd_freq:=50 \\
    -p auto_enable:=true \\
    -p max_joint_step_rad:=0.20 \\
    -p follow_ratio:=1.0 \\
    -p command_rate_hz:=50.0




    python3 ~/tcvg/exton_bridge.py --ros-args \
  -p robot_ip:=192.168.61.60 -p robot_port:=30485 \
  -p control_mode:=movej_canfd -p command_rate_hz:=25.0 \
  -p follow_ratio:=0.45 -p max_joint_step_rad:=0.06 \
  -p max_joint_jump_rad:=3.0 -p command_eps_rad:=0.01 \
  -p debug_tcp:=true
"""

from __future__ import annotations

import json
import math
import socket
import threading
import time
from typing import Dict, List, Optional, Tuple

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
ALL_URDF_JOINTS = RIGHT_URDF_JOINTS + LEFT_URDF_JOINTS  # 发布顺序：右+左（与常见 IK name 并存均可，按名字匹配）

# 档案右臂软限位（度）→ 运行时转弧度裁剪；左臂默认用 URDF ±π
ARCHIVE_RIGHT_LIMITS_DEG: List[Tuple[float, float]] = [
    (-179.9 + 2.0, 179.9 - 2.0),
    (-148.97 + 2.0, 154.70 - 2.0),
    (-39.53 + 2.0, 129.49 - 2.0),
    (-148.97 + 2.0, 154.70 - 2.0),
    (-89.95 + 2.0, 89.95 - 2.0),
    (-154.70 + 2.0, 154.70 - 2.0),
    (-154.70 + 2.0, 154.70 - 2.0),
]
URDF_LIMITS_RAD: List[Tuple[float, float]] = [(-3.14 + 0.05, 3.14 - 0.05)] * ARM_DOF

# 档案 default_arm_pose（度）→ 仅作右臂缺省参考，真正下发前会被反馈覆盖
DEFAULT_RIGHT_HOME_DEG = [-86.14, -88.6, 98.22, 18.61, 90.0, -13.45, 89.81]


def deg_to_rad(deg: float) -> float:
    return deg * math.pi / 180.0


def clip(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def limits_deg_to_rad(limits_deg: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    return [(deg_to_rad(lo), deg_to_rad(hi)) for lo, hi in limits_deg]


class ProtocolTcpClient:
    """协议 V0.3：JSON + '#' 帧尾，线程安全发送，缓冲接收。"""

    def __init__(self, host: str, port: int, recv_timeout: float = 0.05):
        self.host = host
        self.port = port
        self.recv_timeout = recv_timeout
        self._sock: Optional[socket.socket] = None
        self._send_lock = threading.Lock()
        self._recv_buffer = ''
        self._latest_joint_rad: Optional[List[float]] = None
        self._connected = False

    @property
    def connected(self) -> bool:
        return self._connected and self._sock is not None

    def connect(self) -> None:
        self.close()
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)
        sock.connect((self.host, self.port))
        sock.settimeout(self.recv_timeout)
        self._sock = sock
        self._connected = True
        self._recv_buffer = ''

    def close(self) -> None:
        self._connected = False
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
        self._sock = None
        self._recv_buffer = ''

    def send_command(self, payload: dict) -> None:
        if not self.connected:
            raise ConnectionError('控制器 TCP 未连接')
        # 协议：json 数据域 + 帧尾 #
        line = json.dumps(payload, ensure_ascii=False, separators=(',', ':')) + FRAME_TAIL
        with self._send_lock:
            self._sock.sendall(line.encode('utf-8'))

    def poll_recv(self) -> List[dict]:
        if not self.connected:
            return []

        messages: List[dict] = []
        try:
            while True:
                chunk = self._sock.recv(4096)
                if not chunk:
                    raise ConnectionError('控制器 TCP 连接已关闭')
                self._recv_buffer += chunk.decode('utf-8', errors='ignore')
                while FRAME_TAIL in self._recv_buffer:
                    raw, self._recv_buffer = self._recv_buffer.split(FRAME_TAIL, 1)
                    raw = raw.strip()
                    if not raw:
                        continue
                    try:
                        messages.append(json.loads(raw))
                    except json.JSONDecodeError:
                        continue
        except socket.timeout:
            pass
        except BlockingIOError:
            pass

        for msg in messages:
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
        return messages

    def get_latest_joint_rad(self) -> Optional[List[float]]:
        return self._latest_joint_rad


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
        # 1.0=每帧尽量跟上 IK 目标；<1 会更滞后（回原位跟不回来时常因过小）
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

        self.tcp = ProtocolTcpClient(host, port)
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
        fb_hz = float(self.get_parameter('feedback_rate_hz').value)
        self.create_timer(1.0 / max(cmd_hz, 1.0), self._flush_commands)
        self.create_timer(1.0 / max(pub_hz, 1.0), self._publish_joint_states)
        self.create_timer(1.0 / max(fb_hz, 1.0), self._poll_and_query)
        self.create_timer(1.0, self._ensure_connected)
        self.create_timer(2.0, self._status_watchdog)

        self._connect_controller()
        self.get_logger().info(
            f'exton_bridge V0.3 | {host}:{port} | mode={self.control_mode} | '
            f'v={self.move_speed} freq={self.canfd_freq} | '
            f'L→{LEFT_URDF_JOINTS[0]}.. R→{RIGHT_URDF_JOINTS[0]}.. | '
            f'debug_tcp={self.debug_tcp}'
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
                f'已连接控制器 {self.tcp.host}:{self.tcp.port}（协议 V0.3，帧尾 #）'
            )
            if self.auto_enable:
                self._send_raw({'command': 'enable_robot'})
                self.get_logger().warn('已发送 enable_robot（请确认臂处于安全姿态）')
            self._query_joint_position()
        except OSError as exc:
            self.get_logger().error(f'连接控制器失败: {exc}（请确认已关闭 30485 图形界面）')

    def _ensure_connected(self) -> None:
        if self.tcp.connected or not self.auto_reconnect:
            return
        self.get_logger().warn('尝试重连控制器 ...')
        self._connect_controller()

    def _send_raw(self, payload: dict) -> None:
        try:
            self.tcp.send_command(payload)
            cmd = payload.get('command')
            msgs = self.tcp.poll_recv()
            # 仅首次用反馈同步；同步前禁止运动下发，避免把错误初值（如 ±π）发给真机
            if not self._fb_synced:
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
            if self.debug_tcp and cmd != 'get_joint_position':
                self.get_logger().info(f'TCP>> {cmd} keys={list(payload.keys())}')
                if msgs:
                    self.get_logger().info(f'TCP<< {msgs[:2]}')
            if cmd == 'enable_robot':
                for m in msgs:
                    if isinstance(m, dict) and 'fail' in str(m.get('status', '')).lower():
                        self.get_logger().error(
                            f'enable_robot 失败: {m} —— 请在示教器/wanghuo 清报警并确认远程使能'
                        )
        except OSError as exc:
            self.get_logger().error(f'发送失败: {exc}')
            self.tcp.close()

    def _query_joint_position(self) -> None:
        if not self.tcp.connected:
            return
        self._send_raw({'command': 'get_joint_position'})

    def _poll_and_query(self) -> None:
        if not self.tcp.connected:
            return
        try:
            self.tcp.poll_recv()
            self._query_joint_position()
        except OSError as exc:
            self.get_logger().warn(f'反馈异常: {exc}')
            self.tcp.close()

    def _status_watchdog(self) -> None:
        self.get_logger().info(
            f'状态: connected={self.tcp.connected} '
            f'cmd_rx={self._cmd_rx_count} sent={self._send_count} '
            f'fb_synced={self._fb_synced} skip={self._last_skip_reason}'
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

    # ------------------------------------------------------------------
    # 控制链：IK → 17 维 → TCP
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
                # 仅极端错误初值(接近 ±π 量级)才丢弃；普通大偏差仍限速追，避免反向时“失灵”
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
        self._send_motion(joints)
        self._last_sent_joint = list(joints)
        if self.debug_tcp and (self._send_count <= 5 or self._send_count % 25 == 0):
            self.get_logger().info(
                f'匹配左{hit_left}/右{hit_right}轴后下发（跳过{skipped}） step={max_step_abs:.4f}'
            )

    def _send_motion(self, joints: List[float]) -> None:
        if self.control_mode == 'movej_canfd':
            payload = {
                'command': 'movej_canfd',
                'joint': joints,
                'freq': self.canfd_freq,
            }
        else:
            payload = {
                'command': 'movej',
                'joint': joints,
                'v': self.move_speed,
            }
        try:
            self.tcp.send_command(payload)
            msgs = self.tcp.poll_recv()
            self._send_count += 1
            failed = any(
                isinstance(m, dict) and str(m.get('status', '')).endswith('failed')
                for m in msgs
            )
            if failed:
                self._fail_count += 1
                if self._fail_count <= 5 or self._fail_count % 20 == 0:
                    self.get_logger().error(
                        f'控制器拒绝运动 program_run_failed (累计{self._fail_count}) '
                        f'R0={joints[IDX_RIGHT_START]} L0={joints[IDX_LEFT_START]} resp={msgs[:1]}'
                    )
            elif self.debug_tcp and (self._send_count <= 10 or self._send_count % 10 == 0):
                self.get_logger().info(
                    f'已下发#{self._send_count} {self.control_mode} '
                    f'R0={joints[IDX_RIGHT_START]:.3f} L0={joints[IDX_LEFT_START]:.3f} '
                    f'resp={msgs[:1]}'
                )
        except OSError as exc:
            self.get_logger().error(f'下发运动失败: {exc}')
            self.tcp.close()

    # ------------------------------------------------------------------
    # 反馈链：get_joint_position → /client_robot/joint_states
    # ------------------------------------------------------------------
    def _publish_joint_states(self) -> None:
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
                node.tcp.send_command({'command': 'stop_motion'})
        except Exception:
            pass
        node.tcp.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
