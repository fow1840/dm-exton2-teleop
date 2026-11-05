#!/usr/bin/env python3
"""假控制器：协议 V0.3 TCP 仿真服务端（离线测试 exton_bridge 用，不连真机）。

行为模型（可配置）：
  - get_joint_position → 立即回当前 17 维关节角
  - movej → 若上一条还没"执行完"且 reject_overlap=True，回 program_run_failed；
    否则接受：关节立即/延迟到位（带可选过冲 overshoot_rad 或下垂 sag_rad）
  - ack_mode：
      'accept'    收到 movej 立即回 status（受理即回——lock-step 退化为 ~0 间隔）
      'complete'  执行 exec_time_s 后才回 status（lock-step = 按运动周期串行）
镜像真机两大未验证项（R2/ack 语义、单条 movej 过冲），用于离线刻画桥的行为。
"""
import json
import socket
import socketserver
import threading
import time

HOLD = -100.0
N_JOINTS = 17


class FakeControllerHandler(socketserver.BaseRequestHandler):
    def handle(self):
        srv = self.server
        srv._set_sock(self.request)
        buf = ''
        self.request.settimeout(0.2)
        while not srv.closing:
            try:
                data = self.request.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            if not data:
                break
            buf += data.decode('utf-8', errors='ignore')
            while '#' in buf:
                raw, buf = buf.split('#', 1)
                raw = raw.strip()
                if not raw:
                    continue
                try:
                    msg = json.loads(raw)
                except json.JSONDecodeError:
                    continue
                for reply in srv.dispatch(msg):
                    if not srv._send(reply):
                        return


class FakeController(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, port=0, *, exec_time_s=0.0, overshoot_rad=0.0,
                 ack_mode='accept', reject_overlap=True, sag_rad=0.0):
        super().__init__(('127.0.0.1', port), FakeControllerHandler)
        self.port = self.server_address[1]
        self.exec_time_s = exec_time_s
        self.overshoot_rad = overshoot_rad
        self.ack_mode = ack_mode
        self.reject_overlap = reject_overlap
        self.sag_rad = sag_rad  # UBT 下垂模型：到位后恒降 sag（重力下垂对照）
        self.closing = False
        self._lock = threading.Lock()
        self._sock = None
        self._joints = [HOLD] * N_JOINTS
        self._busy_until = 0.0
        self._overshoot_sign = 1.0
        self._pending_replies = []
        self.movej_accepted = 0
        self.movej_rejected = 0
        self.query_count = 0
        self.movej_targets = []  # 收到的 17 维目标序列（事后断言用）

    # ---- 连接与发送 ----
    def _set_sock(self, sock):
        with self._lock:
            self._sock = sock

    def _send(self, reply):
        with self._lock:
            sock = self._sock
        if sock is None:
            return False
        try:
            sock.sendall((json.dumps(reply, ensure_ascii=False) + '#').encode('utf-8'))
            return True
        except OSError:
            return False

    # ---- 状态操作（测试直接驱动） ----
    def set_arm_state(self, left, right):
        with self._lock:
            self._joints[1:8] = list(left)
            self._joints[8:15] = list(right)

    def get_arm_state(self):
        with self._lock:
            return list(self._joints[1:8]), list(self._joints[8:15])

    def stop(self):
        self.closing = True
        self.server_close()

    # ---- 协议分发 ----
    def dispatch(self, msg):
        out = []
        with self._lock:
            pending, self._pending_replies = self._pending_replies, []
        for r in pending:
            out.append(r)
        cmd = msg.get('command')
        if cmd == 'get_joint_position':
            with self._lock:
                self.query_count += 1
            out.append({'command': 'get_joint_position', 'joint': list(self._joints)})
        elif cmd == 'movej':
            out.extend(self._handle_movej(msg))
        return out

    def _handle_movej(self, msg):
        target = list(msg.get('joint') or [])
        now = time.monotonic()
        with self._lock:
            if now < self._busy_until and self.reject_overlap:
                self.movej_rejected += 1
                return [{'command': 'movej', 'status': 'program_run_failed'}]
            self.movej_accepted += 1
            self.movej_targets.append(target)
            self._busy_until = now + self.exec_time_s
            newjoints = list(self._joints)
            moved = False
            for i, v in enumerate(target):
                if abs(v - HOLD) < 1e-9:
                    continue
                delta = v - self._joints[i]
                if abs(delta) > 1e-9:
                    moved = True
                    if self.overshoot_rad:
                        self._overshoot_sign = -self._overshoot_sign
                        newjoints[i] = v + self._overshoot_sign * self.overshoot_rad
                    elif self.sag_rad:
                        newjoints[i] = v - self.sag_rad
                    else:
                        newjoints[i] = v
            self._joints = newjoints
            need_delay = self.ack_mode == 'complete' and self.exec_time_s > 0
        if moved and need_delay:
            # 执行完成才回 status：用线程模拟（不阻塞查询线程，贴近真机）
            timer = threading.Timer(self.exec_time_s, self._send,
                                    args=({'command': 'movej', 'status': 'program_ok'},))
            timer.daemon = True
            timer.start()
            return []
        return [{'command': 'movej', 'status': 'program_ok'}]
