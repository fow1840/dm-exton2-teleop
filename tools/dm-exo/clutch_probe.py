#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
离合触发录包器：平时环形缓冲（60s），离合按下→松开后自动落盘事件窗口。
每次按下-松开存 bags/pressNN_<side>_<时刻>.npz，并打印即时诊断。

用法：
  python3 clutch_probe.py            # 持续运行，Ctrl-C 结束（未落盘事件补存）
  python3 clutch_probe.py 300        # 跑 300 秒自动结束

离线深度分析（窗口含全部话题，格式与 record_compare.py 相同）：
  python3 analyze_compare.py bags/press01_right_xxx.npz

诊断判据（即时打印）：
  - 目标流断档：Puppetry 松开停发，按下时断档时长
  - S→N 跳变：断档前旧目标 vs 恢复后新目标的位置/姿态差
  - IK瞬跳：按下后 0.5s 内该臂关节最大变化
  - 手动：按下前 1s / 按住期间外骨骼关节变化
  - 真机位移：按下后 2s 内该臂关节变化
"""
import os
import sys
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

RING_S = 60.0   # 环形缓冲时长（按下前最多回溯 60s）
PRE_S = 5.0     # 事件窗口：按下前保留
POST_S = 3.0    # 事件窗口：松开后保留
OUTDIR = os.path.expanduser('~/桌面/dm-exo/bags')

JS_TOPICS = {
    '/exoskeleton/joint_states': '/exo',
    '/target_robot/joint_commands': '/ik',
    '/client_robot/joint_states': '/rob',
}
# 臂 → 各数组列范围（与 analyze_compare.py 一致）
ARM = {
    'left':  {'exo': (0, 6),  'jnt': (7, 14), 'jn': [f'{i}L' for i in range(1, 8)]},
    'right': {'exo': (6, 12), 'jnt': (0, 7),  'jn': [f'{i}R' for i in range(1, 8)]},
}


def quat_angle_deg(q1, q2):
    """两四元数(xyzw)夹角(度)。"""
    d = abs(float(np.dot(q1, q2)))
    return float(np.degrees(2 * np.arccos(min(1.0, d))))


class Probe(Node):
    def __init__(self):
        super().__init__('clutch_probe')
        self.t0 = None
        self.buf = {}      # key -> (deque_t, deque_v)
        self.names = {}    # key -> msg.name（存一次）
        self.clutch = {'left': False, 'right': False}
        self.pending = []  # {'side','t_press','t_rel','done'}
        self.nev = 0
        os.makedirs(OUTDIR, exist_ok=True)

        for topic, key in JS_TOPICS.items():
            self.create_subscription(
                JointState, topic, lambda m, k=key: self._js(k, m), 30)
        for side in ('left', 'right'):
            self.create_subscription(
                PoseStamped, f'/target_robot/{side}_ee/target',
                lambda m, s=side: self._pose(f'/ee_{s}_tgt', m), 30)
            self.create_subscription(
                PoseStamped, f'/target_robot/{side}_ee/state',
                lambda m, s=side: self._pose(f'/ee_{s}_state', m), 30)
            self.create_subscription(
                Bool, f'/clutch/{side}', lambda m, s=side: self._clutch(m, s), 30)
        self.get_logger().info(f'离合触发录包器就绪 → {OUTDIR}（环形缓冲{RING_S:.0f}s，'
                               f'窗口 按下前{PRE_S:.0f}s~松开后{POST_S:.0f}s）')

    # ---------- 缓冲 ----------
    def _now(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        return now - self.t0

    def _push(self, key, t, v):
        d = self.buf.setdefault(key, (deque(), deque()))
        d[0].append(t)
        d[1].append(v)
        if len(d[0]) > 4000 and d[0][0] < t - RING_S:
            while d[0] and d[0][0] < t - RING_S:
                d[0].popleft()
                d[1].popleft()

    def _js(self, key, m):
        if key not in self.names and m.name:
            self.names[key] = list(m.name)
        self._push(key, self._now(), list(m.position))

    def _pose(self, key, m):
        p, q = m.pose.position, m.pose.orientation
        self._push(key, self._now(), [p.x, p.y, p.z, q.x, q.y, q.z, q.w])

    def _bool(self, key, m):
        self._push(key, self._now(), [1.0 if m.data else 0.0])

    # ---------- 离合事件 ----------
    def _clutch(self, m, side):
        t = self._now()
        val = bool(m.data)
        self._push(f'/clutch_{side}', t, [1.0 if val else 0.0])
        if val and not self.clutch[side]:
            self.pending.append({'side': side, 't_press': t, 't_rel': None, 'done': False})
            self.get_logger().info(f'⏺ {side} 按下 t={t:.2f}s（第{len(self.pending)}次）')
        elif not val and self.clutch[side]:
            for ev in reversed(self.pending):
                if ev['side'] == side and ev['t_rel'] is None:
                    ev['t_rel'] = t
            self.get_logger().info(f'⏹ {side} 松开 t={t:.2f}s，{POST_S:.0f}s后落盘')
        self.clutch[side] = val

    def dump_ready(self):
        now = self._now()
        for ev in self.pending:
            if (not ev['done'] and ev['t_rel'] is not None
                    and now >= ev['t_rel'] + POST_S):
                self._save(ev)

    def _slice(self, lo, hi):
        out = {}
        for key, (ts, vs) in self.buf.items():
            tt, vv = [], []
            for a, b in zip(ts, vs):
                if lo <= a <= hi:
                    tt.append(a)
                    vv.append(b)
            if tt:
                out[f'{key}_t'] = np.array(tt)
                out[f'{key}_v'] = np.array(vv)
        for key, names in self.names.items():
            if f'{key}_t' in out:
                out[f'{key}_names'] = np.array(names)
        return out

    def _save(self, ev):
        ev['done'] = True
        self.nev += 1
        lo = max(0.0, ev['t_press'] - PRE_S)
        hi = ev['t_rel'] + POST_S
        data = self._slice(lo, hi)
        name = f"press{self.nev:02d}_{ev['side']}_{time.strftime('%H%M%S')}.npz"
        path = os.path.join(OUTDIR, name)
        np.savez_compressed(path, **data)
        self.get_logger().info(f'💾 {path}（窗口{lo:.1f}~{hi:.1f}s, {len(data)}通道）')
        self.verdict(ev, path)

    # ---------- 即时诊断 ----------
    def verdict(self, ev, path):
        side, t_p, t_r = ev['side'], ev['t_press'], ev['t_rel']
        z = np.load(path)
        ec0, ec1 = ARM[side]['exo']
        j0, j1 = ARM[side]['jnt']
        jn = ARM[side]['jn']
        lines = [f'== 诊断 {os.path.basename(path)}（{side}，保持{t_r-t_p:.1f}s） ==']

        # 1. 目标流断档 + S→N 跳变
        kt = f'/ee_{side}_tgt_t'
        if kt in z.files and len(z[kt]) >= 2:
            tt, tv = z[kt], z[kt[:-2] + '_v']
            i = int(np.searchsorted(tt, t_p))
            if 0 < i < len(tt):
                gap = (t_p - tt[i - 1]) * 1000
                S, N = tv[i - 1], tv[i]
                dp = float(np.linalg.norm(N[:3] - S[:3])) * 1000
                da = quat_angle_deg(S[3:], N[3:])
                lines.append(f'目标流断档 {gap:.0f}ms | S→N: 位置{dp:.1f}mm 姿态{da:.1f}°')
                if gap > 200 and (dp > 5 or da > 2):
                    lines.append('  ⚠️ 断档+目标跳变（若仍瞬跳，查滤波重置是否生效）')
                elif gap > 200:
                    lines.append('  ✅ 断档但目标连续（重锚定干净）')
            else:
                lines.append('目标流: 窗口内无按下前数据')
        # 2. 手动
        kt = '/exo_t'
        if kt in z.files:
            exo = z['/exo_v']
            m = (z[kt] >= t_p - 1.0) & (z[kt] < t_p)
            if m.sum() > 10:
                pre = (exo[m][:, ec0:ec1].max(0) - exo[m][:, ec0:ec1].min(0)).max()
                m2 = (z[kt] >= t_p) & (z[kt] <= t_r)
                hold = (exo[m2][:, ec0:ec1].max(0) - exo[m2][:, ec0:ec1].min(0)).max()
                lines.append(f'手动: 按下前1s {pre*1000:.0f}mrad | 按住期间 {hold*1000:.0f}mrad')
        # 3. IK 瞬跳 + 真机位移
        if '/ik_t' in z.files:
            ik = z['/ik_v']
            m = (z['/ik_t'] >= t_p) & (z['/ik_t'] < t_p + 0.5)
            if m.sum() > 10:
                d = np.abs(ik[m][:, j0:j1] - ik[m][0][j0:j1]).max(axis=0)
                j = int(np.argmax(d))
                lines.append(f'IK瞬跳(按下后0.5s): {d[j]*1000:.0f}mrad(J{jn[j]})'
                             + ('  ⚠️ ' if d[j] > 0.05 else '  ✅'))
        if '/rob_t' in z.files:
            rob = z['/rob_v']
            m = (z['/rob_t'] >= t_p) & (z['/rob_t'] < t_p + 2.0)
            if m.sum() > 10:
                d = np.abs(rob[m][:, j0:j1] - rob[m][0][j0:j1]).max(axis=0)
                lines.append(f'真机位移(按下后2s): {d.max()*1000:.0f}mrad')
        # 4. 按住中手静窗口的 IK 漂移（姿态回拉复查）
        if '/ik_t' in z.files and '/exo_t' in z.files:
            w = t_p + 1.0
            worst = 0.0
            while w < t_r - 1.0:
                mh = (z['/exo_t'] >= w) & (z['/exo_t'] < w + 0.5)
                if mh.sum() > 50:
                    hand = (z['/exo_v'][mh][:, ec0:ec1].max(0)
                            - z['/exo_v'][mh][:, ec0:ec1].min(0)).max()
                    if hand < 0.015:
                        mi = (z['/ik_t'] >= w) & (z['/ik_t'] < w + 0.5)
                        if mi.sum() > 20:
                            worst = max(worst, float(np.abs(
                                z['/ik_v'][mi][:, j0:j1]
                                - z['/ik_v'][mi][0][j0:j1]).max()))
                w += 0.5
            if worst > 0:
                lines.append(f'按住中手静窗口 IK漂移: 峰值{worst*1000:.1f}mrad'
                             + ('  ⚠️' if worst > 0.02 else '  ✅'))
        for ln in lines:
            self.get_logger().info(ln)


def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
    rclpy.init()
    node = Probe()
    t_start = time.monotonic()
    try:
        while dur <= 0 or time.monotonic() - t_start < dur:
            rclpy.spin_once(node, timeout_sec=0.1)
            node.dump_ready()
    except KeyboardInterrupt:
        print('\n手动停止', flush=True)
    # 补存未落盘事件
    now = node._now()
    for ev in node.pending:
        if not ev['done']:
            if ev['t_rel'] is None:
                ev['t_rel'] = now
            node._save(ev)
    node.get_logger().info(f'结束，共{node.nev}个事件包在 {OUTDIR}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
