#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""全流程只读监视器（T21）：回答「遥操为什么突然不动」。

只订阅、不发布、不碰控制路径。每 2s 打一行，同时增量收集 T3 日志里的
拒绝/状态行。关键判据不是「有没有消息」，而是**每条链路距上一帧多久**
（age），因为 bridge 停发反馈后话题仍然存在、旧值会一直看着像正常。

用法：
  python3 chain_monitor.py [输出前缀=chain] [最长秒数=0 表示不限]
输出：
  ~/桌面/dm-exo/<前缀>.csv     每 2s 一行指标，供离线对账
  ~/桌面/dm-exo/<前缀>.log     事件流水（离合跳变、拒绝新增、反馈断流）
"""
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped

ARM_L = ['Joint1L', 'Joint2L', 'Joint3L', 'Joint4L', 'Joint5L', 'Joint6L', 'Joint7L']
ARM_R = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6', 'Joint7']

OUT_DIR = Path.home() / '桌面/dm-exo'
T3_LOG = OUT_DIR / 'logs/T3_bridge.log'


class Monitor(Node):
    def __init__(self, prefix: str):
        super().__init__('chain_monitor')
        self.prefix = prefix
        self.t0 = time.monotonic()
        self.last = {}        # 话题 -> (单调时刻, 向量)
        self.count = {}       # 话题 -> 累计帧数
        self.names = {}       # 话题 -> 关节名顺序

        self.create_subscription(JointState, '/exoskeleton/joint_states',
                                 lambda m: self._js('exo', m), 20)
        self.create_subscription(JointState, '/target_robot/joint_commands',
                                 lambda m: self._js('ik', m), 30)
        self.create_subscription(JointState, '/client_robot/joint_states',
                                 lambda m: self._js('rob', m), 30)
        for side in ('left', 'right'):
            self.create_subscription(PoseStamped, f'/target_robot/{side}_ee/target',
                                     lambda m, s=side: self._pose(f'tgt_{s}', m), 20)
            self.create_subscription(PoseStamped, f'/target_robot/{side}_ee/state',
                                     lambda m, s=side: self._pose(f'st_{s}', m), 20)
            self.create_subscription(Bool, f'/clutch/{side}',
                                     lambda m, s=side: self._clutch(s, m), 1)

        # T3 日志增量读取位置
        self._log_off = T3_LOG.stat().st_size if T3_LOG.exists() else 0
        self._reject_total = 0
        self._reject_seen = 0
        self._t3_age = None

        self.csv = open(OUT_DIR / f'{prefix}.csv', 'w', buffering=1)
        self.ev = open(OUT_DIR / f'{prefix}.log', 'w', buffering=1)
        self.csv.write('t,exo_hz,ik_hz,rob_hz,exo_age,ik_age,rob_age,'
                       'clutch_l,clutch_r,cmd_err_l,cmd_err_r,reject_new,reject_total,'
                       'rob_frozen_s,t3_fb_age,t3_sent,t3_skip,t3_ik_moved\n')
        # 反馈数值冻结计时：bridge 断线后仍以 50Hz 重发缓存快照，
        # 所以「有消息」不等于「电机反馈新鲜」，必须看数值是否在变。
        self._rob_prev = None
        self._rob_frozen_since = None
        self._rob_frozen_s = 0.0
        self._t3 = {'fb_age': float('nan'), 'sent': 0, 'skip': '?', 'ik_moved': 0,
                    'reject_win': 0, 'max_err': float('nan')}
        self.ev.write(f'# start {time.strftime("%F %T")} prefix={prefix}\n')
        self.clutch = {'left': None, 'right': None}
        self._win = {}
        self._t_win = time.monotonic()
        self._last_reject_print = 0.0

    # ---------------- 订阅回调 ----------------
    def _js(self, key, msg):
        vals = list(msg.position)
        if key == 'rob':
            now = time.monotonic()
            if self._rob_prev is not None and vals == self._rob_prev:
                if self._rob_frozen_since is None:
                    self._rob_frozen_since = now
                self._rob_frozen_s = now - self._rob_frozen_since
            else:
                self._rob_frozen_since = None
                self._rob_frozen_s = 0.0
            self._rob_prev = vals
        self.last[key] = (time.monotonic(), vals)
        self.names[key] = list(msg.name)
        self.count[key] = self.count.get(key, 0) + 1

    def _pose(self, key, msg):
        p, q = msg.pose.position, msg.pose.orientation
        self.last[key] = (time.monotonic(), [p.x, p.y, p.z, q.x, q.y, q.z, q.w])
        self.count[key] = self.count.get(key, 0) + 1

    def _clutch(self, side, msg):
        val = bool(msg.data)
        self.count[f'clutch_{side}'] = self.count.get(f'clutch_{side}', 0) + 1
        if self.clutch[side] != val:
            self.clutch[side] = val
            self.ev.write(f'{self._ts()} CLUTCH {side}={int(val)}\n')
        self.last[f'clutch_{side}'] = (time.monotonic(), [1.0 if val else 0.0])

    # ---------------- 工具 ----------------
    def _ts(self):
        return f'{time.monotonic() - self.t0:8.3f}s {time.strftime("%H:%M:%S")}'

    def _age(self, key):
        if key not in self.last:
            return float('inf')
        return time.monotonic() - self.last[key][0]

    def _cmd_err(self, arm):
        """IK 指令与真机反馈的同名关节最大绝对差（rad）。"""
        if 'ik' not in self.last or 'rob' not in self.last:
            return float('nan')
        ik_names, ik_v = self.names['ik'], self.last['ik'][1]
        rb_names, rb_v = self.names['rob'], self.last['rob'][1]
        diffs = [abs(ik_v[i] - rb_v[rb_names.index(n)])
                 for i, n in enumerate(ik_names) if n in arm and n in rb_names]
        return max(diffs) if diffs else float('nan')

    def poll_t3(self):
        """增量读取 T3 日志：统计拒绝、转发状态行、记录日志静默时长。"""
        if not T3_LOG.exists():
            return
        size = T3_LOG.stat().st_size
        if size < self._log_off:      # 文件被轮转/重写
            self._log_off = 0
        if size == self._log_off:
            return
        self._t3_age = 0.0
        with T3_LOG.open('r', errors='replace') as f:
            f.seek(self._log_off)
            chunk = f.read()
            self._log_off = f.tell()
        now = time.monotonic()
        for line in chunk.splitlines():
            if 'program_run_failed' in line:
                self._reject_total += 1
                if now - self._last_reject_print > 5.0:
                    self.ev.write(f'{self._ts()} T3 {line.strip()}\n')
                    self._last_reject_print = now
            elif '状态:' in line:
                self.ev.write(f'{self._ts()} T3 {line.strip()}\n')
                self._parse_status(line)

    def _parse_status(self, line):
        """从 bridge 状态行取 fb_age/sent/skip/ik_moved —— bridge 自己怎么看反馈。"""
        import re
        m = re.search(r'skip=(\w+)', line)
        if m:
            self._t3['skip'] = m.group(1)
        m = re.search(r'fb_age=([\d.]+)s', line)
        if m:
            self._t3['fb_age'] = float(m.group(1))
        m = re.search(r'sent=(\d+)', line)
        if m:
            self._t3['sent'] = int(m.group(1))
        m = re.search(r'ik_moved=(\d+)', line)
        if m:
            self._t3['ik_moved'] = int(m.group(1))

    def tick(self):
        self.poll_t3()
        now = time.monotonic()
        hz = {}
        for k in ('exo', 'ik', 'rob'):
            prev = self._win.get(k, self.count.get(k, 0))
            hz[k] = (self.count.get(k, 0) - prev) / 2.0
        self._win = {k: self.count.get(k, 0) for k in ('exo', 'ik', 'rob')}

        ages = {k: self._age(k) for k in ('exo', 'ik', 'rob')}
        el = self._cmd_err(ARM_L)
        er = self._cmd_err(ARM_R)
        new_rej = self._reject_total - self._reject_seen
        self._reject_seen = self._reject_total

        self.csv.write(
            f'{now - self.t0:.3f},{hz["exo"]:.1f},{hz["ik"]:.1f},{hz["rob"]:.1f},'
            f'{min(ages["exo"], 999):.3f},{min(ages["ik"], 999):.3f},{min(ages["rob"], 999):.3f},'
            f'{int(bool(self.clutch["left"]))},{int(bool(self.clutch["right"]))},'
            f'{el:.4f},{er:.4f},{new_rej},{self._reject_total},'
            f'{self._rob_frozen_s:.2f},{self._t3["fb_age"]:.2f},{self._t3["sent"]},'
            f'{self._t3["skip"]},{self._t3["ik_moved"]}\n')

        self.get_logger().info(
            f'flow t={now - self.t0:7.1f}s exo={hz["exo"]:.0f}Hz/{ages["exo"]:.2f}s '
            f'ik={hz["ik"]:.0f}Hz/{ages["ik"]:.2f}s rob={hz["rob"]:.0f}Hz/{ages["rob"]:.2f}s '
            f'frozen={self._rob_frozen_s:.1f}s bridge_fb_age={self._t3["fb_age"]:.2f}s '
            f'skip={self._t3["skip"]} sent={self._t3["sent"]} '
            f'clutch={int(bool(self.clutch["left"]))}{int(bool(self.clutch["right"]))} '
            f'errL={el:.3f} errR={er:.3f} reject+{new_rej}({self._reject_total})')

        # 事件：反馈/指令断流（阈值 1.5s，明显大于正常 50Hz/250Hz 间隔）
        for key, th in (('rob', 1.5), ('ik', 1.5), ('exo', 1.5)):
            age = ages[key]
            if age > th and not getattr(self, f'_gap_{key}', False):
                setattr(self, f'_gap_{key}', True)
                self.ev.write(f'{self._ts()} GAP {key} 无消息 {age:.2f}s\n')
            elif age <= th and getattr(self, f'_gap_{key}', False):
                setattr(self, f'_gap_{key}', False)
                self.ev.write(f'{self._ts()} RESUME {key}\n')
        if new_rej:
            self.ev.write(f'{self._ts()} REJECT +{new_rej} 累计{self._reject_total}\n')

        # 事件：bridge 自家 fb_age 变大 = 控制器不再回反馈（最贴近「手臂失联」的判据）
        if self._t3['fb_age'] > 1.5 and not getattr(self, '_gap_fb', False):
            self._gap_fb = True
            self.ev.write(f"{self._ts()} STALE 控制器反馈停更 fb_age={self._t3['fb_age']:.1f}s "
                          f"skip={self._t3['skip']}\n")
        elif self._t3['fb_age'] <= 1.5 and getattr(self, '_gap_fb', False):
            self._gap_fb = False
            self.ev.write(f'{self._ts()} FRESH 控制器反馈恢复 fb_age={self._t3["fb_age"]:.2f}s\n')
        if self._rob_frozen_s > 2.0 and not getattr(self, '_gap_frozen', False):
            self._gap_frozen = True
            self.ev.write(f'{self._ts()} FROZEN 关节反馈数值冻结 {self._rob_frozen_s:.1f}s\n')
        elif self._rob_frozen_s <= 0.5 and getattr(self, '_gap_frozen', False):
            self._gap_frozen = False
            self.ev.write(f'{self._ts()} UNFROZEN 关节反馈数值恢复变化\n')

    def close(self):
        self.csv.close()
        self.ev.close()


def main():
    prefix = sys.argv[1] if len(sys.argv) > 1 else 'chain'
    limit = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    rclpy.init()
    node = Monitor(prefix)
    print(f'监视开始 prefix={prefix} limit={limit or "inf"}', flush=True)
    start = time.monotonic()
    try:
        while rclpy.ok() and (limit <= 0 or time.monotonic() - start < limit):
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() - node._t_win >= 2.0:
                node._t_win = time.monotonic()
                node.tick()
    except KeyboardInterrupt:
        print('手动停止', flush=True)
    node.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
