#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
遥操作链路三层信号录制器（诊断延迟/方向用）。

录制话题（接收时刻统一用本机时钟，保证可比）：
  1. /exoskeleton/joint_states      你的手臂（外骨骼 12 关节）
  2. /target_robot/joint_commands   IK 目标（14 关节）
  3. /client_robot/joint_states     真机反馈（14 关节）
  4. /ee_poses                      你的手臂末端位姿（双臂 2×7）
  5. /target_robot/{left,right}_ee/target   IK 末端目标
  6. /target_robot/{left,right}_ee/state    末端实际（FK 由真机反馈算出）
  7. /clutch/{left,right}           离合开关（诊断"按下就动"用）

用法：
  python3 record_compare.py [秒数=60] [输出前缀=rec]
输出：~/桌面/dm-exo/<前缀>.npz
"""
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray


class Recorder(Node):
    def __init__(self, duration: float, prefix: str):
        super().__init__('record_compare')
        self.duration = duration
        self.prefix = prefix
        self.t0 = None
        self.data = {}      # topic -> dict(t=[], v=[])
        self.counts = {}

        self.create_subscription(JointState, '/exoskeleton/joint_states',
                                 lambda m: self._js('/exo', m), 20)
        self.create_subscription(JointState, '/target_robot/joint_commands',
                                 lambda m: self._js('/ik', m), 30)
        self.create_subscription(JointState, '/client_robot/joint_states',
                                 lambda m: self._js('/rob', m), 30)
        self.create_subscription(Float32MultiArray, '/ee_poses',
                                 lambda m: self._arr('/ee_op', np.array(m.data, dtype=float)), 20)
        for side in ('left', 'right'):
            self.create_subscription(PoseStamped, f'/target_robot/{side}_ee/target',
                                     lambda m, s=side: self._pose(f'/ee_{s}_tgt', m), 20)
            self.create_subscription(PoseStamped, f'/target_robot/{side}_ee/state',
                                     lambda m, s=side: self._pose(f'/ee_{s}_state', m), 20)
            self.create_subscription(Bool, f'/clutch/{side}',
                                     lambda m, s=side: self._bool(f'/clutch_{s}', m), 1)

    def _t(self) -> float:
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        return now - self.t0

    def _js(self, key: str, msg: JointState) -> None:
        t = self._t()
        d = self.data.setdefault(key, {'t': [], 'v': [], 'names': None})
        d['t'].append(t)
        d['v'].append(list(msg.position))
        d['names'] = list(msg.name)
        self.counts[key] = self.counts.get(key, 0) + 1

    def _arr(self, key: str, vals: np.ndarray) -> None:
        t = self._t()
        d = self.data.setdefault(key, {'t': [], 'v': [], 'names': None})
        d['t'].append(t)
        d['v'].append(list(vals))
        self.counts[key] = self.counts.get(key, 0) + 1

    def _pose(self, key: str, msg: PoseStamped) -> None:
        p, q = msg.pose.position, msg.pose.orientation
        self._arr(key, np.array([p.x, p.y, p.z, q.x, q.y, q.z, q.w]))

    def _bool(self, key: str, msg: Bool) -> None:
        t = self._t()
        d = self.data.setdefault(key, {'t': [], 'v': [], 'names': None})
        d['t'].append(t)
        d['v'].append([1.0 if msg.data else 0.0])
        self.counts[key] = self.counts.get(key, 0) + 1

    def save(self) -> str:
        out = {}
        for key, d in self.data.items():
            out[f'{key}_t'] = np.array(d['t'])
            out[f'{key}_v'] = np.array(d['v'])
            if d['names']:
                out[f'{key}_names'] = np.array(d['names'])
        path = f'/home/tc/桌面/dm-exo/{self.prefix}.npz'
        np.savez_compressed(path, **out)
        return path

    def print_progress(self, elapsed: float) -> None:
        rate = {k: (c / elapsed if elapsed > 0 else 0) for k, c in self.counts.items()}
        self.get_logger().info(
            f'{elapsed:5.1f}s | ' +
            ' '.join(f"{k}:{rate.get(k, 0):.0f}Hz" for k in ('/exo', '/ik', '/rob', '/ee_op')))


def main() -> None:
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    prefix = sys.argv[2] if len(sys.argv) > 2 else 'rec'
    rclpy.init()
    node = Recorder(duration, prefix)
    print(f'开始录制 {duration:.0f}s ...', flush=True)
    t_start = time.monotonic()
    try:
        while time.monotonic() - t_start < duration:
            rclpy.spin_once(node, timeout_sec=0.2)
            elapsed = time.monotonic() - t_start
            if int(elapsed) % 10 == 0 and abs(elapsed - round(elapsed)) < 0.25:
                node.print_progress(round(elapsed))
                time.sleep(0.3)  # 避免同秒重复打印
    except KeyboardInterrupt:
        print('\n提前停止', flush=True)
    path = node.save()
    node.print_progress(time.monotonic() - t_start)
    print(f'已保存: {path}', flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
