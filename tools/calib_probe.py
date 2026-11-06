#!/usr/bin/env python3
"""姿态标定采样：给定标签，采 N 秒，打印各源的均值。
用法: python3 calib_probe.py <标签> [秒数]
数据源：外骨骼关节/末端位姿、机器人关节、IK 目标位姿。
"""
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

LABEL = sys.argv[1] if len(sys.argv) > 1 else 'pose'
DUR = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0


class Sampler(Node):
    def __init__(self):
        super().__init__('calib_sampler')
        self.buf = {'exo': [], 'ee': [], 'rob': [], 'tgt_l': [], 'st_l': []}
        self.last = {}
        self.create_subscription(JointState, '/exoskeleton/joint_states',
                                 lambda m: self._js('exo', m), 30)
        self.create_subscription(JointState, '/client_robot/joint_states',
                                 lambda m: self._js('rob', m), 30)
        self.create_subscription(Float32MultiArray, '/ee_poses',
                                 lambda m: self.buf['ee'].append(np.array(m.data, float)), 30)
        self.create_subscription(PoseStamped, '/target_robot/left_ee/target',
                                 lambda m: self._pose('tgt_l', m), 30)
        self.create_subscription(PoseStamped, '/target_robot/left_ee/state',
                                 lambda m: self._pose('st_l', m), 30)

    def _js(self, key, m):
        names = [str(n) for n in m.name]
        v = np.array(m.position, float)
        self.last[f'{key}_names'] = names
        self.buf[key].append(v)

    def _pose(self, key, m):
        p = m.pose.position
        q = m.pose.orientation
        self.buf[key].append(np.array([p.x, p.y, p.z, q.x, q.y, q.z, q.w]))


def main():
    rclpy.init()
    n = Sampler()
    t0 = time.monotonic()
    while time.monotonic() - t0 < DUR:
        rclpy.spin_once(n, timeout_sec=0.05)
    print(f'=== {LABEL} ===')
    deg = np.rad2deg
    if n.buf['exo']:
        names = n.last.get('exo_names', [])
        m = np.mean(n.buf['exo'][-max(1, len(n.buf['exo']) // 2):], axis=0)
        print('EXO_J ' + ' '.join(f'{nm}={deg(v):8.2f}' for nm, v in zip(names, m)))
        ptp = np.rad2deg(np.ptp(np.array(n.buf['exo']), axis=0))
        print('EXO_PTP ' + ' '.join(f'{nm}={p:7.2f}' for nm, p in zip(names, ptp)))
    if n.buf['ee']:
        ee = np.mean(n.buf['ee'][-max(1, len(n.buf['ee']) // 2):], axis=0)
        for side, off in (('L', 0), ('R', 7)):
            print(f'EE_{side} xyz={ee[off]:7.4f} {ee[off + 1]:7.4f} {ee[off + 2]:7.4f} '
                  f'quat={ee[off + 3]:7.4f} {ee[off + 4]:7.4f} {ee[off + 5]:7.4f} {ee[off + 6]:7.4f}')
    if n.buf['rob']:
        names = n.last.get('rob_names', [])
        m = np.mean(n.buf['rob'][-max(1, len(n.buf['rob']) // 2):], axis=0)
        print('ROB_J ' + ' '.join(f'{nm}={deg(v):8.2f}' for nm, v in zip(names, m)))
    for key in ('tgt_l', 'st_l'):
        if n.buf[key]:
            v = np.mean(n.buf[key][-max(1, len(n.buf[key]) // 2):], axis=0)
            print(f'{key.upper()} xyz={v[0]:7.4f} {v[1]:7.4f} {v[2]:7.4f} '
                  f'quat={v[3]:7.4f} {v[4]:7.4f} {v[5]:7.4f} {v[6]:7.4f}')
        else:
            print(f'{key.upper()} (无数据)')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
