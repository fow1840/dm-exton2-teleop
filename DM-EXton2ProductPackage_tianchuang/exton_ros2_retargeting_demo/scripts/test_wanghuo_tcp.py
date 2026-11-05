#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
脱离 ROS，单独测 wanghuo TCP 30485 是否接受运动。
用法（PC-A，先停掉 exton_bridge，人在急停旁）：

  python3 test_wanghuo_tcp.py
  python3 test_wanghuo_tcp.py --delta-deg 2.0
  python3 test_wanghuo_tcp.py --mode movej_canfd
"""

from __future__ import annotations

import argparse
import json
import math
import socket
import time
from typing import List, Optional

HOST = '192.168.61.60'
PORT = 30485
N = 17
HOLD = -100.0
TAIL = '#'


def recv_frames(sock: socket.socket, wait_s: float = 1.0) -> List[dict]:
    sock.settimeout(0.05)
    buf = ''
    end = time.time() + wait_s
    out: List[dict] = []
    while time.time() < end:
        try:
            chunk = sock.recv(4096)
            if not chunk:
                break
            buf += chunk.decode('utf-8', errors='ignore')
            while TAIL in buf:
                raw, buf = buf.split(TAIL, 1)
                raw = raw.strip()
                if not raw:
                    continue
                try:
                    out.append(json.loads(raw))
                except json.JSONDecodeError:
                    print('  [非JSON]', raw[:200])
        except socket.timeout:
            pass
    return out


def send(sock: socket.socket, payload: dict) -> None:
    line = json.dumps(payload, ensure_ascii=False, separators=(',', ':')) + TAIL
    print('>>> SEND', line)
    sock.sendall(line.encode('utf-8'))


def extract_joint(msgs: List[dict]) -> Optional[List[float]]:
    for m in msgs:
        for k in ('joint', 'joints'):
            if k in m and isinstance(m[k], list) and len(m[k]) >= 7:
                return [float(x) for x in m[k]]
    return None


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', default=HOST)
    ap.add_argument('--port', type=int, default=PORT)
    ap.add_argument('--mode', choices=('movej', 'movej_canfd'), default='movej')
    ap.add_argument('--delta-deg', type=float, default=2.0, help='只动右臂第1轴的增量（度）')
    ap.add_argument('--as-deg', action='store_true', help='按“度”下发（默认按弧度，与 V0.3 文档一致）')
    args = ap.parse_args()

    print(f'连接 {args.host}:{args.port} ...')
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5.0)
    sock.connect((args.host, args.port))
    print('已连接')

    send(sock, {'command': 'get_joint_position'})
    msgs = recv_frames(sock, 1.5)
    print('<<< 反馈应答', json.dumps(msgs, ensure_ascii=False)[:800])
    q = extract_joint(msgs)
    if not q:
        print('未解析到 joint，请把上面应答全文发给支持/开发')
        sock.close()
        return

    # 补齐 17 维
    while len(q) < N:
        q.append(HOLD)
    q = q[:N]
    print('当前 joint[:17] =', [round(x, 4) for x in q])
    print('数值量级判断: 若多数 |值|>6.5，更像“度”；若多数 |值|<3.5，更像“弧度”')

    send(sock, {'command': 'enable_robot'})
    print('<<< enable 应答', json.dumps(recv_frames(sock, 1.0), ensure_ascii=False)[:500])

    # 只改右臂第1轴（协议下标 8），其余保持
    target = list(q)
    delta = args.delta_deg if args.as_deg else math.radians(args.delta_deg)
    # 若反馈是度而我们按弧度理解，这里仍用“在反馈数值上直接加 delta”
    # --as-deg: delta 用度；默认: 把反馈当弧度，加弧度增量
    if args.as_deg:
        # 假设反馈也是度
        target[8] = float(q[8]) + args.delta_deg
        unit = 'deg'
    else:
        target[8] = float(q[8]) + math.radians(args.delta_deg)
        unit = 'rad'

    for i in (0, 15, 16):
        target[i] = HOLD

    print(f'准备小动右臂J1: +{args.delta_deg}°（按 {unit} 编码），急停旁确认后回车...')
    input()

    if args.mode == 'movej':
        payload = {'command': 'movej', 'joint': target, 'v': 10.0}
    else:
        payload = {'command': 'movej_canfd', 'joint': target, 'freq': 50}

    send(sock, payload)
    print('<<< 运动应答', json.dumps(recv_frames(sock, 2.0), ensure_ascii=False)[:800])

    time.sleep(1.0)
    send(sock, {'command': 'get_joint_position'})
    msgs2 = recv_frames(sock, 1.5)
    q2 = extract_joint(msgs2)
    print('<<< 运动后反馈', json.dumps(msgs2, ensure_ascii=False)[:800])
    if q2:
        while len(q2) < N:
            q2.append(HOLD)
        print('运动后 joint[8] =', q2[8], '  变化量 =', float(q2[8]) - float(q[8]))

    send(sock, {'command': 'stop_motion'})
    recv_frames(sock, 0.5)
    sock.close()
    print('完成')


if __name__ == '__main__':
    main()
