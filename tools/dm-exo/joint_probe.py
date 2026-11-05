#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机械臂关节直连探测 / 微动测试（协议 V0.3：TCP JSON + '#' 帧尾）。

⚠️ TCP 为单连接：运行前必须先停 exton_bridge（T3），否则会抢连接。
   停止：bash /home/tc/桌面/dm-exo/stop_bridge_only.sh（只停 bridge，不影响其余组件）

用法：
  只读探测（安全，不下发任何运动）：
    python3 joint_probe.py read
  单关节微动测试（需显式给出关节和幅度）：
    python3 joint_probe.py move <joint> [delta_rad] [v]
    例:  python3 joint_probe.py move R7 0.05 5     # 右臂J7 正向 0.05rad(≈2.9°)，速度5
         python3 joint_probe.py move L1 -0.05 5    # 左臂J1 负向
  立即停止运动：
    python3 joint_probe.py stop
"""
import json
import math
import socket
import sys
import time

HOST = '192.168.61.60'
PORT = 30485
N = 17
IDX_WAIST = 0
IDX_LEFT = 1     # L1..L7 → 1..7
IDX_RIGHT = 8    # R1..R7 → 8..14
IDX_HEAD_SHAKE = 15
IDX_HEAD_NOD = 16
HOLD = -100.0
TAIL = '#'

# 17 维名称表
DIM_NAMES = (
    ['腰']
    + [f'左臂J{i+1}(L{i+1})' for i in range(7)]
    + [f'右臂J{i+1}(R{i+1})' for i in range(7)]
    + ['摇头', '点头']
)

# 右臂软限位（度，来自档案）——move 时用于预检
RIGHT_LIMITS_DEG = [
    (-177.9, 177.9), (-146.97, 152.70), (-37.53, 127.49),
    (-146.97, 152.70), (-87.95, 87.95), (-152.70, 152.70), (-152.70, 152.70),
]


def connect() -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5.0)
    sock.connect((HOST, PORT))
    sock.settimeout(0.3)
    return sock


def send(sock: socket.socket, payload: dict) -> None:
    sock.sendall((json.dumps(payload, ensure_ascii=False, separators=(',', ':')) + TAIL).encode('utf-8'))


def recv_frames(sock: socket.socket, window: float = 0.3) -> list:
    """在 window 时间窗内收全 JSON 帧。"""
    buf = ''
    deadline = time.time() + window
    while time.time() < deadline:
        try:
            chunk = sock.recv(4096)
            if not chunk:
                raise ConnectionError('连接被控制器关闭')
            buf += chunk.decode('utf-8', errors='ignore')
        except socket.timeout:
            break
    frames = []
    while TAIL in buf:
        raw, buf = buf.split(TAIL, 1)
        raw = raw.strip()
        if raw:
            try:
                frames.append(json.loads(raw))
            except json.JSONDecodeError:
                pass
    return frames


def get_joints(sock: socket.socket, retries: int = 3) -> list | None:
    """get_joint_position → 17 维弧度列表。"""
    for _ in range(retries):
        try:
            send(sock, {'command': 'get_joint_position'})
            deadline = time.time() + 2.0
            while time.time() < deadline:
                for m in recv_frames(sock, 0.3):
                    j = m.get('joint', m.get('joints'))
                    if isinstance(j, list) and len(j) >= N:
                        return [float(x) for x in j[:N]]
        except (OSError, ConnectionError) as exc:
            print(f'  读取异常: {exc}')
            raise
    return None


def parse_joint_arg(arg: str) -> int:
    """'R7'/'L1' 或协议下标 1~17 → 17 维下标。"""
    a = arg.strip().upper()
    if a.startswith('L') and a[1:].isdigit():
        return IDX_LEFT + int(a[1:]) - 1
    if a.startswith('R') and a[1:].isdigit():
        return IDX_RIGHT + int(a[1:]) - 1
    if a.isdigit() and 1 <= int(a) <= N:
        return int(a) - 1
    raise SystemExit(f'无法识别关节 {arg}（用 R1~R7 / L1~L7 / 1~17）')


def print_joints(j: list, title: str) -> None:
    print(f'--- {title} ---')
    print(f'{"下标":>4} {"名称":<14} {"弧度":>10} {"角度":>10}')
    for i, v in enumerate(j):
        mark = '' if abs(v - HOLD) < 1e-6 else ''
        pos = f'{v:+.4f}' if abs(v - HOLD) >= 1e-6 else '  HOLD '
        ang = f'{math.degrees(v):+8.2f}°' if abs(v - HOLD) >= 1e-6 else '    -    '
        print(f'{i:>4} {DIM_NAMES[i]:<14} {pos:>10} {ang:>10}{mark}')


def check_limit(idx: int, target_rad: float) -> bool:
    """仅右臂有档案软限位，做预检；返回 True=越限。"""
    if IDX_RIGHT <= idx < IDX_RIGHT + 7:
        lo, hi = RIGHT_LIMITS_DEG[idx - IDX_RIGHT]
        deg = math.degrees(target_rad)
        if deg < lo or deg > hi:
            print(f'❌ 越限: {DIM_NAMES[idx]} 目标 {deg:.2f}° 超出 [{lo:.1f}, {hi:.1f}]')
            return True
    return False


def query_once(sock: socket.socket, timeout: float = 0.5):
    """单次查询-等首帧应答，返回 (往返耗时ms, 17维) 或 (None, None)。"""
    send(sock, {'command': 'get_joint_position'})
    t0 = time.perf_counter()
    buf = ''
    while time.perf_counter() - t0 < timeout:
        try:
            chunk = sock.recv(4096)
            if not chunk:
                raise ConnectionError('连接被控制器关闭')
            buf += chunk.decode('utf-8', errors='ignore')
        except socket.timeout:
            continue
        while TAIL in buf:
            raw, buf = buf.split(TAIL, 1)
            raw = raw.strip()
            if not raw:
                continue
            try:
                m = json.loads(raw)
            except json.JSONDecodeError:
                continue
            j = m.get('joint', m.get('joints'))
            if isinstance(j, list) and len(j) >= N:
                return (time.perf_counter() - t0) * 1000.0, [float(x) for x in j[:N]]
    return None, None


def do_bench(sock: socket.socket) -> None:
    """只读基准：get_joint_position 往返延迟 + 可持续查询频率。"""
    import statistics
    print('预热 3 次 ...')
    for _ in range(3):
        ms, _ = query_once(sock)
        if ms is None:
            raise SystemExit('❌ 预热读取失败')

    lat = []
    for _ in range(50):
        ms, _ = query_once(sock)
        if ms is None:
            print('⚠️ 一次读取超时，跳过')
            continue
        lat.append(ms)
        time.sleep(0.02)
    if lat:
        lat.sort()
        print(f'往返延迟 ms: n={len(lat)} min={lat[0]:.1f} '
              f'中位={statistics.median(lat):.1f} '
              f'p95={lat[max(0, int(len(lat) * 0.95) - 1)]:.1f} max={lat[-1]:.1f}')

    print('可持续频率测试 5 秒（发→收→立即再发）...')
    n = 0
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < 5.0:
        _, j = query_once(sock)
        if j is not None:
            n += 1
    dt = time.perf_counter() - t0
    print(f'可持续查询频率: {n / dt:.1f} Hz（{n} 帧 / {dt:.1f}s）')


def do_read(sock: socket.socket) -> None:
    for k in range(3):
        j = get_joints(sock)
        if j is None:
            print('❌ 3 次未读到关节反馈，检查连接/协议')
            return
        print_joints(j, f'第 {k + 1} 次读取')
        time.sleep(0.5)


def do_move(sock: socket.socket, joint_arg: str, delta: float, v: int) -> None:
    idx = parse_joint_arg(joint_arg)
    if not (IDX_LEFT <= idx < IDX_RIGHT + 7):
        raise SystemExit('腰/头轴不做微动测试（遥操作中始终保持）')

    print('读取当前关节 ...')
    cur = get_joints(sock)
    if cur is None:
        raise SystemExit('❌ 读不到反馈，禁止下发')
    print_joints(cur, '当前关节')

    target = cur[idx] + delta
    if check_limit(idx, target):
        raise SystemExit('❌ 目标越软限位，已取消')

    # 构造 17 维：14 个臂关节全部发绝对值（未动轴=当前值，不产生运动），
    # 腰/头发 -100（保持）——与 bridge movej 模式一致的已验证模式
    joints = list(cur)
    joints[IDX_WAIST] = HOLD
    joints[IDX_HEAD_SHAKE] = HOLD
    joints[IDX_HEAD_NOD] = HOLD
    joints[idx] = target

    print(f'\n>>> movej: {DIM_NAMES[idx]}  {math.degrees(cur[idx]):+.2f}° → {math.degrees(target):+.2f}° '
          f'(Δ={math.degrees(delta):+.2f}°), v={v}, 其余轴保持')
    print('    3 秒后下发，Ctrl-C 取消 ...')
    try:
        for s in (3, 2, 1):
            print(f'    {s} ...')
            time.sleep(1)
    except KeyboardInterrupt:
        print('\n已取消')
        return

    try:
        send(sock, {'command': 'movej', 'joint': joints, 'v': v})
        t0 = time.time()
        ack = []
        while time.time() - t0 < 1.5:
            ack += recv_frames(sock, 0.3)
            if ack:
                break
        print(f'控制器应答: {ack if ack else "（无应答）"}')

        # 轮询反馈直到到位或超时
        t0 = time.time()
        while time.time() - t0 < 10.0:
            time.sleep(0.3)
            now = get_joints(sock, retries=1)
            if now is None:
                continue
            err = abs(now[idx] - target)
            print(f'  t+{time.time() - t0:4.1f}s  {DIM_NAMES[idx]}={math.degrees(now[idx]):+8.2f}°  误差={math.degrees(err):5.2f}°')
            if err < math.radians(0.5):
                print('✅ 到位')
                return
        print('⚠️ 10 秒未到位（可能被限速/限位/未使能），请人工确认臂状态')
    except KeyboardInterrupt:
        print('\n⚠️ 中断 —— 发送 stop_motion')
        send(sock, {'command': 'stop_motion'})


def main() -> None:
    if len(sys.argv) < 2 or sys.argv[1] not in ('read', 'move', 'stop', 'bench'):
        print(__doc__)
        raise SystemExit(1)

    mode = sys.argv[1]
    sock = connect()
    print(f'已连接 {HOST}:{PORT}')
    sock.settimeout(0.05)
    try:
        if mode == 'read':
            do_read(sock)
        elif mode == 'bench':
            do_bench(sock)
        elif mode == 'stop':
            send(sock, {'command': 'stop_motion'})
            print('已发送 stop_motion')
        elif mode == 'move':
            # argv: [0]=脚本 [1]=move [2]=关节 [3]=delta [4]=v
            if len(sys.argv) < 3:
                raise SystemExit('用法: joint_probe.py move <R1~R7|L1~L7> [delta_rad=0.05] [v=5]')
            delta = float(sys.argv[3]) if len(sys.argv) > 3 else 0.05
            v = int(sys.argv[4]) if len(sys.argv) > 4 else 5
            if abs(delta) > 0.15:
                raise SystemExit('❌ 单次幅度限制 ±0.15 rad（≈±8.6°），如需更大请分多次')
            if not (1 <= v <= 20):
                raise SystemExit('❌ 测试阶段 v 限制 1~20')
            do_move(sock, sys.argv[2], delta, v)
    finally:
        sock.close()


if __name__ == '__main__':
    main()
