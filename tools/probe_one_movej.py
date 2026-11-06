#!/usr/bin/env python3
"""只动 1° 的执行层探针：get_device_state → 基线 → 单帧 movej(+1° J1L) → 轮询反馈。
不 enable/disable，不清故障。用法: python3 -I probe_one_movej.py [host] [port]"""
import json
import socket
import sys
import time

host = sys.argv[1] if len(sys.argv) > 1 else '192.168.61.60'
port = int(sys.argv[2]) if len(sys.argv) > 2 else 30485
HOLD = -100.0

sock = socket.create_connection((host, port), timeout=5)
sock.settimeout(2.0)
buf = b''


def send(cmd):
    sock.sendall((json.dumps(cmd, separators=(',', ':')) + '#').encode())


def drain(window):
    global buf
    out = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < window:
        try:
            chunk = sock.recv(4096)
        except socket.timeout:
            continue
        if not chunk:
            break
        buf += chunk
        while b'#' in buf:
            raw, buf = buf.split(b'#', 1)
            if raw.strip():
                out.append(json.loads(raw.decode('utf-8', errors='ignore')))
    return out


send({'command': 'get_device_state'})
for m in drain(1.0):
    print('STATE', json.dumps(m, ensure_ascii=False)[:2000])

if len(sys.argv) > 3 and sys.argv[3] == 'readonly':
    send({'command': 'get_config'})
    for m in drain(1.0):
        print('CFG', json.dumps(m, ensure_ascii=False)[:2000])
    sock.close()
    sys.exit(0)

send({'command': 'get_joint_position'})
base = None
for m in drain(1.0):
    if isinstance(m.get('joint'), list) and len(m['joint']) >= 17:
        base = [float(x) for x in m['joint']]
print('BASE', [round(v, 4) for v in base] if base else None)

if base is None:
    print('NO_BASELINE')
    sys.exit(1)

tgt = list(base)
tgt[1] = base[1] + 0.0175  # 左 J1 +1°
tgt[0] = HOLD
tgt[15] = HOLD
tgt[16] = HOLD
print('TARGET J1L', round(tgt[1], 4), 'delta', round(tgt[1] - base[1], 4))
send({'command': 'movej', 'joint': tgt, 'v': 10.0})
for m in drain(1.2):
    print('ACK', json.dumps(m, ensure_ascii=False)[:300])

t0 = time.monotonic()
while time.monotonic() - t0 < 3.0:
    send({'command': 'get_joint_position'})
    for m in drain(0.35):
        if isinstance(m.get('joint'), list) and len(m['joint']) >= 17:
            j = [float(x) for x in m['joint']]
            print(f'FB t={time.monotonic() - t0:4.1f}s L1={j[1]:.4f} dL1={j[1] - base[1]:+.4f} '
                  f'L5={j[5]:.3f} L7={j[7]:.3f}')
sock.close()
