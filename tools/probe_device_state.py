#!/usr/bin/env python3
"""只读探针：查控制器设备状态与故障码（get_device_state / get_config）。
不发送任何运动命令。用法: python3 -I probe_device_state.py [host] [port]"""
import json
import socket
import sys

host = sys.argv[1] if len(sys.argv) > 1 else '192.168.61.60'
port = int(sys.argv[2]) if len(sys.argv) > 2 else 30485

sock = socket.create_connection((host, port), timeout=5)
sock.settimeout(2.0)


def send(cmd: dict) -> None:
    sock.sendall((json.dumps(cmd, separators=(',', ':')) + '#').encode())


def drain(window: float) -> list:
    out, buf = [], b''
    import time
    t0 = time.monotonic()
    while time.monotonic() - t0 < window:
        try:
            chunk = sock.recv(4096)
        except socket.timeout:
            break
        if not chunk:
            break
        buf += chunk
        while b'#' in buf:
            raw, buf = buf.split(b'#', 1)
            if raw.strip():
                out.append(raw.decode('utf-8', errors='ignore'))
    return out

for cmd in ({'command': 'get_device_state'},
            {'command': 'get_joint_position'},
            {'command': 'get_config'}):
    send(cmd)
    for line in drain(1.5):
        print(line[:1200])
    print('---')
sock.close()
