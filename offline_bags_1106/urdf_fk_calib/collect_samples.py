#!/usr/bin/env python3
"""真机采数器：URDF-vs-真机对账的第一步（T20.2，在 tc 主机上跑）。

⚠️ 前置（硬性）：
  1. TCP 30485 单连接 —— 先停 T3 bridge（pgrep -f "exton_bridg""e.py" | xargs -r kill）；
  2. 采数过程中手臂由操作员低速移动（示教/手动），本脚本默认【只读，不发任何运动命令】；
  3. ROS_DOMAIN 无关（纯 TCP）。

用法（tc 主机）：
  python3 collect_samples.py --probe            # 单发三条查询，打印原始应答（先验协议）
  python3 collect_samples.py --config           # 拉取 get_config 软限位并存盘
  python3 collect_samples.py --collect N        # 交互采 N 组：操作员摆好一个位姿 → 回车采样
输出：calib_samples_<ts>.jsonl（每行 {t, q17, tcp14, note}）+ config_limits_<ts>.json

协议要点（机械臂通信协议说明_V0.3）：
  - get_joint_position → joint[17]（弧度，腰/L1-7/R1-7/摇头/点头）
  - get_tcp_pose      → pose[14] = left(x,y,z,qw,qx,qy,qz) + right(x,y,z,qw,qx,qy,qz)
    ⚠️ 四元数是 qw-first，与 ROS 的 xyzw 相反；单位说明栏写“弧度”疑为文档笔误（xyz 应为米）
  - get_config        → upperLimits/lowerLimits（软限位）
"""
import argparse
import json
import socket
import sys
import time

HOST, PORT = '192.168.61.60', 30485
TAIL = '#'


def send_recv(sock, payload, timeout=2.0):
    sock.sendall((json.dumps(payload) + TAIL).encode('utf-8'))
    sock.settimeout(timeout)
    buf = ''
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            chunk = sock.recv(4096)
        except socket.timeout:
            break
        if not chunk:
            raise ConnectionError('控制器关闭连接')
        buf += chunk.decode('utf-8', errors='ignore')
        if TAIL in buf:
            raw = buf.split(TAIL)[0].strip()
            return json.loads(raw)
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--probe', action='store_true', help='单发三条查询验证协议')
    ap.add_argument('--config', action='store_true', help='拉取软限位存盘')
    ap.add_argument('--collect', type=int, default=0, help='交互采数组数')
    ap.add_argument('--host', default=HOST)
    args = ap.parse_args()

    sock = socket.create_connection((args.host, PORT), timeout=5)
    print(f'已连接 {args.host}:{PORT}（确认 bridge 已停！）')

    if args.probe or not (args.config or args.collect):
        for cmd in ('get_joint_position', 'get_tcp_pose', 'get_config'):
            r = send_recv(sock, {'command': cmd})
            print(f'--- {cmd} ---')
            print(json.dumps(r, ensure_ascii=False, indent=1)[:800])
        if not (args.config or args.collect):
            return

    if args.config:
        r = send_recv(sock, {'command': 'get_config'})
        fn = f'config_limits_{time.strftime("%m%d_%H%M%S")}.json'
        with open(fn, 'w', encoding='utf-8') as f:
            json.dump(r, f, ensure_ascii=False, indent=1)
        print(f'软限位已存 {fn}')
        print(json.dumps(r, ensure_ascii=False)[:600])

    if args.collect:
        n = args.collect
        fn = f'calib_samples_{time.strftime("%m%d_%H%M%S")}.jsonl'
        print(f'开始采 {n} 组 → {fn}')
        print('流程：操作员把臂摆到一个新位姿（尽量覆盖工作空间各区域，含近奇异与'
              '工作空间边缘）→ 操作员确认静止后，AI 按回车采样。')
        got = 0
        with open(fn, 'w', encoding='utf-8') as f:
            while got < n:
                input(f'[{got+1}/{n}] 摆好位姿后回车采样（Ctrl-C 结束）…')
                q = send_recv(sock, {'command': 'get_joint_position'})
                tcp = send_recv(sock, {'command': 'get_tcp_pose'})
                if q is None or tcp is None:
                    print('  !! 查询超时/无应答，重试本次')
                    continue
                rec = {'t': time.time(), 'q17': q.get('joint'), 'q_status': q.get('status'),
                       'tcp14': tcp.get('pose'), 'tcp_status': tcp.get('status'),
                       'note': input('  位姿备注（可空，如"低位近奇异"）: ').strip()}
                f.write(json.dumps(rec, ensure_ascii=False) + '\n')
                f.flush()
                got += 1
                print(f'  已存 {got}/{n}: q[:3]={[round(x,3) for x in (rec["q17"] or [])[:3]]}')
        print(f'完成 → {fn}（拷回本地 tools/dm-exo/urdf_fk_calib/ 后跑 compare_fk.py）')
    sock.close()


if __name__ == '__main__':
    sys.exit(main())
