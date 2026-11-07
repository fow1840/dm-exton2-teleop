#!/usr/bin/env python3
"""T39 真机采数驱动：按离线位姿清单逐位姿驱动+采样（sweep_v2 护栏全继承 + 三项新增）。

与 sweep_v2 的关系：护栏逻辑逐条照搬（真机反馈出发/步进 0.005/禁区位姿/每步
device_state/拒绝减半/静定双读），不改动 sweep_v2 本体（无损改动=新增文件）。
新增：
  1. 位姿清单来自 gen_calib_poses.py 离线 JSON（现场不现场发挥）；
  2. 反馈活性校验：构型间距 ≥min_q_dist 的两次采样，关节反馈变化必须 ≥0.05rad，
     否则判「反馈缓存/冻结」（T26 签名）→ 立即全停，无效数据不入库；
  3. 途经点禁区检查：两非禁区构型的关节空间直线段可能穿过肘堵转禁区
     （{J2<-1.5 ∧ J4>1.0} 是凸集，线段可穿入）→ 逐段 9 点采样，命中则插入
     脱困路点（J2 拉回 ≥-1.3 或 J4 压回 ≤0.9）；
  4. 默认 dry-run（只打印行动计划，不动臂），--go 才真正驱动。

用法（tc 主机，先停 T3 bridge、5566 清故障+enable+关网页）：
  python3 sweep_calib.py --poses calib_poses_left_60.json            # dry-run
  python3 sweep_calib.py --poses calib_poses_left_60.json --go       # 执行
输出：/home/tc/tcvg/urdf_fk_calib/calib_<side>_full_<时间戳>.jsonl
"""
import argparse
import json
import socket
import sys
import time

HOST, PORT = "192.168.61.60", 30485
HOLD = -100.0
STEP0 = 0.005
SETTLE_S = 0.6
STATIC_TH = 0.002
MIN_Q_DIST = 0.9          # 与 gen_calib_poses 一致
LIVENESS_TH = 0.05        # 反馈活性门：相邻采样构型距 ≥MIN_Q_DIST，反馈变化必须 ≥ 此值


def forbidden(q7):
    """肘堵转禁区（sweep_v2 规则）：J2<-1.5 且 J4>1.0 的收臂负载组合。"""
    return q7[1] < -1.5 and q7[3] > 1.0


class Ctrl:
    """连接与协议封装（sweep_v2 原样照搬，仅加 rejects 计数清零语义不变）。"""

    def __init__(self):
        self.sock = socket.create_connection((HOST, PORT), timeout=5)
        self.sock.settimeout(0.05)
        self.buf = ""
        self.rejects = 0
        self.step = STEP0

    def send(self, p):
        self.sock.sendall((json.dumps(p) + "#").encode())

    def pump(self, t=0.03):
        t0 = time.monotonic()
        out = []
        while time.monotonic() - t0 < t:
            try:
                chunk = self.sock.recv(4096)
            except socket.timeout:
                continue
            if not chunk:
                break
            self.buf += chunk.decode("utf-8", errors="ignore")
            while "#" in self.buf:
                raw, self.buf = self.buf.split("#", 1)
                raw = raw.strip()
                if raw:
                    try:
                        out.append(json.loads(raw))
                    except json.JSONDecodeError:
                        pass
        for f in out:
            if f.get("status") == "program_run_failed":
                self.rejects += 1
        return out

    def rpc(self, payload, key, timeout=1.5):
        self.send(payload)
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            for f in self.pump(0.05):
                if key in f:
                    return f
        return None

    def q(self):
        f = self.rpc({"command": "get_joint_position"}, "joint")
        return f["joint"] if f else None

    def tcp(self):
        f = self.rpc({"command": "get_tcp_pose"}, "pose")
        return f["pose"] if f else None

    def dev_state(self):
        f = self.rpc({"command": "get_device_state"}, "state")
        return (f or {}).get("state", "?")

    def drive(self, side_idx, target):
        """从真机反馈逐帧走向 target（sweep_v2 drive 原语义）。返回 (是否到位, q17)。"""
        q17 = None
        t0 = time.monotonic()
        while time.monotonic() - t0 < 90:
            if self.rejects >= 3:
                print("    连续拒绝 %d 帧 → stop_motion 清队" % self.rejects)
                self.rejects = 0
                self.send({"command": "stop_motion"})
                time.sleep(0.6)
                self.pump(0.3)
                continue
            real = self.q()
            if real is None:
                continue
            q17 = real
            for _ in range(4):
                delta = [target[i] - q17[side_idx + i] for i in range(7)]
                dmax = max(abs(d) for d in delta)
                if dmax < 0.008:
                    return True, q17
                frame = list(q17)
                for i in range(7):
                    frame[side_idx + i] = q17[side_idx + i] + max(
                        -self.step, min(self.step, delta[i]))
                frame[0] = HOLD
                frame[15] = HOLD
                frame[16] = HOLD
                self.send({"command": "movej_canfd", "joint": frame, "freq": 50})
                self.pump(0.03)
                q17 = frame
            if self.rejects:
                self.step = max(0.003, self.step / 2)
                print("    拒绝 %d 帧 → 步进 %.4f" % (self.rejects, self.step))
                self.rejects = 0
        return False, q17


def waypoints(a, b):
    """a→b 的关节空间路点序列：直线 9 点查禁区，命中插脱困点（一深度）。"""
    seg = [a, b]
    out = []
    for s, e in zip(seg[:-1], seg[1:]):
        pts = [tuple(s[i] + (e[i] - s[i]) * t / 8.0 for i in range(7)) for t in range(9)]
        hit = next((p for p in pts if forbidden(p)), None)
        if hit is None:
            out.append(e)
            continue
        # 脱困点：中点拉回安全侧（J2 抬到 -1.3 优先，否则 J4 压到 0.9）
        mid = [(s[i] + e[i]) / 2 for i in range(7)]
        if mid[1] < -1.5:
            mid[1] = -1.3
        if mid[3] > 0.9:
            mid[3] = 0.9
        if forbidden(mid):
            mid = list(s)  # 兜底：退回起点侧（几乎不会走到）
            mid[1] = max(mid[1], -1.3)
            mid[3] = min(mid[3], 0.9)
        print("    ⚠️ 途经禁区（q≈%s）→ 插脱困路点 %s" % (
            [round(v, 2) for v in hit], [round(v, 2) for v in mid]))
        out.append(mid)
        out.append(e)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--poses', required=True, help='gen_calib_poses.py 生成的 JSON 清单')
    ap.add_argument('--go', action='store_true', help='真正驱动（默认 dry-run）')
    ap.add_argument('--limit', type=int, default=0, help='只执行前 N 个位姿（0=全部），首次跑建议 --limit 3')
    args = ap.parse_args()

    meta = json.load(open(args.poses, encoding='utf-8'))
    side = meta['side']
    side_idx = 1 if side == 'left' else 8
    poses = meta['poses']
    if args.limit:
        poses = poses[:args.limit]
    tag = 'LF' if side == 'left' else 'RF'
    print('清单: %s  臂=%s  位姿数=%d  min_q_dist=%.2f  禁区=%s' % (
        args.poses, side, len(poses), MIN_Q_DIST, meta.get('forbidden', '?')))
    print('模式: %s' % ('--go 执行' if args.go else 'DRY-RUN（不加 --go 不动臂）'))

    c = Ctrl()
    st = c.dev_state()
    print('初始 state:', st)
    if st not in ('ready', 'idle'):
        print('!! 控制器非就绪（须 5566 清故障+人工 enable+关网页），退出')
        return 1

    if not args.go:
        prev = None
        for k, tgt in enumerate(poses):
            d = 0.0 if prev is None else float(np_l2(prev, tgt))
            wps = waypoints(prev, tgt) if prev is not None and d > 0.9 else [tgt]
            print('  %s%02d  Δq=%.2f rad  途经点 %d 个  q=%s' % (
                tag, k, d, len(wps), [round(v, 2) for v in tgt]))
            prev = tgt
        print('dry-run 结束。确认无禁线路径后加 --go 执行（建议先 --limit 3 首段验证）。')
        return 0

    fn = '/home/tc/tcvg/urdf_fk_calib/calib_%s_full_%s.jsonl' % (
        tag[:1], time.strftime('%m%d_%H%M'))
    fout = open(fn, 'a')
    ok_n = 0
    prev_q = None
    t_start = time.time()
    for k, tgt in enumerate(poses):
        st = c.dev_state()
        if st not in ('ready', 'idle'):
            print('!! 位姿 %d 前 state=%s → 全停' % (k, st))
            c.send({"command": "stop_motion"})
            break
        qcur = c.q()
        if qcur is None:
            print('!! 关节读数失败 → 全停')
            break
        # 途经点（从当前实际构型出发，不是从上一个目标出发）
        cur7 = [qcur[side_idx + i] for i in range(7)]
        for wp in waypoints(cur7, tgt):
            reached, q17 = c.drive(side_idx, wp)
            if not reached:
                print('  %s%02d 途经点未到位（超时）→ 全停' % (tag, k))
                c.send({"command": "stop_motion"})
                fout.close()
                print('已完成 %d/%d，部分结果在 %s' % (ok_n, len(poses), fn))
                return 1
        time.sleep(SETTLE_S)
        q1 = c.q()
        time.sleep(0.3)
        q2 = c.q()
        if q1 is None or q2 is None or max(
                abs(a - b) for a, b in zip(q1[1:15], q2[1:15])) > STATIC_TH:
            print('  %s%02d 未静止，跳过采样' % (tag, k))
            continue
        # 反馈活性校验（T26 缓存/冻结签名防御）
        arm_q = q1[side_idx:side_idx + 7]
        if prev_q is not None:
            moved = max(abs(a - b) for a, b in zip(arm_q, prev_q))
            if moved < LIVENESS_TH:
                print('!! %s%02d 反馈疑似冻结（目标距 %.2f rad 但反馈仅变 %.3f rad）→ 全停，' % (
                    tag, k, MIN_Q_DIST, moved),
                    '按 T26 流程处理（断电重启/查供电），本文件数据存疑')
                c.send({"command": "stop_motion"})
                fout.close()
                return 2
        st = c.dev_state()
        if st not in ('ready', 'idle'):
            print('!! 采样前 state=%s → 全停' % st)
            c.send({"command": "stop_motion"})
            break
        tcp = c.tcp()
        if tcp is None:
            print('  %s%02d tcp 查询失败，跳过' % (tag, k))
            continue
        rec = {"t": time.time(), "pose_idx": k, "q17": q1, "tcp14": tcp,
               "note": "%s%02d" % (tag, k)}
        fout.write(json.dumps(rec) + "\n")
        fout.flush()
        ok_n += 1
        prev_q = arm_q
        dt = time.time() - t_start
        eta = dt / ok_n * (len(poses) - ok_n) if ok_n else 0
        print('  %s%02d ok (%.0fs, ETA %.0fs) q=%s' % (
            tag, k, dt, eta, [round(v, 2) for v in arm_q]))
    fout.close()
    c.send({"command": "stop_motion"})
    print('完成 %d/%d → %s（stop_motion 已发）' % (ok_n, len(poses), fn))
    c.sock.close()
    return 0


def np_l2(a, b):
    return sum((x - y) ** 2 for x, y in zip(a, b)) ** 0.5


if __name__ == '__main__':
    sys.exit(main())
