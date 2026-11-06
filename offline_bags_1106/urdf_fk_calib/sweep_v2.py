#!/usr/bin/env python3
"""T20.2 安全采数驱动 v2（2026-11-06，肘堵转事故后重写）。

与 v1 的区别（事故教训，见 WORKLOG 2026-11-06 09:33-10:0x 条目）：
  1. 每帧从真机反馈出发（v1 用本地模型外推，帧被拒后跑飞）；
  2. 步进 0.005 rad/帧（v1 是 0.01）；
  3. 位姿集避开肘负载收臂组合：J2 >= -1.5 且 J4 <= +1.0（事故位姿是 J2=-2.1+J4=+1.2）；
  4. 每个位姿驱动前查 device_state，非 ready/idle 立即全停；
  5. 驱动中监听 program_run_failed，观测到即把步进减半并告警；
  6. 采样前双读静止判定 + device_state 复查。

用法（tc 主机，先停 bridge）：
  python3 sweep_v2.py left    # 左臂 12 位姿
  python3 sweep_v2.py right   # 右臂 6 位姿
输出：/home/tc/tcvg/urdf_fk_calib/calib_{L2,R2}_drive.jsonl
"""
import json
import socket
import sys
import time

HOST, PORT = "192.168.61.60", 30485
HOLD = -100.0
STEP0 = 0.005
SETTLE_S = 0.6
STATIC_TH = 0.002

# get_config 实测限位（左右相同）内缩 0.15
LIM = [(-2.64, 2.64), (-2.21, 0.11), (-2.64, 2.64), (-0.54, 2.11),
       (-2.64, 2.64), (-1.42, 1.42), (-2.64, 2.64)]

LEFT_T = [
    [-0.3, -0.8, -0.9, 0.2, -0.15, 0.5, -0.9],
    [-0.6, -1.4, -0.5, 0.6, -0.3, 0.9, -1.2],
    [0.3, -1.2, 0.5, -0.4, 0.2, -0.5, 0.6],
    [0.8, -0.6, 1.0, 0.4, 0.5, 0.3, 1.5],
    [-1.5, -1.0, -1.5, 0.8, -0.6, 1.1, -2.0],
    [1.5, -1.3, 1.4, 0.1, 0.8, -1.0, 1.8],
    [0.0, -0.4, 0.0, 0.95, 0.1, 0.0, 0.0],
    [-2.0, -0.7, -0.6, 0.3, 0.6, -0.8, -1.8],
    [2.0, -0.9, 0.8, 0.5, -0.4, 0.9, 1.2],
    [0.4, -1.5, 0.2, 0.7, 1.0, -0.6, -0.4],
    [-0.8, -0.5, 1.2, -0.3, -0.9, 0.8, 2.0],
    [-0.0, -1.4, -0.2, 0.9, 0.0, 0.2, -1.4],
]
RIGHT_T = [
    [0.5, -1.0, 0.3, 0.5, -0.5, 0.6, 0.8],
    [-0.7, -1.5, -0.3, 0.8, 0.3, -0.7, 1.5],
    [1.2, -0.8, 0.9, 0.2, -0.8, 0.5, -1.6],
    [-1.6, -1.2, 1.1, 0.6, 0.5, 1.0, -1.9],
    [0.2, -0.5, 0.0, 0.9, 0.2, -0.4, 0.4],
    [-0.4, -1.1, -0.8, 0.4, -1.0, -1.1, 1.0],
]


class Ctrl:
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

    def drive(self, q17, side_idx, target):
        """从真机反馈逐帧走向 target；每 4 帧同步一次反馈（漂移上界 4×step）。
        返回 (是否到位, 最终q17)。"""
        t0 = time.monotonic()
        while time.monotonic() - t0 < 90:
            # 队列灌满自愈：连续拒绝 → stop_motion 清队 → 重新同步 → 继续
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
            for _ in range(4):  # 批内从同一 real 出发，漂移上界 4×step≈0.02rad
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
                self.pump(0.03)  # ~30Hz 流率，减轻控制器队列压力
                q17 = frame
            if self.rejects:
                self.step = max(0.003, self.step / 2)
                print("    拒绝 %d 帧 → 步进 %.4f" % (self.rejects, self.step))
                self.rejects = 0
        return False, q17


def main():
    side = sys.argv[1] if len(sys.argv) > 1 else "left"
    side_idx = 1 if side == "left" else 8
    tag = "L2" if side == "left" else "R2"
    targets = LEFT_T if side == "left" else RIGHT_T

    c = Ctrl()
    st = c.dev_state()
    print("初始 state:", st)
    if st not in ("ready", "idle"):
        print("!! 控制器非就绪，退出")
        return 1

    fn = "/home/tc/tcvg/urdf_fk_calib/calib_%s_drive.jsonl" % tag
    fout = open(fn, "w")
    ok_n = 0
    for k, tgt in enumerate(targets):
        tgt = [max(lo, min(hi, v)) for v, (lo, hi) in zip(tgt, LIM)]
        st = c.dev_state()
        if st not in ("ready", "idle"):
            print("!! 位姿 %d 前 state=%s → 全停" % (k, st))
            c.send({"command": "stop_motion"})
            break
        # 大位移先拆半程路点（长行程末段掉速，J7 实测单段 >1.5rad 走不完）
        qcur = c.q()
        if qcur:
            d = [tgt[i] - qcur[side_idx + i] for i in range(7)]
            if max(abs(x) for x in d) > 0.9:
                mid = [qcur[side_idx + i] + 0.5 * d[i] for i in range(7)]
                c.drive(None, side_idx, mid)
        reached, q17 = c.drive(None, side_idx, tgt)
        if not reached:
            print("  %s%02d 未到位（超时）→ 全停" % (tag, k))
            c.send({"command": "stop_motion"})
            break
        time.sleep(SETTLE_S)
        q1 = c.q()
        time.sleep(0.3)
        q2 = c.q()
        if q1 is None or q2 is None or max(
                abs(a - b) for a, b in zip(q1[1:15], q2[1:15])) > STATIC_TH:
            print("  %s%02d 未静止，跳过采样" % (tag, k))
            continue
        st = c.dev_state()
        if st not in ("ready", "idle"):
            print("!! 采样前 state=%s → 全停" % st)
            c.send({"command": "stop_motion"})
            break
        tcp = c.tcp()
        if tcp is None:
            print("  %s%02d tcp 查询失败，跳过" % (tag, k))
            continue
        rec = {"t": time.time(), "q17": q1, "tcp14": tcp,
               "note": "%s%02d" % (tag, k)}
        fout.write(json.dumps(rec) + "\n")
        fout.flush()
        ok_n += 1
        arm = q1[side_idx:side_idx + 7]
        zc = tcp[2] if side == "left" else tcp[10]
        print("  %s%02d ok q=%s tcp_z=%.4f" % (tag, k, [round(x, 3) for x in arm], zc))
    fout.close()
    c.send({"command": "stop_motion"})
    print("完成 %d/%d → %s（stop_motion 已发）" % (ok_n, len(targets), fn))
    c.sock.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
