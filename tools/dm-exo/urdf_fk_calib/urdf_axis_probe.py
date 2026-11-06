#!/usr/bin/env python3
"""URDF 关节轴方向探针（T20 符号层修复的实测依据）。

对每个关节：从当前位姿做 +0.12rad 微扰（canfd 逐帧反馈同步），测真机 TCP 的
位置增量与姿态增量；同姿势下用 URDF FK 雅可比算模型预测方向。两个点积：
  - 姿态轴点积（工具长度无关，主判据）：log3(R1·R0ᵀ) 轴 vs 雅可比 ω 列
  - 位置点积（参考，腕关节受 0.145m 工具偏移影响）
点积 < -0.5 → INVERTED（需翻转 URDF 该关节 axis）；> +0.5 → ALIGNED；其余 UNCERTAIN。

用法（tc 主机 pink venv，bridge 已停）：
  pink/bin/python urdf_axis_probe.py <urdf路径> [left|right|both]
输出：逐关节判定 + 翻转表（可直接用于改 URDF）。
"""
import json
import math
import socket
import sys
import time

import numpy as np
import pinocchio as pin

HOST, PORT = "192.168.61.60", 30485
HOLD = -100.0
PERT = 0.12     # 微扰幅值（rad）
SETTLE = 0.45

# get_config 实测限位内缩（左右相同）
LIM = [(-2.64, 2.64), (-2.21, 0.11), (-2.64, 2.64), (-0.54, 2.11),
       (-2.64, 2.64), (-1.42, 1.42), (-2.64, 2.64)]


class Ctrl:
    def __init__(self):
        self.sock = socket.create_connection((HOST, PORT), timeout=5)
        self.sock.settimeout(0.05)
        self.buf = ""

    def send(self, p):
        self.sock.sendall((json.dumps(p) + "#").encode())

    def pump(self, t=0.03):
        t0 = time.monotonic()
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
                        return json.loads(raw)
                    except json.JSONDecodeError:
                        pass
        return None

    def rpc(self, payload, key, timeout=1.5):
        self.send(payload)
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            f = self.pump(0.05)
            if f and key in f:
                return f
        return None

    def q(self):
        f = self.rpc({"command": "get_joint_position"}, "joint")
        return f["joint"] if f else None

    def tcp(self):
        f = self.rpc({"command": "get_tcp_pose"}, "pose")
        return f["pose"] if f else None

    def dev(self):
        f = self.rpc({"command": "get_device_state"}, "state")
        return (f or {}).get("state", "?")

    def drive_to(self, q17, side_idx, target, step=0.005, timeout=45):
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            real = self.q()
            if real is None:
                continue
            q17 = real
            delta = [target[i] - q17[side_idx + i] for i in range(7)]
            if max(abs(d) for d in delta) < 0.008:
                return True, q17
            for _ in range(4):
                delta = [target[i] - q17[side_idx + i] for i in range(7)]
                if max(abs(d) for d in delta) < 0.008:
                    return True, q17
                frame = list(q17)
                for i in range(7):
                    frame[side_idx + i] = q17[side_idx + i] + max(
                        -step, min(step, delta[i]))
                frame[0] = HOLD
                frame[15] = HOLD
                frame[16] = HOLD
                self.send({"command": "movej_canfd", "joint": frame, "freq": 50})
                self.pump(0.03)
                q17 = frame
        return False, q17


def quat_to_R(v):
    w, x, y, z = [float(c) for c in v]
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def main():
    urdf = sys.argv[1]
    which = sys.argv[2] if len(sys.argv) > 2 else "both"
    model = pin.buildModelFromUrdf(urdf)
    data = model.createData()
    JOINTS = {
        "left": (["Joint1L", "Joint2L", "Joint3L", "Joint4L", "Joint5L",
                  "Joint6L", "Joint7L"], "Link7L", 1),
        "right": (["Joint1", "Joint2", "Joint3", "Joint4", "Joint5",
                   "Joint6", "Joint7"], "Link7", 8),
    }

    c = Ctrl()
    st = c.dev()
    print("初始 state:", st)
    if st not in ("ready", "idle"):
        print("!! 控制器非就绪")
        return 1

    flips = []
    for side in ("left", "right"):
        if which != "both" and side != which:
            continue
        names, fname, idx = JOINTS[side]
        fid = model.getFrameId(fname)
        print("== %s 臂微扰实测（每关节 +%.2frad 后回原位）==" % (side, PERT))
        for j in range(7):
            st = c.dev()
            if st not in ("ready", "idle"):
                print("!! state=%s → 全停" % st)
                c.send({"command": "stop_motion"})
                break
            q0 = c.q()
            tcp0 = c.tcp()
            if q0 is None or tcp0 is None:
                print("  J%d 查询失败，跳过" % (j + 1))
                continue
            base = [q0[idx + i] for i in range(7)]
            tgt = base[:]
            tgt[j] = max(LIM[j][0], min(LIM[j][1], base[j] + PERT))
            if abs(tgt[j] - base[j]) < 0.05:
                print("  J%d 距限位过近，跳过" % (j + 1))
                continue
            # 微扰出去
            ok, _ = c.drive_to(q0, idx, tgt)
            time.sleep(SETTLE)
            q1 = c.q()
            tcp1 = c.tcp()
            # 回原位（守卫在回位动作之前，防 None 崩溃——v1 教训）
            if q1 is not None:
                back = q1[:]
                for i in range(7):
                    back[idx + i] = base[i]
                c.drive_to(q1, idx, back)
                time.sleep(SETTLE)
            if not ok or q1 is None or tcp1 is None:
                print("  J%d 未到位或读取失败，跳过" % (j + 1))
                continue
            dq = [(q1[idx + i] - base[i]) for i in range(7)]
            if abs(dq[j]) < 0.03:
                print("  J%d 实际位移过小(%.3f)，结果不可信" % (j + 1, dq[j]))
                continue
            # 真机增量
            dp_real = np.array([tcp1[i] - tcp0[i] for i in range(3)])
            R0, R1 = quat_to_R(tcp0[3:7]), quat_to_R(tcp1[3:7])
            w_real = np.array(pin.log3(R1 @ R0.T))
            # 模型雅可比（LOCAL_WORLD_ALIGNED：v/ω 都在世界系）
            qq = pin.neutral(model)
            for i, n in enumerate(names):
                qq[model.joints[model.getJointId(n)].idx_q] = base[i]
            pin.forwardKinematics(model, data, qq)
            pin.updateFramePlacements(model, data)
            J = pin.computeFrameJacobian(model, data, qq, fid,
                                         pin.LOCAL_WORLD_ALIGNED)
            dp_mod = J[:3, j] * dq[j]
            w_mod = J[3:, j] * dq[j]
            rot_dot = float(np.dot(w_real, w_mod) /
                            (np.linalg.norm(w_real) * np.linalg.norm(w_mod) + 1e-12))
            pos_dot = float(np.dot(dp_real, dp_mod) /
                            (np.linalg.norm(dp_real) * np.linalg.norm(dp_mod) + 1e-12))
            verdict = ("ALIGNED" if rot_dot > 0.5 else
                       "INVERTED" if rot_dot < -0.5 else "UNCERTAIN")
            if verdict == "INVERTED":
                flips.append((side, j))
            print("  J%d %-9s rot_dot=%+.2f pos_dot=%+.2f |Δq|=%.3f "
                  "|Δp_real|=%.0fmm |Δp_mod|=%.0fmm"
                  % (j + 1, verdict, rot_dot, pos_dot, abs(dq[j]),
                     np.linalg.norm(dp_real) * 1000,
                     np.linalg.norm(dp_mod) * 1000))
    print("\n==== 翻转表（URDF 需翻转 axis 的关节）====")
    for side, j in flips:
        print("  %s J%d" % (side, j + 1))
    if not flips:
        print("  （无——全部同向）")
    c.send({"command": "stop_motion"})
    c.sock.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
