#!/usr/bin/env python3
"""URDF-vs-真机 FK 对账（T20.3，在 tc 主机 pink venv 里跑，本地无 pinocchio）。

用法（tc 主机，source pink venv 后）：
  <pink>/bin/python compare_fk.py <urdf路径> <calib_samples_xxx.jsonl> [--tool-len 0.145]

方法论（docs/UBT_URDF对账方法论.md 七条规则）：
  - 判决用残差：位置(mm) + 姿态(deg) 两个维度都报（M5 教训：只验位置漏 90°）；
  - 符号可救性判定：整体残差大时，枚举 s∈{±1}^7 × 零偏移（不用 LM，先看符号枚举
    能不能救），救得回=约定错（可修），救不回=结构性（URDF 重建，找控制器 D-H）；
  - 工具补偿：控制器 TCP 含 0.145m 末端工具（档案），TCP 应≈ Link7 原点 + R_link7·(0,0,-0.145)
    方向未知 → 同时报告「raw」与「-0.145m 沿工具 z」两种假设，看哪种残差塌缩。
输出：逐样本残差 + 汇总（max/median）+ 结论建议，全部打印（贴 WORKLOG）。
"""
import argparse
import json
import math
import sys

import numpy as np
import pinocchio as pin

URDF_JOINTS_R = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6', 'Joint7']
URDF_JOINTS_L = ['Joint1L', 'Joint2L', 'Joint3L', 'Joint4L', 'Joint5L', 'Joint6L', 'Joint7L']


def quat_wfirst_to_matrix(v):
    """协议四元数 (qw,qx,qy,qz) → 3x3 旋转矩阵。"""
    w, x, y, z = [float(c) for c in v]
    n = math.sqrt(w*w + x*x + y*y + z*z)
    if n < 1e-9:
        return np.eye(3)
    w, x, y, z = w/n, x/n, y/n, z/n
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)],
    ])


def load_samples(path):
    out = []
    with open(path, encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if line:
                out.append(json.loads(line))
    return out


def fk_frames(model, data, q_map):
    """按 URDF 关节名填 q，返回 {frame_name: SE3}。"""
    q = pin.neutral(model)
    for name, val in q_map.items():
        if model.existJointName(name):
            jid = model.getJointId(name)
            q[model.joints[jid].idx_q] = val
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    return {fid: data.oMf[fid] for fid in range(model.nframes)}


def residuals(T_model, p_tcp, R_tcp, tool_len, tool_sign):
    """两种工具假设下的末端残差。"""
    cands = {}
    p_model = np.array(T_model.translation)
    R_model = np.array(T_model.rotation)
    # 假设A：TCP == Link7 原点（raw）
    cands['raw'] = (p_tcp - p_model, pin.log3(R_model.T @ R_tcp))
    # 假设B：TCP = Link7 原点 + tool_sign*tool_len 沿工具 z
    p_model_tool = p_model + R_model @ np.array([0.0, 0.0, tool_sign * tool_len])
    cands['tool'] = (p_tcp - p_model_tool, pin.log3(R_model.T @ R_tcp))
    return cands


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('urdf')
    ap.add_argument('samples')
    ap.add_argument('--tool-len', type=float, default=0.145)
    ap.add_argument('--side', choices=['left', 'right'], default='left',
                    help='先对单臂判决（采数时另一臂保持不动即可）')
    args = ap.parse_args()

    model = pin.buildModelFromUrdf(args.urdf)
    data = model.createData()
    samples = load_samples(args.samples)
    print(f'URDF: {args.urdf}  样本: {len(samples)}  对账臂: {args.side}')

    names = URDF_JOINTS_L if args.side == 'left' else URDF_JOINTS_R
    # 协议 17 维：0=腰, 1..7=L1..L7, 8..14=R1..R7, 15/16=头
    q_off = 1 if args.side == 'left' else 8

    tcp_idx = 0 if args.side == 'left' else 7  # pose[14] = left7 + right7

    per_mode_acc = {'raw': [], 'tool': []}
    rows = []
    for k, s in enumerate(samples):
        q17 = s.get('q17')
        tcp = s.get('tcp14')
        if not q17 or not tcp or len(q17) < 15 or len(tcp) < 14:
            print(f'[{k}] 样本字段缺失，跳过'); continue
        q_map = {names[i]: float(q17[q_off + i]) for i in range(7)}
        frames = fk_frames(model, data, q_map)
        # 末端 frame：URDF 的 Link7 / Link7L
        fname = 'Link7L' if args.side == 'left' else 'Link7'
        fid = model.getFrameId(fname)
        T = frames[fid]
        p_tcp = np.array([float(c) for c in tcp[tcp_idx:tcp_idx+3]])
        R_tcp = quat_wfirst_to_matrix(tcp[tcp_idx+3:tcp_idx+7])
        line = f'[{k}] {s.get("note","")[:16]:16s}'
        for mode, (dp, dr) in residuals(T, p_tcp, R_tcp, args.tool_len, -1.0).items():
            mm = float(np.linalg.norm(dp)) * 1000
            deg = float(np.linalg.norm(dr)) * 180 / math.pi
            per_mode_acc[mode].append((mm, deg))
            line += f'  {mode}:{mm:7.1f}mm/{deg:6.1f}°'
        rows.append((k, p_tcp, R_tcp))
        print(line)

    print('\n==== 汇总（两种工具假设）====')
    for mode, acc in per_mode_acc.items():
        if not acc:
            continue
        arr = np.array(acc)
        print(f'{mode:5s}: 位置 max={arr[:,0].max():8.1f}mm median={np.median(arr[:,0]):8.1f}mm | '
              f'姿态 max={arr[:,1].max():7.1f}° median={np.median(arr[:,1]):7.1f}°')

    print('\n==== 符号可救性判定（s∈{±1}^7 零偏移枚举，位置残差）====')
    best = None
    for si in range(128):
        signs = [(1 if (si >> j) & 1 else -1) for j in range(7)]
        errs = []
        for k, s in enumerate(samples):
            q17, tcp = s.get('q17'), s.get('tcp14')
            if not q17 or not tcp or len(q17) < 15 or len(tcp) < 14:
                continue
            q_map = {names[i]: signs[i] * float(q17[q_off + i]) for i in range(7)}
            frames = fk_frames(model, data, q_map)
            fid = model.getFrameId('Link7L' if args.side == 'left' else 'Link7')
            T = frames[fid]
            p_tcp = np.array([float(c) for c in tcp[tcp_idx:tcp_idx+3]])
            errs.append(float(np.linalg.norm(p_tcp - np.array(T.translation))) * 1000)
        if errs:
            m = float(np.median(errs))
            if best is None or m < best[0]:
                best = (m, signs)
    if best:
        print(f'最优符号组合 s={best[1]} → 位置 median={best[0]:.1f}mm')
        if best[0] < 20.0:
            print('结论：符号枚举可救 → 是约定/符号问题（可映射或重建修正）')
        else:
            print('结论：符号枚举救不回 → 轴系/零位结构性差异 → 走「同源重建」路线'
                  '（向厂商索控制器 D-H/运动学参数，或 get_config/固件逆向）')


if __name__ == '__main__':
    sys.exit(main())
