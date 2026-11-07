#!/usr/bin/env python3
"""T39 激励位姿生成器：为 URDF 数值重建采数生成离线位姿清单（纯离线，真机窗口只执行）。

为什么离线生成：真机窗口稀缺（用户级纪律），采数窗口只做「执行+验证」；
位姿清单要可审计（拒绝采样规则全部落盘），现场不现场发挥。

生成规则（每臂 N 个构型）：
  1. 关节范围 = get_config 实测限位（左右相同，见持久记忆/台账）内缩 inset；
  2. 禁区剔除（肘堵转教训，sweep_v2 同规则）：J2 < -1.5 且 J4 > 1.0 的收臂负载组合不采；
  3. 构型间距：与已选构型的关节 L2 距离 ≥ min_q_dist（避免近重复样本浪费窗口）；
  4. 覆盖验收：对生成清单算逐关节覆盖直方图 + 成对距离矩阵最小值，
     关节 10 分位覆盖宽度 < 60% 行程 → 报警（激励不足，需加样）；
  5. 腰固定 0（采数纪律，DH 链按固定基座拟合）。

输出：data/calib_poses_<side>_<N>.json（位姿列表 + 生成参数 + 覆盖统计）。

用法：python3 -I gen_calib_poses.py --side left --n 60 [--seed 42]
"""
import argparse
import json
import math
import os
import sys

import numpy as np

# get_config 实测软限位（左右臂相同，rad）——静态资料已与 5566 GUI 双源核对
LIMITS = [(-2.79, 2.79), (-2.356, 0.26), (-2.79, 2.79), (-0.69, 2.26),
          (-2.79, 2.79), (-1.57, 1.57), (-2.79, 2.79)]


def gen(n, seed, inset=0.15, min_q_dist=0.9):
    rng = np.random.default_rng(seed)
    lo = np.array([l + inset for l, h in LIMITS])
    hi = np.array([h - inset for l, h in LIMITS])
    poses = []
    tries = 0
    while len(poses) < n and tries < n * 2000:
        tries += 1
        q = rng.uniform(lo, hi)
        # 禁区：肘收臂负载组合（sweep_v2 事故位姿 J2=-2.1+J4=+1.2 的邻域）
        if q[1] < -1.5 and q[3] > 1.0:
            continue
        # 腕部（J6）不要顶死，留辨识空间
        if abs(q[5]) > 1.42 - inset:
            continue
        if poses:
            d = np.min(np.linalg.norm(np.array(poses) - q, axis=1))
            if d < min_q_dist:
                continue
        poses.append(q)
    return np.array(poses), tries


def coverage(poses):
    """逐关节覆盖宽度（10%~90% 分位行程占可用行程比例）。"""
    lo = np.array([l for l, h in LIMITS])
    hi = np.array([h for l, h in LIMITS])
    rows = []
    for j in range(7):
        span = hi[j] - lo[j]
        used = np.percentile(poses[:, j], 90) - np.percentile(poses[:, j], 10)
        rows.append(used / span)
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--side', choices=['left', 'right'], required=True)
    ap.add_argument('--n', type=int, default=60)
    ap.add_argument('--seed', type=int, default=42)
    ap.add_argument('--min-q-dist', type=float, default=0.9)
    ap.add_argument('--out', default='')
    args = ap.parse_args()

    poses, tries = gen(args.n, args.seed, min_q_dist=args.min_q_dist)
    if len(poses) < args.n:
        print(f'❌ 只生成 {len(poses)}/{args.n}（尝试 {tries} 次）——约束太紧，调 min-q-dist 或分批')
        sys.exit(1)
    cov = coverage(poses)
    dmat = np.linalg.norm(poses[:, None, :] - poses[None, :, :], axis=2)
    offdiag = dmat[~np.eye(len(poses), dtype=bool)]
    print(f'生成 {len(poses)} 个构型（{args.side}，seed={args.seed}，尝试 {tries} 次）')
    print(f'逐关节 10~90% 分位覆盖比例: {["%.0f%%" % (c*100) for c in cov]}')
    if min(cov) < 0.6:
        print('⚠️ 存在关节覆盖 <60%——激励不足，建议加样或放宽 min_q_dist')
    else:
        print('✅ 全关节覆盖 ≥60%')
    print(f'构型间最小 L2 距离 = {offdiag.min():.2f} rad（要求 ≥{args.min_q_dist}）')
    out = args.out or os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   'data', f'calib_poses_{args.side}_{args.n}.json')
    with open(out, 'w', encoding='utf-8') as f:
        json.dump({
            'side': args.side, 'n': len(poses), 'seed': args.seed,
            'limits_rad': LIMITS, 'inset': 0.15, 'min_q_dist': args.min_q_dist,
            'forbidden': 'J2<-1.5 且 J4>1.0（肘堵转禁区，sweep_v2 规则）',
            'coverage_10_90': [float(c) for c in cov],
            'min_pair_dist': float(offdiag.min()),
            'poses': [[float(v) for v in q] for q in poses],
        }, f, ensure_ascii=False, indent=1)
    print(f'已落盘: {out}')
    print('真机执行需配 sweep_calib.py（按 sweep_v2 护栏逐位姿驱动+采数），现场另备。')


if __name__ == '__main__':
    sys.exit(main())
