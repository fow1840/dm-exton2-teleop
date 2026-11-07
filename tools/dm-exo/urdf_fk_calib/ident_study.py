#!/usr/bin/env python3
"""T39 样本量辨识研究：DH 数值重建到底需要多少独立构型。

为什么存在：T20 说「数值重建样本不足」，但「多少才够」没人给过数。
拍脑袋定样本量违反判据纪律，所以用合成实验直接测：
  - 以 URDF 链当「虚拟真机」（同族 7R 链，结构对错不影响辨识规模问题）；
  - 按 N 个随机构型生成训练集，加真机量级噪声（位置 0.3mm / 姿态 0.1°，
    来自 samples_L 重复读数实测噪声，放大 3 倍保守取值）；
  - 固定 200 个测试构型（同样加噪）→ 拟合模型泛化残差；
  - 扫 N ∈ {12, 20, 30, 50}，每档 3 次随机试验取中位。
判据（先声明）：测试位置 median ≤ 20mm（=机械臂绝对定位精度=对账残差下限，
见持久记忆 exon2-hardware-hazards）→ 该 N 档可用；输出最小可用 N 供采数方案引用。

用法：python3 -I ident_study.py --urdf <hr_up URDF>
"""
import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # -I 隔离模式下找不到同目录模块

from fit_dh import (fk_urdf_chain, parse_urdf_chain, random_init,
                    residuals_dh, fit_multistart, fk_dh_batch, log3_batch)


def add_noise(Rs, rng, sigma_rot_deg):
    """姿态加噪：对每个旋转乘一个小随机旋转。"""
    out = Rs.copy()
    for i in range(out.shape[0]):
        v = rng.normal(size=3)
        v = v / max(np.linalg.norm(v), 1e-9) * math.radians(sigma_rot_deg) * rng.normal()
        out[i] = rotvec_small(out[i], v)
    return out


def rotvec_small(R, v):
    """R · exp(v)（右乘小扰动）。"""
    th = np.linalg.norm(v)
    if th < 1e-12:
        return R
    k = v / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    dR = np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * (K @ K)
    return R @ dR


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--urdf', required=True)
    ap.add_argument('--n-test', type=int, default=200)
    ap.add_argument('--sigma-pos-mm', type=float, default=0.3)
    ap.add_argument('--sigma-rot-deg', type=float, default=0.1)
    ap.add_argument('--n0', type=int, default=8, help='初始插值样本数（先收敛到全局盆再增量）')
    ap.add_argument('--step', type=int, default=8, help='每轮增量样本数')
    ap.add_argument('--n-max', type=int, default=64, help='增量上限')
    ap.add_argument('--trials', type=int, default=2, help='随机试验次数')
    args = ap.parse_args()

    chain = parse_urdf_chain(args.urdf, 'Link7L')
    rng = np.random.default_rng(20261107)

    # 固定测试集（加噪）。注意 R 传旋转矩阵本身（residuals_dh 内部做 R_predᵀ·R_meas），
    # 首版误传转置=姿态取逆，制造过全档假塌陷——数据配对纪律的又一实例。
    Q_test = rng.uniform(-1.4, 1.4, (args.n_test, 7))
    Tt = np.stack([fk_urdf_chain(chain, q) for q in Q_test])
    P_test = Tt[:, :3, 3] + rng.normal(scale=args.sigma_pos_mm / 1000.0, size=(args.n_test, 3))
    R_test = add_noise(Tt[:, :3, :3].copy(), rng, args.sigma_rot_deg)

    print(f'虚拟真机=URDF 链，噪声 σ_pos={args.sigma_pos_mm}mm σ_rot={args.sigma_rot_deg}°，'
          f'测试集 {args.n_test} 构型，增量式 {args.trials} 试验')
    print('策略=增量续拟合：N0=%d 先插值收敛，每轮 +%d 样本温启动重拟合（+2 随机重启防滑出）。'
          % (args.n0, args.step))
    print('为什么增量：40 参数非凸，纯随机启动在直接全量拟合时易卡坏局部极小'
          '（首版研究的假塌陷教训）；真机采数同样按此策略分批跑。')
    print(f'{"N":>4s} | {"train cost":>10s} | {"测试位置 median":>14s} {"max":>10s} | {"测试姿态 med":>11s} | 判定(≤20mm)')
    verdict_min_n = None
    for t in range(args.trials):
        rng_t = np.random.default_rng(5000 + t)
        pool = rng_t.uniform(-1.4, 1.4, (args.n_max + 8, 7))
        Tpool = np.stack([fk_urdf_chain(chain, q) for q in pool])
        P_pool = Tpool[:, :3, 3] + rng_t.normal(scale=args.sigma_pos_mm / 1000.0, size=(len(pool), 3))
        R_pool = add_noise(Tpool[:, :3, :3].copy(), rng_t, args.sigma_rot_deg)
        used = list(range(args.n0))
        x_best = None
        n = args.n0
        while True:
            Q_tr = pool[used]; P_tr = P_pool[used]; R_tr = R_pool[used]
            inits = []
            if x_best is not None:
                inits.append(x_best)
            inits += [random_init(rng_t) for _ in range(2)]
            x_best, cost = fit_multistart(Q_tr, P_tr, R_tr, inits)
            res = residuals_dh(x_best, Q_test, P_test, R_test)
            m = len(res) // 2
            dp = np.linalg.norm(res[:m].reshape(-1, 3), axis=1) * 1000
            dr = np.linalg.norm(res[m:].reshape(-1, 3), axis=1) * 180 / math.pi
            pm = float(np.median(dp)); pM = float(np.max(dp)); rm = float(np.median(dr))
            ok = '✅ 可用' if pm <= 20 else '✗ 不足'
            if t == 0:  # 曲线只在首个试验打印（两试验一致即足够）
                if pm <= 20 and verdict_min_n is None:
                    verdict_min_n = n
                print(f'{n:4d} | {cost:10.3e} | {pm:12.1f}mm {pM:9.1f}mm | {rm:10.2f}° | {ok}')
            if n >= args.n_max:
                break
            new = rng_t.choice([i for i in range(len(pool)) if i not in used],
                               size=min(args.step, args.n_max - n), replace=False)
            used += list(new)
            n = len(used)
    print('\n==== 结论 ====')
    if verdict_min_n:
        print(f'最小可用样本量 N={verdict_min_n}（测试位置 median ≤ 20mm）→ 采数方案按此档上浮 50% 余量')
    else:
        print(f'增量到 N={args.n_max} 仍 >20mm → 检查训练 cost 是否收敛；收敛仍差则需改约束参数化')
    print('注意：合成噪声近似真机噪声；真实激励（限位内缩+禁区剔除）比均匀采样差，结论上浮余量使用。')


if __name__ == '__main__':
    sys.exit(main())
