#!/usr/bin/env python3
"""离线方向审计：从遥操录包量化「IK 指令 ↔ 真机反馈」的跟踪质量与敏感度分布。

为什么存在（T30/T29 配套，2026-11-06）：
  bridge 把 ik 数值原样透传给控制器，控制器按自己的约定闭环到该数值——所以
  ik↔rob 的**数值**相关恒为正，**物理轴翻转在数值层面不可见**（翻转发生在
  「数值→物理方向」层，只有 T30 的 get_tcp_pose 单轴微扰能观测）。
  本工具回答的是录包能回答的另外三个问题：
    ① 固件是否做了数值反演/某关节数值不跟踪（排除一层嫌疑）；
    ② 每关节跟踪滞后与增益（健康窗 vs 半执行窗），供 sixdir 录制协议设计；
    ③ 外骨骼末端每轴增量驱动哪些关节（软件侧期望表 A_ik，T30 探针判读的对照物）。

方法：
  1) 离合按住窗内取 DWIN 帧差分，|Δrob|≥门槛过滤冻结/静置窗；
  2) (d_ik_j, d_rob_j) 逐关节 ±滞后搜索相关 → 跟踪质量（TRACK/WEAK/DEAD）；
  3) 逐按压段净量符号投票 + 增益 median(|Δrob_j|/|Δik_j|)（仅 |Δik_j|>0.03rad 段）；
  4) 外骨骼末端 (dx,dy,dz) 增量 → 关节增量的最小二乘敏感表（ik 与 rob 各一张）。

用法：
  python3 offline_direction_audit.py --data <录包目录> --side left|right
录包为 record_compare.py 的 npz（字段带前导斜杠）；14 关节序 = Joint1..7(右) + Joint1L..7L(左)。
"""
import argparse
import glob
import os

import numpy as np

DWIN = 5            # 增量窗（rob 50Hz × 5 ≈ 100ms）
MIN_ROB_STEP = 0.005    # 运动窗门槛：|Δrob|rad/窗
MIN_SEG_STEP = 0.05     # 净量段门槛：|Δrob|rad/段（≈2.9°）
MIN_IK_EXCUR = 0.03     # 增益计算时指令最小行程（rad）
MIN_SEG_LEN = 10        # 段最短帧数（0.2s）
GAP_MERGE = 5           # 段间缝隙合并（帧）
LAGS = list(range(-2, 41))   # rob 相对 ik 滞后搜索（帧，≤800ms）
CONF_R = 0.6            # 判定置信门限
MIN_EE_STEP = 0.002     # 末端敏感表样本门槛：|Δp|≥2mm/窗


def load(path):
    d = np.load(path, allow_pickle=True)
    return {k: d[k] for k in d.files}


def pressed_mask_on_rob(cl_t, cl_v, rob_t):
    """把离合信号插值到 rob 时间轴，>0.5 视为按住。"""
    if cl_t[-1] < rob_t[0] or cl_t[0] > rob_t[-1]:
        return np.zeros(len(rob_t), bool)
    return np.interp(rob_t, cl_t, cl_v.astype(float)) > 0.5


def segments(mask, min_len=MIN_SEG_LEN):
    """True 段提取 + 短缝合并 + 去超短段。"""
    segs, k, n = [], 0, len(mask)
    while k < n:
        if mask[k]:
            e = k
            while e + 1 < n and mask[e + 1]:
                e += 1
            segs.append([k, e])
            k = e + 1
        else:
            k += 1
    merged = []
    for s in segs:
        if merged and s[0] - merged[-1][1] <= GAP_MERGE:
            merged[-1][1] = s[1]
        else:
            merged.append(s)
    return [s for s in merged if s[1] - s[0] >= min_len]


def safe_corr(x, y):
    sx, sy = x.std(), y.std()
    if sx < 1e-9 or sy < 1e-9:
        return 0.0
    x = x - x.mean()
    y = y - y.mean()
    return float((x * y).mean() / (sx * sy))


def lagged_best(x, y):
    """y（rob）相对 x（ik）滞后 lag 帧内找 |corr| 最大值。lag>0=rob 更晚。"""
    best_r, best_l = 0.0, 0
    n = len(x)
    for lag in LAGS:
        if lag > 0 and n > lag:
            r = safe_corr(x[:-lag], y[lag:])
        elif lag < 0 and n > -lag:
            r = safe_corr(x[-lag:], y[:lag])
        else:
            r = safe_corr(x, y)
        if abs(r) > abs(best_r):
            best_r, best_l = r, lag
    return best_r, best_l


def audit(bags, side_idx, side_name):
    """跨包累积增量样本与净量段，输出跟踪质量 + 增益 + 敏感表。"""
    DIK, DROB, DEE = [], [], []
    NET = []
    used, skipped = [], []
    ee_key = f'/ee_{side_name}_state_v'
    ee_tkey = f'/ee_{side_name}_state_t'
    for path in bags:
        try:
            d = load(path)
        except Exception as exc:
            skipped.append((os.path.basename(path), f'加载失败 {exc}'))
            continue
        need = ['/rob_t', '/rob_v', '/ik_t', '/ik_v']
        if any(k not in d for k in need):
            skipped.append((os.path.basename(path), '缺核心字段'))
            continue
        cl_k = f'/clutch_{side_name}_v'
        if cl_k not in d:
            skipped.append((os.path.basename(path), f'无 {cl_k}'))
            continue
        rob_t, rob_v = d['/rob_t'], d['/rob_v']
        ik_t, ik_v = d['/ik_t'], d['/ik_v']
        if rob_t[-1] < ik_t[0] or rob_t[0] > ik_t[-1]:
            skipped.append((os.path.basename(path), '时间轴不重叠'))
            continue
        ik_on_rob = np.stack([np.interp(rob_t, ik_t, ik_v[:, j])
                              for j in range(ik_v.shape[1])], axis=1)
        has_ee = ee_key in d and ee_tkey in d and d[ee_key].shape[1] >= 3
        ee_on_rob = None
        if has_ee:
            ee = d[ee_key]
            ee_on_rob = np.stack([np.interp(rob_t, d[ee_tkey], ee[:, c])
                                  for c in range(3)], axis=1)
        cl_v = d[cl_k][:, 0]
        cl_t = d[f'/clutch_{side_name}_t']
        mask = pressed_mask_on_rob(cl_t, cl_v, rob_t)
        segs = segments(mask)
        n_samp = 0
        for s0, s1 in segs:
            for k in range(s0, s1 - DWIN):
                drob = rob_v[k + DWIN, side_idx] - rob_v[k, side_idx]
                dik = ik_on_rob[k + DWIN, side_idx] - ik_on_rob[k, side_idx]
                if np.linalg.norm(drob) < MIN_ROB_STEP:
                    continue
                if np.max(np.abs(dik)) > 1.5:   # 指令野点（重启/跳变）剔除
                    continue
                DIK.append(dik)
                DROB.append(drob)
                if ee_on_rob is not None:
                    dee = ee_on_rob[k + DWIN] - ee_on_rob[k]
                    if np.linalg.norm(dee) >= MIN_EE_STEP:
                        DEE.append(dee)
                    else:
                        DEE.append(np.full(3, np.nan))   # 占位对齐
                n_samp += 1
            d_rob = rob_v[s1, side_idx] - rob_v[s0, side_idx]
            d_ik = ik_on_rob[s1, side_idx] - ik_on_rob[s0, side_idx]
            if np.linalg.norm(d_rob) >= MIN_SEG_STEP and np.max(np.abs(d_ik)) < 1.5:
                NET.append((d_ik, d_rob))
        (used if n_samp >= 30 else skipped).append(
            (os.path.basename(path), f'{n_samp} 样本/{len(segs)} 段'
             if n_samp else f'{len(segs)} 段全冻结（按住但真机未动）'))
    DIK = np.array(DIK) if DIK else np.zeros((0, 7))
    DROB = np.array(DROB) if DROB else np.zeros((0, 7))
    print(f'\n===== {side_name.upper()} 臂审计 =====')
    print(f'可用包: {len(used)}  跳过: {len(skipped)}  增量样本: {len(DIK)}  净量段: {len(NET)}')
    for nm, why in used:
        print(f'  [用] {nm}: {why}')
    for nm, why in skipped:
        print(f'  [弃] {nm}: {why}')
    if len(DIK) < 100:
        print('  样本不足（<100），该臂无法离线判定')
        return None
    labels = [f'J{i+1}' for i in range(7)]

    # ① 每关节跟踪质量（对角线即所需；非对角=指令相关性带来的自然耦合，仅供参照）
    print('\n指令号跟踪（ik→rob 数值层；恒应≈+1。物理轴翻转在此层面不可见，见 T30）:')
    verdicts = []
    for j in range(7):
        r, lag = lagged_best(DIK[:, j], DROB[:, j])
        if abs(r) >= 0.8:
            v = f'TRACK 强 r={r:+.2f} lag={lag}帧'
        elif abs(r) >= CONF_R:
            v = f'TRACK 中 r={r:+.2f} lag={lag}帧'
        elif abs(r) >= 0.3:
            v = f'WEAK r={r:+.2f} lag={lag}帧（激励不足或间歇跟踪）'
        else:
            v = f'DEAD/无激励 r={r:+.2f}'
        verdicts.append((j, r, lag, v))
        print(f'  {labels[j]}: {v}')

    # ② 净量投票 + 增益
    V = np.zeros((7, 7), int)
    gains = [[] for _ in range(7)]
    for d_ik, d_rob in NET:
        V += np.outer(np.sign(d_ik), np.sign(d_rob)).astype(np.int64)
        for j in range(7):
            if abs(d_ik[j]) > MIN_IK_EXCUR:
                gains[j].append(abs(d_rob[j]) / abs(d_ik[j]))
    print('\n净量符号投票（行=ik，列=rob；段数=%d）与跟踪增益:' % len(NET))
    for j in range(7):
        g = float(np.median(gains[j])) if gains[j] else float('nan')
        diag = V[j, j]
        print(f'  {labels[j]}: 对角投票 {diag:+d}  增益 median={g:.2f}'
              f'（n={len(gains[j])}）'
              + ('  ⚠疑似不执行' if gains[j] and g < 0.3 else ''))

    # ③ 末端敏感表（软件侧期望 ik + 机器实际 rob 各一张）
    DEE = np.array(DEE) if DEE else np.zeros((0, 3))
    ok = ~np.isnan(DEE[:, 0]) if len(DEE) else np.zeros(0, bool)
    print(f'\n外骨骼末端增量敏感表（样本 {int(ok.sum()) if len(ok) else 0}，单位 rad/m；'
          f'ik=软件期望，rob=机器数值实际）:')
    if ok.sum() >= 30:
        for tag, arr in (('ik', DIK[ok]), ('rob', DROB[ok])):
            B = np.linalg.lstsq(DEE[ok], arr, rcond=None)[0]
            print(f'  [{tag}] 关节: ' + '  '.join(
                f'{labels[j]}={"XYZ"[int(np.argmax(np.abs(B[:, j])))]}'
                f'{"+" if B[int(np.argmax(np.abs(B[:, j]))), j] > 0 else "-"}'
                f'{B[int(np.argmax(np.abs(B[:, j]))), j]:+.1f}' for j in range(7)))
    else:
        print('  （末端增量有效样本不足）')
    return {'verdicts': verdicts, 'V': V, 'n_samp': len(DIK), 'n_seg': len(NET)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--data', required=True, help='录包目录（含 bags/ 递归 glob）')
    ap.add_argument('--side', required=True, choices=['left', 'right'])
    args = ap.parse_args()
    bags = sorted(glob.glob(os.path.join(args.data, '**', '*.npz'), recursive=True))
    if not bags:
        raise SystemExit(f'目录无 npz: {args.data}')
    # 关节序约定：前 7 无后缀=右臂，后 7 带 L=左臂（rob_names/ik_names 实测一致）
    idx = np.arange(7, 14) if args.side == 'left' else np.arange(0, 7)
    print(f'数据目录: {args.data}  包数: {len(bags)}  臂: {args.side}（14 关节中取 {list(idx)}）')
    audit(bags, idx, args.side)


if __name__ == '__main__':
    main()
