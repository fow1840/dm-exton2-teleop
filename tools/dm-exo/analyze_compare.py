#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
对比分析：你的手臂(外骨骼) vs IK目标 vs 真机反馈。

输入：record_compare.py 保存的 .npz
输出：4 页 PNG + 文字摘要(stdout + .txt)

指标：
  - 关节级：IK目标→真机反馈 的滞后(互相关)、跟踪误差RMS/最大、相关性
  - 臂级：  你的手臂运动能量 vs 真机运动能量 的相关与滞后（跟手性/方向）
  - 离合级：每次按下/按住期间 手动 vs IK目标漂移 vs 真机（判定"按下就动"成因）
  - 末端级：IK末端目标 vs 末端实际 的滞后与误差
"""
import sys

import numpy as np
from scipy.signal import correlate

GRID_HZ = 100.0
MAX_LAG_S = 1.0
MOVE_THRESH_RAD = 0.03   # 关节活动判定：std 阈值
CLUTCH_STILL_STD = 0.01  # 手静判定：按住期间 exo 关节 std (rad)
CLUTCH_DRIFT_RAD = 0.05  # IK 自漂判定：按住期间 IK 目标变化幅度 (rad)


def resample(t: np.ndarray, v: np.ndarray, grid: np.ndarray) -> np.ndarray:
    if v.ndim == 1:
        return np.interp(grid, t, v)
    return np.vstack([np.interp(grid, t, v[:, i]) for i in range(v.shape[1])]).T


def lag_corr(a: np.ndarray, b: np.ndarray, dt: float):
    """返回 (lag_s, corr)：b 相对 a 的最优滞后（b 滞后为正）。"""
    a_ = a - a.mean()
    b_ = b - b.mean()
    n = len(a_)
    max_lag = int(MAX_LAG_S / dt)
    full = correlate(b_, a_, mode='full')
    mid = n - 1
    lo, hi = max(0, mid - max_lag), min(len(full), mid + max_lag + 1)
    seg = full[lo:hi]
    k = np.argmax(seg) + lo
    lag = (k - mid) * dt
    denom = (np.linalg.norm(a_) * np.linalg.norm(b_))
    corr = full[k] / denom if denom > 1e-12 else 0.0
    return lag, corr


def main() -> None:
    path = sys.argv[1] if len(sys.argv) > 1 else '/home/tc/桌面/dm-exo/rec.npz'
    z = np.load(path, allow_pickle=True)
    lines = []

    def log(s=''):
        print(s)
        lines.append(s)

    T_end = float(max(z[k].max() for k in z.files if k.endswith('_t') and len(z[k])))
    grid = np.arange(0.0, T_end, 1.0 / GRID_HZ)
    dt = 1.0 / GRID_HZ

    # ---------- 数据准备 ----------
    rob_names = [str(n) for n in z['/rob_names']] if '/rob_names' in z.files else []
    exo_names = [str(n) for n in z['/exo_names']] if '/exo_names' in z.files else []

    rob_t, rob_v = z['/rob_t'], z['/rob_v']
    ik_t, ik_v = z['/ik_t'], z['/ik_v']
    exo_t, exo_v = z['/exo_t'], z['/exo_v']

    log(f'录制时长 {T_end:.1f}s | 帧数: 外骨骼{len(exo_t)} IK目标{len(ik_t)} 真机{len(rob_t)}')
    log(f'录制期平均频率: 外骨骼{len(exo_t)/T_end:.0f}Hz IK{len(ik_t)/T_end:.0f}Hz 真机{len(rob_t)/T_end:.0f}Hz')
    log('')

    # ---------- 关节级：IK 目标 vs 真机反馈 ----------
    rob_g = resample(rob_t, rob_v, grid)
    ik_g = resample(ik_t, ik_v, grid)

    log('== 关节级：IK目标 → 真机反馈（滞后为正=反馈落后） ==')
    log(f'{"关节":<10}{"滞后ms":>8}{"相关":>8}{"RMS_err":>9}{"max_err":>9}{"活动":>6}')
    joint_bad = []
    for i, name in enumerate(rob_names):
        tgt, fb = ik_g[:, i], rob_g[:, i]
        active = tgt.std() > MOVE_THRESH_RAD or fb.std() > MOVE_THRESH_RAD
        if not active:
            continue
        lag, corr = lag_corr(tgt, fb, dt)
        err = fb - tgt
        log(f'{name:<10}{lag*1000:>8.0f}{corr:>8.3f}'
            f'{np.sqrt(np.mean(err**2)):>9.3f}{np.abs(err).max():>9.3f}{"是":>6}')
        if corr < 0.5:
            joint_bad.append(name)
    if not rob_names:
        log('（无关节数据）')
    if joint_bad:
        log(f'⚠️ 低相关关节: {joint_bad}')
    else:
        log('✅ 所有活动关节相关>0.5，桥接追跡方向一致')
    log('')

    # ---------- 臂级：你的手臂 vs 真机（运动能量对比） ----------
    log('== 臂级：你的手臂(外骨骼) → 真机 运动能量对比 ==')
    exo_g = resample(exo_t, exo_v, grid)
    # 外骨骼: 前6=左臂, 后6=右臂（left_1..6, right_1..6）
    exo_left = exo_g[:, :6] if len(exo_names) == 12 else exo_g[:, :6]
    exo_right = exo_g[:, 6:12] if len(exo_names) == 12 else exo_g[:, 6:12]
    rob_left = rob_g[:, 7:14]    # Joint1L..7L
    rob_right = rob_g[:, 0:7]    # Joint1..7

    def energy(v: np.ndarray) -> np.ndarray:
        vel = np.linalg.norm(np.diff(v, axis=0), axis=1) * GRID_HZ
        vel = np.convolve(vel, np.ones(25) / 25, mode='same')  # 250ms 平滑
        return vel

    for side, e_op, e_rb in (('左臂', energy(exo_left), energy(rob_left)),
                             ('右臂', energy(exo_right), energy(rob_right))):
        lag, corr = lag_corr(e_op, e_rb, dt)
        op_move = e_op.max() > 0.05
        rb_move = e_rb.max() > 0.05
        status = ('✅ 同向跟随' if corr > 0.4 else
                  ('⚠️ 相关弱' if corr > 0 else '❌ 疑似反向')) if op_move and rb_move else '（静止/未动）'
        log(f'{side}: 你动{op_move} 机动{rb_move} | 相关={corr:+.2f} 滞后={lag*1000:+.0f}ms 峰速你{e_op.max():.2f} 机{e_rb.max():.2f} rad/s → {status}')
    log('')

    # ---------- 离合按下事件：按住期间"手动 vs 机动"（诊断按下就动） ----------
    def hold_grid(t_evt, v_evt, g):
        """稀疏 Bool 事件按"保持上一值"重采样到统一网格。"""
        out = np.zeros(len(g))
        idx = np.searchsorted(t_evt, g, side='right') - 1
        ok = idx >= 0
        out[ok] = np.asarray(v_evt).ravel()[idx[ok]]
        return out

    clutch_any_g = None
    clutch_segs = []   # (t_start, t_end) 用于绘图阴影
    log('== 离合按下事件：按住期间 手动 vs IK目标漂移 vs 真机 ==')
    log(f'判据：手静(exo std<{CLUTCH_STILL_STD}rad)但IK目标仍漂移(>{CLUTCH_DRIFT_RAD}rad) '
        f'→ IK自身回拉（非命令延迟/队列）')
    for side in ('left', 'right'):
        kt = f'/clutch_{side}_t'
        if kt not in z.files or len(z[kt]) == 0:
            log(f'{side}: 未录到 /clutch/{side}（老版本录制器无此通道）')
            continue
        c = hold_grid(z[kt], z[f'/clutch_{side}_v'], grid)
        ag = hold_grid(z[f'/clutch_left_t'], z[f'/clutch_left_v'], grid) if '/clutch_left_t' in z.files else c
        bg = hold_grid(z[f'/clutch_right_t'], z[f'/clutch_right_v'], grid) if '/clutch_right_t' in z.files else c
        clutch_any_g = np.maximum(ag, bg)
        d = np.diff(c)
        risings = np.where(d == 1)[0] + 1
        fallings = np.where(d == -1)[0] + 1
        if c[0] == 1:
            risings = np.concatenate([[0], risings])
        if len(fallings) < len(risings):
            fallings = np.concatenate([fallings, [len(grid)]])
        exo_side = exo_g[:, :6] if side == 'left' else exo_g[:, 6:12]
        n_evt = 0
        for k, (a, b) in enumerate(zip(risings, fallings)):
            b = min(b, len(grid) - 1)
            dur = grid[b] - grid[a]
            if dur < 0.5:   # 抖动太短，忽略
                continue
            n_evt += 1
            clutch_segs.append((grid[a], grid[b]))
            exo_std = float(exo_side[a:b].std())
            ik_range = ik_g[a:b].max(axis=0) - ik_g[a:b].min(axis=0)
            rob_range = rob_g[a:b].max(axis=0) - rob_g[a:b].min(axis=0)
            j = int(np.argmax(ik_range))
            name = rob_names[j] if j < len(rob_names) else f'#{j}'
            # 按下瞬跳诊断：目标流断档（Puppetry 松开时停发）+ 按下后0.3s内IK位移
            t_press = grid[a]
            kt_t, kt_v = f'/ee_{side}_tgt_t', f'/ee_{side}_tgt_v'
            gap_ms = float('nan')
            tgt_jump_mm = float('nan')
            if kt_t in z.files and len(z[kt_t]):
                tt, tv = z[kt_t], z[kt_v][:, :3]
                idx = int(np.searchsorted(tt, t_press))
                if 0 < idx < len(tt):
                    gap_ms = (t_press - tt[idx - 1]) * 1000.0
                    tgt_jump_mm = float(np.linalg.norm(tv[idx] - tv[idx - 1])) * 1000.0
            j0 = a
            j1 = min(a + int(0.3 * GRID_HZ), b)
            ik_inst = np.abs(ik_g[j0:j1] - ik_g[j0]).max(axis=0)
            ji = int(np.argmax(ik_inst))
            still = exo_std < CLUTCH_STILL_STD
            drift = float(ik_range[j]) > CLUTCH_DRIFT_RAD
            if gap_ms > 300 and float(ik_inst[ji]) > 0.02:
                verdict = '⚠️ 按下瞬跳：目标流断档后锚到陈旧目标（非延迟命令）'
            elif still and drift:
                verdict = '⚠️ 手静但IK目标自漂 → 姿态任务回拉实锤（非延迟命令）'
            elif still:
                verdict = '✅ 手静且IK不动（正常）'
            else:
                verdict = '正常跟随（按下时手在动）'
            gap_s = f'{gap_ms:.0f}ms' if gap_ms == gap_ms else 'n/a'
            jump_s = f'{tgt_jump_mm:.0f}mm' if tgt_jump_mm == tgt_jump_mm else 'n/a'
            log(f'{side} 按下#{n_evt} t={t_press:.1f}s 保持{dur:.1f}s | '
                f'手动std={exo_std:.3f}rad | IK瞬跳(0.3s){ik_inst[ji]*1000:.0f}mrad({rob_names[ji] if ji < len(rob_names) else ji}) '
                f'IK漂移{ik_range[j]:.3f}rad({name}) 真机{rob_range[j]:.3f}rad | '
                f'目标断档{gap_s} 目标跳变{jump_s} → {verdict}')
        if n_evt == 0:
            log(f'{side}: 有数据但无≥0.5s的按住段')
    if clutch_any_g is None:
        log('（无任何离合通道 → 请用新版 record_compare.py 重录）')
    log('')

    # ---------- 末端级 ----------
    log('== 末端级：IK末端目标 vs 末端实际 ==')
    for side in ('left', 'right'):
        kt, kv = f'/ee_{side}_tgt_t', f'/ee_{side}_tgt_v'
        st, sv = f'/ee_{side}_state_t', f'/ee_{side}_state_v'
        if kt not in z.files or st not in z.files:
            continue
        tgt = resample(z[kt], z[kv][:, :3], grid)
        sta = resample(z[st], z[sv][:, :3], grid)
        for j, ax in enumerate('xyz'):
            if tgt[:, j].std() < 0.01:
                continue
            lag, corr = lag_corr(tgt[:, j], sta[:, j], dt)
            err = sta[:, j] - tgt[:, j]
            log(f'{side} EE {ax}: 滞后{lag*1000:>5.0f}ms 相关{corr:+.2f} RMS{np.sqrt(np.mean(err**2))*1000:.1f}mm max{np.abs(err).max()*1000:.1f}mm')
    log('')

    # ---------- 绘图 ----------
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    base = path.replace('.npz', '')

    # P1 你的手臂（外骨骼关节）
    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    for gi, (title, sl) in enumerate((('Exoskeleton LEFT (operator)', slice(0, 6)),
                                      ('Exoskeleton RIGHT (operator)', slice(6, 12)))):
        ax = axes[gi]
        for j in range(6):
            ax.plot(grid, np.degrees(exo_g[:, sl.start + j]), label=f'exo{j+1}', lw=0.8)
        ax.set_title(title)
        ax.set_ylabel('deg')
        ax.legend(ncol=6, fontsize=7, loc='upper right')
        ax.grid(alpha=0.3)
    axes[1].set_xlabel('time (s)')
    fig.tight_layout()
    fig.savefig(f'{base}_p1_exo.png', dpi=110)
    plt.close(fig)

    # P2 真机 14 关节
    fig, axes = plt.subplots(2, 1, figsize=(14, 8), sharex=True)
    for gi, (title, sl) in enumerate((('ROBOT LEFT (feedback)', slice(7, 14)),
                                      ('ROBOT RIGHT (feedback)', slice(0, 7)))):
        ax = axes[gi]
        for j in range(7):
            ax.plot(grid, np.degrees(rob_g[:, sl.start + j]), label=f'J{j+1}', lw=0.8)
        ax.set_title(title)
        ax.set_ylabel('deg')
        ax.legend(ncol=7, fontsize=7, loc='upper right')
        ax.grid(alpha=0.3)
    axes[1].set_xlabel('time (s)')
    fig.tight_layout()
    fig.savefig(f'{base}_p2_rob.png', dpi=110)
    plt.close(fig)

    # P3 每关节 目标vs反馈 叠加（含滞后标注）
    fig, axes = plt.subplots(4, 4, figsize=(16, 12), sharex=True)
    axes = axes.ravel()
    for i, name in enumerate(rob_names[:16]):
        ax = axes[i]
        ax.plot(grid, np.degrees(ik_g[:, i]), lw=1.0, label='IK target')
        ax.plot(grid, np.degrees(rob_g[:, i]), lw=1.0, alpha=0.8, label='robot fb')
        active = ik_g[:, i].std() > MOVE_THRESH_RAD
        if active:
            lag, corr = lag_corr(ik_g[:, i], rob_g[:, i], dt)
            ax.set_title(f'{name}  lag={lag*1000:.0f}ms c={corr:.2f}', fontsize=9)
        else:
            ax.set_title(f'{name} (static)', fontsize=9)
        ax.grid(alpha=0.3)
        ax.set_ylabel('deg', fontsize=8)
    for i in range(len(rob_names), 16):
        axes[i].axis('off')
    axes[0].legend(fontsize=8)
    fig.suptitle('IK target vs robot feedback (per joint)', fontsize=12)
    fig.tight_layout()
    fig.savefig(f'{base}_p3_overlay.png', dpi=110)
    plt.close(fig)

    # P4 运动能量 + 末端轨迹
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)
    for side, sl_exo, sl_rob, c in (('LEFT', slice(0, 6), slice(7, 14), 'tab:blue'),
                                    ('RIGHT', slice(6, 12), slice(0, 7), 'tab:red')):
        e_op = energy(exo_g[:, sl_exo])
        e_rb = energy(rob_g[:, sl_rob])
        ax = axes[0 if side == 'LEFT' else 1]
        ax.plot(grid[1:], e_op, color=c, lw=1, alpha=0.9, label=f'operator {side} (exo)')
        ax.plot(grid[1:], e_rb, color=c, lw=1, ls='--', label=f'robot {side}')
        ax.set_ylabel('joint speed norm (rad/s)')
        ax.set_title(f'{side} arm: operator vs robot motion energy')
        ax.legend(fontsize=8)
        ax.grid(alpha=0.3)
    if clutch_any_g is not None:
        dd = np.diff(clutch_any_g)
        starts = np.where(dd == 1)[0] + 1
        ends = np.where(dd == -1)[0] + 1
        if clutch_any_g[0] == 1:
            starts = np.concatenate([[0], starts])
        if len(ends) < len(starts):
            ends = np.concatenate([ends, [len(grid) - 1]])
        for ax in (axes[0], axes[1]):
            for a, b in zip(starts, ends):
                ax.axvspan(grid[a], grid[b], color='gray', alpha=0.15)
        axes[0].set_title(axes[0].get_title() + '（灰区=离合按住）')
    ax = axes[2]
    for side, c in (('left', 'tab:blue'), ('right', 'tab:red')):
        kt, kv = f'/ee_{side}_tgt_t', f'/ee_{side}_tgt_v'
        st, sv = f'/ee_{side}_state_t', f'/ee_{side}_state_v'
        if kt in z.files:
            tgt = resample(z[kt], z[kv][:, :3], grid)
            ax.plot(grid, tgt[:, 0], color=c, lw=1, label=f'{side} EE tgt x')
        if st in z.files:
            sta = resample(z[st], z[sv][:, :3], grid)
            ax.plot(grid, sta[:, 0], color=c, lw=1, ls='--', label=f'{side} EE state x')
    ax.set_title('End-effector X: target(solid) vs state(dashed)')
    ax.set_ylabel('m')
    ax.legend(fontsize=8, ncol=4)
    ax.grid(alpha=0.3)
    ax.set_xlabel('time (s)')
    fig.tight_layout()
    fig.savefig(f'{base}_p4_energy_ee.png', dpi=110)
    plt.close(fig)

    summary_path = f'{base}_summary.txt'
    with open(summary_path, 'w') as f:
        f.write('\n'.join(lines))
    log(f'\n图片: {base}_p1~p4*.png')
    log(f'摘要: {summary_path}')


if __name__ == '__main__':
    main()
