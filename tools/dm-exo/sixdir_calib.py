#!/usr/bin/env python3
"""六方向平移标定与验证（T29 主从映射模块的配套流程工具）。

角色划分（与 docs/方向诊断与校准系统.md 一致）：
  · 本工具只标定「映射层」：外骨骼末端增量 → IK 目标增量（/ee_op → /ee_{side}_tgt）。
    这层完全可离线验证，不需要真机 TCP 真值。
  · 「真机 TCP 是否跟着目标方向走」属于 T20 运动学层（结构性不一致，等厂商 URDF/D-H）。
    预留 --tcp 传入实测 TCP 增量（npz），将来 bridge 加 get_tcp_pose 流后即插即用。

流程（操作员在场）：
  1. record_compare.py 录制遥操段：按住离合，依次沿 ±X/±Y/±Z 慢移 ~50mm，
     每个方向一个「按-动-松」段，段间离合松开停 1s 以上；
  2. segment  ：按离合沿自动分段，给出每段外骨骼/目标位移表和主轴建议；
  3. fit      ：用标注段拟合 3×3 正交映射 + 缩放，落盘 calib JSON；
  4. verify   ：拿另一次录制（不放标定时录的）离线验收：主轴方向一致率、泄漏。

用法：
  python3 sixdir_calib.py segment rec.npz --side left
  python3 sixdir_calib.py fit     rec.npz --side left --labels "0:+X,1:-X,2:+Y,3:-Y,4:+Z,5:-Z" \
                                  --out ~/桌面/dm-exo/calib_left_v1.json
  python3 sixdir_calib.py verify  rec2.npz calib_left_v1.json --side left
  python3 sixdir_calib.py selftest          # 合成数据离线自检（无 ROS 依赖）
"""
import argparse
import json
import os
import sys

import numpy as np

# master_slave_map：本地仓库路径优先，tc 主机运行时包路径兜底
_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_CANDIDATES = [
    os.path.abspath(os.path.join(_HERE, '..', '..', 'ros2_ws', 'src',
                                 'robot_ik_retargeting', 'robot_ik_retargeting')),
    '/home/tc/DM-EXton2ProductPackage_tianchuang/exton_ros2_retargeting_demo'
    '/src/robot_ik_retargeting/robot_ik_retargeting',
]
for _p in _PKG_CANDIDATES:
    if os.path.isdir(_p):
        sys.path.insert(0, _p)
from master_slave_map import ArmPoseMap, fit_orthogonal_map, fit_residual  # noqa: E402

LEAK_LIMIT_M = 0.003   # 验收：非主轴泄漏 < 3mm（T29 验收判据）
EDGE_BLUR_S = 0.3      # 段首尾静止中值窗


def load_npz(path):
    """读取 record_compare 格式（扁平 {key}_t/{key}_v）→ {key: {t,v}}。"""
    z = np.load(path, allow_pickle=False)
    bases = {k[:-2] for k in z.files if k.endswith(('_t', '_v'))}
    return {k: {'t': z[f'{k}_t'], 'v': z[f'{k}_v']} for k in bases}


def clutch_windows(clutch_t, clutch_v):
    """离合上升/下降沿 → [(t_on, t_off)]，t_off 缺失时取录像结尾。"""
    v = np.asarray(clutch_v, dtype=float).ravel().astype(bool)
    t = np.asarray(clutch_t, dtype=float).ravel()
    wins = []
    on = None
    for i in range(len(v)):
        if v[i] and on is None:
            on = t[i]
        elif not v[i] and on is not None:
            wins.append((on, t[i]))
            on = None
    if on is not None:
        wins.append((on, t[-1]))
    return wins


def _resample_to(t_src, v_src, t_query):
    """最近邻重采样（各话题频率不同，统一到查询时刻）。"""
    idx = np.searchsorted(t_src, t_query).clip(1, len(t_src) - 1)
    left = t_src[idx - 1]
    right = t_src[idx]
    pick = np.where(np.abs(t_query - left) <= np.abs(right - t_query), idx - 1, idx)
    return np.asarray(v_src, dtype=float)[pick]


def segment(npz, side):
    """按离合分段，返回每段的 dp_exo / dp_tgt 及主轴建议。"""
    ct_key = f'/clutch_{side}'
    tg_key = f'/ee_{side}_tgt'
    for k in (ct_key, '/ee_op', tg_key):
        if k not in npz:
            raise KeyError(f'npz 缺少话题 {k}')
    ct = npz[ct_key]
    wins = clutch_windows(ct['t'], ct['v'])
    ee_op = np.asarray(npz['/ee_op']['v'], dtype=float)   # (N,14)
    t_op = np.asarray(npz['/ee_op']['t'], dtype=float)
    tg = np.asarray(npz[tg_key]['v'], dtype=float)        # (M,7) x y z qx qy qz qw
    t_tg = np.asarray(npz[tg_key]['t'], dtype=float)
    # 确定该臂在 ee_op 里的偏移：target 位置 vs 两半前 3 维，逐段相关取和最大者
    scores = [0.0, 0.0]
    for (on, off) in wins:
        m = (t_op >= on) & (t_op <= off)
        if m.sum() < 4:
            continue
        q = _resample_to(t_tg, tg[:, :3], t_op[m])
        for half in (0, 1):
            blk = ee_op[m][:, 3 * half:3 * half + 3]
            a = blk - blk.mean(0)
            b = q - q.mean(0)
            denom = (np.linalg.norm(a) * np.linalg.norm(b)) or 1.0
            # 取绝对值：镜像/反向臂的相关是负的，也是「这一半就是该臂」的证据
            scores[half] += abs(float(np.sum(a * b)) / denom)
    off_idx = 0 if scores[0] >= scores[1] else 1
    print(f'臂块检测: 左半相关={scores[0]:.2f} 右半相关={scores[1]:.2f} → '
          f'ee_op[{3 * off_idx}:{3 * off_idx + 3}] 为 {side} 臂')
    rows = []
    for k, (on, off) in enumerate(wins):
        m_op = (t_op >= on + 0.1 * 0) & (t_op <= off)
        if m_op.sum() < 4:
            rows.append((k, on, off, None, None))
            continue
        p = ee_op[m_op][:, 3 * off_idx:3 * off_idx + 3]
        qt = _resample_to(t_tg, tg[:, :3], t_op[m_op])
        t_rel = t_op[m_op] - on
        head = (t_rel < EDGE_BLUR_S) | (t_rel > (off - on) - EDGE_BLUR_S)
        # dp = 段末中值 − 段初中值（避开按/松沿的抖动）
        dp_exo = np.median(p[~head][-5:], 0) - np.median(p[head][:5], 0)
        dp_tgt = np.median(qt[~head][-5:], 0) - np.median(qt[head][:5], 0)
        rows.append((k, on, off, dp_exo, dp_tgt))
    print(f'\n{"段":>3} {"起点s":>8} {"时长s":>6} {"exo dp (mm)":^24} {"tgt dp (mm)":^24} 主轴建议')
    for k, on, off, dp_exo, dp_tgt in rows:
        if dp_exo is None:
            print(f'{k:>3} {on:8.2f} {off - on:6.2f}   （段太短，弃用）')
            continue
        main = 'XYZ'[int(np.argmax(np.abs(dp_exo)))]
        print(f'{k:>3} {on:8.2f} {off - on:6.2f} '
              f'{np.array2string(dp_exo * 1000, precision=1):^24} '
              f'{np.array2string(dp_tgt * 1000, precision=1):^24} {main}')
    return rows


def fit(npz, side, labels, out_path, npz_path='(inline)', mode='measured',
        position_scale=1.0):
    """两种模式：
    measured：dst = 每段实测目标位移 → 回收「当前链路有效映射」（诊断用）。
    desired ：dst = 标注的理想方向 × 段长 → 直接生成期望映射（写标定文件用，
              通常就是恒等阵 + 操作员要的缩放）。
    verify 都是对实测比对——desired 模式的标定必须部署后再录一段验证。
    """
    rows = segment(npz, side)
    src, dst = [], []
    for spec in labels.split(','):
        k, lab = spec.split(':')
        dp_exo, dp_tgt = rows[int(k)][3], rows[int(k)][4]
        if dp_exo is None:
            raise ValueError(f'段 {k} 不可用')
        src.append(dp_exo)
        if mode == 'measured':
            dst.append(dp_tgt)
        else:
            axis = {'+X': 0, '-X': 0, '+Y': 1, '-Y': 1, '+Z': 2, '-Z': 2}[lab]
            sign = 1.0 if lab[0] == '+' else -1.0
            dst.append(sign * np.eye(3)[axis] * np.linalg.norm(dp_tgt))
    P = fit_orthogonal_map(src, dst)
    res = fit_residual(P, src, dst)
    print(f'\n[{mode}] 拟合 P（行=机器人轴，列=外骨骼轴）:\n{np.round(P, 4)}')
    print(f'最大残差 {res * 1000:.2f} mm（代理层；>3mm 说明样本噪声大或方向标错）')
    rep = ArmPoseMap(position_map=P).translation_axis_report(move_m=0.05)  # 50mm 标定动作尺度
    for k, v in rep.items():
        print(f'  外骨骼 {k} → 主轴 {v["main"]} 增益 {v["gain"]:+.3f} 泄漏 {v["leak"] * 1000:.1f}mm')
    if any(v['leak'] > LEAK_LIMIT_M for v in rep.values()):
        print('⚠️ 存在超 3mm 泄漏的方向：检查标注或补测该方向')
    m = ArmPoseMap(position_map=P, position_scale=position_scale)
    m.save_json(out_path, meta={'source': os.path.basename(npz_path), 'mode': mode,
                                'labels': labels, 'residual_m': res,
                                'proxy': 'ee_tgt(代理，非真机 TCP)'})
    print(f'已保存 {out_path}')


def verify(npz, side, calib_path):
    """验收：回放录制，逐段比对 map(dp_exo) 与实际目标位移。"""
    m = ArmPoseMap.load_json(calib_path)
    rows = segment(npz, side)
    ok = True
    n_used = 0
    print(f'\n验收（泄漏门限 {LEAK_LIMIT_M * 1000:.0f}mm）:')
    for k, on, off, dp_exo, dp_tgt in rows:
        if dp_exo is None or np.linalg.norm(dp_exo) < 0.01:
            continue
        n_used += 1
        pred = m.map_translation(dp_exo)
        err = pred - dp_tgt
        leak = float(np.max(np.abs(err)))
        flag = '✅' if leak < LEAK_LIMIT_M else '❌'
        ok = ok and leak < LEAK_LIMIT_M
        print(f'  段{k} {flag} 误差 {np.array2string(err * 1000, precision=1)} mm '
              f'(最大 {leak * 1000:.1f}mm)')
    if n_used < 3:
        print(f'⚠️ 可用段只有 {n_used} 个（<3）：验收不成立，不算 PASS')
        ok = False
    print('验收结论:', 'PASS' if ok else 'FAIL')
    return 0 if ok else 1


def selftest():
    """合成链路自检：真映射 → 合成录制 → segment/fit/verify 全链还原。"""
    true_P = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    rng = np.random.default_rng(3)
    t = np.arange(0, 30, 0.004)
    clutch = np.zeros_like(t, dtype=bool)
    dirs = [('0:+X', [1, 0, 0]), ('1:-Y', [0, -1, 0]), ('2:+Z', [0, 0, 1])]
    labels = [lab for lab, _ in dirs]
    # 全时间轴合成：每段在窗口内 0.3s 静止 → 1s 匀速 50mm → 保持；目标 = 真映射 + 噪声
    ee_op = np.zeros((len(t), 14))
    tg_rows, tg_t = [], []
    for i, (lab, d) in enumerate(dirs):
        on0, on1 = 3 + i * 8, 3 + i * 8 + 4
        clutch[(t >= on0) & (t <= on1)] = True
        in_win = (t >= on0) & (t <= on1)
        tw = t[in_win]
        ramp = np.clip((tw - on0 - 0.3) / 1.0, 0.0, 1.0)
        block = np.outer(ramp, np.array(d, dtype=float)) * 0.05  # 本段内相对窗口起点的位移
        # 段与段之间外骨骼回中（松离合时回位，符合真实标定流程）
        p = np.where(ramp.reshape(-1, 1) > 0.0, block, 0.0)
        ee_op[in_win, 0:3] = p
        for tt, pk in zip(tw, p):
            tgt = true_P @ pk + rng.normal(scale=5e-4, size=3)
            tg_rows.append(np.concatenate([tgt, [0, 0, 0, 1]]))
            tg_t.append(tt)
    npz = {
        '/clutch_left_t': t,
        '/clutch_left_v': clutch.astype(float).reshape(-1, 1),
        '/ee_op_t': t,
        '/ee_op_v': ee_op,
        '/ee_left_tgt_t': np.array(tg_t),
        '/ee_left_tgt_v': np.array(tg_rows),
    }
    import tempfile
    with tempfile.TemporaryDirectory() as d:
        path = os.path.join(d, 'syn.npz')
        np.savez(path, **npz)
        rows = segment(load_npz(path), 'left')
        usable = [r for r in rows if r[3] is not None]
        assert len(usable) == 3, f'分段数 {len(usable)} != 3'
        for r in usable:  # 合成数据里 exo 位移必须是 50mm 级，防止「真空通过」
            assert abs(np.linalg.norm(r[3]) - 0.05) < 5e-3, f'exo dp 异常: {r[3]}'
        # measured 模式必须从带真映射的代理数据里回收出 true_P
        cal = os.path.join(d, 'cal.json')
        fit(load_npz(path), 'left', ','.join(labels), cal, path, mode='measured')
        m = ArmPoseMap.load_json(cal)
        np.testing.assert_allclose(m.position_map, true_P, atol=5e-2)  # 5e-4 噪声→~1% 偏差
        rc = verify(load_npz(path), 'left', cal)
        assert rc == 0, 'verify 未通过'
    print('\nselftest PASS ✅')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('cmd', choices=['segment', 'fit', 'verify', 'selftest'])
    ap.add_argument('npz', nargs='?')
    ap.add_argument('--side', default='left')
    ap.add_argument('--labels', default='')
    ap.add_argument('--out', default='')
    ap.add_argument('--mode', default='measured', choices=['measured', 'desired'])
    ap.add_argument('--position-scale', type=float, default=1.0)
    args = ap.parse_args()
    if args.cmd == 'selftest':
        selftest()
        return
    npz = load_npz(args.npz)
    if args.cmd == 'segment':
        segment(npz, args.side)
    elif args.cmd == 'fit':
        fit(npz, args.side, args.labels, args.out or 'calib.json', args.npz,
            mode=args.mode, position_scale=args.position_scale)
    elif args.cmd == 'verify':
        sys.exit(verify(npz, args.side, args.out))


if __name__ == '__main__':
    main()
