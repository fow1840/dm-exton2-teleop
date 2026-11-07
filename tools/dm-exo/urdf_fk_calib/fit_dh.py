#!/usr/bin/env python3
"""T39 数值重建可行性预演：用现存 (q, get_tcp_pose) 样本拟合 MDH 链（纯 numpy）。

为什么存在：T20 已判 URDF 结构性不可修补（292~1895mm），修复路线二选一——
厂商 URDF/D-H，或 (q, get_tcp_pose) 数值重建。本脚本回答「重建这条路在数据上
走不走得通」：拟合残差量级 + 留一交叉验证（LOO）泛化残差，据 LOO 判定需要的
样本量与激励轨迹设计（见 docs/URDF数值重建采数方案.md）。

约定（与 compare_fk.py 一致）：
  - 样本 tcp14：左 7 + 右 7；四元数 qw-first（协议序，与 ROS 相反）；xyz 单位米；
  - 17 维关节：0=腰, 1..7=L1..L7, 8..14=R1..R7, 15/16=头；
  - 腰恒 0（采数纪律），故按固定基座拟合单臂 7R 链。

参数化（每臂 40 参数）：
  T_base = exp(rotvec_b), p_b        （base_link → DH frame0）
  标准 DH 7 关节：T_i = Rz(q_i+off_i)·Tz(d_i)·Tx(a_i)·Rx(α_i)
  T_tool = exp(rotvec_t), p_t        （Link7 → TCP；自由变换吸收 0.145m 工具偏移，
                                      因此不需要 raw/tool 两种假设）
残差 = [p_pred - p_meas (m), w_rot × log3(R_predᵀ R_meas) (rad)]
优化器 = 手写阻尼 Gauss-Newton（LM 自适应阻尼 + lstsq 求步）。
  为什么不用 scipy：本地 scipy 1.8 与 numpy 2.2.6 二进制不兼容；
  纯 numpy 让本脚本在任何机器（含 tc 主机）可复现。

自测（判据先校准再用）：
  A: 用 URDF 链生成无噪合成样本 → DH 拟合残差应→0（验证 DH 表达力+优化器正确）；
  B: URDF 链对真实样本残差应复现 T20 记录的 292~1895mm/100~167° 量级
     （验证样本加载/四元数约定；对不上即查数据配对，UBT 第 7 幕教训）。
正式输出：真实样本 DH 拟合残差 + LOO 交叉验证残差 + 最优 DH 表落盘 data/fit_dh_result_{side}.json

用法：
  python3 fit_dh.py --side left --urdf <hr_up URDF> \
      --samples data/calib_samples_L_1106_0943.jsonl data/calib_L_drive.jsonl data/calib_L2_drive.jsonl
"""
import argparse
import itertools
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np

# ---------------------------------------------------------------- 旋转工具

def quat_wfirst_to_matrix(v):
    """协议四元数 (qw,qx,qy,qz) → 3x3 旋转矩阵（与 compare_fk.py 同式）。"""
    w, x, y, z = [float(c) for c in v]
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-9:
        return np.eye(3)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rotvec_to_matrix(v):
    """旋转向量 → 旋转矩阵（Rodrigues）。"""
    theta = float(np.linalg.norm(v))
    if theta < 1e-12:
        return np.eye(3)
    k = v / theta
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(theta) * K + (1 - math.cos(theta)) * (K @ K)


def log3(R):
    """旋转矩阵 → 旋转向量。走四元数（Shepperd 分支 + atan2），
    为什么不用 arccos 反对称式：θ→π 附近反对称部分抵消、θ 小时数值病态；
    atan2(n,w) 在全区间平滑，无特殊分支。"""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] >= R[1, 1] and R[0, 0] >= R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] >= R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    if w < 0:  # q 与 -q 同一个旋转；固定 w≥0 消除符号歧义
        w, x, y, z = -w, -x, -y, -z
    nv = math.sqrt(x * x + y * y + z * z)
    if nv < 1e-12:
        return 2.0 * np.array([x, y, z])  # 小角度近似
    angle = 2.0 * math.atan2(nv, w)
    return angle * np.array([x, y, z]) / nv


def rpy_to_matrix(r, p, y):
    """URDF rpy 约定：R = Rz(y)·Ry(p)·Rx(r)。"""
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def make_T(R, p):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


# ---------------------------------------------------------------- 模型链

def fk_urdf_chain(joints, q):
    """URDF 关节链 FK。joints: [{'xyz','rpy','axis','type'}...]，q 对应活动关节。"""
    T = np.eye(4)
    qi = 0
    for j in joints:
        R = rpy_to_matrix(*[float(c) for c in j['rpy'].split()])
        p = np.array([float(c) for c in j['xyz'].split()])
        Tj = make_T(R, p)
        if j['type'] == 'revolute':
            ax = np.array([float(c) for c in j['axis'].split()])
            Tj = Tj @ make_T(rotvec_to_matrix(ax * float(q[qi])), np.zeros(3))
            qi += 1
        T = T @ Tj
    return T


def fk_dh_batch(x, Q):
    """DH 链 FK（批量化）。x=[rotvec_b(3),p_b(3), a7,al7,d7,off7 (28), rotvec_t(3),p_t(3)]。
    Q: (N,7) → (N,4,4)，base_link 系下的 TCP 位姿。
    为什么批量：数值雅可比每轮要对 40 参数各做 2 次残差评估，逐样本 Python 循环
    在 LOO×多重启动规模下不可接受（实测 >10 分钟），向量化后 ~10 倍以上。"""
    Q = np.atleast_2d(np.asarray(Q, dtype=float))
    N = Q.shape[0]
    T = np.tile(make_T(rotvec_to_matrix(x[0:3]), x[3:6]), (N, 1, 1))
    dh = x[6:34].reshape(7, 4)
    a, al, d, off = dh[:, 0], dh[:, 1], dh[:, 2], dh[:, 3]
    for k in range(7):
        th = Q[:, k] + off[k]
        ct, st = np.cos(th), np.sin(th)
        ca, sa = math.cos(al[k]), math.sin(al[k])
        # Rz(th)·Tz(d)·Tx(a)·Rx(α)（标准 DH）
        M = np.empty((N, 4, 4))
        M[:, 0, 0] = ct;  M[:, 0, 1] = -st * ca; M[:, 0, 2] = st * sa;  M[:, 0, 3] = a[k] * ct
        M[:, 1, 0] = st;  M[:, 1, 1] = ct * ca;  M[:, 1, 2] = -ct * sa; M[:, 1, 3] = a[k] * st
        M[:, 2, 0] = 0.0; M[:, 2, 1] = sa;       M[:, 2, 2] = ca;       M[:, 2, 3] = d[k]
        M[:, 3, 0] = 0.0; M[:, 3, 1] = 0.0;      M[:, 3, 2] = 0.0;      M[:, 3, 3] = 1.0
        T = T @ M
    Tt = make_T(rotvec_to_matrix(x[34:37]), x[37:40])
    return T @ Tt


def fk_dh(x, q):
    """单样本 FK（report 用）。"""
    return fk_dh_batch(x, np.asarray(q)[None, :])[0]


def log3_batch(Rs):
    """log3 的批量版：(N,3,3) → (N,3)。四分支按掩码向量化。"""
    Rs = np.asarray(Rs, dtype=float)
    N = Rs.shape[0]
    tr = Rs[:, 0, 0] + Rs[:, 1, 1] + Rs[:, 2, 2]
    w = np.empty(N); xv = np.empty(N); yv = np.empty(N); zv = np.empty(N)
    m1 = tr > 0
    m2 = (~m1) & (Rs[:, 0, 0] >= Rs[:, 1, 1]) & (Rs[:, 0, 0] >= Rs[:, 2, 2])
    m3 = (~m1) & (~m2) & (Rs[:, 1, 1] >= Rs[:, 2, 2])
    m4 = (~m1) & (~m2) & (~m3)
    s = 2.0 * np.sqrt(np.maximum(1.0 + tr[m1], 0.0))
    w[m1] = 0.25 * s
    xv[m1] = (Rs[m1, 2, 1] - Rs[m1, 1, 2]) / s
    yv[m1] = (Rs[m1, 0, 2] - Rs[m1, 2, 0]) / s
    zv[m1] = (Rs[m1, 1, 0] - Rs[m1, 0, 1]) / s
    s = 2.0 * np.sqrt(np.maximum(1.0 + Rs[m2, 0, 0] - Rs[m2, 1, 1] - Rs[m2, 2, 2], 0.0))
    w[m2] = (Rs[m2, 2, 1] - Rs[m2, 1, 2]) / s
    xv[m2] = 0.25 * s
    yv[m2] = (Rs[m2, 0, 1] + Rs[m2, 1, 0]) / s
    zv[m2] = (Rs[m2, 0, 2] + Rs[m2, 2, 0]) / s
    s = 2.0 * np.sqrt(np.maximum(1.0 + Rs[m3, 1, 1] - Rs[m3, 0, 0] - Rs[m3, 2, 2], 0.0))
    w[m3] = (Rs[m3, 0, 2] - Rs[m3, 2, 0]) / s
    xv[m3] = (Rs[m3, 0, 1] + Rs[m3, 1, 0]) / s
    yv[m3] = 0.25 * s
    zv[m3] = (Rs[m3, 1, 2] + Rs[m3, 2, 1]) / s
    s = 2.0 * np.sqrt(np.maximum(1.0 + Rs[m4, 2, 2] - Rs[m4, 0, 0] - Rs[m4, 1, 1], 0.0))
    w[m4] = (Rs[m4, 1, 0] - Rs[m4, 0, 1]) / s
    xv[m4] = (Rs[m4, 0, 2] + Rs[m4, 2, 0]) / s
    yv[m4] = (Rs[m4, 1, 2] + Rs[m4, 2, 1]) / s
    zv[m4] = 0.25 * s
    neg = w < 0
    w[neg] = -w[neg]; xv[neg] = -xv[neg]; yv[neg] = -yv[neg]; zv[neg] = -zv[neg]
    nv = np.sqrt(xv * xv + yv * yv + zv * zv)
    ang = 2.0 * np.arctan2(nv, w)
    out = np.empty((N, 3))
    small = nv < 1e-12
    out[small] = 2.0 * np.stack([xv[small], yv[small], zv[small]], axis=1)
    ns = ~small
    out[ns] = (ang[ns] / nv[ns])[:, None] * np.stack([xv[ns], yv[ns], zv[ns]], axis=1)
    return out


def residuals_dh(x, Q, P, Rs, w_rot=1.0):
    """拼接残差向量（批量化）。Q (N,7)、P (N,3)、Rs (N,3,3)。"""
    Q = np.asarray(Q, dtype=float).reshape(-1, 7)
    P = np.asarray(P, dtype=float).reshape(-1, 3)
    Rs = np.asarray(Rs, dtype=float).reshape(-1, 3, 3)
    T = fk_dh_batch(x, Q)
    dp = T[:, :3, 3] - P
    dr = w_rot * log3_batch(np.transpose(T[:, :3, :3], (0, 2, 1)) @ Rs)
    return np.concatenate([dp.reshape(-1), dr.reshape(-1)])


def lm_fit(res_fn, x0, max_iter=300, lam0=1e-3, tol=1e-14, verbose=False):
    """手写 LM：数值雅可比（中央差分）+ 自适应阻尼 + lstsq（容忍秩亏/gauge 自由度）。"""
    x = np.array(x0, dtype=float)
    r = res_fn(x)
    cost = float(r @ r)
    lam = lam0
    n = len(x)
    for it in range(max_iter):
        J = np.empty((len(r), n))
        for j in range(n):
            h = 1e-6 * max(1.0, abs(x[j]))
            xp = x.copy(); xp[j] += h
            xm = x.copy(); xm[j] -= h
            J[:, j] = (res_fn(xp) - res_fn(xm)) / (2 * h)
        A = J.T @ J
        g = J.T @ r
        improved = False
        for _ in range(25):
            try:
                delta = np.linalg.lstsq(A + lam * np.diag(np.maximum(np.diag(A), 1e-10)), -g, rcond=1e-12)[0]
            except np.linalg.LinAlgError:
                lam *= 10
                continue
            xn = x + delta
            rn = res_fn(xn)
            cn = float(rn @ rn)
            if cn < cost:
                x, r, cost = xn, rn, cn
                lam = max(lam / 3, 1e-12)
                improved = True
                break
            lam *= 5
        if verbose and it % 20 == 0:
            print(f'  iter{it:3d} cost={cost:.6e} lam={lam:.1e}')
        if not improved or cost < tol:
            break
        if float(np.linalg.norm(delta)) < 1e-12:
            break
    return x, cost


# ---------------------------------------------------------------- 数据

def load_samples(paths):
    out = []
    for p in paths:
        with open(p, encoding='utf-8') as f:
            for line in f:
                line = line.strip()
                if line:
                    out.append(json.loads(line))
    return out


def dedupe(samples, side):
    """按左/右臂 q 四舍五入去重（重复读数只留一条）。
    为什么：samples_L 文件里 5 条是同一构型重复读，不去重会虚增样本量。"""
    q_off = 1 if side == 'left' else 8
    seen, out = set(), []
    for s in samples:
        q17, tcp = s.get('q17'), s.get('tcp14')
        if not q17 or not tcp or len(q17) < 15 or len(tcp) < 14:
            continue
        key = tuple(round(float(v), 3) for v in q17[q_off:q_off + 7])
        if key in seen:
            continue
        seen.add(key)
        ti = 0 if side == 'left' else 7
        out.append({
            'q': np.array([float(v) for v in q17[q_off:q_off + 7]]),
            'p': np.array([float(c) for c in tcp[ti:ti + 3]]),
            'R': quat_wfirst_to_matrix(tcp[ti + 3:ti + 7]),
            'note': s.get('note', ''),
        })
    return out


def parse_urdf_chain(urdf_path, leaf):
    """解析 base_link → leaf 的关节链（含 fixed）。"""
    root = ET.parse(urdf_path).getroot()
    joints = {}
    for j in root.findall('joint'):
        o = j.find('origin')
        ax = j.find('axis')
        joints[j.get('name')] = {
            'type': j.get('type'),
            'parent': j.find('parent').get('link'),
            'child': j.find('child').get('link'),
            'xyz': (o.get('xyz') if o is not None else '0 0 0') or '0 0 0',
            'rpy': (o.get('rpy') if o is not None and o.get('rpy') else '0 0 0'),
            'axis': (ax.get('xyz') if ax is not None else '0 0 1'),
        }
    by_child = {j['child']: n for n, j in joints.items()}
    chain, link = [], leaf
    while link in by_child and link != 'base_link':
        n = by_child[link]
        chain.append(joints[n])
        link = joints[n]['parent']
    return list(reversed(chain))


def urdf_pose(chain, q):
    """URDF 链末端位姿（q 长度=活动关节数）。"""
    return fk_urdf_chain(chain, q)


# ---------------------------------------------------------------- 主流程

def random_init(rng, n_joints=7):
    x = np.zeros(40)
    x[0:3] = rng.uniform(-math.pi, math.pi, 3)  # base 旋转
    x[3:6] = rng.uniform(-0.5, 0.5, 3)          # base 平移
    dh = np.zeros((n_joints, 4))
    dh[:, 0] = rng.uniform(-0.35, 0.35)         # a
    dh[:, 1] = rng.choice([0.0, math.pi / 2, -math.pi / 2]) + rng.uniform(-0.2, 0.2)
    dh[:, 2] = rng.uniform(-0.45, 0.45)         # d
    dh[:, 3] = rng.uniform(-math.pi, math.pi)   # θ 偏移
    x[6:34] = dh.reshape(-1)
    x[34:37] = rng.uniform(-math.pi, math.pi, 3)
    x[37:40] = rng.uniform(-0.3, 0.3, 3)
    return x


def fit_multistart(Q, P, Rs, inits, w_rot=1.0, verbose=False):
    best = None
    for i, x0 in enumerate(inits):
        x, cost = lm_fit(lambda x: residuals_dh(x, Q, P, Rs, w_rot), x0, verbose=verbose)
        if verbose:
            print(f'  start{i}: cost={cost:.6e}')
        if best is None or cost < best[1]:
            best = (x, cost)
    return best


def report(name, Q, P, Rs, x, w_rot=1.0):
    """逐样本残差表 + 汇总。返回 (pos_mm list, rot_deg list)。"""
    pos, rot = [], []
    lines = []
    for k, q in enumerate(Q):
        T = fk_dh(x, q)
        dp = float(np.linalg.norm(T[:3, 3] - P[k])) * 1000
        dr = float(np.linalg.norm(log3(T[:3, :3].T @ Rs[k]))) * 180 / math.pi
        pos.append(dp); rot.append(dr)
        lines.append(f'    [{k}] q≈[{q[0]:+.2f},{q[1]:+.2f},{q[2]:+.2f},{q[3]:+.2f},{q[4]:+.2f},{q[5]:+.2f},{q[6]:+.2f}]  {dp:9.1f}mm / {dr:7.2f}°  {"" if k else ""}')
    arr = np.array([pos, rot])
    print(f'  {name}: 位置 max={arr[0].max():9.1f}mm median={np.median(arr[0]):9.1f}mm | '
          f'姿态 max={arr[1].max():8.2f}° median={np.median(arr[1]):8.2f}°')
    for ln in lines:
        print(ln)
    return pos, rot


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--side', choices=['left', 'right'], default='left')
    ap.add_argument('--urdf', required=True)
    ap.add_argument('--samples', nargs='+', required=True)
    ap.add_argument('--w-rot', type=float, default=1.0)
    ap.add_argument('--n-start', type=int, default=12, help='随机多重启动数')
    ap.add_argument('--out', default='', help='结果 JSON 落盘路径（默认 data/fit_dh_result_<side>.json）')
    ap.add_argument('--no-loo', action='store_true')
    args = ap.parse_args()

    rng = np.random.default_rng(42)
    side = args.side

    # ---- 数据质检 + 去重
    raw = load_samples(args.samples)
    samples = dedupe(raw, side)
    Q = [s['q'] for s in samples]
    P = [s['p'] for s in samples]
    Rs = [s['R'] for s in samples]
    print(f'样本：原始 {len(raw)} 条 → 去重后独立构型 {len(samples)} 个（去重键=q 保留 3 位小数）')

    # ---- 自测B：URDF 链对真实样本残差（应复现 T20 量级，校准判据）
    chain = parse_urdf_chain(args.urdf, 'Link7L' if side == 'left' else 'Link7')
    print(f'URDF 链（→{ "Link7L" if side=="left" else "Link7"}，{sum(1 for j in chain if j["type"]=="revolute")} 活动关节）：')
    pos_b, rot_b = [], []
    for k, q in enumerate(Q):
        T = urdf_pose(chain, q)
        dp = float(np.linalg.norm(T[:3, 3] - P[k])) * 1000
        dr = float(np.linalg.norm(log3(T[:3, :3].T @ Rs[k]))) * 180 / math.pi
        pos_b.append(dp); rot_b.append(dr)
    print(f'  自测B URDF-vs-真机（raw，无工具补偿）: 位置 max={max(pos_b):.1f}mm median={np.median(pos_b):.1f}mm | '
          f'姿态 max={max(rot_b):.1f}° median={np.median(rot_b):.1f}°   ← 对照 T20: 292~1895mm / 100~167°')

    # ---- 自测A：DH 拟合 URDF 合成样本（无噪），残差应→0
    nq = sum(1 for j in chain if j['type'] == 'revolute')
    Qsyn, Psyn, Rsyn = [], [], []
    for _ in range(60):
        q = rng.uniform(-1.5, 1.5, nq)
        T = urdf_pose(chain, q)
        Qsyn.append(q); Psyn.append(T[:3, 3]); Rsyn.append(T[:3, :3])
    inits_syn = [random_init(rng) for _ in range(6)]
    x_syn, cost_syn = fit_multistart(Qsyn, Psyn, Rsyn, inits_syn, args.w_rot)
    r_syn = residuals_dh(x_syn, Qsyn, Psyn, Rsyn, args.w_rot)
    m_half = len(r_syn) // 2  # 残差布局 = [dp(N×3) | dr(N×3)]
    print(f'自测A DH↔URDF 合成拟合: cost={cost_syn:.3e} 位置max={float(np.max(np.abs(r_syn[:m_half])))*1000:.4f}mm '
          f'姿态max={float(np.max(np.abs(r_syn[m_half:])))*180/math.pi:.5f}° '
          f'{"✅ 通过" if cost_syn < 1e-8 else "❌ 未通过——优化器/表达力有问题，先修再信正式结果"}')
    # 自测A 的解同时是正式拟合的「URDF 等价」初始值（结构上最接近 URDF 的 DH）
    init_urdf = x_syn.copy()

    # ---- 正式：真实样本拟合（多重启动）
    inits = [init_urdf] + [random_init(rng) for _ in range(args.n_start)]
    print(f'正式拟合（{len(inits)} 个启动：URDF 等价解 + {args.n_start} 随机）…')
    x_best, cost_best = fit_multistart(Q, P, Rs, inits, args.w_rot)
    print(f'  最优 cost={cost_best:.6e}')
    pos_f, rot_f = report('拟合残差（全样本内）', Q, P, Rs, x_best, args.w_rot)

    # ---- LOO 留一交叉验证（泛化能力 = 可行性的真实判据）
    if not args.no_loo and len(Q) >= 5:
        print(f'LOO 留一交叉验证（{len(Q)} 折：{len(Q)-1} 训 1 测，每折 {args.n_start} 启动）…')
        loo_p, loo_r = [], []
        for k in range(len(Q)):
            idx = [i for i in range(len(Q)) if i != k]
            Qk = [Q[i] for i in idx]; Pk = [P[i] for i in idx]; Rk = [Rs[i] for i in idx]
            xk, _ = fit_multistart(Qk, Pk, Rk, [init_urdf] + [random_init(rng) for _ in range(args.n_start)], args.w_rot)
            T = fk_dh(xk, Q[k])
            dp = float(np.linalg.norm(T[:3, 3] - P[k])) * 1000
            dr = float(np.linalg.norm(log3(T[:3, :3].T @ Rs[k]))) * 180 / math.pi
            loo_p.append(dp); loo_r.append(dr)
            print(f'    折{k}: 留出样本预测 {dp:9.1f}mm / {dr:7.2f}°')
        print(f'  LOO 汇总: 位置 max={max(loo_p):.1f}mm median={np.median(loo_p):.1f}mm | '
              f'姿态 max={max(loo_r):.2f}° median={np.median(loo_r):.2f}°')
    else:
        loo_p = loo_r = []

    # ---- 落盘
    dh = x_best[6:34].reshape(7, 4)
    out_path = args.out or os.path.join(os.path.dirname(args.samples[0]), f'fit_dh_result_{side}.json')
    result = {
        'side': side,
        'n_samples_raw': len(raw),
        'n_samples_unique': len(samples),
        'w_rot': args.w_rot,
        'fit_cost': cost_best,
        'fit_pos_mm_max': float(np.max(pos_f)),
        'fit_pos_mm_median': float(np.median(pos_f)),
        'fit_rot_deg_max': float(np.max(rot_f)),
        'fit_rot_deg_median': float(np.median(rot_f)),
        'loo_pos_mm': [float(v) for v in loo_p],
        'loo_rot_deg': [float(v) for v in loo_r],
        'urdf_baseline_pos_mm': [float(v) for v in pos_b],
        'urdf_baseline_rot_deg': [float(v) for v in rot_b],
        'x40': [float(v) for v in x_best],
        'dh_table': {
            f'J{i+1}': {'a': float(dh[i, 0]), 'alpha': float(dh[i, 1]),
                        'd': float(dh[i, 2]), 'theta_offset': float(dh[i, 3])}
            for i in range(7)
        },
        'T_base_rotvec': [float(v) for v in x_best[0:3]],
        'T_base_p': [float(v) for v in x_best[3:6]],
        'T_tool_rotvec': [float(v) for v in x_best[34:37]],
        'T_tool_p': [float(v) for v in x_best[37:40]],
        'note': 'T39 数值重建可行性预演；DH 表仅为可行性证据，非交付模型（样本不足，LOO 判泛化）',
    }
    with open(out_path, 'w', encoding='utf-8') as f:
        json.dump(result, f, ensure_ascii=False, indent=2)
    print(f'结果已落盘: {out_path}')

    # ---- 判定（门限先声明再判；median+max 双门限——只看 median 会掩盖尾部塌陷，
    #      实测 7 折里 5 折 <3.4mm、2 折 >389mm，median=1.5mm 是假好看）
    if loo_p:
        med = float(np.median(loo_p))
        mx = float(np.max(loo_p))
        print('\n==== 预演判定 ====')
        if med < 30 and mx < 100:
            print(f'LOO 位置 median={med:.1f}mm max={mx:.1f}mm → 双门限过，重建路线可行')
        elif med < 30:
            print(f'LOO 位置 median={med:.1f}mm 但 max={mx:.1f}mm → 插值准、外推塌：'
                  'DH 家族与控制器运动学匹配（强支持信号），短板=样本覆盖，按采数方案扩样')
        else:
            print(f'LOO 位置 median={med:.1f}mm max={mx:.1f}mm → 覆盖严重不足，必须按采数方案采样后再判')
    print('\n注意：本脚本不判「URDF 该怎么改」，只判「数值重建走不走得通」。修复交付见采数方案文档。')


if __name__ == '__main__':
    sys.exit(main())
