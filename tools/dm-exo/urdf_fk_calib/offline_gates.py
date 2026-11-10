#!/usr/bin/env python3
"""URDF 桌面三门（G0 限位/资料、G1 结构、G2 自洽）离线体检。

对应 docs/URDF真机验收方案.md 的 G0/G1/G2——这三门不需要真机，任何 URDF 接入链路前
必须先在桌面过掉。本工具只读，不改任何文件。

用法：
  python3 offline_gates.py <urdf> [--config-json <get_config快照>] [--bridge <exton_bridge.py>]

G2 用独立实现（自己按 URDF 语义组合 origin×axis）与 pinocchio FK 对比：
只证「实现自洽」，**不证 URDF 匹配真机**——后者只有 G3（控制器 get_tcp_pose 真值）能给。
"""
import argparse
import ast
import hashlib
import math
import xml.etree.ElementTree as ET

import numpy as np


# ---------- 基础几何 ----------

def rpy_to_matrix(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def axis_angle(axis, q):
    axis = np.array(axis, dtype=float)
    n = np.linalg.norm(axis)
    if n < 1e-12:
        return np.eye(3)
    k = axis / n
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * (K @ K)


def parse_origin(elem):
    o = elem.find('origin')
    if o is None:
        return np.zeros(3), np.zeros(3)
    xyz = [float(x) for x in o.get('xyz', '0 0 0').split()]
    rpy = [float(x) for x in o.get('rpy', '0 0 0').split()]
    return np.array(xyz), np.array(rpy)


def load_urdf(path):
    root = ET.parse(path).getroot()
    joints, links = {}, {l.get('name') for l in root.findall('link')}
    for j in root.findall('joint'):
        xyz, rpy = parse_origin(j)
        axis_el = j.find('axis')
        axis = [float(x) for x in axis_el.get('xyz').split()] if axis_el is not None else [1, 0, 0]
        lim = j.find('limit')
        joints[j.get('name')] = {
            'type': j.get('type'),
            'parent': j.find('parent').get('link'),
            'child': j.find('child').get('link'),
            'xyz': xyz, 'rpy': rpy, 'axis': axis,
            'limit': (None, None) if lim is None else (
                float(lim.get('lower', 'nan')), float(lim.get('upper', 'nan'))),
        }
    return root, joints, links


def chain_to(joints, links, target_link, root_link):
    """返回 root_link→target_link 的关节序列（固定关节也包含）。"""
    child_of = {}
    for name, j in joints.items():
        child_of.setdefault(j['child'], []).append(name)
    # 向上找父链
    parent_joint = {}
    for name, j in joints.items():
        parent_joint[j['child']] = name
    seq, cur = [], target_link
    while cur != root_link:
        if cur not in parent_joint:
            raise SystemExit(f'链路断开：{cur} 不是 {root_link} 的后代')
        jn = parent_joint[cur]
        seq.append(jn)
        cur = joints[jn]['parent']
    return list(reversed(seq))


def fk_independent(joints, seq, qmap):
    p = np.zeros(3)
    R = np.eye(3)
    for jn in seq:
        j = joints[jn]
        p = p + R @ j['xyz']
        R = R @ rpy_to_matrix(*j['rpy'])
        if j['type'] in ('revolute', 'continuous'):
            q = float(qmap.get(jn, 0.0))
            R = R @ axis_angle(j['axis'], q)
        elif j['type'] == 'prismatic':
            p = p + R @ (np.array(j['axis'], float) * float(qmap.get(jn, 0.0)))
    return p, R


def rot_err_deg(Ra, Rb):
    c = (np.trace(Ra.T @ Rb) - 1) / 2
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def md5(path):
    h = hashlib.md5()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


# ---------- G0 ----------

def g0(joints, links, root, urdf_path, cfg_json, bridge_path):
    print('=' * 78)
    print('G0 资料层')
    print('=' * 78)
    print(f'URDF: {urdf_path}')
    print(f'md5 : {md5(urdf_path)}')
    print(f'robot 名: {root.get("name")}   关节数: {len(joints)}   连杆数: {len(links)}')
    movable = [n for n, j in joints.items() if j['type'] in ('revolute', 'continuous')]
    print(f'可动关节: {len(movable)} → {movable}')

    urdf_lim = {n: joints[n]['limit'] for n in movable}
    zero_lim = [n for n in movable if urdf_lim[n] == (0.0, 0.0)]
    print(f'限位为 0/0 的关节数: {len(zero_lim)}' + (f' → {zero_lim}' if zero_lim else ''))

    cfg_lim = None
    if cfg_json:
        import json
        with open(cfg_json, encoding='utf-8') as f:
            cfg = json.load(f)
        lo, hi = cfg['lowerLimits'], cfg['upperLimits']
        print(f'\nget_config 快照: {cfg_json}  (n={len(lo)})')
        # 现场 get_config 返回的是 14 维（无腰/头）：前 7 = 一臂，后 7 = 另一臂，两半同值。
        # 与协议 17 维（0=腰,1..7=L1..L7,8..14=R1..R7）不同，这里按 14 维解析。
        if len(lo) == 14:
            cfg_lim = {'L': list(zip(lo[0:7], hi[0:7])), 'R': list(zip(lo[7:14], hi[7:14]))}
            print('  14 维解析: 前 7 = 第一臂, 后 7 = 第二臂（无双腰，两半同值）')
        else:
            cfg_lim = {'L': list(zip(lo[1:8], hi[1:8])), 'R': list(zip(lo[8:15], hi[8:15]))}
            print('  17 维解析: 0=腰, 1..7=L, 8..14=R')
        for side, names in (('R', [f'Joint{i}' for i in range(1, 8)]),
                            ('L', [f'Joint{i}L' for i in range(1, 8)])):
            print(f'  --- {side} 臂：URDF vs get_config（差值 = 真机 − URDF）---')
            bad = 0
            for i, n in enumerate(names):
                u_lo, u_hi = urdf_lim.get(n, (float('nan'),) * 2)
                c_lo, c_hi = cfg_lim[side][i]
                ok = abs(c_lo - u_lo) < 0.021 and abs(c_hi - u_hi) < 0.021
                bad += 0 if ok else 1
                print(f'   {n:9s} URDF[{u_lo:+.4f},{u_hi:+.4f}]  cfg[{c_lo:+.4f},{c_hi:+.4f}]'
                      f'  d=({c_lo-u_lo:+.4f},{c_hi-u_hi:+.4f}){"" if ok else "  <== 不一致"}')
            print(f'   结论: {side} 臂 ' + ('全轴一致（差值≈内缩 0.02rad）' if bad == 0 else f'{bad} 轴不一致'))

    if bridge_path:
        with open(bridge_path, encoding='utf-8') as f:
            src = f.read()
        tree = ast.parse(src)
        consts = {}
        for node in tree.body:
            if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name):
                name = node.targets[0].id
                val = node.value
            elif isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
                name = node.target.id
                val = node.value
            else:
                continue
            try:
                consts[name] = ast.literal_eval(val)
            except Exception:
                # 表里含算术（如 (-179.9 + 2.0, ...)），literal_eval 不支持 → 受限 eval
                try:
                    consts[name] = eval(compile(ast.Expression(val), '<const>', 'eval'),
                                        {'__builtins__': {}}, {})
                except Exception:
                    pass
        print('\nbridge 限位表（exton_bridge.py 常量）:')
        for key in ('LIVE_LIMITS_RAD', 'ARCHIVE_RIGHT_LIMITS_DEG', 'URDF_LIMITS_RAD'):
            if key in consts:
                v = consts[key]
                print(f'  {key} = {v}')
        live = consts.get('LIVE_LIMITS_RAD')
        if live and cfg_lim:
            # bridge LIVE 表顺序 = 左7 + 右7（与 LIVE_LIMITS_RAD 注释一致）
            arr = np.array(live, dtype=float)
            d_l = np.abs(arr[:7] - np.array(cfg_lim['L'])).max()
            d_r = np.abs(arr[7:14] - np.array(cfg_lim['R'])).max()
            print(f'  LIVE vs get_config 最大差: 左 {d_l:.4f} rad, 右 {d_r:.4f} rad'
                  '（内缩 0.02rad 即预期 ~0.02）')
    return urdf_lim, cfg_lim


# ---------- G1 ----------

def g1(joints, links):
    print('\n' + '=' * 78)
    print('G1 静态结构层')
    print('=' * 78)
    movable = [(n, j) for n, j in joints.items() if j['type'] in ('revolute', 'continuous')]
    print('关节序（文件顺序 = 协议假设用序）:', [n for n, _ in movable])
    odd_axis = [(n, j['axis']) for n, j in movable if [round(a, 6) for a in j['axis']] != [0, 0, 1]]
    print(f'非 z 轴关节: {len(odd_axis)}' + (f' → {odd_axis}' if odd_axis else '（全为 [0,0,1]）'))

    print('\n左右臂 origin 逐字对比（镜像性检查）:')
    for i in range(1, 8):
        r, l = joints.get(f'Joint{i}'), joints.get(f'Joint{i}L')
        if not r or not l:
            print(f'  Joint{i} 缺一侧'); continue
        same = (np.allclose(r['xyz'], l['xyz'], atol=1e-9)
                and np.allclose(r['rpy'], l['rpy'], atol=1e-9))
        mirrored = (np.allclose(r['xyz'], l['xyz'] * np.array([-1, 1, 1]), atol=1e-6))
        print(f'  Joint{i}: R xyz={np.round(r["xyz"],4)} rpy={np.round(r["rpy"],4)} | '
              f'L xyz={np.round(l["xyz"],4)} rpy={np.round(l["rpy"],4)}'
              f'  {"逐字相同" if same else ("x 镜像" if mirrored else "非简单镜像/非相同")}')

    roots = [l for l in links if l not in {j['child'] for j in joints.values()}]
    print(f'\n根连杆: {roots}')
    fixed = [(n, j) for n, j in joints.items() if j['type'] == 'fixed']
    print(f'固定关节 {len(fixed)} 个: ' + ', '.join(
        f'{n}({j["parent"]}→{j["child"]}, rpy={np.round(j["rpy"],4)})' for n, j in fixed[:8]))
    tips = [l for l in links if l not in {j['parent'] for j in joints.values()}]
    print(f'末端连杆: {tips}')
    print('\n零位语义：URDF 零位 = 全部关节 q=0 时各 origin 直接叠加；'
          '若真机零位与之不同（SDK 前伸 vs 模型垂臂），必须在 G3 拟合零偏，不能靠肉眼对。')


# ---------- G2 ----------

def g2_with_pin(urdf_path, joints, root_link, tips):
    import pinocchio as pin
    model = pin.buildModelFromUrdf(urdf_path)
    data = model.createData()
    rng = np.random.default_rng(0)
    movable = [n for n, j in joints.items() if j['type'] in ('revolute', 'continuous')]
    print('\n' + '=' * 78)
    print('G2 自洽层：独立 URDF 语义组合 vs pinocchio FK')
    print('=' * 78)
    for tip in tips:
        try:
            fid = model.getFrameId(tip)
            if fid == len(model.frames):
                print(f'  {tip}: pinocchio 无此 frame，跳过'); continue
            seq = chain_to(joints, None, tip, root_link)
        except SystemExit as e:
            print(f'  {tip}: {e}'); continue
        worst_p = worst_r = 0.0
        for _ in range(20):
            qmap = {n: float(rng.uniform(-0.8, 0.8)) for n in movable}
            p1, R1 = fk_independent(joints, seq, qmap)
            q = pin.neutral(model)
            for n, v in qmap.items():
                if model.existJointName(n):
                    jid = model.getJointId(n)
                    q[model.joints[jid].idx_q] = v
            pin.forwardKinematics(model, data, q)
            pin.updateFramePlacements(model, data)
            T = data.oMf[fid]
            worst_p = max(worst_p, float(np.linalg.norm(np.array(T.translation) - p1)) * 1000)
            worst_r = max(worst_r, rot_err_deg(np.array(T.rotation), R1))
        print(f'  {tip:10s} 20 随机位形：位置 max={worst_p:.6f}mm  姿态 max={worst_r:.6f}°')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('urdf')
    ap.add_argument('--config-json', default='')
    ap.add_argument('--bridge', default='')
    ap.add_argument('--root-link', default='base_link')
    args = ap.parse_args()
    root, joints, links = load_urdf(args.urdf)
    g0(joints, links, root, args.urdf, args.config_json, args.bridge)
    g1(joints, links)
    tips = [l for l in links if l not in {j['parent'] for j in joints.values()}]
    g2_with_pin(args.urdf, joints, args.root_link, tips)


if __name__ == '__main__':
    main()
