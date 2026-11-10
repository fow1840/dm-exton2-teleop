#!/usr/bin/env python3
"""用既有运行包对 URDF 做离线体检（G3 前置的可行/不可行边界判定 + 双发布污染量化）。

背景：/target_robot/left_ee/state 有两个发布者——ik_node 的指令 FK、robot_fk_adapter 的
反馈 FK。两者都用同一份 URDF 算，所以「包里 state 对得上 URDF FK」只能证明链路自洽，
**不能**证明「URDF = 真机」（G3 需要控制器 get_tcp_pose 真值，包里没有）。

本工具做两件事：
  1. 逐包判定 state 每条消息来自哪一路（对 FK(q_ik) / FK(q_rob) 的最小残差）；
     → 量化 T11「state 双发布」污染，也解释为什么这类包不能当 G3 数据。
  2. 对反馈 FK 路的消息报残差（应 ~0，证明 URDF 在 IK 与 FK adapter 两侧同源）。

用法（pinocchio 环境）：
  <pink>/bin/python verify_bags_urdf.py <urdf> <bag1.npz> [bag2.npz ...]
"""
import argparse
import math

import numpy as np

LEFT = ['Joint1L', 'Joint2L', 'Joint3L', 'Joint4L', 'Joint5L', 'Joint6L', 'Joint7L']


def quat_ros_to_matrix(v):
    """PoseStamped 四元数序 (x,y,z,w) → R。"""
    x, y, z, w = [float(c) for c in v]
    n = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class FK:
    def __init__(self, urdf, frame, joints):
        import pinocchio as pin
        self.pin = pin
        self.model = pin.buildModelFromUrdf(urdf)
        self.data = self.model.createData()
        self.frame = frame
        self.joints = joints
        self.fid = self.model.getFrameId(frame)
        if self.fid == len(self.model.frames):
            raise SystemExit(f'URDF 里没有 frame: {frame}')

    def __call__(self, q7):
        pin, model, data = self.pin, self.model, self.data
        q = pin.neutral(model)
        for name, val in zip(self.joints, q7):
            if not model.existJointName(name):
                raise SystemExit(f'URDF 里没有关节: {name}')
            jid = model.getJointId(name)
            q[model.joints[jid].idx_q] = float(val)
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)
        T = data.oMf[self.fid]
        return np.array(T.translation), np.array(T.rotation)


def interp_at(t, ts, vs):
    if t < ts[0] - 1e-6 or t > ts[-1] + 1e-6:
        return None
    return np.array([np.interp(t, ts, vs[:, c]) for c in range(vs.shape[1])])


def audit_bag(path, fk, frame, pin):
    d = np.load(path, allow_pickle=True)
    need = ['/rob_t', '/rob_v', '/ik_t', '/ik_v', '/ee_left_state_t', '/ee_left_state_v', '/rob_names']
    if not all(k in d.files for k in need):
        return {'path': path, 'skip': '缺通道'}
    names = [str(x) for x in d['/rob_names']]
    if not all(n in names for n in LEFT):
        return {'path': path, 'skip': 'rob_names 无左臂关节'}
    cols = [names.index(n) for n in LEFT]
    rob_t, rob_v = d['/rob_t'], d['/rob_v'][:, cols]
    ik_t, ik_v = d['/ik_t'], d['/ik_v'][:, cols]
    st_t, st_v = d['/ee_left_state_t'], d['/ee_left_state_v']

    cls = {'ik': 0, 'rob': 0, 'none': 0}
    res = {'ik': [], 'rob': []}
    for k in range(len(st_t)):
        t = float(st_t[k])
        p_st = np.array(st_v[k][:3], dtype=float)
        R_st = quat_ros_to_matrix(st_v[k][3:7])
        best, best_err = 'none', 1e9
        for src, (ts, vs) in (('ik', (ik_t, ik_v)), ('rob', (rob_t, rob_v))):
            q7 = interp_at(t, ts, vs)
            if q7 is None:
                continue
            p, R = fk(q7)
            mm = float(np.linalg.norm(p_st - p)) * 1000
            deg = float(np.linalg.norm(pin.log3(R.T @ R_st))) * 180 / math.pi
            res[src].append((mm, deg))
            # 位置 1mm 内且姿态 5° 内才算「同源」
            if mm < 1.0 and deg < 5.0 and mm < best_err:
                best, best_err = src, mm
        cls[best] += 1
    out = {'path': path, 'n_state': int(len(st_t)), 'cls': cls}
    for src in ('ik', 'rob'):
        if res[src]:
            a = np.array(res[src])
            out[src] = (float(np.median(a[:, 0])), float(np.max(a[:, 0])),
                        float(np.median(a[:, 1])))
    out['rob_t_cover'] = (float(rob_t[0]), float(rob_t[-1]))
    out['state_t_cover'] = (float(st_t[0]), float(st_t[-1]))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('urdf')
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--frame', default='Link7L', help='左臂末端 frame（默认 Link7L）')
    args = ap.parse_args()
    import pinocchio as pin
    fk = FK(args.urdf, args.frame, LEFT)
    print(f'URDF: {args.urdf}\nframe: {args.frame}\n')
    for p in args.bags:
        r = audit_bag(p, fk, args.frame, pin)
        name = p.rsplit('/', 1)[-1]
        if 'skip' in r:
            print(f'{name:34s} 跳过：{r["skip"]}')
            continue
        c = r['cls']
        tot = max(1, r['n_state'])
        print(f'{name:34s} state={r["n_state"]:4d}  '
              f'源判定 ik={c["ik"]:4d}({c["ik"]*100//tot:2d}%) rob={c["rob"]:4d}({c["rob"]*100//tot:2d}%) 未匹配={c["none"]:4d}')
        if 'ik' in r:
            print(f'{"":34s}   vs FK(q_ik)  位置 median={r["ik"][0]:.3f}mm max={r["ik"][1]:.1f}mm  姿态 median={r["ik"][2]:.2f}°')
        if 'rob' in r:
            print(f'{"":34s}   vs FK(q_rob) 位置 median={r["rob"][0]:.3f}mm max={r["rob"][1]:.1f}mm  姿态 median={r["rob"][2]:.2f}°')


if __name__ == '__main__':
    main()
