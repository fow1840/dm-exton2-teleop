#!/usr/bin/env python3
"""左臂轴排列×符号全枚举：控制器 L1..L7 → URDF Joint1L..7L 找映射。"""
import json, sys, itertools, math
import numpy as np
import pinocchio as pin

URDF = sys.argv[1]
samples = []
for f in ["calib_samples_L_1106_0943.jsonl","calib_L_drive.jsonl","calib_L2_drive.jsonl"]:
    for line in open(f):
        line=line.strip()
        if line: samples.append(json.loads(line))

def quat_wfirst(v):
    w,x,y,z=[float(c) for c in v]
    n=math.sqrt(w*w+x*x+y*y+z*z); w,x,y,z=w/n,x/n,y/n,z/n
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

model = pin.buildModelFromUrdf(URDF)
data = model.createData()
names = [f"Joint{i}L" for i in range(1,8)]
jid = [model.getJointId(n) for n in names]
iq = [model.joints[j].idx_q for j in jid]
fid = model.getFrameId("Link7L")

# 预提取：q_ctrl (N×7), p_tcp, R_tcp
qc = np.array([s["q17"][1:8] for s in samples])
tgt=[]
for s in samples:
    t=s["tcp14"]
    tgt.append((np.array(t[0:3]), quat_wfirst(t[3:7])))

def fk_vec(qrow):
    q = pin.neutral(model)
    for k in range(7): q[iq[k]] = qrow[k]
    pin.forwardKinematics(model,data,q)
    pin.updateFramePlacements(model,data)
    M = data.oMf[fid]
    return np.array(M.translation), np.array(M.rotation)

best=[]
perms = list(itertools.permutations(range(7)))
signs = [np.array(s) for s in itertools.product([-1,1],repeat=7)]
results=[]
for p in perms:
    Q = qc[:,p]
    for sg in signs:
        Qs = Q*sg
        errs=[]
        for k in range(len(Qs)):
            pm,Rm = fk_vec(Qs[k])
            pt,Rt = tgt[k]
            em = np.linalg.norm(pt-pm)*1000
            ed = np.linalg.norm(pin.log3(Rm.T@Rt))*180/math.pi
            errs.append((em,ed))
        arr=np.array(errs)
        med_pos=np.median(arr[:,0]); med_ang=np.median(arr[:,1])
        results.append((max(med_pos,med_ang*2), med_pos, med_ang, p, list(sg)))
results.sort(key=lambda r:r[0])
for r in results[:8]:
    print(f"score={r[0]:8.1f}  pos_med={r[1]:8.1f}mm  ang_med={r[2]:7.1f}°  perm={r[3]} sign={r[4]}")
