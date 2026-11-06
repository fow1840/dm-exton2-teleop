#!/usr/bin/env python3
"""左臂映射搜索第二级：置换×符号候选 + LM 零偏拟合。"""
import json, sys, itertools, math
import numpy as np
import pinocchio as pin
from scipy.optimize import least_squares

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
iq = [model.joints[model.getJointId(n)].idx_q for n in names]
fid = model.getFrameId("Link7L")

qc = np.array([s["q17"][1:8] for s in samples])
tgt=[(np.array(s["tcp14"][0:3]), quat_wfirst(s["tcp14"][3:7])) for s in samples]

def fk(qrow):
    q = pin.neutral(model)
    for k in range(7): q[iq[k]] = qrow[k]
    pin.forwardKinematics(model,data,q)
    pin.updateFramePlacements(model,data)
    M=data.oMf[fid]
    return np.array(M.translation), np.array(M.rotation)

def resid(x, perm, sg):
    off=x[:7]
    Q = qc[:,perm]*sg + off
    r=[]
    for k in range(len(Q)):
        pm,Rm=fk(Q[k]); pt,Rt=tgt[k]
        r.extend((pt-pm)*1000)
        r.extend(pin.log3(Rm.T@Rt))
    return np.array(r)

perms = list(itertools.permutations(range(7)))
signs = [np.array(s) for s in itertools.product([-1,1],repeat=7)]
cands=[]
for p in perms:
    Q=qc[:,p]
    for sg in signs:
        Qs=Q*sg
        errs=[]
        for k in range(len(Qs)):
            pm,Rm=fk(Qs[k]); pt,Rt=tgt[k]
            errs.append(np.linalg.norm(pt-pm)*1000 + 2*np.linalg.norm(pin.log3(Rm.T@Rt))*180/math.pi)
        cands.append((np.median(errs),p,list(sg)))
cands.sort(key=lambda c:c[0])

best=None
for score,p,sg in cands[:400]:
    try:
        sol=least_squares(resid, np.zeros(7), args=(p,np.array(sg)), method='lm', max_nfev=4000)
    except Exception:
        continue
    r=resid(sol.x,p,np.array(sg))
    pos=np.linalg.norm(r[:3]); ang=math.degrees(np.linalg.norm(r[3:6]))
    rms=np.sqrt(np.mean(np.sum(r.reshape(-1,6)[:,0:3]**2,axis=1)))
    if best is None or rms<best[0]:
        best=(rms,sol.x.copy(),p,sg,pos,ang)
    if rms < 1.0:
        print(f"命中! rms_pos={rms:.2f}mm perm={p} sign={list(sg)}")
        print("  偏移deg:", [round(math.degrees(o),1) for o in sol.x])
        break
if best:
    rms,x,p,sg,pos,ang=best
    print(f"最优: rms_pos={rms:.2f}mm  末样本 pos={pos:.1f}mm ang={ang:.1f}° perm={p} sign={list(sg)}")
    print("  偏移deg:", [round(math.degrees(o),1) for o in x])
