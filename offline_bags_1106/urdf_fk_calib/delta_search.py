#!/usr/bin/env python3
import json, sys, itertools, math
import numpy as np
import pinocchio as pin

URDF = sys.argv[1]
raw = {"files": []}
samples = []
for f in ["calib_samples_L_1106_0943.jsonl","calib_L_drive.jsonl","calib_L2_drive.jsonl"]:
    prev=None
    for line in open(f):
        line=line.strip()
        if not line: continue
        s=json.loads(line)
        if prev is not None:
            dq = np.array(s["q17"][1:8], float) - np.array(prev["q17"][1:8], float)
            if np.linalg.norm(dq) > 0.15:
                samples.append((np.array(prev["q17"][1:8],float), dq,
                                np.array(s["tcp14"][0:3],float)-np.array(prev["tcp14"][0:3],float),
                                np.array(s["tcp14"][3:7],float), np.array(prev["tcp14"][3:7],float)))
        prev=s
print("有效位移对:", len(samples))

def quat_wfirst(v):
    w,x,y,z=[float(c) for c in v]
    n=math.sqrt(w*w+x*x+y*y+z*z); w,x,y,z=w/n,x/n,y/n,z/n
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

model = pin.buildModelFromUrdf(URDF)
data = model.createData()
names=[f"Joint{i}L" for i in range(1,8)]
iq=[model.joints[model.getJointId(n)].idx_q for n in names]
fid=model.getFrameId("Link7L")

def jac(qrow):
    q=pin.neutral(model)
    for k in range(7): q[iq[k]]=qrow[k]
    return pin.computeFrameJacobian(model,data,q,fid,pin.LOCAL_WORLD_ALIGNED)[:, iq]

pairs=[]
for qa,dq,dp,qt,qt0 in samples:
    dR = quat_wfirst(qt).T @ quat_wfirst(qt0)
    dw = pin.log3(dR)
    n = np.linalg.norm(dw)
    if n<1e-6: continue
    pd = dp/np.linalg.norm(dp) if np.linalg.norm(dp)>1e-6 else None
    wd = dw/n
    pairs.append((jac(qa), dq, pd, wd))
print("可用对:", len(pairs))

results=[]
for p in itertools.permutations(range(7)):
    idx=np.array(p)
    for sgv in itertools.product([-1,1],repeat=7):
        sg=np.array(sgv)
        cs_w=[]; cs_p=[]
        ok=True
        for J,dq,pd,wd in pairs:
            pred = J @ (sg*dq[idx])
            nw=np.linalg.norm(pred[3:])
            if nw<1e-9: ok=False; break
            cs_w.append(pred[3:]/nw @ wd)
            if pd is not None:
                npd=np.linalg.norm(pred[:3])
                if npd<1e-9: ok=False; break
                cs_p.append(pred[:3]/npd @ pd)
        if not ok or not cs_w: continue
        score=float(np.mean(cs_w)*2 + (np.mean(cs_p) if cs_p else 0.0))
        results.append((score, float(np.mean(cs_w)), float(np.mean(cs_p)) if cs_p else float("nan"), p, list(sgv)))
results.sort(reverse=True)
print("==== top12（score=2*角余弦+位置余弦）====")
for r in results[:12]:
    print(f"score={r[0]:+.3f} 角={r[1]:+.3f} 位={r[2]:+.3f} perm={r[3]} sign={r[4]}")
for tag,(pp,ss) in {"identity":((0,1,2,3,4,5,6),[1]*7),"全负":((0,1,2,3,4,5,6),[-1]*7)}.items():
    for r in results:
        if r[3]==pp and r[4]==ss:
            print(f"对照{tag}: score={r[0]:+.3f} 角={r[1]:+.3f} 位={r[2]:+.3f}")
            break
