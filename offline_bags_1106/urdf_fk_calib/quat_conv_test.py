#!/usr/bin/env python3
"""四元数约定 × 符号 联合判决。约定: wxyz / xyzw / 共轭变体。"""
import json, math, sys
import numpy as np
import pinocchio as pin
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot

URDF=sys.argv[1]
samples=[]
for f in ["calib_samples_L_1106_0943.jsonl","calib_L_drive.jsonl","calib_L2_drive.jsonl"]:
    for line in open(f):
        line=line.strip()
        if line: samples.append(json.loads(line))
model=pin.buildModelFromUrdf(URDF); data=model.createData()
qc=np.array([s["q17"][1:8] for s in samples],float)
raw=[np.array(s["tcp14"][3:7],float) for s in samples]
pos=[np.array(s["tcp14"][0:3],float) for s in samples]
names=[f"Joint{i}L" for i in range(1,8)]
iq=[model.joints[model.getJointId(n)].idx_q for n in names]
fid=model.getFrameId("Link7L")
def fk(qrow,sgn):
    q=pin.neutral(model)
    for k in range(7): q[iq[k]]=qrow[k]*sgn[k]
    pin.forwardKinematics(model,data,q); pin.updateFramePlacements(model,data)
    M=data.oMf[fid]; return np.array(M.translation), np.array(M.rotation)
def conv(v,mode):
    if mode=="wxyz": r=[v[1],v[2],v[3],v[0]]
    elif mode=="xyzw": r=[v[0],v[1],v[2],v[3]]
    elif mode=="wxyz_inv": r=[v[1],-v[2],-v[3],-v[0]] if False else [v[1],-v[2],-v[3],v[0]]
    elif mode=="xyzw_inv": r=[-v[0],-v[1],-v[2],v[3]]
    return Rot.from_quat(r).as_matrix()
def eval_cfg(mode,sgn):
    def resid(x):
        Rb=Rot.from_rotvec(x[:3]).as_matrix(); pb=x[3:6]; tool=x[6]
        r=[]
        for k in range(len(qc)):
            pm,Rm=fk(qc[k],sgn); Rt=conv(raw[k],mode)
            r.extend(pos[k]-(Rb@(pm+Rm@np.array([0,0,tool]))+pb))
            r.extend(np.sin(pin.log3((Rb@Rm).T@Rt)))
        return np.array(r)
    best=None
    for t in range(24):
        x0=np.zeros(7); x0[:3]=np.random.uniform(-3.14,3.14,3); x0[3:6]=np.random.uniform(-1.2,1.2,3); x0[6]=np.random.uniform(-0.25,0.25)
        try: sol=least_squares(resid,x0,method='lm',max_nfev=6000)
        except Exception: continue
        f=sol.fun.reshape(-1,6)
        rp=float(np.sqrt(np.mean(np.sum(f[:,:3]**2,axis=1))))
        ra=math.degrees(float(np.sqrt(np.mean(np.sum(f[:,3:]**2,axis=1)))))
        if best is None or rp+ra<best[0]: best=(rp+ra,rp,ra,sol.x.copy())
    return best
for mode in ["wxyz","xyzw","wxyz_inv","xyzw_inv"]:
    for tag,sgn in [("id",[1]*7),("全负",[-1]*7),("Δ符号",[1,-1,1,-1,1,-1,1])]:
        r=eval_cfg(mode,np.array(sgn,float))
        flag=" ←←← 命中" if r[2]<5 else ""
        print(f"{mode:9s} {tag}: pos={r[1]:6.1f}mm ang={r[2]:6.1f}° tool={r[3][6]:+.3f}{flag}")
