#!/usr/bin/env python3
"""决定性验证：左链×候选符号 + 基座/工具拟合 → 姿态残差是否塌缩。"""
import json, math, sys, itertools
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
def quat_wfirst(v):
    return Rot.from_quat([float(v[1]),float(v[2]),float(v[3]),float(v[0])]).as_matrix()
model=pin.buildModelFromUrdf(URDF); data=model.createData()
qc=np.array([s["q17"][1:8] for s in samples],float)
tgt=[(np.array(s["tcp14"][0:3],float), quat_wfirst(s["tcp14"][3:7])) for s in samples]
names=[f"Joint{i}L" for i in range(1,8)]
iq=[model.joints[model.getJointId(n)].idx_q for n in names]
fid=model.getFrameId("Link7L")
def fk(qrow,sgn):
    q=pin.neutral(model)
    for k in range(7): q[iq[k]]=qrow[k]*sgn[k]
    pin.forwardKinematics(model,data,q); pin.updateFramePlacements(model,data)
    M=data.oMf[fid]; return np.array(M.translation), np.array(M.rotation)

def eval_sign(sgn):
    def resid(x):
        Rb=Rot.from_rotvec(x[:3]).as_matrix(); pb=x[3:6]; tool=x[6]
        r=[]
        for k in range(len(qc)):
            pm,Rm=fk(qc[k],sgn); pt,Rt=tgt[k]
            r.extend(pt-(Rb@(pm+Rm@np.array([0,0,tool]))+pb))
            r.extend(np.sin(pin.log3((Rb@Rm).T@Rt)))
        return np.array(r)
    best=None
    for t in range(40):
        x0=np.zeros(7); x0[:3]=np.random.uniform(-3.14,3.14,3); x0[3:6]=np.random.uniform(-1.2,1.2,3); x0[6]=np.random.uniform(-0.25,0.25)
        try: sol=least_squares(resid,x0,method='lm',max_nfev=6000)
        except Exception: continue
        f=sol.fun.reshape(-1,6)
        rp=float(np.sqrt(np.mean(np.sum(f[:,:3]**2,axis=1))))
        ra=math.degrees(float(np.sqrt(np.mean(np.sum(f[:,3:]**2,axis=1)))))
        if best is None or rp+ra<best[0]: best=(rp+ra,rp,ra,sol.x.copy())
    return best

cands=[
  ("delta搜索符号 [1,-1,1,-1,1,-1,1]", [1,-1,1,-1,1,-1,1]),
  ("全负 [-1]*7", [-1]*7),
  ("identity", [1]*7),
]
# 加：右链（执行器统一假设）delta符号
for tag,sgn in cands:
    r=eval_sign(np.array(sgn,float))
    print(f"{tag}: rms_pos={r[1]:.1f}mm rms_ang={r[2]:.1f}°")
    print(f"    基座pos={np.round(r[3][3:6],3)} tool={r[3][6]:+.3f} rot={np.round(np.degrees(r[3][:3]),1)}")
