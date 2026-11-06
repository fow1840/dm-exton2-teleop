#!/usr/bin/env python3
"""假设检验：真机左臂 = 右手执行器。URDF右链FK(左q17) + 基座变换+工具 → 真机左tcp。"""
import json, math, sys
import numpy as np
import pinocchio as pin
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot

URDF = sys.argv[1]
samples=[]
for f in ["calib_samples_L_1106_0943.jsonl","calib_L_drive.jsonl","calib_L2_drive.jsonl"]:
    for line in open(f):
        line=line.strip()
        if line: samples.append(json.loads(line))

def quat_wfirst(v):
    return Rot.from_quat([float(v[1]),float(v[2]),float(v[3]),float(v[0])]).as_matrix()

model = pin.buildModelFromUrdf(URDF)
data = model.createData()
names=[f"Joint{i}" for i in range(1,8)]   # 右链
iq=[model.joints[model.getJointId(n)].idx_q for n in names]
fid=model.getFrameId("Link7")

qc=np.array([s["q17"][1:8] for s in samples],float)
tgt=[(np.array(s["tcp14"][0:3],float), quat_wfirst(s["tcp14"][3:7])) for s in samples]

def fk(qrow):
    q=pin.neutral(model)
    for k in range(7): q[iq[k]]=qrow[k]
    pin.forwardKinematics(model,data,q); pin.updateFramePlacements(model,data)
    M=data.oMf[fid]
    return np.array(M.translation), np.array(M.rotation)

def resid(x):
    Rb=Rot.from_rotvec(x[:3]).as_matrix(); pb=x[3:6]; tool=x[6]
    r=[]
    for k in range(len(qc)):
        pm,Rm=fk(qc[k])
        pt,Rt=tgt[k]
        p_model = Rb@(pm + Rm@np.array([0,0,tool])) + pb
        R_model = Rb@Rm
        r.extend(pt-p_model)
        r.extend(np.sin(pin.log3(R_model.T@Rt)))
    return np.array(r)

best=None
for trial in range(20):
    x0=np.zeros(7); x0[:3]=np.random.uniform(-3,3,3); x0[3:6]=np.random.uniform(-0.5,0.5,3); x0[6]=np.random.uniform(-0.2,0.2)
    try:
        sol=least_squares(resid,x0,method='lm',max_nfev=8000)
    except Exception: continue
    rms=float(np.sqrt(np.mean(np.sum(sol.fun.reshape(-1,6)[:,:3]**2,axis=1))))
    ang=math.degrees(float(np.sqrt(np.mean(np.sum(sol.fun.reshape(-1,6)[:,3:]**2,axis=1)))))
    if best is None or rms<best[0]:
        best=(rms,ang,sol.x.copy())
rms,ang,x=best
print(f"最优 rms_pos={rms:.1f}mm rms_ang={ang:.1f}°")
print("基座rotvec(deg):", np.round(np.degrees(x[:3]),1))
print("基座pos(m):", np.round(x[3:6],3))
print("工具z(m):", round(float(x[6]),3))
# 对照：同样拟合但用左链（Joint{i}L）identity 映射
namesL=[f"Joint{i}L" for i in range(1,8)]
iqL=[model.joints[model.getJointId(n)].idx_q for n in namesL]
fidL=model.getFrameId("Link7L")
def fkL(qrow):
    q=pin.neutral(model)
    for k in range(7): q[iqL[k]]=qrow[k]
    pin.forwardKinematics(model,data,q); pin.updateFramePlacements(model,data)
    M=data.oMf[fidL]
    return np.array(M.translation), np.array(M.rotation)
def residL(x):
    Rb=Rot.from_rotvec(x[:3]).as_matrix(); pb=x[3:6]; tool=x[6]
    r=[]
    for k in range(len(qc)):
        pm,Rm=fkL(qc[k])
        pt,Rt=tgt[k]
        r.extend(pt-(Rb@(pm + Rm@np.array([0,0,tool])) + pb))
        r.extend(np.sin(pin.log3((Rb@Rm).T@Rt)))
    return np.array(r)
best=None
for trial in range(20):
    x0=np.zeros(7); x0[:3]=np.random.uniform(-3,3,3); x0[3:6]=np.random.uniform(-0.5,0.5,3); x0[6]=np.random.uniform(-0.2,0.2)
    try:
        sol=least_squares(residL,x0,method='lm',max_nfev=8000)
    except Exception: continue
    rms=float(np.sqrt(np.mean(np.sum(sol.fun.reshape(-1,6)[:,:3]**2,axis=1))))
    if best is None or rms<best[0]: best=(rms,sol.x.copy())
print(f"对照(左链identity+基座拟合): rms_pos={best[0]:.1f}mm")
