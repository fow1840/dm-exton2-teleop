#!/usr/bin/env python3
"""主从末端增量遥操映射模块（纯 numpy，无 ROS/pinocchio 依赖，可离线单测）。

为什么存在（T27/T29/T30 结论链）：
  真机方向混乱的根因是 URDF 与控制器运动学结构性不一致（T20：位置残差 292~1895mm），
  且主从之间只用「逐分量 mirror 翻符号」，修不了轴交换/任意基旋转（T29）。
  本模块把「外骨骼增量 → 机器人基坐标系增量」的映射抽成可配置、可标定、可保存的
  纯逻辑对象；IK（ik_node._incremental_desired）只调用它，不再内嵌方向假设。

语义（与 2026-11-06 现行为逐位等价的默认值）：
  平移:  dp_robot = position_scale · P_map · (p_exo − p_exo0)
         P_map 默认 = diag(mirror_xyz_side)，position_scale 默认 1.0
  姿态:  R_delta = R_exo · R_exo0ᵀ            （外骨骼系下的相对旋转）
         ω_robot = orientation_scale · sgn(det(Q)) · Q · log(R_delta)
         R_des    = Exp(ω_robot) · R_robot0
         Q 默认 = I（现行为姿态无坐标变换）；sgn(det) 项是伪向量在正交变换下的
         符号规则——镜像阵（det=−1）共轭会把转轴反向，不补符号就悄悄翻转向。
  初始位: p_robot0/R_robot0 与 p_exo0/R_exo0 来自离合按下时的锚定（ik_node 现有
         anchor 机制），本模块不做锚定，只消费锚。

安全设计：
  · 默认值 == 旧行为：部署本模块本身不改变任何运动方向；只有加载标定文件才变。
  · validate() 拒绝非正交映射阵与超范围缩放（0<scale≤2），加载坏文件直接抛错，
    不允许「半生效」状态。
  · 六方向诊断/Procrustes 拟合的配套工具见 tools/dm-exo/sixdir_calib.py（门控脚本，
    真机采集只在操作员确认下进行）。
"""
import json
import math

import numpy as np

# 缩放的合法范围：既防手抖配 0（臂死），也防配出放大器（危险）
POSITION_SCALE_RANGE = (0.05, 2.0)
ORIENTATION_SCALE_RANGE = (0.05, 2.0)


def so3_exp(omega: np.ndarray) -> np.ndarray:
    """旋转向量 → 旋转矩阵（Rodrigues）。|omega|=0 恒等处理。"""
    omega = np.asarray(omega, dtype=float)
    theta = float(np.linalg.norm(omega))
    if theta < 1e-12:
        return np.eye(3)
    axis = omega / theta
    K = np.array([[0.0, -axis[2], axis[1]],
                  [axis[2], 0.0, -axis[0]],
                  [-axis[1], axis[0], 0.0]])
    return np.eye(3) + math.sin(theta) * K + (1.0 - math.cos(theta)) * (K @ K)


def so3_log(R: np.ndarray) -> np.ndarray:
    """旋转矩阵 → 旋转向量（轴角）。|theta|≤π 分支；恒等阵返回零向量。

    theta 接近 π 时用 trace<0 分支（标准公式），避免轴长塌缩。
    输入含非正交阵时不做修正（调用方应先 validate）；误差上抛为 NaN 由测试兜底。
    """
    R = np.asarray(R, dtype=float)
    cos_theta = (np.trace(R) - 1.0) / 2.0
    cos_theta = max(-1.0, min(1.0, cos_theta))
    theta = math.acos(cos_theta)
    if theta < 1e-12:
        return np.zeros(3)
    if theta > math.pi - 1e-6:
        # trace<0：theta≈π，轴从对称部分提取
        A = (R + np.eye(3)) / 2.0
        axis = np.sqrt(np.maximum(np.diag(A), 0.0))
        k = int(np.argmax(axis))
        if axis[k] > 1e-9:
            axis = A[:, k] / axis[k]
        axis = axis / max(np.linalg.norm(axis), 1e-12)
        return axis * theta
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    axis = axis / (2.0 * math.sin(theta))
    return axis * theta


def rotate_frame_omega(Q: np.ndarray, omega: np.ndarray) -> np.ndarray:
    """伪向量（旋转向量）在正交变换 Q 下的坐标变换：ω' = det(Q)·Q·ω。

    为什么有 det 因子：真向量基变换是 Q·v；旋转向量是轴角伪向量，镜面反射
    （det=−1）在换基的同时反转定向，若漏掉符号，轴会以「反转后的转角」出现。
    """
    return float(np.linalg.det(Q)) * (np.asarray(Q, dtype=float) @ np.asarray(omega, dtype=float))


class ArmPoseMap:
    """单臂主从位姿增量映射。默认参数 == 2026-11-06 之前的现行为。"""

    def __init__(self,
                 position_map: np.ndarray = None,
                 position_scale: float = 1.0,
                 orientation_map: np.ndarray = None,
                 orientation_scale: float = 1.0):
        self.position_map = (np.eye(3) if position_map is None
                             else np.asarray(position_map, dtype=float).reshape(3, 3))
        self.position_scale = float(position_scale)
        self.orientation_map = (np.eye(3) if orientation_map is None
                                else np.asarray(orientation_map, dtype=float).reshape(3, 3))
        self.orientation_scale = float(orientation_scale)
        self.validate()

    # ---- 核心语义 ------------------------------------------------------------
    def map_translation(self, dp_exo) -> np.ndarray:
        """外骨骼系平移增量 → 机器人基系平移增量。"""
        return self.position_scale * (self.position_map @ np.asarray(dp_exo, dtype=float))

    def map_rotation(self, R_delta) -> np.ndarray:
        """外骨骼系相对旋转矩阵 → 机器人基系相对旋转矩阵（含缩放）。"""
        omega_robot = self.orientation_scale * rotate_frame_omega(
            self.orientation_map, so3_log(np.asarray(R_delta, dtype=float)))
        return so3_exp(omega_robot)

    def apply(self, R_exo, p_exo, R_exo0, p_exo0, R_robot0, p_robot0):
        """完整增量遥操一步：返回 (p_des, R_des)。

        入参与 ik_node._incremental_desired 的锚一致：exo0/robot0 为按下离合时刻
        的双侧锚定位姿。返回 numpy 数组，调用方自行包 SE3。
        """
        dp = np.asarray(p_exo, dtype=float) - np.asarray(p_exo0, dtype=float)
        p_des = np.asarray(p_robot0, dtype=float) + self.map_translation(dp)
        R_delta = np.asarray(R_exo, dtype=float) @ np.asarray(R_exo0, dtype=float).T
        R_des = self.map_rotation(R_delta) @ np.asarray(R_robot0, dtype=float)
        return p_des, R_des

    # ---- 诊断辅助 ------------------------------------------------------------
    def translation_axis_report(self, move_m: float = 1.0) -> dict:
        """六方向诊断表：外骨骼 ±X/±Y/±Z 移动 move_m 米 → 机器人各轴响应。

        返回 {'+X': {'robot': [x,y,z], 'main': 'X', 'gain': 带符号主轴增益,
        'leak': max|非主轴|（米）}, ...}。leak 与输入同量纲（move_m=0.05 时
        即为 50mm 标定动作下的泄漏米数，可直接对照 3mm 验收门限）。
        """
        report = {}
        for i, name in enumerate('XYZ'):
            for sign in (+1.0, -1.0):
                out = self.map_translation(sign * np.eye(3)[i] * move_m)
                main_idx = int(np.argmax(np.abs(out)))
                report[f'{"+" if sign > 0 else "-"}{name}'] = {
                    'robot': [float(v) for v in out],
                    'main': 'XYZ'[main_idx],
                    'gain': float(out[main_idx] / sign),
                    'leak': float(np.max(np.abs(np.delete(out, main_idx)))),
                }
        return report

    # ---- 校验 ---------------------------------------------------------------
    def validate(self) -> None:
        """启动/加载时硬门禁：非法配置直接抛 ValueError，禁止半生效。"""
        for name, M in (('position_map', self.position_map),
                        ('orientation_map', self.orientation_map)):
            if not np.allclose(M @ M.T, np.eye(3), atol=1e-6):
                raise ValueError(f'{name} 不是正交阵（容差 1e-6）')
            if not np.isclose(abs(np.linalg.det(M)), 1.0, atol=1e-6):
                raise ValueError(f'{name} 行列式异常')
        for name, s, rng in (('position_scale', self.position_scale, POSITION_SCALE_RANGE),
                             ('orientation_scale', self.orientation_scale,
                              ORIENTATION_SCALE_RANGE)):
            if not (rng[0] <= s <= rng[1]):
                raise ValueError(f'{name}={s} 超出安全范围 {rng}')

    # ---- 存取 ---------------------------------------------------------------
    def to_dict(self) -> dict:
        return {
            'position_map': self.position_map.tolist(),
            'position_scale': self.position_scale,
            'orientation_map': self.orientation_map.tolist(),
            'orientation_scale': self.orientation_scale,
        }

    @classmethod
    def from_dict(cls, d: dict) -> 'ArmPoseMap':
        return cls(position_map=d['position_map'],
                   position_scale=d.get('position_scale', 1.0),
                   orientation_map=d['orientation_map'],
                   orientation_scale=d.get('orientation_scale', 1.0))

    def save_json(self, path: str, meta: dict = None) -> None:
        """标定结果落盘。meta 记录日期/样本包名/验收数据，便于追溯。"""
        payload = {'format': 'arm_pose_map_v1', 'meta': meta or {}, **self.to_dict()}
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)

    @classmethod
    def load_json(cls, path: str) -> 'ArmPoseMap':
        with open(path, 'r', encoding='utf-8') as f:
            payload = json.load(f)
        if payload.get('format') != 'arm_pose_map_v1':
            raise ValueError(f'标定文件格式不符: {payload.get("format")}')
        return cls.from_dict(payload)


# ---- 标定拟合（Kabsch/正交 Procrustes）---------------------------------------
def fit_orthogonal_map(src_points, dst_points) -> np.ndarray:
    """由配平移样本 {(dp_exo, dp_robot)} 拟合 3×3 正交映射 P：dst ≈ P·src。

    正交 Procrustes（SVD）：六方向标定的核心步骤。样本要求 ≥3 个不共线方向；
    返回的 P 含反射（det=−1）时是合法结果（镜像主从关系本来就存在），validate 会放行。
    残差由调用方计算并对照验收门限（<3mm 泄漏）。
    """
    src = np.asarray(src_points, dtype=float)  # (N,3)
    dst = np.asarray(dst_points, dtype=float)  # (N,3)
    if src.shape != dst.shape or src.shape[0] < 3:
        raise ValueError('拟合样本不足 3 组或维度不符')
    H = src.T @ dst
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    D = np.diag([1.0, 1.0, d])
    return Vt.T @ D @ U.T


def fit_residual(P: np.ndarray, src_points, dst_points) -> float:
    """拟合后的最大残差（与 dst 同量纲，位置标定即米）。"""
    src = np.asarray(src_points, dtype=float)
    dst = np.asarray(dst_points, dtype=float)
    return float(np.max(np.linalg.norm(src @ P.T - dst, axis=1)))
