#!/usr/bin/env python3
"""master_slave_map 离线回归（无 ROS、无 pinocchio，仅 numpy）。

层次（同 test_bridge_offline 的方法论：语义先离线断言，真机只做验收）：
  A. 恒等等价：默认参数必须与 ik_node 旧公式逐位一致（部署零行为变化的根据）
  B. 方向映射：轴交换/翻转/镜像 det 符号规则
  C. 旋转：共轭换基、伪向量 det 因子、缩放、Exp/Log 往返
  D. 六方向诊断表与泄漏
  E. 标定拟合：Procrustes 还原已知映射、残差
  F. 存取与门禁：JSON 往返、非法配置拒绝
"""
import math
import os
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..',
                                'ros2_ws', 'src', 'robot_ik_retargeting',
                                'robot_ik_retargeting'))
from master_slave_map import (ArmPoseMap, fit_orthogonal_map, fit_residual,  # noqa: E402
                              rotate_frame_omega, so3_exp, so3_log)


def rot_axis(axis, theta):
    return so3_exp(np.asarray(axis, dtype=float) * theta)


def legacy_map(mirror, T_exo_p, T_exo_R, T_exo0_p, T_exo0_R, T_robot0_p, T_robot0_R):
    """ik_node._incremental_desired 旧公式的逐位复刻（等价性对照的基准）。"""
    dp = (np.asarray(T_exo_p) - np.asarray(T_exo0_p)) * mirror
    p_des = np.asarray(T_robot0_p) + dp
    R_delta = np.asarray(T_exo_R) @ np.asarray(T_exo0_R).T
    R_des = R_delta @ np.asarray(T_robot0_R)
    return p_des, R_des


def rand_se3(rng):
    R = rot_axis(rng.normal(size=3), rng.uniform(0.1, 2.5))
    p = rng.normal(scale=0.4, size=3)
    return R, p


class TestIdentityEquivalence(unittest.TestCase):
    """A 类：默认参数 == 旧行为，这是「部署不改变现场行为」的合同。"""

    def test_default_matches_legacy_on_random_poses(self):
        rng = np.random.default_rng(42)
        for _ in range(50):
            Re, pe = rand_se3(rng)
            R0e, p0e = rand_se3(rng)
            R0r, p0r = rand_se3(rng)
            for mirror in ([1.0, 1.0, 1.0], [-1.0, 1.0, 1.0], [-1.0, -1.0, -1.0]):
                m = ArmPoseMap(position_map=np.diag(mirror))
                p_new, R_new = m.apply(Re, pe, R0e, p0e, R0r, p0r)
                p_old, R_old = legacy_map(mirror, pe, Re, p0e, R0e, p0r, R0r)
                np.testing.assert_allclose(p_new, p_old, atol=1e-12)
                np.testing.assert_allclose(R_new, R_old, atol=1e-9)

    def test_default_maps_are_identity(self):
        m = ArmPoseMap()
        np.testing.assert_array_equal(m.position_map, np.eye(3))
        np.testing.assert_array_equal(m.orientation_map, np.eye(3))
        self.assertEqual(m.position_scale, 1.0)
        self.assertEqual(m.orientation_scale, 1.0)


class TestDirectionMapping(unittest.TestCase):
    """B 类：任务书要求的逐轴诊断在模块层的对应物。"""

    def test_axis_flip_and_swap(self):
        # 外骨骼 X → 机器人 −Y，外骨骼 Y → 机器人 X（轴交换 + 翻转，mirror 修不了的那类）
        P = np.array([[0.0, 1.0, 0.0],
                      [-1.0, 0.0, 0.0],
                      [0.0, 0.0, 1.0]])
        m = ArmPoseMap(position_map=P)
        np.testing.assert_allclose(m.map_translation([0.1, 0.0, 0.0]),
                                   [0.0, -0.1, 0.0], atol=1e-12)
        np.testing.assert_allclose(m.map_translation([0.0, 0.2, 0.0]),
                                   [0.2, 0.0, 0.0], atol=1e-12)

    def test_scale_translation(self):
        m = ArmPoseMap(position_scale=0.3)
        np.testing.assert_allclose(m.map_translation([0.05, 0.0, 0.0]),
                                   [0.015, 0.0, 0.0], atol=1e-12)


class TestRotation(unittest.TestCase):
    """C 类：换基、det 符号、缩放、基本往返。"""

    def test_exp_log_roundtrip(self):
        rng = np.random.default_rng(7)
        for _ in range(100):
            omega = rng.normal(size=3)
            omega = omega * ((math.pi - 1e-3) / np.linalg.norm(omega))  # 约束 <π，log 取主值
            np.testing.assert_allclose(so3_log(so3_exp(omega)), omega, atol=1e-9)

    def test_proper_conjugation_changes_axis(self):
        # 绕 z 转 90° 的换基：外骨骼系绕 x 的旋转 → 机器人系绕 y
        C = rot_axis([0.0, 0.0, 1.0], math.pi / 2)
        R_delta = rot_axis([1.0, 0.0, 0.0], 0.3)
        m = ArmPoseMap(orientation_map=C)
        expected = rot_axis([0.0, 1.0, 0.0], 0.3)  # det(C)=+1 → 无符号翻转
        np.testing.assert_allclose(m.map_rotation(R_delta), expected, atol=1e-9)

    def test_improper_map_needs_det_sign(self):
        # 镜像阵 det=−1：log(C R Cᵀ) = det(C)·C·ω，漏符号会反转转轴方向
        C = np.diag([-1.0, 1.0, 1.0])
        omega = np.array([0.0, 0.0, 0.4])  # 外骨骼系绕 z 转
        R_delta = so3_exp(omega)
        np.testing.assert_allclose(
            rotate_frame_omega(C, so3_log(R_delta)),
            np.array([0.0, 0.0, -0.4]), atol=1e-12)

    def test_orientation_scale(self):
        # 任务书验收数字：外骨骼单轴旋转 20°，scale=0.2 → 机器人 4°
        m = ArmPoseMap(orientation_scale=0.2)
        R = m.map_rotation(rot_axis([0.0, 1.0, 0.0], math.radians(20.0)))
        theta = np.linalg.norm(so3_log(R))
        self.assertAlmostEqual(math.degrees(theta), 4.0, places=4)


class TestAxisReport(unittest.TestCase):
    """D 类：六方向诊断表的 main/leak 语义。"""

    def test_report_flags_leak_on_axis_swap(self):
        P = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
        rep = ArmPoseMap(position_map=P).translation_axis_report()
        self.assertEqual(rep['+X']['main'], 'Y')   # 主轴判定跟随映射
        self.assertAlmostEqual(rep['+X']['gain'], -1.0, places=9)  # +X → −Y，带符号
        self.assertAlmostEqual(rep['+X']['leak'], 0.0, places=9)  # 完美映射无泄漏

    def test_report_identity(self):
        rep = ArmPoseMap().translation_axis_report()
        for k, v in rep.items():
            self.assertEqual(v['main'], k[1])
            self.assertAlmostEqual(v['leak'], 0.0, places=12)


class TestCalibrationFit(unittest.TestCase):
    """E 类：六方向样本 → Procrustes 还原已知映射。"""

    def test_recover_known_map_from_six_direction_samples(self):
        true_P = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
        rng = np.random.default_rng(11)
        src = np.concatenate([np.eye(3), -np.eye(3)]) * 0.05  # ±X/±Y/±Z 六方向
        dst = src @ true_P.T + rng.normal(scale=1e-4, size=(6, 3))  # 1e-4 m 噪声
        P = fit_orthogonal_map(src, dst)
        np.testing.assert_allclose(P, true_P, atol=1e-3)
        # 噪声 1e-4/分量 → 残差上界 ~3σ；真机验收门限是 3mm，这里只验拟合正确性
        self.assertLess(fit_residual(P, src, dst), 1e-3)


class TestPersistenceAndGate(unittest.TestCase):
    """F 类：存取往返 + 非法配置硬拒绝。"""

    def _sample_map(self):
        return ArmPoseMap(position_map=rot_axis([0.0, 0.0, 1.0], 0.7),
                          position_scale=0.3,
                          orientation_map=rot_axis([0.0, 1.0, 0.0], -0.4),
                          orientation_scale=0.2)

    def test_json_roundtrip(self):
        m = self._sample_map()
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, 'left.json')
            m.save_json(path, meta={'date': '2026-11-06'})
            m2 = ArmPoseMap.load_json(path)
            for k in m.to_dict():
                np.testing.assert_allclose(np.asarray(m.to_dict()[k]),
                                           np.asarray(m2.to_dict()[k]), atol=1e-12)

    def test_bad_json_format_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, 'bad.json')
            with open(path, 'w') as f:
                f.write('{"format": "unknown"}')
            with self.assertRaises(ValueError):
                ArmPoseMap.load_json(path)

    def test_non_orthogonal_rejected(self):
        with self.assertRaises(ValueError):
            ArmPoseMap(position_map=np.diag([2.0, 1.0, 1.0]))
        with self.assertRaises(ValueError):
            ArmPoseMap(orientation_map=np.array([[1.0, 0.1, 0.0],
                                                 [0.0, 1.0, 0.0],
                                                 [0.0, 0.0, 1.0]]))

    def test_out_of_range_scale_rejected(self):
        with self.assertRaises(ValueError):
            ArmPoseMap(position_scale=0.0)
        with self.assertRaises(ValueError):
            ArmPoseMap(position_scale=5.0)
        with self.assertRaises(ValueError):
            ArmPoseMap(orientation_scale=-1.0)


if __name__ == '__main__':
    unittest.main(verbosity=2)
