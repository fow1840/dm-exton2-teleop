#!/usr/bin/env python3
"""exton_bridge 离线回归（不连真机、不依赖 ROS）。

三层：
  L1 StepPlanner 纯逻辑单测（迟滞 / 步进底 / leash / 复位语义）
  L2 闭环仿真：桥计划 ↔ SimRobot（movej 程序语义模型），复现 hunt 并验证修复
  L3 AsyncTcpClient ↔ FakeController 真实 socket（lock-step 节拍 / R4 重连 / R3 反馈年龄）

运行：python3 tools/tests/test_bridge_offline.py   （stdlib unittest，无第三方依赖）
对照：UBT test_pink_bridge_safety.cpp 的做法——安全语义先离线断言，真机只做验收。
"""
import math
import sys
import time
import types
import unittest

# ---- ROS 桩：exton_bridge 顶层 import rclpy/sensor_msgs，本机无 ROS ----
_rclpy = types.ModuleType('rclpy')
_rclpy_node = types.ModuleType('rclpy.node')
_rclpy_node.Node = object
_sensor_msgs = types.ModuleType('sensor_msgs')
_sensor_msgs_msg = types.ModuleType('sensor_msgs.msg')


class _JointState:  # noqa: D101 - 仅类型占位
    pass


_sensor_msgs_msg.JointState = _JointState
_sensor_msgs.msg = _sensor_msgs_msg
sys.modules.setdefault('rclpy', _rclpy)
sys.modules.setdefault('rclpy.node', _rclpy_node)
sys.modules.setdefault('sensor_msgs', _sensor_msgs)
sys.modules.setdefault('sensor_msgs.msg', _sensor_msgs_msg)

sys.path.insert(0, sys.path[0])  # 与本测试同目录即 tools/tests；再补上级 tools/
sys.path.insert(0, __file__.rsplit('/', 2)[0])

import exton_bridge as eb  # noqa: E402
from fake_controller import FakeController  # noqa: E402

LEFT_NAMES = eb.LEFT_URDF_JOINTS
RIGHT_NAMES = eb.RIGHT_URDF_JOINTS
URDF_LIM = eb.URDF_LIMITS_RAD


def make_planner():
    return eb.StepPlanner([
        (LEFT_NAMES, URDF_LIM, eb.IDX_LEFT_START, True),
        (RIGHT_NAMES, URDF_LIM, eb.IDX_RIGHT_START, True),
    ])


def ik_map_left(values):
    return {n: v for n, v in zip(LEFT_NAMES, values)}


class SimRobot:
    """movej 程序语义的离散模型：接受目标后按模型一步演化反馈 fb。

    model:
      exact      一步到位（理想伺服）
      overshoot  到位后交替方向过冲 overshoot_rad（疑似 T12 自振的最简模型）
      sag        到位后恒降 sag_rad（UBT 2026-10-13 重力下垂回归的模型）
    闭环节奏：每次 plan.updated 视为一条 movej 下发并即刻执行（lock-step 节拍
    在 L3 用真实 socket 单测，这里只研究控制语义）。
    """

    def __init__(self, model='overshoot', overshoot_rad=0.035):
        self.fb = [0.0] * eb.N_JOINTS
        self.model = model
        self.overshoot_rad = overshoot_rad
        self.sign = 1.0
        self.sends = 0
        self.cmd_log = []

    def tick(self, planner, ik_values, **plan_kwargs):
        plan = planner.plan(self.fb, ik_map_left(ik_values), **plan_kwargs)
        if plan.updated:
            self.sends += 1
            self.cmd_log.append(list(plan.joints))
            for i, v in enumerate(plan.joints):
                if i < eb.IDX_LEFT_START or i >= eb.IDX_LEFT_START + 7:
                    continue
                prev = self.fb[i]
                if abs(v - prev) <= 1e-9:
                    continue
                if self.model == 'exact':
                    self.fb[i] = v
                elif self.model == 'overshoot':
                    self.sign = -self.sign
                    self.fb[i] = v + self.sign * self.overshoot_rad
                elif self.model == 'sag':
                    self.fb[i] = v - self.overshoot_rad
        return plan


OLD_BEHAVIOUR = dict(step_base='feedback', eps_rad=0.001, hold_enter_rad=0.001,
                     hold_rearm_rad=0.001, jump_rad=3.0, max_step_rad=0.20,
                     follow_ratio=1.0, leash_rad=0.6, ik_moved=False)
T14_DEFAULTS = dict(OLD_BEHAVIOUR, hold_enter_rad=0.01, hold_rearm_rad=0.05)
T17_COMMAND = dict(T14_DEFAULTS, step_base='command')


class TestStepPlannerUnit(unittest.TestCase):
    """L1：纯逻辑断言（含与旧 apply_arm 的等价回归）。"""

    def test_feedback_mode_matches_old_apply_arm(self):
        """feedback 模式 + 关迟滞 = 旧 apply_arm 逐位一致（回归基线）。"""
        p = make_planner()
        fb = [0.0] * eb.N_JOINTS
        fb[eb.IDX_LEFT_START + 3] = 0.10   # Joint4L 现状
        ik = [0.0] * 7
        ik[3] = 0.15                       # IK 要 +0.05
        ik[4] = 0.0
        plan = p.plan(fb, ik_map_left(ik), **OLD_BEHAVIOUR)
        self.assertTrue(plan.updated)
        # 旧逻辑：joints = fb + clip(err*ratio, ±max_step)
        expect = fb[eb.IDX_LEFT_START + 3] + min(0.05, 0.20)
        self.assertAlmostEqual(plan.joints[eb.IDX_LEFT_START + 3], expect, places=12)
        # 未提及的轴保持 fb
        self.assertEqual(plan.joints[eb.IDX_LEFT_START], fb[eb.IDX_LEFT_START])

    def test_eps_gate_keeps_joint(self):
        p = make_planner()
        fb = [0.0] * eb.N_JOINTS
        plan = p.plan(fb, ik_map_left([0.0005, 0, 0, 0, 0, 0, 0]), **OLD_BEHAVIOUR)
        self.assertFalse(plan.updated)

    def test_jump_skip(self):
        p = make_planner()
        fb = [0.0] * eb.N_JOINTS
        plan = p.plan(fb, ik_map_left([3.5, 0, 0, 0, 0, 0, 0]), **OLD_BEHAVIOUR)
        self.assertFalse(plan.updated)
        self.assertEqual(plan.skipped, 1)

    def test_command_mode_ramp_from_last_plan(self):
        """command 模式：斜坡底是上一拍计划值，不受 fb 拖拽。"""
        p = make_planner()
        fb = [0.0] * eb.N_JOINTS
        ik = [0.5, 0, 0, 0, 0, 0, 0]
        p1 = p.plan(fb, ik_map_left(ik), **T17_COMMAND)
        self.assertAlmostEqual(p1.joints[eb.IDX_LEFT_START], 0.20, places=12)
        p2 = p.plan(fb, ik_map_left(ik), **T17_COMMAND)   # fb 没动（假想执行延迟）
        self.assertAlmostEqual(p2.joints[eb.IDX_LEFT_START], 0.40, places=12)
        # fb 已跟上 0.40 后：base 仍是计划 0.40 → 直接到 0.5
        fb[eb.IDX_LEFT_START] = 0.40
        p3 = p.plan(fb, ik_map_left(ik), **T17_COMMAND)
        self.assertAlmostEqual(p3.joints[eb.IDX_LEFT_START], 0.50, places=12)
        p4 = p.plan(fb, ik_map_left(ik), **T17_COMMAND)   # 已收敛 → hold
        self.assertFalse(p4.updated)

    def test_command_mode_converged_stops_despite_fb_off(self):
        """command 模式收敛判据与反馈无关：fb 差 0.035 也不再发（hunt 免疫核心）。"""
        p = make_planner()
        fb = [0.0] * eb.N_JOINTS
        ik = [0.06, 0, 0, 0, 0, 0, 0]
        p.plan(fb, ik_map_left(ik), **T17_COMMAND)
        fb[eb.IDX_LEFT_START] = 0.06 + 0.035   # 真机过冲 2°
        p2 = p.plan(fb, ik_map_left(ik), **T17_COMMAND)
        self.assertFalse(p2.updated)           # 不再纠偏 → 无 hunt 燃料

    def test_hold_rearm_and_ik_moved(self):
        p = make_planner()
        fb = [0.0] * eb.N_JOINTS
        p.plan(fb, ik_map_left([0.06, 0, 0, 0, 0, 0, 0]), **T14_DEFAULTS)
        fb[eb.IDX_LEFT_START] = 0.06 + 0.005   # 0.005 ≤ enter → 进入 hold
        self.assertFalse(p.plan(fb, ik_map_left([0.06, 0, 0, 0, 0, 0, 0]),
                                **T14_DEFAULTS).updated)
        # 目标动了 → 清锁恢复跟随
        plan = p.plan(fb, ik_map_left([0.10, 0, 0, 0, 0, 0, 0]),
                      **dict(T14_DEFAULTS, ik_moved=True))
        self.assertTrue(plan.updated)

    def test_leash_hold_and_release(self):
        """左臂超 leash：该臂 7 轴冻结（per-arm，非整帧），回带内自动释放。"""
        p = make_planner()
        fb = [0.0] * eb.N_JOINTS
        p.plan(fb, ik_map_left([0.3, 0, 0, 0, 0, 0, 0]), **T17_COMMAND)
        cmd1 = p._command[eb.IDX_LEFT_START]      # 注意 max_step 会把 0.3 截到 0.2
        fb[eb.IDX_LEFT_START] = cmd1 + 0.7        # 偏离计划 0.7 > leash 0.6
        plan = p.plan(fb, ik_map_left([0.3, 0, 0, 0, 0, 0, 0]), **T17_COMMAND)
        self.assertFalse(plan.leash_hold)         # 仅左臂超限 → 非整帧语义
        self.assertEqual(plan.held, 7)            # 左臂 7 轴全部冻结
        self.assertFalse(plan.updated)
        fb[eb.IDX_LEFT_START] = cmd1 + 0.35       # 回到 0.8×leash 内 → 释放
        plan = p.plan(fb, ik_map_left([0.3, 0, 0, 0, 0, 0, 0]), **T17_COMMAND)
        self.assertFalse(plan.leash_hold)
        self.assertTrue(plan.updated)             # Joint1L 恢复步进
        self.assertEqual(plan.held, 6)            # Joint2~7 已收敛，按迟滞正常进 hold

    def test_leash_per_arm_isolation(self):
        """左臂超限冻结时右臂照常跟随（UBT ArmSafetyGate×2 隔离语义）。"""
        p = make_planner()
        fb = [0.0] * eb.N_JOINTS
        ik = {n: v for n, v in zip(eb.LEFT_URDF_JOINTS, [0.3, 0, 0, 0, 0, 0, 0])}
        ik.update({n: v for n, v in zip(eb.RIGHT_URDF_JOINTS, [0.1, 0, 0, 0, 0, 0, 0])})
        p.plan(fb, ik, **T17_COMMAND)
        cmd_l = p._command[eb.IDX_LEFT_START]
        cmd_r = p._command[eb.IDX_RIGHT_START]
        # 左臂被拖远（超限），右臂正常移动到新目标
        fb[eb.IDX_LEFT_START] = cmd_l + 0.7
        fb[eb.IDX_RIGHT_START] = cmd_r            # 右臂已到位
        ik2 = dict(ik)
        ik2[eb.RIGHT_URDF_JOINTS[0]] = 0.2
        plan = p.plan(fb, ik2, **T17_COMMAND)
        self.assertTrue(plan.updated)             # 右臂正常更新
        self.assertEqual(plan.joints[eb.IDX_LEFT_START], cmd_l)  # 左臂冻结在计划值
        self.assertAlmostEqual(plan.joints[eb.IDX_RIGHT_START],
                               cmd_r + min(0.1, 0.20), places=12)

    def test_invalidate_rebases(self):
        """指令被拒后 invalidate：下一帧重新以 fb 估（有限重试，防持续 hunt）。"""
        p = make_planner()
        fb = [0.0] * eb.N_JOINTS
        ik = [0.06, 0, 0, 0, 0, 0, 0]
        p.plan(fb, ik_map_left(ik), **T17_COMMAND)
        fb[eb.IDX_LEFT_START] = 0.06 + 0.035
        self.assertFalse(p.plan(fb, ik_map_left(ik), **T17_COMMAND).updated)
        p.invalidate()
        plan = p.plan(fb, ik_map_left(ik), **T17_COMMAND)
        self.assertTrue(plan.updated)          # 重估后再次尝试
        self.assertAlmostEqual(plan.joints[eb.IDX_LEFT_START], 0.06 + 0.035 - 0.035,
                               places=12)      # fb+err → 重新指向 IK 目标


class TestClosedLoopHunt(unittest.TestCase):
    """L2：闭环仿真复现 hunt（机制③）并验证修复。"""

    def test_a0_old_behaviour_hunts_with_overshoot(self):
        """复现：feedback 底 + 旧 eps 行为 + 2° 过冲 → 每帧都在发（永不收敛）。"""
        robot = SimRobot('overshoot', 0.035)
        p = make_planner()
        ik = [0.06, 0, 0, 0, 0, 0, 0]
        for _ in range(50):
            robot.tick(p, ik, **OLD_BEHAVIOUR)
        self.assertGreaterEqual(robot.sends, 45)   # 50 拍几乎全在发
        amp = abs(robot.fb[eb.IDX_LEFT_START] - ik[0])
        self.assertAlmostEqual(amp, 0.035, places=9)  # 真机在 ±2° 摆，与 T12 现象同构

    def test_a1_hysteresis_insufficient_for_big_overshoot(self):
        """T14 迟滞边界：过冲(0.035) ≥ hold_enter(0.01) 时 err 永远带外 → 仍 hunt。"""
        robot = SimRobot('overshoot', 0.035)
        p = make_planner()
        ik = [0.06, 0, 0, 0, 0, 0, 0]
        for _ in range(50):
            robot.tick(p, ik, **T14_DEFAULTS)
        self.assertGreaterEqual(robot.sends, 45)   # 文档化：迟滞不是万能，根治靠 command 底

    def test_a2_hysteresis_works_for_small_overshoot(self):
        """过冲(0.005) < hold_enter(0.01)：1 拍后进入 hold，停发。"""
        robot = SimRobot('overshoot', 0.005)
        p = make_planner()
        ik = [0.06, 0, 0, 0, 0, 0, 0]
        for _ in range(20):
            robot.tick(p, ik, **T14_DEFAULTS)
        self.assertLessEqual(robot.sends, 2)

    def test_a3_command_mode_immune_to_big_overshoot(self):
        """T17：command 底 + 2° 过冲 → 收敛即停发，无 hunt 燃料。"""
        robot = SimRobot('overshoot', 0.035)
        p = make_planner()
        ik = [0.06, 0, 0, 0, 0, 0, 0]
        for _ in range(50):
            robot.tick(p, ik, **T17_COMMAND)
        self.assertLessEqual(robot.sends, 2)

    def test_b_sag_model_feedback_hunts_command_stops(self):
        """UBT 下垂模型：feedback 底每帧都在托（追着下垂走）；command 底一次到位即停。"""
        ik = [0.06, 0, 0, 0, 0, 0, 0]
        robot_fb = SimRobot('sag', 0.02)
        p_fb = make_planner()
        for _ in range(50):
            robot_fb.tick(p_fb, ik, **OLD_BEHAVIOUR)
        self.assertGreaterEqual(robot_fb.sends, 45)  # 「命令追着下垂走」

        robot_cmd = SimRobot('sag', 0.02)
        p_cmd = make_planner()
        for _ in range(50):
            robot_cmd.tick(p_cmd, ik, **T17_COMMAND)
        self.assertLessEqual(robot_cmd.sends, 2)     # 冻结值语义，不再追

    def test_c_following_not_blocked_by_hold(self):
        """迟滞不卡跟随：目标持续变化（ik_moved）时每拍都发。"""
        robot = SimRobot('exact')
        p = make_planner()
        sends = 0
        for k in range(20):
            ik = [0.06 + 0.01 * k, 0, 0, 0, 0, 0, 0]
            plan = robot.tick(p, ik, **dict(T14_DEFAULTS, ik_moved=True))
            sends += int(plan.updated)
        self.assertEqual(sends, 20)


class TestAsyncTcpClient(unittest.TestCase):
    """L3：真实 socket。lock-step 节拍 / R4 重连 / R3 反馈年龄。"""

    def setUp(self):
        self.servers = []
        self.clients = []

    def tearDown(self):
        for c in self.clients:
            c.close()
        for s in self.servers:
            s.stop()

    def _start(self, **kw):
        srv = FakeController(**kw)
        srv.thread = __import__('threading').Thread(target=srv.serve_forever,
                                                    kwargs={'poll_interval': 0.05},
                                                    daemon=True)
        srv.thread.start()
        self.servers.append(srv)
        return srv

    def test_sync_and_feedback_age(self):
        srv = self._start()
        c = eb.AsyncTcpClient('127.0.0.1', srv.port, query_hz=50)
        self.clients.append(c)
        c.connect()
        self.assertIsNone(c.get_latest_joint_rad())      # R4：连接后无旧快照
        t0 = time.monotonic()
        while c.get_latest_joint_rad() is None and time.monotonic() - t0 < 2:
            time.sleep(0.01)
        self.assertIsNotNone(c.get_latest_joint_rad())
        age = c.get_feedback_age()
        self.assertIsNotNone(age)
        self.assertLess(age, 0.5)

    def test_lockstep_accept_mode_high_rate(self):
        """受理即回（R2 假设 A）：10 条目标全部发出。"""
        srv = self._start(ack_mode='accept', exec_time_s=0.0)
        c = eb.AsyncTcpClient('127.0.0.1', srv.port, query_hz=50)
        self.clients.append(c)
        c.connect()
        t0 = time.monotonic()
        while c.get_latest_joint_rad() is None and time.monotonic() - t0 < 2:
            time.sleep(0.01)
        for k in range(10):
            j = [0.0] * eb.N_JOINTS
            j[eb.IDX_LEFT_START] = 0.01 * (k + 1)
            c.set_motion_target({'command': 'movej', 'joint': j, 'v': 15})
            time.sleep(0.05)
        t1 = time.monotonic()
        while srv.movej_accepted < 10 and time.monotonic() - t1 < 3:
            time.sleep(0.02)
        self.assertEqual(srv.movej_accepted, 10)
        self.assertEqual(srv.movej_rejected, 0)

    def test_lockstep_complete_mode_paced_and_rejects_overlap(self):
        """执行完才回（R2 假设 B）+ 执行 0.5s：300ms 超时解锁 → 下一条被拒。
        刻画真机「受理/完成」两种语义下的行为差异，供 T12 现场判别对照。"""
        srv = self._start(ack_mode='complete', exec_time_s=0.5)
        c = eb.AsyncTcpClient('127.0.0.1', srv.port, query_hz=50)
        self.clients.append(c)
        c.connect()
        t0 = time.monotonic()
        while c.get_latest_joint_rad() is None and time.monotonic() - t0 < 2:
            time.sleep(0.01)
        for k in range(5):
            j = [0.0] * eb.N_JOINTS
            j[eb.IDX_LEFT_START] = 0.05 * (k + 1)
            c.set_motion_target({'command': 'movej', 'joint': j, 'v': 15})
            time.sleep(0.02)
        time.sleep(2.2)  # 让 lock-step/超时/拒绝充分展开
        sent, ack, timeout_cnt = c.get_movej_lock_stats()
        self.assertGreaterEqual(timeout_cnt, 1)      # 300ms 超时兜底真的触发
        self.assertGreaterEqual(srv.movej_rejected, 1)  # 程序重叠拒绝发生
        self.assertLessEqual(srv.movej_accepted, 5)
        self.assertGreaterEqual(ack, 1)

    def test_reconnect_drops_stale_snapshot(self):
        """R4 回归：重连后旧快照必须清空，同步等新应答。"""
        srv1 = self._start()
        c = eb.AsyncTcpClient('127.0.0.1', srv1.port, query_hz=50)
        self.clients.append(c)
        c.connect()
        t0 = time.monotonic()
        while c.get_latest_joint_rad() is None and time.monotonic() - t0 < 2:
            time.sleep(0.01)
        self.assertIsNotNone(c.get_latest_joint_rad())
        # 模拟断线重连：同端口起新服务端（真机断线重连就是同一地址端口）
        port = srv1.port
        srv1.stop()
        time.sleep(0.3)
        srv2 = FakeController(port=port)
        srv2.thread = __import__('threading').Thread(
            target=srv2.serve_forever, kwargs={'poll_interval': 0.05}, daemon=True)
        srv2.thread.start()
        self.servers.append(srv2)
        srv2.set_arm_state([0.1] * 7, [0.2] * 7)
        c.close()
        c.connect()
        # connect() 里应已清空旧快照
        self.assertIsNone(c.get_latest_joint_rad())
        t1 = time.monotonic()
        while c.get_latest_joint_rad() is None and time.monotonic() - t1 < 2:
            time.sleep(0.01)
        joints = c.get_latest_joint_rad()
        self.assertIsNotNone(joints)
        self.assertAlmostEqual(joints[eb.IDX_LEFT_START], 0.1, places=6)  # 新会话新值


if __name__ == '__main__':
    unittest.main(verbosity=2)
