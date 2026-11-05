#!/usr/bin/env python3
from collections import deque

import rclpy
from rclpy.node import Node
import numpy as np
import pinocchio as pin
import pink
from pink import solve_ik
from pink.tasks import FrameTask, PostureTask, DampingTask
from pink.limits import VelocityLimit, ConfigurationLimit
import tf2_ros
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Bool
import math
import time
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

# ==================================================================================
#  One Euro Filter Implementation (1€ 滤波器)
# ==================================================================================
class LowPassFilter:
    def __init__(self, alpha):
        self.__setAlpha(alpha)
        self.y = None
        self.s = None

    def __setAlpha(self, alpha):
        alpha = float(alpha)
        if alpha <= 0 or alpha > 1.0:
            raise ValueError("alpha (%s) should be in (0.0, 1.0]" % alpha)
        self.alpha = alpha

    def filter(self, value, timestamp=None, alpha=None):
        if alpha is not None:
            self.__setAlpha(alpha)
        if self.y is None:
            s = value
        else:
            s = self.alpha * value + (1.0 - self.alpha) * self.s
        self.y = value
        self.s = s
        return s

    def lastValue(self):
        return self.y

    def reset(self):
        self.y = None
        self.s = None

class OneEuroFilter:
    def __init__(self, freq, mincutoff=1.0, beta=0.0, dcutoff=1.0):
        if freq <= 0:
            raise ValueError("freq should be >0")
        if mincutoff <= 0:
            raise ValueError("mincutoff should be >0")
        if dcutoff <= 0:
            raise ValueError("dcutoff should be >0")
        self.__freq = float(freq)
        self.__mincutoff = float(mincutoff)
        self.__beta = float(beta)
        self.__dcutoff = float(dcutoff)
        self.__x = LowPassFilter(self.__alpha(self.__mincutoff))
        self.__dx = LowPassFilter(self.__alpha(self.__dcutoff))
        self.__last_time = None

    def reset(self):
        """断档后调用：状态陈旧会导致首帧以~6%/帧爬向新值（按下瞬跳根源）。"""
        self.__x.reset()
        self.__dx.reset()
        self.__last_time = None

    def __alpha(self, cutoff):
        te = 1.0 / self.__freq
        tau = 1.0 / (2 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / te)

    def filter(self, x, timestamp=None):
        if self.__last_time and timestamp:
            rate = 1.0 / (timestamp - self.__last_time)
        else:
            rate = self.__freq
        self.__last_time = timestamp

        if self.__x.lastValue() is None:
            dx = 0.0 * x 
        else:
            dx = (x - self.__x.lastValue()) * rate
            
        edx = self.__dx.filter(dx, timestamp, alpha=self.__alpha(self.__dcutoff))
        cutoff = self.__mincutoff + self.__beta * np.linalg.norm(edx)
        return self.__x.filter(x, timestamp, alpha=self.__alpha(cutoff))

class PoseFilter:
    def __init__(self, freq=500.0, mincutoff=1.0, beta=0.05, dcutoff=1.0):
        self.pos_filter = OneEuroFilter(freq, mincutoff, beta, dcutoff)
        self.quat_filter = OneEuroFilter(freq, mincutoff, beta, dcutoff)

    def reset(self):
        self.pos_filter.reset()
        self.quat_filter.reset()

    def process(self, pos, quat, timestamp=None):
        filtered_pos = self.pos_filter.filter(pos, timestamp)
        filtered_quat_raw = self.quat_filter.filter(quat, timestamp)
        norm = np.linalg.norm(filtered_quat_raw)
        if norm > 1e-6:
            filtered_quat = filtered_quat_raw / norm
        else:
            filtered_quat = quat 
        return filtered_pos, filtered_quat

# ==================================================================================

class UniversalHumanIK(Node):
    def __init__(self):
        super().__init__('ik_retargeting')

        # 1. 参数声明 (兼容旧 Launch 文件)
        self.declare_parameter('urdf_path', '')
        self.declare_parameter('root_link_name', 'base_link') 
        self.declare_parameter('ref_link_name', '') # 默认为空，自动回退
        self.declare_parameter('target_link_names', ['right_wrist_roll_link', 'left_wrist_roll_link'])
        
        string_array_desc = ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
        double_array_desc = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
        self.declare_parameter('preferred_joints', value=None, descriptor=string_array_desc)
        self.declare_parameter('preferred_angles', value=None, descriptor=double_array_desc)

        # 2. 参数获取
        urdf_path = self.get_parameter('urdf_path').value
        self.root_link = self.get_parameter('root_link_name').value
        
        # 智能参考系处理
        user_ref = self.get_parameter('ref_link_name').value
        if not user_ref:
            self.ref_link = self.root_link
        else:
            self.ref_link = user_ref

        target_links = self.get_parameter('target_link_names').value
        self.ee_frames = {'right': target_links[0], 'left': target_links[1]}
        
        pref_joints = self.get_parameter('preferred_joints').value
        pref_angles = self.get_parameter('preferred_angles').value
        if pref_joints is None: pref_joints = []
        if pref_angles is None: pref_angles = []

        self.ik_freq = 250.0
        self.dt = 1.0 / self.ik_freq

        # 3. 滤波器：略加快响应，减少姿态滞后
        input_freq = 500.0
        self.filters = {
            'left': PoseFilter(freq=input_freq, mincutoff=5.0, beta=0.05),
            'right': PoseFilter(freq=input_freq, mincutoff=5.0, beta=0.05),
        }
        
        # 4. 模型初始化
        base_model = pin.buildModelFromUrdf(urdf_path)
        
        # 检查参考系
        if base_model.existFrame(self.ref_link):
            self.get_logger().info(f"Reference Frame '{self.ref_link}' active.")
        else:
            self.get_logger().warn(f"Reference Frame '{self.ref_link}' NOT FOUND! Fallback to '{self.root_link}'.")
            self.ref_link = self.root_link

        self.models = {'left': base_model.copy(), 'right': base_model.copy()}
        self.datas = {side: m.createData() for side, m in self.models.items()}
        self.joint_names = [str(n) for n in base_model.names[1:]]

        self.configs = {
            side: pink.Configuration(self.models[side], self.datas[side], pin.neutral(self.models[side])) 
            for side in ['left', 'right']
        }

        # 5. 拟人姿态构建
        self.q_natural = {'left': pin.neutral(base_model), 'right': pin.neutral(base_model)}
        for side in ['left', 'right']:
            if len(pref_joints) > 0:
                for j_name, angle in zip(pref_joints, pref_angles):
                    if self.models[side].existJointName(j_name):
                        jid = self.models[side].getJointId(j_name)
                        idx_q = self.models[side].joints[jid].idx_q
                        self.q_natural[side][idx_q] = angle

        # 6. 任务栈：位置+姿态都跟；增量遥操在 control_loop
        self.declare_parameter('ee_position_cost', 300.0)
        self.declare_parameter('ee_orientation_cost', 40.0)
        self.declare_parameter('mirror_xyz', [1.0, 1.0, 1.0])
        self.declare_parameter('mirror_xyz_left', [-1.0, -1.0, -1.0])
        self.declare_parameter('mirror_xyz_right', [1.0, 1.0, 1.0])
        self.declare_parameter('publish_commanded_ee_state', True)
        # 仅抑制噪声，勿用大切换死区（会卡顿）
        self.declare_parameter('position_deadband_m', 0.002)
        self.declare_parameter('orientation_deadband_rad', 0.02)
        pos_cost = float(self.get_parameter('ee_position_cost').value)
        orn_cost = float(self.get_parameter('ee_orientation_cost').value)
        self.mirror_xyz = {
            'left': np.array(
                [float(x) for x in self.get_parameter('mirror_xyz_left').value], dtype=float
            ),
            'right': np.array(
                [float(x) for x in self.get_parameter('mirror_xyz_right').value], dtype=float
            ),
        }
        self.publish_commanded_ee_state = bool(
            self.get_parameter('publish_commanded_ee_state').value
        )
        self.position_deadband_m = float(self.get_parameter('position_deadband_m').value)
        self.orientation_deadband_rad = float(
            self.get_parameter('orientation_deadband_rad').value
        )

        self.tasks = {}
        for side in ['left', 'right']:
            self.tasks[side] = {
                "ee": FrameTask(
                    self.ee_frames[side],
                    position_cost=pos_cost,
                    orientation_cost=orn_cost,
                ),
                "posture": PostureTask(cost=0.3),
                "damping": DampingTask(cost=12.0),
            }
            self.tasks[side]["posture"].set_target(self.q_natural[side])

        self.limits = [ConfigurationLimit(base_model), VelocityLimit(base_model)]

        # 7. 状态：按下 Clutch 记锚点 → 之后只跟相对增量（位姿）
        self.q_measured = pin.neutral(base_model)
        self.q_commanded = None

        self.is_initialized = False
        self.clutch = {'left': False, 'right': False}
        self.prev_clutch = {'left': False, 'right': False}
        self.last_velocity = np.zeros(base_model.nv)

        self.latest_target_in_ref = {'left': None, 'right': None}
        self.last_target_time = {'left': None, 'right': None}  # 目标流新鲜度（陈旧锚点防护）
        self.await_anchor = {'left': False, 'right': False}  # 断档后等第一帧新鲜目标再锚定
        # 目标流安定检测：Puppetry 起流后 bias 渐进生效（手静时目标姿态仍~23°/s自转），
        # 立即锚定会被安定过程拖着走（实测J7被拉68°/2s）→ 等流安定再锚
        self.await_hist = {'left': deque(), 'right': deque()}  # (t, SE3)
        self.await_t0 = {'left': None, 'right': None}  # 首帧新鲜目标时刻（等待计时基准）
        self.anchor_window_s = 0.30    # 安定判定窗口
        self.anchor_min_wait_s = 0.50  # 最短等待（自首帧起）
        self.anchor_max_wait_s = 1.50  # 超时强制锚定（自首帧起）
        self.anchor_settle_dp = 0.010  # 位置安定阈值 m/s
        self.anchor_settle_da = 0.087  # 姿态安定阈值 rad/s (≈5°/s)
        self.robot_anchor_in_ref = {'left': pin.SE3.Identity(), 'right': pin.SE3.Identity()}
        self.exo_anchor_in_ref = {'left': pin.SE3.Identity(), 'right': pin.SE3.Identity()}
        self.last_desired_in_ref = {'left': None, 'right': None}

        self.get_logger().info(
            f'IK INCREMENTAL teleop (pos+orn) | pos_cost={pos_cost} orn_cost={orn_cost} '
            f'mirror L={self.mirror_xyz["left"].tolist()} R={self.mirror_xyz["right"].tolist()} '
            f'noise_db={self.position_deadband_m}m'
        )

        # 8. 通信
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(JointState, "/client_robot/joint_states", self.state_cb, 1)
        self.create_subscription(PoseStamped, "/target_robot/left_ee/target", lambda m: self.target_cb(m, 'left'), 1)
        self.create_subscription(PoseStamped, "/target_robot/right_ee/target", lambda m: self.target_cb(m, 'right'), 1)
        self.create_subscription(Bool, "/clutch/left", lambda m: self.clutch_cb(m, 'left'), 1)
        self.create_subscription(Bool, "/clutch/right", lambda m: self.clutch_cb(m, 'right'), 1)
        self.pub_cmd = self.create_publisher(JointState, "/target_robot/joint_commands", 1)
        self.pub_ee_state = {
            'left': self.create_publisher(PoseStamped, '/target_robot/left_ee/state', 10),
            'right': self.create_publisher(PoseStamped, '/target_robot/right_ee/state', 10),
        }

        self.timer = self.create_timer(self.dt, self.control_loop)

    def state_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            if self.models['right'].existJointName(name):
                idx = self.models['right'].joints[self.models['right'].getJointId(name)].idx_q
                self.q_measured[idx] = pos
        self.is_initialized = True

    def clutch_cb(self, msg, side): self.clutch[side] = msg.data

    def _side_joint_indices(self, side):
        """Return q-vector indices for one arm.
        - Names containing left/right (e.g. G1)
        - tc_urdf: left = jointL*, right = joint1..joint7 (no L)
        - hr_up_7dof_fixed: left = Joint1L..Joint7L（L 后缀）, right = Joint1..Joint7
          大小写不敏感匹配，兼容两种命名风格
        """
        lower = [n.lower() for n in self.joint_names]
        if side == 'left':
            return [
                i for i, n in enumerate(lower)
                if 'left' in n
                or n.startswith('jointl')
                or (n.startswith('joint') and n.endswith('l'))
            ]
        idxs = [i for i, n in enumerate(lower) if 'right' in n]
        if idxs:
            return idxs
        left = set(self._side_joint_indices('left'))
        return [
            i for i, n in enumerate(lower)
            if n.startswith('joint') and i not in left
        ]

    def target_cb(self, msg, side):
        raw_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        raw_quat = np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ])

        # 目标流断档（Puppetry 松开即停发）后滤波器状态陈旧：
        # 冻结在旧值并以~6%/帧爬向新值 → 按下后机器人被拖着走（实测76mm/1200mrad）。
        # 断档>0.2s 就重置，让第一帧新目标直接穿过。
        now_s = self.get_clock().now().nanoseconds * 1e-9
        t_last = self.last_target_time[side]
        if t_last is not None and now_s - t_last > 0.2:
            self.filters[side].reset()

        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        filt_pos, filt_quat = self.filters[side].process(raw_pos, raw_quat, timestamp)

        q_obj = pin.Quaternion(filt_quat[3], filt_quat[0], filt_quat[1], filt_quat[2])
        self.latest_target_in_ref[side] = pin.SE3(q_obj.toRotationMatrix(), filt_pos)
        self.last_target_time[side] = self.get_clock().now().nanoseconds * 1e-9

    def _incremental_desired(self, side, T_exo, T_robot0, T_exo0):
        """基座系增量：按下后外骨骼（Puppetry目标）怎么动，机器人末端跟同样相对位姿变化。
        位置可按臂 mirror；姿态用目标相对旋转叠到按下时机器人姿态上。
        """
        mirror = self.mirror_xyz[side]
        dp = (np.array(T_exo.translation) - np.array(T_exo0.translation)) * mirror
        p_des = np.array(T_robot0.translation) + dp
        R_delta = T_exo.rotation @ T_exo0.rotation.T
        R_des = R_delta @ T_robot0.rotation
        return pin.SE3(R_des, p_des)

    def control_loop(self):
        if not self.is_initialized:
            return

        if self.q_commanded is None:
            self.q_commanded = self.q_measured.copy()

        now_s = self.get_clock().now().nanoseconds * 1e-9
        for side in ['left', 'right']:
            if self.clutch[side] and not self.prev_clutch[side]:
                self.q_commanded = self.q_measured.copy()
                self.last_velocity.fill(0.0)
                # 姿态任务锚到当前关节角：否则按住期间被持续拉向全零位
                # （preferred_angles=0），手完全不动关节也会漂（实测Joint7L~150mrad/0.5s）
                for s in ['left', 'right']:
                    self.tasks[s]["posture"].set_target(self.q_commanded.copy())
                # 每次按下都走安定等待：Puppetry 起流后 bias 渐进生效（手静时目标
                # 姿态仍~23°/s自转~1s），必须等安定再锚定；断档>0.2s 的旧目标先作废，
                # 等第一帧新鲜目标（外骨骼锚=目标流自身，位置+姿态同源，避免与真机
                # 姿态失配被一次性套上——实测 J5 瞬跳 1000+mrad）。
                t_last = self.last_target_time[side]
                if t_last is None or now_s - t_last > 0.2:
                    self.latest_target_in_ref[side] = None
                self.await_anchor[side] = True
                self.await_t0[side] = None
                self.await_hist[side].clear()

        clutch_any = any(self.clutch.values())
        q_ik = self.q_commanded if clutch_any else self.q_measured
        combined_velocity = np.zeros(self.models['right'].nv)

        for side in ['left', 'right']:
            self.configs[side].update(q_ik)

            T_base_ee = self.configs[side].get_transform_frame_to_world(
                self.tasks[side]["ee"].frame
            )
            ref_id = self.models[side].getFrameId(self.ref_link)
            if ref_id >= len(self.datas[side].oMf):
                ref_id = 0
            T_base_ref = self.datas[side].oMf[ref_id]
            T_current_in_ref = T_base_ref.inverse() * T_base_ee

            if self.clutch[side]:
                T_exo = self.latest_target_in_ref[side]

                if self.await_anchor[side]:
                    if T_exo is None:
                        # 目标流尚未恢复：本拍冻结（不求解，速度贡献为0）
                        self.broadcast_tf(T_current_in_ref, f"ik_target_{side}", self.ref_link)
                        self.prev_clutch[side] = self.clutch[side]
                        continue
                    # 等目标流安定（Puppetry 起流 bias 渐进生效），期间冻结。
                    # 计时基准=首帧新鲜目标时刻 await_t0（不能 hist[0]——窗口裁剪
                    # 会把 elapsed 永远压在 window_s 内，导致永不锚定=按下冻结）
                    if self.await_t0[side] is None:
                        self.await_t0[side] = now_s
                        self.await_hist[side].clear()
                    h = self.await_hist[side]
                    h.append((now_s, T_exo))
                    while h and now_s - h[0][0] > self.anchor_window_s:
                        h.popleft()
                    since_start = now_s - self.await_t0[side]
                    settled = False
                    if since_start >= self.anchor_min_wait_s and len(h) >= 2:
                        win = max(now_s - h[0][0], 1e-3)
                        dp = float(np.linalg.norm(
                            np.array(T_exo.translation) - np.array(h[0][1].translation)))
                        da = float(np.linalg.norm(
                            pin.log3(h[0][1].rotation.T @ T_exo.rotation)))
                        settled = (dp / win < self.anchor_settle_dp
                                   and da / win < self.anchor_settle_da)
                    if settled or since_start > self.anchor_max_wait_s:
                        # 安定（或超时）：外骨骼锚=当前目标（位置+姿态与后续同源），
                        # 机器人锚=当前真机位姿
                        self.await_anchor[side] = False
                        self.await_t0[side] = None
                        h.clear()
                        self.robot_anchor_in_ref[side] = T_current_in_ref.copy()
                        self.exo_anchor_in_ref[side] = T_exo.copy()
                        self.last_desired_in_ref[side] = T_current_in_ref.copy()
                    else:
                        self.broadcast_tf(T_current_in_ref, f"ik_target_{side}", self.ref_link)
                        self.prev_clutch[side] = self.clutch[side]
                        continue
                elif not self.prev_clutch[side]:
                    # 目标流在按下时就是新鲜的：立即锚定
                    self.robot_anchor_in_ref[side] = T_current_in_ref.copy()
                    self.exo_anchor_in_ref[side] = T_exo.copy()
                    self.last_desired_in_ref[side] = T_current_in_ref.copy()

                T_desired = self._incremental_desired(
                    side,
                    T_exo,
                    self.robot_anchor_in_ref[side],
                    self.exo_anchor_in_ref[side],
                )

                # 仅噪声抑制：变化极小时保持上一目标，绝不弹回按下点（避免卡顿）
                prev = self.last_desired_in_ref[side]
                dp_n = float(np.linalg.norm(T_desired.translation - prev.translation))
                R_err = prev.rotation.T @ T_desired.rotation
                ang = float(np.linalg.norm(pin.log3(R_err)))
                if (
                    dp_n < self.position_deadband_m
                    and ang < self.orientation_deadband_rad
                ):
                    T_display_in_ref = prev
                else:
                    T_display_in_ref = T_desired
                    self.last_desired_in_ref[side] = T_desired.copy()

                self.tasks[side]["damping"].cost = 12.0
                self.tasks[side]["posture"].cost = 0.3
                self.tasks[side]["ee"].set_target(T_base_ref * T_display_in_ref)

                v_side = solve_ik(
                    self.configs[side],
                    self.tasks[side].values(),
                    self.dt,
                    solver="osqp",
                    safety_break=False,
                )
                for idx in self._side_joint_indices(side):
                    combined_velocity[idx] = v_side[idx]
            else:
                T_display_in_ref = T_current_in_ref
                self.last_desired_in_ref[side] = None

            self.broadcast_tf(T_display_in_ref, f"ik_target_{side}", self.ref_link)
            self.prev_clutch[side] = self.clutch[side]

        filter_alpha = 0.85
        filtered_velocity = (
            filter_alpha * combined_velocity + (1.0 - filter_alpha) * self.last_velocity
        )
        self.last_velocity = filtered_velocity

        if clutch_any:
            self.q_commanded = pin.integrate(
                self.models['right'], self.q_commanded, filtered_velocity * self.dt
            )
            q_next = self.q_commanded
        else:
            self.q_commanded = self.q_measured.copy()
            q_next = self.q_commanded

        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name = self.joint_names
        cmd.position = q_next.tolist()
        self.pub_cmd.publish(cmd)

        if self.publish_commanded_ee_state and clutch_any:
            self._publish_ee_states(self.q_commanded)

    def _publish_ee_states(self, q):
        for side in ['left', 'right']:
            self.configs[side].update(q)
            T_base_ee = self.configs[side].get_transform_frame_to_world(
                self.tasks[side]["ee"].frame
            )
            ref_id = self.models[side].getFrameId(self.ref_link)
            if ref_id >= len(self.datas[side].oMf):
                ref_id = 0
            T_base_ref = self.datas[side].oMf[ref_id]
            T_in_ref = T_base_ref.inverse() * T_base_ee

            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.ref_link
            msg.pose.position.x = float(T_in_ref.translation[0])
            msg.pose.position.y = float(T_in_ref.translation[1])
            msg.pose.position.z = float(T_in_ref.translation[2])
            quat = pin.Quaternion(T_in_ref.rotation)
            msg.pose.orientation.x = float(quat.x)
            msg.pose.orientation.y = float(quat.y)
            msg.pose.orientation.z = float(quat.z)
            msg.pose.orientation.w = float(quat.w)
            self.pub_ee_state[side].publish(msg)

    def broadcast_tf(self, T, child_frame, parent_frame):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = T.translation
        q = pin.Quaternion(T.rotation)
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q.x, q.y, q.z, q.w
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    rclpy.spin(UniversalHumanIK())
    rclpy.shutdown()

if __name__ == "__main__": main()