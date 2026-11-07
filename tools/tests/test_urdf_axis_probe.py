"""Offline tests for T33. No controller connections or Pinocchio required."""

from contextlib import redirect_stderr, redirect_stdout
from dataclasses import replace
import hashlib
import importlib.util
import io
import json
import math
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import Mock, patch


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools/dm-exo/urdf_fk_calib"))
import urdf_axis_probe as probe


URDF = next((ROOT / "ros2_ws/src/hr_up_7dof_fixed/urdf").glob("*.urdf"))
LIMITS = probe.load_urdf(URDF.read_bytes(), probe.SIDES)


def plan(side="left"):
    return probe.Plan.from_json(json.dumps({
        "host": "controller.invalid", "port": 30485, "side": side, "joint": 2,
        "baseline": [0] * 17, "delta_rad": 0.05, "speed_pct": 1,
        "limit_margin_rad": 0.01, "urdf_limit_tolerance_rad": 0.03,
        "target_tolerance_rad": 0.005, "hold_tolerance_rad": 0.003,
        "settle_tolerance_rad": 0.001, "settle_s": 0.06, "point_timeout_s": 0.5,
        "rpc_timeout_s": 0.1, "max_pair_s": 0.1, "min_tcp_rotation_rad": 0.01,
        "base_frame": "base_link", "tcp_frame": probe.SIDES[side][2],
        "tcp_position_unit": "mm",
    }))


class Clock:
    now = 100.0

    def advance(self, seconds):
        self.now += seconds


class FakeController:
    def __init__(self, clock, config, fault=None):
        self.clock = clock
        self.plan = config
        self.fault = fault
        self.commands = []
        self.q = list(config.baseline)
        self.moves = 0
        self.closed = False

    def request(self, command, deadline=None, **payload):
        sent = self.clock.now
        self.clock.advance(0.002)
        if deadline is not None and self.clock.now >= deadline:
            raise TimeoutError("deadline expired")
        self.commands.append(command)
        raw = {"command": command, "status": probe.SUCCESS}
        if command == "get_device_state":
            raw.update(state="idle", erroeCode=0, hardwareErrors=[],
                       softwareErrors=[123] if self.fault == "fault" else [],
                       motorErrorDetails=None)
        elif command == "get_config":
            joints = [name for side in probe.SIDES for name in probe.arm_joints(side)]
            raw["lowerLimits"] = [LIMITS[name][0] for name in joints]
            raw["upperLimits"] = [LIMITS[name][1] for name in joints]
            if self.fault == "limit":
                raw["lowerLimits"].pop()
        elif command == "get_joint_position":
            raw["joint"] = self.q.copy()
            if self.fault == "bad_q":
                raw["joint"][0] = probe.HOLD
            if self.fault == "unselected" and self.moves:
                raw["joint"][0] = 0.04
        elif command == "get_tcp_pose":
            angle = self.q[self.plan.index] if self.fault != "frozen_tcp" else 0.0
            rotated = [math.cos(angle / 2), math.sin(angle / 2), 0, 0]
            left = rotated if self.plan.side == "left" else [1, 0, 0, 0]
            right = rotated if self.plan.side == "right" else [1, 0, 0, 0]
            raw["pose"] = [100, 0, 0, *left, 200, 0, 0, *right]
            if self.fault == "bad_tcp":
                raw["pose"][3] = 0
        elif command == "movej":
            self.moves += 1
            assert len(payload["joint"]) == 17
            assert all(value == probe.HOLD for i, value in enumerate(payload["joint"])
                       if i != self.plan.index)
            if self.fault == "disconnect":
                raise ConnectionError("transport disconnected")
            if self.fault != "frozen" and not (self.fault == "return" and self.moves == 2):
                self.q[self.plan.index] = payload["joint"][self.plan.index]
        elif command == "stop_motion" and self.fault == "disconnect":
            raise ConnectionError("stop unavailable")
        return probe.Reply(raw, sent, self.clock.now)

    def close(self):
        self.closed = True


class SamplingTests(unittest.TestCase):
    def run_case(self, side="left", fault=None):
        clock = Clock()
        cfg = plan(side)
        controller = FakeController(clock, cfg, fault)
        records = []
        with patch.object(probe.time, "monotonic", lambda: clock.now), \
                patch.object(probe.time, "sleep", clock.advance):
            if fault:
                with self.assertRaises((probe.ProbeError, OSError)):
                    probe.run_probe(controller, cfg, LIMITS, records.append)
            else:
                probe.run_probe(controller, cfg, LIMITS, records.append)
        self.assertTrue(controller.closed)
        return controller, records

    def test_five_points_for_both_arms(self):
        for side in probe.SIDES:
            with self.subTest(side=side):
                ctrl, records = self.run_case(side)
                samples = [r for r in records if r["type"] == "sample"]
                self.assertEqual(tuple(row["point"] for row in samples), probe.POINTS)
                self.assertEqual([row["q17"][ctrl.plan.index] for row in samples],
                                 [0, 0.05, 0, -0.05, 0])
                self.assertEqual([samples[i]["return_error_rad"] for i in (2, 4)], [0, 0])
                self.assertEqual(samples[1]["tcp14"][0], 0.1)
                self.assertEqual(samples[1]["tcp14"][7], 0.2)
                self.assertAlmostEqual(samples[1]["tcp_rotation_from_baseline_rad"], 0.05)
                self.assertEqual(records[-1]["type"], "completed")
                self.assertEqual(ctrl.commands.count("movej"), 4)
                self.assertNotIn("stop_motion", ctrl.commands)

    def test_invalid_preflight_never_moves(self):
        for fault in ("fault", "limit", "bad_q", "bad_tcp"):
            with self.subTest(fault=fault):
                ctrl, records = self.run_case(fault=fault)
                self.assertNotIn("movej", ctrl.commands)
                self.assertFalse(records[-1]["stop"]["attempted"])

    def test_motion_failures_stop_without_dispatching_later_points(self):
        for fault, count in (("return", 2), ("frozen", 1), ("frozen_tcp", 1),
                             ("disconnect", 1), ("unselected", 1)):
            with self.subTest(fault=fault):
                ctrl, records = self.run_case(fault=fault)
                self.assertEqual(ctrl.commands.count("movej"), count)
                self.assertEqual(ctrl.commands[-1], "stop_motion")
                self.assertTrue(any(r.get("point") == "baseline" for r in records))
                self.assertEqual(records[-1]["type"], "aborted")
                self.assertEqual(records[-1]["stop"]["acknowledged"], fault != "disconnect")

    def test_limits_require_agreement_and_enough_margin(self):
        cfg = plan()
        with self.assertRaisesRegex(probe.ProbeError, "exceeds"):
            probe.check_limits(replace(cfg, delta_rad=1), LIMITS)
        with self.assertRaisesRegex(probe.ProbeError, "disagree"):
            probe.check_limits(cfg, LIMITS, probe.Reply({
                "lowerLimits": [-3] * 14, "upperLimits": [3] * 14}, 0, 0))

    def test_default_and_noninteractive_execute_cannot_connect(self):
        with patch.object(probe.socket, "create_connection") as connect, \
                redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            self.assertEqual(probe.main([str(URDF)]), 0)
            self.assertEqual(probe.main([str(URDF), "--execute"]), 1)
        connect.assert_not_called()

    def test_axis_scores_use_both_signs_and_reject_small_signal(self):
        self.assertEqual(probe.direction_score([1, 0, 0], [-2, 0, 0]), -1)
        self.assertIsNone(probe.direction_score([0.001, 0, 0], [1, 0, 0], 0.01))
        self.assertEqual(probe.classify([1, 1]), "ALIGNED")
        self.assertEqual(probe.classify([-1, -1]), "INVERTED")
        for scores in ([1, -1], [1, None], []):
            self.assertEqual(probe.classify(scores), "UNCERTAIN")


@unittest.skipUnless(importlib.util.find_spec("pinocchio"), "Pinocchio not installed")
class AnalysisTests(unittest.TestCase):
    def test_pinocchio_replay_uses_the_selected_arm_and_joint(self):
        import pinocchio as pin

        model = pin.buildModelFromUrdf(str(URDF))
        data = model.createData()
        for side in probe.SIDES:
            with self.subTest(side=side), TemporaryDirectory() as temp:
                config = plan(side)
                records = [{"type": "plan", "plan": probe.asdict(config),
                            "urdf_sha256": hashlib.sha256(URDF.read_bytes()).hexdigest()}]
                q0 = pin.neutral(model)
                name = probe.arm_joints(side)[config.joint - 1]
                index = model.joints[model.getJointId(name)].idx_q
                for point, target in zip(probe.POINTS, config.targets):
                    q = q0.copy()
                    q[index] = target
                    pin.forwardKinematics(model, data, q)
                    pin.updateFramePlacements(model, data)
                    q17 = list(config.baseline)
                    q17[config.index] = target
                    tcp = []
                    for arm in probe.SIDES:
                        frame = data.oMf[model.getFrameId(probe.SIDES[arm][2])]
                        x, y, z, w = pin.Quaternion(frame.rotation).coeffs()
                        tcp.extend([*frame.translation, w, x, y, z])
                    records.append({"type": "sample", "point": point, "q17": q17, "tcp14": tcp})
                records.append({"type": "completed"})
                path = Path(temp) / "synthetic.jsonl"
                path.write_text("".join(json.dumps(record) + "\n" for record in records))
                self.assertEqual(probe.analyze(URDF, path, 0.001)["verdict"], "ALIGNED")


class WireTests(unittest.TestCase):
    def test_fragmented_response_and_command_correlation(self):
        sock = Mock()
        sock.recv.side_effect = [
            b'{"command":"get_joint_',
            b'position","status":"program_run_finish"}#'
            b'{"command":"movej","status":"program_run_finish"}#',
        ]
        ctrl = probe.Controller(sock, 1)
        self.assertEqual(ctrl.request("get_joint_position").raw["command"], "get_joint_position")
        with self.assertRaisesRegex(probe.ProbeError, "unsolicited data"):
            ctrl.request("get_tcp_pose")
        self.assertEqual(sock.sendall.call_count, 1)

    def test_late_move_reply_is_not_stop_ack(self):
        sock = Mock()
        sock.recv.side_effect = [
            b'{"command":"movej","status":"program_run_finish"}#',
            b'{"command":"stop_motion","status":"program_run_failed"}#',
        ]
        with self.assertRaisesRegex(probe.ProbeError, "stop_motion failed"):
            probe.Controller(sock, 1).request("stop_motion")


if __name__ == "__main__":
    unittest.main()
