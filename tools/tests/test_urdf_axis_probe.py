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
            state_fault = self.fault == "fault" or (
                self.fault == "fault_after_move" and self.moves > 0
            )
            raw.update(state="idle", erroeCode=0, hardwareErrors=[],
                       softwareErrors=[123] if state_fault else [],
                       motorErrorDetails=None)
            if self.fault == "missing_state":
                raw.pop("state")
            if self.fault == "missing_hardware_errors":
                raw.pop("hardwareErrors")
            if self.fault == "missing_software_errors":
                raw.pop("softwareErrors")
            if self.fault == "missing_motor_details":
                raw.pop("motorErrorDetails")
            if self.fault == "missing_error_code":
                raw.pop("erroeCode")
            if self.fault == "motor_detail":
                raw["motorErrorDetails"] = [{"motor": 1, "error": "overcurrent"}]
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
            if self.fault == "move_rejected":
                raise probe.ControllerStateError("movej failed: 'program_run_failed'")
            if self.fault not in ("frozen", "stale_feedback") and not (
                self.fault == "return" and self.moves == 2
            ):
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
                left_quat = samples[1]["tcp14"][3:7]
                right_quat = samples[1]["tcp14"][10:14]
                moving_quat, stationary_quat = (
                    (left_quat, right_quat) if side == "left" else (right_quat, left_quat)
                )
                self.assertAlmostEqual(moving_quat[1], math.sin(0.025))
                self.assertAlmostEqual(stationary_quat[0], 1.0)
                self.assertEqual(records[-1]["type"], "completed")
                self.assertEqual(ctrl.commands.count("movej"), 4)
                self.assertNotIn("stop_motion", ctrl.commands)

    def test_invalid_preflight_never_moves(self):
        for fault in ("fault", "missing_state", "missing_hardware_errors",
                      "missing_software_errors", "missing_motor_details",
                      "missing_error_code", "motor_detail", "limit", "bad_q", "bad_tcp"):
            with self.subTest(fault=fault):
                ctrl, records = self.run_case(fault=fault)
                self.assertNotIn("movej", ctrl.commands)
                self.assertFalse(records[-1]["stop"]["attempted"])

    def test_fault_after_motion_does_not_send_stop_motion(self):
        ctrl, records = self.run_case(fault="fault_after_move")
        self.assertEqual(ctrl.commands.count("movej"), 1)
        self.assertNotIn("stop_motion", ctrl.commands)
        self.assertEqual(records[-1]["type"], "aborted")
        self.assertFalse(records[-1]["stop"]["attempted"])
        self.assertFalse(records[-1]["stop"]["acknowledged"])
        self.assertIn("skipped_reason", records[-1]["stop"])

    def test_explicit_move_rejection_does_not_send_stop_motion(self):
        ctrl, records = self.run_case(fault="move_rejected")
        self.assertEqual(ctrl.commands.count("movej"), 1)
        self.assertNotIn("stop_motion", ctrl.commands)
        self.assertEqual(records[-1]["type"], "aborted")
        self.assertFalse(records[-1]["stop"]["attempted"])
        self.assertIn("skipped_reason", records[-1]["stop"])

    def test_motion_failures_stop_without_dispatching_later_points(self):
        for fault, count in (("return", 2), ("frozen", 1), ("stale_feedback", 1),
                             ("frozen_tcp", 1), ("disconnect", 1), ("unselected", 1)):
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

    def test_plan_rejects_unverified_base_or_arm_tcp_frame(self):
        cfg = probe.asdict(plan())
        with self.assertRaisesRegex(probe.ProbeError, "base_link"):
            probe.Plan.from_json(json.dumps({**cfg, "base_frame": "world"}))
        with self.assertRaisesRegex(probe.ProbeError, "Link7"):
            probe.Plan.from_json(json.dumps({**cfg, "tcp_frame": "tool0"}))

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
    def test_controller_connection_conflict_aborts_before_motion(self):
        with TemporaryDirectory() as temp:
            cfg = plan()
            plan_file = Path(temp) / "plan.json"
            plan_file.write_text(json.dumps(probe.asdict(cfg)), encoding="utf-8")
            output_file = Path(temp) / "probe.jsonl"
            fingerprint = hashlib.sha256(
                plan_file.read_bytes() + bytes([0]) + URDF.read_bytes()
            ).hexdigest()
            fake_stdin = Mock()
            fake_stdin.isatty.return_value = True
            with patch.object(probe.sys, "stdin", fake_stdin):
                with patch("builtins.input", return_value=f"RUN {fingerprint}"):
                    with patch.object(
                        probe.socket, "create_connection",
                        side_effect=OSError("controller already has an owner"),
                    ):
                        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                            result = probe.main([
                                str(URDF), "left", "--plan", str(plan_file),
                                "--execute", "--output", str(output_file),
                            ])
            self.assertEqual(result, 1)
            records = [json.loads(line) for line in output_file.read_text().splitlines()]
            self.assertEqual(records[-1]["type"], "aborted")
            self.assertEqual(records[-1]["point"], "connect")
            self.assertFalse(records[-1]["stop"]["attempted"])

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

    def test_explicit_move_or_state_rejection_is_controller_state_error(self):
        for command in ("movej", "get_device_state"):
            with self.subTest(command=command):
                sock = Mock()
                sock.recv.return_value = (
                    json.dumps({"command": command, "status": "program_run_failed"}).encode() + b"#"
                )
                payload = {"joint": [0] * 17, "v": 1} if command == "movej" else {}
                with self.assertRaises(probe.ControllerStateError):
                    probe.Controller(sock, 1).request(command, **payload)

    def test_mismatched_state_reply_is_controller_state_error(self):
        sock = Mock()
        sock.recv.return_value = b'{"command":"get_joint_position","status":"program_run_finish"}#'
        with self.assertRaises(probe.ControllerStateError):
            probe.Controller(sock, 1).request("get_device_state")

    def test_read_only_inspection_uses_only_four_query_commands(self):
        class ReadOnlyFake:
            def __init__(self):
                self.commands = []

            def request(self, command):
                self.commands.append(command)
                return probe.Reply(
                    {"command": command, "status": probe.SUCCESS, "payload": command},
                    10.0, 10.01,
                )

        ctrl = ReadOnlyFake()
        report = probe.inspect_controller(ctrl, "127.0.0.1", 30485)
        self.assertEqual(
            ctrl.commands,
            ["get_device_state", "get_config", "get_joint_position", "get_tcp_pose"],
        )
        self.assertEqual(report["mode"], "read_only_controller_inspection")
        self.assertEqual(report["host"], "127.0.0.1")
        self.assertEqual(report["port"], 30485)
        self.assertEqual(set(report["responses"]), set(ctrl.commands))
        self.assertNotIn("movej", ctrl.commands)
        self.assertNotIn("stop_motion", ctrl.commands)

    def test_inspection_cli_requires_explicit_endpoint_without_connecting(self):
        with patch.object(probe.socket, "create_connection") as connect, \
                redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            result = probe.main([str(URDF), "--inspect-controller"])
        self.assertEqual(result, 1)
        connect.assert_not_called()

    def test_read_only_inspection_preserves_partial_raw_response_on_failure(self):
        class FailingReadOnlyFake:
            def __init__(self):
                self.commands = []

            def request(self, command):
                self.commands.append(command)
                if command == "get_config":
                    raise probe.ControllerStateError(
                        "get_config failed", raw={"command": command, "status": "fault"}
                    )
                return probe.Reply(
                    {"command": command, "status": probe.SUCCESS},
                    10.0, 10.01,
                )

        ctrl = FailingReadOnlyFake()
        report = probe.inspect_controller(ctrl, "127.0.0.1", 30485)
        self.assertFalse(report["complete"])
        self.assertEqual(report["failure"]["command"], "get_config")
        self.assertEqual(ctrl.commands, ["get_device_state", "get_config"])
        self.assertEqual(
            report["responses"]["get_config"]["raw"],
            {"command": "get_config", "status": "fault"},
        )
        self.assertNotIn("movej", ctrl.commands)
        self.assertNotIn("stop_motion", ctrl.commands)


if __name__ == "__main__":
    unittest.main()
