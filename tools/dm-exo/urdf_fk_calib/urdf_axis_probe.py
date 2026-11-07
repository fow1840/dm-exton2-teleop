#!/usr/bin/env python3
"""Collect a single-joint five-point probe. Default: offline validation only."""

import argparse
from dataclasses import asdict, dataclass
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import socket
import sys
import time
import uuid
import xml.etree.ElementTree as ET


SIDES = {"left": (1, "L", "Link7L"), "right": (8, "", "Link7")}
N_JOINTS = 17
HOLD = -100.0
SUCCESS = "program_run_finish"
POINTS = ("baseline", "positive", "return_positive", "negative", "return_negative")


class ProbeError(RuntimeError):
    pass


class ControllerStateError(ProbeError):
    """Controller state is faulted or incomplete; do not issue recovery motion commands."""

    def __init__(self, message, raw=None):
        super().__init__(message)
        self.raw = raw


def number(value, label):
    if type(value) in (int, float):
        try:
            value = float(value)
        except OverflowError:
            value = math.inf
        if math.isfinite(value):
            return value
    raise ProbeError(f"{label}: expected a finite number")


def vector(value, size, label):
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ProbeError(f"{label}: expected {size} values")
    return tuple(number(item, label) for item in value)


def arm_joints(side):
    suffix = SIDES[side][1]
    return tuple(f"Joint{i}{suffix}" for i in range(1, 8))


def load_urdf(urdf_bytes, sides):
    root = ET.fromstring(urdf_bytes)
    links = {element.get("name") for element in root.findall("link")}
    joints = {element.get("name"): element for element in root.findall("joint")}
    if root.tag != "robot" or "base_link" not in links:
        raise ProbeError("URDF: expected a robot with base_link")
    limits = {}
    for side in sides:
        if SIDES[side][2] not in links:
            raise ProbeError(f"URDF: missing {SIDES[side][2]}")
        for name in arm_joints(side):
            joint = joints.get(name)
            if joint is None or joint.get("type") != "revolute":
                raise ProbeError(f"URDF: missing revolute joint {name}")
            limit = joint.find("limit")
            if limit is None:
                raise ProbeError(f"URDF: missing limits for {name}")
            lo, hi = vector([float(limit.get("lower", "nan")),
                             float(limit.get("upper", "nan"))], 2, name)
            if lo >= hi:
                raise ProbeError(f"URDF: invalid limits for {name}")
            limits[name] = (lo, hi)
    return limits


@dataclass(frozen=True)
class Plan:
    host: str
    port: int
    side: str
    joint: int
    baseline: tuple
    delta_rad: float
    speed_pct: float
    limit_margin_rad: float
    urdf_limit_tolerance_rad: float
    target_tolerance_rad: float
    hold_tolerance_rad: float
    settle_tolerance_rad: float
    settle_s: float
    point_timeout_s: float
    rpc_timeout_s: float
    max_pair_s: float
    min_tcp_rotation_rad: float
    base_frame: str
    tcp_frame: str
    tcp_position_unit: str

    @classmethod
    def from_json(cls, raw):
        fields = json.loads(raw)
        if not isinstance(fields, dict):
            raise ProbeError("expected a JSON object for the plan")
        fields["baseline"] = vector(fields.get("baseline"), N_JOINTS, "baseline")
        plan = cls(**fields)
        if plan.side not in SIDES or type(plan.joint) is not int or not 1 <= plan.joint <= 7:
            raise ProbeError("select one arm and a joint number from 1 to 7")
        if plan.base_frame != "base_link" or plan.tcp_frame != SIDES[plan.side][2]:
            raise ProbeError("this probe requires base_link and the selected arm's Link7 frame")
        if type(plan.port) is not int or not 1 <= plan.port <= 65535:
            raise ProbeError("invalid controller port")
        for name in ("host", "base_frame", "tcp_frame"):
            if not isinstance(fields[name], str) or not fields[name].strip():
                raise ProbeError(f"{name} must be explicit")
        if plan.tcp_position_unit not in ("m", "mm"):
            raise ProbeError("tcp_position_unit must be m or mm")
        for name in ("delta_rad", "speed_pct", "limit_margin_rad",
                     "urdf_limit_tolerance_rad", "target_tolerance_rad",
                     "hold_tolerance_rad", "settle_tolerance_rad", "settle_s",
                     "point_timeout_s", "rpc_timeout_s", "max_pair_s", "min_tcp_rotation_rad"):
            if number(fields[name], name) <= 0:
                raise ProbeError(f"{name} must be positive")
        if (plan.speed_pct > 100 or plan.delta_rad <= 2 * plan.target_tolerance_rad
                or plan.settle_tolerance_rad > plan.target_tolerance_rad
                or plan.min_tcp_rotation_rad >= math.pi
                or max(plan.settle_s, plan.max_pair_s, plan.rpc_timeout_s) >= plan.point_timeout_s
                or HOLD in plan.baseline):
            raise ProbeError("inconsistent speed, feedback thresholds, timeouts or baseline")
        return plan

    @property
    def index(self):
        return SIDES[self.side][0] + self.joint - 1

    @property
    def targets(self):
        q = self.baseline[self.index]
        return (q, q + self.delta_rad, q, q - self.delta_rad, q)


@dataclass(frozen=True)
class Reply:
    raw: dict
    sent: float
    received: float


class Controller:
    """One outstanding V0.3 request at a time on the exclusive controller session."""

    def __init__(self, sock, timeout):
        self.sock = sock
        self.timeout = timeout
        self.buffer = bytearray()

    def close(self):
        self.sock.close()

    def request(self, command, deadline=None, **payload):
        if command != "stop_motion" and self.buffer.strip():
            raise ProbeError("unsolicited data remained from the previous request")
        sent = time.monotonic()
        if command != "movej" or deadline is None:
            deadline = min(deadline if deadline is not None else math.inf, sent + self.timeout)
        self._set_timeout(deadline)
        self.sock.sendall((json.dumps({"command": command, **payload}, allow_nan=False) + "#").encode())
        while True:
            self._set_timeout(deadline)
            end = self.buffer.find(b"#")
            if end < 0:
                chunk = self.sock.recv(4096)
                if not chunk:
                    raise ProbeError("controller connection closed")
                self.buffer.extend(chunk)
                if len(self.buffer) > 65536:
                    raise ProbeError("controller frame exceeds 64 KiB")
                continue
            raw = bytes(self.buffer[:end])
            del self.buffer[:end + 1]
            try:
                reply = json.loads(raw)
            except (ValueError, UnicodeError) as exc:
                raise ProbeError("invalid controller JSON") from exc
            if not isinstance(reply, dict) or reply.get("command") != command:
                # A mismatched state/motion reply leaves controller state ambiguous.
                # Do not issue another control command until an operator inspects it.
                if command == "stop_motion":
                    continue
                detail = f"unexpected reply while waiting for {command}: {reply!r}"
                if command in ("get_device_state", "movej"):
                    raise ControllerStateError(detail, raw=reply)
                raise ProbeError(detail)
            if reply.get("status") != SUCCESS:
                detail = f"{command} failed: {reply.get('status')!r}; response={reply!r}"
                # An explicit motion rejection or failed state query is not a generic
                # transport error. Avoid sending stop_motion into an unknown/faulted state.
                if command in ("get_device_state", "movej"):
                    raise ControllerStateError(detail, raw=reply)
                raise ProbeError(detail)
            return Reply(reply, sent, time.monotonic())

    def _set_timeout(self, deadline):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise ProbeError("controller response timed out")
        self.sock.settimeout(remaining)


def check_device(reply):
    raw = reply.raw
    required = ("state", "hardwareErrors", "softwareErrors", "motorErrorDetails")
    missing = [key for key in required if key not in raw]
    codes = [raw[key] for key in ("erroeCode", "errorCode") if key in raw]
    if (missing
            or not isinstance(raw.get("state"), str)
            or raw.get("state") not in ("idle", "ready")
            or not codes
            or any(type(code) is not int or code != 0 for code in codes)
            or type(raw.get("hardwareErrors")) is not list or raw["hardwareErrors"]
            or type(raw.get("softwareErrors")) is not list or raw["softwareErrors"]
            or (raw.get("motorErrorDetails") is not None
                and (type(raw["motorErrorDetails"]) is not list or raw["motorErrorDetails"]))):
        detail = f"missing required fields: {missing}" if missing else f"unusable controller state: {raw!r}"
        raise ControllerStateError(detail)


def inspect_controller(ctrl, host, port):
    """Read controller preflight responses. Preserve partial evidence; never moves."""
    responses = {}
    failure = None
    for command in ("get_device_state", "get_config", "get_joint_position", "get_tcp_pose"):
        try:
            reply = ctrl.request(command)
        except (Exception, KeyboardInterrupt) as exc:
            item = {"error": str(exc)}
            raw = getattr(exc, "raw", None)
            if raw is not None:
                item["raw"] = raw
            responses[command] = item
            failure = {"command": command, "error": str(exc)}
            break
        responses[command] = {
            "sent_monotonic_s": reply.sent,
            "received_monotonic_s": reply.received,
            "raw": reply.raw,
        }
    return {
        "mode": "read_only_controller_inspection",
        "host": host,
        "port": port,
        "timestamp_unix_s": time.time(),
        "complete": failure is None,
        "failure": failure,
        "responses": responses,
    }


def read_q(reply):
    q = vector(reply.raw.get("joint"), N_JOINTS, "actual joint feedback")
    if HOLD in q:
        raise ProbeError("hold sentinel in actual joint feedback")
    return q


def read_tcp(reply, unit):
    raw = vector(reply.raw.get("pose"), 14, "TCP feedback")
    scale = 0.001 if unit == "mm" else 1.0
    normalized = []
    for offset in (0, 7):
        quat = raw[offset + 3:offset + 7]
        norm = math.hypot(*quat)
        if not math.isclose(norm, 1.0, rel_tol=0.02):
            raise ProbeError("invalid TCP quaternion norm")
        normalized.extend(x * scale for x in raw[offset:offset + 3])
        normalized.extend(x / norm for x in quat)
    return tuple(normalized)


def check_limits(plan, limits, config=None):
    if config is not None:
        lower = vector(config.raw.get("lowerLimits"), 14, "controller lower limits")
        upper = vector(config.raw.get("upperLimits"), 14, "controller upper limits")
    for side, (start, _, _) in SIDES.items():
        for index, name in enumerate(arm_joints(side), start):
            model_lo, model_hi = limits[name]
            lo, hi = model_lo, model_hi
            if config is not None:
                live_lo, live_hi = lower[index - 1], upper[index - 1]
                if max(abs(live_lo - model_lo), abs(live_hi - model_hi)) > plan.urdf_limit_tolerance_rad:
                    raise ProbeError(f"{name}: live and URDF limits disagree")
                lo = max(live_lo + plan.limit_margin_rad, model_lo)
                hi = min(live_hi - plan.limit_margin_rad, model_hi)
            targets = plan.targets if index == plan.index else (plan.baseline[index],)
            tolerance = plan.target_tolerance_rad if index == plan.index else plan.hold_tolerance_rad
            if any(not lo <= angle - tolerance <= angle + tolerance <= hi for angle in targets):
                raise ProbeError(f"{name}: plan or its tolerance exceeds usable limits")


def check_joints(plan, q, start, target):
    if any(abs(value - plan.baseline[i]) > plan.hold_tolerance_rad
           for i, value in enumerate(q) if i != plan.index):
        raise ProbeError("an unselected joint moved")
    lo, hi = sorted((start, target))
    value = q[plan.index]
    if not lo - plan.target_tolerance_rad <= value <= hi + plan.target_tolerance_rad:
        raise ProbeError("selected joint left the approved interval")
    return abs(value - target) <= plan.target_tolerance_rad


def collect_point(ctrl, plan, start, target, deadline):
    anchor = None
    while time.monotonic() < deadline:
        device = ctrl.request("get_device_state", deadline)
        check_device(device)
        before = ctrl.request("get_joint_position", deadline)
        q_before = read_q(before)
        if not check_joints(plan, q_before, start, target):
            anchor = None
        elif anchor is None or any(abs(a - b) > plan.settle_tolerance_rad
                                   for a, b in zip(anchor[0], q_before)):
            anchor = (q_before, before.received)
        elif before.received - anchor[1] >= plan.settle_s:
            tcp_reply = ctrl.request("get_tcp_pose", deadline)
            tcp = read_tcp(tcp_reply, plan.tcp_position_unit)
            after = ctrl.request("get_joint_position", deadline)
            q_after = read_q(after)
            at_target = check_joints(plan, q_after, start, target)
            if after.received - before.sent > plan.max_pair_s:
                raise ProbeError("joint/TCP pair exceeded the approved time window")
            if at_target and all(abs(a - b) <= plan.settle_tolerance_rad
                                 for a, b in zip(q_before, q_after)):
                device = ctrl.request("get_device_state", deadline)
                check_device(device)
                return {
                    "timestamp_unix_s": time.time(),
                    "q17": q_before, "tcp14": tcp,
                    "raw": {"before": asdict(before), "tcp": asdict(tcp_reply),
                            "after": asdict(after), "device": asdict(device)},
                }
            anchor = None
        time.sleep(min(0.02, max(0, deadline - time.monotonic())))
    raise ProbeError("actual feedback did not reach and settle at the target")


def tcp_rotation(a, b, side):
    offset = 3 if side == "left" else 10
    dot = sum(x * y for x, y in zip(a[offset:offset + 4], b[offset:offset + 4]))
    return 2 * math.acos(min(1.0, abs(dot)))


def run_probe(ctrl, plan, limits, record):
    motion_attempted = False
    stage = "preflight"
    try:
        check_device(ctrl.request("get_device_state"))
        config = ctrl.request("get_config")
        check_limits(plan, limits, config)
        record({"type": "config", "reply": asdict(config)})
        previous_target = plan.targets[0]
        for stage, target in zip(POINTS, plan.targets):
            deadline = time.monotonic() + plan.point_timeout_s
            if stage != "baseline":
                check_device(ctrl.request("get_device_state", deadline))
                current = read_q(ctrl.request("get_joint_position", deadline))
                if not check_joints(plan, current, previous_target, previous_target):
                    raise ProbeError("actual feedback changed after the previous sample")
                frame = [HOLD] * N_JOINTS
                frame[plan.index] = target
                motion_attempted = True
                ack = ctrl.request("movej", deadline, joint=frame, v=plan.speed_pct)
                record({"type": "move_ack", "point": stage, "target_rad": target,
                        "reply": asdict(ack)})
            sample = collect_point(ctrl, plan, previous_target, target, deadline)
            actual = sample["q17"][plan.index]
            if stage == "baseline":
                measured_baseline = actual
                baseline_tcp = sample["tcp14"]
            delta = actual - measured_baseline
            rotation = tcp_rotation(baseline_tcp, sample["tcp14"], plan.side)
            is_return = stage.startswith("return_")
            record({"type": "sample", "point": stage, "target_rad": target,
                    "delta_from_baseline_rad": delta, "tcp_rotation_from_baseline_rad": rotation,
                    "return_error_rad": abs(delta) if is_return else None, **sample})
            if is_return and abs(delta) > plan.target_tolerance_rad:
                raise ProbeError("return error exceeds tolerance against measured baseline")
            if stage in ("positive", "negative"):
                signed_delta = delta if stage == "positive" else -delta
                if signed_delta <= plan.settle_tolerance_rad:
                    raise ProbeError("actual motion was not distinguishable from the baseline")
                if rotation < plan.min_tcp_rotation_rad:
                    raise ProbeError("TCP did not show a distinguishable rotation")
            previous_target = target
        record({"type": "completed", "points": len(POINTS)})
    except (Exception, KeyboardInterrupt) as exc:
        stop = {"attempted": False, "acknowledged": False}
        if isinstance(exc, ControllerStateError):
            stop["skipped_reason"] = (
                "controller state is faulted or incomplete; inspect and stop manually "
                "using the site's verified procedure"
            )
        elif motion_attempted:
            stop["attempted"] = True
            try:
                stop["reply"] = asdict(ctrl.request("stop_motion"))
                stop["acknowledged"] = True
            except (Exception, KeyboardInterrupt) as stop_exc:
                stop["error"] = str(stop_exc)
        try:
            record({"type": "aborted", "point": stage, "error": str(exc), "stop": stop})
        except OSError:
            print(f"Cannot write abort record. Stop result: {stop!r}", file=sys.stderr)
        raise
    finally:
        ctrl.close()


def direction_score(measured, predicted, minimum=0.0):
    m, p = math.hypot(*measured), math.hypot(*predicted)
    if m <= minimum or p <= minimum:
        return None
    return max(-1.0, min(1.0, sum((a / m) * (b / p) for a, b in zip(measured, predicted))))


def classify(rot_scores):
    if len(rot_scores) != 2:
        return "UNCERTAIN"
    if all(score is not None and score > 0.5 for score in rot_scores):
        return "ALIGNED"
    if all(score is not None and score < -0.5 for score in rot_scores):
        return "INVERTED"
    return "UNCERTAIN"


def analyze(urdf, sample_file, min_rotation):
    # Import only during offline analysis, so dry-run and collection use the stdlib.
    import numpy as np
    import pinocchio as pin

    with Path(sample_file).open(encoding="utf-8") as stream:
        records = [json.loads(line) for line in stream]
    if not records or records[0].get("type") != "plan" or records[-1].get("type") != "completed":
        raise ProbeError("expected a completed five-point recording")
    if records[0].get("urdf_sha256") != hashlib.sha256(Path(urdf).read_bytes()).hexdigest():
        raise ProbeError("URDF does not match the recorded checksum")
    plan = Plan.from_json(json.dumps(records[0]["plan"]))
    if plan.base_frame != "base_link" or plan.tcp_frame != SIDES[plan.side][2]:
        raise ProbeError("analysis requires the recorded base_link and matching Link7 frame")
    samples = [item for item in records if item["type"] == "sample"]
    if tuple(item["point"] for item in samples) != POINTS:
        raise ProbeError("recording does not contain the required five points")
    model = pin.buildModelFromUrdf(str(urdf))
    data = model.createData()
    baseline = samples[0]
    q0 = pin.neutral(model)
    for side, (start, _, _) in SIDES.items():
        for i, name in enumerate(arm_joints(side), start):
            joint = model.joints[model.getJointId(name)]
            q0[joint.idx_q] = baseline["q17"][i]
    frame_id = model.getFrameId(plan.tcp_frame)
    joint = model.joints[model.getJointId(arm_joints(plan.side)[plan.joint - 1])]
    jacobian = pin.computeFrameJacobian(model, data, q0, frame_id, pin.LOCAL_WORLD_ALIGNED)
    offset = 0 if plan.side == "left" else 7

    def pose(sample):
        x, y, z, w, qx, qy, qz = sample["tcp14"][offset:offset + 7]
        return np.array([x, y, z]), pin.Quaternion(w, qx, qy, qz).toRotationMatrix()

    p0, r0 = pose(baseline)
    scores = {}
    for index in (1, 3):
        sample = samples[index]
        position, rotation = pose(sample)
        predicted = jacobian[:, joint.idx_v] * (sample["q17"][plan.index] - baseline["q17"][plan.index])
        scores[sample["point"]] = {
            "rot_dot": direction_score(pin.log3(rotation @ r0.T), predicted[3:], min_rotation),
            "pos_dot": direction_score(position - p0, predicted[:3]),
        }
    return {"side": plan.side, "joint": plan.joint, "scores": scores,
            "verdict": classify([row["rot_dot"] for row in scores.values()])}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("urdf", type=Path)
    parser.add_argument("side", choices=(*SIDES, "both"), nargs="?", default="both")
    parser.add_argument("--plan", type=Path)
    parser.add_argument("--output", type=Path)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--execute", action="store_true")
    mode.add_argument("--analyze", type=Path)
    mode.add_argument("--inspect-controller", action="store_true")
    parser.add_argument("--host")
    parser.add_argument("--port", type=int)
    parser.add_argument("--rpc-timeout-s", type=float, default=1.0)
    parser.add_argument("--frames-verified", action="store_true")
    parser.add_argument("--min-rotation-rad", type=float)
    args = parser.parse_args(argv)
    try:
        sides = SIDES if args.side == "both" or args.plan or args.analyze else (args.side,)
        urdf_bytes = args.urdf.read_bytes()
        limits = load_urdf(urdf_bytes, sides)
        if args.analyze:
            if not args.frames_verified or number(args.min_rotation_rad, "rotation threshold") <= 0:
                raise ProbeError("analysis requires verified frames and a positive rotation threshold")
            print(json.dumps(analyze(args.urdf, args.analyze, args.min_rotation_rad)))
            return 0
        if args.inspect_controller:
            if args.plan or not args.host or args.port is None or not 1 <= args.port <= 65535:
                raise ProbeError("--inspect-controller requires --host and --port; do not pass --plan")
            if args.output and args.output.exists():
                raise ProbeError("inspection output must be a new file")
            timeout_s = number(args.rpc_timeout_s, "RPC timeout")
            if timeout_s <= 0:
                raise ProbeError("RPC timeout must be positive")
            sock = socket.create_connection((args.host, args.port), timeout=timeout_s)
            ctrl = Controller(sock, timeout_s)
            try:
                report = inspect_controller(ctrl, args.host, args.port)
            finally:
                ctrl.close()
            report["urdf_path"] = str(args.urdf)
            report["urdf_sha256"] = hashlib.sha256(urdf_bytes).hexdigest()
            rendered = json.dumps(report, indent=2, allow_nan=False)
            if args.output:
                with args.output.open("x", encoding="utf-8") as output:
                    output.write(rendered + "\n")
            else:
                print(rendered)
            if not report["complete"]:
                print("Read-only controller inspection incomplete; review the saved partial response.", file=sys.stderr)
            return 0 if report["complete"] else 1
        plan = None
        if args.plan:
            plan_bytes = args.plan.read_bytes()
            plan = Plan.from_json(plan_bytes)
            if args.side != "both" and plan.side != args.side:
                raise ProbeError("CLI arm does not match the plan")
            check_limits(plan, limits)
            fingerprint = hashlib.sha256(plan_bytes + b"\0" + urdf_bytes).hexdigest()
            model_sha = hashlib.sha256(urdf_bytes).hexdigest()
            preview = {"plan": asdict(plan), "targets_rad": plan.targets,
                       "approval_sha256": fingerprint, "urdf_sha256": model_sha}
            print(json.dumps(preview, indent=2))
        if not args.execute:
            print("Dry-run: URDF checked; no controller connection.")
            return 0
        if plan is None or args.output is None or not sys.stdin.isatty():
            raise ProbeError("--execute requires --plan, a new --output and a local operator terminal")
        prompt = f"Confirm exclusive control and the on-site approved plan: RUN {fingerprint}: "
        if input(prompt).strip() != f"RUN {fingerprint}":
            raise ProbeError("operator approval was not entered")
        lock_path = Path("/tmp") / f"exton-axis-probe-{os.getuid()}.lock"
        lock_fd = os.open(lock_path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
        with os.fdopen(lock_fd, "w") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with args.output.open("x", encoding="utf-8") as output:
                run_id = uuid.uuid4().hex

                def record(value):
                    output.write(json.dumps({"run_id": run_id, **value}, allow_nan=False) + "\n")
                    output.flush()

                record({"type": "plan", **preview})
                try:
                    sock = socket.create_connection((plan.host, plan.port), timeout=plan.rpc_timeout_s)
                except OSError as exc:
                    record({"type": "aborted", "point": "connect", "error": str(exc),
                            "stop": {"attempted": False, "acknowledged": False}})
                    raise
                run_probe(Controller(sock, plan.rpc_timeout_s), plan, limits, record)
        return 0
    except (ProbeError, OSError, ValueError, TypeError, EOFError, KeyError,
            ET.ParseError, ImportError, KeyboardInterrupt) as exc:
        print(f"Probe ended: {exc or 'interrupted'}", file=sys.stderr)
        return 1


def interrupt(_signum, _frame):
    raise KeyboardInterrupt


if __name__ == "__main__":
    signal.signal(signal.SIGTERM, interrupt)
    sys.exit(main())
